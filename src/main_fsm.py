"""
任务有限状态机引擎 —— 栈式抢占架构。

管理状态栈，循环执行栈顶状态，
每次迭代执行全局健康检查。
支持抢占（suspend/resume）和不可恢复中断（清空栈）。
"""

import asyncio
import time

from mavsdk.offboard import PositionNedYaw
from interface import PX4Interface
from config import CRUISE_ALTITUDE_M, FSM_LOOP_HZ, MAX_STACK_DEPTH
from logger_manager import get_logger
from states.base_state import BaseState
from states.hover import HoverState
from states.transit import TransitState
from states.land_in_place import LandInPlaceState
from states.land import PrecisionLandState
from states.search import SearchState


class MissionFSM:
    """栈式抢占任务状态机引擎。"""

    def __init__(self, interface: PX4Interface):
        self.interface = interface
        self._stack: list[BaseState] = []

        # 注册不健康回调
        interface._on_unhealthy = self._handle_unhealthy

    def build_mission(self):
        """
        构建任务栈。

        起飞已由 PX4 内建 takeoff 在 FSM 启动前完成，
        因此任务栈从 HoverState 开始（稳定悬停后执行后续任务）。

        栈顶（list[-1]）先执行，完成弹出后下一层接管。
        所以构建顺序与执行顺序相反
        """
        self._stack = [
            PrecisionLandState(timeout_s=30),
            SearchState(timeout_s=60),                             
            TransitState(
                target=PositionNedYaw(
                    31.0, 0.0, -CRUISE_ALTITUDE_M, 0.0   # 场地坐标 → field_to_ned 旋转后发送
                ), speed=5.0, timeout_s=30),

            ]

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    async def run(self):
        """
        主状态机循环。

        启动流程（方案B）：
            连接 → Arm（锁定HOME点）→ PX4内建起飞至巡航高度
            → 切换到Offboard模式 → 启动任务状态机
        """
        self.build_mission()
        await self.interface.connect_and_setup()

        # 阶段1：上锁（PX4 在此刻记录 HOME 点）
        await self.interface.arm()

        # 阶段2：PX4 内建起飞至巡航高度
        await self.interface.takeoff(CRUISE_ALTITUDE_M)

        # 阶段3：切换到 offboard 模式
        await self.interface.switch_to_offboard()

        while self._stack:
            state = self._stack[-1]  # 栈顶 = 当前执行

            # ---- 首次进入 ----
            if not state._entered:
                prev_name = self._stack[-2].name \
                    if len(self._stack) >= 2 and self._stack[-2]._entered \
                    else "start"
                get_logger().log_state_transition(prev_name, state.name)

                # 进入前全局健康检查
                if not await self.interface.global_guard_check(
                        allow_disarmed=state.allow_disarmed):
                    await self._handle_unhealthy(
                        f"进入 {state.name} 前全局守卫失败")
                    return

                await state.enter(self.interface)

            # ---- 每周期全局健康检查 ----
            if not await self.interface.global_guard_check(
                    allow_disarmed=state.allow_disarmed):
                await self._handle_unhealthy(
                    f"{state.name} 执行中健康检查失败")
                return

            # ---- 执行 ----
            try:
                result = await state.execute(self.interface)
            except Exception as e:
                state.error = str(e)
                await self._handle_state_error(state, e)
                return

            # ---- 错误 ----
            if result.error:
                await self._handle_state_error(
                    state, RuntimeError(result.error))
                return

            # ---- 抢占：挂起当前，压入新状态 ----
            if result.interrupt is not None:
                if len(self._stack) >= MAX_STACK_DEPTH:
                    print(f"[警告] 状态栈已达最大深度 {MAX_STACK_DEPTH}，"
                          f"拒绝抢占 {state.name} → {result.interrupt.name}")
                    get_logger().log_message(
                        "warning",
                        f"状态栈已达最大深度 {MAX_STACK_DEPTH}，"
                        f"拒绝抢占 {state.name} → {result.interrupt.name}")
                    continue

                print(f"[状态机] {state.name} 被 {result.interrupt.name} 抢占")
                get_logger().log_message(
                    "state_machine",
                    f"{state.name} 被 {result.interrupt.name} 抢占")
                await state.suspend(self.interface)
                self._stack.append(result.interrupt)
                continue

            # ---- 完成：弹出，恢复下层 ----
            if result.done:
                # 超时处理
                if state.is_timed_out() and not state.is_completed:
                    print(f"[警告] {state.name} 超时 ({state.timeout_s}秒)，"
                          f"跳过")
                    get_logger().log_message(
                        "warning",
                        f"{state.name} 超时 ({state.timeout_s}秒)，跳过",
                        "timeout")

                await state.exit(self.interface)
                self._stack.pop()
                if self._stack:
                    await self._stack[-1].resume(self.interface)
                continue

            await asyncio.sleep(1.0 / FSM_LOOP_HZ)

        # 任务结束 —— 以飞控实际状态为准，确保安全断开上锁
        await self._ensure_disarmed()

    # ------------------------------------------------------------------
    # 安全收尾
    # ------------------------------------------------------------------

    async def _ensure_disarmed(self):
        """
        确保飞控已安全断开上锁。

        正常流程中，PX4 Land 模式着陆后会自动 disarm；
        此方法检测实际状态，必要时发送 disarm 并等待确认。
        若 disarm 失败，尝试 RTL 作为最后手段。

        关停顺序：停止心跳 → 退出 offboard → 断开上锁。
        """
        if not await self._is_armed():
            # 已由 PX4 自动 disarm（如 auto-land 完成后）
            # 仍需停止心跳和 offboard 以清理状态
            self.interface.stop_heartbeat()
            print("[信息] 任务完成，飞控已断开上锁")
            get_logger().log_message("info", "任务完成，飞控已断开上锁")
            return

        # 仍在上锁状态 —— 发送 disarm 并等待确认
        print("[信息] 发送 disarm 指令...")
        await self.interface.disarm()

        if await self._wait_for_disarm(timeout=5.0):
            print("[信息] 任务完成，飞控已断开上锁")
            get_logger().log_message("info", "任务完成，飞控已断开上锁")
            return

        # disarm 失败 —— 降级为 RTL
        # 注意：RTL 前心跳已由 disarm() 停止
        print("[错误] disarm 失败，飞控未响应 —— 尝试 RTL 作为最后手段")
        get_logger().log_message(
            "error", "disarm 失败，飞控未响应，尝试 RTL", "fail")
        try:
            await self.interface.drone.action.return_to_launch()
        except Exception as e:
            print(f"[致命错误] RTL 也失败了: {e}")
            get_logger().log_message(
                "fatal", f"RTL 失败: {e}", "fail")

    async def _is_armed(self) -> bool:
        """查询飞控当前是否处于上锁状态（单次快照）。"""
        async for armed in self.interface.drone.telemetry.armed():
            return armed

    async def _wait_for_disarm(self, timeout: float = 5.0) -> bool:
        """等待飞控断开上锁，返回 True 表示成功断开。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not await self._is_armed():
                return True
            await asyncio.sleep(0.1)
        return False

    async def _handle_unhealthy(self, reason: str):
        """
        紧急情况：健康检查失败时清空栈并触发 RTL。

        不可恢复抢占 —— 丢弃所有挂起状态，让 PX4 自主返航。
        """
        print(f"[紧急] 全局健康检查失败: {reason}")
        print("[紧急] 触发 RTL 返航 —— PX4 将自主爬升、返航、降落")
        get_logger().log_message(
            "emergency", f"全局健康检查失败: {reason}", "fail")
        get_logger().log_message(
            "emergency", "触发 RTL 返航 —— PX4 将自主爬升、返航、降落")
        self._stack.clear()  # 丢弃所有挂起状态
        await self.interface.drone.action.return_to_launch()

    async def _handle_state_error(self, state: BaseState, error: Exception):
        """
        状态执行错误的统一处理。

        不可恢复抢占 —— 丢弃所有挂起状态，触发 RTL。
        """
        print(f"[错误] {state.name} 抛出异常: {error}")
        print("[紧急] 触发 RTL 返航")
        get_logger().log_message(
            "error", f"{state.name} 抛出异常: {error}", "fail")
        get_logger().log_message(
            "emergency", "触发 RTL 返航")
        self._stack.clear()  # 丢弃所有挂起状态
        await self.interface.drone.action.return_to_launch()

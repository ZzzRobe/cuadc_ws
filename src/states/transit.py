"""巡航状态 —— 设置 PX4 内部限速参数，发送目标航点，由 PX4 自主飞行。

控制架构（PX4 原生位置控制）：

  不做的事：每周期手动推进 setpoint（增量斜坡）
  → 斜坡方案要求 Python 侧精确同步 setpoint 流，与心跳、gRPC 延迟耦合。

  做的事：
  1. 设置 MPC_XY_VEL_MAX 为目标速度，由 PX4 Position Controller 内部限速
  2. 发送目标位置 setpoint（一次），心跳维持
  3. 每周期检查与航点距离，小于阈值即到达

  PX4 的 Position Controller + Velocity Controller 在飞控内部以 200Hz+
  运行，自动处理加速、巡航、减速全过程。Python 侧只负责"告诉 PX4 去哪里"。

参见：
  - https://docs.px4.io/main/en/flight_modes/offboard
  - https://docs.px4.io/main/en/advanced_config/parameter_reference
"""

import math
import time

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from config import FSM_LOOP_HZ


class TransitState(BaseState):
    """
    以受控速度飞往目标场地坐标。

    参数 target 是 PositionNedYaw（场地坐标系）：
      - north_m = 沿场地前方方向（米）
      - east_m  = 沿场地右方方向（米）
      - down_m  = 飞行高度（米，向下为正），-5.0 = 5米高
      - yaw_deg  = 场地坐标系中的目标航向（度，0°=场地前方）

    enter() 时通过 interface.field_to_ned() 将场地坐标旋转为
    真北 NED 发送给 PX4。PX4 内部 Position Controller 自动控制
    全程飞行。每周期检查距离判据。
    """

    # 航点到达距离阈值（米）
    ARRIVE_DISTANCE_M = 1.0

    def __init__(self, target: PositionNedYaw,
                 speed: float = 5.0, timeout_s: float = 60):
        super().__init__("Transit", timeout_s)
        self.target = target   # PositionNedYaw（场地坐标系）
        self.speed = speed

        # 状态（在 enter() 中初始化）
        self._target_ned = None   # 真北 NED（经 field_to_ned 转换后）
        self._total_dist = None
        self._original_vel_max = None  # 原始 MPC_XY_VEL_MAX，退出时恢复
        self._debug_counter = 0
        self._debug_t0 = 0.0

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface):
        await super().enter(interface)

        self._target_ned = interface.field_to_ned(self.target)

        pos = await interface.get_position_ned()
        self._total_dist = math.hypot(
            self._target_ned.north_m - pos.north_m,
            self._target_ned.east_m - pos.east_m,
        )

        # ---- 启动 PX4 原生位置飞行（设置限速 + 发送航点 setpoint） ----
        self._original_vel_max = await interface.start_position_flight(
            self._target_ned, self.speed)

        print(f"[巡航] 航点 setpoint (真北): "
              f"N({self._target_ned.north_m:.1f}) "
              f"E({self._target_ned.east_m:.1f}) "
              f"D({self._target_ned.down_m:.1f}) "
              f"Yaw({self._target_ned.yaw_deg:.0f}°)", flush=True)
        print(f"[巡航] -> 场地坐标 N({self.target.north_m:.1f}, "
              f"{self.target.east_m:.1f}), 高度 {-self.target.down_m:.1f}m, "
              f"航向 {self.target.yaw_deg:.0f}° "
              f"@ {self.speed:.1f} m/s, 总距 {self._total_dist:.1f} m",
              flush=True)

        self._debug_t0 = time.monotonic()

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "巡航超时"
            return ExecutionResult(done=True)

        # ---- 获取当前位置，计算到目标的距离 ----
        pos = await interface.get_position_ned()
        drone_dist = math.hypot(
            self._target_ned.north_m - pos.north_m,
            self._target_ned.east_m - pos.east_m,
        )
        alt = await interface._read_altitude_direct()

        # ---- 到达判据 ----
        if drone_dist < self.ARRIVE_DISTANCE_M and abs(alt + self.target.down_m) < 0.5:
            print(f"[巡航] 到达航点 (距离 {drone_dist:.1f}m, 高度 {alt:.1f}m)",
                  flush=True)
            self.is_completed = True
            return ExecutionResult(done=True)

        # ---- 调试输出（约 1 Hz） ----
        self._debug_counter += 1
        if self._debug_counter % FSM_LOOP_HZ == 0:
            elapsed = time.monotonic() - self._debug_t0
            avg_hz = self._debug_counter / elapsed if elapsed > 0 else 0
            pct = (1 - drone_dist / self._total_dist) * 100 \
                if self._total_dist > 0 else 0
            print(f"[巡航] t+{elapsed:.0f}s 剩余 {drone_dist:.1f}m "
                  f"({pct:.0f}%)  alt={alt:.1f}m  "
                  f"FSM≈{avg_hz:.1f}Hz", flush=True)

        return ExecutionResult()

    async def exit(self, interface):
        """退出时恢复原始 MPC_XY_VEL_MAX。"""
        await interface.restore_cruise_speed(self._original_vel_max)
        await super().exit(interface)

"""
PX4 通信层 —— 封装所有 MAVSDK 交互。

提供：
- 健康看门狗（全局守卫）用于故障安全
- Offboard 心跳循环（独立 asyncio 任务，20 Hz）
- 遥测监控（健康状态、电量、连接状态）
- 场地 NED 坐标到真北 NED 坐标的转换（旋转矩阵）
- 高级指令（降落、舵机控制）

坐标系约定：
  - 代码中所有任务坐标均在"场地 NED"坐标系中定义：
    * 原点 = HOME 点（PX4 上电/上锁时自动记录）
    * N（场地北）= 场地前方方向
    * E（场地东）= 场地右方方向
    * D（下）= 垂直向下，高度用 -D（向上为正）表示
  - field_to_ned() 通过旋转矩阵将场地 NED 转换为真北 NED，
    旋转角 = config.FIELD_YAW_DEG（场地前方方向的真北方位角）。
"""

import asyncio
import math
from dataclasses import dataclass
from typing import Optional, Callable

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw
from logger_manager import get_logger
import config


@dataclass
class HealthStatus:
    """飞控健康状态快照。"""

    is_connected: bool = False
    is_armed: bool = False
    is_offboard: bool = False
    is_global_position_ok: bool = False
    is_home_position_ok: bool = False
    battery_pct: float = 100.0
    estimator_flags_ok: bool = True
    gps_fix_type: int = 3  # SITL 始终有 3D 定位；由 _gps_watcher 更新
    altitude_m: float = 0.0

    @property
    def is_healthy(self) -> bool:
        from config import BATTERY_LOW_THRESHOLD_PCT, GPS_FIX_MIN

        return (
            self.is_connected
            and self.is_armed
            and self.is_global_position_ok
            and self.is_home_position_ok
            and self.battery_pct > BATTERY_LOW_THRESHOLD_PCT
            and self.estimator_flags_ok
            and self.gps_fix_type >= GPS_FIX_MIN
        )


class PX4Interface:
    """封装所有 MAVSDK 交互，内置安全机制。"""

    def __init__(self, system_address: str = "udp://0.0.0.0:14540",
                 on_unhealthy: Optional[Callable] = None):
        self.drone = System()
        self.system_address = system_address
        self.health = HealthStatus()
        self._on_unhealthy = on_unhealthy

        # 心跳状态
        self._last_setpoint = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        self._last_velocity = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        self._setpoint_type = "position"  # "position" | "velocity" | "position_velocity"
        self._heartbeat_running = False

        # 场地航向 —— 从 config.py 读取，赛前手动测量并配置
        self.FIELD_YAW_DEG: float = config.FIELD_YAW_DEG

        # 跨状态共享数据存储（例如搜索结果）
        self.shared: dict = {}

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def connect_and_setup(self):
        """连接到 PX4，等待 GPS 锁定。"""
        await self.drone.connect(system_address=self.system_address)

        # 等待连接
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        self.health.is_connected = True
        print("[信息] 已连接到飞控")
        get_logger().log_message("info", "已连接到飞控")

        # 等待全局位置和家点位置
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                break
        self.health.is_global_position_ok = True
        self.health.is_home_position_ok = True
        print("[信息] GPS 已锁定，家点位置已记录")
        get_logger().log_message("info", "GPS 已锁定，家点位置已记录")

        # 场地朝向角使用 config.py 中手动配置的值
        print(f"[信息] 场地朝向角: "
              f"FIELD_YAW_DEG = {self.FIELD_YAW_DEG:.1f} 度（来自 config.py）")
        get_logger().log_message(
            "info",
            f"场地朝向角: FIELD_YAW_DEG = {self.FIELD_YAW_DEG:.1f} 度（来自 config.py）")

        # 启动后台任务
        asyncio.create_task(self._heartbeat_loop())

    async def arm(self):
        """上锁（PX4 在此刻记录 HOME 点作为 NED 原点）。"""
        await self.drone.action.arm()
        self.health.is_armed = True
        print("[信息] 已上锁，HOME 点已记录")
        get_logger().log_message("info", "已上锁，HOME 点已记录")

    async def takeoff(self, altitude_m: float, timeout_s: float = 60):
        """
        使用 PX4 内建起飞逻辑爬升至目标高度。

        PX4 的 action.takeoff() 有专门的地面检测、爬升率 ramp-up
        和地面效应补偿，比 offboard setpoint 爬升更安全可靠。

        参数:
            altitude_m: 目标巡航高度（米）
            timeout_s:  起飞超时时间（秒）
        """
        await self.drone.action.set_takeoff_altitude(altitude_m)
        await self.drone.action.takeoff()
        print(f"[起飞] PX4 内建起飞，目标高度 {altitude_m:.1f} 米")
        get_logger().log_message(
            "takeoff", f"PX4 内建起飞，目标高度 {altitude_m:.1f} 米")

        # 等待到达目标高度（直接读取遥测，不依赖缓存）
        from config import TAKEOFF_COMPLETE_THRESHOLD
        import time as _time
        deadline = _time.monotonic() + timeout_s
        while _time.monotonic() < deadline:
            alt = await self._read_altitude_direct()
            if alt >= altitude_m * (1 - TAKEOFF_COMPLETE_THRESHOLD):
                print(f"[起飞] 已到达巡航高度 {alt:.1f} 米")
                get_logger().log_message(
                    "takeoff", f"已到达巡航高度 {alt:.1f} 米")
                return
            await asyncio.sleep(0.5)

        # 超时
        raise TimeoutError(
            f"起飞超时（{timeout_s}秒），"
            f"当前高度 {await self._read_altitude_direct():.1f} 米，"
            f"目标 {altitude_m:.1f} 米")

    async def switch_to_offboard(self):
        """
        切换到 offboard 模式。

        必须在起飞完成、飞机已在巡航高度悬停后调用。
        采用与旧 arm_and_offboard() 相同的已验证模式：
        显式 set_position_ned() → 立即 offboard.start()，
        中间不插入长延时（避免 MAVSDK 内部 setpoint 状态过期）。

        PX4 端的 setpoint 连续性由心跳循环保证（自
        connect_and_setup 起已在后台以 20Hz 持续发送）。

        调用时机：arm() → takeoff() → switch_to_offboard()
        """
        # 读取当前 NED 位置作为初始 offboard setpoint
        current_pos = await self.get_position_ned()
        self.update_setpoint(current_pos)
        print(f"[信息] 初始 offboard setpoint: "
              f"N({current_pos.north_m:.1f}) E({current_pos.east_m:.1f}) "
              f"D({current_pos.down_m:.1f})")
        get_logger().log_message(
            "info",
            f"初始 offboard setpoint: "
            f"N({current_pos.north_m:.1f}) E({current_pos.east_m:.1f}) "
            f"D({current_pos.down_m:.1f})")

        # 显式调用一次（MAVSDK 要求在 start() 前至少调用一次）
        await self.drone.offboard.set_position_ned(self._last_setpoint)

        # 立即 start —— 与旧 arm_and_offboard 相同的已验证时序
        # 心跳循环已在后台持续发送，满足 PX4 的 setpoint 流要求
        await self.drone.offboard.start()
        self.health.is_offboard = True
        print("[信息] offboard 模式已启用")
        get_logger().log_message("info", "offboard 模式已启用")

    async def disarm(self):
        """
        规范关停流程：停止心跳 → 退出 offboard → 断开上锁。

        心跳必须在 offboard.stop() 之前停止，否则持续发送的
        set_position_ned 会干扰 PX4 的模式切换和 disarm 过程。
        """
        # 步骤1：停止心跳（阻止新的 offboard setpoint 发送）
        self.stop_heartbeat()
        await asyncio.sleep(0.1)  # 等待最后一个心跳周期完成

        # 步骤2：退出 offboard 模式
        try:
            await self.drone.offboard.stop()
            print("[信息] 已退出 offboard 模式")
            get_logger().log_message("info", "已退出 offboard 模式")
        except Exception as e:
            print(f"[警告] 退出 offboard 失败: {e}")
            get_logger().log_message(
                "warning", f"退出 offboard 失败: {e}", "fail")

        # 步骤3：断开上锁
        try:
            await self.drone.action.disarm()
            self.health.is_armed = False
            self.health.is_offboard = False
            print("[信息] 已断开上锁")
            get_logger().log_message("info", "已断开上锁")
        except Exception as e:
            print(f"[警告] 断开上锁失败: {e}")
            get_logger().log_message(
                "warning", f"断开上锁失败: {e}", "fail")

    # ------------------------------------------------------------------
    # 健康看门狗
    # ------------------------------------------------------------------

    async def global_guard_check(self) -> bool:
        """
        每个循环周期调用。返回 True 表示健康。

        所有遥测数据通过即时查询读取，节流至约 2 Hz
        以避免压垮 MAVSDK 的 gRPC 回调队列。
        两次读取之间返回上次缓存的结果。
        """
        import time as _time

        now = _time.monotonic()
        if not hasattr(self, "_last_guard_read"):
            self._last_guard_read = 0.0
        if not hasattr(self, "_cached_healthy"):
            self._cached_healthy = True

        # 节流：实际 MAVSDK 读取仅约 2 Hz
        if now - self._last_guard_read > 0.5:
            self._last_guard_read = now
            try:
                async for state in self.drone.core.connection_state():
                    self.health.is_connected = state.is_connected
                    break
                async for armed in self.drone.telemetry.armed():
                    self.health.is_armed = armed
                    break
                async for health in self.drone.telemetry.health():
                    self.health.is_global_position_ok = health.is_global_position_ok
                    self.health.is_home_position_ok = health.is_home_position_ok
                    break
                async for gps in self.drone.telemetry.gps_info():
                    self.health.gps_fix_type = gps.fix_type.value
                    break
                async for battery in self.drone.telemetry.battery():
                    self.health.battery_pct = battery.remaining_percent
                    break
                async for pos in self.drone.telemetry.position():
                    self.health.altitude_m = pos.relative_altitude_m
                    break
            except Exception as e:
                print(f"[调试] global_guard_check 读取错误: {e}")
                get_logger().log_message(
                    "debug", f"global_guard_check 读取错误: {e}", "fail")
            self._cached_healthy = self.health.is_healthy

        return self._cached_healthy

    # ------------------------------------------------------------------
    # Offboard 心跳（独立任务 —— 防止 PX4 超时）
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self):
        """
        以固定频率发送设定值。根据 _setpoint_type 自动选择
        对应的 MAVSDK offboard 方法。

        支持三种类型：
          - "position":          纯位置控制 (set_position_ned)
          - "velocity":          纯速度控制 (set_velocity_ned)
          - "position_velocity": 位置+速度前馈 (set_position_velocity_ned)

        PX4 要求 >= 2 Hz；我们以 OFFBOARD_HEARTBEAT_HZ（约 20 Hz）
        发送以留出余量。

        在 offboard 模式激活前调用 set_position_ned() 会成功发送
        MAVLink 消息，PX4 会缓存这些 setpoint 用于 offboard 进入判定。
        """
        from config import OFFBOARD_HEARTBEAT_HZ

        self._heartbeat_running = True
        interval = 1.0 / OFFBOARD_HEARTBEAT_HZ
        while self._heartbeat_running:
            try:
                if self._setpoint_type == "position_velocity":
                    await self.drone.offboard.set_position_velocity_ned(
                        self._last_setpoint, self._last_velocity)
                elif self._setpoint_type == "velocity":
                    await self.drone.offboard.set_velocity_ned(
                        self._last_velocity)
                else:
                    await self.drone.offboard.set_position_ned(
                        self._last_setpoint)
            except Exception as e:
                print(f"[调试] 心跳 setpoint 发送失败: {e}")
                get_logger().log_message(
                    "debug", f"心跳 setpoint 发送失败: {e}", "fail")
            await asyncio.sleep(interval)
        print("[信息] 心跳循环已停止")
        get_logger().log_message("info", "心跳循环已停止")

    def stop_heartbeat(self):
        """停止心跳循环。在 disarm 或紧急关停前调用。"""
        self._heartbeat_running = False

    # ------------------------------------------------------------------
    # Setpoint 更新 API —— 三种类型
    # ------------------------------------------------------------------

    def update_setpoint(self, setpoint: PositionNedYaw):
        """纯位置控制 —— 所有现有状态使用，保持向后兼容。"""
        self._last_setpoint = setpoint
        self._setpoint_type = "position"

    def update_velocity_setpoint(self, vel: VelocityNedYaw):
        """纯速度控制 —— 视觉伺服、避障等场景。"""
        self._last_velocity = vel
        self._setpoint_type = "velocity"

    def update_position_velocity_setpoint(self, pos: PositionNedYaw,
                                           vel: VelocityNedYaw):
        """位置+速度前馈 —— 搜索航线限速巡航。"""
        self._last_setpoint = pos
        self._last_velocity = vel
        self._setpoint_type = "position_velocity"

    def clear_velocity(self):
        """
        紧急清除速度前馈 —— suspend 时调用。

        将 velocity 归零并切回纯位置模式，确保心跳退化为位置保持，
        避免 suspend 窗口期内飞机因残留速度指令漂移。
        """
        self._last_velocity = VelocityNedYaw(0.0, 0.0, 0.0, self.FIELD_YAW_DEG)
        self._setpoint_type = "position"

    # ------------------------------------------------------------------
    # PX4 原生位置飞行（Transit / Search 共用）
    # ------------------------------------------------------------------

    async def start_position_flight(self, target: PositionNedYaw,
                                     speed_mps: float) -> float | None:
        """
        以指定速度飞向目标位置（PX4 原生位置控制）。

        做的事：
        1. 保存并设置 MPC_XY_VEL_MAX 为 speed_mps
        2. 更新心跳 setpoint 缓存为 target

        PX4 内部 Position Controller（200Hz+）自主处理加速、巡航、
        减速全过程。心跳以 20Hz 维持 setpoint 流。

        返回原始 MPC_XY_VEL_MAX 值（None 表示读取失败），
        调用者应在飞行结束后传给 restore_cruise_speed() 恢复。

        后续只需周期性地检查与目标的距离判断到达即可，
        不需要再手动调用 set_position_ned。
        """
        # 保存原始限速
        original = None
        try:
            original = await self.drone.param.get_param_float("MPC_XY_VEL_MAX")
            print(f"[飞行] 原始 MPC_XY_VEL_MAX = {original:.1f} m/s", flush=True)
        except Exception as e:
            print(f"[飞行] 读取 MPC_XY_VEL_MAX 失败: {e}，将不恢复原值", flush=True)

        # 设置巡航速度
        try:
            await self.drone.param.set_param_float("MPC_XY_VEL_MAX", float(speed_mps))
            confirmed = await self.drone.param.get_param_float("MPC_XY_VEL_MAX")
            print(f"[飞行] MPC_XY_VEL_MAX => {confirmed:.1f} m/s", flush=True)
        except Exception as e:
            print(f"[飞行] 设置 MPC_XY_VEL_MAX 失败: {e}", flush=True)

        # 发送目标 setpoint（心跳维持）
        self.update_setpoint(target)

        return original

    async def restore_cruise_speed(self, original: float | None):
        """
        恢复 MPC_XY_VEL_MAX 到飞行前的值。

        参数 original 应为 start_position_flight 的返回值。
        传入 None 时静默跳过。
        """
        if original is None:
            return
        try:
            await self.drone.param.set_param_float("MPC_XY_VEL_MAX", original)
            print(f"[飞行] 已恢复 MPC_XY_VEL_MAX = {original:.1f} m/s", flush=True)
        except Exception as e:
            print(f"[飞行] 恢复 MPC_XY_VEL_MAX 失败: {e}", flush=True)

    # ------------------------------------------------------------------
    # 高级指令
    # ------------------------------------------------------------------

    async def set_actuator(self, index: int, value: float):
        """通过 AUX 输出控制舵机（例如投放舵机）。"""
        await self.drone.action.set_actuator(index, value)
        print(f"[指令] 舵机 {index} -> {value:.2f}")
        get_logger().log_message("command", f"舵机 {index} -> {value:.2f}")

    async def land(self):
        """指令自动降落。（垂直下降）"""
        await self.drone.action.land()
        print("[指令] 降落")
        get_logger().log_message("command", "降落")

    # ------------------------------------------------------------------
    # 遥测查询（快照读取）
    # ------------------------------------------------------------------

    async def get_position_ned(self) -> PositionNedYaw:
        """
        获取当前 NED 位置（单次快照）。

        PositionBody 仅包含 x_m / y_m / z_m，不含航向。
        yaw 使用 FIELD_YAW_DEG（场地朝向），
        这对于初始 offboard setpoint 的位置保持场景是正确的。
        """
        async for odom in self.drone.telemetry.odometry():
            return PositionNedYaw(
                odom.position_body.x_m,
                odom.position_body.y_m,
                odom.position_body.z_m,
                self.FIELD_YAW_DEG,
            )

    async def get_altitude(self) -> float:
        """获取当前相对高度（缓存值，约 2 Hz 更新）。"""
        return self.health.altitude_m

    async def _read_altitude_direct(self) -> float:
        """直接读取当前相对高度（单次快照，不依赖缓存）。"""
        async for pos in self.drone.telemetry.position():
            return pos.relative_altitude_m

    async def get_heading(self) -> float:
        """获取当前航向角度。"""
        async for heading in self.drone.telemetry.heading():
            return heading.heading_deg

    # ------------------------------------------------------------------
    # 场地 NED 到真北 NED 的转换（旋转矩阵）
    # ------------------------------------------------------------------

    def field_to_ned(self, field_pos: PositionNedYaw) -> PositionNedYaw:
        """
        将场地坐标系的 PositionNedYaw 转换为真北 NED 坐标系。

        代码中所有任务坐标均在"场地 NED"坐标系中定义：
          - 原点 = HOME 点
          - N 轴 = 场地前方方向
          - E 轴 = 场地右方方向

        此方法对位置和航向同时应用二维旋转矩阵：

            [true_N]   [cos(θ)  -sin(θ)] [field_pos.north_m]
            [true_E] = [sin(θ)   cos(θ)] [field_pos.east_m]

            true_yaw  = field_pos.yaw_deg + θ

        其中 θ = FIELD_YAW_DEG（场地前方方向的真北方位角）。

        注意：down_m 直接透传（垂直轴两个坐标系共用），不做变换。

        参数:
            field_pos: 场地坐标系中的目标位姿
                       - north_m, east_m: 场地 NED 水平坐标（米）
                       - down_m:          真北 NED 垂直坐标（米，向下为正）
                       - yaw_deg:         场地坐标系中的目标航向（度，
                                          0°=场地前方）

        返回:
            PositionNedYaw，真北 NED 坐标系，可直接发送给 PX4
        """
        theta = math.radians(self.FIELD_YAW_DEG)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        true_north = field_pos.north_m * cos_t - field_pos.east_m * sin_t
        true_east = field_pos.north_m * sin_t + field_pos.east_m * cos_t
        true_yaw = field_pos.yaw_deg + self.FIELD_YAW_DEG

        return PositionNedYaw(true_north, true_east, field_pos.down_m, true_yaw)

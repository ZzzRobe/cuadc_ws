"""起飞状态 —— 通过 offboard setpoint 爬升至目标高度。

注意：初始起飞（地面 → 巡航高度）已改用 PX4 内建 action.takeoff()
在 main_fsm.run() 中完成。此状态用于任务中途需要改变飞行高度的场景。
"""

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from config import TAKEOFF_COMPLETE_THRESHOLD
from logger_manager import get_logger


class TakeoffState(BaseState):
    """通过 offboard set_position_ned 爬升至目标高度。"""

    def __init__(self, target_alt: float = 5.0, timeout_s: float = 30):
        super().__init__("Takeoff", timeout_s)
        self.target_alt = target_alt

    async def enter(self, interface):
        await super().enter(interface)
        # 在场地 NED 原点上空悬停爬升；心跳循环以 20 Hz 持续发送
        sp = interface.field_to_ned(PositionNedYaw(0.0, 0.0, -self.target_alt, 0.0))
        interface.update_setpoint(sp)
        print(f"[起飞] 目标高度 {self.target_alt:.1f} 米")
        get_logger().log_message(
            "takeoff", f"目标高度 {self.target_alt:.1f} 米")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "起飞超时"
            get_logger().log_message("takeoff", "起飞超时", "timeout")
            return ExecutionResult(done=True)

        alt = await interface.get_altitude()
        error = abs(alt - self.target_alt) / self.target_alt

        if error < TAKEOFF_COMPLETE_THRESHOLD:
            self.is_completed = True
            print(f"[起飞] 已到达目标高度 {alt:.1f} 米 "
                  f"(误差 {error:.1%})")
            get_logger().log_message(
                "takeoff",
                f"已到达目标高度 {alt:.1f} 米 (误差 {error:.1%})")
            return ExecutionResult(done=True)

        return ExecutionResult()

"""
降落 —— 触发 PX4 RTL 模式，飞控自主返航着陆。
"""

from .base_state import BaseState, ExecutionResult
from logger_manager import get_logger


class PrecisionLandState(BaseState):
    """降落：触发 RTL，PX4 自主爬升→返航→降落→自动 disarm。"""

    def __init__(self, timeout_s: float = 60):
        super().__init__("Land", timeout_s)

    async def enter(self, interface):
        await super().enter(interface)
        # ---- RTL 前设置返航参数 ----
        await interface.drone.param.set_param_float("MPC_XY_CRUISE", 5.0)
        await interface.drone.param.set_param_float("MPC_XY_VEL_MAX", 12.0)
        await interface.drone.param.set_param_float("RTL_RETURN_ALT", 10.0)
        print("[降落] RTL: 返航速度 5 m/s，返航高度 10 m")
        get_logger().log_message("land", "RTL: 返航速度 5 m/s，返航高度 10 m")
        await interface.drone.action.return_to_launch()

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "降落超时"
            get_logger().log_message("land", "降落超时", "timeout")
            return ExecutionResult(done=True)

        # 检测着陆完成：PX4 RTL 着陆后自动 disarm，armed 变为 False
        async for armed in interface.drone.telemetry.armed():
            if not armed:
                self.is_completed = True
                print("[降落] 着陆完成，已自动断开上锁")
                get_logger().log_message("land", "着陆完成，已自动断开上锁")
                return ExecutionResult(done=True)
            break

        return ExecutionResult()


# ============================================================================
# 原精准降落逻辑（视觉 + GPS 回退，暂时注释保留备用）
# ============================================================================
#
# """
# 精准降落 —— 双策略。
# ...
# """

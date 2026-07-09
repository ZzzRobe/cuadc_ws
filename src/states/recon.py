"""
侦察状态 —— 在侦察区域上方进行蛇形扫描。

在低空沿预计算的矩形扫描路径覆盖 8×5 米的侦察区域，
使 FPV 视频流能够捕捉危险识别标记。
不进行机载分类 —— 地面站人员观看视频流进行判读。

航点坐标使用场地 NED 坐标系（north=场地前方, east=场地右方, up=高度）。
"""

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from config import RECON_ZONE_DISTANCE_M, RECON_ALTITUDE_M, RECON_SCAN_STEP_M


class ReconState(BaseState):
    """在侦察区域上方进行蛇形扫描以进行视频识别。"""

    def __init__(self, timeout_s: float = 120):
        super().__init__("Recon", timeout_s)
        self._waypoints = []
        self._current_wp = 0

    async def enter(self, interface):
        await super().enter(interface)

        cx = RECON_ZONE_DISTANCE_M  # 侦察区域中心距离
        cy = 0.0                    # 中心线
        half_w, half_h = 4.0, 2.5   # 8×5 米区域的半宽和半高
        step = RECON_SCAN_STEP_M

        self._waypoints = [
            (cx,        cy + half_h, RECON_ALTITUDE_M),
            (cx,        cy - half_h, RECON_ALTITUDE_M),
            (cx + step, cy - half_h, RECON_ALTITUDE_M),
            (cx + step, cy + half_h, RECON_ALTITUDE_M),
            (cx + 2 * step, cy + half_h, RECON_ALTITUDE_M),
            (cx + 2 * step, cy - half_h, RECON_ALTITUDE_M),
            (cx + half_w, cy - half_h, RECON_ALTITUDE_M),
        ]
        self._current_wp = 0

        # 发送第一个航点
        wp = self._waypoints[0]
        sp = interface.field_to_ned(PositionNedYaw(wp[0], wp[1], -wp[2], 0.0))
        interface.update_setpoint(sp)

        print(f"[侦察] 开始蛇形扫描，"
              f"共 {len(self._waypoints)} 个航点")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "侦察超时 —— 部分扫描结果仍可使用"
            return ExecutionResult(done=True)

        if self._current_wp >= len(self._waypoints):
            print("[侦察] 扫描完成")
            self.is_completed = True
            return ExecutionResult(done=True)

        wp_x, wp_y, wp_z = self._waypoints[self._current_wp]
        alt = await interface.get_altitude()

        # 估算每个航点的时间（约 2-3 米间距，约 3 米/秒）
        dist_per_wp = 2.5
        est_time = dist_per_wp / 3.0

        if (self.elapsed() > (self._current_wp + 1) * est_time
                and abs(alt - wp_z) < 0.5):
            self._current_wp += 1
            if self._current_wp < len(self._waypoints):
                nx, ny, nz = self._waypoints[self._current_wp]
                sp = interface.field_to_ned(PositionNedYaw(nx, ny, -nz, 0.0))
                interface.update_setpoint(sp)
                print(f"[侦察] 航点 {self._current_wp}/{len(self._waypoints)}")

        return ExecutionResult()

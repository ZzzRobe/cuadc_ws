"""
PID 位置控制器 —— 输出目标位置 (m)。

API:
    (go_north, go_east) = pid.update(dx, dy, base_n, base_e, now)

    输入:  水平误差 dx, dy (m) + 基准位置 base_n, base_e (m) + 时间戳
    输出:  绝对目标坐标 go_north, go_east (m)
    调用方: 拿 go_north, go_east 自己去调 set_position_ned / field_to_ned

架构:
    视觉误差 (dx,dy)
        → PID → 目标位置 (go_north, go_east)
        → 外部函数调 set_position_ned / update_setpoint
        → 心跳 20Hz → PX4 Position Controller (200Hz+) → 速度 → 加速度 → 推力

    我们只算"目标应该在哪个坐标"，不管怎么通知飞控。
    PX4 负责后面所有高速闭环。

抗风原理:
    恒定风力 → 稳态误差 → I 项持续积分 → 自动生成恒定位置偏移补偿
    → 风力被精确抵消。不需风速传感器，不需手动调偏置。
"""

import time
from dataclasses import dataclass, field


@dataclass
class PositionPID:
    """离散位置 PID。一对 (dx, dy) 入，一对 (go_x, go_y) 出。"""

    # ---- 增益 ----
    kp: float = 0.6      # 比例: 0.5m 误差 → 0.3m 修正
    ki: float = 0.15     # 积分: 5 秒积出 0.15m 补偿 (中等风)
    kd: float = 0.20     # 微分: 抑制阵风导致的过冲

    # ---- 限幅 ----
    max_i: float   = 0.30   # 积分上限 (m) — 对应 ~3m/s 风的稳态偏移
    max_out: float = 1.00   # 单帧输出上限 (m) — 防止跳变

    # ---- 内部状态 (reset() 清零) ----
    _integral: float      = field(default=0.0, init=False)
    _prev_error: float    = field(default=0.0, init=False)
    _prev_time: float     = field(default=0.0, init=False)
    _prev_output: float   = field(default=0.0, init=False)

    # ------------------------------------------------------------------
    def reset(self):
        """重置积分器和历史状态。每次新对准任务开始时调用。"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = 0.0
        self._prev_output = 0.0

    # ------------------------------------------------------------------
    def update(self, error_m: float, now: float) -> float:
        """单轴 PID 步进。"""
        if self._prev_time == 0.0:
            self._prev_time = now
            self._prev_error = error_m
            out = self.kp * error_m
            out = max(-self.max_out, min(self.max_out, out))
            self._prev_output = out
            return out

        dt = now - self._prev_time
        if dt <= 0.0:
            return self._prev_output

        self._prev_time = now

        p = self.kp * error_m

        raw_i = self._integral + error_m * dt
        i = self.ki * raw_i
        out_raw = p + i + (self.kd * (error_m - self._prev_error) / dt)

        if abs(out_raw) < self.max_out or error_m * self._integral < 0:
            self._integral = raw_i

        self._integral = max(-self.max_i, min(self.max_i, self._integral))
        i = self.ki * self._integral

        d = self.kd * (error_m - self._prev_error) / dt
        self._prev_error = error_m

        out = p + i + d
        out = max(-self.max_out, min(self.max_out, out))
        self._prev_output = out
        return out


# ======================================================================
# 使用模板 — (go_north, go_east) = pid(dx, dy, base_n, base_e, now)
# ======================================================================
#
# PID 只输出目标坐标，调用方自己决定怎么通知飞控。

class AlignStateTemplate:
    """对准状态 — 位置版。PID → 目标坐标 → 外部调 set_position_ned。"""

    def __init__(self):
        # 1) 一个 PID 实例管两个轴
        self._pid_north = PositionPID(kp=0.6, ki=0.15, kd=0.2)
        self._pid_east  = PositionPID(kp=0.6, ki=0.15, kd=0.2)
        # 视觉检测变量 (实际由 _do_align 填充)
        self.cur_north = 0.0; self.cur_east = 0.0
        self.target_north = 0.0; self.target_east = 0.0

    async def enter(self, interface):
        self._pid_north.reset()
        self._pid_east.reset()

    async def execute(self, interface):
        # ... 视觉检测、超时检查 ...

        # 2) 视觉检测 → 水平误差
        #    假设 cur_north, cur_east 是桶在 field 坐标系下的偏移 (m)
        #    目标是把桶放在图像正中心 → target = (0, 0)
        dx = target_north - cur_north   # >0: 桶偏北，飞机需北移
        dy = target_east  - cur_east    # >0: 桶偏东，飞机需东移

        # 3) 基准位置 — 飞机当前的 field 坐标
        pos = await interface.get_position_ned()
        base_n = pos.north_m    # 真北 NED 坐标
        base_e = pos.east_m

        # 4) ★ 调用 PID — 输出目标坐标 ★
        now = time.monotonic()
        go_north = base_n + self._pid_north.update(dx, now)
        go_east  = base_e + self._pid_east.update(dy, now)
        #                       ↑
        #  PID.update(dx, now) 返回位置修正量 (m)
        #  base + 修正 = 绝对目标坐标

        # 5) 到达判定
        if abs(dx) < 0.05 and abs(dy) < 0.05:
            self.phase = "descend"
            return

        # 6) ★ 外部发送 — PID 不管 ★
        #    go_north, go_east 已经是绝对真北 NED 坐标
        #    (如 dx, dy 和 base 都是 field 坐标系，
        #     则 sp = field_to_ned(go_field_n, go_field_e, alt))
        #
        # 方案 A — 直接发:
        #   sp = PositionNedYaw(go_north, go_east, -alt, yaw)
        #   await interface.drone.offboard.set_position_ned(sp)
        #
        # 方案 B — 走心跳 (与 Transit/Search 一致):
        #   sp = field_to_ned(go_north, go_east, alt)   # 如在 field 坐标系
        #   interface.update_setpoint(sp)
        pass


# ======================================================================
# 独立运行模拟
# ======================================================================

if __name__ == "__main__":
    pid_n = PositionPID(kp=0.6, ki=0.15, kd=0.2)
    pid_e = PositionPID(kp=0.6, ki=0.15, kd=0.2)
    pid_n.reset()
    pid_e.reset()

    dx, base_n = 0.5, 30.0   # 北向误差 0.5m, 飞机在 field N=30.0
    dy, base_e = 0.3,  2.0   # 东向误差 0.3m, 飞机在 field E=2.0

    print(" t(s) | dx   | go_n  | dy   | go_e  |")
    print("------+------+-------+------+-------|")

    for step in range(40):
        now = time.monotonic()
        go_north = base_n + pid_n.update(dx, now)
        go_east  = base_e + pid_e.update(dy, now)

        if step % 4 == 0:
            print(f" {step*0.05:4.2f} | {dx:4.2f} | {go_north:5.2f} | "
                  f"{dy:4.2f} | {go_east:5.2f} |")

        dx -= (go_north - base_n) * 0.8   # 飞机移了修正量的 80%
        dy -= (go_east  - base_e) * 0.8
        base_n, base_e = go_north, go_east   # 基准位置跟进
        dx += 0.02                           # 模拟风

        time.sleep(0.05)

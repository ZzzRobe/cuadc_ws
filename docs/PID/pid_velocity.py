"""
PID 速度控制器 —— 输出速度指令 (m/s)。

API:
    (vx, vy) = pid.update(dx, dy, now)

    输入:  水平误差 dx, dy (m) + 时间戳
    输出:  速度指令 vx, vy (m/s)
    调用方: 拿 vx, vy 自己去调 set_velocity_ned / update_velocity_setpoint

架构:
    视觉误差 (dx, dy)
        → PID → 速度指令 (vx, vy)
        → 外部函数调 set_velocity_ned / update_velocity_setpoint
        → 心跳 20Hz → PX4 Velocity Controller (200Hz+) → 加速度 → 推力

    我们只算"应该用多大速度逼近"，不管怎么通知飞控。

⚠ 与位置版的关键区别:
    - 位置版: dx → 目标坐标，PX4 自动刹车
    - 速度版: dx → 速度，无位置边界 → 视觉丢失需外部清零速度
    - 退出速度模式前必须调 clear_velocity()，否则飞机漂移

抗风原理:
    恒定风力 → 稳态误差 → I 项持续积分 → 自动生成恒定速度补偿
    → 风力被精确抵消。不需风速传感器，不需手动调偏置。
"""

import time
from dataclasses import dataclass, field


@dataclass
class VelocityPID:
    """离散速度 PID。一对 (dx, dy) 入，一对 (vx, vy) 出。"""

    # ---- 增益 ----
    kp: float = 0.6      # 比例: 0.5m 误差 → 0.3m/s 速度
    ki: float = 0.15     # 积分: 5 秒积出 0.15m/s 补偿 (中等风)
    kd: float = 0.20     # 微分: 阵风抑制

    # ---- 限幅 ----
    max_i: float   = 0.80   # 积分上限 (m/s) — 避免积分失控
    max_out: float = 1.50   # 速度输出上限 (m/s)

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
# 使用模板 — (vx, vy) = pid(dx, dy, now)
# ======================================================================
#
# PID 只输出速度，调用方自己决定怎么通知飞控。

class AlignStateTemplate:
    """对准状态 — 速度版。PID → 速度 → 外部调 set_velocity_ned。"""

    def __init__(self):
        # 1) 一个 PID 实例管两个轴
        self._pid_north = VelocityPID(kp=0.6, ki=0.15, kd=0.2)
        self._pid_east  = VelocityPID(kp=0.6, ki=0.15, kd=0.2)
        # 视觉检测变量 (实际由 _do_align 填充)
        self.cur_north = 0.0; self.cur_east = 0.0
        self.target_north = 0.0; self.target_east = 0.0

    async def enter(self, interface):
        # 2) 每次进入对准时重置
        self._pid_north.reset()
        self._pid_east.reset()

    async def exit(self, interface):
        # 3) ★ 退出前必须清零速度 ★
        interface.clear_velocity()

    async def execute(self, interface):
        # ... 视觉检测、超时检查 ...

        # 4) 视觉检测 → 水平误差
        dx = self.target_north - self.cur_north   # >0: 桶偏北，飞机需北移
        dy = self.target_east  - self.cur_east

        # 5) ★ 调用 PID — 输出速度指令 ★
        now = time.monotonic()
        vx = self._pid_north.update(dx, now)   # → 北向速度 (m/s)
        vy = self._pid_east.update(dy, now)    # → 东向速度 (m/s)
        #                  ↑
        #  PID.update(dx, now) 直接返回速度值
        #  不需基准位置 — 速度模式没有"绝对目标"概念

        # 6) 到达判定
        if abs(dx) < 0.05 and abs(dy) < 0.05:
            interface.clear_velocity()          # ★ 先停速再切阶段
            self.phase = "descend"
            return

        # 7) ★ 外部发送 — PID 不管 ★
        # 方案 A — 直接发:
        #   vel = VelocityNedYaw(vx, vy, 0.0, yaw)
        #   await interface.drone.offboard.set_velocity_ned(vel)
        #
        # 方案 B — 走心跳:
        #   vel = VelocityNedYaw(vx, vy, 0.0, interface.FIELD_YAW_DEG)
        #   interface.update_velocity_setpoint(vel)
        pass

    async def _handle_visual_loss(self, interface):
        # ★ 视觉丢失时必须先清速度再切状态 ★
        interface.clear_velocity()
        # return ExecutionResult(interrupt=HoverState())


# ======================================================================
# 独立运行模拟
# ======================================================================

if __name__ == "__main__":
    pid_n = VelocityPID(kp=0.6, ki=0.15, kd=0.2)
    pid_e = VelocityPID(kp=0.6, ki=0.15, kd=0.2)
    pid_n.reset()
    pid_e.reset()

    dx, dy = 0.5, 0.3    # 初始误差: 桶偏北 0.5m, 偏东 0.3m

    print(" t(s) | dx   | vx    | dy   | vy    |")
    print("------+------+-------+------+-------|")

    for step in range(40):
        now = time.monotonic()
        vx = pid_n.update(dx, now)
        vy = pid_e.update(dy, now)

        if step % 4 == 0:
            print(f" {step*0.05:4.2f} | {dx:4.2f} | {vx:5.2f} | "
                  f"{dy:4.2f} | {vy:5.2f} |")

        dx -= vx * 0.05 * 0.6    # 速度→位移 (效率 0.6)
        dy -= vy * 0.05 * 0.6
        dx += 0.02                 # 模拟 1.5m/s 北风

        time.sleep(0.05)

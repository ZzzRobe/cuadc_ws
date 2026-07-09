"""
对准状态 —— 下降并通过视觉伺服居中到目标圆柱体上方。

从 interface.shared 读取目标（由 SearchState 填充）。
两个子阶段：下降至约 3 米，然后视觉伺服 P 控制对准。

ned_offset 使用场地 NED 坐标系（与 field_to_ned 的输入坐标系一致）。
"""

import asyncio

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from config import (
    DROP_ALIGN_ALTITUDE_M, DROP_ZONE_DISTANCE_M,
    ALIGN_THRESHOLD_M, SEARCH_TIMEOUT_S, VISUAL_SERVO_KP,
)


class AlignState(BaseState):
    """通过视觉伺服在目标圆柱体上方进行精细对准。"""

    def __init__(self, bottle_index: int, timeout_s: float = 60):
        super().__init__("Align", timeout_s)
        self.bottle_index = bottle_index

        # 子阶段
        self.phase = "descend"  # descend → servo → done
        self._search_start = 0.0
        self._target = None

    async def enter(self, interface):
        await super().enter(interface)

        targets = interface.shared.get("drop_targets")
        if targets is None:
            self.error = "共享缓存中没有粗略检测结果"
            print(f"[对准] {self.error}")
            return

        idx = 0 if self.bottle_index == 1 else 1
        self._target = targets[idx]

        if self._target is None:
            self.error = f"瓶子 {self.bottle_index} 没有分配目标"
            print(f"[对准] {self.error}")
            return

        # 向目标圆柱体下降
        sp = interface.field_to_ned(PositionNedYaw(
            DROP_ZONE_DISTANCE_M + self._target.ned_offset[0],
            self._target.ned_offset[1],
            -DROP_ALIGN_ALTITUDE_M,
            0.0,
        ))
        interface.update_setpoint(sp)
        self.phase = "descend"
        self._search_start = self.elapsed()
        print(f"[对准] 瓶子 {self.bottle_index}: 正在下降到 "
              f"目标上方 {DROP_ALIGN_ALTITUDE_M:.1f} 米")

    async def execute(self, interface):
        if self.is_timed_out():
            self.error = "对准超时"
            return ExecutionResult(done=True)

        if self._target is None:
            return ExecutionResult(done=True)  # enter() 已设置错误

        if self.phase == "descend":
            return await self._do_descend(interface)
        elif self.phase == "servo":
            return await self._do_visual_servo(interface)
        return ExecutionResult()

    async def _do_descend(self, interface):
        """等待高度降到约 3 米。"""
        alt = await interface.get_altitude()
        if alt <= 3.0:
            self.phase = "servo"
            self._search_start = self.elapsed()
            print(f"[对准] 瓶子 {self.bottle_index}: 开始视觉伺服")
        return ExecutionResult()

    async def _do_visual_servo(self, interface):
        """视觉伺服循环 —— 检测圆柱体，计算偏移，P 控制。"""
        from config import YOLO_CONFIDENCE_THRESHOLD

        alt = await interface.get_altitude()

        try:
            from vision.yolo_detector import get_detector
            detector = get_detector()
            frame = await _capture_frame_async()
            cylinders = detector.detect_cylinders(frame, alt)
        except Exception as e:
            print(f"[对准] 检测错误: {e}")
            return ExecutionResult()

        best = self._match_target(cylinders)

        if best is None:
            # 目标丢失 —— 搜索超时
            if self.elapsed() - self._search_start > SEARCH_TIMEOUT_S:
                print(f"[警告] 对准 瓶子 {self.bottle_index}: "
                      f"目标丢失超时，放弃")
                return ExecutionResult(done=True)

            # 保持位置，向最后已知位置漂移
            sp = interface.field_to_ned(PositionNedYaw(
                DROP_ZONE_DISTANCE_M + self._target.ned_offset[0],
                self._target.ned_offset[1],
                -alt,
                0.0,
            ))
            interface.update_setpoint(sp)
            return ExecutionResult()

        self._search_start = self.elapsed()  # 重置搜索计时器
        offset_x, offset_y = best.ned_offset
        self._target = best  # 用更精确的低空估计更新

        # 检查对准
        if (abs(offset_x) < ALIGN_THRESHOLD_M
                and abs(offset_y) < ALIGN_THRESHOLD_M):
            interface.shared[f"bottle_{self.bottle_index}_aligned"] = True
            interface.shared[f"bottle_{self.bottle_index}_position"] = best
            self.is_completed = True
            print(f"[对准] 瓶子 {self.bottle_index}: 已对准")
            return ExecutionResult(done=True)

        # P 控制位置调整
        sp = interface.field_to_ned(PositionNedYaw(
            DROP_ZONE_DISTANCE_M + offset_x * VISUAL_SERVO_KP,
            offset_y * VISUAL_SERVO_KP,
            -alt,
            0.0,
        ))
        interface.update_setpoint(sp)
        return ExecutionResult()

    def _match_target(self, cylinders: list):
        """通过最近 NED 偏移匹配检测到的圆柱体与目标。"""
        if not cylinders:
            return None
        tx, ty = self._target.ned_offset
        best = min(cylinders,
                   key=lambda c: (c.ned_offset[0] - tx) ** 2
                                 + (c.ned_offset[1] - ty) ** 2)
        return best


async def _capture_frame_async():
    """从相机捕获单帧图像（在线程池中运行）。"""
    import asyncio
    import concurrent.futures

    from vision.yolo_detector import _camera

    if _camera is None:
        raise RuntimeError("相机未初始化")

    loop = asyncio.get_running_loop()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    ret, frame = await loop.run_in_executor(executor, _camera.read)
    if not ret:
        raise RuntimeError("捕获帧失败")
    return frame

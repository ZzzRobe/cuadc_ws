"""
搜索状态 —— 矩形航线巡逻 + VisionPipeline 检测 + 目标匹配。

飞行模式：
  沿矩形航线四顶点循环飞行，每帧执行
  VisionPipeline 检测（YOLO → Canny 边缘 → HoughCircles → 针孔直径）。
  检测到目标圆柱体后，匹配直径判断瓶型（15cm / 20cm），
  将目标信息存入 interface.shared，通过栈式抢占切入 AlignState。

  对准完成后 resume 继续巡逻搜索。

飞向航点：
  使用与 TransitState 相同的 PX4 原生位置控制逻辑：
  设置 MPC_XY_VEL_MAX，发送目标 setpoint 由心跳维持，
  PX4 内部 Position Controller 自主飞行（200Hz+）。
  心跳以 20Hz 发送缓存 setpoint，_fly_to_target 每周期刷新缓存，
  不做直接 set_position_ned 调用（避免与心跳冲突）。

目标匹配逻辑：
  - 15cm 瓶（goal[0]）：abs(diameter_cm - 15) ≤ EPSILON_DIAMETER_CM
  - 20cm 瓶（goal[1]）：abs(diameter_cm - 20) ≤ EPSILON_DIAMETER_CM

坐标系：
  图像上方 = 场地北，图像右方 = 场地东
  pixel_to_ned_offset 将圆心像素坐标转换为场地 NED 偏移量。
"""

import math
from dataclasses import dataclass
from typing import Tuple, TYPE_CHECKING

from mavsdk.offboard import PositionNedYaw

from .base_state import BaseState, ExecutionResult
from .align import AlignState
from config import (CRUISE_ALTITUDE_M, ARRIVAL_THRESHOLD_M,
                    EPSILON_DIAMETER_CM,
                    SEARCH_SPEED_MPS,
                    SEARCH_RECT_HALF_N_M, SEARCH_RECT_HALF_E_M,
                    SEARCH_RECT_CENTER_N_M, SEARCH_RECT_CENTER_E_M)
from vision.camera import capture_frame_async
from vision.circle_detector import DEFAULT_CAMERA_MATRIX
from vision.pipeline import VisionPipeline

if TYPE_CHECKING:
    from interface import PX4Interface

# 相机内参 — 从 CircleDetector 提取，避免硬编码不同步
_FX = float(DEFAULT_CAMERA_MATRIX[0, 0])
_FY = float(DEFAULT_CAMERA_MATRIX[1, 1])
_CX = float(DEFAULT_CAMERA_MATRIX[0, 2])
_CY = float(DEFAULT_CAMERA_MATRIX[1, 2])
_BUCKET_HEIGHT_M = 0.30


# ---------------------------------------------------------------------------
# 工具：像素坐标 → 场地 NED 偏移
# ---------------------------------------------------------------------------

def pixel_to_ned_offset(cx_px: float, cy_px: float, alt_rel_m: float,
                        fx: float = _FX, fy: float = _FY,
                        cx: float = _CX, cy: float = _CY,
                        bucket_h: float = _BUCKET_HEIGHT_M) -> Tuple[float, float]:
    """下视相机像素坐标 → 场地 NED 偏移 (north_m, east_m)。

    图像上方=场地北，图像右方=场地东。"""
    z_c = alt_rel_m - bucket_h
    if z_c <= 0:
        return (0.0, 0.0)
    dx = cx_px - cx          # 右 = 东
    dy = cy_px - cy          # 下 = 南
    east_m = dx * z_c / fx
    north_m = -dy * z_c / fy  # 下 = 南 = -北
    return (north_m, east_m)


# ---------------------------------------------------------------------------
# 数据桥接：VisionPipeline 检测结果 → AlignState
# ---------------------------------------------------------------------------

@dataclass
class CylinderTarget:
    """SearchState 检测目标，AlignState 通过 .ned_offset[0/1] 消费。"""
    ned_offset: Tuple[float, float]  # (north_m, east_m)
    diameter_m: float
    conf: float
    circle_cx_px: int
    circle_cy_px: int


class SearchState(BaseState):

    def __init__(self, timeout_s: float = 120):
        super().__init__("Search", timeout_s)
        self.goal = [0, 0]          # 0=未找到, 1=已找到
        self._rect_waypoints = []   # 矩形航线航点列表（enter 中从 config 计算）
        self._wp_index = 0
        self._original_vel_max = None  # 原始 MPC_XY_VEL_MAX，退出时恢复

        # 视觉流水线: YOLO → Canny边缘 → HoughCircles → 直径
        # 模型加载失败（无 GPU / TensorRT 不匹配）时降级为无视觉模式，仅巡逻飞行
        self.pipeline = None
        try:
            self.pipeline = VisionPipeline(
                model_path=None,  # 使用 YOLODetector 默认路径 (src/vision/models/)
                yolo_conf=0.5,
                circle_conf_threshold=0.3,
            )
        except Exception as e:
            print(f"[搜索] VisionPipeline 初始化失败: {e}", flush=True)
            print("[搜索] 将以无视觉模式运行（仅巡逻飞行）", flush=True)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    async def enter(self, interface: "PX4Interface"):
        await super().enter(interface)

        # ---- 从 config 计算矩形航线四顶点 ----
        # 矩形 3m(N) × 6m(E)，与投放区同心
        cn = SEARCH_RECT_CENTER_N_M
        ce = SEARCH_RECT_CENTER_E_M
        hn = SEARCH_RECT_HALF_N_M
        he = SEARCH_RECT_HALF_E_M
        self._rect_waypoints = [
            PositionNedYaw(cn - hn, ce + he, -CRUISE_ALTITUDE_M, 0.0),  # 后右（起点）
            PositionNedYaw(cn + hn, ce + he, -CRUISE_ALTITUDE_M, 0.0),  # 前右
            PositionNedYaw(cn + hn, ce - he, -CRUISE_ALTITUDE_M, 0.0),  # 前左
            PositionNedYaw(cn - hn, ce - he, -CRUISE_ALTITUDE_M, 0.0),  # 后左
        ]
        # 计算每个航点的目标航向：从上一个航点指向当前航点的方位角
        for i in range(len(self._rect_waypoints)):
            prev = self._rect_waypoints[(i - 1) % len(self._rect_waypoints)]
            dn = self._rect_waypoints[i].north_m - prev.north_m
            de = self._rect_waypoints[i].east_m - prev.east_m
            yaw = math.degrees(math.atan2(de, dn))
            self._rect_waypoints[i] = PositionNedYaw(
                self._rect_waypoints[i].north_m,
                self._rect_waypoints[i].east_m,
                self._rect_waypoints[i].down_m,
                yaw,
            )
        self._wp_index = 0

        # ---- 启动 PX4 原生位置飞行（设置限速 + 发送第一个航点 setpoint） ----
        target = interface.field_to_ned(self._rect_waypoints[0])
        self._original_vel_max = await interface.start_position_flight(
            target, SEARCH_SPEED_MPS)

        print(f"[搜索] 矩形航线开始，{len(self._rect_waypoints)} 个航点，"
              f"高度 {CRUISE_ALTITUDE_M:.1f}m，速度 {SEARCH_SPEED_MPS:.1f} m/s",
              flush=True)

    async def execute(self, interface: "PX4Interface"):
        # ---- 超时 ----
        if self.is_timed_out():
            self.error = "搜索超时"
            return ExecutionResult(done=True)

        # ---- 飞到当前航点 ----
        # _fly_to_target 会刷新心跳 setpoint 缓存，
        # PX4 Position Controller 以 200Hz+ 自主飞行。
        arrived = await self._fly_to_target(interface, self._wp_index)

        # ---- 执行视觉检测（无 pipeline 或异常时静默跳过） ----
        if self.pipeline is not None:
            try:
                alt = await interface.get_altitude()
                frame = await capture_frame_async()

                # VisionPipeline: YOLO → Canny边缘 → HoughCircles → 针孔模型算直径
                results = self.pipeline.process_frame(frame, alt_rel_m=alt)

                for r in results:
                    if not r["edge_success"]:
                        continue                # 圆检测失败，跳过

                    diameter_cm = r["diameter_m"] * 100   # 真实直径 (cm)
                    # 匹配 15cm 瓶 (goal[0])
                    if abs(diameter_cm - 15) <= EPSILON_DIAMETER_CM and self.goal[0] == 0:
                        self.goal[0] = 1
                        self._save_detection(interface, bottle=1, result=r, alt_m=alt)
                        # 栈式抢占：挂起搜索 → 压入对准 → 对准完成后 resume 继续搜索
                        return ExecutionResult(interrupt=AlignState(bottle_index=1))
                    # 匹配 20cm 瓶 (goal[1])
                    elif abs(diameter_cm - 20) <= EPSILON_DIAMETER_CM and self.goal[1] == 0:
                        self.goal[1] = 1
                        self._save_detection(interface, bottle=2, result=r, alt_m=alt)
                        return ExecutionResult(interrupt=AlignState(bottle_index=2))
            except Exception as e:
                print(f"[搜索] 视觉检测异常: {e}", flush=True)

        if not arrived:
            return ExecutionResult()     # 还在路上，下一帧继续飞 + 检测

        # ---- 到达当前航点 → 推进到下一个，绕圈循环 ----
        self._wp_index = (self._wp_index + 1) % len(self._rect_waypoints)
        target = interface.field_to_ned(self._rect_waypoints[self._wp_index])
        interface.update_setpoint(target)
        return ExecutionResult()

    async def exit(self, interface: "PX4Interface"):
        """退出时恢复原始 MPC_XY_VEL_MAX。"""
        await interface.restore_cruise_speed(self._original_vel_max)
        await super().exit(interface)

    # ------------------------------------------------------------------
    # 航点飞行（PX4 原生位置控制，心跳维持 setpoint）
    # ------------------------------------------------------------------

    async def _fly_to_target(self, interface: "PX4Interface",
                              wp_idx: int) -> bool:
        """
        刷新心跳 setpoint 并检查是否到达航点 wp_idx。

        每周期调用 update_setpoint（同步变量赋值，零开销），
        心跳以 20Hz 将缓存 setpoint 发送给 PX4。
        PX4 Position Controller 内部以 200Hz+ 处理全程飞行。

        不做直接 set_position_ned 调用 —— 所有 setpoint 统一经心跳发送，
        避免双路径交替引发的模式切换/振荡问题。

        返回 True 表示已到达（距离 < ARRIVAL_THRESHOLD_M）。
        """
        target = interface.field_to_ned(self._rect_waypoints[wp_idx])

        # 刷新心跳缓存 —— 确保 resume 后心跳也发送正确目标
        interface.update_setpoint(target)

        pos = await interface.get_position_ned()
        dn = target.north_m - pos.north_m
        de = target.east_m - pos.east_m
        dist = math.hypot(dn, de)
        return dist < ARRIVAL_THRESHOLD_M

    # ------------------------------------------------------------------
    # 检测结果 → 共享缓存
    # ------------------------------------------------------------------

    def _save_detection(self, interface: "PX4Interface", bottle: int,
                         result: dict, alt_m: float):
        """
        将检测结果写入 interface.shared，供 AlignState.enter() 读取。

        VisionPipeline 结果 → 像素转 NED → CylinderTarget →
        shared["drop_targets"][bottle-1]

        AlignState 期望:
            shared["drop_targets"] = [target1_or_None, target2_or_None]
            target.ned_offset[0]  → 场地北偏移 (米)
            target.ned_offset[1]  → 场地东偏移 (米)
        """
        circle = result["circle"]
        ned = pixel_to_ned_offset(circle.cx_px, circle.cy_px, alt_m)
        target = CylinderTarget(
            ned_offset=ned,
            diameter_m=result["diameter_m"],
            conf=result["det"]["conf"],
            circle_cx_px=circle.cx_px,
            circle_cy_px=circle.cy_px,
        )
        if "drop_targets" not in interface.shared:
            interface.shared["drop_targets"] = [None, None]
        interface.shared["drop_targets"][bottle - 1] = target

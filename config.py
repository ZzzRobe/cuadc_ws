"""
CUADC 2026 任务全局参数。

所有状态模块从此处导入 —— 禁止硬编码。
比赛前根据场地实测数据调整这些参数。
"""

# ---------------------------------------------------------------------------
# 场地朝向 —— 场地"前方"方向相对于真北的方位角。
#
# 这是将场地对齐的 NED 坐标系（代码中使用的坐标系，其"北"轴指向场地
# 前方）旋转至真北 NED 坐标系（PX4 使用的坐标系）的旋转角度。
#
# 赛前准备：
#   1. 使用指南针/手机罗盘测量场地前方方向相对于真北的方位角。
#   2. 将测量值填入下面的参数（单位：度）。
#   3. 飞机放在起降点时，机头可以朝向任意方向（不再要求对准场地前方）。
#
# 坐标系约定：
#   - 代码中所有任务坐标均以 HOME 点为原点，使用"场地 NED"坐标系：
#     * N（场地北）= 场地前方方向
#     * E（场地东）= 场地右方方向
#     * D（下）= 垂直向下
#   - PX4Interface.field_to_ned() 内使用旋转矩阵将场地 NED 坐标映射
#     为真北 NED 坐标后发送给 PX4。
#
# 旋转矩阵：
#   [true_N]   [cos(θ)  -sin(θ)] [field_N]
#   [true_E] = [sin(θ)   cos(θ)] [field_E]
#   其中 θ = FIELD_YAW_DEG
# ---------------------------------------------------------------------------
FIELD_YAW_DEG = 0.0

# ---------------------------------------------------------------------------
# 飞行参数
# ---------------------------------------------------------------------------
CRUISE_ALTITUDE_M = 5.0          # 起飞后的安全巡航高度(uncertain)
DROP_ZONE_DISTANCE_M = 30.0      # 起飞点到投掷区前缘的距离
DROP_ZONE_HALF_N_M = 2.5        # 投掷区 N 方向半长（5m/2）
RECON_ZONE_DISTANCE_M = 55.0     # 起飞点到侦察区的距离
DROP_ALIGN_ALTITUDE_M = 2.0      # 圆柱体上方精细对准高度(uncertain)
RECON_ALTITUDE_M = 3.0           # 侦察扫描高度(uncertain)
LAND_START_ALTITUDE_M = 5.0      # 开始降落序列时的高度
LAND_SAFE_ALTITUDE_M = 2.0       # 低于此高度时使用慢速下降(unused)
TRANSIT_SPEED_MPS = 5.0          # 区域间巡航速度(uncertain)
SEARCH_SPEED_MPS = 1.0              # 搜索矩形航线飞行速度(uncertain)
ARRIVAL_REL_THRESHOLD = 0.05        # 航点到达判据：剩余距离 < 航段长度×此值即认为到达
BOTTLE_DIAMETER_TOLERANCE_CM = 2.0  # 瓶子直径匹配容差 (cm)，用于搜索状态识别目标

# 搜索矩形几何 —— 与投放区 (5m×8m) 同心，短边沿飞行前方 (N)
#   N方向 (短边, 沿飞行前方): 3m → half = 1.5m
#   E方向 (长边, 沿飞行右方): 6m → half = 3.0m
SEARCH_RECT_HALF_N_M = 1.5          # 搜索矩形 N 半长（短边/2，沿飞行前方）
SEARCH_RECT_HALF_E_M = 3.0          # 搜索矩形 E 半长（长边/2，沿飞行右方）
SEARCH_RECT_CENTER_N_M = DROP_ZONE_DISTANCE_M + DROP_ZONE_HALF_N_M  # 搜索矩形与投放区同心
SEARCH_RECT_CENTER_E_M = 0.0                    # 搜索矩形中心 E 坐标

# ---------------------------------------------------------------------------
# 控制参数
# ---------------------------------------------------------------------------
OFFBOARD_HEARTBEAT_HZ = 20         # offboard 设定值发送频率（须 >= 2 Hz）
FSM_LOOP_HZ = 20                   # 状态机主循环频率
MAX_STACK_DEPTH = 5                # 状态栈最大深度（防止无限抢占）
TAKEOFF_COMPLETE_THRESHOLD = 0.05  # 起飞完成判据：相对误差小于此值即认为到达目标高度
ALIGN_THRESHOLD_M = 0.05           # 视觉伺服对准阈值
ARRIVAL_THRESHOLD_M = 0.5          # 到达航点的距离判定阈值
SEARCH_TIMEOUT_S = 3.0             # 目标重捕获超时(超时改成在状态时长处理)
VISUAL_SERVO_KP = 0.5              # 视觉伺服 P 控制器增益
LAND_DESCEND_RATE_MPS = 0.3        # 降落时的正常下降速率
DROP_ALTITUDE_M = 5.0              # 投掷阶段粗略接近高度

# ---------------------------------------------------------------------------
# 视觉参数
# ---------------------------------------------------------------------------
YOLO_CONFIDENCE_THRESHOLD = 0.5  # YOLO 检测置信度阈值
CIRCLE_CONF_THRESHOLD = 0.3       # HoughCircles 圆检测的最低 YOLO 置信度
YOLO_MODEL_PATH = "models/yolov11n_800_best_FP16.engine"  # YOLO 模型路径
RECON_CONFIDENCE_THRESHOLD = 0.7 # 侦察分类置信度阈值
RECON_SCAN_STEP_M = 1.5          # 侦察区域扫描线间距
EPSILON_DIAMETER_CM = 2.0        # 圆柱体直径匹配容差（15±2cm, 20±2cm）

# ---------------------------------------------------------------------------
# 安全参数
# ---------------------------------------------------------------------------
BATTERY_LOW_THRESHOLD_PCT = 20.0 # 电量低于此百分比时触发返航
GPS_FIX_MIN = 3                  # 最低 GPS 定位类型（3 = 3D 定位）
GLOBAL_GUARD_INTERVAL_S = 0.05   # 健康检查间隔（与 FSM 循环频率匹配）

# CUADC 2026 — 多旋翼无人机侦察与救援

机载电脑自主飞行控制程序。基于 MAVSDK Python 与 PX4 飞控通信，实现全场自主任务：起飞 → 投放区精准空投 → 侦察区危险品识别 → 返航着陆。

## 技术栈

| 层级 | 组件 | 版本 |
|------|------|------|
| 飞控固件 | PX4 Autopilot | v1.16.1 |
| 通信库 | MAVSDK Python | v3.15.3 |
| 仿真 | Gazebo Harmonic | 8.12 |
| 视觉 | YOLOv8 (Ultralytics) | ≥8.0 |
| 图像处理 | OpenCV | ≥4.8 |
| 运行环境 | Python | ≥3.12 |

## 架构

```
cuadc_ws/
├── run_mission.py              # 任务入口
├── config.py                   # 全局参数（场地/飞行/视觉/安全）
├── requirements.txt
├── scripts/
│   ├── run_sitl.sh             # 单独启动 PX4 SITL
│   ├── run_mission_sim.sh      # tmux 双窗格：SITL + 任务程序
│   └── setup_px4_sitl.sh       # 一键环境搭建
├── src/
│   ├── interface.py            # PX4 通信层（心跳/看门狗/指令/NED转换）
│   ├── main_fsm.py             # 状态机引擎
│   ├── logger_manager.py       # 黑匣子日志
│   ├── config.py               # 全局参数
│   ├── states/
│   │   ├── takeoff.py          # 起飞
│   │   ├── hover.py            # 悬停
│   │   ├── transit.py          # 航线飞行
│   │   ├── search.py           # 粗检测（投放区圆筒）
│   │   ├── align.py            # 精对准（视觉伺服）
│   │   ├── drop.py             # 投放（舵机释放）
│   │   ├── recon.py            # 侦察（扫描危险品标识）
│   │   └── land.py             # 精确着陆
│   └── vision/
│       ├── yolo_detector.py    # YOLOv8 目标检测
│       └── object_size_calculator.py  # 针孔相机模型
└── docs/
    └── v2/                     # 设计文档
```

### 通信架构

```
机载电脑 (Python)                   PX4 飞控
┌──────────────────┐    MAVLink    ┌──────────────┐
│ MAVSDK Python    │◄─────────────►│ px4 SITL 进程  │
│ (asyncio)        │  UDP :14540   │              │
│                  │  ← :14580     │              │
│ _heartbeat_loop  │───setpoint───►│ Offboard 模式  │
│ global_guard     │◄──telemetry───│              │
└──────────────────┘               └──────┬───────┘
                                         │ gz_bridge
                                         ▼
                               ┌──────────────────┐
                               │  Gazebo Harmonic  │
                               │  gz sim + GUI     │
                               └──────────────────┘
```

**MAVLink 端口说明（Gazebo SITL）**：

| 端口 | 方向 | 用途 |
|------|------|------|
| UDP 14540 | PX4 → 程序 | offboard 遥测下行 |
| UDP 14580 | 程序 → PX4 | offboard 指令上行 |
| UDP 18570 | PX4 ↔ QGC | 地面站 |

## 环境搭建

### 1. 克隆 PX4

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.16.1 --depth 1
```

### 2. 安装系统依赖

```bash
# Gazebo Harmonic
sudo apt-get install gz-harmonic libgz-sim8-dev

# PX4 构建依赖
sudo apt-get install cmake ninja-build gcc g++ make \
    libopencv-dev libeigen3-dev libtinyxml2-dev

# Python 包管理器
sudo apt-get install python3-pip
```

### 3. 安装 Python 依赖

```bash
cd ~/Code/cuadc_ws
python3 -m pip install --break-system-packages -r requirements.txt

# PX4 构建所需的额外 Python 包
python3 -m pip install --break-system-packages \
    kconfiglib empy==3.3.4 jinja2 toml packaging pyyaml \
    jsonschema pyros-genmsg
```

### 4. 构建 PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

## 启动仿真

### 终端 1 — 启动 PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot

# 默认渲染引擎 (Ogre2)
make px4_sitl gz_x500

# 若画面黑屏/空白（虚拟机等），使用 Ogre1
PX4_GZ_SIM_RENDER_ENGINE=ogre make px4_sitl gz_x500

# 使用其他地图
PX4_GZ_WORLD=grid PX4_GZ_SIM_RENDER_ENGINE=ogre   PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

### 终端 2 — 运行任务程序

```bash
cd ~/Code/cuadc_ws
python3 run_mission.py --sim
```

### 一键启动（tmux 双窗格）

```bash
cd ~/Code/cuadc_ws
bash scripts/run_mission_sim.sh gz_x500
```

## 关键参数

所有参数在 `config.py` 中统一定义，赛前根据场地实测调整：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FIELD_YAW_DEG` | 0.0 | 场地前方方向的真北方位角（°），赛前手动测量 |
| `CRUISE_ALTITUDE_M` | 7.0 | 安全巡航高度 |
| `DROP_ZONE_DISTANCE_M` | 30.0 | 投放区距离 |
| `RECON_ZONE_DISTANCE_M` | 55.0 | 侦察区距离 |
| `TRANSIT_SPEED_MPS` | 5.0 | 航线速度 |
| `DROP_ALIGN_ALTITUDE_M` | 2.5 | 投放对准高度 |
| `RECON_ALTITUDE_M` | 2.5 | 侦察飞行高度 |
| `BATTERY_LOW_THRESHOLD_PCT` | 20.0 | 低电量返航阈值 |
| `OFFBOARD_HEARTBEAT_HZ` | 20 | offboard 心跳频率 |

## 任务流程

```
arm → [PX4内建起飞至7m] → Hover(1s) → Transit(30m) → Hover(1s)
    → Search(粗检测圆筒) → Align(瓶1) → Drop(瓶1)
    → Align(瓶2) → Drop(瓶2)
    → Transit(55m) → Hover(1s) → Recon(侦察扫描)
    → Transit(返航) → Hover(1s) → PrecisionLand
    → disarm
```

## 安全机制

1. **全局健康看门狗** — 每个主循环周期检查连接/武装/GPS/电池状态，异常时强制悬停
2. **Offboard 心跳** — 独立 asyncio Task 以 20Hz 维持 setpoint 发送，主逻辑卡顿不会导致 PX4 退出 Offboard
3. **低电量返航** — 电池低于阈值自动触发返航
4. **状态超时保护** — 每个状态有独立超时时间，超时跳过或降级
5. **异常兜底** — 未捕获异常自动触发着陆

## TODO:

- [x] 在`main_fsm.py`中的`_handle_unhealthy`处理：怎么处理？合理性？
- [x] 坐标系转换问题 — 已改为手动配置 FIELD_YAW_DEG + 旋转矩阵方案
- [x] 心跳问题
- [] search中处理图像和后续的处理图像时需要考虑上下文
## 许可证

CUADC 2026 竞赛项目 — 仅供团队内部使用

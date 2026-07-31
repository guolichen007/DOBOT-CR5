# DOBOT CR5 三相机标定、三维重建与喷涂研发平台

基于 ROS Noetic + Gazebo Classic 11 的 DOBOT CR5 多相机标定系统。
集成三台固定 RGB-D 相机、多面标定目标、成对相机相对标定、TSDF 三维重建模块。

## 1. 当前能力

### 三相机标定 (稳定版 V1)

**算法主线：** 标定靶检测 → 单相机 PnP → 成对相机相对变换 → RANSAC 共识 → SE(3) 平均

- 五面标定目标 (前/后 ChArUco, 左 AprilTag, 右 ArUco, 顶 AprilTag)
- 三相机同步采集 (skew ≤ 5ms, 640×480@10Hz)
- truth-free 成对相机相对求解器 (pairwise_solver.py)
- 前左相机固定为 rig 基准, 输出 FL→FR 和 FL→RE
- Gazebo 仿真场景完整验证

**Gazebo 稳定基线 (20 组固定姿态):**
- FR: 14.78 mm / 0.680° — PASS (≤15mm / ≤1°)
- RE: 11.04 mm / 0.816° — PASS (≤15mm / ≤1°)

> 15mm / 1° 为 Gazebo 工程基线, 不代表真实 D455 精度上限.
> 实机精度需在三台 D455 安装后独立验证.

### 其它模块

- TSDF 三维重建 (Open3D) — 已实现
- Ceres Bundle Adjustment — 历史/研究组件, 不用于稳定版外参求解
- 喷涂路径生成 / CR5 实机运动 — 待开发

### 实机状态

- 三台真实 D455 联合标定: **待进行**
- 实机标定入口: `run_three_camera_calibration.py`

## 2. 系统结构

### CORE（核心研发包）

| 包 | 职责 |
|----|------|
| `cr5_moveit` | MoveIt 运动规划配置 |
| `cr5_spray_sim` | Gazebo 仿真环境、标定目标模型、诊断工具 |
| `cr5_spray_perception` | 同步采集、PnP、Ceres BA、TSDF 重建 |

### UPSTREAM / VENDOR（上游/Vendor 包）

| 包 | 职责 |
|----|------|
| `dobot_bringup` | CR5 实机驱动 + DOBOT API |
| `dobot_description` | 机器人 URDF 描述 |
| `realsense_gazebo_description` | D455 相机 URDF 描述 |
| `realsense_gazebo_plugin` | Gazebo 相机仿真插件 |

## 3. 快速开始

```bash
# 克隆并构建
cd ~/cr5_ros1_ws
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build.sh
source devel/setup.bash

# 启动仿真 (GUI 模式)
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --gui --object=calibration_target --profile=quality

# 第二终端：加载会话环境
source /tmp/cr5_spray_simulation.env

# 检查相机信号
rostopic list | grep camera

# 启动标定采集服务
roslaunch cr5_spray_perception calibration_pipeline.launch
```

## 4. 三相机

| 相机 | Color Topic | Depth Topic | CameraInfo |
|------|------------|-------------|------------|
| cam_front_left | `/cam_front_left/camera/color/image_raw` | `/cam_front_left/camera/depth/image_raw` | `/cam_front_left/camera/color/camera_info` |
| cam_front_right | `/cam_front_right/camera/color/image_raw` | `/cam_front_right/camera/depth/image_raw` | `/cam_front_right/camera/color/camera_info` |
| cam_rear | `/cam_rear/camera/color/image_raw` | `/cam_rear/camera/depth/image_raw` | `/cam_rear/camera/color/camera_info` |

所有相机内参: fx=462.1, fy=462.1, cx=320, cy=240, 640×480, D=[]（无畸变）

## 5. 标定目标

几何权威来源: `src/cr5_spray_sim/config/calibration/calibration_target.yaml` (schema v2)

| 面 | 图案 | 字典 | ID | 尺寸 (m) | 纹理 |
|----|------|------|-----|---------|------|
| Front | ChArUco 8×6 | DICT_5X5_1000 | 100–123 | 0.24×0.18 | charuco_front.png |
| Left | AprilTag 2×2 | DICT_APRILTAG_36h11 | 4–7 | 0.22×0.18 | apriltag_left.png |
| Right | ArUco 2×2 | DICT_4X4_50 | 10–13 | 0.22×0.18 | aruco_right.png |
| Top | AprilTag 1×1 | DICT_APRILTAG_36h11 | 8 | 0.16×0.16 | apriltag_top.png |
| Back | ChArUco 8×6 | DICT_5X5_1000 | 300–323 | 0.24×0.18 | charuco_back.png |

主体: 0.34×0.28×0.24 m，绳索悬挂

## 6. 标定流水线

完整流程参见 [三相机人工标定操作手册](docs/三相机人工标定操作手册.md)。

```
START → CAMERA CHECK → TIME SYNC → TARGET POSE → DETECT
→ CAPTURE → INSPECT → REPEAT (≥5 poses) → PnP → BA
→ Gazebo TRUTH VALIDATE → schema v2 EXPORT → RELOAD VERIFY
```

**工程平台验收（平台可用）：**
- 同步: inter-camera skew ≤ 5ms, 5/5 PASS
- 检测: 三台相机均能检测到标定图案
- 工具: PnP / BA 程序可运行并输出结果

**标定精度参考（用户人工判断，非平台验收条件）：**
- PnP: 建议 ≥ 8 points/camera, RMSE ≤ 1.5px
- BA: 建议 overall RMSE ≤ 1.0px, per-camera ≤ 1.5px
- Truth: 建议 translation ≤ 10mm, rotation ≤ 0.5°

## 7. 数据边界

```
源码:    ~/cr5_ros1_ws/            (Git 仓库)
数据:    ${CR5_DATA_ROOT}          (默认 ~/cr5_data)
Python:  ${CR5_VENV_DIR}           (默认 ~/.venvs/cr5-spray)
构建:    build/ devel/             (catkin_make 生成)
```

**不要将 build/devel/data/venv 视为项目源码。**

## 8. 文档导航

- [文档中心](docs/README.md)
- [项目架构与代码说明](docs/项目架构与代码说明.md)
- [仿真环境安装与运行](docs/仿真环境安装与运行.md)
- [三相机人工标定操作手册](docs/三相机人工标定操作手册.md)
- [三相机标定与三维重建（算法设计）](docs/三相机标定与三维重建.md)
- [实机接入与安全操作](docs/实机接入与安全操作.md)
- [工程维护与发布](docs/工程维护与发布.md)
- [项目状态与验收](docs/项目状态与验收.md)

## 9. 维护脚本

```bash
bash scripts/build.sh                    # 构建
bash scripts/clean_workspace.sh          # 清理 (默认 dry-run)
bash scripts/clean_workspace.sh --yes    # 执行清理
bash scripts/project_tree.sh             # 源码树
bash scripts/workspace_stats.sh          # 统计概览
bash scripts/check_repository_contract.sh # 仓库契约检查
```

## 10. 安全

- **仿真 ≠ 实机授权。** 未经人工审查禁止 CR5 实机运动。
- **外参未经验证不得用于安全关键决策。**
- 实机操作前必须阅读 [实机接入与安全操作](docs/实机接入与安全操作.md)。
- 本仓库 `config/local.yaml` 包含本机串号/网络配置，**绝对不要提交**。

## 11. 分支策略

| 分支 | 说明 |
|------|------|
| `main` | 稳定工程基线（仅 fast-forward 合并） |
| `feature/*` | 功能开发分支 |
| `fix/*` | 修复分支 |

禁止 force push main。

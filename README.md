# DOBOT CR5 三相机标定与可见表面三维重建平台

基于 ROS Noetic + Gazebo Classic 11 的 DOBOT CR5 多相机系统。集成三台固定 RGB-D 相机、
多面标定目标、成对相机相对标定、TSDF 可见表面三维重建。

---

## 1. 当前能力

### 三相机标定

- 三台固定 RGB-D 相机（前左/前右/后），同步采集（skew ≤ 5ms, 640×480@10Hz）
- 五面标定目标（前/后 ChArUco、左 AprilTag、右 ArUco、顶 AprilTag）
- 单相机 PnP → 成对相机相对求解 → RANSAC 共识 → SE(3) 平均
- 前左相机固定为 rig 基准，输出 FL→FR 和 FL→RE
- Stable V1 外参，Gazebo 工程基线验证

**Gazebo 稳定基线:**
- FR: 14.78 mm / 0.680° — PASS（≤15mm / ≤1°）
- RE: 11.04 mm / 0.816° — PASS（≤15mm / ≤1°）

> 15mm / 1° 为 Gazebo 工程基线，不代表真实 D455 精度上限。
> 实机精度需在三台 D455 安装后独立验证。

### 可见表面三维重建

当前正式重建能力：

- 三台 RGB-D 相机目标区域融合
- Stable V1 外参
- target-only depth mask
- TSDF 可见表面重建
- 保守网格碎片清理
- 质量评价（需显式传入 `--evaluate --visible-gt`）
- 一键运行，可重复输出

**正式参数：**
- voxel_length = 0.005 m
- sdf_trunc = 0.020 m
- depth-edge filter: disabled
- multiview consistency: disabled
- pose refinement: disabled
- bottom surface: UNKNOWN
- partial visible surface（不生成 watertight 模型，不补全不可见表面）

**当前 Gazebo 基线指标：**
- Accuracy median ≈ 3.68 mm
- Accuracy P95 ≈ 15.58 mm
- Production Gate: median ≤ 5mm, P95 ≤ 16mm
- 相同输入重复运行结果一致

**明确边界：**
- 当前指标来自 Gazebo 固定三相机场景
- 当前对象为标定目标
- 尚未代表真实 D455 精度
- 尚未代表任意工件泛化能力
- 尚未进入喷涂路径和机械臂运动阶段

### 实机状态

- 三台真实 D455 联合标定：待下一阶段
- 实机标定入口：`run_three_camera_calibration.py`

---

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

---

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

---

## 4. 正式运行入口

### 可见表面重建

```bash
rosrun cr5_spray_perception run_visible_surface_reconstruction.py \
  --dataset <dataset_path> \
  --group-id 0 \
  --rig <stable_rig.yaml> \
  --config $(rospack find cr5_spray_perception)/config/reconstruction/visible_surface_production_v1.yaml \
  --output <output_path>
```

**评价命令：**

仅当显式传入以下参数时才执行 Gazebo GT 评价：

```bash
--evaluate --visible-gt <visible_gt.ply>
```

- 传入 `--evaluate`：执行评价，生成完整的 accuracy/completeness/chamfer/coverage 指标
- 未传 `--evaluate`：正常生成重建结果，accuracy 状态为 `NOT_AVAILABLE`，不伪造生产精度

### 三相机标定

```bash
rosrun cr5_spray_perception run_three_camera_calibration.py
```

标定操作详见 [三相机人工标定操作手册](docs/三相机人工标定操作手册.md)。

---

## 5. 输出说明

重建完成后在 `--output` 目录下生成：

| 文件 | 说明 |
|------|------|
| `visible_surface_mesh_final.ply` | 最终重建 mesh |
| `visible_surface_pointcloud.ply` | 重建点云 |
| `reconstruction_quality_report.json` | 重建质量报告（含 acceptance Gate） |
| `unknown_surface_regions.json` | 未观测区域记录（bottom=UNKNOWN） |
| `effective_config.yaml` | 实际使用的完整配置副本 |
| `component_report.json` | mesh cleanup 分量报告 |

---

## 6. 三条长期分支

| 分支 | 说明 |
|------|------|
| `main` | 当前完整工程封板基线，下一阶段临时分支的唯一创建起点 |
| `stable/three-camera-calibration-v1` | 标定阶段冻结分支（只读） |
| `stable/visible-surface-reconstruction-v1` | 重建阶段冻结分支（只读） |

**规则：**

- **禁止**在三条长期分支上直接开发
- **禁止** force push main 和 stable/*
- **禁止**删除三条长期分支
- `feature/*`、`fix/*`、`chore/*` 为临时分支，必须从 main 创建，通过自动化 Gate 后合并，合并后立即删除
- 下一阶段只能从 main 创建新的临时 `feature/*` 分支

---

## 7. 三相机

| 相机 | Color Topic | Depth Topic | CameraInfo |
|------|------------|-------------|------------|
| cam_front_left | `/cam_front_left/camera/color/image_raw` | `/cam_front_left/camera/depth/image_raw` | `/cam_front_left/camera/color/camera_info` |
| cam_front_right | `/cam_front_right/camera/color/image_raw` | `/cam_front_right/camera/depth/image_raw` | `/cam_front_right/camera/color/camera_info` |
| cam_rear | `/cam_rear/camera/color/image_raw` | `/cam_rear/camera/depth/image_raw` | `/cam_rear/camera/color/camera_info` |

所有相机内参: fx=462.1, fy=462.1, cx=320, cy=240, 640×480, D=[]（无畸变）

---

## 8. 标定目标

几何权威来源: `src/cr5_spray_sim/config/calibration/calibration_target.yaml` (schema v2)

| 面 | 图案 | 字典 | ID | 尺寸 (m) | 纹理 |
|----|------|------|-----|---------|------|
| Front | ChArUco 8×6 | DICT_5X5_1000 | 100–123 | 0.24×0.18 | charuco_front.png |
| Left | AprilTag 2×2 | DICT_APRILTAG_36h11 | 4–7 | 0.22×0.18 | apriltag_left.png |
| Right | ArUco 2×2 | DICT_4X4_50 | 10–13 | 0.22×0.18 | aruco_right.png |
| Top | AprilTag 1×1 | DICT_APRILTAG_36h11 | 8 | 0.16×0.16 | apriltag_top.png |
| Back | ChArUco 8×6 | DICT_5X5_1000 | 300–323 | 0.24×0.18 | charuco_back.png |

主体: 0.34×0.28×0.24 m，绳索悬挂

---

## 9. 文档导航

- [文档中心](docs/README.md)
- [项目架构与代码说明](docs/项目架构与代码说明.md)
- [仿真环境安装与运行](docs/仿真环境安装与运行.md)
- [三相机人工标定操作手册](docs/三相机人工标定操作手册.md)
- [三相机标定算法说明](docs/三相机标定算法说明.md)
- [实机接入与安全操作](docs/实机接入与安全操作.md)
- [工程维护与发布](docs/工程维护与发布.md)
- [项目状态与验收](docs/项目状态与验收.md)

---

## 10. 数据边界

```
源码:    ~/cr5_ros1_ws/            (Git 仓库)
数据:    ${CR5_DATA_ROOT}          (默认 ~/cr5_data)
Python:  ${CR5_VENV_DIR}           (默认 ~/.venvs/cr5-spray)
构建:    build/ devel/             (catkin_make 生成)
```

不要将 build/devel/data/venv 视为项目源码。

---

## 11. 维护脚本

```bash
bash scripts/build.sh                    # 构建
bash scripts/clean_workspace.sh          # 清理 (默认 dry-run)
bash scripts/clean_workspace.sh --yes    # 执行清理
bash scripts/project_tree.sh             # 源码树
bash scripts/workspace_stats.sh          # 统计概览
bash scripts/check_repository_contract.sh # 仓库契约检查
```

---

## 12. 安全

- **仿真 ≠ 实机授权。** 未经人工审查禁止 CR5 实机运动。
- **外参未经验证不得用于安全关键决策。**
- 实机操作前必须阅读 [实机接入与安全操作](docs/实机接入与安全操作.md)。
- 本仓库 `config/local.yaml` 包含本机串号/网络配置，**绝对不要提交**。

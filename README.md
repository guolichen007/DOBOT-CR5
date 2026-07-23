# DOBOT CR5 多相机标定与三维重建

基于 ROS Noetic + Gazebo Classic 11 的 DOBOT CR5 三相机标定系统。
集成三台固定 D455-like RGB-D 相机，用于多视角标定目标采集与三维重建。

## 快速开始

```bash
# 构建
cd ~/cr5_ros1_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

# 启动仿真
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --gui --object=calibration_target --profile=quality

# 第二终端
source /tmp/cr5_spray_simulation.env
rostopic list
```

## ROS 包 (7 个)

| 包 | 职责 |
|----|------|
| `dobot_bringup` | CR5 实机驱动 |
| `dobot_description` | 机器人 URDF 描述 |
| `cr5_moveit` | MoveIt 运动规划 |
| `cr5_spray_sim` | Gazebo 仿真环境 |
| `cr5_spray_perception` | 标定感知处理 |
| `realsense_gazebo_description` | D455 相机描述 |
| `realsense_gazebo_plugin` | Gazebo 相机插件 |

## 三相机

| 相机 | Color | Depth |
|------|-------|-------|
| cam_front_left | `/cam_front_left/color/image_raw` | `/cam_front_left/depth/image_rect_raw` |
| cam_front_right | `/cam_front_right/color/image_raw` | `/cam_front_right/depth/image_rect_raw` |
| cam_rear | `/cam_rear/color/image_raw` | `/cam_rear/depth/image_rect_raw` |

## 标定目标

| 面 | 图案 | ID | 尺寸 |
|----|------|-----|------|
| Front | ChArUco 8×6 | 100–123 | 0.24×0.18 m |
| Left | ChArUco 6×5 | 200–214 | 0.16×0.12 m |
| Right | AprilTag 2×2 | 4–7 | 0.22×0.18 m |
| Top | AprilTag 1×1 | 8 | 0.16×0.16 m |
| Back | ChArUco 8×6 | 300–323 | 0.24×0.18 m |

主体: 0.34×0.28×0.24 m

## 当前状态

**已完成:**
- Gazebo 仿真环境（CR5、三台固定相机、标定目标）
- 三相机 RGB-D 数据链路（3/3 color + 3/3 depth）
- ChArUco / AprilTag 检测与 PnP 外参骨架
- 确定性启动流程 + 跨终端会话环境
- 资产契约验证（SHA-256 + 几何 + 材质）

**尚未完成:**
- 三相机联合外参优化
- Bundle Adjustment
- RGB-D 点云融合 / TSDF 三维重建
- 喷涂路径生成
- CR5 实机运动

## 分支

| 分支 | 说明 |
|------|------|
| `main` | 稳定工程基线 |
| `develop/multi-camera-calibration` | 三相机标定与重建开发 |

## 文档

- [项目架构与代码说明](docs/项目架构与代码说明.md)
- [仿真环境安装与运行](docs/仿真环境安装与运行.md)
- [三相机标定与三维重建](docs/三相机标定与三维重建.md)
- [实机接入与安全操作](docs/实机接入与安全操作.md)

## 安全

- 仿真中不连接 CR5 实机
- 相机外参未经验证不用于安全关键决策
- 实机操作前参考 `docs/实机接入与安全操作.md`

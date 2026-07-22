# DOBOT CR5 三相机喷涂仿真与标定系统

基于 ROS Noetic + Gazebo Classic 11 的 DOBOT CR5 机器人喷涂仿真环境，
集成三台固定 RGB-D 相机 (Intel RealSense D455)。

## 快速开始

```bash
cd ~/cr5_ros1_ws
catkin_make && source devel/setup.bash

# 启动仿真 (GUI 模式)
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --gui --object=calibration_target --profile=quality

# 第二终端
source /tmp/cr5_spray_simulation.env
```

## 验证

```bash
rosrun cr5_spray_sim validate_calibration_target_geometry.py
bash src/cr5_spray_sim/scripts/validate_calibration_texture.sh --gui
rosrun cr5_spray_sim validate_calibration_visibility.py --output artifacts/calibration/visibility
```

## 标定目标

| 面 | 图案 | ID | 尺寸 |
|----|------|-----|------|
| Front | ChArUco 8×6 | 100–123 | 0.24×0.18 m |
| Left | ChArUco 6×5 | 200–214 | 0.16×0.12 m |
| Right | AprilTag 2×2 | 4–7 | 0.22×0.18 m |
| Top | AprilTag | 8 | 0.16×0.16 m |
| Back | ChArUco 8×6 | 300–323 | 0.24×0.18 m |

主体: 0.34×0.28×0.24 m | 面板间隙: 0.001 m

## 当前状态

**已完成:** 仿真环境、标定目标模型、三相机数据链路、检测与 PnP 骨架

**未完成:** 生产级三相机外参、Bundle Adjustment、点云融合/TSDF、喷涂路径、实机运动

## 分支

- `main` — 稳定工程基线
- `develop/multi-camera-calibration` — 三相机外参与三维重建开发

## 文档

- `docs/SYSTEM_ARCHITECTURE.md`
- `docs/SIMULATION_OPERATION.md`
- `docs/BRANCH_POLICY.md`
- `docs/REPOSITORY_CLEANUP_REPORT.md`
- `docs/NEXT_MULTI_CAMERA_CALIBRATION.md`

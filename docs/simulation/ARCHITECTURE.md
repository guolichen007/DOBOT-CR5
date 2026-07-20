# CR5 喷涂仿真架构

## 系统概述

```
cr5_spray_sim/       — 仿真场景、CR5模型、控制器、启动文件
cr5_spray_perception/ — 数据采集、三维重建、质量评估
cr5_spray_planning/   — 喷涂路径生成、覆盖率评估
realsense_gazebo_plugin/ — Gazebo RGB-D 相机插件
realsense_gazebo_description/ — 相机 URDF/xacro 模型
```

## 数据流

```
Gazebo 仿真场景
  ├── CR5 (ros_control + PositionJointTrajectoryController)
  ├── 门式框架 (static collision model)
  ├── 吊挂工件 (yaw_joint + object_frame)
  └── RGB-D 相机阵列 (RealSensePlugin → ROS topics)
        │
        ▼
Capture Manager (多相机同步采集)
  ├── color.png + depth.npy + camera_info.yaml
  └── T_world_camera.yaml (TF真值)
        │
        ▼
TSDF 重建 (Open3D)
  ├── 位姿模式: ground_truth / noisy_pose / refined_pose
  ├── 提取 mesh (PLY)
  └── 评估指标 (accuracy/completeness/Chamfer)
        │
        ▼
喷涂路径生成
  ├── 喷涂面选择 (法向阈值)
  ├── planar_raster / mesh_slice_raster
  └── spray_nozzle_frame 姿态 (IK → MoveIt → Gazebo)
        │
        ▼
覆盖率评估
  ├── 虚拟喷枪模型 (圆锥/高斯喷幅)
  ├── coverage/under-spray/over-spray ratio
  └── 热力图 mesh (PLY)
```

## 关键约定

1. **TF 约定**: `T_A_B` 将 B 坐标中的点变换到 A
2. **喷嘴约定**: `spray_nozzle_frame` 的 +Z 轴从喷嘴指向被喷表面
3. **相机命名**: 每台相机 unique model_name → unique ROS namespace → unique TF frame
4. **参数外部化**: 所有参数在 YAML/xacro/launch 中，算法不硬编码

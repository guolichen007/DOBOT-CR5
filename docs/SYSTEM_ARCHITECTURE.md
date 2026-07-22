# 系统架构

## 包结构

```
cr5_ros1_ws/src/
├── cr5_spray_sim/          # 仿真与标定主包
│   ├── launch/             # roslaunch 文件
│   ├── scripts/            # Python/shell 脚本
│   ├── config/             # YAML 配置
│   ├── models/             # Gazebo 模型
│   ├── urdf/               # 机器人 URDF/Xacro
│   ├── worlds/             # Gazebo 世界文件
│   └── src/cr5_spray_sim/  # Python 包
├── cr5_spray_perception/   # Open3D 运行时
├── cr5_spray_planning/     # MoveIt 规划 (预留)
└── dobot_bringup/          # DOBOT 驱动
```

## TF 树

```
world → dummy_link → base_link → Link1..Link6 → spray_nozzle_frame
world → object_frame → calibration_target_frame
  ├── calibration_target_front_frame
  ├── calibration_target_left_frame
  ├── calibration_target_right_frame
  ├── calibration_target_top_frame
  └── calibration_target_back_frame
```

## 三相机

| 相机 | Color Topic | Depth Topic | CameraInfo |
|------|-------------|-------------|------------|
| cam_front_left | /cam_front_left/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_front_right | /cam_front_right/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_rear | /cam_rear/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |

## 启动流程

1. A0: 审计旧仿真进程
2. A1/A2: 端口选择 + roscore 启动
3. B: roslaunch spray_simulation.launch (paused=true)
4. C: 模型 spawn + 绝对几何验证
5. D: unpause → controller 启动 → 运行时检查
6. E: 就绪 — 相机采集、标定、喷涂

## 会话环境

启动后写入 `/tmp/cr5_spray_simulation.env`，第二终端 source 后可直接使用。

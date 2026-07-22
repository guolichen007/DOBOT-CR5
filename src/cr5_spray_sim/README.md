# cr5_spray_sim — CR5 喷涂仿真与标定包

## 职责

- Gazebo 仿真环境管理
- 三相机 RGB-D 数据链路
- 标定目标模型与 TF
- ChArUco / AprilTag 检测
- 相机外参估计骨架
- 喷涂仿真 (可选)

## 入口

`bash scripts/run_simulation.sh --gui --object=calibration_target --profile=quality`

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| object_type | calibration_target | 工作对象类型 |
| camera_profile | vm | 相机分辨率 (vm/quality) |
| gui | false | 显示 Gazebo GUI |
| headless | false | 无头模式 |
| paused | true | paused 启动 |
| enable_spray_sim | false | 启用喷涂 |

## 相机话题

| 相机 | Color | Depth | Info |
|------|-------|-------|------|
| cam_front_left | /cam_front_left/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_front_right | /cam_front_right/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_rear | /cam_rear/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |

## 标定资产

- YAML: `config/calibration/calibration_target.yaml`
- SDF: `models/calibration_target/model.sdf`
- Textures: `models/calibration_target/materials/textures/`

## 不支持范围

- 生产级三相机外参
- Bundle Adjustment
- 点云融合/TSDF
- 自动喷涂路径
- CR5 实机运动

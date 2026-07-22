# cr5_spray_sim — CR5 喷涂仿真与标定包

## 包职责

- Gazebo 仿真环境管理
- 三相机 RGB-D 数据链路
- 标定目标模型与 TF 发布
- ChArUco / AprilTag 检测
- 相机外参估计骨架
- 喷涂仿真 (可选)

## 启动入口

```bash
bash scripts/run_simulation.sh --gui --object=calibration_target --profile=quality
```

## Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| object_type | calibration_target | 工作对象类型 |
| camera_profile | vm | 相机分辨率 (vm=424×240, quality=1280×720) |
| gui | false | 显示 Gazebo GUI |
| headless | false | 无头模式 |
| paused | true | 以暂停模式启动 |
| enable_spray_sim | false | 启用喷涂控制 (标定模式默认关闭) |

## 相机话题

| 相机 | Color | Depth | CameraInfo |
|------|-------|-------|------------|
| cam_front_left | /cam_front_left/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_front_right | /cam_front_right/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_rear | /cam_rear/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |

## 标定资产路径

- YAML: `config/calibration/calibration_target.yaml`
- SDF: `models/calibration_target/model.sdf`
- DAE: `models/calibration_target/meshes/panel_unit.dae`
- 材质: `models/calibration_target/materials/scripts/calibration_target.material`
- 纹理: `models/calibration_target/materials/textures/`

## 验证命令

```bash
# 几何一致性
rosrun cr5_spray_sim validate_calibration_target_geometry.py

# 贴图独立检查
bash scripts/validate_calibration_texture.sh --gui

# 可见性检测
rosrun cr5_spray_sim validate_calibration_visibility.py --output artifacts/calibration/visibility

# 外参估计
rosrun cr5_spray_sim estimate_camera_extrinsics.py --output artifacts/calibration/extrinsics
```

## 当前不支持

- 生产级三相机外参
- Bundle Adjustment / 三维重建
- 自动喷涂路径规划
- CR5 实机运动控制

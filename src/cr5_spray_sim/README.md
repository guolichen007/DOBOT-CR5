# cr5_spray_sim — CR5 仿真与标定主包

## 职责

- Gazebo 仿真环境管理
- 三相机 RGB-D 数据链路
- 标定目标模型与 TF 发布
- 场景几何验证

## 启动

```bash
bash scripts/run_simulation.sh --gui --object=calibration_target --profile=quality
```

## 相机话题

| 相机 | Color | Depth |
|------|-------|-------|
| cam_front_left | `/cam_front_left/camera/color/image_raw` | `/cam_front_left/camera/depth/image_raw` |
| cam_front_right | `/cam_front_right/camera/color/image_raw` | `/cam_front_right/camera/depth/image_raw` |
| cam_rear | `/cam_rear/camera/color/image_raw` | `/cam_rear/camera/depth/image_raw` |

## 配置

- 场景: `config/simulation_scene.yaml`
- 相机模板: `urdf/fixed_rgbd_camera.urdf.xacro`
- 标定模型: `models/calibration_target/`

## 验证

```bash
rosrun cr5_spray_sim validate_calibration_target_geometry.py
bash scripts/validate_calibration_texture.sh
```

## 文档

- [仿真环境安装与运行](../../docs/仿真环境安装与运行.md)
- [三相机标定与三维重建](../../docs/三相机标定与三维重建.md)
- [项目架构与代码说明](../../docs/项目架构与代码说明.md)

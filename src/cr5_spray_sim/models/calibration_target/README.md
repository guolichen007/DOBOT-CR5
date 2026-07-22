# Calibration Target — 标定目标模型

## 坐标系

- +X: Front ChArUco 面板外法向
- +Y: Left 面板外法向
- +Z: 向上

## 几何

- 主体: 0.34 × 0.28 × 0.24 m
- 顶部偏置块: 0.06 × 0.05 × 0.04 m

## 面板

见 `docs/CALIBRATION_TARGET.md`

## 文件结构

```
calibration_target/
├── model.config        # Gazebo 模型元数据
├── model.sdf           # 自包含 Gazebo 模型
├── meshes/
│   └── panel_unit.dae  # 1×1m UV 平面 (双面)
└── materials/
    ├── scripts/
    │   └── calibration_target.material  # OGRE 材质定义
    └── textures/
        ├── charuco_front.png            # ChArUco 8×6, IDs 100-123
        ├── charuco_left.png             # ChArUco 6×5, IDs 200-214
        ├── charuco_back.png             # ChArUco 8×6, IDs 300-323
        ├── apriltag_right.png           # AprilTag 2×2, IDs 4-7
        └── apriltag_top.png             # AprilTag, ID 8
```

## 修改流程

1. 更新 model.sdf
2. 更新 config/calibration/calibration_target.yaml
3. 运行 validate_calibration_target_geometry.py
4. 运行 validate_calibration_texture.sh
5. 在 Gazebo GUI 中目视确认

## 禁止

- 非等比缩放 PNG 纹理
- 修改 Marker/Tag 真实物理尺寸
- 在多个位置保留 PNG 副本

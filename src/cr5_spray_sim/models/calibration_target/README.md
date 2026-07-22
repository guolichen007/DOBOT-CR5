# 标定目标模型

## 坐标系

- +X: Front 面板外法向 (ChArUco)
- +Y: Left 面板外法向
- +Z: 向上

## 几何

- 主体: 0.34 × 0.28 × 0.24 m
- 面板间隙: 0.001 m
- 顶部偏置块: 0.06 × 0.05 × 0.04 m，位置 (0.13, 0.11, 0.14)
- 吊索: Y = ±0.115 m

## 面板

| 面 | 图案 | ID | 尺寸 |
|----|------|-----|------|
| Front | ChArUco 8×6 | 100–123 | 0.24×0.18 m |
| Left | ChArUco 6×5 | 200–214 | 0.16×0.12 m |
| Right | AprilTag 2×2 | 4–7 | 0.22×0.18 m |
| Top | AprilTag | 8 | 0.16×0.16 m |
| Back | ChArUco 8×6 | 300–323 | 0.24×0.18 m |

## 文件结构

```
calibration_target/
├── model.config        # Gazebo 模型元数据
├── model.sdf           # 自包含 SDF 模型 (1.7)
├── meshes/
│   └── panel_unit.dae  # 1×1m UV 平面 (Z_UP, 双面)
└── materials/
    ├── scripts/
    │   └── calibration_target.material  # OGRE 材质 (5 个材质)
    └── textures/
        ├── charuco_front.png     # ChArUco 8×6, IDs 100-123
        ├── charuco_left.png      # ChArUco 6×5, IDs 200-214
        ├── charuco_back.png      # ChArUco 8×6, IDs 300-323
        ├── apriltag_right.png    # AprilTag 2×2, IDs 4-7
        └── apriltag_top.png      # AprilTag, ID 8
```

## 修改流程

1. 更新 `model.sdf`
2. 更新 `config/calibration/calibration_target.yaml`
3. 运行 `validate_calibration_target_geometry.py` 验证一致性
4. 运行 `validate_calibration_texture.sh` 验证贴图
5. Gazebo GUI 中目视确认

## 禁止

- 非等比缩放 PNG 纹理
- 修改 Marker/Tag 真实物理尺寸
- 在多个位置保留 PNG 副本

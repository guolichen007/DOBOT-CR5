# cr5_spray_perception — 标定感知处理

## 职责

- 三相机标定验证（可见性、外参）
- 多相机同步采集入口
- TSDF 三维重建（下一阶段）

## 入口

```bash
source /tmp/cr5_spray_simulation.env

# 可见性检测
rosrun cr5_spray_perception validate_calibration_visibility.py \
  --output artifacts/calibration/visibility

# 外参估计
rosrun cr5_spray_perception estimate_camera_extrinsics.py \
  --truth-source gazebo --output artifacts/calibration/extrinsics
```

## 下一阶段

- `capture_manager.py` — 三相机同步采集
- `reconstruct_tsdf.py` — TSDF 重建 (experimental)
- `evaluate_reconstruction.py` — 重建评价 (experimental)

## 文档

- [三相机标定与三维重建](../../docs/三相机标定与三维重建.md)
- [项目架构与代码说明](../../docs/项目架构与代码说明.md)

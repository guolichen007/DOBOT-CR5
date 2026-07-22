# 下一阶段: 三相机标定与三维重建

## 入口

```bash
source /tmp/cr5_spray_simulation.env
rosrun cr5_spray_sim estimate_camera_extrinsics.py --output artifacts/calibration/extrinsics
```

## 待完成

1. 三相机联合外参优化 (PnP + 多视图约束)
2. 多视角 Bundle Adjustment
3. RGB-D 点云融合
4. TSDF 三维重建
5. 相机位姿自标定

## 约束

- 不调整相机物理安装位置
- 不缩放标定板真实尺寸
- 不改变 Marker/Tag ID 和图案

## 当前基线

- 目标模型几何准确
- 五面贴图正常渲染
- 三台相机 color/depth 话题正常
- ChArUco/AprilTag 检测可用
- PnP 求解骨架可用

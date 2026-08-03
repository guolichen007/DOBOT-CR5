"""
CR5 Spray Perception — Reconstruction package.

三相机 RGB-D 三维重建:
  - contracts:    外参 schema 校验
  - transforms:   深度反投影与点云变换
  - extrinsics:   Stable V1 → calibrated_rig 桥接
  - rgbd_io:      RGB-D 数据加载与契约验证
  - pointcloud_fusion: 三相机点云融合与重叠度量

生产链路禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

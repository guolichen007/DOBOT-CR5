# 第三方代码声明 (Third Party Notices)

## realsense_gazebo_plugin

- **来源**: Intel RealSense Gazebo plugin (forked from pal-robotics-forks/realsense)
- **许可证**: Apache 2.0
- **原始仓库**: https://github.com/pal-robotics-forks/realsense
- **本地修改**:
  - 修复 package.xml 缺失依赖 (image_transport, sensor_msgs, cv_bridge)
  - 适配 Gazebo 11 API
  - 保留多相机 unique name/namespace/frame 能力

## realsense_gazebo_description

- **来源**: Intel RealSense Gazebo description (forked from pal-robotics-forks/realsense)
- **许可证**: Apache 2.0
- **原始仓库**: https://github.com/pal-robotics-forks/realsense
- **本地修改**:
  - 新增 `_d455_like.urdf.xacro` — 参数化 RGB-D 模拟器模型
  - 新增 `_d455_like.gazebo.xacro` — Gazebo 传感器和噪声参数
  - 新增 `d455_like_test.urdf.xacro` — 单相机测试模型
  - 新增 `d455_like_smoke.launch` — 烟雾测试启动文件
  - 新增 `d455_like_smoke_test.py` — Python 烟雾测试脚本
  - 新增 `config/d455_like_default.yaml` — 默认参数配置

## 重要声明

D455-like RGB-D Simulator 不是真实 Intel RealSense D455 物理模型。
它是基于通用 RGB-D Gazebo 插件的参数化近似模拟器。
所有内参、分辨率、FOV、噪声均为可配置参数，不代表真实设备规格。

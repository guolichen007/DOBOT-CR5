# RealSense ROS1 独立工作空间设置

## 1. 概述

当前使用 `realsense-ros` ROS1 **2.3.2** 版本。

基线 commit：

```
f400d682beee6c216052a419f419e95b797255ad
```

旧源码位置：

```
~/cr5_ws/src/realsense-ros
```

## 2. 实机已验证数据

### 2.1 RealSense ROS Wrapper

```
realsense-ros 2.3.2
commit f400d682beee6c216052a419f419e95b797255ad
```

### 2.2 Librealsense

```
Built with LibRealSense v2.54.2
Running with LibRealSense v2.54.2
运行时库: /usr/local/lib/librealsense2.so.2.54
CMake 配置: /usr/local/lib/cmake/realsense2/realsense2Config.cmake
```

### 2.3 D455 硬件

```
型号: Intel RealSense D455
序列号: 311322304396
固件: 5.15.1
USB: 3.2
```

### 2.4 流配置

```
Depth: 848 × 480 @ 30 FPS, Z16
Color: 1280 × 720 @ 30 FPS, RGB8
```

### 2.5 实际频率

```
/camera/color/image_raw: 约 28.6 Hz
/camera/aligned_depth_to_color/image_raw: 约 27.7～29 Hz
```

### 2.6 成功发布的话题

```
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/depth/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
/camera/extrinsics/depth_to_color
/tf
/tf_static
```

## 3. 独立工作空间

采用软链接方式创建独立 RealSense ROS1 工作空间：

```
~/realsense_ros1_ws
```

包含两个软链接：

```
~/realsense_ros1_ws/src/realsense2_camera
  → ~/cr5_ws/src/realsense-ros/realsense2_camera

~/realsense_ros1_ws/src/realsense2_description
  → ~/cr5_ws/src/realsense-ros/realsense2_description
```

## 4. 设置脚本

运行以下命令创建独立工作空间：

```bash
cd ~/cr5_ros1_ws
bash scripts/laptop/setup_realsense_ros1.sh
```

脚本会：

1. 检查旧 RealSense 仓库的 HEAD 和版本
2. 创建软链接
3. 检查依赖（跳过 librealsense2）
4. 编译工作空间（使用实机验证成功的参数）
5. 验证运行时库链接
6. 验证包可用

## 5. 环境加载顺序（正确方式）

**重要**：以下是在实机验证成功的正确叠加方式。

新终端需要按顺序加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/realsense_ros1_ws/devel/setup.bash
source ~/cr5_ros1_ws/devel/setup.bash --extend
```

**不要使用**：

```bash
# ❌ 错误方式（已在实机验证无效）
source ~/cr5_ros1_ws/devel/local_setup.bash
```

`local_setup.bash` 不会将 `cr5_ros1_ws/src` 加入 `ROS_PACKAGE_PATH`。

## 6. 启动 D455

### 方式一：使用项目专用 launch 文件

```bash
roslaunch cr5_book_spray_demo d455_camera.launch
```

或指定序列号：

```bash
roslaunch cr5_book_spray_demo d455_camera.launch serial_no:=311322304396
```

### 方式二：使用上游 launch 文件

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### 方式三：使用脚本

```bash
bash scripts/laptop/test_d455_vision_only.sh camera
```

## 7. 检查话题

```bash
bash scripts/laptop/test_d455_vision_only.sh topics
```

预期话题：

```
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 8. 话题一致性检查

```bash
python3 scripts/laptop/check_d455_topics.py
```

检查项目：

- 话题存在性
- 分辨率一致性
- frame_id
- 时间同步

## 9. 启动书本识别

```bash
bash scripts/laptop/test_d455_vision_only.sh vision
```

**重要**：必须使用 `start_camera:=false`，避免同一 D455 被两个进程打开。

## 10. 环境诊断

```bash
bash scripts/laptop/test_d455_vision_only.sh environment
```

诊断内容：

- 环境变量
- 环境文件检查
- ROS 包验证
- ROS_PACKAGE_PATH

## 11. 常见问题

### 11.1 rospack 找不到包

**现象**：`[rospack] Error: package 'cr5_book_spray_demo' not found`

**原因**：未正确加载 CR5 工作空间环境，或使用了 `local_setup.bash`。

**解决**：

```bash
# 使用 --extend 叠加
source ~/cr5_ros1_ws/devel/setup.bash --extend

# 验证
rospack find cr5_book_spray_demo
rospack find realsense2_camera
```

### 11.2 USB 只有 480M

**现象**：`lsusb -t` 显示 D455 接口速度为 480M。

**原因**：USB 2.0 接口或线缆。

**解决**：

1. 使用 USB 3.x 接口
2. 使用 D455 原装线缆
3. 避免使用 USB Hub

### 11.3 相机被重复占用

**现象**：`Failed to allocate sensor` 或类似错误。

**原因**：多个进程尝试打开同一 D455。

**解决**：

```bash
# 检查占用进程
ps aux | grep -E "realsense|rs_camera"

# 杀死占用进程
pkill -f realsense2_camera_node
pkill -f rs_camera.launch
```

### 11.4 控制柜 ping 不通

**现象**：`ping 192.168.110.214` 超时。

**原因**：网络配置问题。

**解决**：

1. 检查网线连接
2. 确认笔记本 IP 与控制柜在同一网段
3. 检查防火墙设置

**注意**：控制柜网络不影响 D455 视觉测试。

### 11.5 aligned depth 不发布

**现象**：`/camera/aligned_depth_to_color/image_raw` 无数据。

**原因**：启动参数不正确。

**解决**：

确保启动时包含 `align_depth:=true`：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### 11.6 power_line_frequency 警告

**现象**：`Param '/camera/rgb_camera/power_line_frequency' has value 3 that is not in enum 0/1/2`

**说明**：

- 0：Disabled
- 1：50 Hz
- 2：60 Hz
- 3 在当前 ROS Wrapper 2.3.2 动态参数枚举中无效

**处理**：该警告未阻止相机启动，当前不视为阻断问题。如需设置，建议使用 `power_line_frequency:=1`（50 Hz 市电环境）。

## 12. 文件清单

```
docs/REALSENSE_ROS1_SETUP.md                    # 本文档
docs/REALSENSE_ROS1_2_3_2_LOCAL_DIFF_AUDIT.md   # RealSense 本地修改审计
scripts/laptop/setup_realsense_ros1.sh          # RealSense 工作空间设置脚本
src/cr5_book_spray_demo/launch/d455_camera.launch  # 项目专用 D455 启动文件
```

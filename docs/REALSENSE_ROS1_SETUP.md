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

## 2. 独立工作空间

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

## 3. 设置脚本

运行以下命令创建独立工作空间：

```bash
cd ~/cr5_ros1_ws
bash scripts/laptop/setup_realsense_ros1.sh
```

脚本会：

1. 检查旧 RealSense 仓库的 HEAD 和版本
2. 创建软链接
3. 检查依赖
4. 编译工作空间
5. 验证包可用

## 4. 环境加载顺序

新终端需要按顺序加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/realsense_ros1_ws/devel/setup.bash
source ~/cr5_ros1_ws/devel/local_setup.bash
```

**重要**：必须使用 `local_setup.bash`，避免覆盖 RealSense overlay。

## 5. 启动 D455

```bash
bash scripts/laptop/test_d455_vision_only.sh camera
```

或手动启动：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

## 6. 检查话题

```bash
bash scripts/laptop/test_d455_vision_only.sh topics
```

预期话题：

```
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 7. 启动书本识别

```bash
bash scripts/laptop/test_d455_vision_only.sh vision
```

**重要**：必须使用 `start_camera:=false`，避免同一 D455 被两个进程打开。

## 8. 常见问题

### 8.1 rospack 找不到包

**现象**：`[rospack] Error: package 'realsense2_camera' not found`

**原因**：未正确加载 RealSense 工作空间环境。

**解决**：

```bash
source ~/realsense_ros1_ws/devel/setup.bash
rospack find realsense2_camera
```

### 8.2 USB 只有 480M

**现象**：`lsusb -t` 显示 D455 接口速度为 480M。

**原因**：USB 2.0 接口或线缆。

**解决**：

1. 使用 USB 3.x 接口
2. 使用 D455 原装线缆
3. 避免使用 USB Hub

### 8.3 相机被重复占用

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

### 8.4 控制柜 ping 不通

**现象**：`ping 192.168.110.214` 超时。

**原因**：网络配置问题。

**解决**：

1. 检查网线连接
2. 确认笔记本 IP 与控制柜在同一网段
3. 检查防火墙设置

**注意**：控制柜网络不影响 D455 视觉测试。

### 8.5 aligned depth 不发布

**现象**：`/camera/aligned_depth_to_color/image_raw` 无数据。

**原因**：启动参数不正确。

**解决**：

确保启动时包含 `align_depth:=true`：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

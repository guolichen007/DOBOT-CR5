# RealSense ROS1 2.3.2 本地修改审计

## 1. 基本信息

- **HEAD**: `f400d682beee6c216052a419f419e95b797255ad`
- **Tag**: `2.3.2`
- **版本**: `realsense2_camera 2.3.2`, `realsense2_description 2.3.2`
- **Dirty 状态**: 是（有本地修改）

## 2. 修改文件审计

### 2.1 rs_camera.launch

**修改内容**：

```diff
- <arg name="infra_width" default="848"/>
+ <arg name="infra_width" default="640"/>
```

**分析**：将红外分辨率从 848x480 改为 640x480。

**结论**：❌ **不采用**

**原因**：
- 当前书本识别使用彩色图，不使用红外
- 与单台 D455 无关
- 使用上游默认值 848

### 2.2 rs_aligned_depth.launch

**修改内容**：

```diff
- <arg name="enable_pointcloud" default="false"/>
+ <arg name="enable_pointcloud" default="true"/>
```

**分析**：启用点云发布。

**结论**：❌ **不采用**

**原因**：
- 当前书本识别不需要点云
- 点云会增加带宽和 CPU 负载
- 使用上游默认值 false

### 2.3 rs_rgbd.launch

**修改内容**：

```diff
- <arg name="fisheye_fps" default="-1"/>
- <arg name="depth_fps" default="-1"/>
- <arg name="infra_fps" default="-1"/>
- <arg name="color_fps" default="-1"/>
- <arg name="gyro_fps" default="-1"/>
- <arg name="accel_fps" default="-1"/>
+ <arg name="fisheye_fps" default="10"/>
+ <arg name="depth_fps" default="10"/>
+ <arg name="infra_fps" default="10"/>
+ <arg name="color_fps" default="10"/>
+ <arg name="gyro_fps" default="10"/>
+ <arg name="accel_fps" default="10"/>
```

**分析**：将所有帧率从默认值改为 10fps。

**结论**：❌ **不采用**

**原因**：
- 降低帧率是为了多相机场景节省带宽
- 单台 D455 不需要限制帧率
- 使用上游默认值（设备最大帧率）

### 2.4 rs_multiple_devices.launch

**修改内容**：

1. 添加硬编码序列号：
   - camera1: `233522075143`
   - camera2: `233522075276`
   - camera3: `250122074393`
   - camera4: `311322301427`

2. 添加第 4 个相机支持

3. 为每个相机添加参数：
   ```xml
   <arg name="enable_pointcloud" default="true"/>
   <arg name="enable_sync" default="true"/>
   <arg name="align_depth" default="true"/>
   ```

**分析**：多相机配置，包含硬编码序列号。

**结论**：❌ **不采用**

**原因**：
- 纯多相机配置，与单台 D455 无关
- 包含硬编码序列号，不通用
- 不应复制到新项目

### 2.5 rs_multiple_devices_copy.launch

**文件状态**：未跟踪文件

**分析**：是 `rs_multiple_devices.launch` 的备份副本，包含 3 个相机配置（没有第 4 个）。

**结论**：❌ **舍弃**

**原因**：
- 名称含 "copy" 的文件默认不纳入项目
- 是备份文件，不是正式修改
- 与单台 D455 无关

## 3. 总结

| 文件 | 修改类型 | 是否采用 | 原因 |
|------|----------|----------|------|
| rs_camera.launch | 参数修改 | ❌ | 与书本识别无关 |
| rs_aligned_depth.launch | 参数修改 | ❌ | 不需要点云 |
| rs_rgbd.launch | 参数修改 | ❌ | 单台不需要限帧 |
| rs_multiple_devices.launch | 多相机配置 | ❌ | 纯多相机，含硬编码 |
| rs_multiple_devices_copy.launch | 备份副本 | ❌ | 舍弃 |

## 4. 决策

- **不将整个 realsense-ros 复制到 DOBOT-CR5 仓库**
- **使用上游 2.3.2 默认的 launch 文件**
- **采用软链接方式创建独立工作空间**
- **在项目自己的 launch 或命令参数中显式配置所需参数**

## 5. 后续建议

如需项目专用相机启动文件，可创建：

```
src/cr5_book_spray_demo/launch/d455_camera.launch
```

该文件只 include 上游 `realsense2_camera/launch/rs_camera.launch` 并传递经确认兼容的参数：

```xml
<launch>
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true"/>
  </include>
</launch>
```

不要修改 Intel 的原始包文件。

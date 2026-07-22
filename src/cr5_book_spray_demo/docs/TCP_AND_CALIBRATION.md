# D455 外参与工具 TCP 处理

## 1. D455 是 Eye-in-Hand

完整变换链应当类似：

```text
base_link → ... → Link6 → camera_link → camera_color_optical_frame
```

`Link6 → camera_link` 是固定手眼外参；`camera_link → camera_color_optical_frame` 通常由 RealSense 驱动提供。

## 2. 先搜索旧工程

运行：

```bash
rosrun cr5_book_spray_demo find_legacy_calibration.sh ~/cr5_ws ~/cr5_ros1_ws
```

重点比较：

- 当前 `/robot_description` 中是否已有 camera 固定关节；
- 旧 `cr5_ws` 中是否有同名固定关节；
- launch 中是否还有 `static_transform_publisher`；
- D455 支架、孔位和朝向是否与旧工程完全相同；
- 是否存在 hand-eye YAML/JSON 或标定报告。

## 3. 不允许重复发布

错误示例：

```text
URDF 发布 Link6 → camera_link
同时 launch 又发布 Link6 → camera_link
```

这可能造成 TF 冲突、时间跳变或来源不确定。保留一个来源，优先 URDF/Xacro。

## 4. 如何判断旧外参能否沿用

只有同时满足以下条件才沿用：

- 同一台 D455；
- 同一末端安装板；
- 同一组安装孔；
- 安装方向未改变；
- 支架无弯曲、松动或重新装配；
- 实物验证误差满足任务要求。

否则重新做 hand-eye 标定。旧数据只能作为初值。

## 5. D455 内参

本节点每帧从：

```text
/camera/color/camera_info
```

读取内参，不应把另一台相机或另一分辨率的 `fx/fy/cx/cy` 写死。

## 6. 新建 spray_tcp

当前 `Tool_end` 是否对应照片中的真实工具作用点并不确定。建议新建：

```text
Link6 → spray_tcp
```

约定：

- 原点位于喷嘴出口或干跑指示针作用点；
- `+Z` 从工具指向目标表面；
- `+X` 作为喷涂移动的主方向；
- 固定外参由实测或 TCP 标定获得。

模板位于 `spray_tcp_template.xacro`。

## 7. 启用自动法向对齐

只有 `spray_tcp` 验证后，才能使用：

```bash
orientation_mode:=align_to_book eef_link:=spray_tcp
```

规划器采用：

```text
spray_tcp +X = book +X
spray_tcp +Z = -book +Z
```

也就是工具喷射方向指向封面。

## 8. 最低验证方法

在 RViz 中同时显示：

- RobotModel；
- TF；
- `book_locked`；
- `spray_tcp`；
- 规划路径 Marker。

让机器人保持静止，用直尺或已知尺寸块比较：

- 相机识别书本中心是否落在实体中心；
- `book +Z` 是否从封面向上；
- `spray_tcp` 原点是否在实际工具末端；
- `spray_tcp +Z` 是否朝向书本。

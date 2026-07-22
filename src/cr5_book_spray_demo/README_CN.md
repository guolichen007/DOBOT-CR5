# CR5 + D455 书本识别与“仿喷涂”干跑验证

本包用于在现有 `~/cr5_ros1_ws` 上增加一条独立功能链：

```text
D455 彩色图 + 对齐深度
→ 识别绿色输送台上的深色矩形书本
→ 拟合书本封面平面
→ 将书本位姿转换到 base_link
→ 人工锁定稳定目标
→ MoveIt 生成从封面视觉上端到下端的路径
→ 默认只规划
→ 经过明确解锁后，CR5 低速、无喷料、无接触干跑
```

## 1. 当前工程边界

本包不会：

- 启动或重复启动 `/cr5_robot`；
- 自动启动现有 MoveIt；
- 控制喷阀、气源或喷漆；
- 在识别到目标后自动运动；
- 使用旧 YOLO 模型直接决定实机轨迹。

本包只增加两个核心节点：

- `book_pose_estimator.py`：识别与目标锁定，不控制机器人；
- `book_spray_planner.py`：MoveIt 路径规划，默认禁止执行。

## 2. 为什么第一版不依赖 YOLO

照片中的目标具有很强的几何和颜色先验：

- 书本封面为深蓝色矩形；
- 支撑面为绿色输送台；
- D455 能提供与彩色图对齐的深度；
- 任务只需要封面平面、中心、长边、短边和法向。

因此第一版采用：

```text
排除绿色背景
→ 最大候选旋转矩形
→ 书本内部深度点
→ 平面拟合
→ 四角射线与平面求交
```

这比仅使用 YOLO 检测框更适合生成毫米级三维路径。旧 YOLO 代码和模型继续保留，第二阶段可改成“书本实例分割 mask + 同一套深度平面拟合”。

## 3. 坐标约定

检测节点输出 `book_locked`：

- 原点：书本封面中心；
- `+X`：在相机图像中从书本视觉上端指向下端，沿书本长边；
- `+Y`：书本短边方向；
- `+Z`：从封面向外、朝相机一侧。

所以封面上方安全距离为：

```text
P = book_center + x * book_X + y * book_Y + standoff * book_Z
```

第一条单线轨迹使用 `y=0`，从 `x=-L/2` 到 `x=+L/2`。

## 4. 目录

```text
cr5_book_spray_demo/
├── CMakeLists.txt
├── package.xml
├── README_CN.md
├── CLAUDE_DESKTOP_EXECUTION_PROMPT.md
├── config/
│   ├── book_vision.yaml
│   ├── book_path.yaml
│   └── safety.yaml
├── launch/
│   ├── vision_only.launch
│   ├── planner_only.launch
│   └── book_demo.launch
├── scripts/
│   ├── book_pose_estimator.py
│   ├── book_spray_planner.py
│   ├── find_legacy_calibration.sh
│   ├── preflight_book_demo.sh
│   ├── book_demo_cli.sh
│   └── offline_book_detector_tuner.py
└── docs/
    ├── ARCHITECTURE_AND_SAFETY.md
    ├── TCP_AND_CALIBRATION.md
    ├── TEST_ACCEPTANCE.md
    ├── spray_tcp_template.xacro
    └── reference/
```

## 5. 安装

```bash
cd ~/cr5_ros1_ws
cp -a /path/to/cr5_book_spray_demo src/
chmod +x src/cr5_book_spray_demo/scripts/*

source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

如果工作空间使用 `catkin build`，改用现有构建方式，不要同时混用。

## 6. 先找旧标定，不要直接复制未知外参

```bash
cd ~/cr5_ros1_ws
source devel/setup.bash
rosrun cr5_book_spray_demo find_legacy_calibration.sh \
  ~/cr5_ws ~/cr5_ros1_ws
```

该脚本会搜索：

- `camera_link`、`camera_color_optical_frame`；
- `Link6`、`Tool_end`、`spray_tcp`；
- URDF/Xacro 固定关节；
- `static_transform_publisher`；
- hand-eye/calibration YAML/JSON；
- RealSense launch；
- 旧模型文件。

只有在确认 D455 支架与旧工程完全没有移动时，才允许沿用旧 `Link6 → camera_link` 外参。一个父子 TF 只能由一处发布：优先写入 URDF，不要同时再启动重复 static TF。

## 7. 视觉测试：机器人保持静止或失能

测量书本真实尺寸并修改：

```text
config/book_path.yaml
  book_length_m
  book_width_m
  book_thickness_m
```

然后：

```bash
roslaunch cr5_book_spray_demo vision_only.launch start_camera:=true
```

另开终端：

```bash
rqt_image_view /book_demo/estimator/debug_image
```

TF 与话题检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rosrun tf tf_echo base_link camera_color_optical_frame
```

让 D455 能完整看到书本，并在四周保留一定背景。输送带必须停止，书本不得移动。

锁定目标：

```bash
rosservice call /book_demo/estimator/lock_target '{}'
```

查看：

```bash
rostopic echo -n 1 /book_demo/estimator/locked_pose
rostopic echo -n 1 /book_demo/estimator/locked_size
```

RViz 添加：

- `MarkerArray`：`/book_demo/estimator/markers`；
- `TF`：检查 `book_locked`；
- `Image`：`/book_demo/estimator/debug_image`。

清除锁定：

```bash
rosservice call /book_demo/estimator/clear_target '{}'
```

## 8. MoveIt 只规划

保持现有 CR5 驱动和 MoveIt 按原工程方式启动。不要重复启动同名节点。

```bash
roslaunch cr5_book_spray_demo planner_only.launch \
  allow_execution:=false \
  path_mode:=single_stroke \
  orientation_mode:=keep_current \
  eef_link:=Tool_end
```

`keep_current` 表示保留当前工具姿态。第一次测试应先在 RViz 中手动将工具大致朝向封面法向，再锁定书本并规划。

规划：

```bash
rosservice call /book_demo/planner/plan_path '{}'
```

这一步不会发送实机运动。RViz 中必须检查：

- 从书本视觉上端到下端；
- 路径位于封面内部边距内；
- 高度为配置的 `standoff_m`；
- 没有翻腕；
- 没有撞向输送台、书本、底座或线缆；
- Cartesian fraction 不低于 `0.995`。

## 9. 第一次实体干跑

第一阶段只允许：

```text
单条中心线
无气源
无喷漆
无喷阀
工具不接触书本
控制器全局速度 5%
standoff 建议先保持 100 mm
手在物理急停附近
```

执行需要重新以允许执行模式启动 planner，并重新规划：

```bash
roslaunch cr5_book_spray_demo planner_only.launch \
  allow_execution:=true \
  path_mode:=single_stroke \
  orientation_mode:=keep_current \
  eef_link:=Tool_end
```

再次规划并复核 RViz：

```bash
rosservice call /book_demo/planner/plan_path '{}'
```

确认机器人状态、报警和 5% 速度后，设置一次性令牌：

```bash
rosparam set /book_demo/confirm_execute CR5_BOOK_DRY_RUN_EXECUTE
```

真正执行：

```bash
rosservice call /book_demo/planner/execute_path '{}'
```

> 上述 execute 服务会造成实体 CR5 运动。节点内部还有 5 秒倒计时，但不能替代物理急停。

## 10. 从单线升级到覆盖路径

通过顺序：

```text
30~50 mm 小线段
→ 封面中心完整单线
→ 3 条扫描线
→ 完整 raster
```

完整覆盖：

```bash
roslaunch cr5_book_spray_demo planner_only.launch \
  allow_execution:=false \
  path_mode:=raster \
  orientation_mode:=keep_current
```

修改 `pass_spacing_m` 可调整扫描线间距。当前仅验证几何和控制链路，不代表真实喷涂速度、流量、重叠率或膜厚已经受控。

## 11. `align_to_book` 的启用条件

只有完成以下事项后才改为：

```text
orientation_mode:=align_to_book
eef_link:=spray_tcp
```

前提：

- URDF 中存在准确的 `spray_tcp`；
- `spray_tcp` 原点在实际喷嘴/指示针作用点；
- `spray_tcp +Z` 从喷嘴指向目标；
- RViz 坐标轴与实体一致；
- 工具碰撞模型与线缆风险已检查。

详见 `docs/TCP_AND_CALIBRATION.md`。

## 12. 常见问题

### 找不到书本

优先调整：

```text
book_vision.yaml:
  roi_norm
  background_hsv_lower / upper
  min_area_px
  expected_aspect
```

也可将 `detection_mode` 改为 `edge_rectangle`。

### 书本框正确但无法锁定

检查：

- 机械臂是否完全静止；
- D455 深度是否存在空洞；
- TF 是否能按图像时间戳查询；
- `plane_rmse` 是否过大；
- D455 或支架是否松动。

### 路径方向错误

检测节点定义 `book +X` 为相机图像中视觉上端到下端。先确认 D455 图像的实际上下方向。必要时在代码中改变长轴符号规则，不能通过实机试错。

### 规划成功但 Tool_end 与实体工具不一致

不要执行。先建立并验证 `spray_tcp`，或使用更大的安全距离进行纯几何空跑。

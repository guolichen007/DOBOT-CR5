# CR5 仿真基线审计报告

> 生成时间：2026-07-20
> 审计分支：feature/book-vision-spray-demo-v1 (HEAD: 7a6856a)
> 目标分支：feature/cr5-spray-gazebo-demo-v1

## 环境信息

| 项目 | 值 |
|------|-----|
| OS | Ubuntu 20.04.6 LTS, Kernel 5.15.0-139-generic |
| ROS | Noetic 1.17.4 |
| Gazebo | 11.15.1 |
| MoveIt | 1.1.16 |
| Python | 3.8.10 |
| catkin | 0.8.12 |
| PCL | 1.10.0 |
| OpenCV | 4.2.0 |
| gazebo-ros-control | 2.9.3 |

## 1. 仓库结构审查

```
src/
├── a4_spray_demo/          # A4纸喷涂demo参考
├── cr5_book_spray_demo/     # 书本喷涂demo（含视觉、路径）
├── cr5_moveit/              # MoveIt配置包
├── dobot_bringup/           # 实机驱动（禁止在仿真中使用）
├── dobot_description/       # 实机URDF模型
└── dobot_moveit/            # DOBOT MoveIt配置（旧）
```

## 2. 关键文件审查

### 2.1 URDF: `cr5_robot.urdf`
- ✅ 定义完整六轴机械臂 Link1-Link6 + base_link + dummy_link
- ✅ 包含相机手眼标定：camera_link, camera_color_frame, camera_color_optical_frame
- ✅ 碰撞模型使用 DAE mesh
- ⚠️ 所有 joint effort/velocity 均为 0（实机安全设置，仿真需用非零值）
- ⚠️ joint limits 与 DOBOT 官网一致

### 2.2 末端工具结构
- Tool_box1 → Tool_box2 → Tool_end（实际工具末端）
- camera_link 通过 camera_to_hand_joint 固定在 Link6
- Tool_end 位置在 Link6 前面约 (X:-0.0068, Y:0.15, Z:0.225) 处

### 2.3 SRDF: `cr5_robot.srdf`
- ✅ 定义 planning group `cr5_arm`: joint1-joint6
- ✅ virtual_joint: world → dummy_link (fixed)
- ✅ 禁用相邻及非可能碰撞对
- ✅ home state: all zeros

### 2.4 MoveIt Config
- `joint_limits.yaml`: ⚠️ 所有关节 has_velocity_limits=false, max_velocity=0
- `kinematics.yaml`: KDL solver, timeout 0.05s
- `gazebo_cr5_robot.urdf`: ✅ 已有 transmission + gazebo_ros_control plugin
  - 但 transmission 使用 `EffortJointInterface`
- `gazebo_controllers.yaml`: ⚠️ 仅有 joint_state_controller，缺少 arm_controller
- `ros_controllers.yaml`: 定义 controller_list 使用 `cr5_robot/joint_controller`

### 2.5 Launch
- `demo.launch`: 使用 fake_execution=true，fake controller manager
- `demo_gazebo.launch`: 组合 gazebo.launch + demo.launch
- `gazebo.launch`: 加载 `gazebo_cr5_robot.urdf`，启动 Gazebo + controllers
- `planning_context.launch`: 加载 `cr5_robot.urdf`（实机用），非 Gazebo 版本

## 3. 仿真缺口分析

| 缺口 | 描述 | 影响 |
|------|------|------|
| 无 arm_controller | gazebo_controllers.yaml 缺少 JointTrajectoryController | 无法在 Gazebo 中控制机械臂运动 |
| Effort 接口 | transmission 使用 EffortJointInterface | 需改为 PositionJointInterface 或保持（取决于控制方案） |
| effort=0 | URDF joint limits effort=0 | Gazebo 不会施加任何力/力矩限制 |
| fake execution | MoveIt 默认使用 fake controller | Gazebo 真执行时需切换到 ros_control |
| 无碰撞简化 | 碰撞体使用完整 DAE mesh | 仿真性能可能较低 |
| 无场景模型 | 无框架、工件、工作台模型 | 需要新建 |
| 无 RGB-D 插件 | 无 Gazebo 深度相机插件 | 需要从 src.zip 引入 |
| 无 damping/friction | 关节无阻尼/摩擦 | 可能导致仿真抖动 |

## 4. 基线编译结果

```
✅ catkin_make 编译成功
   - 6 packages traversed: cr5_book_spray_demo, dobot_moveit, dobot_bringup,
     dobot_description, cr5_moveit, a4_spray_demo
   - 0 errors, 0 warnings
```

## 5. 基线 MoveIt Smoke Test

```
✅ roslaunch cr5_moveit demo.launch 启动成功
   - /move_group 节点运行正常
   - /robot_description 参数加载成功
   - /robot_description_semantic 参数加载成功
   - /joint_states 发布 joint1-joint6（全为 0）
   - planning group cr5_arm 可用
   - OMPL 规划器加载成功
   - fake_manipulator_controller 加载成功
   - MoveGroup Action/Service 全部注册
⚠️ Rviz MotionPlanning 插件加载失败（GUI仅，不影响 headless 仿真）
```

## 6. src.zip 审查

- 位置：`/home/ydkj/data/src.zip` (120MB)
- 解压到：`/tmp/cr5_spray_src_20260720_*`
- 候选复用包：
  - `realsense_gazebo_description` — RGB-D 相机 xacro 模型
  - `realsense_gazebo_plugin` — Gazebo RealSense 插件（C++）
- 排除：smartcar_description, navigation, waterplus_map_tools, franka_description, .git

## 7. 下一步建议

1. 引入 realsense_gazebo_plugin/description 并适配 Noetic/Gazebo11
2. 创建 cr5_spray_sim 包，包含 SIM 专用 xacro overlay
3. 配置 PositionJointTrajectoryController
4. 建立场景模型（框架、工件、相机）
5. 打通 MoveIt ↔ Gazebo 真执行链路

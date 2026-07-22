# CR5 + D455 当前启动流程与视觉调试指南

## 1. 已验证硬件和版本

### 1.1 CR5 机械臂

```
IP: 192.168.110.214
端口: 29999, 30003, 30004
```

### 1.2 D455 相机

```
型号: Intel RealSense D455
序列号: 311322304396
USB: 3.2
固件: 5.15.1
RealSense ROS: 2.3.2
Librealsense: 2.54.2
运行时库: /usr/local/lib/librealsense2.so.2.54
```

### 1.3 流配置

```
Depth: 848 × 480 @ 30 FPS, Z16
Color: 1280 × 720 @ 30 FPS, RGB8
实际 Color: 约 28.6 Hz
实际 aligned depth: 约 27.7～29 Hz
```

### 1.4 成功话题

```
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
```

## 2. 环境加载

```bash
cd ~/cr5_ros1_ws
source scripts/dev/env.sh
```

验证：

```bash
rospack find realsense2_camera
rospack find cr5_book_spray_demo
rospack find dobot_bringup
```

## 3. 系统诊断

```bash
doctor --offline    # 离线检查
doctor --runtime    # 运行时检查
doctor --vision     # 视觉检查
```

## 4. 启动 CR5 Driver

```bash
start_driver
```

检查状态：

```bash
robot_status
```

## 5. 安全使能机器人

```bash
enable_robot_safe
```

必须输入 `ENABLE_CR5` 确认。

验证：

```
RobotMode = 5
is_enable = True
EnableStatus = 1
ErrorStatus = 0
```

## 6. 启动 MoveIt

```bash
start_moveit
```

RViz 设置：

```
Start State = Current
Velocity Scaling = 0.03～0.05
Accel. Scaling = 0.03～0.05
```

先 `Plan`，审核关节变化，再单独 `Execute`。

## 7. 调整完成后下使能

```bash
disable_robot_safe
```

验证：

```
is_enable = False
EnableStatus = 0
ErrorStatus = 0
RunQueuedCmd = 0
```

注意：下使能时可能有抱闸收敛，机械臂可能会有轻微移动。

**常规视觉调试不按急停。急停只用于异常运动或人员进入危险区域。**

## 8. 启动 D455 相机

```bash
start_camera
```

检查：

```bash
timeout 10 rostopic hz /camera/color/image_raw
timeout 10 rostopic hz /camera/aligned_depth_to_color/image_raw
```

## 9. 检查机器人到相机 TF

```bash
timeout 10 rosrun tf tf_echo base_link camera_color_optical_frame
```

如果 TF 不存在：

- 允许继续查看彩色/深度/调试图
- 禁止锁定为机器人基座目标
- 禁止 MoveIt plan-only

## 10. 启动书本视觉

```bash
start_book_demo
```

或一键启动纯视觉模式：

```bash
start_vision
```

查看输出：

```bash
rqt_image_view /book_demo/estimator/debug_image
rostopic echo /book_demo/estimator/book_pose
rostopic echo /book_demo/estimator/plane_rmse
```

锁定目标：

```bash
rosservice call /book_demo/estimator/lock_target '{}'
```

## 11. 当前阶段限制

当前必须保持：

```
allow_execution = false
机械臂下使能
不发送喷涂轨迹
不连接气源和喷阀
```

禁止：

- `allow_execution:=true`
- `/book_demo/planner/execute_path`
- `/book_demo/confirm_execute`
- `EnableRobot`（自动调用）
- 机械臂实体运动
- 喷阀、气源、喷漆

## 12. 已知问题

以下问题未解决前不得执行自动喷涂：

### 12.1 J6 角度绕圈

J6 存在角度绕圈问题，后续 MoveIt 实体执行前必须处理最短角距离。

### 12.2 MoveIt 起点同步

MoveIt 规划起点可能与实际关节位置不同步。

### 12.3 trajectory_duration

当前 `trajectory_duration=0.30`，可能需要调整。

### 12.4 大 IK 跳变

IK 求解可能出现大跳变，需要平滑处理。

## 13. 标准命令速查

```bash
# 环境加载
source scripts/dev/env.sh

# 诊断
doctor --runtime

# Driver
start_driver
robot_status

# 使能/下使能
enable_robot_safe
disable_robot_safe

# MoveIt
start_moveit

# 视觉
start_camera
doctor --vision
start_book_demo

# 或一键启动
start_vision

# 查看
rqt_image_view /book_demo/estimator/debug_image
rostopic echo /book_demo/estimator/book_pose
rostopic echo /book_demo/estimator/plane_rmse

# 锁定
rosservice call /book_demo/estimator/lock_target '{}'

# 停止
stop_all
```

## 14. 文件清单

```
scripts/dev/
├── env.sh                  # 环境加载
├── common.sh               # 公共函数
├── doctor.sh               # 系统诊断
├── build.sh                # 编译项目
├── robot_status.sh         # 机器人状态（只读）
├── enable_robot_safe.sh    # 安全使能
├── disable_robot_safe.sh   # 安全下使能
├── start_driver.sh         # 启动 CR5 Driver
├── start_moveit.sh         # 启动 MoveIt
├── start_camera.sh         # 启动 D455 相机
├── start_book_demo.sh      # 启动书本识别
├── start_vision.sh         # 纯视觉一键启动
├── start_all.sh            # 启动所有（需要参数）
├── stop_all.sh             # 停止所有
└── clean_logs.sh           # 清理日志

scripts/laptop/
├── setup_realsense_ros1.sh     # RealSense 工作空间设置
├── pull_build_book_demo.sh     # 拉取代码并编译
├── test_d455_vision_only.sh    # D455 视觉测试
├── check_d455_topics.py        # 话题一致性检查
├── plan_book_demo_only.sh      # MoveIt plan-only
└── load_book_demo_environment.sh  # 环境加载（复用 common.sh）

docs/
├── DEV_TOOLKIT.md                                    # 开发工具链文档
├── CR5_D455_CURRENT_STARTUP_AND_VISION_DEBUG.md      # 本文档
├── REALSENSE_ROS1_SETUP.md                           # RealSense 设置
├── REALSENSE_ROS1_2_3_2_LOCAL_DIFF_AUDIT.md          # RealSense 审计
└── BOOK_SPRAY_LAPTOP_REMOTE_TEST.md                  # 笔记本测试流程
```

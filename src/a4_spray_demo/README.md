# a4_spray_demo（审查修正版）

用于现有 DOBOT CR5 ROS 1 Noetic 工作空间的 A4 面积往复式覆盖演示。

## 当前阶段验证什么

验证链路：

```text
/joint_states
→ MoveIt 1
→ A4 局部平面栅格路径
→ 笛卡尔插值与 IK
→ JointTrajectory
→ /cr5_robot/joint_controller/follow_joint_trajectory
→ 固定周期 ServoJ
→ CR5 实机
```

只验证几何覆盖，不验证喷涂速度、喷幅或漆膜均匀性。

## 与原 Claude 版本相比的修正

- 修复 launch 中无效的 `$(arg execute:=--execute)` 写法；
- 通过 `catkin_install_python` 安装脚本，避免 `rosrun` 找不到不可执行脚本；
- 真正读取 YAML 参数；
- 路径沿所选 TCP 的局部 X/Y 轴生成，不再错误地沿 `base_link` X/Y；
- 默认使用 `Tool_end`，也可切换到 `Link6`；
- 执行要求 99.9% 以上完整路径；
- 加入确认口令、实时连接/使能/报警检查；
- 禁止执行速度倍率超过 5%；
- 发布 `/a4_spray_demo/path_marker` 和 `/move_group/display_planned_path`；
- 移除危险的“plan-only 模式仍然自动回 home”逻辑；
- 增加可返回非零状态的预检程序。

## 安装

把本目录替换原来的 `~/cr5_ws/src/a4_spray_demo`：

```bash
cd ~/cr5_ws
rm -rf src/a4_spray_demo
cp -r /path/to/a4_spray_demo_reviewed src/a4_spray_demo

source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
export DOBOT_TYPE=cr5
```

## 正确的服务请求格式

当前源码中的 `EnableRobot.srv` 是 `float64[] args`，不是 `load` 字段：

```bash
rosservice call /dobot_bringup/srv/ClearError "{}"
rosservice call /dobot_bringup/srv/EnableRobot "{args: []}"
rosservice call /dobot_bringup/srv/SpeedFactor "{ratio: 5}"
```

紧急停止：

```bash
rosservice call /dobot_bringup/srv/EmergencyStop "{}"
```

物理急停必须始终有人可立即触达。

## 启动

终端 1：

```bash
roscore
```

终端 2：

```bash
source ~/cr5_ws/devel/setup.bash
export DOBOT_TYPE=cr5
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214
```

终端 3：

```bash
source ~/cr5_ws/devel/setup.bash
export DOBOT_TYPE=cr5
roslaunch dobot_moveit moveit.launch
```

终端 4，预检：

```bash
source ~/cr5_ws/devel/setup.bash
export DOBOT_TYPE=cr5
roslaunch a4_spray_demo quick_check.launch eef_link:=Tool_end
```

## 第一次：小范围、只规划

人工将 `Tool_end` 移到 100 × 30 mm 测试区域的第一个角点，并确保：

- Tool_end 局部 +X 沿长边；
- Tool_end 局部 +Y 沿短边；
- Tool_end 局部 Z 垂直纸面；
- 工具与纸面保持安全距离。

```bash
roslaunch a4_spray_demo a4_raster_demo.launch \
  config:=$(rospack find a4_spray_demo)/config/small_test.yaml \
  execute:=false \
  eef_link:=Tool_end
```

在 RViz 添加 Marker：

```text
/a4_spray_demo/path_marker
```

## 小范围实机执行

先清错、使能并设置控制器侧 5% 速度：

```bash
rosservice call /dobot_bringup/srv/ClearError "{}"
rosservice call /dobot_bringup/srv/EnableRobot "{args: []}"
rosservice call /dobot_bringup/srv/SpeedFactor "{ratio: 5}"
```

然后：

```bash
roslaunch a4_spray_demo a4_raster_demo.launch \
  config:=$(rospack find a4_spray_demo)/config/small_test.yaml \
  execute:=true \
  confirmation:=CR5_A4_EXECUTE \
  eef_link:=Tool_end
```

## 完整 A4：先规划，再执行

只规划：

```bash
roslaunch a4_spray_demo a4_raster_demo.launch \
  execute:=false \
  eef_link:=Tool_end
```

确认 RViz、机械臂可达性和线缆安全后：

```bash
roslaunch a4_spray_demo a4_raster_demo.launch \
  execute:=true \
  confirmation:=CR5_A4_EXECUTE \
  eef_link:=Tool_end
```

## Tool_end 必须现场确认

当前 URDF 中：

```text
Link6 → Tool_end
xyz = (-0.00683, 0.150178, 0.22462) m
rpy = (-0.0044, 0.04684, -1.631) rad
```

这是一段很大的工具偏移。如果该红色 Tool_end 不等于当前实体指针/模拟喷头的位置，
必须改用 `Link6` 或重新标定测试 TCP，不能盲目执行完整 A4。

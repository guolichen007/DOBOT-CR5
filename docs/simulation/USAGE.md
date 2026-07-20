# CR5 喷涂仿真使用指南

## 快速启动

```bash
source /opt/ros/noetic/setup.bash
source ~/cr5_ros1_ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=~/cr5_ros1_ws/devel/lib:/opt/ros/noetic/lib
```

### 1. CR5-only Gazebo 测试

```bash
roslaunch cr5_spray_sim cr5_only_gazebo.launch gui:=false headless:=true
# 检查控制器
rosservice call /controller_manager/list_controllers
# 检查关节
rostopic echo -n 1 /joint_states
```

### 2. RGB-D 相机烟雾测试

```bash
roslaunch realsense_gazebo_description d455_like_smoke.launch gui:=false headless:=true
python3 /path/to/d455_like_smoke_test.py
```

### 3. 完整场景（headless）

```bash
roslaunch cr5_spray_sim full_scene.launch gui:=false headless:=true
```

### 4. MoveIt + Gazebo 真执行

```bash
roslaunch cr5_spray_sim moveit_gazebo.launch gui:=true headless:=false
```

### 5. Headless 集成测试

```bash
bash ~/cr5_ros1_ws/src/cr5_spray_sim/test/test_sim_headless.sh
```

### 6. 手动 Gazebo/RViz

```bash
# Gazebo GUI
roslaunch cr5_spray_sim full_scene.launch gui:=true

# RViz
roslaunch cr5_moveit demo.launch
```

## 相机配置

切换相机布局：
- 理想（6相机）: `full_scene.launch camera_layout:=ideal`
- 现实（2相机）: `full_scene.launch camera_layout:=realistic`

自定义：编辑 `config/camera_layout_*.yaml`

## 工件切换

```bash
roslaunch cr5_spray_sim full_scene.launch object_type:=cylinder_part
# 可选: flat_panel, cylinder_part, asymmetric_part
```

## 注意事项

1. 这是纯仿真任务，禁止启动 dobot_bringup
2. 禁止连接 192.168.* 机械臂控制柜
3. 所有参数在 YAML/xacro/launch 中配置
4. D455-like 不是真实 D455 物理模型

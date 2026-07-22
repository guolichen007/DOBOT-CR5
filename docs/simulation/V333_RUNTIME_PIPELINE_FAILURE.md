# V3.3.2 运行时管道故障记录

> 日期: 2026-07-21  
> 仓库: guolichen007/DOBOT-CR5  
> 分支: fix/cr5-spray-tool-v33-stable-painting  
> HEAD: df05062

## GUI 建模状态: 正常 ✓

- CR5 直立，六轴零位
- Link6 和喷枪几何可见
- 门架、圆柱吊件、三台相机支架位置正确
- 模型不散架，无地下塌陷

## 运行时故障清单

### 1. 控制器 loaded but stopped

**现象**: `rosservice call /controller_manager/list_controllers "{}"` 显示两个控制器均为 `stopped`

**根因**: `scene_v33_spray.launch` 第 75 行:

```xml
args="--stopped joint_state_controller arm_controller"
```

wrapper 脚本 `run_scene_v33_spray.sh` 删除了原有的 `switch_controller` 调用，只被动等待 `/joint_states`。控制器被明确加载为 stopped，wrapper 不启动它们，却等待它们产出的 joint_states —— 逻辑死锁。

**影响**:
- 无 `/joint_states` 消息
- robot_state_publisher 不发布 CR5 运动关节 TF
- `world→spray_nozzle_frame` TF 链断裂
- 喷枪服务 TF 查询挂起

### 2. unpause 失败被静默吞掉

**现象**: 即使 `/clock` 不推进，wrapper 仍打印 `CR5_INITIAL_POSE_READY`

**根因**: `run_scene_v33_spray.sh` 第 321 行:

```bash
rosservice call /gazebo/unpause_physics 2>/dev/null || true
```

无论服务调用成功、失败、transport error，脚本都继续执行。

**影响**: Gazebo 可能仍然 paused，`/clock` 不推进，全部仿真时间相关操作冻结

### 3. 无 /clock 推进检查

**现象**: `rostopic echo /cam_front_left/camera/color/camera_info` 提示 `simulated time is active. Is /clock being published?`

**根因**: 相机 sensor 只在 Gazebo 仿真步进时发布图像。当前只检查:
- topic 是否注册
- publisher 是否存在
- 服务是否可调用

但从未验证 `/clock` 实际在推进。

**影响**: CameraInfo topic 名存在，publisher 已注册，Gazebo 暂停 → 0 帧图像

### 4. 喷枪服务在 sim time 冻结时无限等待

**现象**: `rosservice call /spray_demo/set_spray "data: true"` 长时间卡住不返回

**根因**: `spray_simulator_v33.py` 第 234-237 行:

```python
nozzle_world = self.tf_buffer.lookup_transform(
    "world", self.nozzle_frame, rospy.Time(0), rospy.Duration(1.0))
```

该超时依赖 ROS 时间。当 `use_sim_time=true`、`/clock` 不推进、TF 不可用时，1 秒仿真时间永远无法过去，回调持续阻塞。

**影响**: 所有调用 `/spray_demo/set_spray` 的客户端被无限阻塞

### 5. CameraInfo wait_for_message 超时不可靠

**现象**: `check_scene_v332.py` check_camera_topics() 可能在冻结 sim time 下卡住

**根因**: `rospy.wait_for_message(topic, CameraInfo, timeout=15.0)` 使用 ROS time，在 `/clock` 不推进时行为不可预测

### 6. Gazebo 无喷涂视觉

**现象**: Gazebo GUI 中看不到喷雾或工件上色

**根因**: 当前 `spray_simulator_v33.py` 只发布 `visualization_msgs/Marker` 和 `MarkerArray`，这些仅被 RViz 渲染，Gazebo Classic 不会显示。

**影响**: 用户误以为"喷涂不工作"，实际是 Marker 只对 RViz 可见

### 7. 假成功状态

`/spray_demo/state` topic 存在且显示 "OFF"，3 个 spray 服务名可见 → 造成"系统已就绪"的假象。

实际运行链:
```
控制器 stopped → 无 joint_states → TF 断裂 → 喷枪 TF 查询挂起
→ Gazebo 仍 paused → /clock 不推进 → 相机无帧
```

## V3.3.3 修复方向

1. 删除 `--stopped`，Gazebo paused 状态下控制器直接启动
2. unpause 后验证 `/clock` 实际推进
3. 相机检查改为 wall-time，要求实际帧数据
4. 喷枪服务使用 wall-time 非阻塞 TF 查询
5. 增加 clock watchdog，sim time 冻结时快速拒绝
6. 可选：Gazebo 喷雾锥 visual

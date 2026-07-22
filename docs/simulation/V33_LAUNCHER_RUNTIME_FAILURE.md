# V3.3.1 启动器运行时故障记录

> 分支：`fix/cr5-spray-tool-v33-stable-painting`  
> HEAD：`e35d588`  
> 记录日期：2026-07-21

## 已确认 Bug

### 1. 参数解析器 bug（严重）

`run_scene_v33_spray.sh` 使用 `for arg in "$@"` 遍历参数，但在 `--object` 分支内执行 `shift` 后读取 `$1`。
当参数顺序为 `--gui --isolated --object motor_housing_cylinder` 时，`for` 循环先处理 `--gui`，
然后是 `--isolated`。进入 `--object` 分支时，第一个 shift 移除了 `--gui` 而非 `--object`，
导致 `$1` = `--isolated`，`OBJECT` 被赋值为 `--isolated`。

**根因**：`for arg` 和循环内 `shift` 不兼容。

**症状**：`Unknown object type: --isolated`

### 2. 后台 tf_echo 刷屏（严重）

启动器用 `rosrun tf tf_echo ... &` 检查 TF，但：
- tf_echo 是无限输出程序，不会自动退出
- stdout 未重定向
- PID 未可靠记录
- 正常路径不会 kill

**根因**：用持续工具做一次性检查。

### 3. Ctrl+C 后进程泄漏（严重）

cleanup 只管理 `ROS_MASTER_PID` 和 `GZSERVER_PID`，但：
- `GZSERVER_PID` 未被赋值
- `LAUNCH_PID` 未进入 cleanup
- tf_echo 未被结束
- roslaunch 子进程不在同一进程组

**症状**：Ctrl+C 后持续刷 XmlRpcClient write error: Connection refused

### 4. 喷枪独立 link 不可靠（中等）

喷枪使用 `spray_gun_assembly` 独立 link，mass=0、inertia=0。
Gazebo 可能忽略或错误 lump 这种 link，导致 visual 不可见。

### 5. 健康检查方法无效（中等）

使用 `rosparam get /cr5_robot/base_link` 检查模型位姿，
但 Gazebo link pose 不在 ROS 参数服务器中。

### 6. 表面上色范围未明确（中等）

报告中未说明 paint_patches 仅 RViz 可见，Gazebo 工件不变色。

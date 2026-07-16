# CR5 开发工具链（Developer Toolkit）

## 1. 概述

开发工具链将环境、编译、启动、诊断、日志、停止全部统一封装，提供一条命令式的开发体验。

## 2. 快速开始

```bash
# 1. 加载环境（一条命令）
source scripts/dev/env.sh

# 2. 诊断系统
doctor

# 3. 编译项目
build

# 4. 启动各个组件
start_driver
start_moveit
start_camera
start_book_demo

# 5. 或者一键启动
start_all

# 6. 停止所有
stop_all
```

## 3. 脚本清单

| 脚本 | 功能 | 使用方法 |
|------|------|----------|
| `env.sh` | 加载 ROS 环境 | `source scripts/dev/env.sh` |
| `doctor.sh` | 全面系统诊断 | `doctor` 或 `scripts/dev/doctor.sh` |
| `build.sh` | 编译项目 | `build` 或 `scripts/dev/build.sh` |
| `start_driver.sh` | 启动 CR5 Driver | `start_driver` 或 `scripts/dev/start_driver.sh` |
| `start_moveit.sh` | 启动 MoveIt | `start_moveit` 或 `scripts/dev/start_moveit.sh` |
| `start_camera.sh` | 启动 D455 相机 | `start_camera` 或 `scripts/dev/start_camera.sh` |
| `start_book_demo.sh` | 启动书本识别 | `start_book_demo` 或 `scripts/dev/start_book_demo.sh` |
| `start_all.sh` | 一键启动所有 | `start_all` 或 `scripts/dev/start_all.sh` |
| `stop_all.sh` | 停止所有 | `stop_all` 或 `scripts/dev/stop_all.sh` |
| `clean_logs.sh` | 清理日志 | `clean_logs` 或 `scripts/dev/clean_logs.sh` |

## 4. 详细说明

### 4.1 env.sh - 环境加载

加载顺序：

```bash
source /opt/ros/noetic/setup.bash
source ~/realsense_ros1_ws/devel/setup.bash
source ~/cr5_ros1_ws/devel/setup.bash --extend
```

自动设置：

- `ROS_MASTER_URI=http://127.0.0.1:11311`
- 快捷命令别名

### 4.2 doctor.sh - 系统诊断

检查项目：

- Git 状态
- ROS 环境
- ROS 包可用性
- 网络连接（CR5 控制柜）
- USB 设备（D455）
- ROS Master
- 运行中的进程
- 磁盘空间
- 日志目录

输出格式：

```
[PASS] xxx
[WARN] xxx
[FAIL] xxx
```

### 4.3 build.sh - 编译项目

功能：

- 加载 ROS 环境
- 执行 `catkin_make`
- 保存编译日志到 `~/cr5_test_logs/`

### 4.4 start_driver.sh - 启动 CR5 Driver

功能：

- 检查是否已在运行
- 检查网络连接
- 启动 `dobot_bringup`

启动命令：

```bash
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214
```

### 4.5 start_moveit.sh - 启动 MoveIt

功能：

- 检查 CR5 Driver 是否运行
- 检查是否已在运行
- 启动 MoveIt

启动命令：

```bash
roslaunch dobot_moveit moveit.launch
```

### 4.6 start_camera.sh - 启动 D455 相机

功能：

- 检查是否已在运行
- 检查 USB 设备
- 启动项目专用 D455 launch

启动命令：

```bash
roslaunch cr5_book_spray_demo d455_camera.launch
```

### 4.7 start_book_demo.sh - 启动书本识别

功能：

- 检查相机是否运行
- 检查相机话题
- 启动书本识别

启动命令：

```bash
roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false
```

### 4.8 start_all.sh - 一键启动

启动顺序：

1. CR5 Driver
2. MoveIt
3. D455 相机
4. 书本识别

每一步都会检查前置条件，如果失败会跳过并提示。

### 4.9 stop_all.sh - 停止所有

停止顺序：

1. 书本识别
2. RealSense 相机
3. MoveIt
4. CR5 Driver
5. roscore（可选）

### 4.10 clean_logs.sh - 清理日志

清理选项：

- 清理 7 天前的日志
- 清理 30 天前的日志
- 清理所有日志
- 清理 ROS 日志
- 清理 catkin 构建日志

## 5. 开发工作流

### 5.1 日常开发

```bash
# 加载环境
source scripts/dev/env.sh

# 编辑代码
vim src/cr5_book_spray_demo/scripts/book_pose_estimator.py

# 编译
build

# 测试
start_camera
start_book_demo
```

### 5.2 完整测试

```bash
# 加载环境
source scripts/dev/env.sh

# 诊断
doctor

# 一键启动
start_all

# 测试...

# 停止
stop_all
```

### 5.3 问题排查

```bash
# 加载环境
source scripts/dev/env.sh

# 诊断
doctor

# 查看日志
ls ~/cr5_test_logs/

# 清理日志
clean_logs
```

## 6. 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CR5_WS` | `$HOME/cr5_ros1_ws` | CR5 工作空间路径 |
| `REALSENSE_WS` | `$HOME/realsense_ros1_ws` | RealSense 工作空间路径 |
| `LOG_DIR` | `$HOME/cr5_test_logs` | 日志目录 |

## 7. 注意事项

1. **环境加载**：每次打开新终端都需要 `source scripts/dev/env.sh`
2. **启动顺序**：建议按 Driver → MoveIt → Camera → Book Demo 顺序启动
3. **停止顺序**：建议按相反顺序停止
4. **日志管理**：定期清理日志，避免磁盘空间不足
5. **网络检查**：启动前确保 CR5 控制柜网络可达

## 8. 故障排除

### 8.1 命令找不到

**现象**：`bash: doctor: command not found`

**原因**：未加载环境

**解决**：

```bash
source scripts/dev/env.sh
```

### 8.2 编译失败

**现象**：`catkin_make` 失败

**解决**：

```bash
# 查看详细日志
ls -lt ~/cr5_test_logs/build_*.log | head -1

# 清理后重新编译
cd ~/cr5_ros1_ws
rm -rf build/ devel/
build
```

### 8.3 启动失败

**现象**：组件启动失败

**解决**：

```bash
# 诊断
doctor

# 查看进程
ps aux | grep -E "dobot|moveit|realsense"

# 强制停止
stop_all
```

### 8.4 ROS Master 不可达

**现象**：`Unable to communicate with master`

**解决**：

```bash
# 启动 roscore
roscore &

# 或者重新加载环境
source scripts/dev/env.sh
```

## 9. 文件清单

```
scripts/dev/
├── env.sh              # 环境加载
├── doctor.sh           # 全面诊断
├── build.sh            # 编译项目
├── start_driver.sh     # 启动 CR5 Driver
├── start_moveit.sh     # 启动 MoveIt
├── start_camera.sh     # 启动 D455 相机
├── start_book_demo.sh  # 启动书本识别
├── start_all.sh        # 一键启动所有
├── stop_all.sh         # 停止所有
└── clean_logs.sh       # 清理日志
```

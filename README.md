# DOBOT CR5 A4 ROS1 工作空间

基于 ROS1 Noetic 的 DOBOT CR5 机械臂 A4 幅面喷涂轨迹演示工作空间。

## 项目概述

本项目是一个精简的 catkin 工作空间，包含 CR5 机械臂驱动、URDF 模型、MoveIt 配置和 A4 幅面轨迹演示所需的全部 ROS 包。

## 工作空间结构

| 路径 | 说明 |
|------|------|
| `src/dobot_bringup` | V3 TCP/IP 驱动、ROS 服务、反馈和轨迹 Action |
| `src/dobot_description` | CR5 URDF 模型和 DAE 网格文件，含 Tool_end/camera 变换 |
| `src/cr5_moveit` | CR5 MoveIt 1 配置 |
| `src/dobot_moveit` | 基于 DOBOT_TYPE 的 MoveIt 启动器 |
| `src/a4_spray_demo` | A4 幅面喷涂轨迹演示和预检程序 |
| `scripts/` | 编译、驱动启动、MoveIt 启动、网络配置等辅助脚本 |
| `docs/` | 项目文档（部署指南、来源说明等） |
| `config/` | 实验室环境配置文件 |
| `optional/` | ArUco 标定参考代码（不在主工作空间中） |

## 未包含的组件

以下组件已从本工作空间中排除，完整的 `cr5_ws.zip` 应离线保留作为参考：

- `slamit`（CUDA/TensorRT 密集，首期演示不需要）
- 完整的 RealSense ROS 驱动
- V4 驱动、Gazebo、Qt 和 RViz 控制示例包
- 编译生成的 `build/` 和 `devel/` 目录

## 环境要求

- Ubuntu 20.04 LTS
- ROS Noetic（完整桌面版）
- Git
- SSH 客户端

## 快速部署

本项目使用 SSH 访问 GitHub，仓库地址：

```text
git@github-dobot-cr5:guolichen007/DOBOT-CR5.git
```

Windows 主机负责开发和推送，Ubuntu 笔记本通过 Wi-Fi 从 GitHub 拉取代码；Ubuntu 有线网口仅用于连接 DOBOT CR5 控制柜。

完整的 SSH 配置、双网卡设置、首次克隆、编译与日常更新步骤请查看：

- [GitHub SSH 与 Ubuntu 部署指南](docs/GITHUB_SSH_UBUNTU_DEPLOY.md)

## 首次编译

在 Ubuntu 笔记本上克隆仓库后执行：

```bash
cd ~/cr5_ros1_ws
./scripts/build.sh
source devel/setup.bash
```

## 机械臂网络配置

将有线网口直连 DOBOT CR5 控制柜，使用项目脚本配置（`enp3s0` 为示例接口名）：

```bash
cd ~/cr5_ros1_ws
sudo ./scripts/configure_robot_network.sh enp3s0
./scripts/network_check.sh
```

## 启动顺序

启动前请确认：机械臂已上电、控制柜网络已连通、Ubuntu 已完成首次编译。

**终端 1** — ROS 核心：

```bash
roscore
```

**终端 2** — CR5 驱动：

```bash
cd ~/cr5_ros1_ws
./scripts/start_driver.sh
```

**终端 3** — MoveIt：

```bash
cd ~/cr5_ros1_ws
./scripts/start_moveit.sh
```

**终端 4** — 预检：

```bash
cd ~/cr5_ros1_ws
./scripts/preflight.sh
```

## 演示步骤

**第一步** — 规划 100 x 30 mm 小幅测试轨迹：

```bash
./scripts/plan_small.sh
```

**第二步** — 完成物理安全检查后，将控制器侧 SpeedFactor 设为 5，执行小幅测试：

```bash
./scripts/enable_robot_5pct.sh
./scripts/preflight.sh
./scripts/execute_small.sh
```

**第三步** — 小幅测试成功后，规划完整 A4 幅面轨迹：

```bash
./scripts/plan_a4.sh
```

> 完整 A4 幅面的真实执行命令保留在各包的 README 中，需要手动启动，不会通过快捷脚本意外触发。

## 安全提示

- 当前 URDF 中 `Tool_end` 相对于 `Link6` 有较大偏移，执行真实运动前请先与物理工具核实
- 现有驱动以固定周期发送 ServoJ 点位，不遵循 MoveIt 的 `time_from_start`；本演示仅验证几何轨迹覆盖
- 任何涉及真实运动的操作前，请确保操作人员在急停按钮旁
- **首次实机测试请严格按 [实机基础测试指南](docs/REAL_MACHINE_TEST_GUIDE.md) 执行**

## 项目文档

- [CR5 + D455 启动流程与视觉调试指南](docs/CR5_D455_CURRENT_STARTUP_AND_VISION_DEBUG.md) — 当前标准启动流程
- [开发工具链文档](docs/DEV_TOOLKIT.md) — scripts/dev/ 工具链使用说明
- [RealSense ROS1 设置](docs/REALSENSE_ROS1_SETUP.md) — D455 相机工作空间配置
- [笔记本测试流程](docs/BOOK_SPRAY_LAPTOP_REMOTE_TEST.md) — 开发主机与实机笔记本协同测试
- [实机基础测试指南](docs/REAL_MACHINE_TEST_GUIDE.md) — 从网络检查到首次运动的完整安全测试流程
- [GitHub SSH 与 Ubuntu 部署指南](docs/GITHUB_SSH_UBUNTU_DEPLOY.md) — 完整的从零配置到日常使用指南
- [代码传输工作流](docs/CODE_TRANSFER.md) — Windows 主机与 Ubuntu 笔记本之间的版本管理方案
- [来源说明](docs/SOURCE_PROVENANCE.md) — 代码来源和整理记录
- [DOBOT 许可证](docs/DOBOT_VENDOR_LICENSE) — MIT 许可证

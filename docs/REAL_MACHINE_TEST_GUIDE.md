# CR5 实机基础测试指南

本文档指导你在 DOBOT CR5 实机上完成从零到首次运动的全流程测试。遵循"先只读后运动、先小幅度再大范围、先低速再提速"的安全原则。

---

## 目录

1. [测试路线总览](#1-测试路线总览)
2. [更新并编译工作空间](#2-更新并编译工作空间)
3. [检查控制柜网络](#3-检查控制柜网络)
4. [实机测试前的人工安全检查](#4-实机测试前的人工安全检查)
5. [启动驱动（不启动 MoveIt）](#5-启动驱动不启动-moveit)
6. [只读状态检查](#6-只读状态检查)
7. [低速使能机器人](#7-低速使能机器人)
8. [第一次运动：单关节相对移动 1 度](#8-第一次运动单关节相对移动-1-度)
9. [运动后下使能](#9-运动后下使能)
10. [紧急停止方法](#10-紧急停止方法)
11. [启动 MoveIt 并仅规划（不执行）](#11-启动-moveit-并仅规划不执行)
12. [已知问题与注意事项](#12-已知问题与注意事项)
13. [建议保存的测试日志](#13-建议保存的测试日志)
14. [测试通过后的下一步](#14-测试通过后的下一步)

---

## 1. 测试路线总览

完整的测试流程分为多个阶段，每通过一个阶段才进入下一个：

```text
网络连通
  → 编译工作空间
    → 单独启动 DOBOT 驱动（不启动 MoveIt）
      → 只读取状态（不使能、不运动）
        → 5% 速度使能
          → 单关节相对移动 1 度
            → 原路返回
              → 下使能
                → 启动 MoveIt（仅规划，不执行）
```

关键原则：
- **第一次测试不启动 MoveIt**，直接通过 ROS Service 控制
- **第一次运动使用相对偏移**（`RelJointMovJ`），不使用绝对角度或笛卡尔坐标
- **速度始终限制在 5%**，关节速度和加速度都设为 5
- **不执行完整 A4 轨迹**，也不通过 MoveIt 拖动模型执行

---

## 2. 更新并编译工作空间

在 Ubuntu 笔记本上执行：

```bash
cd ~/cr5_ros1_ws

# 切换到主线并拉取最新代码
git switch main
git pull --ff-only origin main

# 确保脚本有执行权限
chmod +x scripts/*.sh

# 编译
./scripts/build.sh
```

编译成功后加载工作空间环境：

```bash
source ~/cr5_ros1_ws/devel/setup.bash
```

验证工作区干净：

```bash
git status --short
```

正常应无输出（没有未提交的修改）。

**说明**：`build.sh` 会先执行 `init_workspace.sh`（初始化 catkin 工作空间软链接），然后调用 `catkin_make` 编译所有 ROS 包。

---

## 3. 检查控制柜网络

确认 Ubuntu 笔记本和 CR5 控制柜的网络配置：

```text
Ubuntu 笔记本有线网口：192.168.110.100/24
CR5 控制柜：192.168.110.214
```

### 3.1 运行项目网络检查脚本

```bash
cd ~/cr5_ros1_ws
./scripts/network_check.sh
```

### 3.2 手动验证 ping

```bash
ping -c 3 192.168.110.214
```

### 3.3 检查三个 TCP 端口

CR5 控制柜通过三条 TCP 连接与上位机通信：

```bash
# 端口 29999：Dashboard 控制（使能、禁用、速度设置等）
nc -zvw3 192.168.110.214 29999

# 端口 30003：运动指令（MovJ、MovL、ServoJ 等）
nc -zvw3 192.168.110.214 30003

# 端口 30004：实时状态反馈（关节角度、位姿、IO 状态，1440 字节/8ms）
nc -zvw3 192.168.110.214 30004
```

### 3.4 预期结果

```text
ping 成功         ← 网络层连通
29999 成功        ← Dashboard 指令通道就绪
30003 成功        ← 运动指令通道就绪
30004 成功        ← 实时反馈通道就绪
```

四个全部通过才能继续。如果某个端口不通，检查：
- 网线是否插好
- 控制柜是否已上电
- 有线网口 IP 是否配置正确（`ip -br addr`）
- 有线网口是否配了网关导致路由冲突（不应配网关）

---

## 4. 实机测试前的人工安全检查

**在输入任何运动命令之前**，现场操作人员必须完成以下检查：

### 4.1 环境安全

- 机械臂工作范围内**无人**
- 机械臂周围**没有桌面、支架、相机架等碰撞物**
- 物理**急停按钮有人可以立即按下**
- 喷枪、相机、线缆、气管**不会因为关节旋转被拉扯**

### 4.2 控制柜状态

- 控制柜**没有报警**（面板无红灯闪烁）
- 控制柜处于**允许外部 TCP 控制**的模式

### 4.3 测试约束

- 第一次只使用 **5% 速度**
- 暂时**不喷涂**、不控制喷枪输出
- 暂时**不执行**完整 A4 路径

### 4.4 末端工具注意

当前 URDF 中 `Link6` 到 `Tool_end` 的偏移为：

```text
xyz = (-0.00683, 0.150178, 0.22462) 米
```

这个偏移较大。在没有确认它与实体喷枪的工具中心点（TCP）一致之前，**不应执行完整笛卡尔轨迹**。

---

## 5. 启动驱动（不启动 MoveIt）

第一次简单运动**不需要 MoveIt**，只启动 DOBOT 驱动即可。

### 5.1 终端 1：启动 ROS Master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

保持这个终端运行，不要关闭。

**说明**：`roscore` 是 ROS 的名称服务和参数服务器，所有 ROS 节点都需要通过它进行通信。

### 5.2 终端 2：启动 CR5 驱动

打开新终端：

```bash
cd ~/cr5_ros1_ws
source devel/setup.bash
./scripts/start_driver.sh
```

**说明**：该脚本执行 `roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214`。驱动启动后会建立三条 TCP 连接到控制柜：

| 连接 | 端口 | 用途 |
|------|------|------|
| Dashboard | 29999 | 使能、禁用、速度设置、IO 控制等 |
| Motion | 30003 | 运动指令（MovJ、MovL、ServoJ 等） |
| Feedback | 30004 | 实时状态（关节角度、位姿、IO、错误码） |

驱动启动后会发布以下 ROS 话题：

```text
/joint_states                    ← 六个关节位置（弧度）
/dobot_bringup/msg/RobotStatus   ← 连接状态、使能状态
/dobot_bringup/msg/ToolVectorActual ← 末端笛卡尔位姿
/dobot_bringup/msg/FeedInfo      ← 实时反馈（含 ErrorStatus）
```

等待驱动输出提示 TCP 连接成功。如果报连接失败，返回第 3 节检查网络。

---

## 6. 只读状态检查

打开终端 3，**只读取状态，不发送任何运动命令**：

```bash
cd ~/cr5_ros1_ws
source devel/setup.bash
```

### 6.1 检查驱动节点是否存在

```bash
rosnode list
```

应能看到 `/cr5_robot` 节点。

### 6.2 检查控制柜连接状态

```bash
rostopic echo -n 1 /dobot_bringup/msg/RobotStatus
```

理想结果：

```yaml
is_enable: false
is_connected: true
```

此时 `is_enable: false` 是正常的（还没使能）。关键是 `is_connected: true`，表示驱动已成功连接到控制柜。

### 6.3 检查实时反馈

```bash
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo
```

理想数据包含：

```json
{
  "EnableStatus": 0,
  "ErrorStatus": 0,
  "RunQueuedCmd": 0,
  "QactualVec": [...]
}
```

**重点确认**：`ErrorStatus = 0`，表示控制柜没有报警。

### 6.4 查看当前关节状态

```bash
rostopic echo -n 1 /joint_states
```

**说明**：输出的关节位置单位是**弧度**（ROS 标准），不是度。保存这个输出用于后续对比：

```bash
mkdir -p ~/cr5_test_logs

rostopic echo -n 1 /joint_states \
  | tee ~/cr5_test_logs/joint_before_first_motion.txt
```

### 6.5 查看末端实际位置

```bash
rostopic echo -n 1 /dobot_bringup/msg/ToolVectorActual
```

保存：

```bash
rostopic echo -n 1 /dobot_bringup/msg/ToolVectorActual \
  | tee ~/cr5_test_logs/tool_before_first_motion.txt
```

### 6.6 检查关键服务是否注册

```bash
rosservice list | grep dobot_bringup
```

至少确认以下服务存在：

```text
/dobot_bringup/srv/ClearError         ← 清除报警
/dobot_bringup/srv/EnableRobot        ← 使能
/dobot_bringup/srv/DisableRobot       ← 下使能
/dobot_bringup/srv/SpeedFactor        ← 全局速度百分比
/dobot_bringup/srv/SpeedJ             ← 关节速度比例
/dobot_bringup/srv/AccJ               ← 关节加速度比例
/dobot_bringup/srv/RelJointMovJ       ← 相对关节运动
/dobot_bringup/srv/Sync               ← 等待运动完成
/dobot_bringup/srv/EmergencyStop      ← 紧急停止
/dobot_bringup/srv/GetAngle           ← 查询关节角度
/dobot_bringup/srv/GetPose            ← 查询笛卡尔位姿
```

此时仍然是只读操作，不会产生任何运动。

---

## 7. 低速使能机器人

确认物理环境安全（第 4 节）后，执行使能：

```bash
cd ~/cr5_ros1_ws
./scripts/enable_robot_5pct.sh
```

**说明**：该脚本依次调用三个 ROS 服务：

```text
1. ClearError    → 清除控制柜可能存在的报警
2. SpeedFactor=5 → 设置全局运动速度为 5%
3. EnableRobot   → 使能机器人（控制柜上电，关节锁定）
```

使能成功后，进一步限制关节速度和加速度：

```bash
rosservice call /dobot_bringup/srv/SpeedJ "{r: 5}"
rosservice call /dobot_bringup/srv/AccJ "{r: 5}"
```

**说明**：`SpeedJ` 控制关节运动速度比例（1-100），`AccJ` 控制关节加速度比例（1-100）。设为 5 表示只用最大值的 5%，即使运动方向错误也能及时停止。

### 7.1 确认使能状态

```bash
rostopic echo -n 1 /dobot_bringup/msg/RobotStatus
```

应看到：

```yaml
is_connected: true
is_enable: true
```

```bash
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo
```

确认：

```text
ErrorStatus: 0
```

### 7.2 关于 ClearError 返回值的已知问题

当前驱动代码中 `ClearError` 的实现会无条件将 `response.res = 0` 写回，因此**不能只看 `ClearError` 的返回结果判断报警是否真正清除**。必须以 `/dobot_bringup/msg/FeedInfo` 中的 `ErrorStatus` 字段为准。

---

## 8. 第一次运动：单关节相对移动 1 度

### 8.1 为什么使用 RelJointMovJ

第一次运动选择 `RelJointMovJ`（相对关节运动）而不是绝对角度或笛卡尔坐标，原因：

- **不需要人工填写当前绝对姿态**：相对偏移只需指定变化量
- **不容易因为单位或坐标系错误跑到很远**：偏移量有限
- **原路返回简单**：把偏移量取反即可
- **运动量可以严格控制**：只动 1 度

### 8.2 推荐测试关节

| 场景 | 推荐关节 | 说明 |
|------|----------|------|
| 末端没有易缠绕的气管线缆 | J6 | 空间扫掠较小，但会旋转末端工具 |
| 末端有喷管/相机线/气管 | J1 或 J2 | 由现场人员选择当前姿态下最安全的关节 |

**无论选哪个关节，第一次只移动 1 度。**

### 8.3 执行正向移动

手放在物理急停按钮附近，然后执行（以 J6 为例）：

```bash
rosservice call /dobot_bringup/srv/RelJointMovJ "{
  offset1: 0.0,
  offset2: 0.0,
  offset3: 0.0,
  offset4: 0.0,
  offset5: 0.0,
  offset6: 1.0,
  paramValue: []
}"
```

等待运动队列执行完成：

```bash
rosservice call /dobot_bringup/srv/Sync "{}"
```

**说明**：
- `RelJointMovJ` 发送一次性运动指令到控制柜，不会持续运动
- `Sync` 等待控制柜运动队列中的所有指令执行完毕后才返回
- `offset1` 到 `offset6` 对应 J1 到 J6 的偏移量

### 8.4 单位说明

- `RelJointMovJ` 的 `offset` 单位是**度**（控制柜角度值）
- ROS 的 `/joint_states` 输出单位是**弧度**（ROS 标准）
- 驱动在通过 `FollowJointTrajectory` Action 接收 MoveIt 轨迹时会将弧度转换为度（乘以 `180/π`），但直接调用 `RelJointMovJ` 服务时**不做转换**
- 因此 `offset6: 1.0` 就是 1 度，不要把 `/joint_states` 中的弧度值直接填到这里

### 8.5 确认运动结果

```bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo
```

确认：

```text
ErrorStatus = 0    ← 无报警
```

保存运动后状态：

```bash
rostopic echo -n 1 /joint_states \
  | tee ~/cr5_test_logs/joint_after_plus_1deg.txt
```

### 8.6 原路返回

```bash
rosservice call /dobot_bringup/srv/RelJointMovJ "{
  offset1: 0.0,
  offset2: 0.0,
  offset3: 0.0,
  offset4: 0.0,
  offset5: 0.0,
  offset6: -1.0,
  paramValue: []
}"
```

等待完成：

```bash
rosservice call /dobot_bringup/srv/Sync "{}"
```

再次检查：

```bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo
```

确认 `ErrorStatus = 0`，关节角度与运动前基本一致。

---

## 9. 运动后下使能

基础测试完成后，禁用机器人：

```bash
rosservice call /dobot_bringup/srv/DisableRobot "{}"
```

确认状态：

```bash
rostopic echo -n 1 /dobot_bringup/msg/RobotStatus
```

应回到：

```yaml
is_enable: false
is_connected: true
```

`is_connected: true` 说明驱动仍然连接，只是关节不再锁定。

---

## 10. 紧急停止方法

### 10.1 首选：物理急停

永远优先使用**控制柜或示教器上的物理急停按钮**。

### 10.2 软件紧急停止

```bash
rosservice call /dobot_bringup/srv/EmergencyStop "{}"
```

软件急停**依赖以下条件全部正常**：
- Ubuntu 系统正常运行
- ROS 正常运行
- 网线正常连接
- 控制柜 TCP 连接正常

如果以上任一环节出问题，软件急停命令无法到达控制柜。因此**软件急停不能替代物理急停**。

### 10.3 关于 MoveJog 的已知问题

**当前不要使用 `/dobot_bringup/srv/MoveJog`**。

当前代码中连续点动存在大小写不一致问题：启动点动发送 `MoveJog`，但停止函数构造的是 `moveJog`。在修复并验证前，第一次运动只使用：

```text
RelJointMovJ → Sync
```

这种一次性、有限偏移的命令。

---

## 11. 启动 MoveIt 并仅规划（不执行）

**只有在第 8 节和第 9 节全部通过后**才进入此阶段。

通过标准：

```text
网络稳定，驱动不掉线
RobotStatus 正常（is_connected: true）
ErrorStatus = 0
1 度相对运动正常
反向返回正常
DisableRobot 正常
```

### 11.1 终端 4：启动 MoveIt

```bash
cd ~/cr5_ros1_ws
source devel/setup.bash
./scripts/start_moveit.sh
```

**说明**：该脚本执行 `roslaunch dobot_moveit moveit.launch`，会根据环境变量 `DOBOT_TYPE=cr5` 路由到 `cr5_moveit_planning_execution.launch`，启动 MoveIt 规划组 `cr5_arm`，并通过 `FollowJointTrajectory` Action 连接到驱动。

### 11.2 终端 5：运行预检

```bash
cd ~/cr5_ros1_ws
source devel/setup.bash
./scripts/preflight.sh
```

**说明**：预检脚本（`quick_check.launch`）会逐项检查：

| 检查项 | 说明 |
|--------|------|
| CR5 驱动节点存在 | 确认 `cr5_robot` 节点在线 |
| MoveIt `move_group` 存在 | 确认 MoveIt 规划器就绪 |
| `robot_state_publisher` 存在 | 确认 TF 发布正常 |
| `/joint_states` 有消息 | 关节名和顺序正确 |
| 控制柜连接正常 | `is_connected: true` |
| 机器人已使能 | `is_enable: true` |
| `ErrorStatus = 0` | 无报警 |
| 关键 Service 存在 | EnableRobot、DisableRobot、ClearError 等 |
| FollowJointTrajectory Action 可用 | MoveIt 可以发送轨迹 |
| TF `base_link` → `Tool_end` 可查询 | 运动学链完整 |

### 11.3 执行小幅面规划（不执行）

```bash
cd ~/cr5_ros1_ws
./scripts/plan_small.sh
```

**说明**：该脚本设置 `execute:=false`，只规划不执行。当前小幅面配置为：

```text
幅面：100 x 30 mm
扫描线数：2 条
速度倍率：3%
加速度倍率：3%
碰撞检查：开启
路径完整度要求：99.9%
```

规划完成后可以在 RViz 中预览轨迹。确认以下内容无误后再考虑真机执行：

- 路径形状正确（蛇形光栅）
- 当前工具方向与实体喷枪一致
- 线缆和气管不会被拉扯
- 运动范围在实际安全区域内

---

## 12. 已知问题与注意事项

### 12.1 ClearError 返回值

当前 `ClearError` 实现无条件返回 `response.res = 0`，不能以此判断报警是否真正清除。**必须以 FeedInfo 中的 `ErrorStatus` 为准**。

### 12.2 MoveJog 大小写问题

启动和停止点动的命令大小写不一致，在修复前不要使用 `MoveJog`。

### 12.3 Tool_end 偏移

URDF 中 `Link6` → `Tool_end` 偏移约 225mm，需要与实体工具核实。不一致时不应执行笛卡尔轨迹。

### 12.4 单位混用

| 接口 | 单位 |
|------|------|
| `RelJointMovJ` offset | 度 |
| `/joint_states` | 弧度 |
| `MovJ` / `MovL` 位置 (x,y,z) | 毫米 |
| `MovJ` / `MovL` 姿态 (a,b,c) | 度（RPY） |

---

## 13. 建议保存的测试日志

将以下内容保存到 `~/cr5_test_logs/`，用于后续分析和问题追溯：

```bash
mkdir -p ~/cr5_test_logs
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 网络检查结果
./scripts/network_check.sh 2>&1 | tee ~/cr5_test_logs/network_${TIMESTAMP}.log

# 驱动启动日志（终端 2 的输出）
# 需要在启动时用 tee 捕获

# 运动前关节状态
rostopic echo -n 1 /joint_states | tee ~/cr5_test_logs/joints_before_${TIMESTAMP}.txt

# 运动后关节状态
rostopic echo -n 1 /joint_states | tee ~/cr5_test_logs/joints_after_${TIMESTAMP}.txt

# 运动前后 FeedInfo
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo | tee ~/cr5_test_logs/feed_${TIMESTAMP}.txt
```

---

## 14. 测试通过后的下一步

全部基础测试通过后，建议：

### 14.1 创建安全优化分支

```bash
cd ~/cr5_ros1_ws
git switch main
git pull --ff-only origin main
git switch -c feature/first-motion-safety
```

### 14.2 建议优先修改

| 优先级 | 修改项 | 说明 |
|--------|--------|------|
| 1 | 新增 `scripts/robot_status.sh` | 一次输出连接状态、使能状态、ErrorStatus、关节角和 TCP 位姿 |
| 2 | 新增 `scripts/first_joint_motion.sh` | 限制只允许 0.5/1/2 度偏移；默认 5% 速度；需要确认口令；自动 Sync；自动提示原路返回 |
| 3 | 修复 `StopmoveJog` | 统一 MoveJog 命令大小写，增加停止验证 |
| 4 | 修复 `ClearError` 返回值 | 不再无条件返回 0，保留控制柜真实返回值，结合 ErrorStatus 判断 |

### 14.3 第一轮测试建议的完整流程

```text
1. 驱动启动
2. 状态读取（确认 ErrorStatus=0）
3. 5% 使能（SpeedFactor=5, SpeedJ=5, AccJ=5）
4. 某一安全关节 +1 度（RelJointMovJ + Sync）
5. 确认 ErrorStatus=0
6. 同一关节 -1 度（原路返回 + Sync）
7. 确认 ErrorStatus=0
8. DisableRobot
```

### 14.4 暂时不要执行

- `./scripts/execute_small.sh`（真机执行 A4 演示）
- 任何 MoveIt Plan & Execute
- 任何笛卡尔空间长距离运动

直到 Tool_end 偏移与实体工具核实一致。

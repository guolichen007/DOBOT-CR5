# CR5 书本识别 Demo：开发主机与实机笔记本协同测试流程

## 1. 系统角色

### 开发主机 Ubuntu VM
负责修改 `cr5_book_spray_demo`、静态检查、提交和 Push。

开发分支：

```text
feature/book-vision-spray-demo-v1
```

### 实机 Ubuntu 笔记本
负责 Pull、编译、D455、TF、书本识别、目标锁定和 MoveIt plan-only 测试。

实机笔记本已单独配置 SSH，可自行 Push/Pull。不要复制开发主机的 RSA 或其他私钥。

## 2. 物理连接

```text
D455 --USB 3.x--> 实机 Ubuntu 笔记本
实机 Ubuntu 笔记本 --以太网--> CR5 控制柜 192.168.110.214
开发主机 Ubuntu VM --GitHub--> 代码修改与 Push
```

D455 只连接实机笔记本，不连接开发主机，也不连接 CR5 控制柜。

## 3. 版本同步原则

每次测试前，两端都记录：

```bash
git rev-parse HEAD
git branch --show-current
git remote get-url origin
```

笔记本测试 SHA 必须与 GitHub 功能分支最新 SHA 一致。

推荐流程：

```text
开发主机修改
→ 静态检查
→ commit
→ push
→ 笔记本 pull
→ catkin_make
→ D455/TF/识别测试
→ 保存结果
→ 开发主机继续修改
```

禁止两台机器同时修改同一源码。

## 4. 笔记本完整执行顺序

### 4.1 拉取最新代码

```bash
cd ~/cr5_ros1_ws
git status --short --branch
git switch feature/book-vision-spray-demo-v1
git pull --ff-only
```

### 4.2 设置 RealSense 工作空间

```bash
bash scripts/laptop/setup_realsense_ros1.sh
```

该脚本会：

1. 检查旧 RealSense 仓库的 HEAD 和版本
2. 创建 `~/realsense_ros1_ws` 独立工作空间
3. 创建软链接指向 `~/cr5_ws/src/realsense-ros`
4. 检查依赖
5. 编译工作空间

详细说明见 `docs/REALSENSE_ROS1_SETUP.md`。

### 4.3 编译 CR5 书本识别 Demo

```bash
bash scripts/laptop/pull_build_book_demo.sh
```

该脚本会：

1. 检查工作树是否干净
2. 切换到功能分支
3. 拉取最新代码
4. 加载 RealSense 和 CR5 工作空间环境
5. 执行 catkin_make
6. 验证 cr5_book_spray_demo 包可用

### 4.4 D455 预检

```bash
bash scripts/laptop/test_d455_vision_only.sh precheck
```

检查项目：

- Git 分支和 SHA
- D455 USB 设备检测
- USB 速度（需要 5000M 或更高）
- Librealsense 可用性
- ROS 包可用性
- CR5 控制柜网络（独立检查，不影响 D455 判定）

### 4.5 启动 D455 相机

```bash
bash scripts/laptop/test_d455_vision_only.sh camera
```

### 4.6 检查相机话题

在另一个终端执行：

```bash
bash scripts/laptop/test_d455_vision_only.sh topics
```

必须确认以下话题存在：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

### 4.7 启动书本视觉识别

在另一个终端执行：

```bash
bash scripts/laptop/test_d455_vision_only.sh vision
```

**重要**：使用 `start_camera:=false`，避免同一 D455 被两个进程打开。

### 4.8 视觉调试

打开调试图：

```bash
rqt_image_view /book_demo/estimator/debug_image
```

检测稳定后锁定：

```bash
rosservice call /book_demo/estimator/lock_target '{}'
rostopic echo -n 1 /book_demo/estimator/locked_pose
rostopic echo -n 1 /book_demo/estimator/locked_size
rostopic echo -n 1 /book_demo/estimator/target_locked
```

清除：

```bash
rosservice call /book_demo/estimator/clear_target '{}'
```

重复 5 次，记录中心、长宽、姿态和 `plane_rmse`。

建议门槛：

```text
中心重复性 ≤ 5 mm
姿态重复性 ≤ 1°
plane_rmse ≤ 3～5 mm
```

### 4.9 TF 检查

```bash
rosrun tf tf_echo camera_link camera_color_optical_frame
rosrun tf tf_echo base_link camera_color_optical_frame
```

正确链路：

```text
base_link → Link6 → camera_link → camera_color_optical_frame
```

如果不存在，停止目标锁定和路径规划，检查旧标定与 `robot_description`，禁止猜外参。

### 4.10 MoveIt plan-only

只有视觉与 TF 通过后执行：

```bash
bash scripts/laptop/plan_book_demo_only.sh
```

另一个终端：

```bash
rosservice call /book_demo/planner/plan_path '{}'
```

必须确认：

```text
allow_execution=false
未设置确认令牌
未调用 execute_path
路径位于书本上方
不穿过输送台
不翻腕
```

当前文档不包含实体执行命令。

## 5. 安全边界

当前阶段禁止：

- `allow_execution:=true`
- `/book_demo/planner/execute_path`
- `/book_demo/confirm_execute`
- `EnableRobot`
- 机械臂实体运动
- 气源、喷阀、喷漆

只允许：

- D455 检测
- 相机话题验证
- TF 查看
- 书本识别
- 目标锁定
- RViz 可视化
- MoveIt plan-only

## 6. 测试记录

在笔记本创建：

```text
~/cr5_test_logs/BOOK_SPRAY_D455_TEST_<日期>.md
```

记录：

- Git 分支与 SHA；
- D455 序列号、固件和 USB 速度；
- 彩色和深度帧率；
- 实际话题；
- TF 链；
- 书本实测尺寸；
- 五次锁定数据；
- `plane_rmse`；
- plan-only 结果和 Cartesian fraction；
- 错误、警告与待修改参数。

把结果交给开发主机 Claude。只提交整理后的 Markdown 和小截图，不提交 bag、视频、完整日志、模型、压缩包、build 或 devel。

## 7. 文件清单

仓库中与笔记本测试相关的文件：

```text
docs/BOOK_SPRAY_LAPTOP_REMOTE_TEST.md          # 本文档
docs/REALSENSE_ROS1_SETUP.md                    # RealSense 工作空间设置说明
docs/REALSENSE_ROS1_2_3_2_LOCAL_DIFF_AUDIT.md   # RealSense 本地修改审计
scripts/laptop/setup_realsense_ros1.sh          # RealSense 工作空间设置脚本
scripts/laptop/pull_build_book_demo.sh          # 拉取代码并编译
scripts/laptop/test_d455_vision_only.sh         # D455 视觉测试
scripts/laptop/plan_book_demo_only.sh           # MoveIt plan-only 测试
```

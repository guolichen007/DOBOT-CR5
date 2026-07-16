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

## 4. 仓库应新增的文件

```text
docs/BOOK_SPRAY_LAPTOP_REMOTE_TEST.md
scripts/laptop/pull_build_book_demo.sh
scripts/laptop/test_d455_vision_only.sh
scripts/laptop/plan_book_demo_only.sh
```

文档保存操作步骤，脚本保存可重复执行的命令。

这些脚本不得：

- 使能机械臂；
- 调用 `/book_demo/planner/execute_path`；
- 设置 `/book_demo/confirm_execute`；
- 使用 `allow_execution:=true`；
- 控制气源、喷阀或喷漆；
- 自动覆盖笔记本未提交修改。

## 5. 笔记本 Pull 与编译

```bash
cd ~/cr5_ros1_ws
bash scripts/laptop/pull_build_book_demo.sh
```

该脚本应检查工作树、切换功能分支、`git pull --ff-only`、记录 SHA、执行 `catkin_make`、执行 `rospack find` 并保存日志。

## 6. D455 纯视觉测试

D455 通过 USB 3.x 直接连接实机笔记本，不用 HUB，优先原装线。

预检：

```bash
bash scripts/laptop/test_d455_vision_only.sh precheck
```

USB 最好显示 `5000M`。如果只有 `480M`，停止后续测试并检查接口和线缆。

启动 RealSense：

```bash
bash scripts/laptop/test_d455_vision_only.sh camera
```

另一个终端检查话题：

```bash
bash scripts/laptop/test_d455_vision_only.sh topics
```

必须确认：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

启动书本识别：

```bash
bash scripts/laptop/test_d455_vision_only.sh vision
```

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

## 7. TF 检查

```bash
rosrun tf tf_echo camera_link camera_color_optical_frame
rosrun tf tf_echo base_link camera_color_optical_frame
```

正确链路：

```text
base_link → Link6 → camera_link → camera_color_optical_frame
```

如果不存在，停止目标锁定和路径规划，检查旧标定与 `robot_description`，禁止猜外参。

## 8. MoveIt plan-only

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

## 9. 测试记录

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

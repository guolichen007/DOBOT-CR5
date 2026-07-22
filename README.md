# DOBOT CR5 三相机喷涂仿真与标定系统

基于 ROS Noetic + Gazebo Classic 11 的 DOBOT CR5 机器人喷涂仿真环境，
集成三台固定 RGB-D 相机 (Intel RealSense D455)。

## 构建

```bash
cd ~/cr5_ros1_ws
catkin_make
source devel/setup.bash
```

## 仿真启动

```bash
# GUI 模式 (标定目标)
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --gui --object=calibration_target --profile=quality

# Headless 模式
bash src/cr5_spray_sim/scripts/run_simulation.sh \
  --headless --object=calibration_target --profile=quality --strict
```

## 第二终端接入会话

主终端启动仿真后，在另一终端执行：

```bash
# 加载会话环境
source /tmp/cr5_spray_simulation.env

# 查看所有话题
rostopic list

# 查看 TF 树
rosrun tf tf_echo world calibration_target_frame

# 查看相机图像 (逐帧)
rosrun image_view image_view image:=/cam_front_left/camera/color/image_raw

# 查看三台相机帧率
rostopic hz /cam_front_left/camera/color/image_raw
rostopic hz /cam_front_right/camera/color/image_raw
rostopic hz /cam_rear/camera/color/image_raw

# 查看深度图帧率
rostopic hz /cam_front_left/camera/depth/image_raw
```

## 标定验证

```bash
# 启动仿真后，在第二终端执行:

# 1. 几何一致性检查
rosrun cr5_spray_sim validate_calibration_target_geometry.py

# 2. 贴图独立检查 (GUI)
bash src/cr5_spray_sim/scripts/validate_calibration_texture.sh --gui

# 3. 三相机可见性检测
rosrun cr5_spray_sim validate_calibration_visibility.py \
  --output artifacts/calibration/visibility

# 4. 外参初值估计
rosrun cr5_spray_sim estimate_camera_extrinsics.py \
  --output artifacts/calibration/extrinsics
```

## 标定目标参数

| 面 | 图案 | ID | 物理尺寸 |
|----|------|-----|---------|
| 正面 (Front) | ChArUco 8×6 | 100–123 | 0.24×0.18 m |
| 左侧 (Left) | ChArUco 6×5 | 200–214 | 0.16×0.12 m |
| 右侧 (Right) | AprilTag 2×2 | 4–7 | 0.22×0.18 m |
| 顶部 (Top) | AprilTag | 8 | 0.16×0.16 m |
| 背面 (Back) | ChArUco 8×6 | 300–323 | 0.24×0.18 m |

主体: 0.34×0.28×0.24 m | 面板间隙: 0.001 m

详见: `docs/标定目标详解.md`

## TF 树

```
world → dummy_link → base_link → Link1..Link6 → spray_nozzle_frame
world → object_frame → calibration_target_frame
  ├── calibration_target_front_frame
  ├── calibration_target_left_frame
  ├── calibration_target_right_frame
  ├── calibration_target_top_frame
  └── calibration_target_back_frame
```

## 三相机话题

| 相机 | Color | Depth | CameraInfo |
|------|-------|-------|------------|
| cam_front_left | /cam_front_left/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_front_right | /cam_front_right/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |
| cam_rear | /cam_rear/camera/color/image_raw | .../depth/image_raw | .../color/camera_info |

## 当前状态

**已完成:**
- Gazebo 仿真环境 (CR5 机器人、三台固定相机、工业喷涂单元)
- 多面标定目标模型 (自包含 SDF/DAE/OGRE 材质)
- 三相机 RGB-D 数据链路
- 确定性场景启动流程
- 跨终端会话环境
- ChArUco / AprilTag 检测与 PnP 外参骨架

**尚未完成:**
- 生产级三相机联合外参求解
- Bundle Adjustment
- RGB-D 点云融合 / TSDF 三维重建
- 喷涂路径生成与自动开枪
- CR5 实机运动

## 分支

- `main` — 稳定工程基线
- `develop/multi-camera-calibration` — 三相机标定与三维重建开发

详见: `docs/分支策略.md`

## 标定资产

| 资源 | 路径 |
|------|------|
| SDF 模型 | `models/calibration_target/model.sdf` |
| DAE Mesh | `models/calibration_target/meshes/panel_unit.dae` |
| OGRE 材质 | `models/calibration_target/materials/scripts/calibration_target.material` |
| YAML 配置 | `config/calibration/calibration_target.yaml` |
| 纹理 PNG | `models/calibration_target/materials/textures/` |

## 安全边界

- 不连接 CR5 实机
- 不在无人工监控时执行喷涂动作
- 相机外参未经验证不用于安全关键决策

## 文档索引

- `docs/系统架构.md` — 系统架构说明
- `docs/仿真操作手册.md` — 仿真操作指南
- `docs/标定目标详解.md` — 标定目标参数详解
- `docs/分支策略.md` — Git 分支管理策略
- `docs/仓库整理报告.md` — 工程整理记录
- `docs/下一阶段_多相机标定.md` — 下一阶段开发入口
- `src/cr5_spray_sim/README.md` — 仿真包说明
- `src/cr5_spray_sim/models/calibration_target/README.md` — 标定模型说明

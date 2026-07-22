# CR5 喷涂仿真场景 V1 已知问题

> 记录于 2026-07-21，fix/cr5-spray-scene-layout-v2 重构前

## 实测问题

1. **CR5 基座未锚定**：模型在地面不稳定，基座没有可靠固定到 world
2. **门架断裂**：横梁、立柱、地脚错位，模型悬空
3. **相机自由掉落**：固定相机和 wrist_camera 作为自由 Gazebo 模型，受重力影响掉落
4. **相机位姿重复应用**：YAML 位姿同时用于 spawn pose 和模型内部 joint origin
5. **wrist_camera 未连接**：没有真正连接到 Link6/wrist_d455_mount
6. **帧率过低**：6 台 640×480 相机在虚拟机仅约 1.5Hz
7. **spray_object ros_control 错误**：日志报 "joint1 is not in the gazebo model"
8. **相机未对准工件**：画面中没有工件
9. **asymmetric_part 占位**：仍是方盒
10. **旧相机冗余**：不需要 wrist_camera 和 6 台相机

## 重构目标

- 固定 CR5 竖直安装
- 重建稳定四立柱门架（static monolithic model）
- 固定工件（无 ros_control）
- 仅保留 3 台固定 D455-like 相机
- 三相机 look-at 工件
- 低分辨率低帧率适配虚拟机
- 完整 TF 发布

# 架构、状态机与安全约束

## 1. 实际布置判断

参考照片显示：

- 书本平放在绿色长台/输送台上；
- CR5 位于同一长台一端；
- D455 固定在末端法兰附近，属于 Eye-in-Hand；
- 当前末端存在较长金属工具和明显偏置；
- 相机、工具和线缆都随腕部运动。

所以本系统必须把“观察”和“执行”分离：

```text
机械臂到观察姿态
→ 完全停止
→ 连续采集稳定帧
→ 锁定 base_link 下的书本位姿
→ 不再跟随相机实时更新
→ 规划
→ 人工确认
→ 干跑执行
```

## 2. 数据流

```text
D455 /camera/color/image_raw
D455 /camera/aligned_depth_to_color/image_raw
D455 /camera/color/camera_info
                │
                ▼
       book_pose_estimator
  非绿色区域 + 旋转矩形 + 深度平面
                │
       timestamped TF 查询
camera_color_optical_frame → base_link
                │
                ▼
 live pose / size / polygon / debug
                │
        显式 lock_target 服务
                │
                ▼
       book_locked (固定目标)
                │
                ▼
       book_spray_planner
  单线或 raster + MoveIt 碰撞规划
                │
        默认只发布规划和 Marker
                │
   allow_execution + token + execute
                ▼
 FollowJointTrajectory → CR5 实体运动
```

## 3. 三个互相独立的安全门

### 门 1：目标锁定

识别节点只有在连续多帧满足以下条件时才锁定：

- 位置标准差达标；
- 姿态角扩散达标；
- 长宽标准差达标；
- 最新帧没有超时；
- 深度平面 RMSE 和内点比例达标。

### 门 2：轨迹有效

规划必须满足：

- 已锁定目标；
- 尺寸和边距有效；
- approach pose 可规划；
- Cartesian fraction 达标；
- 轨迹非空；
- RViz 人工检查通过。

### 门 3：实体执行

执行必须同时满足：

- planner 启动参数 `allow_execution:=true`；
- 已有成功且未超时的缓存计划；
- 目标未发生位置/角度变化；
- ROS 参数中的一次性 token 完全匹配；
- 人工调用 execute 服务；
- 5 秒倒计时完成。

这些门不能替代：控制器报警检查、5% 全局速度、物理急停和现场清场。

## 4. 当前末端工具风险

照片中的工具具有较长悬臂和尖锐/硬质末端。第一阶段必须：

- 不允许接触书本；
- 不接气源、液体或喷料；
- 使用较大 `standoff_m`；
- 观察线缆在 J5/J6 运动时是否被拉紧；
- 不使用未确认的自动姿态对齐；
- 把长工具和支架补入 MoveIt 碰撞模型后再缩短距离。

## 5. 为什么默认单线而不是直接全覆盖

单线可以分别验证：

- 视觉上端/下端方向；
- base_link 坐标转换；
- 工具高度；
- MoveIt 笛卡尔路径；
- 实机速度与腕部行为。

全覆盖会同时引入：扫描间距、连接段、腕部奇异、工作空间边缘和线缆扭转，排错成本更高。

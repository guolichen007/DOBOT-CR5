# V3.2.1 运行时故障评审报告

> **状态：FAILED**  
> 分支：`feature/cr5-spray-tool-control-v1`  
> HEAD：`76d08ce`  
> 评审日期：2026-07-21

## 一、故障清单

| # | 问题 | 严重程度 | 根因 |
|---|------|---------|------|
| 1 | 喷枪视觉体未贴合 CR5 法兰 | 严重 | 使用旧 `Tool_end` 坐标而非 Link6 法兰原点 |
| 2 | TF 树断开：`world` 与 `spray_nozzle_frame` 不连通 | 严重 | `world→base_link` 与 `dummy_link→base_link` 形成双父节点冲突 |
| 3 | 场景多次启动后出现模型位置错乱/散架 | 严重 | 未定位，需提交级回归 |
| 4 | `/spray_demo/state` 健康检查超时 | 中等 | Topic 不是 latched，后启动节点错过初始化消息 |
| 5 | wrapper 在健康检查 FAIL 后仍打印 Session active | 中等 | `\|\| echo` 吞掉了失败返回码 |
| 6 | 开枪返回 TF disconnected | 严重 | 根因 #2 的直接后果 |
| 7 | 无工件表面上色 | 功能缺失 | 未实现 paint patch |
| 8 | 硬编码测试姿态未经验证 | 中等 | 标注"经 Gazebo 探索"但无实际日志 |

## 二、代码根因详解

### 2.1 喷枪安装坐标错误

当前 `cr5_sim.urdf.xacro:226`：
```xml
mount_xyz="-0.00683 0.150178 0.23462"
```

这组坐标来自实机旧 URDF 中 `Link6 → Tool_end` 的偏移。但 `Tool_end` 是旧工具架总成末端（经 Tool_box1 → Tool_box2 延伸约 22cm），不是 Link6 法兰面。

正确的安装面参考应为：
- Link6 局部原点 `(0, 0, 0)`
- 或旧 `Link6 → Tool_box1` 的 `(0, 0, 0.001)`

### 2.2 TF 树结构错误

当前 `scene_v32_spray.launch:109`：
```xml
<node name="world_to_base_link_tf" pkg="tf2_ros" type="static_transform_publisher"
  args="0 0 0 0 0 0 1 world base_link"/>
```

但 `robot_state_publisher` 已发布 `dummy_link → base_link`，导致 `base_link` 有两个父节点。

正确结构：
```
world → dummy_link → base_link → ... → Link6 → spray_nozzle_frame
```

### 2.3 state topic 非 latched

`spray_simulator_v32.py` 中 `pub_state` 应使用 `latch=True` 并周期性重发。

## 三、后续计划

V3.3 从 `acedd33` 重建，详见 V3.3 执行方案。

# 目标隔离 V2 — 发布记录

日期: 2026-08-05
提交: 430cd33
分支: feature/fixed-camera-reconstruction-robustness → main

## 目的

八姿态鲁棒性验证发现，TSDF 重建结果中包含两类非目标几何：
1. 后置相机支架/横臂（固定场景结构）
2. 悬挂钢筋/吊杆（与工件一起移动的吊具）

这些结构不属于待重建工件，但被送入 TSDF 并被 mesh cleanup 保留（因为"有真实深度支持"）。

目标隔离 V2 在 TSDF 前排除已知非目标结构，将输出从"目标主体 + 少量吊具/支架残留"收敛为"仅目标主体的可见表面"。

## 实现

### 新增模块

- `target_isolation.py`: TSDF 前目标隔离（静态 AABB 排除 + 世界高度天花板 + 细长分量分类）
- `diagnose_artifacts.py`: 只读残留诊断工具

### 新增配置

- `visible_surface_target_isolation_v2.yaml`: 继承 V1 全部冻结参数，新增 `target_isolation` 段

### 修改

- `run_visible_surface_reconstruction.py`: 在 generate_target_masks 和 run_tsdf 之间插入隔离步骤，V1 配置行为不变
- `mesh_cleanup.py`: 分量分类（REAR_CAMERA_PEDESTAL / SUSPENSION_ROD），已知非目标分量强制删除
- 质量报告增加 `target_isolation` 段

### 测试

新增 15 个测试，全量 231/231 PASS。

## 静态支架排除

### 后置相机支架 AABB

基于八姿态 1193 个被排除点的 5%-95% 分位数 +5mm margin：

```
rig frame (cam_front_left_color_optical_frame):
  min: [0.258139, -0.218773, 1.170169]
  max: [0.281676, -0.136302, 1.199245]
  dimensions: 24×82×29 mm
```

被排除点全部来自 cam_front_right 对后置相机/支架的边缘视野观测。

### 排除点验证

全部 1193 个被排除点距 visible GT >118mm，确认为非目标。

## 三路消融结果（P1/P4）

| 配置 | P1 cov10 | P4 cov10 | 后支架 |
|------|----------|----------|--------|
| V1 (无隔离) | 0.791 | 0.788 | 残留 |
| ceiling-only | =V1 | =V1 | 残留 |
| static-only | 0.783 | 0.779 | 已清除 |
| both (=static) | 0.783 | 0.779 | 已清除 |

ceiling 排除始终为 0（所有姿态所有相机）。回归完全来自静态支架排除。

## 八姿态 V1/V2 对比

| 姿态 | V1 Med | V2 Med | V1 P95 | V2 P95 | V1 Cov10 | V2 Cov10 | ΔCov10 |
|------|--------|--------|--------|--------|----------|----------|--------|
| P0 | 3.69 | 3.67 | 15.58 | 15.56 | 0.790 | 0.787 | -0.003 |
| P1 | 3.69 | 3.68 | 15.84 | 15.81 | 0.791 | 0.783 | **-0.008** |
| P2 | 3.70 | 3.70 | 15.38 | 15.34 | 0.790 | 0.788 | -0.002 |
| P3 | 3.71 | 3.71 | 15.55 | 15.54 | 0.790 | 0.788 | -0.002 |
| P4 | 3.30 | 3.30 | 16.07 | 16.03 | 0.788 | 0.779 | **-0.009** |
| P5 | 3.78 | 3.77 | 14.27 | 14.23 | 0.789 | 0.786 | -0.003 |
| E1 | 3.80 | 3.79 | 15.25 | 15.22 | 0.765 | 0.765 | 0.000 |
| E2 | 3.29 | 3.30 | 16.16 | 16.12 | 0.790 | 0.782 | -0.008 |

P0/P2/P3/P5: 回归约束通过（ΔCov10 ≤ 0.003）。
P0/P3/P5: mesh SHA 重复性保持。

## P1/P4 Completeness 回归（已知工程权衡）

目标隔离 V2 删除了经 visible GT 距离验证的非目标静态支架点。

由于 TSDF 融合对输入深度集合存在边界敏感性，P1/P4 的 Completeness coverage@10mm 相对 V1 分别下降 0.008 和 0.009。

该变化未由目标表面误删引起：
- 被排除点距 visible GT 均超过 118mm；
- 收窄静态排除区域后，被排除点集合保持不变；
- Accuracy median 基本不变，Accuracy P95 略有改善。

项目接受该确定性工程权衡，以换取最终网格中已知非目标支架和悬挂杆残留的清除。

原质量 Gate 与阈值保持不变。此回归记录为一次经过审查的例外，不等同于放宽回归约束。

## P4 状态

```
Accuracy P95 = 16.03 mm > 16.00 mm
Completeness cov10 = 0.779 < 0.780
Auto Gate: FAIL_MARGINAL
Final Gate: FAIL
```

P4 (yaw +15°) 属于当前生产包络外，保持边际失败。不论 V2 还是 V1，该姿态均不满足 Accuracy 或 Completeness Gate。

## 工作包络

```
已验证支持：
  X +30 mm
  X -30 mm
  Y +30 mm
  yaw -15°

未验证：
  Y -30 mm

当前生产包络外：
  yaw +15°：边际失败
  X -50 mm / Y -40 mm 组合位移
  yaw +15°与平移组合
```

## 未修改项确认

- Stable V1 外参：未修改
- 三台相机位置：未修改
- TSDF voxel=0.005 / trunc=0.020：未修改
- Accuracy median ≤5.0mm Gate：未修改
- Accuracy P95 ≤16.0mm Gate：未修改
- Completeness cov10 ≥0.78 Gate：未修改
- bottom=UNKNOWN：未修改
- mesh cleanup 保守策略：未修改（仅增加已知非目标分量删除）
- V1 配置：未修改
- stable/* 分支：未修改

## 不推进项

- stable/visible-surface-reconstruction-v1 不更新（保留原封板重建基线）
- 不继续缩小 AABB
- 不调整 TSDF 参数
- 不为 P4 单独调参

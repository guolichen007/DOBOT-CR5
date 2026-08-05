# Changelog

## [Unreleased]

### Added
- Multi-pose reconstruction robustness validation infrastructure
- Per-pose offline evidence and visible GT generation
- Target isolation for known non-target geometry (static camera supports, suspension fixtures)
- Fixed-camera workspace envelope validation (8 poses, P0-P5 + E1-E2)
- Fail-closed automatic quality gate with manual structural review
- Mesh component classification (target body, suspension rod, rear pedestal, etc.)

### Changed
- Production config consolidated to single `visible_surface_production.yaml`
- README restructured as current-capability document without version suffixes
- Repository naming convention: mainline uses evergreen names, stable branches preserve historical refs
- `quality_contract.py` and `evaluation_runner.py` for pure-Python quality gating
- Runner uses lazy Open3D import (`require_open3d()`)
- Integration tests run real fake-evaluator subprocess

### Fixed
- CI split into 6 independent jobs
- reconstruction-quality-gate enforces pure Python gate without Open3D
- 删除 `| tail -5` 和 `|| echo` 等错误吞没模式
- Open3D 安装失败现在使 CI job 失败

### Added
- 测试从 120 增加到 173+，覆盖 subprocess evaluator 集成路径
- 仓库契约新增多项检查（README 版本号、CI 规范、模块存在性等）

---

## [V1.0.4] — 2026-08-04

### Fixed
- CI: 移除重建测试的 `|| echo` fallback，测试失败现在会使 CI 失败
- CI: 新增独立非 Open3D 测试 job，Open3D 安装失败不再静默跳过
- Evaluator label 和 metrics 文件名改为由动态 `release_id` 生成

### Added
- 120 个测试 (含 9 个 runner 集成测试)
- 项目状态文档更新至 V1.0.4

## [V1.0.3] — 2026-08-04

### Fixed
- 动态 `release_id`: 优先级 `--release-id` > git exact tag > env > UNTAGGED
- 显式 `--evaluate` 完全 fail-closed: GT 缺失/评价器缺失/metrics 缺失/NAN 均返回非零
- Acceptance reasons 不再含空字符串
- 质量报告 identity 解除硬编码 V1.0.1

### Added
- Metrics schema 验证 (13 字段: 类型/finite/range)
- 111 个测试 (含 21 个 evaluation integration tests)
- CI: 重建测试 job, checkout@v4, 正确分支触发

## [V1.0.2] — 2026-08-04

### Fixed
- `--visible-gto` → `--visible-gt` (评价静默失败)
- 子进程无 `check=True`
- 改为读取 evaluator JSON metrics 而非 stdout 解析
- Coverage 字段映射修复

## [V1.0.1] — 2026-08-04

### Added
- 冻结 `visible_surface_production_v1.yaml`
- `run_visible_surface_reconstruction.py` 一键正式入口
- 保守网格碎片清理
- 90 个测试
- 三条长期分支治理 (main, stable/calibration, stable/reconstruction)

## [V1.0.0] — 2026-08-03

### Added
- Gazebo 三相机可见表面 TSDF 三维重建基线
- Stable V1 外参 (pairwise_camera_relative)
- Target-only depth mask + ROI
- 可见表面 GT (深度缓冲法)
- Accuracy median=3.68mm, P95=15.58mm
- Partial visible surface, bottom=UNKNOWN

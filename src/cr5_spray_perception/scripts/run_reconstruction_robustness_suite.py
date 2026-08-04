#!/usr/bin/env python3
"""
多姿态重建鲁棒性验证套件 (V2 — fail-closed Gate).

对多个目标姿态分别调用正式重建入口, 汇总验证:
  - Accuracy median ≤ 5mm, P95 ≤ 16mm
  - Visible completeness coverage@10mm ≥ 0.80
  - removed_fragment_area_ratio ≤ 0.03
  - bottom == UNKNOWN
  - 重复性 (同一姿态多次运行的 mesh SHA256 一致)
  - 所有 run 必须成功且通过 Gate
  - 结构项标记 MANUAL_REVIEW_REQUIRED

不复制重建算法 — 只通过 subprocess 调用 run_visible_surface_reconstruction.py.

Usage:
  rosrun cr5_spray_perception run_reconstruction_robustness_suite.py \
    --dataset <multi_pose_dataset> \
    --poses <robustness_poses.yaml> \
    --rig <stable_rig.yaml> \
    --config <production_config.yaml> \
    --output <results_dir>
"""
import os, sys, json, yaml, csv, hashlib, logging, argparse, subprocess, datetime

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("robustness")

WS = os.path.join(os.path.dirname(__file__), "..")
_RUNNER = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")

# ── Gate 阈值 ──
GATE_ACC_MEDIAN = 5.0
GATE_ACC_P95 = 16.0
GATE_COMPLETENESS_COV10 = 0.80
GATE_REMOVED_RATIO = 0.03


def sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_poses(poses_yaml):
    with open(poses_yaml) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("poses", []), cfg.get("acceptance", {})


def run_reconstruction(dataset_path, rig_path, config_path, output_dir,
                       visible_gt=None, release_id=None, model_pose_json=None):
    """调用正式重建入口 (fail-closed). 返回 (success, quality_report_path)."""
    os.makedirs(output_dir, exist_ok=True)
    cmd = [
        sys.executable, _RUNNER,
        "--dataset", dataset_path,
        "--group-id", "0",
        "--rig", rig_path,
        "--config", config_path,
        "--output", output_dir,
    ]
    if release_id:
        cmd += ["--release-id", release_id]
    if visible_gt and os.path.isfile(visible_gt):
        cmd += ["--evaluate", "--visible-gt", visible_gt]
    if model_pose_json and os.path.isfile(model_pose_json):
        cmd += ["--model-pose-json", model_pose_json]

    logger.debug("  runner cmd: %s", " ".join(cmd))

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        logger.error("  TIMEOUT after 600s")
        return False, None

    if result.returncode != 0:
        logger.error("  FAIL (exit=%d)", result.returncode)
        if result.stderr:
            for line in result.stderr.strip().split("\n")[-5:]:
                logger.error("    stderr: %s", line)
        return False, None

    qpath = os.path.join(output_dir, "reconstruction_quality_report.json")
    if not os.path.isfile(qpath):
        logger.error("  quality_report.json not found after run")
        return False, None

    return True, qpath


def load_quality_report(qpath):
    with open(qpath) as f:
        return json.load(f)


def evaluate_pose_gate(report, n_runs_completed, n_runs_expected,
                        mesh_sha_set, all_runs_pass):
    """对单个姿态的所有 run 执行综合 Gate 判断。

    Returns: (pose_result_dict, auto_gate_pass, failures_list)
    """
    failures = []
    manual_review_items = []

    acc = report.get("accuracy", {})
    acc_med = acc.get("median_mm")
    acc_p95 = acc.get("p95_mm")
    acc_rmse = acc.get("rmse_mm")
    acc_cov10 = acc.get("coverage_10mm")

    comp = report.get("completeness", {})
    comp_med = comp.get("median_mm")
    comp_p95 = comp.get("p95_mm")
    comp_cov10 = comp.get("coverage_10mm")

    quality = report.get("quality", {})
    chamfer = quality.get("chamfer_mm")

    acceptance = report.get("acceptance", {})
    prod_pass = acceptance.get("production_pass")

    mesh = report.get("mesh", {})
    removed_ratio = mesh.get("removed_fragment_area_ratio", 0)
    raw_components = mesh.get("raw_components", 0)
    cleaned_components = mesh.get("cleaned_components", 0)

    unobs = report.get("unobserved", {})
    bottom_status = unobs.get("bottom_surface", "")

    # ── 自动 Gate ──
    # 运行完整性
    if n_runs_completed != n_runs_expected:
        failures.append(f"runs_completed={n_runs_completed} != expected={n_runs_expected}")

    if not all_runs_pass:
        failures.append("not all runs passed")

    # mesh SHA 唯一性 (仅 repeat>1 时)
    if n_runs_expected > 1:
        if len(mesh_sha_set) == 0:
            failures.append("no mesh SHA collected")
        elif len(mesh_sha_set) != n_runs_expected or len(set(mesh_sha_set)) != 1:
            failures.append(f"mesh SHA not unique: {len(set(mesh_sha_set))}/{n_runs_expected}")

    # 生产 Gate
    if prod_pass is not True:
        failures.append("production_pass is not True")

    # Accuracy
    if acc_med is None:
        failures.append("accuracy median missing")
    elif acc_med > GATE_ACC_MEDIAN:
        failures.append(f"accuracy median={acc_med:.2f} > {GATE_ACC_MEDIAN}")

    if acc_p95 is None:
        failures.append("accuracy P95 missing")
    elif acc_p95 > GATE_ACC_P95:
        failures.append(f"accuracy P95={acc_p95:.2f} > {GATE_ACC_P95}")

    # Completeness coverage
    if comp_cov10 is not None and comp_cov10 < GATE_COMPLETENESS_COV10:
        failures.append(f"completeness coverage@10mm={comp_cov10:.3f} < {GATE_COMPLETENESS_COV10}")

    # Mesh cleanup
    if removed_ratio is not None and removed_ratio > GATE_REMOVED_RATIO:
        failures.append(f"removed_fragment_area_ratio={removed_ratio:.3f} > {GATE_REMOVED_RATIO}")

    # Bottom
    if bottom_status != "UNKNOWN":
        failures.append(f"bottom_surface={bottom_status} != UNKNOWN")

    # ── 结构项 (无法自动判断 → MANUAL_REVIEW_REQUIRED) ──
    manual_review_items = [
        "no_mirror",
        "no_double_surface",
        "top_offset_block_preserved",
        "no_global_layer_shift",
        "no_gantry_floor_residual",
    ]

    # ── 汇总 ──
    auto_pass = len(failures) == 0

    if auto_pass:
        status = "AUTO_PASS"
    else:
        status = "AUTO_FAIL"

    result = {
        "runs_completed": n_runs_completed,
        "runs_expected": n_runs_expected,
        "all_runs_pass": all_runs_pass,
        "accuracy_median_mm": acc_med,
        "accuracy_p95_mm": acc_p95,
        "accuracy_rmse_mm": acc_rmse,
        "accuracy_coverage_10mm": acc_cov10,
        "completeness_median_mm": comp_med,
        "completeness_p95_mm": comp_p95,
        "completeness_coverage_10mm": comp_cov10,
        "chamfer_mm": chamfer,
        "production_pass": prod_pass,
        "removed_fragment_area_ratio": removed_ratio,
        "raw_components": raw_components,
        "cleaned_components": cleaned_components,
        "bottom_status": bottom_status,
        "mesh_hashes": sorted(list(set(mesh_sha_set))) if mesh_sha_set else [],
        "mesh_sha_count": len(mesh_sha_set) if mesh_sha_set else 0,
        "status": status,
        "auto_gate_pass": auto_pass,
        "auto_failures": failures,
        "manual_review_required": manual_review_items,
    }
    return result, auto_pass, failures


def run_robustness_suite(dataset_root, poses_yaml, rig_path, config_path, output_root):
    """主入口: 运行多姿态验证套件 (V2 hard Gate)."""
    poses, acceptance_cfg = load_poses(poses_yaml)
    os.makedirs(output_root, exist_ok=True)

    logger.info("多姿态鲁棒性验证: %d poses", len(poses))
    logger.info("  rig: %s", rig_path)
    logger.info("  config: %s", config_path)
    logger.info("  dataset: %s", dataset_root)

    per_pose = []
    ts_start = datetime.datetime.utcnow()

    for pose in poses:
        pose_id = pose["id"]
        preset = pose["preset"]
        category = pose.get("category", "production")
        repeat = pose.get("repeat", 1)
        desc = pose.get("description", "")

        logger.info("── Pose %s (%s, repeat=%d) ──", pose_id, preset, repeat)
        logger.info("    %s", desc)

        pose_dir = os.path.join(dataset_root, f"pose_{pose_id.lower()}")
        if not os.path.isdir(pose_dir):
            logger.warning("    dataset dir not found: %s, SKIP", pose_dir)
            per_pose.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "SKIP", "reason": "dataset_missing",
            })
            continue

        # per-pose visible GT
        per_pose_gt = os.path.join(pose_dir, "visible_gt", "visible_union.ply")
        if category == "production" and not os.path.isfile(per_pose_gt):
            logger.error("    per-pose visible GT missing: %s", per_pose_gt)
            per_pose.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "FAIL", "reason": "visible_gt_missing",
            })
            continue

        visible_gt = per_pose_gt if os.path.isfile(per_pose_gt) else None

        run_hashes = []
        run_reports = []
        run_all_pass = True

        for run_i in range(repeat):
            run_label = f"{pose_id}_r{run_i}" if repeat > 1 else pose_id
            run_output = os.path.join(output_root, f"run_{pose_id.lower()}_r{run_i}")
            release_id = f"robustness-{pose_id.lower()}"

            # pose evidence JSON for offline GT evaluation
            pose_evidence = os.path.join(pose_dir, "pose_evidence.json")

            success, qpath = run_reconstruction(
                dataset_path=pose_dir,
                rig_path=rig_path,
                config_path=config_path,
                output_dir=run_output,
                visible_gt=visible_gt,
                release_id=release_id,
                model_pose_json=pose_evidence,
            )

            if not success:
                run_all_pass = False
                logger.warning("    run %d/%d FAILED", run_i + 1, repeat)
                continue

            report = load_quality_report(qpath)
            acceptance = report.get("acceptance", {})
            if acceptance.get("production_pass") is not True:
                run_all_pass = False
                logger.warning("    run %d/%d Gate FAIL", run_i + 1, repeat)

            run_reports.append(report)

            mesh_path = os.path.join(run_output, "visible_surface_mesh_final.ply")
            mesh_sha = sha256_file(mesh_path)
            if mesh_sha:
                run_hashes.append(mesh_sha)

        # ── 汇总单个姿态 ──
        if not run_reports:
            per_pose.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "FAIL", "reason": "all_runs_failed",
                "runs_completed": 0, "runs_expected": repeat,
            })
            continue

        # 使用最差 run 的指标 (max median, max P95)
        worst = None
        for r in run_reports:
            acc = r.get("accuracy", {})
            med = acc.get("median_mm")
            p95 = acc.get("p95_mm")
            if med is not None and p95 is not None:
                if worst is None or med > worst[0] or (med == worst[0] and p95 > worst[1]):
                    worst = (med, p95)

        # 用最差指标构建用于 Gate 判断的报告
        worst_report = run_reports[0].copy()
        if worst:
            worst_report.setdefault("accuracy", {})["median_mm"] = worst[0]
            worst_report.setdefault("accuracy", {})["p95_mm"] = worst[1]

        mesh_sha_set = set(run_hashes) if run_hashes else set()

        gate_result, auto_pass, failures = evaluate_pose_gate(
            report=worst_report,
            n_runs_completed=len(run_reports),
            n_runs_expected=repeat,
            mesh_sha_set=mesh_sha_set,
            all_runs_pass=run_all_pass,
        )

        gate_result["pose_id"] = pose_id
        gate_result["preset"] = preset
        gate_result["category"] = category
        gate_result["description"] = desc

        per_pose.append(gate_result)

        icon = "✓" if auto_pass else "✗"
        logger.info("    => %s  auto=%s  acc_med=%.2f  acc_p95=%.2f  runs=%d/%d",
                    icon, "PASS" if auto_pass else "FAIL",
                    gate_result.get("accuracy_median_mm") or 0,
                    gate_result.get("accuracy_p95_mm") or 0,
                    gate_result["runs_completed"], gate_result["runs_expected"])
        if failures:
            for f in failures:
                logger.info("       FAIL: %s", f)

    # ── 汇总报告 ──
    ts_end = datetime.datetime.utcnow()
    _write_robustness_report(output_root, per_pose, poses_yaml, rig_path, config_path,
                              ts_start, ts_end, acceptance_cfg)
    _write_summary_md(output_root, per_pose)
    _write_per_pose_csv(output_root, per_pose)

    # ── 最终 Gate: 所有 production 姿态必须 AUTO_PASS ──
    production_results = [r for r in per_pose if r["category"] == "production"]
    if not production_results:
        logger.error("PRODUCTION GATE FAIL: no production poses evaluated")
        return 1

    auto_fails = [r for r in production_results
                  if r.get("auto_gate_pass") is not True]
    manual_pending = [r for r in production_results
                      if r.get("manual_review_required")]

    if auto_fails:
        logger.error("PRODUCTION GATE FAIL: %d/%d poses failed auto Gate",
                     len(auto_fails), len(production_results))
        for r in auto_fails:
            logger.error("  %s: %s", r["pose_id"], r.get("auto_failures", []))
        return 1

    logger.info("AUTO GATE: all %d production poses pass", len(production_results))
    if manual_pending:
        logger.info("MANUAL_REVIEW_PENDING: %d poses require structural review",
                    len(manual_pending))
        logger.info("  items: %s", manual_pending[0].get("manual_review_required", []))

    logger.info("FINAL: AUTO_GATE_PASS (manual structural review pending)")
    return 0


def _write_robustness_report(output_root, per_pose, poses_yaml, rig_path, config_path,
                              ts_start, ts_end, acceptance_cfg):
    production = [r for r in per_pose if r["category"] == "production"]
    boundary = [r for r in per_pose if r["category"] == "boundary"]

    # 最差值 (仅已评估的)
    evaluated = [r for r in production
                 if r.get("accuracy_median_mm") is not None]

    worst_med = None
    worst_med_pose = None
    worst_p95 = None
    worst_p95_pose = None
    worst_comp = None
    worst_comp_pose = None

    for r in evaluated:
        m = r["accuracy_median_mm"]
        if worst_med is None or m > worst_med:
            worst_med = m
            worst_med_pose = r["pose_id"]
        p = r["accuracy_p95_mm"]
        if worst_p95 is None or p > worst_p95:
            worst_p95 = p
            worst_p95_pose = r["pose_id"]
        c = r.get("completeness_coverage_10mm")
        if c is not None and (worst_comp is None or c < worst_comp):
            worst_comp = c
            worst_comp_pose = r["pose_id"]

    auto_pass = all(r.get("auto_gate_pass") is True for r in production)
    report = {
        "schema": "cr5_robustness_report_v2",
        "timestamp": ts_start.isoformat() + "Z",
        "duration_s": (ts_end - ts_start).total_seconds(),
        "input": {
            "poses_yaml": os.path.abspath(poses_yaml),
            "rig_path": os.path.abspath(rig_path),
            "config_path": os.path.abspath(config_path),
        },
        "acceptance_criteria": {
            "accuracy_median_mm": GATE_ACC_MEDIAN,
            "accuracy_p95_mm": GATE_ACC_P95,
            "completeness_coverage_10mm": GATE_COMPLETENESS_COV10,
            "removed_fragment_area_ratio": GATE_REMOVED_RATIO,
        },
        "summary": {
            "total_poses": len(per_pose),
            "production_poses": len(production),
            "boundary_poses": len(boundary),
            "pass_count": len([r for r in per_pose if r.get("auto_gate_pass") is True]),
            "fail_count": len([r for r in per_pose if r.get("auto_gate_pass") is False]),
            "skip_count": len([r for r in per_pose if r["status"] == "SKIP"]),
            "production_pass": len([r for r in production if r.get("auto_gate_pass") is True]),
            "production_fail": len([r for r in production if r.get("auto_gate_pass") is False]),
            "auto_gate": "PASS" if auto_pass else "FAIL",
            "worst_accuracy_median_mm": worst_med,
            "worst_accuracy_median_pose": worst_med_pose,
            "worst_accuracy_p95_mm": worst_p95,
            "worst_accuracy_p95_pose": worst_p95_pose,
            "worst_completeness_coverage_10mm": worst_comp,
            "worst_completeness_pose": worst_comp_pose,
            "repeatability_failures": len([r for r in per_pose
                                            if r.get("mesh_sha_count", 0) > 0
                                            and len(r.get("mesh_hashes", [])) != 1]),
            "manual_review_pending_count": len([r for r in production
                                                 if r.get("manual_review_required")]),
        },
        "per_pose": per_pose,
    }
    path = os.path.join(output_root, "robustness_report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    logger.info("报告: %s", path)


def _write_summary_md(output_root, per_pose):
    production = [r for r in per_pose if r["category"] == "production"]
    boundary = [r for r in per_pose if r["category"] == "boundary"]
    prod_auto_pass = all(r.get("auto_gate_pass") is True for r in production)

    lines = [
        "# 多姿态重建鲁棒性验证报告",
        "",
        f"## 自动 Gate: {'PASS' if prod_auto_pass else 'FAIL'}",
        "",
        "| 姿态 | 类别 | Auto | Acc Med | Acc P95 | Comp Cov10 | Removed | Bottom | Runs | SHA |",
        "|------|------|------|---------|---------|------------|---------|--------|------|-----|",
    ]

    for r in per_pose:
        med = f"{r['accuracy_median_mm']:.2f}" if r.get("accuracy_median_mm") is not None else "N/A"
        p95 = f"{r['accuracy_p95_mm']:.2f}" if r.get("accuracy_p95_mm") is not None else "N/A"
        cov10 = f"{r['completeness_coverage_10mm']:.3f}" if r.get("completeness_coverage_10mm") is not None else "N/A"
        removed = f"{r['removed_fragment_area_ratio']:.3f}" if r.get("removed_fragment_area_ratio") is not None else "N/A"
        bottom = r.get("bottom_status", "N/A")
        runs = f"{r.get('runs_completed',0)}/{r.get('runs_expected',0)}"
        sha_ok = "OK" if r.get("mesh_sha_count", 0) == r.get("runs_expected", 0) and len(r.get("mesh_hashes", [])) == 1 else (
            "N/A" if r.get("runs_expected", 1) == 1 else "FAIL")
        auto = "PASS" if r.get("auto_gate_pass") else "FAIL"
        lines.append(f"| {r['pose_id']} | {r['category']} | {auto} | {med} | {p95} | {cov10} | {removed} | {bottom} | {runs} | {sha_ok} |")

    fail_details = []
    for r in per_pose:
        if r.get("auto_failures"):
            fail_details.append(f"- **{r['pose_id']}**: {', '.join(r['auto_failures'])}")

    lines += [
        "",
        f"生产姿态通过: {len([r for r in production if r.get('auto_gate_pass') is True])}/{len(production)}",
        f"边界姿态: {len(boundary)}",
        "",
        "## 自动 Gate 标准",
        f"- Accuracy median ≤ {GATE_ACC_MEDIAN} mm",
        f"- Accuracy P95 ≤ {GATE_ACC_P95} mm",
        f"- Completeness coverage@10mm ≥ {GATE_COMPLETENESS_COV10}",
        f"- removed_fragment_area_ratio ≤ {GATE_REMOVED_RATIO}",
        "- bottom == UNKNOWN",
        "- 所有 run 成功且 Gate 通过",
        "- 同姿态重复运行 mesh SHA256 一致",
        "",
        "## 失败详情" if fail_details else "## 无自动 Gate 失败",
    ]
    if fail_details:
        lines.extend(fail_details)
    else:
        lines.append("所有自动 Gate 通过。")

    # 手动审查项
    manual_poses = [r for r in production if r.get("manual_review_required")]
    if manual_poses:
        lines += [
            "",
            "## 手动结构审查 (MANUAL_REVIEW_REQUIRED)",
            "",
            "以下项目无法自动判断，需要人工审查：",
            "- no_mirror: 不出现整体镜像",
            "- no_double_surface: 不出现明显双层表面",
            "- top_offset_block_preserved: 顶部偏置块保留",
            "- no_global_layer_shift: 不出现全局平移错层",
            "- no_gantry_floor_residual: 无门架/地面残留",
        ]

    path = os.path.join(output_root, "robustness_summary.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    logger.info("汇总: %s", path)


def _write_per_pose_csv(output_root, per_pose):
    path = os.path.join(output_root, "per_pose_metrics.csv")
    fields = [
        "pose_id", "category", "status", "auto_gate_pass",
        "runs_completed", "runs_expected", "all_runs_pass",
        "accuracy_median_mm", "accuracy_p95_mm", "accuracy_rmse_mm",
        "accuracy_coverage_10mm",
        "completeness_median_mm", "completeness_p95_mm",
        "completeness_coverage_10mm",
        "chamfer_mm", "removed_fragment_area_ratio",
        "raw_components", "cleaned_components",
        "bottom_status", "mesh_sha_count",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for r in per_pose:
            writer.writerow({k: r.get(k, "") for k in fields})
    logger.info("CSV: %s", path)


def main():
    parser = argparse.ArgumentParser(description="多姿态重建鲁棒性验证套件 V2")
    parser.add_argument("--dataset", "-d", required=True,
                        help="多姿态数据集根目录")
    parser.add_argument("--poses", "-p", required=True,
                        help="robustness_poses.yaml 路径")
    parser.add_argument("--rig", "-r", required=True,
                        help="Stable rig YAML/JSON 路径")
    parser.add_argument("--config", "-c", required=True,
                        help="Production config YAML 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出目录")
    args = parser.parse_args()

    if not os.path.isfile(_RUNNER):
        logger.error("正式 runner 未找到: %s", _RUNNER)
        sys.exit(1)

    exit_code = run_robustness_suite(
        dataset_root=args.dataset,
        poses_yaml=args.poses,
        rig_path=args.rig,
        config_path=args.config,
        output_root=args.output,
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
多姿态重建鲁棒性验证套件 (V3 — hard fail-closed Gate).

Gate 强制检查:
  - 所有 run 成功并通过 production Gate
  - accuracy median ≤ 5mm, P95 ≤ 16mm (最差 run)
  - completeness coverage@10mm ≥ 0.80 (最差 run)
  - removed_fragment_area_ratio ≤ 0.03 (最差 run)
  - bottom == UNKNOWN (任一 run 违反即失败)
  - mesh SHA 全部存在, 同姿态重复运行一致
  - 缺失任一强制字段 → FAIL
  - 结构项 MANUAL_REVIEW_REQUIRED → exit 2 (自动 PASS 但人工 pending)

Exit codes:
  0 = FINAL_PASS (auto + manual approved)
  1 = AUTO_FAIL
  2 = AUTO_PASS + MANUAL_REVIEW_PENDING

Usage:
  rosrun cr5_spray_perception run_reconstruction_robustness_suite.py \
    --dataset <multi_pose_dataset> \
    --poses <robustness_poses.yaml> \
    --rig <stable_rig.yaml> \
    --config <production_config.yaml> \
    --output <results_dir> \
    [--manual-review-json <review.json>]
"""
import os, sys, json, yaml, csv, hashlib, logging, argparse, subprocess, datetime

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("robustness")

WS = os.path.join(os.path.dirname(__file__), "..")
_RUNNER = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")

GATE_ACC_MEDIAN = 5.0
GATE_ACC_P95 = 16.0
GATE_COMPLETENESS_COV10 = 0.80
GATE_REMOVED_RATIO = 0.03

# 生产 runner 导出符号供测试直接调用
EXIT_AUTO_FAIL = 1
EXIT_MANUAL_PENDING = 2


def sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_poses(poses_yaml):
    with open(poses_yaml) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("poses", []), cfg.get("acceptance", {})


def load_manual_review(path):
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        return json.load(f)


def extract_run_metrics(report):
    """从 quality_report 提取所有 Gate 相关指标. 缺失字段返回 None (不是默认值)."""
    acc = report.get("accuracy", {})
    comp = report.get("completeness", {})
    quality = report.get("quality", {})
    acceptance = report.get("acceptance", {})
    mesh = report.get("mesh", {})
    unobs = report.get("unobserved", {})

    return {
        "accuracy_median_mm": acc.get("median_mm"),
        "accuracy_p95_mm": acc.get("p95_mm"),
        "accuracy_rmse_mm": acc.get("rmse_mm"),
        "accuracy_coverage_10mm": acc.get("coverage_10mm"),
        "completeness_median_mm": comp.get("median_mm"),
        "completeness_p95_mm": comp.get("p95_mm"),
        "completeness_coverage_10mm": comp.get("coverage_10mm"),
        "chamfer_mm": quality.get("chamfer_mm"),
        "production_pass": acceptance.get("production_pass"),
        "removed_fragment_area_ratio": mesh.get("removed_fragment_area_ratio"),
        "raw_components": mesh.get("raw_components"),
        "cleaned_components": mesh.get("cleaned_components"),
        "bottom_status": unobs.get("bottom_surface"),
    }


def aggregate_worst_metrics(run_metrics_list):
    """从多个 run 的 metrics dict 聚合成最差值.

    距离指标取 max, coverage 取 min.
    返回 (worst_dict, errors).
    """
    if not run_metrics_list:
        return {}, ["no run metrics to aggregate"]

    errors = []

    # 验证必填字段
    required = [
        "accuracy_median_mm", "accuracy_p95_mm",
        "completeness_coverage_10mm",
        "removed_fragment_area_ratio",
        "bottom_status",
        "production_pass",
    ]
    for i, m in enumerate(run_metrics_list):
        for key in required:
            if m.get(key) is None:
                errors.append(f"run {i}: missing required field '{key}'")

    if errors:
        return {}, errors

    # 聚合
    worst = {}

    # 距离 — max across runs
    for key in ["accuracy_median_mm", "accuracy_p95_mm", "accuracy_rmse_mm",
                 "completeness_median_mm", "completeness_p95_mm", "chamfer_mm"]:
        vals = [m[key] for m in run_metrics_list if m.get(key) is not None]
        worst[key] = max(vals) if vals else None

    # coverage — min across runs
    for key in ["accuracy_coverage_10mm", "completeness_coverage_10mm"]:
        vals = [m[key] for m in run_metrics_list if m.get(key) is not None]
        worst[key] = min(vals) if vals else None

    # removed ratio — max
    vals = [m["removed_fragment_area_ratio"] for m in run_metrics_list
            if m.get("removed_fragment_area_ratio") is not None]
    worst["removed_fragment_area_ratio"] = max(vals) if vals else None

    # bottom — 任一非 UNKNOWN 即失败
    bottoms = set(m.get("bottom_status") for m in run_metrics_list)
    if bottoms == {"UNKNOWN"}:
        worst["bottom_status"] = "UNKNOWN"
    elif len(bottoms) == 1:
        worst["bottom_status"] = list(bottoms)[0]
    else:
        worst["bottom_status"] = "INCONSISTENT"

    # production_pass — 全部必须 true
    worst["production_pass"] = all(
        m.get("production_pass") is True for m in run_metrics_list)

    # raw/cleaned components — max
    for key in ["raw_components", "cleaned_components"]:
        vals = [m[key] for m in run_metrics_list if m.get(key) is not None]
        worst[key] = max(vals) if vals else None

    return worst, []


def evaluate_pose_gate(worst_metrics, mesh_hashes, n_runs_completed, n_runs_expected):
    """使用最差聚合指标判断单姿态 Gate.

    Args:
        worst_metrics: aggregate_worst_metrics 的返回值
        mesh_hashes: list[str] — 所有成功运行的 mesh SHA (非 set)
        n_runs_completed: 成功完成数量
        n_runs_expected: 预期数量

    Returns: (result_dict, auto_pass, failures_list)
    """
    failures = []

    # 运行完整性
    if n_runs_completed != n_runs_expected:
        failures.append(f"runs_completed={n_runs_completed}/{n_runs_expected}")

    # mesh SHA
    if len(mesh_hashes) != n_runs_expected:
        failures.append(f"mesh SHA count={len(mesh_hashes)}/{n_runs_expected}")
    elif any(not sha for sha in mesh_hashes):
        failures.append("mesh SHA missing (empty)")
    elif len(set(mesh_hashes)) != 1:
        failures.append("mesh SHA mismatch across runs")

    # production_pass (全部 run)
    if worst_metrics.get("production_pass") is not True:
        failures.append("not all runs passed production Gate")

    # accuracy
    acc_med = worst_metrics.get("accuracy_median_mm")
    if acc_med is None:
        failures.append("accuracy_median_mm missing")
    elif acc_med > GATE_ACC_MEDIAN:
        failures.append(f"accuracy median={acc_med:.2f} > {GATE_ACC_MEDIAN}")

    acc_p95 = worst_metrics.get("accuracy_p95_mm")
    if acc_p95 is None:
        failures.append("accuracy_p95_mm missing")
    elif acc_p95 > GATE_ACC_P95:
        failures.append(f"accuracy P95={acc_p95:.2f} > {GATE_ACC_P95}")

    # completeness coverage@10mm
    comp_cov10 = worst_metrics.get("completeness_coverage_10mm")
    if comp_cov10 is None:
        failures.append("completeness_coverage_10mm missing")
    elif comp_cov10 < GATE_COMPLETENESS_COV10:
        failures.append(f"completeness_cov10={comp_cov10:.3f} < {GATE_COMPLETENESS_COV10}")

    # removed ratio
    removed = worst_metrics.get("removed_fragment_area_ratio")
    if removed is None:
        failures.append("removed_fragment_area_ratio missing")
    elif removed > GATE_REMOVED_RATIO:
        failures.append(f"removed_ratio={removed:.3f} > {GATE_REMOVED_RATIO}")

    # bottom
    bottom = worst_metrics.get("bottom_status")
    if bottom is None:
        failures.append("bottom_status missing")
    elif bottom != "UNKNOWN":
        failures.append(f"bottom_status={bottom} != UNKNOWN")

    auto_pass = len(failures) == 0

    result = {
        "runs_completed": n_runs_completed,
        "runs_expected": n_runs_expected,
        "mesh_hashes": mesh_hashes,
        "mesh_sha_count": len(mesh_hashes),
        "mesh_sha_unique": len(set(mesh_hashes)) if mesh_hashes else 0,
        "worst_metrics": worst_metrics,
        "status": "AUTO_PASS" if auto_pass else "AUTO_FAIL",
        "auto_gate_pass": auto_pass,
        "auto_failures": failures,
        "manual_review_required": [
            "no_mirror",
            "no_double_surface",
            "top_offset_block_preserved",
            "no_global_layer_shift",
            "no_gantry_floor_residual",
        ],
    }
    return result, auto_pass, failures


def run_reconstruction(dataset_path, rig_path, config_path, output_dir,
                       visible_gt=None, release_id=None, model_pose_json=None):
    """调用正式重建入口 (fail-closed)."""
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
    if model_pose_json:
        cmd += ["--model-pose-json", model_pose_json]

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
        logger.error("  quality_report.json not found")
        return False, None

    return True, qpath


def run_robustness_suite(dataset_root, poses_yaml, rig_path, config_path,
                          output_root, manual_review_json=None):
    """主入口."""
    poses, _ = load_poses(poses_yaml)
    os.makedirs(output_root, exist_ok=True)

    logger.info("多姿态鲁棒性验证: %d poses", len(poses))
    logger.info("  rig: %s", rig_path)
    logger.info("  config: %s", config_path)

    manual_review = load_manual_review(manual_review_json)
    per_pose = []
    ts_start = datetime.datetime.utcnow()
    fatal_errors = []

    for pose in poses:
        pose_id = pose["id"]
        preset = pose["preset"]
        category = pose.get("category", "production")
        repeat = pose.get("repeat", 1)
        desc = pose.get("description", "")

        logger.info("── Pose %s (%s, repeat=%d) ──", pose_id, preset, repeat)

        pose_dir = os.path.join(dataset_root, f"pose_{pose_id.lower()}")
        if not os.path.isdir(pose_dir):
            logger.warning("  dataset dir missing: SKIP")
            per_pose.append({"pose_id": pose_id, "preset": preset,
                             "category": category, "status": "SKIP",
                             "reason": "dataset_missing"})
            if category == "production":
                fatal_errors.append(f"{pose_id}: dataset missing")
            continue

        # per-pose visible GT (production 强制)
        per_pose_gt = os.path.join(pose_dir, "visible_gt", "visible_union.ply")
        if category == "production" and not os.path.isfile(per_pose_gt):
            logger.error("  per-pose visible GT missing: %s", per_pose_gt)
            per_pose.append({"pose_id": pose_id, "preset": preset,
                             "category": category, "status": "FAIL",
                             "reason": "visible_gt_missing"})
            fatal_errors.append(f"{pose_id}: visible GT missing")
            continue

        # pose evidence (production 强制)
        pose_evidence = os.path.join(pose_dir, "pose_evidence.json")
        if category == "production" and not os.path.isfile(pose_evidence):
            logger.error("  pose evidence missing: %s", pose_evidence)
            per_pose.append({"pose_id": pose_id, "preset": preset,
                             "category": category, "status": "FAIL",
                             "reason": "pose_evidence_missing"})
            fatal_errors.append(f"{pose_id}: pose evidence missing")
            continue

        visible_gt = per_pose_gt if os.path.isfile(per_pose_gt) else None

        run_hashes = []
        run_metrics_list = []
        all_runs_success = True

        for run_i in range(repeat):
            run_output = os.path.join(output_root, f"run_{pose_id.lower()}_r{run_i}")
            release_id = f"robustness-{pose_id.lower()}"

            success, qpath = run_reconstruction(
                dataset_path=pose_dir,
                rig_path=rig_path,
                config_path=config_path,
                output_dir=run_output,
                visible_gt=visible_gt,
                release_id=release_id,
                model_pose_json=pose_evidence if os.path.isfile(pose_evidence) else None,
            )

            if not success:
                all_runs_success = False
                logger.warning("  run %d/%d FAILED", run_i + 1, repeat)
                continue

            report = json.load(open(qpath))
            metrics = extract_run_metrics(report)

            if metrics.get("production_pass") is not True:
                all_runs_success = False
                logger.warning("  run %d/%d Gate FAIL", run_i + 1, repeat)

            run_metrics_list.append(metrics)

            mesh_path = os.path.join(run_output, "visible_surface_mesh_final.ply")
            mesh_sha = sha256_file(mesh_path) or ""
            run_hashes.append(mesh_sha)

        # ── 聚合 ──
        if not run_metrics_list:
            per_pose.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "FAIL", "reason": "all_runs_failed",
                "runs_completed": 0, "runs_expected": repeat,
            })
            if category == "production":
                fatal_errors.append(f"{pose_id}: all runs failed")
            continue

        worst_metrics, agg_errors = aggregate_worst_metrics(run_metrics_list)

        if agg_errors:
            for e in agg_errors:
                logger.error("  aggregation error: %s", e)

        gate_result, auto_pass, failures = evaluate_pose_gate(
            worst_metrics=worst_metrics,
            mesh_hashes=run_hashes,
            n_runs_completed=len(run_metrics_list),
            n_runs_expected=repeat,
        )

        gate_result["pose_id"] = pose_id
        gate_result["preset"] = preset
        gate_result["category"] = category
        gate_result["description"] = desc
        if agg_errors:
            gate_result["aggregation_errors"] = agg_errors
            gate_result["auto_gate_pass"] = False
            gate_result["status"] = "AUTO_FAIL"
            failures.extend(agg_errors)

        per_pose.append(gate_result)

        icon = "✓" if gate_result["auto_gate_pass"] else "✗"
        logger.info("  => %s  acc_med=%.2f  p95=%.2f  cov10=%.3f  removed=%.3f  bottom=%s  sha=%d/%d",
                    icon,
                    worst_metrics.get("accuracy_median_mm") or 0,
                    worst_metrics.get("accuracy_p95_mm") or 0,
                    worst_metrics.get("completeness_coverage_10mm") or 0,
                    worst_metrics.get("removed_fragment_area_ratio") or 0,
                    worst_metrics.get("bottom_status", "?"),
                    gate_result.get("mesh_sha_unique", 0),
                    gate_result["runs_expected"])
        if failures:
            for f in failures:
                logger.info("    FAIL: %s", f)

    # ── 最终 Gate ──
    ts_end = datetime.datetime.utcnow()
    production = [r for r in per_pose if r["category"] == "production"]
    all_auto_pass = all(r.get("auto_gate_pass") is True for r in production if r["status"] != "SKIP")

    # 检查人工审查
    manual_approved = {}
    for r in production:
        pid = r["pose_id"]
        if pid in manual_review:
            manual_approved[pid] = manual_review[pid].get("status") == "APPROVED"

    all_manual_approved = (
        len(manual_approved) == len(production)
        and all(manual_approved.values())
    ) if production else False

    # ── 写报告 ──
    report = _build_report(per_pose, poses_yaml, rig_path, config_path,
                            ts_start, ts_end, all_auto_pass, all_manual_approved)
    _write_json_report(output_root, report)
    _write_summary_md(output_root, per_pose, all_auto_pass, all_manual_approved)
    _write_per_pose_csv(output_root, per_pose)

    # ── 退出码 ──
    if fatal_errors:
        logger.error("FATAL: %d errors", len(fatal_errors))
        for e in fatal_errors:
            logger.error("  %s", e)

    if not all_auto_pass:
        logger.error("GATE: AUTO_FAIL")
        return EXIT_AUTO_FAIL

    if not all_manual_approved:
        logger.info("GATE: AUTO_PASS, MANUAL_REVIEW_PENDING (%d poses)",
                    len(production) - len(manual_approved))
        return EXIT_MANUAL_PENDING

    logger.info("GATE: FINAL_PASS")
    return 0


def _build_report(per_pose, poses_yaml, rig_path, config_path,
                   ts_start, ts_end, all_auto_pass, all_manual_approved):
    production = [r for r in per_pose if r["category"] == "production"]
    evaluated = [r for r in production if r.get("worst_metrics")]

    worst_med, worst_med_pose = None, None
    worst_p95, worst_p95_pose = None, None
    worst_cov10, worst_cov10_pose = None, None

    for r in evaluated:
        wm = r.get("worst_metrics", {})
        m = wm.get("accuracy_median_mm")
        if m is not None and (worst_med is None or m > worst_med):
            worst_med, worst_med_pose = m, r["pose_id"]
        p = wm.get("accuracy_p95_mm")
        if p is not None and (worst_p95 is None or p > worst_p95):
            worst_p95, worst_p95_pose = p, r["pose_id"]
        c = wm.get("completeness_coverage_10mm")
        if c is not None and (worst_cov10 is None or c < worst_cov10):
            worst_cov10, worst_cov10_pose = c, r["pose_id"]

    manual_pending = [r["pose_id"] for r in production
                      if r.get("auto_gate_pass") is True]

    return {
        "schema": "cr5_robustness_report_v3",
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
            "auto_gate": "PASS" if all_auto_pass else "FAIL",
            "manual_gate": "APPROVED" if all_manual_approved else "PENDING",
            "final_gate": "PASS" if (all_auto_pass and all_manual_approved) else "PENDING",
            "worst_accuracy_median_mm": worst_med,
            "worst_accuracy_median_pose": worst_med_pose,
            "worst_accuracy_p95_mm": worst_p95,
            "worst_accuracy_p95_pose": worst_p95_pose,
            "worst_completeness_coverage_10mm": worst_cov10,
            "worst_completeness_pose": worst_cov10_pose,
            "repeatability_failures": len([r for r in per_pose
                                            if r.get("mesh_sha_unique", 1) != 1]),
            "manual_review_pending": manual_pending,
        },
        "per_pose": per_pose,
    }


def _write_json_report(output_root, report):
    path = os.path.join(output_root, "robustness_report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    logger.info("报告: %s", path)


def _write_summary_md(output_root, per_pose, all_auto_pass, all_manual_approved):
    production = [r for r in per_pose if r["category"] == "production"]

    lines = [
        "# 多姿态重建鲁棒性验证报告",
        "",
        f"## Auto Gate: {'PASS' if all_auto_pass else 'FAIL'}",
        f"## Manual Review: {'APPROVED' if all_manual_approved else 'PENDING'}",
        f"## Final: {'PASS' if (all_auto_pass and all_manual_approved) else 'PENDING'}",
        "",
        "| 姿态 | Auto | Acc Med | Acc P95 | Cov10 | Removed | Bottom | SHA | Manual |",
        "|------|------|---------|---------|-------|---------|--------|-----|--------|",
    ]
    for r in per_pose:
        wm = r.get("worst_metrics", {})
        med = f"{wm.get('accuracy_median_mm', '?'):.2f}" if wm.get("accuracy_median_mm") is not None else "MISS"
        p95 = f"{wm.get('accuracy_p95_mm', '?'):.2f}" if wm.get("accuracy_p95_mm") is not None else "MISS"
        cov10 = f"{wm.get('completeness_coverage_10mm', '?'):.3f}" if wm.get("completeness_coverage_10mm") is not None else "MISS"
        removed = f"{wm.get('removed_fragment_area_ratio', '?'):.3f}" if wm.get("removed_fragment_area_ratio") is not None else "MISS"
        bottom = wm.get("bottom_status", "MISS")
        sha = f"{r.get('mesh_sha_unique',0)}/{r.get('runs_expected',0)}"
        manual = "PENDING" if r.get("auto_gate_pass") else "—"
        auto = "PASS" if r.get("auto_gate_pass") else "FAIL"
        lines.append(f"| {r['pose_id']} | {auto} | {med} | {p95} | {cov10} | {removed} | {bottom} | {sha} | {manual} |")

    if not all_auto_pass:
        lines += ["", "## Auto Failures"]
        for r in per_pose:
            if r.get("auto_failures"):
                lines.append(f"- **{r['pose_id']}**: {', '.join(r['auto_failures'])}")

    lines += [
        "",
        "## Manual Review Required",
        "- no_mirror, no_double_surface, top_offset_block_preserved",
        "- no_global_layer_shift, no_gantry_floor_residual",
        "",
        "Use --manual-review-json to approve.",
    ]

    path = os.path.join(output_root, "robustness_summary.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def _write_per_pose_csv(output_root, per_pose):
    path = os.path.join(output_root, "per_pose_metrics.csv")
    fields = [
        "pose_id", "category", "status", "auto_gate_pass",
        "runs_completed", "runs_expected",
        "accuracy_median_mm", "accuracy_p95_mm", "accuracy_rmse_mm",
        "completeness_median_mm", "completeness_p95_mm",
        "completeness_coverage_10mm", "chamfer_mm",
        "removed_fragment_area_ratio", "bottom_status",
        "mesh_sha_count", "mesh_sha_unique",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for r in per_pose:
            row = {k: r.get(k, "") for k in fields if k in r}
            wm = r.get("worst_metrics", {})
            for k in fields:
                if k in wm and k not in row:
                    row[k] = wm[k]
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="多姿态重建鲁棒性验证套件 V3")
    parser.add_argument("--dataset", "-d", required=True)
    parser.add_argument("--poses", "-p", required=True)
    parser.add_argument("--rig", "-r", required=True)
    parser.add_argument("--config", "-c", required=True)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--manual-review-json", default=None,
                        help="人工结构审查 JSON (P0-P5 APPROVED 后 FINAL_PASS)")
    args = parser.parse_args()

    if not os.path.isfile(_RUNNER):
        logger.error("runner not found: %s", _RUNNER)
        sys.exit(1)

    exit_code = run_robustness_suite(
        dataset_root=args.dataset,
        poses_yaml=args.poses,
        rig_path=args.rig,
        config_path=args.config,
        output_root=args.output,
        manual_review_json=args.manual_review_json,
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

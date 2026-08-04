#!/usr/bin/env python3
"""
多姿态重建鲁棒性验证套件.

对多个目标姿态分别调用正式重建入口, 汇总验证:
  - 每个姿态的 Accuracy Gate (median≤5mm, P95≤16mm)
  - 重复性 (同一姿态多次运行的 mesh SHA256 一致)
  - 结构完整性 (非镜像、无错层、无虚假底面)

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


def sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def load_poses(poses_yaml):
    with open(poses_yaml) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("poses", []), cfg.get("acceptance", {})


def run_reconstruction(dataset_path, rig_path, config_path, output_dir,
                       visible_gt=None, release_id=None):
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

    logger.info("  runner: %s", " ".join(cmd))

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        logger.error("  TIMEOUT after 600s")
        return False, None

    if result.returncode != 0:
        logger.error("  FAIL (exit=%d): %s", result.returncode,
                     result.stderr[-300:] if result.stderr else "(no stderr)")
        return False, None

    qpath = os.path.join(output_dir, "reconstruction_quality_report.json")
    if not os.path.isfile(qpath):
        logger.error("  quality_report.json not found after run")
        return False, None

    return True, qpath


def load_quality_report(qpath):
    with open(qpath) as f:
        return json.load(f)


def classify_failure(report, mesh_path=None):
    """对失败重建进行初步分类. 返回 failure_category 和描述."""
    categories = []

    acc = report.get("accuracy", {})
    if acc.get("status") == "NOT_AVAILABLE":
        categories.append("NOT_EVALUATED")

    mesh = report.get("mesh", {})
    if mesh.get("removed_fragment_area_ratio", 0) > 0.03:
        categories.append("D_MESH_OVER_CLEANUP")

    # 检查是否有 bottom 被意外填充
    unobs = report.get("unobserved", {})
    if unobs.get("bottom_surface") != "UNKNOWN":
        categories.append("FALSE_BOTTOM")

    acceptance = report.get("acceptance", {})
    if not acceptance:
        categories.append("NO_ACCEPTANCE")

    if not categories:
        categories.append("METRICS_FAIL")

    return categories


def run_robustness_suite(dataset_root, poses_yaml, rig_path, config_path, output_root,
                          visible_gt=None):
    """主入口: 运行多姿态验证套件."""
    poses, acceptance_cfg = load_poses(poses_yaml)
    os.makedirs(output_root, exist_ok=True)

    logger.info("多姿态鲁棒性验证: %d poses", len(poses))
    logger.info("  rig: %s", rig_path)
    logger.info("  config: %s", config_path)
    logger.info("  dataset: %s", dataset_root)

    results = []
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
            results.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "SKIP", "reason": "dataset_missing",
            })
            continue

        run_hashes = []
        run_reports = []
        all_pass = True

        for run_i in range(repeat):
            run_label = f"{pose_id}_r{run_i}" if repeat > 1 else pose_id
            run_output = os.path.join(output_root, f"run_{pose_id.lower()}_r{run_i}")
            release_id = f"robustness-{pose_id.lower()}"

            success, qpath = run_reconstruction(
                dataset_path=pose_dir,
                rig_path=rig_path,
                config_path=config_path,
                output_dir=run_output,
                visible_gt=visible_gt,
                release_id=release_id,
            )

            if not success:
                all_pass = False
                continue

            report = load_quality_report(qpath)
            run_reports.append(report)

            # mesh SHA256 for repeatability
            mesh_path = os.path.join(run_output, "visible_surface_mesh_final.ply")
            mesh_sha = sha256_file(mesh_path)
            if mesh_sha:
                run_hashes.append(mesh_sha)

        # ── 汇总单个姿态 ──
        if not run_reports:
            results.append({
                "pose_id": pose_id, "preset": preset, "category": category,
                "status": "FAIL", "reason": "all_runs_failed",
            })
            continue

        # 取最后一次运行的 report 作为主报告 (单次运行就是唯一报告)
        main_report = run_reports[-1]
        acc = main_report.get("accuracy", {})
        acc_med = acc.get("median_mm")
        acc_p95 = acc.get("p95_mm")
        acceptance = main_report.get("acceptance", {})
        prod_pass = acceptance.get("production_pass")

        # 重复性检查
        hashes_unique = len(set(run_hashes)) if run_hashes else 0
        repeatable = hashes_unique == 1 if run_hashes else None

        # 结构检查
        mesh = main_report.get("mesh", {})
        removed_ratio = mesh.get("removed_fragment_area_ratio", 0)
        unobs = main_report.get("unobserved", {})
        bottom_ok = unobs.get("bottom_surface") == "UNKNOWN"

        pose_result = {
            "pose_id": pose_id,
            "preset": preset,
            "category": category,
            "runs_completed": len(run_reports),
            "runs_expected": repeat,
            "accuracy_median_mm": acc_med,
            "accuracy_p95_mm": acc_p95,
            "production_pass": prod_pass,
            "repeatable": repeatable,
            "mesh_hashes": list(set(run_hashes)),
            "removed_fragment_area_ratio": removed_ratio,
            "bottom_ok": bottom_ok,
        }

        # 总体状态
        failures = []
        if prod_pass is not True:
            failures.append("GATE_FAIL")
        if repeat > 1 and repeatable is False:
            failures.append("NOT_REPEATABLE")
        if not bottom_ok:
            failures.append("BOTTOM_VIOLATION")

        if not failures:
            pose_result["status"] = "PASS"
        else:
            pose_result["status"] = "FAIL"
            pose_result["failure_reasons"] = failures
            pose_result["failure_categories"] = classify_failure(main_report)

        results.append(pose_result)

        status_icon = "PASS" if pose_result["status"] == "PASS" else "FAIL"
        logger.info("    => %s  acc_med=%.2f  acc_p95=%.2f  pass=%s  repeatable=%s",
                    status_icon, acc_med or 0, acc_p95 or 0, prod_pass, repeatable)

    # ── 汇总报告 ──
    ts_end = datetime.datetime.utcnow()
    _write_robustness_report(output_root, results, poses_yaml, rig_path, config_path,
                              ts_start, ts_end, acceptance_cfg)
    _write_summary_md(output_root, results, acceptance_cfg)
    _write_per_pose_csv(output_root, results)

    # ── 最终 Gate ──
    production_fails = [r for r in results
                        if r["category"] == "production" and r["status"] != "PASS"]
    if production_fails:
        logger.error("PRODUCTION GATE FAIL: %d/%d poses failed",
                     len(production_fails),
                     len([r for r in results if r["category"] == "production"]))
        return 1
    else:
        logger.info("PRODUCTION GATE PASS: all production poses pass")
        return 0


def _write_robustness_report(output_root, results, poses_yaml, rig_path, config_path,
                              ts_start, ts_end, acceptance_cfg):
    production_results = [r for r in results if r["category"] == "production"]
    boundary_results = [r for r in results if r["category"] == "boundary"]

    worst_acc_med = None
    worst_acc_p95 = None
    worst_pose = None
    for r in production_results:
        if r.get("accuracy_median_mm") is not None:
            if worst_acc_med is None or r["accuracy_median_mm"] > worst_acc_med:
                worst_acc_med = r["accuracy_median_mm"]
                worst_pose = r["pose_id"]
        if r.get("accuracy_p95_mm") is not None:
            if worst_acc_p95 is None or r["accuracy_p95_mm"] > worst_acc_p95:
                worst_acc_p95 = r["accuracy_p95_mm"]

    report = {
        "schema": "cr5_robustness_report_v1",
        "timestamp": ts_start.isoformat() + "Z",
        "duration_s": (ts_end - ts_start).total_seconds(),
        "input": {
            "poses_yaml": os.path.abspath(poses_yaml),
            "rig_path": os.path.abspath(rig_path),
            "config_path": os.path.abspath(config_path),
        },
        "acceptance_criteria": acceptance_cfg,
        "summary": {
            "total_poses": len(results),
            "production_poses": len(production_results),
            "boundary_poses": len(boundary_results),
            "pass_count": len([r for r in results if r["status"] == "PASS"]),
            "fail_count": len([r for r in results if r["status"] == "FAIL"]),
            "skip_count": len([r for r in results if r["status"] == "SKIP"]),
            "production_pass_count": len([r for r in production_results if r["status"] == "PASS"]),
            "production_fail_count": len([r for r in production_results if r["status"] == "FAIL"]),
            "worst_accuracy_median_mm": worst_acc_med,
            "worst_accuracy_p95_mm": worst_acc_p95,
            "worst_pose": worst_pose,
            "repeatability_failures": len([r for r in results
                                            if r.get("repeatable") is False]),
            "production_gate": "PASS" if all(r["status"] == "PASS"
                                              for r in production_results) else "FAIL",
        },
        "per_pose": results,
    }
    path = os.path.join(output_root, "robustness_report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    logger.info("报告: %s", path)


def _write_summary_md(output_root, results, acceptance_cfg):
    """生成人类可读的汇总 Markdown."""
    production_results = [r for r in results if r["category"] == "production"]
    boundary_results = [r for r in results if r["category"] == "boundary"]
    prod_pass = all(r["status"] == "PASS" for r in production_results)

    lines = [
        "# 多姿态重建鲁棒性验证报告",
        "",
        f"## 总体结果: {'PASS' if prod_pass else 'FAIL'}",
        "",
        "| 姿态 | 类别 | 状态 | Acc Median | Acc P95 | Gate | 重复性 |",
        "|------|------|------|-----------|---------|------|--------|",
    ]

    for r in results:
        med = f"{r['accuracy_median_mm']:.2f}" if r.get("accuracy_median_mm") else "N/A"
        p95 = f"{r['accuracy_p95_mm']:.2f}" if r.get("accuracy_p95_mm") else "N/A"
        gate = "PASS" if r.get("production_pass") else ("FAIL" if r.get("production_pass") is False else "N/A")
        rep = ""
        if r.get("repeatable") is True:
            rep = "OK"
        elif r.get("repeatable") is False:
            rep = "FAIL"
        elif r.get("runs_completed", 0) > 1:
            rep = "N/A"
        lines.append(f"| {r['pose_id']} | {r['category']} | {r['status']} | {med} | {p95} | {gate} | {rep} |")

    lines += [
        "",
        f"生产姿态通过: {len([r for r in production_results if r['status'] == 'PASS'])}/{len(production_results)}",
        f"边界探索姿态: {len(boundary_results)}",
        "",
        "## 验收标准",
        f"- Accuracy median ≤ {acceptance_cfg.get('production', {}).get('accuracy_median_mm', 5.0)} mm",
        f"- Accuracy P95 ≤ {acceptance_cfg.get('production', {}).get('accuracy_p95_mm', 16.0)} mm",
        "- 同姿态重复运行 mesh SHA256 一致",
        "- 不出现镜像、错层、双层、误删、虚假底面",
        "- bottom 始终为 UNKNOWN",
    ]

    path = os.path.join(output_root, "robustness_summary.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    logger.info("汇总: %s", path)


def _write_per_pose_csv(output_root, results):
    path = os.path.join(output_root, "per_pose_metrics.csv")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "pose_id", "category", "status", "runs_completed", "runs_expected",
            "accuracy_median_mm", "accuracy_p95_mm", "production_pass",
            "repeatable", "removed_fragment_area_ratio", "bottom_ok",
        ])
        writer.writeheader()
        for r in results:
            writer.writerow({k: r.get(k, "") for k in writer.fieldnames})
    logger.info("CSV: %s", path)


def main():
    parser = argparse.ArgumentParser(description="多姿态重建鲁棒性验证套件")
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
    parser.add_argument("--visible-gt", default=None,
                        help="visible_union.ply 路径 (可选)")
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
        visible_gt=args.visible_gt,
    )
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""CR5 Reconstruction — 位姿细化运行入口."""
import os, sys, json, yaml, logging, argparse
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("refine_rig")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract)
from cr5_spray_perception.reconstruction.extrinsics import load_calibrated_rig
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest)
from cr5_spray_perception.reconstruction.registration import load_registration_evidence
from cr5_spray_perception.reconstruction.pose_refinement import refine_reconstruction_rig


def main():
    parser = argparse.ArgumentParser(description="位姿细化")
    parser.add_argument("--dataset", "-d", required=True)
    parser.add_argument("--rig", "-r", required=True, help="Stable V1 calibrated_rig YAML")
    parser.add_argument("--config", "-c", default=None)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--group-id", "-g", type=int, default=0)
    parser.add_argument("--registration-evidence", default=None)
    args = parser.parse_args()

    # 配置
    if args.config and os.path.isfile(args.config):
        with open(args.config) as f:
            config = yaml.safe_load(f) or {}
    else:
        config = {}
    config.setdefault("depth_min_m", 0.15)
    config.setdefault("depth_max_m", 2.0)

    # 数据集
    layout = resolve_capture_group(args.dataset, args.group_id)
    rig = load_calibrated_rig(args.rig)
    if rig.get("status") != "PASS":
        logger.error("rig status=%s", rig.get("status")); sys.exit(1)

    ev_path = args.registration_evidence or os.path.join(layout.group_dir, "depth_registration.json")
    evidence = load_registration_evidence(ev_path) if os.path.isfile(ev_path) else None

    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True, registration_evidence=evidence)
        if not rgbd.is_valid:
            for e in rgbd.errors: logger.error("  %s", e)
            sys.exit(1)
        rgbd_list.append(rgbd)

    # 运行细化
    os.makedirs(args.output, exist_ok=True)
    refined_rig, report, pair_results = refine_reconstruction_rig(rgbd_list, rig, config)

    # 保存 refined rig
    rig_path = os.path.join(args.output, "refined_rig_reconstruction.yaml")
    with open(rig_path, "w") as f:
        yaml.dump(refined_rig, f, default_flow_style=None, sort_keys=False, width=120)

    # 保存输入 rig 副本
    import shutil
    shutil.copy2(args.rig, os.path.join(args.output, "calibrated_rig_stable_v1_input.yaml"))

    # 保存报告
    report_path = os.path.join(args.output, "refinement_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)

    # 保存残差
    res_before = {}
    res_after = {}
    for pk, pr in pair_results.items():
        if pr.get("status") == "PASS":
            res_before[pk] = pr.get("residual_before", {})
            res_after[pk] = pr.get("residual_after", {})
    for name, data in [("residuals_before", res_before), ("residuals_after", res_after)]:
        with open(os.path.join(args.output, f"{name}.json"), "w") as f:
            json.dump(data, f, indent=2)

    # 控制台
    print(f"\n{'='*60}")
    print(f"位姿细化: {report['status']}")
    print(f"{'='*60}")
    for cam_name in REQUIRED_CAMERAS:
        cd = refined_rig["cameras"].get(cam_name, {})
        dt = cd.get("translation_delta_mm", 0)
        dr = cd.get("rotation_delta_deg", 0)
        print(f"  {cam_name}: Δt={dt:.2f}mm Δr={dr:.3f}°")
    print(f"  三角闭环: {report['triangle_closure']['after_mm']:.2f}mm "
          f"{report['triangle_closure']['after_deg']:.3f}°")
    for pk, pr in pair_results.items():
        if pr.get("status") == "PASS":
            print(f"  {pk}: n={pr['n_correspondences']} "
                  f"res_before={pr['residual_before']['median_mm']:.2f}mm "
                  f"res_after={pr['residual_after']['median_mm']:.2f}mm")
        else:
            print(f"  {pk}: {pr.get('status')} ({pr.get('reason', '?')})")
    print(f"\n📁 {args.output}")


if __name__ == "__main__":
    main()

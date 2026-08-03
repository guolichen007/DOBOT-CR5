#!/usr/bin/env python3
"""
CR5 Reconstruction — 数据集契约验证脚本.

验证三相机同步采集数据是否满足重建契约:
  - 必需文件齐全
  - 图像尺寸与 CameraInfo 精确一致
  - CameraInfo K 合法 (3×3, finite, fx/fy>0)
  - depth dtype 和单位
  - frame_id 与 calibrated_rig optical_frame 一致
  - depth-color 配准状态 (COLOCATED/REGISTERED)
  - 三相机时间偏差 (优先 group_manifest.json)

用法:
  rosrun cr5_spray_perception validate_reconstruction_dataset.py \
    --dataset ~/cr5_data/reconstruction/runs/run_001 \
    --rig ~/cr5_data/reconstruction/calibrated_rig.yaml \
    --group-id 0
"""

import os, sys, argparse, json, logging
from typing import List

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("validate_dataset")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract, validate_sync_group, RGBDData,
)
from cr5_spray_perception.reconstruction.extrinsics import load_calibrated_rig
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest,
)


def main():
    parser = argparse.ArgumentParser(
        description="验证三相机重建数据集契约")
    parser.add_argument("--dataset", "-d", required=True,
                        help="数据集路径 (run 根目录 / groups 根目录 / 直接 group 目录)")
    parser.add_argument("--rig", "-r", required=True,
                        help="calibrated_rig.yaml 路径")
    parser.add_argument("--group-id", "-g", type=int, default=0,
                        help="验证的 group ID (默认 0)")
    parser.add_argument("--max-skew-ms", type=float, default=5.0,
                        help="最大跨相机时间偏差 (ms)")
    parser.add_argument("--allow-legacy-fallback", action="store_true", default=False,
                        help="允许降级到 quality.yaml 时间戳 (无 manifest 时)")
    args = parser.parse_args()

    # 加载 calibrated_rig
    logger.info("加载 calibrated_rig: %s", args.rig)
    rig = load_calibrated_rig(args.rig)
    optical_frames = {
        cam: rig["cameras"][cam]["optical_frame"]
        for cam in REQUIRED_CAMERAS
    }
    logger.info("rig_frame: %s", rig["rig_frame"])

    # 解析数据集路径
    logger.info("解析数据集路径: %s (group %d)", args.dataset, args.group_id)
    try:
        layout = resolve_capture_group(args.dataset, args.group_id)
    except FileNotFoundError as e:
        logger.error(str(e))
        sys.exit(1)
    logger.info("group_dir: %s", layout.group_dir)

    # 加载 manifest
    manifest = load_group_manifest(layout.group_dir)
    if manifest:
        logger.info("group_manifest.json: success=%s, captured=%d/%d, method=%s, skew=%.2fms",
                    manifest.get("success"),
                    manifest.get("captured", 0), manifest.get("expected", 0),
                    manifest.get("cross_camera_sync", {}).get("method", "?"),
                    manifest.get("cross_camera_sync", {}).get("max_inter_camera_skew_s", 0) * 1000)
    else:
        logger.warning("group_manifest.json 不存在")
        if not args.allow_legacy_fallback:
            logger.error("正式 Gate 要求 manifest 同步证据. 使用 --allow-legacy-fallback 可降级.")
            sys.exit(1)

    # 加载三台相机数据
    rgbd_list: List[RGBDData] = []
    all_passed = True

    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name)
        if cam_dir is None:
            cam_dir = os.path.join(layout.group_dir, cam_name)

        logger.info("--- %s ---", cam_name)
        if not os.path.isdir(cam_dir):
            logger.error("相机目录不存在: %s", cam_dir)
            all_passed = False
            continue

        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True)

        if not rgbd.is_valid:
            for e in rgbd.errors:
                logger.error("  ❌ %s", e)
            all_passed = False
        else:
            logger.info("  ✅ color: %dx%d (cinfo: %dx%d), frame=%s",
                        rgbd.color_width, rgbd.color_height,
                        rgbd.color_cinfo_width, rgbd.color_cinfo_height,
                        rgbd.color_frame_id)
            logger.info("  ✅ depth: %dx%d (cinfo: %dx%d), dtype=%s, unit=%s, frame=%s",
                        rgbd.depth_width, rgbd.depth_height,
                        rgbd.depth_cinfo_width, rgbd.depth_cinfo_height,
                        rgbd.depth_raw.dtype, rgbd.depth_unit, rgbd.depth_frame_id)
            logger.info("  ✅ color K: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        rgbd.color_K[0,0], rgbd.color_K[1,1],
                        rgbd.color_K[0,2], rgbd.color_K[1,2])
            logger.info("  ✅ depth K: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        rgbd.depth_K[0,0], rgbd.depth_K[1,1],
                        rgbd.depth_K[0,2], rgbd.depth_K[1,2])

            if rgbd.registration:
                reg = rgbd.registration
                logger.info("  ✅ 配准: status=%s, t=%.4fmm, r=%.4f°, evidence=%s",
                            reg.status, reg.translation_mm, reg.rotation_deg,
                            reg.evidence_source)
            else:
                logger.warning("  ⚠️ 无配准状态")

        for w in rgbd.warnings:
            logger.warning("  ⚠️ %s", w)

        rgbd_list.append(rgbd)

    # 跨相机同步验证
    logger.info("--- 跨相机同步 ---")
    passed, errors, warnings, sync_report = validate_sync_group(
        rgbd_list, REQUIRED_CAMERAS,
        max_inter_camera_skew_ms=args.max_skew_ms,
        calibrated_optical_frames=optical_frames,
        manifest=manifest,
        allow_legacy_fallback=args.allow_legacy_fallback,
    )

    for e in errors:
        logger.error("  ❌ %s", e)
    for w in warnings:
        logger.warning("  ⚠️ %s", w)

    if not passed:
        all_passed = False

    # optical_frame 一致性检查
    logger.info("--- optical_frame 一致性 ---")
    for rgbd in rgbd_list:
        expected = optical_frames.get(rgbd.camera_name, "?")
        actual = rgbd.color_frame_id
        match = "✅" if actual == expected else "❌"
        logger.info("  %s %s: expected=%s, actual=%s", match, rgbd.camera_name, expected, actual)
        if actual != expected:
            all_passed = False

    # 写入 validation_report.json
    report_path = os.path.join(args.dataset, "validation_report.json")
    report = {
        "dataset": os.path.abspath(args.dataset),
        "group_id": args.group_id,
        "group_dir": layout.group_dir,
        "layout_type": layout.layout_type,
        "passed": all_passed,
        "cameras": {
            r.camera_name: {
                "valid": r.is_valid,
                "errors": r.errors,
                "warnings": r.warnings,
                "color": {
                    "width": r.color_width, "height": r.color_height,
                    "cinfo_width": r.color_cinfo_width,
                    "cinfo_height": r.color_cinfo_height,
                    "frame_id": r.color_frame_id,
                },
                "depth": {
                    "width": r.depth_width, "height": r.depth_height,
                    "cinfo_width": r.depth_cinfo_width,
                    "cinfo_height": r.depth_cinfo_height,
                    "dtype": str(r.depth_raw.dtype) if r.depth_raw is not None else "N/A",
                    "unit": r.depth_unit,
                    "frame_id": r.depth_frame_id,
                },
                "registration": {
                    "status": r.registration.status if r.registration else "unknown",
                    "translation_mm": r.registration.translation_mm if r.registration else None,
                    "rotation_deg": r.registration.rotation_deg if r.registration else None,
                } if r.registration else None,
            } for r in rgbd_list
        },
        "sync": {
            "source": sync_report["source"],
            "method": sync_report["method"],
            "actual_skew_ms": sync_report["actual_skew_ms"],
            "maximum_allowed_ms": sync_report["maximum_allowed_ms"],
            "passed": sync_report["passed"],
        },
    }
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)
    logger.info("验证报告已保存: %s", report_path)

    print("\n" + "=" * 50)
    if all_passed:
        print("✅ 数据集契约验证通过!")
    else:
        print("❌ 数据集契约验证失败!")
    print(f"   同步源: {sync_report['source']}, 偏差: {sync_report.get('actual_skew_ms', 'N/A')} ms")
    print("=" * 50)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()

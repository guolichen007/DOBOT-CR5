#!/usr/bin/env python3
"""
CR5 Reconstruction — 数据集契约验证脚本.

验证三相机同步采集数据是否满足重建契约:
  - 必需文件齐全 (color.png, depth.npy, CameraInfo, quality.yaml)
  - 图像尺寸与 CameraInfo 一致
  - 深度 dtype 和单位
  - CameraInfo K 有效
  - frame_id 与 calibrated_rig optical_frame 一致
  - depth 是否已对齐到 color
  - 三相机时间偏差

用法:
  rosrun cr5_spray_perception validate_reconstruction_dataset.py \
    --dataset ~/cr5_data/reconstruction/runs/run_001 \
    --rig ~/cr5_data/reconstruction/calibrated_rig.yaml \
    --group-id 0
"""

import os, sys, argparse, json, yaml, logging
from typing import List

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("validate_dataset")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract, validate_sync_group, RGBDData,
)
from cr5_spray_perception.reconstruction.extrinsics import (
    load_calibrated_rig,
)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS


def main():
    parser = argparse.ArgumentParser(
        description="验证三相机重建数据集契约")
    parser.add_argument("--dataset", "-d", required=True,
                        help="数据集根目录 (包含 groups/ 或 views/ 子目录)")
    parser.add_argument("--rig", "-r", required=True,
                        help="calibrated_rig.yaml 路径")
    parser.add_argument("--group-id", "-g", type=int, default=0,
                        help="验证的 group ID (默认 0)")
    parser.add_argument("--require-registered", action="store_true", default=True,
                        help="要求 depth 已对齐到 color (默认 True)")
    parser.add_argument("--max-skew-ms", type=float, default=5.0,
                        help="最大跨相机时间偏差 (ms)")
    args = parser.parse_args()

    # 加载 calibrated_rig
    logger.info("加载 calibrated_rig: %s", args.rig)
    rig = load_calibrated_rig(args.rig)
    optical_frames = {
        cam: rig["cameras"][cam]["optical_frame"]
        for cam in REQUIRED_CAMERAS
    }
    logger.info("rig_frame: %s", rig["rig_frame"])

    # 确定数据集结构
    group_dir = os.path.join(args.dataset, f"group_{args.group_id:04d}")
    if not os.path.isdir(group_dir):
        # 尝试 views 格式
        group_dir = os.path.join(args.dataset, f"view_{args.group_id:04d}")
    if not os.path.isdir(group_dir):
        logger.error("数据集目录不存在: %s (tried group_%04d and view_%04d)",
                     args.dataset, args.group_id, args.group_id)
        sys.exit(1)

    logger.info("数据集目录: %s", group_dir)

    # 加载三台相机数据
    rgbd_list: List[RGBDData] = []
    all_passed = True

    for cam_name in REQUIRED_CAMERAS:
        cam_dir = os.path.join(group_dir, cam_name)
        if not os.path.isdir(cam_dir):
            logger.error("相机目录不存在: %s", cam_dir)
            all_passed = False
            continue

        logger.info("--- %s ---", cam_name)
        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(
            rgbd, require_registered_to_color=args.require_registered)

        if not rgbd.is_valid:
            for e in rgbd.errors:
                logger.error("  ❌ %s", e)
            all_passed = False
        else:
            logger.info("  ✅ color: %dx%d, frame=%s",
                        rgbd.color_width, rgbd.color_height, rgbd.color_frame_id)
            logger.info("  ✅ depth: %dx%d, dtype=%s, unit=%s, frame=%s",
                        rgbd.depth_width, rgbd.depth_height,
                        rgbd.depth_raw.dtype, rgbd.depth_unit, rgbd.depth_frame_id)
            logger.info("  ✅ color K: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        rgbd.color_K[0, 0], rgbd.color_K[1, 1],
                        rgbd.color_K[0, 2], rgbd.color_K[1, 2])
            logger.info("  ✅ depth K: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                        rgbd.depth_K[0, 0], rgbd.depth_K[1, 1],
                        rgbd.depth_K[0, 2], rgbd.depth_K[1, 2])
            if rgbd.depth_registered_to_color:
                logger.info("  ✅ depth 已对齐到 color")
            else:
                logger.warning("  ⚠️  depth 未对齐到 color")

        for w in rgbd.warnings:
            logger.warning("  ⚠️  %s", w)

        rgbd_list.append(rgbd)

    # 跨相机同步验证
    logger.info("--- 跨相机同步 ---")
    passed, errors, warnings = validate_sync_group(
        rgbd_list, REQUIRED_CAMERAS,
        max_inter_camera_skew_ms=args.max_skew_ms,
        calibrated_optical_frames=optical_frames,
    )

    for e in errors:
        logger.error("  ❌ %s", e)
    for w in warnings:
        logger.warning("  ⚠️  %s", w)

    if not passed:
        all_passed = False

    # optical_frame 一致性检查
    logger.info("--- optical_frame 一致性 ---")
    for rgbd in rgbd_list:
        expected = optical_frames.get(rgbd.camera_name, "?")
        actual = rgbd.color_frame_id
        match = "✅" if actual == expected else "❌"
        logger.info("  %s %s: expected=%s, actual=%s",
                    match, rgbd.camera_name, expected, actual)
        if actual != expected:
            all_passed = False

    # 最终结果
    with open(os.path.join(args.dataset, "validation_report.json"), "w") as f:
        report = {
            "dataset": os.path.abspath(args.dataset),
            "group_id": args.group_id,
            "passed": all_passed,
            "cameras": {
                r.camera_name: {
                    "valid": r.is_valid,
                    "errors": r.errors,
                    "warnings": r.warnings,
                    "color": {
                        "width": r.color_width, "height": r.color_height,
                        "frame_id": r.color_frame_id,
                    },
                    "depth": {
                        "width": r.depth_width, "height": r.depth_height,
                        "dtype": str(r.depth_raw.dtype) if r.depth_raw is not None else "N/A",
                        "unit": r.depth_unit,
                        "frame_id": r.depth_frame_id,
                        "registered_to_color": r.depth_registered_to_color,
                    },
                } for r in rgbd_list
            },
            "sync": {"passed": passed, "errors": errors, "warnings": warnings},
        }
        json.dump(report, f, indent=2, default=str)

    print("\n" + "=" * 50)
    if all_passed:
        print("✅ 数据集契约验证通过!")
    else:
        print("❌ 数据集契约验证失败!")
    print("=" * 50)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()

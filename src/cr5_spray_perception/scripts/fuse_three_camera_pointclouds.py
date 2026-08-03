#!/usr/bin/env python3
"""
CR5 Reconstruction — 三相机点云融合脚本.

使用 Stable V1 calibrated_rig 外参, 将三台相机的深度图反投影并转换到
rig frame, 输出独立 PLY、按相机着色融合 PLY (raw+deduplicated)、
RGB 融合 PLY、以及 raw + common 重叠度量.

生产链路: 禁止读取 T_world_camera.yaml / Gazebo TF / gazebo_msgs.
默认禁止 ICP 对齐.

用法:
  rosrun cr5_spray_perception fuse_three_camera_pointclouds.py \
    --dataset ~/cr5_data/reconstruction/runs/run_001 \
    --rig ~/cr5_data/reconstruction/calibrated_rig.yaml \
    --config $(rospack find cr5_spray_perception)/config/reconstruction/three_camera_reconstruction.yaml \
    --output ~/cr5_data/reconstruction/output/run_001_fused \
    --group-id 0
"""

import os, sys, argparse, json, yaml, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("fuse_pointclouds")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract, RGBDData,
)
from cr5_spray_perception.reconstruction.extrinsics import load_calibrated_rig
from cr5_spray_perception.reconstruction.pointcloud_fusion import (
    fuse_three_camera_pointclouds, CAMERA_COLORS, OVERLAP_PAIRS,
)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest, DatasetLayout,
)


def _merge_config_with_cli(config_file: str, args) -> tuple:
    """统一 CLI/YAML 优先级: 显式 CLI > YAML > 默认值.

    Returns:
        (config_dict, source_map) — source_map 记录每个参数来源.
    """
    # 加载 YAML
    if config_file and os.path.isfile(config_file):
        with open(config_file, "r") as f:
            config = yaml.safe_load(f) or {}
        logger.info("配置已加载: %s", config_file)
    else:
        config = {}

    sources = {}

    # CLI 覆盖参数 (仅当显式传入时, 非 None 值)
    cli_overrides = {
        "depth_min_m": args.depth_min,
        "depth_max_m": args.depth_max,
        "voxel_downsample_m": args.voxel_size,
    }

    for key, val in cli_overrides.items():
        if val is not None:
            config[key] = val
            sources[key] = "cli"

    # roi_min / roi_max (仅当非默认值或显式传入)
    if args.roi_min != [-0.5, -0.5, 0.0] or hasattr(args, '_explicit_roi_min'):
        config.setdefault("roi_rig", {})["min"] = list(args.roi_min)
        sources["roi_rig.min"] = "cli"
    if args.roi_max != [1.5, 0.5, 2.0] or hasattr(args, '_explicit_roi_max'):
        config.setdefault("roi_rig", {})["max"] = list(args.roi_max)
        sources["roi_rig.max"] = "cli"

    # 设置默认值 (未被 YAML 或 CLI 覆盖的)
    config.setdefault("camera_names", REQUIRED_CAMERAS)
    config.setdefault("overlap_thresholds_mm", [10, 20, 30])
    config.setdefault("common_overlap_max_distance_m", 0.05)
    config.setdefault("min_common_points", 1000)

    if "roi_rig" not in config:
        config["roi_rig"] = {"min": [-0.5, -1.0, 0.0], "max": [2.0, 1.0, 2.5]}

    # 确定所有参数来源 (用于可追溯性)
    full_sources = {}
    for key in config:
        if key in sources:
            full_sources[key] = sources[key]
        elif key not in cli_overrides or cli_overrides[key] is None:
            full_sources[key] = "yaml" if config_file else "default"
        else:
            full_sources[key] = "cli"

    config["_effective_config_sources"] = full_sources
    return config, full_sources


def print_overlap_summary(raw_metrics: dict, common_metrics: dict):
    """打印重叠指标摘要 (raw + common)."""
    print("\n=== Raw Pairwise 重叠指标 ===")
    _print_metrics_dict(raw_metrics)

    print("\n=== Common Overlap 指标 (共同可见区域) ===")
    for pair_key, pair_data in sorted(common_metrics.items()):
        print(f"\n{pair_key}:")
        support = pair_data.get("support", {})
        print(f"  支持: A={support.get('n_common_a',0)}/{support.get('n_a_total',0)} "
              f"({support.get('common_ratio_a',0)*100:.1f}%), "
              f"B={support.get('n_common_b',0)}/{support.get('n_b_total',0)} "
              f"({support.get('common_ratio_b',0)*100:.1f}%)")
        cm = pair_data.get("common_metrics", {})
        for direction, d in cm.items():
            if isinstance(d, dict) and "median_mm" in d:
                print(f"  {direction}: median={d.get('median_mm',0):.2f}mm  "
                      f"p95={d.get('p95_mm',0):.2f}mm  "
                      f"rmse={d.get('rmse_mm',0):.2f}mm")
                for k, v in d.items():
                    if k.startswith("coverage_"):
                        print(f"    {k}: {v*100:.1f}%")


def _print_metrics_dict(metrics: dict):
    for pair_key, pair_metrics in sorted(metrics.items()):
        print(f"\n{pair_key}:")
        for direction in sorted(pair_metrics.keys()):
            d = pair_metrics[direction]
            if isinstance(d, dict) and "median_mm" in d:
                print(f"  {direction}: n={d.get('n_source',0)}, "
                      f"median={d.get('median_mm',0):.2f}mm, "
                      f"p95={d.get('p95_mm',0):.2f}mm, "
                      f"rmse={d.get('rmse_mm',0):.2f}mm")


def main():
    parser = argparse.ArgumentParser(
        description="三相机点云融合 — Stable V1 calibrated_rig")
    parser.add_argument("--dataset", "-d", required=True,
                        help="数据集路径 (run 根目录 / groups 根目录 / 直接 group 目录)")
    parser.add_argument("--rig", "-r", required=True,
                        help="calibrated_rig.yaml 路径")
    parser.add_argument("--config", "-c", default=None,
                        help="融合配置 YAML 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出目录")
    parser.add_argument("--group-id", "-g", type=int, default=0,
                        help="融合的 group ID (默认 0)")
    parser.add_argument("--allow-icp", action="store_true", default=False,
                        help="[DIAGNOSTIC ONLY] 允许 ICP")
    # CLI 参数: 默认 None, 只有显式传入时才覆盖 YAML
    parser.add_argument("--depth-min", type=float, default=None,
                        help="最小深度 (m), 覆盖 YAML")
    parser.add_argument("--depth-max", type=float, default=None,
                        help="最大深度 (m), 覆盖 YAML")
    parser.add_argument("--voxel-size", type=float, default=None,
                        help="体素下采样尺寸 (m), 覆盖 YAML")
    parser.add_argument("--roi-min", nargs=3, type=float, default=None,
                        help="rig-frame AABB 最小角 (x y z)")
    parser.add_argument("--roi-max", nargs=3, type=float, default=None,
                        help="rig-frame AABB 最大角 (x y z)")
    args = parser.parse_args()

    # 统一配置优先级
    config, config_sources = _merge_config_with_cli(args.config, args)

    # 解析数据集路径
    logger.info("解析数据集路径: %s (group %d)", args.dataset, args.group_id)
    try:
        layout = resolve_capture_group(args.dataset, args.group_id)
    except FileNotFoundError as e:
        logger.error(str(e))
        sys.exit(1)
    logger.info("  group_dir: %s", layout.group_dir)
    logger.info("  layout_type: %s", layout.layout_type)

    # 加载 calibrated_rig
    logger.info("加载 calibrated_rig: %s", args.rig)
    rig = load_calibrated_rig(args.rig)
    logger.info("  rig_frame: %s, status: %s", rig["rig_frame"], rig["status"])

    # 加载 manifest (如有)
    manifest = load_group_manifest(layout.group_dir)
    if manifest:
        logger.info("  manifest: success=%s, captured=%d/%d, skew=%.2fms",
                    manifest.get("success"),
                    manifest.get("captured", 0), manifest.get("expected", 0),
                    manifest.get("cross_camera_sync", {}).get("max_inter_camera_skew_s", 0) * 1000)
    else:
        logger.warning("  group_manifest.json 不存在")

    # 加载三台相机数据
    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name)
        if cam_dir is None:
            cam_dir = os.path.join(layout.group_dir, cam_name)
        if not os.path.isdir(cam_dir):
            logger.error("相机目录不存在: %s", cam_dir)
            sys.exit(1)

        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True)

        if not rgbd.is_valid:
            logger.error("%s 数据验证失败:", cam_name)
            for e in rgbd.errors:
                logger.error("  %s", e)
            sys.exit(1)
        for w in rgbd.warnings:
            logger.warning("  [%s] %s", cam_name, w)
        rgbd_list.append(rgbd)

    # 点云融合
    logger.info("开始三相机点云融合...")
    if args.allow_icp:
        logger.warning("⚠️ ICP 模式 — 仅诊断用途!")

    result = fuse_three_camera_pointclouds(
        rgbd_list=rgbd_list,
        calibrated_rig=rig,
        config=config,
        output_dir=args.output,
        allow_icp=args.allow_icp,
    )

    # 输出摘要
    print("\n=== 融合统计 ===")
    for cam_name, stats in result["per_camera_stats"].items():
        print(f"  {cam_name}: raw={stats['n_raw']}, crop={stats['n_after_crop']}, "
              f"final={stats['n_final']}, reg={stats.get('registration_status','?')}")
    print(f"  fused: raw={result['fused_stats']['n_points_raw']}, "
          f"dedup={result['fused_stats']['n_points_deduplicated']}")

    print("\n=== 输出文件 ===")
    for key, path in result["paths"].items():
        print(f"  {key}: {path}")

    print_overlap_summary(
        result.get("raw_pairwise_metrics", {}),
        result.get("common_overlap_metrics", {}))

    # 保存完整结果
    result_path = os.path.join(args.output, "fusion_result.json")
    result_serializable = {
        "paths": result["paths"],
        "per_camera_stats": result["per_camera_stats"],
        "fused_stats": result["fused_stats"],
        "raw_pairwise_metrics": result["raw_pairwise_metrics"],
        "common_overlap_metrics": result["common_overlap_metrics"],
        "config_effective": result["config_effective"],
        "config_sources": config_sources,
        "input_rig_sha256": rig.get("source_calibration", {}).get("sha256", "N/A"),
        "dataset": os.path.abspath(args.dataset),
        "group_id": args.group_id,
        "manifest_available": manifest is not None,
        "allow_icp": args.allow_icp,
    }
    with open(result_path, "w") as f:
        json.dump(result_serializable, f, indent=2, default=str)
    logger.info("融合结果已保存: %s", result_path)

    print("\n✅ 三相机点云融合完成!")


if __name__ == "__main__":
    main()

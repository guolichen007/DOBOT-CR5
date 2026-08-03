#!/usr/bin/env python3
"""
CR5 Reconstruction — 三相机点云融合脚本.

使用 Stable V1 calibrated_rig 外参, 将三台相机的深度图反投影并转换到
rig frame, 输出独立 PLY、按相机着色融合 PLY、RGB 融合 PLY,
以及相机间的重叠度量.

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


def load_config(config_path: str) -> dict:
    """从 YAML 文件加载配置, 同时合并命令行参数."""
    if not os.path.isfile(config_path):
        logger.error("配置文件不存在: %s", config_path)
        sys.exit(1)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    logger.info("配置已加载: %s", config_path)
    return config


def print_overlap_summary(metrics: dict):
    """打印重叠指标摘要."""
    print("\n=== 重叠指标摘要 ===")
    for pair_key, pair_metrics in sorted(metrics.items()):
        print(f"\n{pair_key}:")
        for direction in ["cam_front_left_to_cam_front_right",
                          "cam_front_right_to_cam_front_left",
                          "cam_front_left_to_cam_rear",
                          "cam_rear_to_cam_front_left",
                          "cam_front_right_to_cam_rear",
                          "cam_rear_to_cam_front_right"]:
            if direction in pair_metrics:
                d = pair_metrics[direction]
                print(f"  {direction}:")
                print(f"    n_source={d.get('n_source',0)}, n_target={d.get('n_target',0)}")
                print(f"    mean={d.get('mean_mm',0):.2f}mm  median={d.get('median_mm',0):.2f}mm")
                print(f"    p90={d.get('p90_mm',0):.2f}mm  p95={d.get('p95_mm',0):.2f}mm")
                print(f"    rmse={d.get('rmse_mm',0):.2f}mm")
                for k, v in d.items():
                    if k.startswith("coverage_"):
                        print(f"    {k}: {v*100:.1f}%")
        if "chamfer_mm" in pair_metrics:
            print(f"  chamfer: {pair_metrics['chamfer_mm']:.2f}mm")


def main():
    parser = argparse.ArgumentParser(
        description="三相机点云融合 — Stable V1 calibrated_rig")
    parser.add_argument("--dataset", "-d", required=True,
                        help="数据集根目录")
    parser.add_argument("--rig", "-r", required=True,
                        help="calibrated_rig.yaml 路径")
    parser.add_argument("--config", "-c",
                        help="融合配置 YAML 路径")
    parser.add_argument("--output", "-o", required=True,
                        help="输出目录")
    parser.add_argument("--group-id", "-g", type=int, default=0,
                        help="融合的 group ID (默认 0)")
    parser.add_argument("--allow-icp", action="store_true", default=False,
                        help="[DIAGNOSTIC ONLY] 允许 ICP 对齐 (ICP 结果不作为正式外参)")
    parser.add_argument("--depth-min", type=float, default=0.15,
                        help="最小深度 (m)")
    parser.add_argument("--depth-max", type=float, default=2.0,
                        help="最大深度 (m)")
    parser.add_argument("--voxel-size", type=float, default=0.005,
                        help="体素下采样尺寸 (m)")
    parser.add_argument("--roi-min", nargs=3, type=float,
                        default=[-0.5, -0.5, 0.0],
                        help="rig-frame AABB 最小角 (x y z)")
    parser.add_argument("--roi-max", nargs=3, type=float,
                        default=[1.5, 0.5, 2.0],
                        help="rig-frame AABB 最大角 (x y z)")
    args = parser.parse_args()

    # 加载配置
    if args.config:
        config = load_config(args.config)
    else:
        config = {}
    # CLI 参数覆盖配置文件
    config["depth_min_m"] = args.depth_min
    config["depth_max_m"] = args.depth_max
    config.setdefault("voxel_downsample_m", args.voxel_size)
    config.setdefault("camera_names", REQUIRED_CAMERAS)
    config.setdefault("overlap_thresholds_mm", [10, 20, 30])
    config.setdefault("roi_rig", {"min": list(args.roi_min), "max": list(args.roi_max)})

    # 加载 calibrated_rig
    logger.info("加载 calibrated_rig: %s", args.rig)
    rig = load_calibrated_rig(args.rig)
    logger.info("rig_frame: %s, status: %s", rig["rig_frame"], rig["status"])

    # 确定数据集目录
    group_dir = os.path.join(args.dataset, f"group_{args.group_id:04d}")
    if not os.path.isdir(group_dir):
        group_dir = os.path.join(args.dataset, f"view_{args.group_id:04d}")
    if not os.path.isdir(group_dir):
        logger.error("数据集 group 目录不存在: %s", group_dir)
        sys.exit(1)
    logger.info("数据集: %s", group_dir)

    # 加载三台相机数据
    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = os.path.join(group_dir, cam_name)
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
        rgbd_list.append(rgbd)

    # 点云融合
    logger.info("开始三相机点云融合...")
    if args.allow_icp:
        logger.warning("⚠️  ICP 模式已启用 — 仅用于诊断, ICP 后结果不作为正式外参!")

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
        print(f"  {cam_name}: raw={stats['n_raw']}, "
              f"crop={stats['n_after_crop']}, final={stats['n_final']}")
    print(f"  fused: {result['fused_stats']['n_points']} points")

    print("\n=== 输出文件 ===")
    for key, path in result["paths"].items():
        print(f"  {key}: {path}")

    print_overlap_summary(result["overlap_metrics"])

    # 保存完整结果
    result_path = os.path.join(args.output, "fusion_result.json")
    result_serializable = {
        "paths": result["paths"],
        "per_camera_stats": result["per_camera_stats"],
        "fused_stats": result["fused_stats"],
        "overlap_metrics": result["overlap_metrics"],
        "config_summary": result["config_summary"],
        "input_rig_sha256": rig.get("source_calibration", {}).get("sha256", "N/A"),
        "dataset": os.path.abspath(args.dataset),
        "group_id": args.group_id,
        "allow_icp": args.allow_icp,
    }
    with open(result_path, "w") as f:
        json.dump(result_serializable, f, indent=2, default=str)
    logger.info("融合结果已保存: %s", result_path)

    print("\n✅ 三相机点云融合完成!")
    print(f"   按相机着色: {result['paths']['fused_colored_by_camera']}")
    print(f"   RGB 融合:   {result['paths']['fused_rgb']}")
    print(f"   重叠指标:   {os.path.join(result['paths']['fused_dir'], 'overlap_metrics.json')}")


if __name__ == "__main__":
    main()

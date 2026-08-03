"""
CR5 Reconstruction — 三相机点云融合与重叠度量.

提供:
  - fuse_three_camera_pointclouds: 顶层融合函数
  - compute_common_overlap:        共同可见区域重叠指标
  - CAMERA_COLORS:                 按相机着色的默认颜色

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
禁止默认开启 ICP 对齐。

输出四套 PLY:
  1. fused_colored_by_camera_raw.ply   — 拼接 per-camera 下采样结果, 纯色
  2. fused_colored_by_camera_deduplicated.ply — 整体去重
  3. fused_rgb_raw.ply                 — RGB 颜色拼接
  4. fused_rgb_deduplicated.ply        — RGB 去重

同时输出:
  - raw_pairwise_metrics   (全部 ROI 点云)
  - common_overlap_metrics (仅共同可见区域)
"""

import os, json, logging
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb,
    compute_bidirectional_overlap,
)
from cr5_spray_perception.reconstruction.extrinsics import (
    get_T_rig_camera,
)

logger = logging.getLogger(__name__)

# 按相机着色颜色 (RGB, 0-1 range)
CAMERA_COLORS = {
    "cam_front_left":  [1.0, 0.2, 0.2],  # 红色
    "cam_front_right": [0.2, 1.0, 0.2],  # 绿色
    "cam_rear":        [0.2, 0.4, 1.0],  # 蓝色
}

# 重叠对
OVERLAP_PAIRS = [
    ("cam_front_left", "cam_front_right"),
    ("cam_front_left", "cam_rear"),
    ("cam_front_right", "cam_rear"),
]


def _has_open3d() -> bool:
    try:
        import open3d
        return True
    except ImportError:
        return False


def _write_ply_colored(filepath: str, points: np.ndarray, colors: np.ndarray):
    """写入带颜色的 PLY 点云."""
    if _has_open3d() and points.shape[0] > 0:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
        o3d.io.write_point_cloud(filepath, pcd)
    elif points.shape[0] > 0:
        N = points.shape[0]
        with open(filepath, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {N}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for i in range(N):
                r = int(np.clip(colors[i, 0] * 255, 0, 255))
                g = int(np.clip(colors[i, 1] * 255, 0, 255))
                b = int(np.clip(colors[i, 2] * 255, 0, 255))
                f.write(f"{points[i,0]:.6f} {points[i,1]:.6f} {points[i,2]:.6f} {r} {g} {b}\n")


def _voxel_downsample_with_colors(points: np.ndarray, colors: np.ndarray,
                                   voxel_size_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """体素下采样, 保留颜色."""
    if voxel_size_m <= 0 or points.shape[0] == 0:
        return points, colors
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
        pcd_down = pcd.voxel_down_sample(voxel_size_m)
        return (np.asarray(pcd_down.points, dtype=np.float32),
                np.asarray(pcd_down.colors, dtype=np.float32))
    except ImportError:
        return points, colors


def compute_common_overlap(points_a: np.ndarray, points_b: np.ndarray,
                            max_distance_m: float = 0.05,
                            thresholds_mm: list = None) -> dict:
    """计算共同可见区域的重叠指标.

    方案 A: 互为最近邻 + 距离上限.
      1. A→B 最近邻, B→A 最近邻
      2. 仅保留 mutual nearest neighbor (互为最近邻)
      3. 使用 max_distance_m 宽松过滤
      4. 在该共同支持集上计算 median/P90/P95/coverage

    Args:
        points_a: (N_A, 3) 点云 A.
        points_b: (N_B, 3) 点云 B.
        max_distance_m: 互为最近邻的最大距离 (米).
        thresholds_mm: coverage 阈值列表.

    Returns:
        dict 含 common_metrics, raw_pairwise, support_info.
    """
    if thresholds_mm is None:
        thresholds_mm = [10, 20, 30]

    n_a, n_b = len(points_a), len(points_b)

    if n_a == 0 or n_b == 0:
        return {
            "common_metrics": {},
            "raw_pairwise": {},
            "support": {"n_a_total": n_a, "n_b_total": n_b,
                        "n_common_a": 0, "n_common_b": 0,
                        "common_ratio_a": 0.0, "common_ratio_b": 0.0},
        }

    # 原始双向最近邻 (全部点)
    raw_pairwise = compute_bidirectional_overlap(
        points_a, points_b, "A", "B", thresholds_mm)

    # 共同可见区域: 互为最近邻 + 距离上限
    try:
        from scipy.spatial import KDTree
        tree_a = KDTree(points_a)
        tree_b = KDTree(points_b)

        # A→B 最近邻
        dist_ab, idx_ab = tree_b.query(points_a)
        # B→A 最近邻
        dist_ba, idx_ba = tree_a.query(points_b)

        # 互为最近邻: A中第i点→B中第j点, 且 B中第j点→A中第i点
        mutual_a = np.zeros(n_a, dtype=bool)
        mutual_b = np.zeros(n_b, dtype=bool)

        for i in range(n_a):
            j = idx_ab[i]
            if dist_ab[i] <= max_distance_m and idx_ba[j] == i and dist_ba[j] <= max_distance_m:
                mutual_a[i] = True
                mutual_b[j] = True

        common_a = points_a[mutual_a]
        common_b = points_b[mutual_b]
        n_common_a = len(common_a)
        n_common_b = len(common_b)

        # 在共同支持集上计算表面距离 (A 中共同点到 B)
        if n_common_a > 0 and n_common_b > 0:
            common_dist_ab, _ = tree_b.query(common_a)
            common_dist_ba, _ = tree_a.query(common_b)
        else:
            common_dist_ab = np.array([])
            common_dist_ba = np.array([])
    except ImportError:
        # 无 scipy, 回退到全部点
        logger.warning("scipy 不可用, common overlap 回退到全部点")
        common_dist_ab = np.array([])
        common_dist_ba = np.array([])
        n_common_a, n_common_b = n_a, n_b

    def _stats(dist_m, label=""):
        if len(dist_m) == 0:
            return {"mean_mm": float("nan"), "median_mm": float("nan"),
                    "p90_mm": float("nan"), "p95_mm": float("nan"),
                    "rmse_mm": float("nan"), "n": 0}
        d_mm = dist_m * 1000.0
        r = {
            "mean_mm": float(np.mean(d_mm)),
            "median_mm": float(np.median(d_mm)),
            "p90_mm": float(np.percentile(d_mm, 90)),
            "p95_mm": float(np.percentile(d_mm, 95)),
            "rmse_mm": float(np.sqrt(np.mean(d_mm ** 2))),
            "n": len(d_mm),
        }
        for t in thresholds_mm:
            r[f"coverage_{t}mm"] = float(np.mean(d_mm < t))
        return r

    common_metrics = {
        "A_to_B": _stats(common_dist_ab),
        "B_to_A": _stats(common_dist_ba),
    }
    if (not np.isnan(common_metrics["A_to_B"].get("mean_mm", float("nan")))
            and not np.isnan(common_metrics["B_to_A"].get("mean_mm", float("nan")))):
        common_metrics["chamfer_mm"] = (
            common_metrics["A_to_B"]["mean_mm"] + common_metrics["B_to_A"]["mean_mm"]
        ) / 2.0
    else:
        common_metrics["chamfer_mm"] = float("nan")

    return {
        "common_metrics": common_metrics,
        "raw_pairwise": raw_pairwise,
        "support": {
            "n_a_total": n_a,
            "n_b_total": n_b,
            "n_common_a": n_common_a,
            "n_common_b": n_common_b,
            "common_ratio_a": float(n_common_a / n_a) if n_a > 0 else 0.0,
            "common_ratio_b": float(n_common_b / n_b) if n_b > 0 else 0.0,
        },
    }


def backproject_camera_pointcloud(
    rgbd_data,
    depth_min_m: float,
    depth_max_m: float,
    expected_depth_unit: str = "meter",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """从 RGBDData 反投影生成 camera optical frame 点云.

    Returns:
        (points_cam, valid_mask, colors_rgb).
    """
    from cr5_spray_perception.reconstruction.transforms import (
        convert_depth_to_meters, depth_image_to_pointcloud)

    depth_m, detected_unit = convert_depth_to_meters(
        rgbd_data.depth_raw, expected_depth_unit)

    points_cam, valid_mask = depth_image_to_pointcloud(
        depth_m, rgbd_data.depth_K, depth_min_m, depth_max_m)

    if rgbd_data.color is not None:
        color_rgb = cv2.cvtColor(rgbd_data.color, cv2.COLOR_BGR2RGB)
        colors_rgb = color_rgb[valid_mask].astype(np.float32) / 255.0
    else:
        colors_rgb = np.zeros((points_cam.shape[0], 3), dtype=np.float32)

    return points_cam, valid_mask, colors_rgb


def fuse_three_camera_pointclouds(
    rgbd_list: list,
    calibrated_rig: dict,
    config: dict,
    output_dir: str,
    allow_icp: bool = False,
) -> dict:
    """三相机点云融合主函数.

    流程:
      1. 反投影 → camera optical frame 点云
      2. T_rig_camera 变换 → rig frame
      3. AABB 裁剪
      4. 体素下采样 (per-camera)
      5. 输出 raw/deduplicated PLY (4 套)
      6. 配对 raw + common 重叠度量
    """
    os.makedirs(output_dir, exist_ok=True)

    depth_min_m = config.get("depth_min_m", 0.15)
    depth_max_m = config.get("depth_max_m", 2.0)
    voxel_size_m = config.get("voxel_downsample_m", 0.005)
    roi_min = np.array(config.get("roi_rig", {}).get("min", [-1.0, -1.0, -1.0]))
    roi_max = np.array(config.get("roi_rig", {}).get("max", [2.0, 1.0, 2.0]))
    overlap_thresholds_mm = config.get("overlap_thresholds_mm", [10, 20, 30])
    camera_names = config.get("camera_names",
                              ["cam_front_left", "cam_front_right", "cam_rear"])
    common_overlap_max_dist_m = config.get("common_overlap_max_distance_m", 0.05)
    min_common_points = config.get("min_common_points", 1000)

    rgbd_map = {r.camera_name: r for r in rgbd_list}

    per_camera_pcd_rig = {}
    per_camera_colors_rgb = {}
    per_camera_stats = {}

    all_rig_points_raw = []
    all_camera_colors_raw = []
    all_rgb_colors_raw = []

    for cam_name in camera_names:
        if cam_name not in rgbd_map:
            logger.warning(f"跳过缺失相机: {cam_name}")
            continue

        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(calibrated_rig, cam_name)

        # RGB 配准检查
        rgb_safe = rgbd.rgb_indexing_safe if hasattr(rgbd, 'rgb_indexing_safe') else rgbd.depth_registered_to_color
        if not rgb_safe and rgbd.color is not None:
            logger.warning(f"[{cam_name}] RGB 颜色索引不安全 (status=%s), "
                          "RGB PLY 可能不准确",
                          rgbd.registration.status if rgbd.registration else "unknown")

        logger.info(f"[{cam_name}] 反投影...")
        points_cam, valid_mask, colors_rgb = backproject_camera_pointcloud(
            rgbd, depth_min_m, depth_max_m)
        n_raw = points_cam.shape[0]
        logger.info(f"[{cam_name}] 原始点数: {n_raw}")

        # 变换到 rig frame
        points_rig = transform_pointcloud(points_cam, T_rc)

        # AABB 裁剪
        points_rig_cropped, crop_mask = crop_pointcloud_aabb(points_rig, roi_min, roi_max)
        colors_rgb_cropped = colors_rgb[crop_mask]
        n_cropped = points_rig_cropped.shape[0]
        logger.info(f"[{cam_name}] AABB 裁剪: {n_raw} → {n_cropped}")

        # 体素下采样
        points_rig_final, colors_rgb_final = _voxel_downsample_with_colors(
            points_rig_cropped, colors_rgb_cropped, voxel_size_m)
        n_final = points_rig_final.shape[0]
        if voxel_size_m > 0:
            logger.info(f"[{cam_name}] 下采样: {n_cropped} → {n_final}")

        per_camera_pcd_rig[cam_name] = points_rig_final
        per_camera_colors_rgb[cam_name] = colors_rgb_final
        per_camera_stats[cam_name] = {
            "n_raw": int(n_raw), "n_after_crop": int(n_cropped),
            "n_final": int(n_final),
            "depth_unit": rgbd.depth_unit,
            "registration_status": (
                rgbd.registration.status if rgbd.registration else "unknown"),
        }

        # 累积到 raw 列表
        all_rig_points_raw.append(points_rig_final)
        cam_color = np.tile(
            np.array(CAMERA_COLORS.get(cam_name, [0.5, 0.5, 0.5]), dtype=np.float32),
            (n_final, 1))
        all_camera_colors_raw.append(cam_color)
        all_rgb_colors_raw.append(colors_rgb_final)

    # ── 输出独立 PLY ──
    per_camera_dir = os.path.join(output_dir, "per_camera")
    os.makedirs(per_camera_dir, exist_ok=True)
    for cam_name in camera_names:
        if cam_name in per_camera_pcd_rig:
            ply_path = os.path.join(per_camera_dir, f"{cam_name}.ply")
            _write_ply_colored(ply_path, per_camera_pcd_rig[cam_name],
                             per_camera_colors_rgb[cam_name])

    # ── 融合 PLY (4 套) ──
    fused_dir = os.path.join(output_dir, "fused")
    os.makedirs(fused_dir, exist_ok=True)

    fused_points_raw = np.vstack(all_rig_points_raw) if all_rig_points_raw else np.zeros((0, 3))
    fused_cam_colors_raw = np.vstack(all_camera_colors_raw) if all_camera_colors_raw else np.zeros((0, 3))
    fused_rgb_colors_raw = np.vstack(all_rgb_colors_raw) if all_rgb_colors_raw else np.zeros((0, 3))

    # 1. raw 三色 (不跨相机下采样, 保留纯色)
    raw_colored_path = os.path.join(fused_dir, "fused_colored_by_camera_raw.ply")
    _write_ply_colored(raw_colored_path, fused_points_raw, fused_cam_colors_raw)

    # 2. raw RGB
    raw_rgb_path = os.path.join(fused_dir, "fused_rgb_raw.ply")
    _write_ply_colored(raw_rgb_path, fused_points_raw, fused_rgb_colors_raw)

    # 3. deduplicated 三色
    dedup_voxel = voxel_size_m * 0.5 if voxel_size_m > 0 else 0.005
    fused_points_dedup, fused_cam_colors_dedup = _voxel_downsample_with_colors(
        fused_points_raw, fused_cam_colors_raw, dedup_voxel)
    dedup_colored_path = os.path.join(fused_dir, "fused_colored_by_camera_deduplicated.ply")
    _write_ply_colored(dedup_colored_path, fused_points_dedup, fused_cam_colors_dedup)

    # 4. deduplicated RGB (从原始重建, 避免颜色混合)
    fused_points_dedup2, fused_rgb_colors_dedup = _voxel_downsample_with_colors(
        fused_points_raw, fused_rgb_colors_raw, dedup_voxel)
    dedup_rgb_path = os.path.join(fused_dir, "fused_rgb_deduplicated.ply")
    _write_ply_colored(dedup_rgb_path, fused_points_dedup2, fused_rgb_colors_dedup)

    logger.info(f"融合 PLY: raw={fused_points_raw.shape[0]}, dedup={fused_points_dedup.shape[0]}")

    # ── 重叠度量 (raw + common) ──
    raw_overlap = {}
    common_overlap = {}

    for cam_a, cam_b in OVERLAP_PAIRS:
        if cam_a not in per_camera_pcd_rig or cam_b not in per_camera_pcd_rig:
            continue
        pair_key = f"{cam_a}_{cam_b}"
        logger.info(f"重叠度量: {pair_key}...")

        points_a = per_camera_pcd_rig[cam_a]
        points_b = per_camera_pcd_rig[cam_b]

        # Raw pairwise
        raw_pair = compute_bidirectional_overlap(
            points_a, points_b, cam_a, cam_b, overlap_thresholds_mm)
        raw_overlap[pair_key] = raw_pair

        # Common overlap
        common = compute_common_overlap(
            points_a, points_b,
            max_distance_m=common_overlap_max_dist_m,
            thresholds_mm=overlap_thresholds_mm,
        )
        common_overlap[pair_key] = common

        # 支持度检查
        support = common["support"]
        if support["n_common_a"] < min_common_points or support["n_common_b"] < min_common_points:
            logger.warning(
                f"{pair_key}: 共同支持点不足 (A={support['n_common_a']}, "
                f"B={support['n_common_b']}, min={min_common_points})")

    # ── 构建结果 ──
    result = {
        "paths": {
            "per_camera_dir": per_camera_dir,
            "fused_dir": fused_dir,
            "fused_colored_by_camera_raw": raw_colored_path,
            "fused_colored_by_camera_deduplicated": dedup_colored_path,
            "fused_rgb_raw": raw_rgb_path,
            "fused_rgb_deduplicated": dedup_rgb_path,
        },
        "per_camera_stats": per_camera_stats,
        "fused_stats": {
            "n_points_raw": int(fused_points_raw.shape[0]),
            "n_points_deduplicated": int(fused_points_dedup.shape[0]),
        },
        "raw_pairwise_metrics": raw_overlap,
        "common_overlap_metrics": common_overlap,
        "config_effective": {
            "depth_min_m": depth_min_m,
            "depth_max_m": depth_max_m,
            "voxel_downsample_m": voxel_size_m,
            "roi_min": roi_min.tolist(),
            "roi_max": roi_max.tolist(),
            "allow_icp": allow_icp,
            "common_overlap_max_distance_m": common_overlap_max_dist_m,
            "min_common_points": min_common_points,
        },
    }

    # 保存指标
    metrics_path = os.path.join(fused_dir, "overlap_metrics.json")
    with open(metrics_path, "w") as f:
        json.dump({
            "raw_pairwise_metrics": raw_overlap,
            "common_overlap_metrics": common_overlap,
        }, f, indent=2, default=str)

    return result

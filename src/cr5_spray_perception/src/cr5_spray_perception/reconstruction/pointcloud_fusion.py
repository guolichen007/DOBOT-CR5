"""
CR5 Reconstruction — 三相机点云融合与重叠度量 (v2).

变更:
  - 正式 Gate PASS/FAIL (支持度/median/P95/coverage)
  - 空点云检测与失败
  - PLY readback 验证
  - pixel_correspondence_safe 控制 RGB 输出
  - 移除 ICP 参数

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import os, json, hashlib, logging
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb,
    compute_bidirectional_overlap,
)
from cr5_spray_perception.reconstruction.extrinsics import get_T_rig_camera

logger = logging.getLogger(__name__)

CAMERA_COLORS = {
    "cam_front_left":  [1.0, 0.2, 0.2],
    "cam_front_right": [0.2, 1.0, 0.2],
    "cam_rear":        [0.2, 0.4, 1.0],
}

OVERLAP_PAIRS = [
    ("cam_front_left", "cam_front_right"),
    ("cam_front_left", "cam_rear"),
    ("cam_front_right", "cam_rear"),
]

GATE_DEFAULTS = {
    "max_correspondence_distance_m": 0.05,
    "min_common_points": 1000,
    "min_common_ratio": 0.02,
    "median_gate_mm": 12.0,
    "p95_gate_mm": 30.0,
    "min_coverage_20mm": 0.80,
}


def _has_open3d():
    try:
        import open3d; return True
    except ImportError:
        return False


def _write_ply_colored(filepath, points, colors):
    """写入 PLY, 空点云抛出异常."""
    if points.shape[0] == 0:
        raise ValueError(f"拒绝写入空点云 PLY: {filepath}")
    if _has_open3d():
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
        o3d.io.write_point_cloud(filepath, pcd)
    else:
        N = points.shape[0]
        with open(filepath, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {N}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for i in range(N):
                r, g, b = [int(np.clip(colors[i, j] * 255, 0, 255)) for j in range(3)]
                f.write(f"{points[i,0]:.6f} {points[i,1]:.6f} {points[i,2]:.6f} {r} {g} {b}\n")


def _verify_ply(filepath, expected_points):
    """验证 PLY 文件."""
    result = {"file_exists": os.path.isfile(filepath),
              "file_size_bytes": 0, "readback_points": 0,
              "sha256": "", "points_match": False}
    if not result["file_exists"]:
        return result
    result["file_size_bytes"] = os.path.getsize(filepath)
    if result["file_size_bytes"] < 50:
        return result
    result["sha256"] = hashlib.sha256(open(filepath, "rb").read()).hexdigest()
    if _has_open3d():
        import open3d as o3d
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            result["readback_points"] = len(pcd.points)
            result["points_match"] = (result["readback_points"] == expected_points)
        except Exception:
            pass
    return result


def _voxel_downsample_with_colors(points, colors, voxel_size_m):
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


def compute_common_overlap(points_a, points_b, max_distance_m=0.05, thresholds_mm=None):
    """计算共同可见区域重叠指标."""
    if thresholds_mm is None:
        thresholds_mm = [10, 20, 30]
    n_a, n_b = len(points_a), len(points_b)
    if n_a == 0 or n_b == 0:
        return {"common_metrics": {}, "raw_pairwise": {},
                "support": {"n_a_total": n_a, "n_b_total": n_b,
                            "n_common_a": 0, "n_common_b": 0,
                            "common_ratio_a": 0.0, "common_ratio_b": 0.0}}

    raw_pairwise = compute_bidirectional_overlap(points_a, points_b, "A", "B", thresholds_mm)

    try:
        from scipy.spatial import KDTree
        tree_a, tree_b = KDTree(points_a), KDTree(points_b)
        dist_ab, idx_ab = tree_b.query(points_a)
        dist_ba, idx_ba = tree_a.query(points_b)
        mutual_a = np.zeros(n_a, dtype=bool); mutual_b = np.zeros(n_b, dtype=bool)
        for i in range(n_a):
            j = idx_ab[i]
            if dist_ab[i] <= max_distance_m and idx_ba[j] == i and dist_ba[j] <= max_distance_m:
                mutual_a[i] = True; mutual_b[j] = True
        common_a = points_a[mutual_a]; common_b = points_b[mutual_b]
        n_common_a, n_common_b = len(common_a), len(common_b)
        if n_common_a > 0 and n_common_b > 0:
            common_dist_ab, _ = tree_b.query(common_a)
            common_dist_ba, _ = tree_a.query(common_b)
        else:
            common_dist_ab = np.array([]); common_dist_ba = np.array([])
    except ImportError:
        common_dist_ab = np.array([]); common_dist_ba = np.array([])
        n_common_a, n_common_b = n_a, n_b

    def _stats(dist_m):
        if len(dist_m) == 0:
            return {"mean_mm": float("nan"), "median_mm": float("nan"),
                    "p90_mm": float("nan"), "p95_mm": float("nan"),
                    "rmse_mm": float("nan"), "n": 0}
        d_mm = dist_m * 1000.0
        r = {"mean_mm": float(np.mean(d_mm)), "median_mm": float(np.median(d_mm)),
             "p90_mm": float(np.percentile(d_mm, 90)), "p95_mm": float(np.percentile(d_mm, 95)),
             "rmse_mm": float(np.sqrt(np.mean(d_mm ** 2))), "n": len(d_mm)}
        for t in thresholds_mm:
            r[f"coverage_{t}mm"] = float(np.mean(d_mm < t))
        return r

    common_metrics = {"A_to_B": _stats(common_dist_ab), "B_to_A": _stats(common_dist_ba)}
    ma, mb = common_metrics["A_to_B"].get("mean_mm", float("nan")), common_metrics["B_to_A"].get("mean_mm", float("nan"))
    common_metrics["chamfer_mm"] = (ma + mb) / 2.0 if not np.isnan(ma) and not np.isnan(mb) else float("nan")

    return {"common_metrics": common_metrics, "raw_pairwise": raw_pairwise,
            "support": {"n_a_total": n_a, "n_b_total": n_b,
                        "n_common_a": n_common_a, "n_common_b": n_common_b,
                        "common_ratio_a": float(n_common_a / n_a) if n_a > 0 else 0.0,
                        "common_ratio_b": float(n_common_b / n_b) if n_b > 0 else 0.0}}


def evaluate_gate(common_overlap_metrics, config):
    """评估 common-overlap Gate.

    Returns:
        {"passed": bool, "reasons": [...], "pair_results": {...}}
    """
    gate_cfg = config.get("common_overlap", GATE_DEFAULTS)
    min_points = gate_cfg.get("min_common_points", 1000)
    min_ratio = gate_cfg.get("min_common_ratio", 0.02)
    median_gate = gate_cfg.get("median_gate_mm", 12.0)
    p95_gate = gate_cfg.get("p95_gate_mm", 30.0)
    min_cov_20 = gate_cfg.get("min_coverage_20mm", 0.80)

    reasons = []
    pair_results = {}
    all_passed = True

    for pair_key, pair_data in sorted(common_overlap_metrics.items()):
        support = pair_data.get("support", {})
        cm = pair_data.get("common_metrics", {})
        pair_pass = True
        pr = {"support_passed": True, "median_passed": True, "p95_passed": True,
              "coverage_passed": True, "passed": True}

        n_a, n_b = support.get("n_common_a", 0), support.get("n_common_b", 0)
        ratio_a, ratio_b = support.get("common_ratio_a", 0), support.get("common_ratio_b", 0)

        if n_a < min_points or n_b < min_points:
            reasons.append(f"{pair_key}: 共同点不足 (A={n_a}, B={n_b}, min={min_points})")
            pr["support_passed"] = False; pair_pass = False
        if ratio_a < min_ratio or ratio_b < min_ratio:
            reasons.append(f"{pair_key}: 共同比例不足 (A={ratio_a:.4f}, B={ratio_b:.4f}, min={min_ratio})")
            pr["support_passed"] = False; pair_pass = False

        for direction in ["A_to_B", "B_to_A"]:
            d = cm.get(direction, {})
            med = d.get("median_mm", float("nan"))
            p95 = d.get("p95_mm", float("nan"))
            cov20 = d.get("coverage_20mm", 0.0)

            if np.isnan(med):
                reasons.append(f"{pair_key}/{direction}: median=NaN"); pr["median_passed"] = False; pair_pass = False
            elif med > median_gate:
                reasons.append(f"{pair_key}/{direction}: median={med:.2f}mm > {median_gate}mm")
                pr["median_passed"] = False; pair_pass = False

            if np.isnan(p95):
                reasons.append(f"{pair_key}/{direction}: p95=NaN"); pr["p95_passed"] = False; pair_pass = False
            elif p95 > p95_gate:
                reasons.append(f"{pair_key}/{direction}: p95={p95:.2f}mm > {p95_gate}mm")
                pr["p95_passed"] = False; pair_pass = False

            if np.isnan(cov20) or cov20 < min_cov_20:
                reasons.append(f"{pair_key}/{direction}: coverage@20mm={cov20:.3f} < {min_cov_20}")
                pr["coverage_passed"] = False; pair_pass = False

        pr["passed"] = pair_pass
        pair_results[pair_key] = pr
        if not pair_pass: all_passed = False

    return {"passed": all_passed, "reasons": reasons, "pair_results": pair_results}


def fuse_three_camera_pointclouds(rgbd_list, calibrated_rig, config, output_dir, allow_icp=False):
    """三相机点云融合主函数 (v2)."""
    os.makedirs(output_dir, exist_ok=True)

    depth_min_m = config.get("depth_min_m", 0.15)
    depth_max_m = config.get("depth_max_m", 2.0)
    voxel_size_m = config.get("voxel_downsample_m", 0.005)
    roi_min = np.array(config.get("roi_rig", {}).get("min", [-0.5, -1.0, 0.0]))
    roi_max = np.array(config.get("roi_rig", {}).get("max", [2.0, 1.0, 2.5]))
    overlap_thresholds_mm = config.get("overlap_thresholds_mm", [10, 20, 30])
    camera_names = config.get("camera_names", ["cam_front_left", "cam_front_right", "cam_rear"])
    common_overlap_max_dist_m = config.get("common_overlap", GATE_DEFAULTS).get("max_correspondence_distance_m", 0.05)

    rgbd_map = {r.camera_name: r for r in rgbd_list}
    per_camera_pcd_rig = {}; per_camera_colors_rgb = {}; per_camera_stats = {}
    all_rig_points_raw = []; all_camera_colors_raw = []; all_rgb_colors_raw = []
    any_rgb_unsafe = False

    for cam_name in camera_names:
        if cam_name not in rgbd_map:
            logger.warning(f"跳过缺失相机: {cam_name}"); continue

        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(calibrated_rig, cam_name)
        pixel_safe = rgbd.pixel_correspondence_safe if hasattr(rgbd, 'pixel_correspondence_safe') else rgbd.rgb_indexing_safe
        if not pixel_safe: any_rgb_unsafe = True

        points_cam, valid_mask, colors_rgb = backproject_camera_pointcloud(rgbd, depth_min_m, depth_max_m)
        n_raw = points_cam.shape[0]
        logger.info(f"[{cam_name}] 原始点数: {n_raw}, pixel_safe={pixel_safe}")

        points_rig = transform_pointcloud(points_cam, T_rc)
        points_rig_cropped, crop_mask = crop_pointcloud_aabb(points_rig, roi_min, roi_max)
        colors_rgb_cropped = colors_rgb[crop_mask]
        n_cropped = points_rig_cropped.shape[0]
        logger.info(f"[{cam_name}] AABB: {n_raw} → {n_cropped}")

        points_rig_final, colors_rgb_final = _voxel_downsample_with_colors(points_rig_cropped, colors_rgb_cropped, voxel_size_m)
        n_final = points_rig_final.shape[0]
        if n_final == 0:
            logger.error(f"[{cam_name}] 点云为空! (raw={n_raw}, crop={n_cropped})")

        per_camera_pcd_rig[cam_name] = points_rig_final
        per_camera_colors_rgb[cam_name] = colors_rgb_final
        per_camera_stats[cam_name] = {"n_raw": int(n_raw), "n_after_crop": int(n_cropped),
                                       "n_final": int(n_final), "depth_unit": rgbd.depth_unit,
                                       "pixel_correspondence_safe": pixel_safe,
                                       "registration_status": (rgbd.registration.status if rgbd.registration else "unknown")}

        all_rig_points_raw.append(points_rig_final)
        cam_color = np.tile(np.array(CAMERA_COLORS.get(cam_name, [0.5, 0.5, 0.5]), dtype=np.float32), (n_final, 1))
        all_camera_colors_raw.append(cam_color)
        all_rgb_colors_raw.append(colors_rgb_final)

    # ── 独立 PLY ──
    per_camera_dir = os.path.join(output_dir, "per_camera"); os.makedirs(per_camera_dir, exist_ok=True)
    for cam_name in camera_names:
        if cam_name in per_camera_pcd_rig and per_camera_pcd_rig[cam_name].shape[0] > 0:
            _write_ply_colored(os.path.join(per_camera_dir, f"{cam_name}.ply"),
                              per_camera_pcd_rig[cam_name], per_camera_colors_rgb[cam_name])

    # ── 融合 PLY ──
    fused_dir = os.path.join(output_dir, "fused"); os.makedirs(fused_dir, exist_ok=True)
    fused_points_raw = np.vstack(all_rig_points_raw) if all_rig_points_raw else np.zeros((0, 3))
    fused_cam_colors_raw = np.vstack(all_camera_colors_raw) if all_camera_colors_raw else np.zeros((0, 3))
    fused_rgb_colors_raw = np.vstack(all_rgb_colors_raw) if all_rgb_colors_raw else np.zeros((0, 3))

    if fused_points_raw.shape[0] == 0:
        raise RuntimeError("融合点云为空! 所有相机的有效深度点均为零.")

    # raw 三色 PLY
    raw_colored_path = os.path.join(fused_dir, "fused_colored_by_camera_raw.ply")
    _write_ply_colored(raw_colored_path, fused_points_raw, fused_cam_colors_raw)

    # raw RGB PLY (仅 pixel_safe 时输出)
    raw_rgb_path = os.path.join(fused_dir, "fused_rgb_raw.ply")
    if any_rgb_unsafe:
        logger.error("pixel_correspondence_safe=false → 不生成 RGB PLY!")
        raw_rgb_path = None
    else:
        _write_ply_colored(raw_rgb_path, fused_points_raw, fused_rgb_colors_raw)

    # deduplicated
    dedup_voxel = voxel_size_m * 0.5 if voxel_size_m > 0 else 0.005
    fps, fcs = _voxel_downsample_with_colors(fused_points_raw, fused_cam_colors_raw, dedup_voxel)
    dedup_colored_path = os.path.join(fused_dir, "fused_colored_by_camera_deduplicated.ply")
    _write_ply_colored(dedup_colored_path, fps, fcs)

    dedup_rgb_path = os.path.join(fused_dir, "fused_rgb_deduplicated.ply")
    if any_rgb_unsafe:
        dedup_rgb_path = None
    else:
        fps2, frs2 = _voxel_downsample_with_colors(fused_points_raw, fused_rgb_colors_raw, dedup_voxel)
        _write_ply_colored(dedup_rgb_path, fps2, frs2)

    # ── 重叠度量 ──
    raw_overlap = {}; common_overlap = {}
    for cam_a, cam_b in OVERLAP_PAIRS:
        if cam_a not in per_camera_pcd_rig or cam_b not in per_camera_pcd_rig: continue
        pair_key = f"{cam_a}_{cam_b}"
        points_a = per_camera_pcd_rig[cam_a]; points_b = per_camera_pcd_rig[cam_b]
        raw_overlap[pair_key] = compute_bidirectional_overlap(points_a, points_b, cam_a, cam_b, overlap_thresholds_mm)
        common_overlap[pair_key] = compute_common_overlap(points_a, points_b, common_overlap_max_dist_m, overlap_thresholds_mm)

    # ── Gate ──
    gate = evaluate_gate(common_overlap, config)
    if not gate["passed"]:
        logger.error("Common-overlap Gate FAILED: %s", "; ".join(gate["reasons"]))

    # ── PLY readback 验证 ──
    ply_verification = {}
    for label, path in [("raw_colored", raw_colored_path), ("dedup_colored", dedup_colored_path),
                         ("raw_rgb", raw_rgb_path), ("dedup_rgb", dedup_rgb_path)]:
        if path is not None:
            ply_verification[label] = _verify_ply(path, expected_points=(
                fused_points_raw.shape[0] if "raw" in label else fps.shape[0]))

    result = {"paths": {"per_camera_dir": per_camera_dir, "fused_dir": fused_dir,
                        "fused_colored_by_camera_raw": raw_colored_path,
                        "fused_colored_by_camera_deduplicated": dedup_colored_path,
                        "fused_rgb_raw": raw_rgb_path, "fused_rgb_deduplicated": dedup_rgb_path},
              "per_camera_stats": per_camera_stats,
              "fused_stats": {"n_points_raw": int(fused_points_raw.shape[0]),
                              "n_points_deduplicated": int(fps.shape[0])},
              "raw_pairwise_metrics": raw_overlap,
              "common_overlap_metrics": common_overlap,
              "gate": gate,
              "ply_verification": ply_verification,
              "rgb_output_suppressed": any_rgb_unsafe,
              "config_effective": {"depth_min_m": depth_min_m, "depth_max_m": depth_max_m,
                                   "voxel_downsample_m": voxel_size_m,
                                   "roi_min": roi_min.tolist(), "roi_max": roi_max.tolist(),
                                   "common_overlap_max_distance_m": common_overlap_max_dist_m}}

    metrics_path = os.path.join(fused_dir, "overlap_metrics.json")
    with open(metrics_path, "w") as f:
        json.dump({"raw_pairwise_metrics": raw_overlap, "common_overlap_metrics": common_overlap,
                   "gate": gate}, f, indent=2, default=str)
    return result


def backproject_camera_pointcloud(rgbd_data, depth_min_m, depth_max_m, expected_depth_unit="meter"):
    from cr5_spray_perception.reconstruction.transforms import convert_depth_to_meters, depth_image_to_pointcloud
    depth_m, detected_unit = convert_depth_to_meters(rgbd_data.depth_raw, expected_depth_unit)
    points_cam, valid_mask = depth_image_to_pointcloud(depth_m, rgbd_data.depth_K, depth_min_m, depth_max_m)
    n_pts = points_cam.shape[0]
    if rgbd_data.color is not None and n_pts > 0:
        color_rgb = cv2.cvtColor(rgbd_data.color, cv2.COLOR_BGR2RGB)
        h, w = valid_mask.shape
        if color_rgb.shape[:2] == (h, w):
            colors_rgb = color_rgb[valid_mask].astype(np.float32) / 255.0
        else:
            logger.warning("color/depth 尺寸不匹配: color=%s depth=%s", color_rgb.shape[:2], valid_mask.shape)
            colors_rgb = np.zeros((n_pts, 3), dtype=np.float32)
    else:
        colors_rgb = np.zeros((n_pts, 3), dtype=np.float32)
    return points_cam, valid_mask, colors_rgb

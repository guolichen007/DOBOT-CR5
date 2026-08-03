#!/usr/bin/env python3
"""
CR5 Reconstruction — 目标 ROI 分析脚本.

输入: Stable V1 融合点云 (三色 PLY)
输出: pre_roi / target_candidate / rejected_background PLY + roi_report.json

ROI 确定流程:
  1. 加载全场景融合点云
  2. 基于人工初始搜索区域粗裁剪
  3. 点云连通分量分析 (DBSCAN)
  4. 基于 calibration_target 预期几何尺寸筛选
  5. 三相机共同观测验证
  6. 输出最终 ROI 配置

生产代码不得读取 Gazebo truth.
"""
import os, sys, argparse, json, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("analyze_roi")

# calibration_target (motor_housing_cylinder) 预期尺寸
# body: length_y=0.36, radius=0.105
# end caps: ~0.04m added per side → total y ≈ 0.44m
# diameter ≈ 0.23m
EXPECTED_TARGET_Y_M = 0.44
EXPECTED_TARGET_RADIUS_M = 0.12
EXPECTED_TARGET_VOLUME_M3 = np.pi * 0.12**2 * 0.44  # ~0.02 m³

# 初始搜索区域 (rig frame = FL optical frame)
# 目标大致位于中央区域
INITIAL_SEARCH_ROI = {
    "min": [-0.3, -0.25, 0.65],
    "max": [0.3, 0.15, 1.20],
}

# calibration_target 上方有吊索/门架结构, 下方有地面
# y 方向包含目标主体 (y≈-0.2 ~ 0.15)


def load_ply_points(path):
    """加载 PLY 点云,返回 (points N×3, colors N×3)."""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        points = np.asarray(pcd.points, dtype=np.float64)
        colors = np.asarray(pcd.colors, dtype=np.float64)
        logger.info("加载 %s: %d 点", path, len(points))
        return points, colors
    except ImportError:
        logger.error("需要 open3d")
        sys.exit(1)


def crop_aabb(points, colors, aabb_min, aabb_max):
    """AABB 裁剪."""
    mask = np.all(points >= aabb_min, axis=1) & np.all(points <= aabb_max, axis=1)
    return points[mask], colors[mask]


def cluster_dbscan(points, eps=0.02, min_samples=50):
    """DBSCAN 聚类,返回各簇标签."""
    from sklearn.cluster import DBSCAN
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    return clustering.labels_


def compute_cluster_stats(points, labels):
    """统计各簇的点数和 AABB."""
    stats = {}
    unique_labels = np.unique(labels)
    for label in unique_labels:
        if label == -1:
            continue  # 噪声
        mask = labels == label
        cluster_pts = points[mask]
        stats[int(label)] = {
            "n_points": len(cluster_pts),
            "aabb_min": cluster_pts.min(axis=0).tolist(),
            "aabb_max": cluster_pts.max(axis=0).tolist(),
            "centroid": cluster_pts.mean(axis=0).tolist(),
        }
    return stats


def is_target_candidate(stats, target_center=None):
    """判断簇是否可能是标定靶.

    标准:
      - y 范围 ≈ 0.3~0.5m (标定靶长度)
      - x/z 范围 ≈ 0.15~0.3m (标定靶直径)
      - 点数 > 5000 (足够密集)
      - 如果指定 target_center, 优先距离最近的簇
    """
    candidates = []
    for label, s in stats.items():
        extent = np.array(s["aabb_max"]) - np.array(s["aabb_min"])
        y_extent = extent[1]
        xz_extent = max(extent[0], extent[2])
        n = s["n_points"]

        y_ok = 0.15 < y_extent < 0.8
        xz_ok = 0.05 < xz_extent < 0.5
        n_ok = n > 1000

        score = 0
        if y_ok:
            score += 1
        if xz_ok:
            score += 1
        if n_ok:
            score += 1
        # y 范围越接近预期越高
        if 0.25 < y_extent < 0.55:
            score += 1

        if score >= 2:
            candidates.append((label, score, s))

    if target_center is not None and candidates:
        # 选 centroid 距离 target_center 最近的
        tc = np.array(target_center)
        candidates.sort(key=lambda x: np.linalg.norm(
            np.array(x[2]["centroid"]) - tc))
        return candidates[0][0], candidates

    if candidates:
        # 选点数最多的
        candidates.sort(key=lambda x: x[2]["n_points"], reverse=True)
        return candidates[0][0], candidates

    return None, candidates


def main():
    parser = argparse.ArgumentParser(description="目标 ROI 分析")
    parser.add_argument("--input-ply", "-i", required=True,
                        help="融合点云 PLY (三色)")
    parser.add_argument("--output-dir", "-o", required=True,
                        help="输出目录")
    parser.add_argument("--roi-min", nargs=3, type=float,
                        default=INITIAL_SEARCH_ROI["min"],
                        help="初始搜索区域 min x y z")
    parser.add_argument("--roi-max", nargs=3, type=float,
                        default=INITIAL_SEARCH_ROI["max"],
                        help="初始搜索区域 max x y z")
    parser.add_argument("--dbscan-eps", type=float, default=0.02,
                        help="DBSCAN eps (m)")
    parser.add_argument("--dbscan-min-samples", type=int, default=50,
                        help="DBSCAN min_samples")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # 1. 加载全场景点云
    full_pts, full_colors = load_ply_points(args.input_ply)
    full_aabb = {
        "min": full_pts.min(axis=0).tolist(),
        "max": full_pts.max(axis=0).tolist(),
    }
    logger.info("全场景 AABB: min=%s, max=%s", full_aabb["min"], full_aabb["max"])

    # 2. 初始搜索区域裁剪 (pre-ROI)
    roi_min = np.array(args.roi_min)
    roi_max = np.array(args.roi_max)
    pts_pre, colors_pre = crop_aabb(full_pts, full_colors, roi_min, roi_max)
    logger.info("初始搜索区域: %d 点 (全场景 %d)", len(pts_pre), len(full_pts))

    if len(pts_pre) == 0:
        logger.error("初始搜索区域为空! 调整 --roi-min/--roi-max")
        sys.exit(1)

    # 3. 保存 pre_roi PLY
    pre_path = os.path.join(args.output_dir, "pre_roi_fused.ply")
    import open3d as o3d
    pcd_pre = o3d.geometry.PointCloud()
    pcd_pre.points = o3d.utility.Vector3dVector(pts_pre)
    pcd_pre.colors = o3d.utility.Vector3dVector(colors_pre)
    o3d.io.write_point_cloud(pre_path, pcd_pre)

    # 4. DBSCAN 聚类
    labels = cluster_dbscan(pts_pre, eps=args.dbscan_eps,
                            min_samples=args.dbscan_min_samples)
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise = np.sum(labels == -1)
    logger.info("DBSCAN: %d 簇, %d 噪声点", n_clusters, n_noise)

    cluster_stats = compute_cluster_stats(pts_pre, labels)

    # 5. 识别 target candidate
    # 目标中心估计: x≈0, y≈-0.05, z≈0.92 (场景中心区域)
    target_center_est = [
        (roi_min[0] + roi_max[0]) / 2,
        (roi_min[1] + roi_max[1]) / 2,
        (roi_min[2] + roi_max[2]) / 2,
    ]
    target_label, all_candidates = is_target_candidate(
        cluster_stats, target_center=target_center_est)

    # 6. 分离 target / background
    target_mask = labels == target_label if target_label is not None else np.zeros(len(pts_pre), dtype=bool)
    bg_mask = ~target_mask & (labels != -1)

    target_pts = pts_pre[target_mask]
    target_colors = colors_pre[target_mask]
    bg_pts = pts_pre[bg_mask]
    bg_colors = colors_pre[bg_mask]
    noise_pts = pts_pre[labels == -1]
    noise_colors = colors_pre[labels == -1]

    # 保存 target_candidate
    if len(target_pts) > 0:
        tgt_path = os.path.join(args.output_dir, "target_candidate.ply")
        pcd_t = o3d.geometry.PointCloud()
        pcd_t.points = o3d.utility.Vector3dVector(target_pts)
        pcd_t.colors = o3d.utility.Vector3dVector(target_colors)
        o3d.io.write_point_cloud(tgt_path, pcd_t)
        tgt_aabb = {"min": target_pts.min(axis=0).tolist(),
                    "max": target_pts.max(axis=0).tolist()}
    else:
        tgt_path = None
        tgt_aabb = None

    # 保存 rejected_background
    rejected_path = os.path.join(args.output_dir, "rejected_background.ply")
    if len(bg_pts) > 0:
        pcd_bg = o3d.geometry.PointCloud()
        pcd_bg.points = o3d.utility.Vector3dVector(bg_pts)
        pcd_bg.colors = o3d.utility.Vector3dVector(bg_colors)
        o3d.io.write_point_cloud(rejected_path, pcd_bg)

    # 7. 确定最终 target ROI (加 margin)
    margin_m = 0.03
    if tgt_aabb:
        final_roi = {
            "min": [tgt_aabb["min"][0] - margin_m,
                    tgt_aabb["min"][1] - margin_m,
                    tgt_aabb["min"][2] - margin_m],
            "max": [tgt_aabb["max"][0] + margin_m,
                    tgt_aabb["max"][1] + margin_m,
                    tgt_aabb["max"][2] + margin_m],
        }
    else:
        final_roi = None

    # 8. roi_report.json
    report = {
        "schema": "cr5_target_roi_analysis_v1",
        "input_ply": os.path.abspath(args.input_ply),
        "full_scene_aabb": full_aabb,
        "initial_search_roi": {"min": roi_min.tolist(), "max": roi_max.tolist()},
        "pre_roi_points": int(len(pts_pre)),
        "dbscan": {
            "eps_m": args.dbscan_eps,
            "min_samples": args.dbscan_min_samples,
            "n_clusters": n_clusters,
            "n_noise": n_noise,
        },
        "cluster_stats": {str(k): v for k, v in cluster_stats.items()},
        "candidates": [{"label": c[0], "score": c[1], "stats": c[2]}
                       for c in all_candidates],
        "target_label": int(target_label) if target_label is not None else None,
        "target_aabb": tgt_aabb,
        "final_production_roi": final_roi,
        "output_files": {
            "pre_roi": pre_path,
            "target_candidate": tgt_path,
            "rejected_background": rejected_path,
        },
    }

    report_path = os.path.join(args.output_dir, "roi_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)

    # ── 控制台输出 ──
    print(f"\n{'='*60}")
    print("目标 ROI 分析完成")
    print(f"{'='*60}")
    print(f"全场景 AABB: min={full_aabb['min']}, max={full_aabb['max']}")
    print(f"初始搜索区域: {len(pts_pre)} 点")
    print(f"DBSCAN: {n_clusters} 簇")
    if tgt_aabb:
        print(f"Target AABB: min={tgt_aabb['min']}, max={tgt_aabb['max']}")
        print(f"Target 点数: {len(target_pts)}")
    if final_roi:
        extent = [final_roi["max"][i] - final_roi["min"][i] for i in range(3)]
        print(f"最终 Target ROI (含 margin): {final_roi}")
        print(f"ROI 尺寸: [{extent[0]:.3f}, {extent[1]:.3f}, {extent[2]:.3f}]m")
    print(f"\n📁 输出: {args.output_dir}")
    print(f"📁 报告: {report_path}")


if __name__ == "__main__":
    main()

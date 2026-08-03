#!/usr/bin/env python3
"""误差区域分析 — 重建点按误差大小和表面区域分类."""
import os, sys, json, argparse, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("error_analyze")

try:
    import open3d as o3d
except ImportError:
    logger.error("需要 open3d"); sys.exit(1)

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(PKG_DIR, "src"))


def analyze_errors(recon_mesh_path, gt_pts_rig, output_dir, roi_min, roi_max,
                   sample_n=50000):
    """分析重建误差按区域分布."""
    os.makedirs(output_dir, exist_ok=True)
    np.random.seed(42)

    mesh = o3d.io.read_triangle_mesh(recon_mesh_path)
    recon_pcd = mesh.sample_points_uniformly(number_of_points=sample_n)
    recon_pts = np.asarray(recon_pcd.points)

    gt_pcd = o3d.geometry.PointCloud()
    gt_pcd.points = o3d.utility.Vector3dVector(gt_pts_rig)
    gt_tree = o3d.geometry.KDTreeFlann(gt_pcd)

    # 计算每个重建点到 GT 的距离
    dists = []
    for pt in recon_pts:
        _, idx, dist2 = gt_tree.search_knn_vector_3d(pt, 1)
        dists.append(np.sqrt(dist2[0]))
    dists = np.array(dists) * 1000.0

    # 按误差分层
    bins = [(0, 5, "0_5mm"), (5, 10, "5_10mm"), (10, 20, "10_20mm"), (20, 1e6, "over_20mm")]
    for lo, hi, label in bins:
        mask = (dists >= lo) & (dists < hi)
        if np.any(mask):
            pts = recon_pts[mask]
            colors = np.zeros((len(pts), 3))
            # 红色=高误差
            if label == "0_5mm":
                colors[:, 1] = 1.0  # green
            elif label == "5_10mm":
                colors[:, 0] = 0.5; colors[:, 1] = 0.5  # yellow
            elif label == "10_20mm":
                colors[:, 0] = 1.0; colors[:, 1] = 0.5  # orange
            else:
                colors[:, 0] = 1.0  # red
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud(
                os.path.join(output_dir, f"accuracy_error_{label}.ply"), pcd)

    # 按表面区域分类 (基于 GT centroid 和方向)
    roi_center = (np.array(roi_min) + np.array(roi_max)) / 2.
    regions = {
        "front": lambda p: p[:, 0] > roi_center[0] + 0.05,
        "back": lambda p: p[:, 0] < roi_center[0] - 0.05,
        "left": lambda p: p[:, 1] > roi_center[1] + 0.03,
        "right": lambda p: p[:, 1] < roi_center[1] - 0.03,
        "top": lambda p: p[:, 2] > roi_center[2] + 0.03,
        "observation_boundary": lambda p: (
            (p[:, 0] < roi_center[0] - 0.1) | (p[:, 2] > roi_center[2] + 0.08)),
        "geometric_edges": lambda p: (
            (abs(p[:, 0] - roi_center[0]) < 0.06) &
            (abs(p[:, 1] - roi_center[1]) < 0.03)),
    }

    region_stats = {}
    for region_name, mask_fn in regions.items():
        mask = mask_fn(recon_pts)
        if np.any(mask):
            rd = dists[mask]
            region_stats[region_name] = {
                "n_points": int(np.sum(mask)),
                "median_mm": round(float(np.median(rd)), 2),
                "p95_mm": round(float(np.percentile(rd, 95)), 2),
                "mean_mm": round(float(np.mean(rd)), 2),
                "coverage_5mm": round(float(np.mean(rd < 5)), 3),
                "coverage_10mm": round(float(np.mean(rd < 10)), 3),
                "coverage_20mm": round(float(np.mean(rd < 20)), 3),
            }

    # 全局统计
    global_stats = {
        "n_points": int(len(dists)),
        "median_mm": round(float(np.median(dists)), 2),
        "p95_mm": round(float(np.percentile(dists, 95)), 2),
        "mean_mm": round(float(np.mean(dists)), 2),
        "rmse_mm": round(float(np.sqrt(np.mean(dists**2))), 2),
    }

    # 误差分布直方图
    edges = [0, 3, 5, 8, 10, 15, 20, 30, 50, 100, 1000]
    hist, _ = np.histogram(dists, edges)
    dist_hist = {f"{int(edges[i])}-{int(edges[i+1])}mm": int(hist[i])
                 for i in range(len(edges)-1) if hist[i] > 0}

    report = {
        "global": global_stats,
        "error_distribution": dist_hist,
        "by_region": region_stats,
    }

    report_path = os.path.join(output_dir, "error_analysis.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n误差分析:")
    print(f"  全局: median={global_stats['median_mm']}mm P95={global_stats['p95_mm']}mm")
    print(f"  误差分布: {dist_hist}")
    print(f"\n  区域分析:")
    for rn, rs in sorted(region_stats.items()):
        print(f"    {rn}: n={rs['n_points']} median={rs['median_mm']}mm P95={rs['p95_mm']}mm "
              f"cov5={rs['coverage_5mm']} cov10={rs['coverage_10mm']}")
    print(f"\n📁 {report_path}")

    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--recon-mesh", "-r", required=True)
    parser.add_argument("--gt-ply", "-g", required=True, help="GT PLY (rig frame)")
    parser.add_argument("--output-dir", "-o", required=True)
    parser.add_argument("--roi-min", nargs=3, type=float, default=[-0.281, -0.216, 0.668])
    parser.add_argument("--roi-max", nargs=3, type=float, default=[0.277, 0.179, 1.195])
    parser.add_argument("--samples", type=int, default=50000)
    args = parser.parse_args()

    gt_pcd = o3d.io.read_point_cloud(args.gt_ply)
    gt_pts = np.asarray(gt_pcd.points)
    logger.info("GT: %d pts", len(gt_pts))

    analyze_errors(args.recon_mesh, gt_pts, args.output_dir,
                   args.roi_min, args.roi_max, args.samples)


if __name__ == "__main__":
    main()

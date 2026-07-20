#!/usr/bin/env python3
"""
CR5 Spray Demo: 三维重建质量评估
输出 accuracy/completeness/Chamfer/RMSE/覆盖率等指标
"""
import os
import sys
import yaml
import json
import argparse
import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def evaluate(reconstructed_ply, ground_truth_ply, output_json):
    if not HAS_OPEN3D:
        print(json.dumps({"error": "Open3D not available"}, indent=2))
        sys.exit(1)

    recon = o3d.io.read_triangle_mesh(reconstructed_ply)
    gt = o3d.io.read_triangle_mesh(ground_truth_ply)

    if len(recon.vertices) == 0:
        print(json.dumps({"error": "Empty reconstructed mesh"}, indent=2))
        return

    # Sample points
    n_pts = 50000
    recon_pcd = recon.sample_points_uniformly(n_pts)
    gt_pcd = gt.sample_points_uniformly(n_pts)

    # Accuracy: recon → GT
    dists_recon_to_gt = np.asarray(
        recon_pcd.compute_point_cloud_distance(gt_pcd))
    accuracy_rmse = float(np.sqrt(np.mean(dists_recon_to_gt ** 2)))
    accuracy_median = float(np.median(dists_recon_to_gt))
    accuracy_p90 = float(np.percentile(dists_recon_to_gt, 90))
    accuracy_p95 = float(np.percentile(dists_recon_to_gt, 95))

    # Completeness: GT → recon
    dists_gt_to_recon = np.asarray(
        gt_pcd.compute_point_cloud_distance(recon_pcd))
    completeness_rmse = float(np.sqrt(np.mean(dists_gt_to_recon ** 2)))
    completeness_median = float(np.median(dists_gt_to_recon))
    completeness_p90 = float(np.percentile(dists_gt_to_recon, 90))
    completeness_p95 = float(np.percentile(dists_gt_to_recon, 95))

    # Chamfer
    chamfer = float(np.mean(dists_recon_to_gt) + np.mean(dists_gt_to_recon))

    # Coverage: threshold-based
    threshold = 0.01  # 1cm
    coverage_recon = float(np.mean(dists_recon_to_gt < threshold))
    coverage_gt = float(np.mean(dists_gt_to_recon < threshold))
    coverage_f1 = (2 * coverage_recon * coverage_gt /
                   max(coverage_recon + coverage_gt, 1e-10))

    metrics = {
        "accuracy": {
            "rmse_m": accuracy_rmse,
            "median_m": accuracy_median,
            "p90_m": accuracy_p90,
            "p95_m": accuracy_p95,
        },
        "completeness": {
            "rmse_m": completeness_rmse,
            "median_m": completeness_median,
            "p90_m": completeness_p90,
            "p95_m": completeness_p95,
        },
        "chamfer_distance_m": chamfer,
        "coverage": {
            "reconstruction": round(coverage_recon, 4),
            "ground_truth": round(coverage_gt, 4),
            "f1": round(coverage_f1, 4),
        },
        "mesh_stats": {
            "recon_vertices": len(recon.vertices),
            "recon_triangles": len(recon.triangles),
            "gt_vertices": len(gt.vertices),
            "gt_triangles": len(gt.triangles),
        },
    }

    print(json.dumps(metrics, indent=2))
    with open(output_json, "w") as f:
        json.dump(metrics, f, indent=2)

    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("reconstructed_ply")
    parser.add_argument("ground_truth_ply")
    parser.add_argument("--output", default="metrics.json")
    args = parser.parse_args()
    evaluate(args.reconstructed_ply, args.ground_truth_ply, args.output)

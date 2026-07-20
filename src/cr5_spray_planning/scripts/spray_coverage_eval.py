#!/usr/bin/env python3
"""
CR5 Spray Demo: 虚拟喷涂覆盖率评估
模拟喷枪沿轨迹对地面真值 mesh 的剂量累积。
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


class SprayCoverageEvaluator:
    def __init__(self, config):
        self.config = config

    def evaluate(self, mesh, spray_path, output_dir):
        if not HAS_OPEN3D:
            return None

        spray_radius = self.config.get("spray_radius", 0.03)
        angle_falloff_factor = self.config.get("angle_falloff_factor", 0.5)
        distance_falloff = self.config.get("distance_falloff_m", 0.02)
        speed_m_s = self.config.get("speed_m_s", 0.05)
        dose_per_s = self.config.get("dose_per_s", 1.0)

        mesh.compute_vertex_normals()
        vertices = np.asarray(mesh.vertices)
        normals = np.asarray(mesh.vertex_normals)
        dose = np.zeros(len(vertices))

        # Accumulate dose along path
        for i, wp in enumerate(spray_path):
            pos = np.array(wp["position"])
            normal = np.array(wp["normal"])

            # Distance to all vertices
            dists = np.linalg.norm(vertices - pos, axis=1)

            # Within spray radius
            in_range = dists < spray_radius

            if not np.any(in_range):
                continue

            # Incidence angle factor
            spray_dir = np.array(wp.get("normal", [0, -1, 0]))
            cos_incidence = np.abs(np.dot(normals[in_range], spray_dir))
            angle_factor = np.clip(cos_incidence, 0, 1) ** angle_falloff_factor

            # Distance falloff (linear)
            dist_factor = np.clip(
                1.0 - dists[in_range] / spray_radius, 0, 1)

            # Combined dose from this waypoint
            wp_dose = dose_per_s * angle_factor * dist_factor
            dose[in_range] += wp_dose

        # Compute metrics
        d_threshold = self.config.get("coverage_threshold", 0.5)
        n_total = len(vertices)
        n_covered = np.sum(dose >= d_threshold)
        n_under = np.sum((dose > 0) & (dose < d_threshold))
        n_zero = np.sum(dose == 0)

        metrics = {
            "total_points": int(n_total),
            "covered_points": int(n_covered),
            "under_sprayed_points": int(n_under),
            "unsprayed_points": int(n_zero),
            "coverage_ratio": float(n_covered / n_total),
            "under_spray_ratio": float(n_under / n_total),
            "unsprayed_ratio": float(n_zero / n_total),
            "dose_mean": float(np.mean(dose)),
            "dose_std": float(np.std(dose)),
            "dose_min": float(np.min(dose)),
            "dose_max": float(np.max(dose)),
            "path_length": len(spray_path),
            "config": self.config,
        }

        # Save dose-colored mesh
        if n_total > 0:
            dose_normalized = dose / max(np.max(dose), 1e-10)
            colors = np.zeros((n_total, 3))
            # Red = high dose, Blue = low dose
            colors[:, 0] = dose_normalized  # R
            colors[:, 2] = 1.0 - dose_normalized  # B
            mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_triangle_mesh(
                os.path.join(output_dir, "coverage_mesh.ply"), mesh)

        with open(os.path.join(output_dir, "coverage_metrics.json"), "w") as f:
            json.dump(metrics, f, indent=2)

        return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_file")
    parser.add_argument("spray_path_json")
    parser.add_argument("--output-dir", default=".")
    parser.add_argument("--spray-radius", type=float, default=0.03)
    parser.add_argument("--speed", type=float, default=0.05)
    args = parser.parse_args()

    if not HAS_OPEN3D:
        print(json.dumps({"error": "Open3D not installed"}))
        sys.exit(1)

    config = {
        "spray_radius": args.spray_radius,
        "speed_m_s": args.speed,
        "angle_falloff_factor": 0.5,
        "distance_falloff_m": 0.02,
        "dose_per_s": 1.0,
        "coverage_threshold": 0.5,
    }

    mesh = o3d.io.read_triangle_mesh(args.mesh_file)
    with open(args.spray_path_json) as f:
        path_data = json.load(f)

    evaluator = SprayCoverageEvaluator(config)
    metrics = evaluator.evaluate(mesh, path_data["path"], args.output_dir)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()

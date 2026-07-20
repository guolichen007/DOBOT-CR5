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

        spray_cone_half_angle_deg = self.config.get("spray_cone_half_angle_deg", 15.0)
        stand_off_m = self.config.get("stand_off_m", 0.10)
        speed_m_s = self.config.get("speed_m_s", 0.05)
        dose_per_s = self.config.get("dose_per_s", 1.0)
        angle_falloff_factor = self.config.get("angle_falloff_factor", 0.5)
        gaussian_sigma_factor = self.config.get("gaussian_sigma_factor", 0.33)

        cone_half_angle_rad = np.deg2rad(spray_cone_half_angle_deg)
        # Spray radius at stand-off distance
        spray_radius = stand_off_m * np.tan(cone_half_angle_rad)

        mesh.compute_vertex_normals()
        vertices = np.asarray(mesh.vertices)
        normals = np.asarray(mesh.vertex_normals)
        dose = np.zeros(len(vertices))

        # Build ray casting scene for occlusion
        ray_scene = o3d.t.geometry.RaycastingScene()
        ray_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        ray_scene.add_triangles(ray_mesh)

        # Accumulate dose along trajectory
        for i, wp in enumerate(spray_path):
            nozzle_pos = np.array(wp["position"])
            spray_dir_wp = np.array(wp.get("normal", [0, -1, 0]))
            # Nozzle +Z = -surface_normal → spray direction = -normal (away from surface)
            spray_direction = -spray_dir_wp / np.linalg.norm(spray_dir_wp)

            # Compute segment time (distance to next waypoint / speed)
            if i < len(spray_path) - 1:
                next_pos = np.array(spray_path[i + 1]["position"])
                segment_len = np.linalg.norm(next_pos - nozzle_pos)
            else:
                segment_len = 0.01  # last point, assume small
            dt = segment_len / max(speed_m_s, 1e-6)

            # Ray cast from nozzle to mesh vertices to check occlusion
            for v_idx in range(len(vertices)):
                v_pos = vertices[v_idx]
                ray_dir = v_pos - nozzle_pos
                ray_len = np.linalg.norm(ray_dir)
                if ray_len < 1e-6:
                    continue
                ray_dir /= ray_len

                # Check if ray hits the front face (not back face)
                cos_to_normal = np.dot(ray_dir, normals[v_idx])
                if cos_to_normal > 0:
                    continue  # hitting back face

                # Check occlusion: does anything block the ray?
                rays = o3d.core.Tensor(
                    np.array([[nozzle_pos[0], nozzle_pos[1], nozzle_pos[2],
                               ray_dir[0], ray_dir[1], ray_dir[2]]]),
                    dtype=o3d.core.Dtype.Float32)
                ans = ray_scene.cast_rays(rays)
                hit_dist = ans["t_hit"].numpy()[0, 0]

                # If first hit is approximately this vertex, it's visible
                if abs(hit_dist - ray_len) > 0.01:
                    continue  # occluded or hit something else

                # Angular falloff from cone center
                angle_from_center = np.arccos(np.clip(
                    np.dot(ray_dir, spray_direction), -1, 1))
                if angle_from_center > cone_half_angle_rad * 2:
                    continue  # outside spray cone

                # Gaussian lateral distribution
                lateral_angle_norm = angle_from_center / max(cone_half_angle_rad, 1e-6)
                gaussian_weight = np.exp(
                    -0.5 * (lateral_angle_norm / gaussian_sigma_factor) ** 2)

                # Distance falloff
                dist_factor = np.clip(1.0 - ray_len / (stand_off_m + spray_radius), 0, 1)

                # Incidence angle falloff
                cos_incidence = abs(np.dot(-ray_dir, normals[v_idx]))
                angle_factor = cos_incidence ** angle_falloff_factor

                # Accumulate dose
                dose[v_idx] += dose_per_s * dt * gaussian_weight * dist_factor * angle_factor

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
        "spray_cone_half_angle_deg": 15.0,
        "stand_off_m": args.stand_off,
        "speed_m_s": args.speed,
        "angle_falloff_factor": 0.5,
        "gaussian_sigma_factor": 0.33,
        "dose_per_s": 1.0,
        "coverage_threshold": 0.5,
    }
    # Also update argparser to add stand_off
    if not hasattr(args, 'stand_off') or args.stand_off is None:
        args.stand_off = 0.10

    mesh = o3d.io.read_triangle_mesh(args.mesh_file)
    with open(args.spray_path_json) as f:
        path_data = json.load(f)

    evaluator = SprayCoverageEvaluator(config)
    metrics = evaluator.evaluate(mesh, path_data["path"], args.output_dir)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()

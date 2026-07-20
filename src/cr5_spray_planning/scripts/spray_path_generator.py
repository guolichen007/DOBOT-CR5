#!/usr/bin/env python3
"""
CR5 Spray Demo: 从 Mesh 生成喷涂路径
支持 planar_raster 和 mesh_slice_raster 两种模式。
生成 spray_nozzle_frame 姿态（+Z 指向表面）。
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


class SprayPathGenerator:
    def __init__(self, config):
        self.config = config

    def generate_from_mesh(self, mesh, output_path):
        """Generate spray path from mesh."""
        if not HAS_OPEN3D:
            raise RuntimeError("Open3D required")

        mode = self.config.get("path_mode", "planar_raster")
        line_spacing = self.config.get("line_spacing", 0.02)
        point_spacing = self.config.get("point_spacing", 0.01)
        stand_off = self.config.get("stand_off", 0.10)
        spray_direction = np.array(self.config.get(
            "spray_direction", [0, -1, 0]))  # 期望喷涂方向
        normal_threshold_deg = self.config.get("normal_threshold_deg", 60)

        mesh.compute_vertex_normals()
        vertices = np.asarray(mesh.vertices)
        normals = np.asarray(mesh.vertex_normals)

        # Select spray surface: normal close to -spray_direction
        spray_dir_norm = spray_direction / np.linalg.norm(spray_direction)
        cos_angles = np.dot(normals, -spray_dir_norm)
        threshold = np.cos(np.deg2rad(normal_threshold_deg))
        selected = cos_angles > threshold

        if not np.any(selected):
            raise ValueError("No surface faces spray direction. "
                             "Try adjusting normal_threshold_deg or spray_direction.")

        selected_vertices = vertices[selected]
        selected_normals = normals[selected]

        if mode == "planar_raster":
            return self._planar_raster(selected_vertices, selected_normals,
                                       line_spacing, point_spacing, stand_off,
                                       spray_dir_norm)
        elif mode == "mesh_slice_raster":
            return self._mesh_slice_raster(mesh, line_spacing, point_spacing,
                                           stand_off, spray_dir_norm)
        else:
            raise ValueError("Unknown path_mode: {}".format(mode))

    def _planar_raster(self, vertices, normals, line_spacing, point_spacing,
                       stand_off, spray_dir):
        """Planar raster pattern on selected surface."""
        # Project vertices onto plane perpendicular to spray direction
        # Simple approach: find bounding box in projection plane
        # Build basis vectors
        if abs(spray_dir[0]) < 0.99:
            u = np.cross(spray_dir, [1, 0, 0])
        else:
            u = np.cross(spray_dir, [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(spray_dir, u)
        v /= np.linalg.norm(v)

        proj_u = np.dot(vertices, u)
        proj_v = np.dot(vertices, v)

        u_min, u_max = proj_u.min(), proj_u.max()
        v_min, v_max = proj_v.min(), proj_v.max()

        # Generate grid
        u_vals = np.arange(u_min, u_max + line_spacing, line_spacing)
        v_vals = np.arange(v_min, v_max + point_spacing, point_spacing)

        path = []
        edge_margin = self.config.get("edge_margin", 0.0)
        alternate = self.config.get("alternate_direction", True)

        for i, v_val in enumerate(v_vals):
            # Alternate direction for each line
            u_line = u_vals if (i % 2 == 0 or not alternate) else u_vals[::-1]
            for u_val in u_line:
                # Check if point is within margin
                if edge_margin > 0:
                    if (u_val - u_min < edge_margin or
                        u_max - u_val < edge_margin or
                        v_val - v_min < edge_margin or
                        v_max - v_val < edge_margin):
                        continue

                point = (spray_dir * (np.dot(vertices, spray_dir)).mean()
                         + u * u_val + v * v_val)

                # Find closest normal
                distances = np.linalg.norm(vertices - point, axis=1)
                closest_idx = np.argmin(distances)
                normal = normals[closest_idx]

                # Nozzle pose: +Z = -normal, tangent from path direction
                z_axis = -normal / np.linalg.norm(normal)
                path_dir = u if (i % 2 == 0) else -u
                y_axis = np.cross(z_axis, path_dir)
                y_axis /= np.linalg.norm(y_axis)
                x_axis = np.cross(y_axis, z_axis)

                R = np.column_stack([x_axis, y_axis, z_axis])
                nozzle_pos = point + normal * stand_off

                path.append({
                    "position": nozzle_pos.tolist(),
                    "normal": (-z_axis).tolist(),
                    "rotation_matrix": R.tolist(),
                })

        return path

    def _mesh_slice_raster(self, mesh, line_spacing, point_spacing,
                           stand_off, spray_dir):
        """Mesh slice raster."""
        bbox = mesh.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        # Simplified: fallback to planar
        vertices = np.asarray(mesh.vertices)
        normals = np.asarray(mesh.vertex_normals)
        return self._planar_raster(vertices, normals, line_spacing,
                                   point_spacing, stand_off, spray_dir)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_file", help="Input mesh PLY/OBJ")
    parser.add_argument("--output", default="spray_path.json")
    parser.add_argument("--mode", default="planar_raster",
                        choices=["planar_raster", "mesh_slice_raster"])
    parser.add_argument("--line-spacing", type=float, default=0.02)
    parser.add_argument("--point-spacing", type=float, default=0.01)
    parser.add_argument("--stand-off", type=float, default=0.10)
    parser.add_argument("--spray-dir", nargs=3, type=float,
                        default=[0, -1, 0])
    args = parser.parse_args()

    if not HAS_OPEN3D:
        print(json.dumps({"error": "Open3D not installed"}))
        sys.exit(1)

    config = {
        "path_mode": args.mode,
        "line_spacing": args.line_spacing,
        "point_spacing": args.point_spacing,
        "stand_off": args.stand_off,
        "spray_direction": args.spray_dir,
        "alternate_direction": True,
    }

    mesh = o3d.io.read_triangle_mesh(args.mesh_file)
    gen = SprayPathGenerator(config)
    path = gen.generate_from_mesh(mesh, args.output)

    with open(args.output, "w") as f:
        json.dump({"path": path, "config": config}, f, indent=2)

    print(f"Generated {len(path)} spray points → {args.output}")


if __name__ == "__main__":
    main()

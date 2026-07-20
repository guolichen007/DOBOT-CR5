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
        """Real mesh slice raster: intersect parallel planes with mesh triangles."""
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        mesh.compute_vertex_normals()
        vertex_normals = np.asarray(mesh.vertex_normals)

        # Build per-triangle normals
        tri_normals = np.cross(
            vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
            vertices[triangles[:, 2]] - vertices[triangles[:, 0]])
        tri_normals /= np.linalg.norm(tri_normals, axis=1, keepdims=True) + 1e-10

        # Select triangles facing spray direction
        cos_angles = np.dot(tri_normals, -spray_dir)
        threshold = np.cos(np.deg2rad(self.config.get("normal_threshold_deg", 60)))
        selected_tris = cos_angles > threshold
        if not np.any(selected_tris):
            raise ValueError("No triangles face spray direction")

        # Build largest connected component of selected triangles
        if self.config.get("use_largest_component", True):
            selected_tris = self._largest_component(selected_tris, triangles)

        # Edge margin: shrink selected faces
        edge_margin = self.config.get("edge_margin", 0.0)
        if edge_margin > 0:
            selected_tris = self._apply_edge_margin(
                selected_tris, triangles, vertices, edge_margin)

        # Get bounding box in projection plane
        selected_verts = np.unique(triangles[selected_tris].flatten())
        pts = vertices[selected_verts]

        if abs(spray_dir[0]) < 0.99:
            u_vec = np.cross(spray_dir, [1, 0, 0])
        else:
            u_vec = np.cross(spray_dir, [0, 1, 0])
        u_vec /= np.linalg.norm(u_vec)
        v_vec = np.cross(spray_dir, u_vec)
        v_vec /= np.linalg.norm(v_vec)

        proj_u = np.dot(pts, u_vec)
        proj_v = np.dot(pts, v_vec)

        # Slice planes perpendicular to v direction (scan lines)
        v_min, v_max = proj_v.min(), proj_v.max()
        v_vals = np.arange(v_min, v_max + line_spacing, line_spacing)

        path = []
        alternate = self.config.get("alternate_direction", True)

        for line_idx, v_val in enumerate(v_vals):
            # Intersect slicing plane with each selected triangle
            line_pts = self._slice_plane_triangles(
                v_val, v_vec, spray_dir, vertices, triangles,
                selected_tris, vertex_normals)

            if len(line_pts) < 2:
                continue

            # Sort along u direction
            line_pts = sorted(line_pts, key=lambda p: np.dot(p["pos"], u_vec))

            # Resample at point_spacing
            u_min_line = np.dot(line_pts[0]["pos"], u_vec)
            u_max_line = np.dot(line_pts[-1]["pos"], u_vec)
            n_pts_line = max(2, int((u_max_line - u_min_line) / point_spacing) + 1)

            if alternate and line_idx % 2 == 1:
                sample_range = np.linspace(u_max_line, u_min_line, n_pts_line)
            else:
                sample_range = np.linspace(u_min_line, u_max_line, n_pts_line)

            for u_val in sample_range:
                # Find closest slice point
                dists = [abs(np.dot(p["pos"], u_vec) - u_val) for p in line_pts]
                closest = line_pts[np.argmin(dists)]

                normal = closest["normal"]
                z_axis = -normal / np.linalg.norm(normal)

                # Tangent direction
                direction = (1 if (line_idx % 2 == 0) else -1) * u_vec \
                    if alternate else u_vec

                y_axis = np.cross(z_axis, direction)
                ny = np.linalg.norm(y_axis)
                if ny < 1e-10:
                    y_axis = np.cross(z_axis, np.array([0, 0, 1]))
                    ny = np.linalg.norm(y_axis)
                y_axis /= ny

                x_axis = np.cross(y_axis, z_axis)
                nx = np.linalg.norm(x_axis)
                if nx > 1e-10:
                    x_axis /= nx

                nozzle_pos = closest["pos"] + normal * stand_off
                path.append({
                    "position": nozzle_pos.tolist(),
                    "surface_point": closest["pos"].tolist(),
                    "normal": (-z_axis).tolist(),
                    "tangent": direction.tolist(),
                    "line_id": line_idx,
                    "segment_id": 0,
                })

        return path

    def _slice_plane_triangles(self, v_val, v_vec, spray_dir, vertices,
                                triangles, selected_tris, vertex_normals):
        """Intersect a plane (v = v_val) with selected triangles. Return list of {pos, norm}."""
        points = []
        for tri_idx in np.where(selected_tris)[0]:
            tri = triangles[tri_idx]
            p0, p1, p2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
            n0, n1, n2 = vertex_normals[tri[0]], vertex_normals[tri[1]], vertex_normals[tri[2]]

            d0, d1, d2 = np.dot(p0, v_vec), np.dot(p1, v_vec), np.dot(p2, v_vec)

            # Find intersections
            intersections = []
            for (pa, pb, da, db, na, nb) in [
                (p0, p1, d0, d1, n0, n1),
                (p1, p2, d1, d2, n1, n2),
                (p2, p0, d2, d0, n2, n0),
            ]:
                if (da - v_val) * (db - v_val) < 0:
                    t = (v_val - da) / (db - da + 1e-10)
                    t = np.clip(t, 0, 1)
                    pos = pa + t * (pb - pa)
                    norm = na + t * (nb - na)
                    norm /= np.linalg.norm(norm) + 1e-10
                    intersections.append({"pos": pos, "normal": norm})

            points.extend(intersections)
        return points

    def _largest_component(self, mask, triangles):
        """Return mask for the largest connected component of triangles."""
        n = len(mask)
        adj = {i: set() for i in range(n) if mask[i]}
        edge_to_tris = {}
        for i in adj:
            for j in range(3):
                e = tuple(sorted([triangles[i][j], triangles[i][(j+1)%3]]))
                if e in edge_to_tris:
                    other = edge_to_tris[e]
                    if other in adj:
                        adj[i].add(other)
                        adj[other].add(i)
                else:
                    edge_to_tris[e] = i

        visited = set()
        largest = set()
        for start in adj:
            if start in visited:
                continue
            comp = set()
            stack = [start]
            while stack:
                node = stack.pop()
                if node in comp:
                    continue
                comp.add(node)
                stack.extend(adj[node] - comp)
            visited.update(comp)
            if len(comp) > len(largest):
                largest = comp

        result = np.zeros(n, dtype=bool)
        for i in largest:
            result[i] = True
        return result

    def _apply_edge_margin(self, mask, triangles, vertices, margin):
        """Remove triangles near the edge of the selected region."""
        edge_verts = set()
        for i in np.where(mask)[0]:
            for j in range(3):
                for k in range(3):
                    v1, v2 = triangles[i][j], triangles[i][(j+1)%3]
                    shared = False
                    for other_i in np.where(mask)[0]:
                        if other_i == i:
                            continue
                        if (v1 in triangles[other_i] and v2 in triangles[other_i]):
                            shared = True
                            break
                    if not shared:
                        edge_verts.add(v1)
                        edge_verts.add(v2)

        result = mask.copy()
        for i in np.where(mask)[0]:
            for j in range(3):
                v = triangles[i][j]
                if v in edge_verts:
                    p = vertices[v]
                    for ev in edge_verts:
                        if np.linalg.norm(p - vertices[ev]) < margin:
                            result[i] = False
                            break
        return result


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

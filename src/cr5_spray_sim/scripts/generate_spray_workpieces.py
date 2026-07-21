#!/usr/bin/env python3
"""
Programmatic spray-workpiece mesh generator.
Outputs visual STL, collision STL, ground-truth PLY, and metadata YAML.

Types:
  cabinet_door_panel    — flat panel with edge folds + corner boss
  automotive_fender_panel — parametric curved surface (default demo)
  bumper_corner_panel   — corner piece with high curvature
  geometry_debug_part   — big block + small block (legacy debug)

All units: meters. Normals consistent. No NaN. Repeatable.
"""
import os
import sys
import yaml
import numpy as np
import open3d as o3d


# ── output directory ──
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "meshes", "workpieces")
os.makedirs(OUT_DIR, exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════
# Utility
# ═══════════════════════════════════════════════════════════════════════

def save_meshes(name, visual, collision=None, gt=None, metadata=None):
    """Save visual STL, collision STL, GT PLY, and metadata YAML."""
    base = os.path.join(OUT_DIR, name)

    # Visual (decent resolution)
    vf = os.path.join(OUT_DIR, f"{name}_visual.stl")
    o3d.io.write_triangle_mesh(vf, visual)
    print(f"  visual: {vf}  ({len(visual.vertices)} verts, {len(visual.triangles)} tris)")

    # Collision (simplified)
    if collision is None:
        collision = visual.simplify_quadric_decimation(
            max(4, len(visual.triangles) // 8))
    cf = os.path.join(OUT_DIR, f"{name}_collision.stl")
    o3d.io.write_triangle_mesh(cf, collision)
    print(f"  collision: {cf}  ({len(collision.vertices)} verts, {len(collision.triangles)} tris)")

    # Ground truth (higher resolution PLY)
    if gt is None:
        gt = visual
    gtf = os.path.join(OUT_DIR, f"{name}_gt.ply")
    o3d.io.write_triangle_mesh(gtf, gt)
    print(f"  gt: {gtf}  ({len(gt.vertices)} verts, {len(gt.triangles)} tris)")

    # Metadata
    if metadata is None:
        metadata = {}
    mf = os.path.join(OUT_DIR, f"{name}_metadata.yaml")
    with open(mf, 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False)
    print(f"  metadata: {mf}")


def make_box_mesh(size_x, size_y, size_z):
    """Create an axis-aligned box mesh centered at origin."""
    mesh = o3d.geometry.TriangleMesh.create_box(size_x, size_y, size_z)
    mesh.translate([-size_x/2, -size_y/2, -size_z/2])
    return mesh


def extrude_surface(vertices, triangles, thickness, direction=-1):
    """
    Given a surface mesh (open top), extrude downward/backward
    and add side walls to form a solid.
    direction: -1 extrudes along -Z, 1 extrudes along +Z.
    Returns closed solid mesh.
    """
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.compute_vertex_normals()
    mesh.remove_duplicated_vertices()

    # Bottom surface: copy top, offset by thickness
    n_verts = len(vertices)
    bottom_verts = vertices.copy()
    bottom_verts[:, 2] += direction * thickness

    # Combine vertices
    all_verts = np.vstack([vertices, bottom_verts])
    all_tris = list(triangles)

    # Side walls: for each boundary edge of top surface
    boundary_edges = find_boundary_edges(triangles, n_verts)
    for e in boundary_edges:
        i0, i1 = e
        # Two triangles form the side quad: (i0, i1, i1+n) and (i0, i1+n, i0+n)
        all_tris.append([i0, i1, i1 + n_verts])
        all_tris.append([i0, i1 + n_verts, i0 + n_verts])

    # Bottom triangles (flip winding for downward normal)
    for t in triangles:
        all_tris.append([t[0] + n_verts, t[2] + n_verts, t[1] + n_verts])

    result = o3d.geometry.TriangleMesh()
    result.vertices = o3d.utility.Vector3dVector(np.array(all_verts))
    result.triangles = o3d.utility.Vector3iVector(np.array(all_tris))
    result.compute_vertex_normals()
    result.remove_duplicated_vertices()
    return result


def find_boundary_edges(triangles, n_verts):
    """Find edges that appear exactly once in a triangle list."""
    edge_count = {}
    for t in triangles:
        for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]:
            key = tuple(sorted(e))
            edge_count[key] = edge_count.get(key, 0) + 1
    return [list(k) for k, v in edge_count.items() if v == 1]


# ═══════════════════════════════════════════════════════════════════════
# Parametric Surface
# ═══════════════════════════════════════════════════════════════════════

def parametric_fender_xy(uv, scale_y=0.45, scale_z=0.34):
    """
    Automotive fender surface in the YZ plane.
    u ∈ [-1,1] maps to Y; v ∈ [-1,1] maps to Z.
    X = f(u,v) is the depth (thickness direction).

    X(u,v) = 0.030*u² + 0.018*v² + 0.012*sin(pi*u)*cos(pi*v/2)
    """
    u, v = uv[:, 0], uv[:, 1]
    # Remap to physical dimensions
    y = u * scale_y / 2.0
    z = v * scale_z / 2.0
    x = (0.030 * u**2 +
         0.018 * v**2 +
         0.012 * np.sin(np.pi * u) * np.cos(np.pi * v / 2.0))
    return np.column_stack([x, y, z])


def create_parametric_surface_mesh(res=60):
    """
    Create the automotive fender top surface as a mesh.
    Returns vertices, triangles as numpy arrays.
    Resolution: res x res grid.
    """
    uu = np.linspace(-1, 1, res)
    vv = np.linspace(-1, 1, res)
    verts = []
    for v in vv:
        for u in uu:
            verts.append([u, v])
    uv = np.array(verts)
    pts = parametric_fender_xy(uv)

    # Triangles: (i,j), (i+1,j), (i,j+1) and (i+1,j), (i+1,j+1), (i,j+1)
    tris = []
    for j in range(res - 1):
        for i in range(res - 1):
            a = j * res + i
            b = a + 1
            c = a + res
            d = c + 1
            tris.append([a, b, c])
            tris.append([b, d, c])

    return pts, np.array(tris)


# ═══════════════════════════════════════════════════════════════════════
# Workpiece Generators
# ═══════════════════════════════════════════════════════════════════════

def generate_cabinet_door_panel():
    """
    Flat panel with edge folds and corner boss.
    Bounding box approx: Y=0.42, Z=0.32, X=0.04
    """
    print("\n=== cabinet_door_panel ===")

    wy, wz, tx = 0.42, 0.32, 0.04  # width Y, height Z, thickness X

    # Main panel: thin plate in YZ plane, X is thickness
    main = o3d.geometry.TriangleMesh.create_box(tx, wy, wz)
    main.translate([-tx/2, -wy/2, -wz/2])

    # Edge folds: 4 thin strips along edges
    fold_w = 0.015  # fold width
    fold_t = 0.003  # fold thickness

    # Top fold
    top_fold = o3d.geometry.TriangleMesh.create_box(tx + 0.004, wy, fold_t)
    top_fold.translate([-tx/2 - 0.002, -wy/2, wz/2 - fold_t])
    main += top_fold

    # Bottom fold
    bot_fold = o3d.geometry.TriangleMesh.create_box(tx + 0.004, wy, fold_t)
    bot_fold.translate([-tx/2 - 0.002, -wy/2, -wz/2])
    main += bot_fold

    # Left fold
    left_fold = o3d.geometry.TriangleMesh.create_box(tx + 0.004, fold_t, wz)
    left_fold.translate([-tx/2 - 0.002, -wy/2, -wz/2])
    main += left_fold

    # Right fold
    right_fold = o3d.geometry.TriangleMesh.create_box(tx + 0.004, fold_t, wz)
    right_fold.translate([-tx/2 - 0.002, wy/2 - fold_t, -wz/2])
    main += right_fold

    # Corner boss (small block at top-right corner)
    boss = o3d.geometry.TriangleMesh.create_box(0.025, 0.04, 0.03)
    boss.translate([tx/2 + 0.002, wy/2 - 0.06, wz/2 - 0.03])
    main += boss

    main.compute_vertex_normals()
    main.remove_duplicated_vertices()

    metadata = {
        "name": "cabinet_door_panel",
        "description": "Flat panel with edge folds and corner boss for basic spray validation",
        "bbox_m": {"x": 0.04, "y": 0.42, "z": 0.32},
        "features": ["edge_folds", "corner_boss"],
    }
    save_meshes("cabinet_door_panel", main, metadata=metadata)
    return main


def generate_automotive_fender_panel():
    """
    Parametric curved automotive panel (default demo workpiece).
    Features: bidirectional gentle curvature, one-side flange, shallow rib.
    Bounding box approx: Y=0.45, Z=0.34, X=0.08
    """
    print("\n=== automotive_fender_panel ===")

    # Create parametric top surface
    res = 80
    top_verts, top_tris = create_parametric_surface_mesh(res)

    # Thickness direction is -X (extrude toward negative X)
    thickness = 0.012  # 12 mm panel thickness
    solid = extrude_surface(top_verts, top_tris, thickness, direction=-1)

    # Add flange on right side (v > 0.7 region, Y positive side)
    flange_verts = []
    flange_tris = []
    uu = np.linspace(-1, 1, 40)
    v_start = 0.65
    v_end = 1.05
    vv = np.linspace(v_start, v_end, 10)
    for j, v in enumerate(vv):
        for i, u in enumerate(uu):
            uv_pt = np.array([[u, v]])
            pt = parametric_fender_xy(uv_pt)[0]
            # Extend flange outward
            if v > 0.95:
                pt[1] += 0.012  # slight outward bend
            flange_verts.append(pt)
    fv = np.array(flange_verts)
    for j in range(len(vv) - 1):
        for i in range(len(uu) - 1):
            a = j * len(uu) + i
            b = a + 1
            c = a + len(uu)
            d = c + 1
            flange_tris.append([a, b, c])
            flange_tris.append([b, d, c])

    flange_mesh = o3d.geometry.TriangleMesh()
    flange_mesh.vertices = o3d.utility.Vector3dVector(fv)
    flange_mesh.triangles = o3d.utility.Vector3iVector(np.array(flange_tris))

    # Extrude flange (thin, 3mm)
    flange_solid = extrude_surface(fv, np.array(flange_tris), 0.003, direction=-1)

    # Add shallow stiffening rib (near center)
    rib_center_u, rib_center_v = 0.2, 0.0
    rib_len = 0.20
    rib_h = 0.006
    rib_w = 0.008
    rib_y_start = rib_center_u * 0.45/2 - rib_len/2
    rib = o3d.geometry.TriangleMesh.create_box(rib_h, rib_len, rib_w)
    # Position rib on top surface
    cx = (0.030 * rib_center_u**2 + 0.018 * rib_center_v**2 +
          0.012 * np.sin(np.pi * rib_center_u) * np.cos(np.pi * rib_center_v/2))
    cy = rib_center_u * 0.45 / 2.0
    cz = rib_center_v * 0.34 / 2.0
    rib.translate([cx, cy - rib_len/2, cz - rib_w/2])
    solid += rib

    solid += flange_solid
    solid.compute_vertex_normals()
    solid.remove_duplicated_vertices()

    # Center the mesh at origin
    bbox = solid.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    solid.translate([-center[0], -center[1], -center[2] + 0.02])

    metadata = {
        "name": "automotive_fender_panel",
        "description": "Parametric curved panel simulating automotive body panel with bidirectional curvature, flange, and stiffening rib",
        "bbox_m": {"x": 0.08, "y": 0.45, "z": 0.34},
        "features": ["bidirectional_curvature", "side_flange", "stiffening_rib"],
        "parametric_formula": "X(u,v)=0.030*u^2+0.018*v^2+0.012*sin(pi*u)*cos(pi*v/2), u,v in [-1,1]",
        "thickness_mm": 12,
    }
    save_meshes("automotive_fender_panel", solid, metadata=metadata)
    return solid


def generate_bumper_corner_panel():
    """
    Corner panel with higher curvature variation.
    Simulates bumper corner — more normal variation.
    Bounding box approx: Y=0.42, Z=0.30, X=0.16
    """
    print("\n=== bumper_corner_panel ===")

    # L-shaped corner profile
    res = 70
    uu = np.linspace(-1, 1, res)
    vv = np.linspace(-1, 1, res)

    verts = []
    for v in vv:
        for u in uu:
            uv = np.array([u, v])
            # More aggressive curvature for corner feel
            y = u * 0.42 / 2.0
            z = v * 0.30 / 2.0
            x = (0.045 * u**2 +
                 0.035 * v**2 +
                 0.025 * np.sin(np.pi * u * 0.8) * np.cos(np.pi * v * 0.6) +
                 0.015 * np.sin(2 * np.pi * u) * np.sin(np.pi * v))
            verts.append([x, y, z])
    pts = np.array(verts)

    tris = []
    for j in range(res - 1):
        for i in range(res - 1):
            a = j * res + i
            b = a + 1
            c = a + res
            d = c + 1
            tris.append([a, b, c])
            tris.append([b, d, c])
    top_tris = np.array(tris)

    # Thickness
    thickness = 0.010
    solid = extrude_surface(pts, top_tris, thickness, direction=-1)

    # Add deeper sidewall on one edge (simulating bumper return)
    side_verts = []
    side_tris_list = []
    n_edge = 30
    for i in range(n_edge):
        u = -1.0 + 2.0 * i / (n_edge - 1)
        v = -1.0
        uv = np.array([[u, v]])
        pt = parametric_fender_xy(uv)[0]
        side_verts.append(pt)
        if i < n_edge - 1:
            side_tris_list.append([i, i + 1, i + n_edge])
            side_tris_list.append([i + 1, i + n_edge + 1, i + n_edge])
    # Bottom edge
    for i in range(n_edge):
        u = -1.0 + 2.0 * i / (n_edge - 1)
        v = -1.0
        uv = np.array([[u, v]])
        pt = parametric_fender_xy(uv)[0]
        pt[0] -= 0.025  # deeper extension
        side_verts.append(pt)

    side_mesh = o3d.geometry.TriangleMesh()
    side_mesh.vertices = o3d.utility.Vector3dVector(np.array(side_verts))
    side_mesh.triangles = o3d.utility.Vector3iVector(np.array(side_tris_list))
    solid += side_mesh

    solid.compute_vertex_normals()
    solid.remove_duplicated_vertices()

    # Center
    bbox = solid.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    solid.translate([-center[0], -center[1], -center[2] + 0.01])

    metadata = {
        "name": "bumper_corner_panel",
        "description": "Corner panel with high curvature variation, simulating bumper corner for advanced spray validation",
        "bbox_m": {"x": 0.16, "y": 0.42, "z": 0.30},
        "features": ["high_curvature", "corner_geometry", "deep_sidewall"],
    }
    # Higher-res GT
    gt_res = 120
    gt_uu = np.linspace(-1, 1, gt_res)
    gt_vv = np.linspace(-1, 1, gt_res)
    gt_verts = []
    for v in gt_vv:
        for u in gt_uu:
            y = u * 0.42 / 2.0
            z = v * 0.30 / 2.0
            x = (0.045 * u**2 + 0.035 * v**2 +
                 0.025 * np.sin(np.pi * u * 0.8) * np.cos(np.pi * v * 0.6) +
                 0.015 * np.sin(2 * np.pi * u) * np.sin(np.pi * v))
            gt_verts.append([x, y, z])
    gt_pts = np.array(gt_verts)
    gt_tris_list = []
    for j in range(gt_res - 1):
        for i in range(gt_res - 1):
            a = j * gt_res + i
            b = a + 1
            c = a + gt_res
            d = c + 1
            gt_tris_list.append([a, b, c])
            gt_tris_list.append([b, d, c])
    gt = o3d.geometry.TriangleMesh()
    gt.vertices = o3d.utility.Vector3dVector(gt_pts)
    gt.triangles = o3d.utility.Vector3iVector(np.array(gt_tris_list))
    gt_bbox = gt.get_axis_aligned_bounding_box()
    gt_center = gt_bbox.get_center()
    gt.translate([-gt_center[0], -gt_center[1], -gt_center[2] + 0.01])

    save_meshes("bumper_corner_panel", solid, gt=gt, metadata=metadata)
    return solid


def generate_geometry_debug_part():
    """
    Legacy debug part: big block + small block (formerly block_combo_part).
    Bounding box: Y=0.18, Z=0.40, X=0.28
    """
    print("\n=== geometry_debug_part ===")
    big = make_box_mesh(0.28, 0.18, 0.30)
    big.translate([0, 0, 0.15])

    small = make_box_mesh(0.10, 0.08, 0.10)
    small.translate([0.09, 0.07, 0.32])

    combined = big + small
    combined.compute_vertex_normals()
    combined.remove_duplicated_vertices()

    metadata = {
        "name": "geometry_debug_part",
        "description": "Legacy two-block debug target for basic geometry verification",
        "bbox_m": {"x": 0.28, "y": 0.18, "z": 0.40},
        "features": ["two_block_asymmetric"],
    }
    save_meshes("geometry_debug_part", combined, metadata=metadata)
    return combined


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate spray workpiece meshes")
    parser.add_argument("--all", action="store_true", default=True,
                        help="Generate all workpiece types")
    parser.add_argument("--type", choices=[
        "cabinet_door_panel", "automotive_fender_panel",
        "bumper_corner_panel", "geometry_debug_part"],
        help="Generate a specific workpiece type")
    args = parser.parse_args()

    print(f"Output directory: {OUT_DIR}")

    if args.type:
        generators = {args.type: globals()[f"generate_{args.type}"]}
    else:
        generators = {
            "cabinet_door_panel": generate_cabinet_door_panel,
            "automotive_fender_panel": generate_automotive_fender_panel,
            "bumper_corner_panel": generate_bumper_corner_panel,
            "geometry_debug_part": generate_geometry_debug_part,
        }

    for name, gen_func in generators.items():
        try:
            gen_func()
        except Exception as e:
            print(f"  FAILED {name}: {e}")
            import traceback
            traceback.print_exc()

    print("\n=== All workpieces generated ===")
    print(f"Files in {OUT_DIR}:")
    for f in sorted(os.listdir(OUT_DIR)):
        size = os.path.getsize(os.path.join(OUT_DIR, f))
        print(f"  {f:50s} {size:>10,d} bytes")

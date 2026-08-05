#!/usr/bin/env python3
"""
CR5 Reconstruction — 非目标残留诊断工具 (只读).

分析 TSDF raw mesh 的连通分量, 分类疑似 artifact (钢筋/支架),
生成彩色 PLY 和结构化报告. 不修改任何生产数据.
"""
import os, sys, json, argparse, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("diagnose_artifacts")

try:
    import open3d as o3d
except ImportError:
    logger.error("需要 open3d")
    sys.exit(1)

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))


def compute_obb(mesh_or_pcd):
    """计算 OBB 的主轴长度和方向."""
    if isinstance(mesh_or_pcd, o3d.geometry.TriangleMesh):
        pcd = mesh_or_pcd.sample_points_uniformly(number_of_points=1000)
    else:
        pcd = mesh_or_pcd
    points = np.asarray(pcd.points)
    if len(points) < 3:
        return None
    obb = o3d.geometry.OrientedBoundingBox.create_from_points(
        o3d.utility.Vector3dVector(points))
    extent = obb.extent  # (dx, dy, dz) sorted?
    center = obb.center
    R = obb.R  # 3x3 rotation
    # 排序: 最大轴、次大轴、最小轴
    order = np.argsort(extent)[::-1]
    sorted_extent = extent[order]
    sorted_R = R[:, order]
    return {
        "center": center.tolist(),
        "extent": sorted_extent.tolist(),
        "extent_mm": (sorted_extent * 1000).tolist(),
        "elongation_ratio": float(sorted_extent[0] / max(sorted_extent[1], 1e-6)),
        "R_columns_ordered": [sorted_R[:, i].tolist() for i in range(3)],
    }


def classify_component(comp_info, target_centroid_approx, min_elongation=4.0,
                       max_cross_section_m=0.02, target_envelope_half=0.25):
    """启发式分量分类.

    Args:
        comp_info: 分量信息 dict (含 centroid, obb, surface_area)
        target_centroid_approx: 目标 body 的大致 centroid
        min_elongation: 细长比阈值
        max_cross_section_m: 最大截面尺寸
        target_envelope_half: 目标主体半边长 (m)

    Returns:
        str: 分类标签
    """
    centroid = np.array(comp_info["centroid"])
    area = comp_info.get("surface_area_m2", 0)
    obb = comp_info.get("obb")
    dist_to_target = float(np.linalg.norm(centroid - np.array(target_centroid_approx)))

    # 主分量 — 面积最大
    if comp_info.get("is_largest"):
        return "TARGET_BODY"

    if obb is None:
        return "UNKNOWN_FRAGMENT" if area < 0.001 else "TARGET_BODY"

    elongation = obb["elongation_ratio"]
    cross_section = min(obb["extent"][1], obb["extent"][2])

    # 细长杆状 + 远离主体 → 吊具/钢筋
    if elongation > min_elongation and cross_section < max_cross_section_m \
       and dist_to_target > target_envelope_half:
        return "SUSPENSION_ROD"

    # 小分量 + 在目标 envelope 外且偏后上方
    if area < 0.002 and dist_to_target > target_envelope_half:
        if centroid[0] > 0.8 and abs(centroid[1]) < 0.2:  # rig frame x > 0.8, 后方
            return "REAR_CAMERA_PEDESTAL"
        return "UNKNOWN_FRAGMENT"

    # 顶部小分量
    if area < 0.005 and centroid[2] > 1.05:  # rig frame 上方
        return "SUSPENSION_ROD"

    # 靠近主体的小分量
    if dist_to_target <= target_envelope_half:
        return "TARGET_BODY" if area > 0.0001 else "UNKNOWN_FRAGMENT"

    return "TARGET_BODY" if area > 0.001 else "UNKNOWN_FRAGMENT"


def load_visible_gt(pose_results_dir):
    """加载 visible GT points (rig frame)."""
    paths = [
        os.path.join(pose_results_dir, "visible_evaluation", "visible_gt_rig_frame.ply"),
        os.path.join(pose_results_dir, "visible_evaluation", "gt_rig_frame.ply"),
    ]
    for p in paths:
        if os.path.isfile(p):
            pcd = o3d.io.read_point_cloud(p)
            pts = np.asarray(pcd.points)
            if len(pts) > 0:
                logger.info("  可见 GT: %s (%d pts)", p, len(pts))
                return pts
    return None


def compute_distance_to_gt(comp_vertices, gt_points):
    """计算分量顶点到 GT 的最小/平均距离."""
    if gt_points is None or len(gt_points) == 0:
        return None
    gt_pcd = o3d.geometry.PointCloud()
    gt_pcd.points = o3d.utility.Vector3dVector(gt_points)
    gt_tree = o3d.geometry.KDTreeFlann(gt_pcd)
    dists = []
    for v in comp_vertices:
        _, _, dist2 = gt_tree.search_knn_vector_3d(v, 1)
        dists.append(np.sqrt(dist2[0]))
    dists = np.array(dists) * 1000  # mm
    return {
        "min_mm": float(np.min(dists)),
        "mean_mm": float(np.mean(dists)),
        "median_mm": float(np.median(dists)),
        "max_mm": float(np.max(dists)),
    }


def diagnose_pose(pose_id, run_dir, visible_gt_pts, output_dir):
    """诊断单个姿态的 raw mesh."""
    raw_mesh_path = os.path.join(run_dir, "tsdf", "visible_surface_mesh_raw.ply")
    if not os.path.isfile(raw_mesh_path):
        raw_mesh_path = os.path.join(run_dir, "visible_surface_mesh_raw.ply")
    if not os.path.isfile(raw_mesh_path):
        logger.error("  raw mesh 不存在: %s", run_dir)
        return None

    mesh = o3d.io.read_triangle_mesh(raw_mesh_path)
    logger.info("  raw mesh: %d verts, %d tris", len(mesh.vertices), len(mesh.triangles))

    # 连通分量
    tri_ids, n_tri_per_comp, areas = mesh.cluster_connected_triangles()
    tri_ids = np.asarray(tri_ids)
    n_components = len(n_tri_per_comp)
    logger.info("  分量数: %d", n_components)

    # 按面积排序
    areas_arr = np.asarray(areas)
    order = np.argsort(areas_arr)[::-1]

    components = []
    colors = np.random.RandomState(42).rand(n_components, 3)

    # 目标 body 大致 centroid (从最大分量)
    largest_centroid = None

    for rank, comp_id in enumerate(order):
        if areas_arr[comp_id] < 1e-9:
            continue
        tri_mask = tri_ids == comp_id
        tri_indices = np.where(tri_mask)[0]
        if len(tri_indices) < 10:
            continue

        # 提取分量 mesh
        comp_mesh = o3d.geometry.TriangleMesh()
        verts = np.asarray(mesh.vertices)
        tris = np.asarray(mesh.triangles)
        comp_tris = tris[tri_indices]
        used_verts = np.unique(comp_tris.flatten())
        old_to_new = {old: new for new, old in enumerate(used_verts)}
        new_verts = verts[used_verts]
        new_tris = np.array([[old_to_new[t[0]], old_to_new[t[1]], old_to_new[t[2]]]
                             for t in comp_tris])

        comp_mesh.vertices = o3d.utility.Vector3dVector(new_verts)
        comp_mesh.triangles = o3d.utility.Vector3iVector(new_tris)

        # 质心
        centroid = np.mean(new_verts, axis=0)
        if rank == 0:
            largest_centroid = centroid

        # AABB
        aabb_min = np.min(new_verts, axis=0)
        aabb_max = np.max(new_verts, axis=0)

        # OBB
        obb = compute_obb(comp_mesh)

        # 表面积
        surface_area = comp_mesh.get_surface_area()

        # 距离到 GT
        dist_to_gt = compute_distance_to_gt(new_verts, visible_gt_pts)

        info = {
            "component_id": int(comp_id),
            "rank_by_area": rank,
            "triangles": len(comp_tris),
            "vertices": len(used_verts),
            "surface_area_m2": float(surface_area),
            "area_ratio": float(surface_area / max(mesh.get_surface_area(), 1e-9)),
            "centroid": centroid.tolist(),
            "aabb_min": aabb_min.tolist(),
            "aabb_max": aabb_max.tolist(),
            "aabb_dimensions_mm": ((aabb_max - aabb_min) * 1000).tolist(),
            "obb": obb,
            "distance_to_visible_gt_mm": dist_to_gt,
            "is_largest": (rank == 0),
        }
        components.append(info)

    # 分类
    if largest_centroid is not None:
        for comp in components:
            comp["classification"] = classify_component(
                comp, largest_centroid.tolist())
    else:
        for comp in components:
            comp["classification"] = "UNKNOWN_FRAGMENT"

    # 汇总
    from collections import Counter
    class_counts = Counter(c["classification"] for c in components)

    report = {
        "pose_id": pose_id,
        "run_dir": run_dir,
        "raw_mesh_path": raw_mesh_path,
        "total_vertices": len(mesh.vertices),
        "total_triangles": len(mesh.triangles),
        "component_count": len(components),
        "classification_summary": dict(class_counts),
        "components": components,
    }

    # ── 保存 ──
    os.makedirs(output_dir, exist_ok=True)

    # 彩色分量 PLY (按分类着色)
    classification_colors = {
        "TARGET_BODY": [0.2, 0.8, 0.2],
        "TOP_OFFSET_BLOCK": [1.0, 0.6, 0.0],
        "SUSPENSION_ROD": [1.0, 0.2, 0.2],
        "REAR_CAMERA_PEDESTAL": [0.8, 0.2, 0.8],
        "UNKNOWN_FRAGMENT": [0.5, 0.5, 0.5],
    }
    comp_colors = np.zeros((len(mesh.vertices), 3))
    for comp in components:
        comp_colors[comp["component_id"]] = classification_colors.get(
            comp["classification"], [0.5, 0.5, 0.5])
    colored = o3d.geometry.TriangleMesh()
    colored.vertices = mesh.vertices
    colored.triangles = mesh.triangles
    colored.vertex_colors = o3d.utility.Vector3dVector(comp_colors)
    o3d.io.write_triangle_mesh(
        os.path.join(output_dir, f"{pose_id}_components_colored.ply"), colored)

    # artifact candidates PLY (非 TARGET_BODY)
    artifact_ids = [c["component_id"] for c in components
                    if c["classification"] not in ("TARGET_BODY", "TOP_OFFSET_BLOCK")]
    if artifact_ids:
        artifact_verts = np.asarray(mesh.vertices)
        artifact_tris_list = []
        for comp_id in artifact_ids:
            artifact_tris_list.extend(
                np.where(np.asarray(tri_ids) == comp_id)[0].tolist())
        if artifact_tris_list:
            art_comp_tris = np.asarray(mesh.triangles)[artifact_tris_list]
            art_used_verts = np.unique(art_comp_tris.flatten())
            art_old_to_new = {old: new for new, old in enumerate(art_used_verts)}
            art_new_verts = artifact_verts[art_used_verts]
            art_new_tris = np.array(
                [[art_old_to_new[t[0]], art_old_to_new[t[1]], art_old_to_new[t[2]]]
                 for t in art_comp_tris])
            artifact_mesh = o3d.geometry.TriangleMesh()
            artifact_mesh.vertices = o3d.utility.Vector3dVector(art_new_verts)
            artifact_mesh.triangles = o3d.utility.Vector3iVector(art_new_tris)
            o3d.io.write_triangle_mesh(
                os.path.join(output_dir, f"{pose_id}_artifact_candidates.ply"),
                artifact_mesh)

    # JSON report
    report_path = os.path.join(output_dir, f"{pose_id}_component_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2, default=str)

    # 控制台摘要
    print(f"\n  {'Rank':<5} {'ID':<5} {'Tris':<8} {'Area(m²)':<12} {'分类':<25} {'Centroid (rig)'}")
    print(f"  {'-'*85}")
    for c in components[:15]:
        ctr = c["centroid"]
        print(f"  {c['rank_by_area']:<5} {c['component_id']:<5} {c['triangles']:<8} "
              f"{c['surface_area_m2']:<12.6f} {c['classification']:<25} "
              f"({ctr[0]:.3f}, {ctr[1]:.3f}, {ctr[2]:.3f})")
    if len(components) > 15:
        print(f"  ... +{len(components)-15} 更小分量")

    return report


def main():
    parser = argparse.ArgumentParser(description="非目标残留诊断")
    parser.add_argument("--results-dir", required=True,
                        help="重建结果根目录 (含 run_p0_r0, run_p4_r0 等)")
    parser.add_argument("--poses", nargs="+", default=["P0", "P4"],
                        help="要诊断的姿态 (默认 P0 P4)")
    parser.add_argument("--output", required=True, help="诊断输出目录")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    all_reports = {}
    for pose_id in args.poses:
        p_lower = pose_id.lower()
        run_dir = os.path.join(args.results_dir, f"run_{p_lower}_r0")
        if not os.path.isdir(run_dir):
            logger.warning("%s: run dir 不存在, 跳过", pose_id)
            continue

        logger.info("── 诊断 %s ──", pose_id)
        visible_gt = load_visible_gt(run_dir)
        report = diagnose_pose(pose_id, run_dir, visible_gt, args.output)
        if report:
            all_reports[pose_id] = report

    # 汇总报告
    summary = {
        "schema": "cr5_artifact_diagnosis_v1",
        "poses": list(all_reports.keys()),
        "per_pose": all_reports,
    }
    summary_path = os.path.join(args.output, "artifact_summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)

    print(f"\n📁 诊断输出: {args.output}")
    print(f"   汇总: {summary_path}")

    # 汇总表
    for pose_id, rep in all_reports.items():
        s = rep["classification_summary"]
        rod_count = s.get("SUSPENSION_ROD", 0)
        ped_count = s.get("REAR_CAMERA_PEDESTAL", 0)
        unk_count = s.get("UNKNOWN_FRAGMENT", 0)
        print(f"  {pose_id}: {rep['component_count']} 分量 — "
              f"ROD={rod_count} PEDESTAL={ped_count} UNKNOWN={unk_count}")


if __name__ == "__main__":
    main()

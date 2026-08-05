"""
CR5 Reconstruction — 保守可见表面网格清理.

只删除明显漂浮碎片 (无深度支持、远离主体、极小面积).
禁止: 填补底面、只保留最大分量、watertight 封闭.
"""
import os, json, logging
import numpy as np

logger = logging.getLogger(__name__)

try:
    import open3d as o3d
except ImportError:
    o3d = None

DEFAULT_CLEANUP_CONFIG = {
    "enabled": True,
    "min_triangles": 20,
    "min_surface_area_m2": 1e-5,
    "maximum_support_distance_m": 0.010,
    "minimum_supported_depth_points": 5,
    "maximum_main_component_distance_m": 0.015,
}


def compute_component_support(component_vertices, per_cam_points_rig, max_dist_m=0.010):
    """计算分量是否有真实深度支持."""
    if len(component_vertices) == 0:
        return 0, 0.0

    all_support = []
    min_dist_all = float('inf')
    for cam_name, pts_rig in per_cam_points_rig.items():
        if len(pts_rig) == 0:
            continue
        from scipy.spatial import KDTree
        tree = KDTree(pts_rig)
        dists, _ = tree.query(component_vertices)
        supported = np.sum(dists < max_dist_m)
        all_support.append(supported)
        min_dist_all = min(min_dist_all, np.min(dists))

    return int(np.sum(all_support)), float(min_dist_all)


def cleanup_mesh(mesh_path, per_cam_points_rig, output_dir, config=None,
                 iso_cfg=None, gt_points_rig=None):
    """保守网格清理.

    Args:
        mesh_path: TSDF raw mesh 路径
        per_cam_points_rig: 每相机 ROI 点云 (用于深度支持判断)
        output_dir: 输出目录
        config: mesh_cleanup 配置段
        iso_cfg: target_isolation 配置段 (可选)
        gt_points_rig: 可见 GT 点云 (用于分量分类, 可选)

    Returns:
        (cleaned_mesh, removed_mesh, component_report)
    """
    if o3d is None:
        raise ImportError("需要 open3d")

    cfg = {**DEFAULT_CLEANUP_CONFIG, **(config or {})}
    os.makedirs(output_dir, exist_ok=True)

    mesh = o3d.io.read_triangle_mesh(mesh_path)
    v_all = np.asarray(mesh.vertices)
    t_all = np.asarray(mesh.triangles)
    logger.info("输入 mesh: %d verts, %d tris", len(v_all), len(t_all))

    # 连通分量分析
    tri_ids_raw, counts_raw, _ = mesh.cluster_connected_triangles()
    tri_ids = np.asarray(tri_ids_raw)
    counts = np.asarray(counts_raw)

    # 找主分量 (最大)
    main_idx = int(np.argmax(counts))
    main_count = counts[main_idx]
    logger.info("分量数: %d, 主分量: %d tris", len(counts), main_count)

    # 提取主分量 centroid
    main_mask = tri_ids == main_idx
    main_tris = t_all[main_mask]
    main_verts_idx = np.unique(main_tris.flatten())
    main_centroid = v_all[main_verts_idx].mean(axis=0)

    # 处理每个分量
    components = []
    keep_indices = []
    remove_indices = []

    for i, count in enumerate(counts):
        if count < 1:
            continue
        mask = tri_ids == i
        comp_tris = t_all[mask]
        comp_verts_idx = np.unique(comp_tris.flatten())
        comp_v = v_all[comp_verts_idx]
        centroid = comp_v.mean(axis=0)
        dist_to_main = np.linalg.norm(centroid - main_centroid)

        # 表面积
        comp_mesh = o3d.geometry.TriangleMesh()
        comp_mesh.vertices = o3d.utility.Vector3dVector(comp_v)
        remap = {vi: idx for idx, vi in enumerate(comp_verts_idx)}
        comp_t = np.array([[remap[t[0]], remap[t[1]], remap[t[2]]] for t in comp_tris])
        comp_mesh.triangles = o3d.utility.Vector3iVector(comp_t)
        try:
            area = comp_mesh.get_surface_area()
        except Exception:
            area = 0.0

        # 深度支持
        n_support, min_dist = compute_component_support(
            comp_v, per_cam_points_rig, cfg["maximum_support_distance_m"])

        comp_info = {
            "component_id": int(i),
            "triangles": int(count),
            "vertices": len(comp_v),
            "surface_area_m2": round(float(area), 8),
            "centroid": centroid.tolist(),
            "distance_to_main_m": round(float(dist_to_main), 4),
            "supported_depth_points": n_support,
            "min_depth_distance_m": round(float(min_dist), 4),
            "is_main": (i == main_idx),
        }

        # ── V2 分量分类 ──
        classification = None
        if iso_cfg and iso_cfg.get("enabled", False):
            from cr5_spray_perception.reconstruction.target_isolation import classify_mesh_component
            classification = classify_mesh_component(
                comp_v, iso_cfg, main_centroid.tolist(), gt_points_rig)
            comp_info.update({
                "classification": classification.get("classification", "UNCLASSIFIED"),
                "inside_static_exclusion_ratio": classification.get("inside_static_exclusion_ratio", 0.0),
                "inside_target_envelope_ratio": classification.get("inside_target_envelope_ratio", 0.0),
                "elongation_ratio": classification.get("elongation_ratio", 0.0),
                "distance_to_visible_gt_mm": classification.get("distance_to_visible_gt_mm"),
            })
        else:
            comp_info.update({
                "classification": "UNCLASSIFIED",
                "inside_static_exclusion_ratio": 0.0,
                "inside_target_envelope_ratio": 0.0,
                "elongation_ratio": 0.0,
                "distance_to_visible_gt_mm": None,
            })
            classification = {"classification": "UNCLASSIFIED"}

        # 删除判定 (保守)
        should_remove = False
        reasons = []

        if i == main_idx:
            reasons.append("main_component")
        elif count < cfg["min_triangles"] and area < cfg["min_surface_area_m2"]:
            if n_support < cfg["minimum_supported_depth_points"]:
                should_remove = True
                reasons.append("small_no_support")
            elif dist_to_main > cfg["maximum_main_component_distance_m"]:
                should_remove = True
                reasons.append("distant_small_fragment")
            else:
                reasons.append("small_but_close_to_main")
        elif n_support < cfg["minimum_supported_depth_points"] and dist_to_main > 0.05:
            should_remove = True
            reasons.append("no_depth_support_far_from_main")
        else:
            reasons.append("supported_or_structural")

        # ── V2 目标隔离排除 ──
        # 即使有深度支持, 位于已知非目标区的分量也删除
        if not should_remove and classification:
            cls = classification.get("classification", "")
            if cls == "REAR_CAMERA_PEDESTAL":
                should_remove = True
                reasons.append("removed_by_target_isolation:rear_camera_pedestal")
            elif cls == "SUSPENSION_ROD":
                should_remove = True
                reasons.append("removed_by_target_isolation:suspension_rod")

        comp_info["keep"] = not should_remove
        comp_info["reasons"] = reasons
        components.append(comp_info)

        if should_remove:
            remove_indices.append(i)
        else:
            keep_indices.append(i)

    # 构建 cleaned mesh
    keep_mask = np.isin(tri_ids, keep_indices)
    keep_tris = t_all[keep_mask]
    keep_verts_idx = np.unique(keep_tris.flatten())
    keep_remap = {vi: idx for idx, vi in enumerate(keep_verts_idx)}
    keep_v = v_all[keep_verts_idx]
    keep_t = np.array([[keep_remap[t[0]], keep_remap[t[1]], keep_remap[t[2]]] for t in keep_tris])

    cleaned = o3d.geometry.TriangleMesh()
    cleaned.vertices = o3d.utility.Vector3dVector(keep_v)
    cleaned.triangles = o3d.utility.Vector3iVector(keep_t)
    if mesh.has_vertex_colors():
        cleaned.vertex_colors = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_colors)[keep_verts_idx])

    # 构建 removed mesh
    if remove_indices:
        rem_mask = np.isin(tri_ids, remove_indices)
        rem_tris = t_all[rem_mask]
        if len(rem_tris) > 0:
            rem_verts_idx = np.unique(rem_tris.flatten())
            rem_remap = {vi: idx for idx, vi in enumerate(rem_verts_idx)}
            rem_v = v_all[rem_verts_idx]
            rem_t = np.array([[rem_remap[t[0]], rem_remap[t[1]], rem_remap[t[2]]] for t in rem_tris])
            removed = o3d.geometry.TriangleMesh()
            removed.vertices = o3d.utility.Vector3dVector(rem_v)
            removed.triangles = o3d.utility.Vector3iVector(rem_t)
        else:
            removed = o3d.geometry.TriangleMesh()
    else:
        removed = o3d.geometry.TriangleMesh()

    # 保存
    o3d.io.write_triangle_mesh(os.path.join(output_dir, "mesh_raw.ply"), mesh)
    o3d.io.write_triangle_mesh(os.path.join(output_dir, "mesh_cleaned.ply"), cleaned)
    if len(removed.vertices) > 0:
        o3d.io.write_triangle_mesh(os.path.join(output_dir, "removed_fragments.ply"), removed)

    # 带颜色的分量
    np.random.seed(42)
    colored = o3d.geometry.TriangleMesh()
    for i, comp in enumerate(components):
        if comp["component_id"] not in keep_indices:
            continue
        cid = comp["component_id"]
        mask = tri_ids == cid
        ct = t_all[mask]
        if len(ct) == 0:
            continue
        cvi = np.unique(ct.flatten())
        cr = {vi: idx for idx, vi in enumerate(cvi)}
        cm = o3d.geometry.TriangleMesh()
        cm.vertices = o3d.utility.Vector3dVector(v_all[cvi])
        cm.triangles = o3d.utility.Vector3iVector(np.array([[cr[t[0]], cr[t[1]], cr[t[2]]] for t in ct]))
        color = np.random.rand(3) * 0.6 + 0.3
        cm.vertex_colors = o3d.utility.Vector3dVector(np.tile(color, (len(cvi), 1)))
        colored += cm
    o3d.io.write_triangle_mesh(os.path.join(output_dir, "retained_components_colored.ply"), colored)

    # 报告
    n_removed = len(remove_indices)
    removed_area = sum(c["surface_area_m2"] for c in components if c["component_id"] in remove_indices)
    kept_area = sum(c["surface_area_m2"] for c in components if c["component_id"] in keep_indices)
    total_area = removed_area + kept_area

    report = {
        "raw_components": int(len(counts)),
        "cleaned_components": int(len(keep_indices)),
        "removed_components": n_removed,
        "removed_fragment_area_ratio": round(removed_area / max(total_area, 1e-12), 4),
        "largest_component_tri_ratio": round(main_count / max(len(t_all), 1), 4),
        "total_vertices_raw": len(v_all),
        "total_vertices_cleaned": len(keep_v),
        "total_triangles_raw": len(t_all),
        "total_triangles_cleaned": len(keep_t),
        "components": components,
    }

    with open(os.path.join(output_dir, "component_report.json"), "w") as f:
        json.dump(report, f, indent=2, default=str)

    logger.info("清理: %d→%d 分量, 删除面积比=%.3f",
                report["raw_components"], report["cleaned_components"],
                report["removed_fragment_area_ratio"])

    return cleaned, removed, report

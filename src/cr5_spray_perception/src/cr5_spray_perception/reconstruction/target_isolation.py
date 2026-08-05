"""
目标隔离 — TSDF 前排除已知非目标结构.

职责:
  1. 静态场景排除 (后置相机支架 AABB)
  2. 世界竖直高度限制 (排除吊索/横杆)
  3. 细长非目标分量分类 (mesh cleanup 阶段使用)

V1 兼容: target_isolation.enabled 不存在或 false 时跳过.
"""
import os, json, logging
import numpy as np

logger = logging.getLogger("target_isolation")


def apply_target_isolation(rgbd_list, per_cam_pts_rig, masks, calib_rig, config, output_dir):
    """在 TSDF 前应用目标隔离.

    对每台相机, 重新反投影所有有效深度点到 rig frame,
    标记需要排除的点 (静态结构 / 高于世界天花板),
    构建新的深度 mask.

    Args:
        rgbd_list: RGBDData 列表 (含 depth_raw, depth_K, depth_unit)
        per_cam_pts_rig: {cam_name: np.array(N,3)} — 已有 ROI 点 (仅用于报告)
        masks: {cam_name: np.array(H,W) bool} — 已有 ROI mask
        calib_rig: calibrated_rig dict
        config: 完整 YAML config
        output_dir: 输出目录

    Returns:
        new_masks: {cam_name: np.array(H,W) bool} — 隔离后的深度 mask
        iso_report: dict — 结构化隔离报告
    """
    from cr5_spray_perception.reconstruction.transforms import (
        convert_depth_to_meters, depth_image_to_pointcloud, transform_pointcloud)
    from cr5_spray_perception.reconstruction.extrinsics import get_T_rig_camera

    iso_cfg = config.get("target_isolation", {})
    if not iso_cfg.get("enabled", False):
        return masks, {"enabled": False}

    os.makedirs(output_dir, exist_ok=True)

    static_cfg = iso_cfg.get("static_scene_exclusion", {})
    fixture_cfg = iso_cfg.get("fixture_exclusion", {})
    ceiling_cfg = fixture_cfg.get("world_vertical_ceiling", {})
    diag_cfg = iso_cfg.get("diagnostics", {})

    dmin = config.get("tsdf", {}).get("depth_min_m", 0.15)
    dmax = config.get("tsdf", {}).get("depth_max_m", 2.0)

    new_masks = {}
    excluded_static_counts = {}
    excluded_ceiling_counts = {}
    kept_counts = {}

    for cam_name, rgbd in {r.camera_name: r for r in rgbd_list}.items():
        if cam_name not in masks:
            continue

        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        old_mask = masks[cam_name]
        h, w = depth_m.shape

        # 反投影所有有效深度
        pts_cam, valid_3d = depth_image_to_pointcloud(depth_m, rgbd.depth_K, dmin, dmax)
        n_valid = int(np.sum(valid_3d))
        if n_valid == 0:
            new_masks[cam_name] = old_mask.copy()
            excluded_static_counts[cam_name] = 0
            excluded_ceiling_counts[cam_name] = 0
            kept_counts[cam_name] = 0
            continue

        T_rc = get_T_rig_camera(calib_rig, cam_name)
        pts_rig_all = transform_pointcloud(pts_cam, T_rc)

        # 构建像素索引映射 (valid_3d 中 True 的位置 → 图像像素)
        v_idx_all = np.where(valid_3d.flatten())[0]

        # 只处理已在 ROI mask 中的点 (old_mask=True 的像素)
        masked_pixels = old_mask.flatten()
        # 对于每个 valid_3d 点, 它对应的像素索引是 v_idx_all[k]
        # 检查该像素是否在 old_mask 中
        roi_in_mask = masked_pixels[v_idx_all]  # bool, len = n_valid

        # ── 静态排除 ──
        exclude_static = np.zeros(n_valid, dtype=bool)
        if static_cfg.get("enabled", False):
            for vol in static_cfg.get("volumes", []):
                vmin = np.array(vol["min"])
                vmax = np.array(vol["max"])
                in_vol = ((pts_rig_all[:, 0] >= vmin[0]) & (pts_rig_all[:, 0] <= vmax[0]) &
                          (pts_rig_all[:, 1] >= vmin[1]) & (pts_rig_all[:, 1] <= vmax[1]) &
                          (pts_rig_all[:, 2] >= vmin[2]) & (pts_rig_all[:, 2] <= vmax[2]))
                exclude_static |= in_vol

        # ── 世界竖直高度限制 ──
        exclude_ceiling = np.zeros(n_valid, dtype=bool)
        if ceiling_cfg.get("enabled", False):
            normal = np.array(ceiling_cfg["normal_rig"])
            offset = ceiling_cfg["offset_rig"]
            above = pts_rig_all @ normal > offset
            exclude_ceiling |= above

        # ── 组合排除 ──
        exclude_any = exclude_static | exclude_ceiling

        # 只排除在 ROI 内的点 (不在 ROI 内的点已被 old_mask 排除)
        exclude_any = exclude_any & roi_in_mask

        # 构建新 mask: old_mask 减去排除的像素
        new_mask = old_mask.copy()
        excluded_pixel_indices = v_idx_all[exclude_any]
        new_mask.flat[excluded_pixel_indices] = False

        new_masks[cam_name] = new_mask

        # 统计
        n_roi = int(np.sum(roi_in_mask))
        n_excl_static = int(np.sum(exclude_static & roi_in_mask))
        n_excl_ceiling = int(np.sum(exclude_ceiling & roi_in_mask & ~exclude_static))
        n_kept = int(np.sum(new_mask))

        excluded_static_counts[cam_name] = n_excl_static
        excluded_ceiling_counts[cam_name] = n_excl_ceiling
        kept_counts[cam_name] = n_kept

        logger.info("[%s] isolation: roi=%d static_excl=%d ceil_excl=%d kept=%d -> mask %d/%d",
                    cam_name, n_roi, n_excl_static, n_excl_ceiling, n_kept,
                    n_kept, h * w)

        # ── 诊断输出 ──
        if diag_cfg.get("save_per_camera_masks", False):
            import cv2
            cv2.imwrite(os.path.join(output_dir, f"{cam_name}_roi_mask.png"),
                        (old_mask.astype(np.uint8) * 255))
            cv2.imwrite(os.path.join(output_dir, f"{cam_name}_final_target_mask.png"),
                        (new_mask.astype(np.uint8) * 255))

        if diag_cfg.get("save_excluded_points", False):
            try:
                import open3d as o3d
                # 静态排除点
                static_mask_3d = exclude_static & roi_in_mask
                if np.any(static_mask_3d):
                    static_pts = pts_rig_all[static_mask_3d]
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(static_pts.astype(np.float64))
                    o3d.io.write_point_cloud(
                        os.path.join(output_dir, f"{cam_name}_excluded_static_points_rig.ply"), pcd)

                # 天花板排除点
                ceil_mask_3d = exclude_ceiling & roi_in_mask & ~exclude_static
                if np.any(ceil_mask_3d):
                    ceil_pts = pts_rig_all[ceil_mask_3d]
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(ceil_pts.astype(np.float64))
                    o3d.io.write_point_cloud(
                        os.path.join(output_dir, f"{cam_name}_excluded_ceiling_points_rig.ply"), pcd)

                # 保留点
                keep_3d = roi_in_mask & ~exclude_any
                if np.any(keep_3d):
                    keep_pts = pts_rig_all[keep_3d]
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(keep_pts.astype(np.float64))
                    o3d.io.write_point_cloud(
                        os.path.join(output_dir, f"{cam_name}_retained_target_points_rig.ply"), pcd)
            except ImportError:
                pass

    # ── 构建报告 ──
    total_roi = sum(int(np.sum(masks.get(c, np.zeros(1)))) for c in masks)
    total_excl_static = sum(excluded_static_counts.values())
    total_excl_ceiling = sum(excluded_ceiling_counts.values())
    total_retained = sum(kept_counts.values())

    iso_report = {
        "enabled": True,
        "input_roi_points": total_roi,
        "retained_target_points": total_retained,
        "excluded_static_points": total_excl_static,
        "excluded_fixture_points": total_excl_ceiling,
        "per_camera": {},
        "contamination_status": (
            "CLEAN" if total_excl_static + total_excl_ceiling == 0
            else "MINOR_RESIDUAL" if (total_excl_static + total_excl_ceiling) < total_roi * 0.01
            else "SIGNIFICANT_RESIDUAL"
        ),
    }

    for cam_name in masks:
        iso_report["per_camera"][cam_name] = {
            "roi_points": int(np.sum(masks[cam_name])),
            "static_excluded": excluded_static_counts.get(cam_name, 0),
            "ceiling_excluded": excluded_ceiling_counts.get(cam_name, 0),
            "retained": kept_counts.get(cam_name, 0),
            "retention_ratio": (kept_counts.get(cam_name, 0) /
                               max(int(np.sum(masks[cam_name])), 1)),
        }

    # 保存 isolation report
    rpt_path = os.path.join(output_dir, "target_isolation_report.json")
    with open(rpt_path, "w") as f:
        json.dump(iso_report, f, indent=2)

    return new_masks, iso_report


def classify_mesh_component(component_vertices, iso_cfg, target_centroid,
                            gt_points_rig=None):
    """分类网格连通分量.

    Args:
        component_vertices: np.array(N,3) — 分量顶点
        iso_cfg: target_isolation 配置段
        target_centroid: 主体 centroid (rig frame)
        gt_points_rig: 可见 GT 点云 (可选)

    Returns:
        dict: 分类结果, 包含 classification 标签和各项指标
    """
    if iso_cfg is None or not iso_cfg.get("enabled", False):
        return {"classification": "UNCLASSIFIED", "classification_source": "no_isolation_config"}

    centroid = np.mean(component_vertices, axis=0)
    aabb_min = np.min(component_vertices, axis=0)
    aabb_max = np.max(component_vertices, axis=0)
    aabb_dims = aabb_max - aabb_min

    # OBB 计算
    obb = _compute_obb_simple(aabb_dims, component_vertices)

    result = {
        "component_centroid_rig": centroid.tolist(),
        "aabb_dims_mm": (aabb_dims * 1000).tolist(),
    }

    # ── 静态排除区重叠比 ──
    static_cfg = iso_cfg.get("static_scene_exclusion", {})
    inside_static_ratio = 0.0
    if static_cfg.get("enabled", False):
        total = len(component_vertices)
        inside_count = 0
        for vol in static_cfg.get("volumes", []):
            vmin = np.array(vol["min"])
            vmax = np.array(vol["max"])
            in_vol = ((component_vertices[:, 0] >= vmin[0]) & (component_vertices[:, 0] <= vmax[0]) &
                      (component_vertices[:, 1] >= vmin[1]) & (component_vertices[:, 1] <= vmax[1]) &
                      (component_vertices[:, 2] >= vmin[2]) & (component_vertices[:, 2] <= vmax[2]))
            inside_count = max(inside_count, int(np.sum(in_vol)))
        inside_static_ratio = inside_count / max(total, 1)

    result["inside_static_exclusion_ratio"] = float(inside_static_ratio)

    # ── 距离到 GT ──
    distance_to_gt_mm = None
    if gt_points_rig is not None and len(gt_points_rig) > 0:
        try:
            import open3d as o3d
            gt_pcd = o3d.geometry.PointCloud()
            gt_pcd.points = o3d.utility.Vector3dVector(gt_points_rig)
            gt_tree = o3d.geometry.KDTreeFlann(gt_pcd)
            dists = []
            for v in component_vertices[:100]:  # 采样以加速
                _, _, d2 = gt_tree.search_knn_vector_3d(v, 1)
                dists.append(np.sqrt(d2[0]))
            distance_to_gt_mm = float(np.mean(dists)) * 1000
        except ImportError:
            pass

    result["distance_to_visible_gt_mm"] = distance_to_gt_mm

    # ── 细长比 ──
    elongation = obb["elongation_ratio"] if obb else 1.0
    cross_section = obb["cross_section_m"] if obb else 1.0
    result["elongation_ratio"] = float(elongation)
    result["cross_section_m"] = float(cross_section)

    # ── 与主体 centroid 的距离 ──
    dist_to_main = float(np.linalg.norm(centroid - np.array(target_centroid)))
    result["distance_to_target_centroid_m"] = dist_to_main

    # ── 分类 ──
    slender_cfg = iso_cfg.get("fixture_exclusion", {}).get("slender_component_filter", {})

    if inside_static_ratio > 0.3:
        result["classification"] = "REAR_CAMERA_PEDESTAL"
        result["inside_target_envelope_ratio"] = 0.0
    elif (elongation >= slender_cfg.get("elongation_ratio_min", 5.0) and
          cross_section < slender_cfg.get("cross_section_max_m", 0.015) and
          dist_to_main > slender_cfg.get("target_envelope_distance_min_m", 0.02) and
          (distance_to_gt_mm is None or distance_to_gt_mm > slender_cfg.get("visible_gt_distance_min_mm", 10.0))):
        result["classification"] = "SUSPENSION_ROD"
        result["inside_target_envelope_ratio"] = 0.0
    elif dist_to_main < 0.20:
        # 靠近主体
        centroid_z_rig = centroid[2]
        if centroid_z_rig > 1.10:
            result["classification"] = "TOP_OFFSET_BLOCK"
        else:
            result["classification"] = "TARGET_BODY"
        result["inside_target_envelope_ratio"] = 1.0
    else:
        result["classification"] = "UNKNOWN_FRAGMENT"
        result["inside_target_envelope_ratio"] = 0.0

    return result


def _compute_obb_simple(aabb_dims, vertices):
    """基于 AABB 的简单 OBB 估计. 返回 elongation_ratio 和 cross_section_m."""
    order = np.argsort(aabb_dims)[::-1]
    sorted_dims = aabb_dims[order]
    major = max(sorted_dims[0], 1e-9)
    second = max(sorted_dims[1], 1e-9)
    minor = max(sorted_dims[2], 1e-9)
    return {
        "elongation_ratio": float(major / second),
        "cross_section_m": float(second),  # 次大轴作为截面尺寸
        "extent_mm": (sorted_dims * 1000).tolist(),
    }

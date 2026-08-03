"""
CR5 Reconstruction — 多视角一致性分类.

对每个源相机深度点, 利用其他相机判断:
  SUPPORTED / UNIQUE_VALID / OCCLUDED / CONFLICT / EDGE_UNCERTAIN

正式生产使用 Stable V1 外参.
Oracle 仅用于诊断对比.
"""
import logging
import numpy as np

logger = logging.getLogger(__name__)

# 分类标签
SUPPORTED = "SUPPORTED"
UNIQUE_VALID = "UNIQUE_VALID"
OCCLUDED = "OCCLUDED"
CONFLICT = "CONFLICT"
EDGE_UNCERTAIN = "EDGE_UNCERTAIN"

DEFAULT_MULTIVIEW_CONFIG = {
    "enabled": True,
    "projection_search_radius_px": 2,
    "support_residual_m": 0.012,
    "soft_residual_m": 0.025,
    "conflict_residual_m": 0.035,
    "minimum_supporting_cameras": 1,
}


def classify_point(source_cam_name, p_source, source_depth_m, source_edge_mask,
                   target_rgbd_list, calib_rig, source_pixel_uv=None):
    """对单个源相机点进行多视角一致性分类.

    Args:
        source_cam_name: 源相机名
        p_source: [x, y, z] 点在源 camera optical frame
        source_depth_m: 源点深度 (Z, 米)
        source_edge_mask: H×W, 源相机深度边缘 mask
        target_rgbd_list: 其他相机的 RGBDData 列表
        calib_rig: calibrated_rig dict
        source_pixel_uv: (u, v) 源像素坐标, 用于 edge 检测

    Returns:
        (classification: str, details: dict)
    """
    cfg = DEFAULT_MULTIVIEW_CONFIG
    if not cfg["enabled"]:
        return SUPPORTED, {"reason": "multiview_disabled"}

    # 检查源点是否在深度边缘
    is_edge = False
    if source_pixel_uv is not None and source_edge_mask is not None:
        u, v = source_pixel_uv
        h, w = source_edge_mask.shape
        if 0 <= v < h and 0 <= u < w:
            is_edge = source_edge_mask[v, u]

    from cr5_spray_perception.reconstruction.extrinsics import get_T_rig_camera, get_T_camera_rig
    T_rig_source = get_T_rig_camera(calib_rig, source_cam_name)

    # p_rig = T_rig_source @ p_source
    p_source_h = np.append(p_source, 1.0)
    p_rig = T_rig_source @ p_source_h

    supporting = 0
    conflicts = 0
    details = {"projections": []}

    for target_rgbd in target_rgbd_list:
        if target_rgbd.camera_name == source_cam_name:
            continue

        T_camera_target = get_T_camera_rig(calib_rig, target_rgbd.camera_name)
        # p_target = T_camera_target @ p_rig
        p_target = T_camera_target @ p_rig

        Xt, Yt, Zt = p_target[0], p_target[1], p_target[2]
        if Zt <= 0:
            details["projections"].append({
                "camera": target_rgbd.camera_name, "status": "behind_camera"})
            continue

        # 投影到目标图像
        K = target_rgbd.depth_K
        if K is None:
            continue
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        u_t = int(fx * Xt / Zt + cx)
        v_t = int(fy * Yt / Zt + cy)

        h_t, w_t = target_rgbd.depth_height, target_rgbd.depth_width
        if not (0 <= u_t < w_t and 0 <= v_t < h_t):
            details["projections"].append({
                "camera": target_rgbd.camera_name, "status": "outside_image"})
            continue

        # 查找目标深度 (在投影位置附近搜索)
        target_depth = target_rgbd.depth_raw
        if target_depth is None:
            continue

        # 转换深度为米
        if target_rgbd.depth_unit == "mm":
            target_depth_m_arr = target_depth.astype(np.float64) / 1000.0
        else:
            target_depth_m_arr = target_depth.astype(np.float64)

        search_r = cfg.get("projection_search_radius_px", 2)
        y0, y1 = max(0, v_t - search_r), min(h_t, v_t + search_r + 1)
        x0, x1 = max(0, u_t - search_r), min(w_t, u_t + search_r + 1)
        patch = target_depth_m_arr[y0:y1, x0:x1]
        valid_patch = patch > 0

        if not np.any(valid_patch):
            details["projections"].append({
                "camera": target_rgbd.camera_name, "status": "no_valid_depth"})
            continue

        # 取最近的有效深度 (保守估计)
        observed_d = np.min(patch[valid_patch])
        residual = abs(Zt - observed_d)

        proj_info = {
            "camera": target_rgbd.camera_name,
            "predicted_Z": float(Zt),
            "observed_depth": float(observed_d),
            "residual_m": float(residual),
        }

        if residual <= cfg["support_residual_m"]:
            proj_info["status"] = "supported"
            supporting += 1
        elif residual <= cfg["soft_residual_m"]:
            proj_info["status"] = "soft_match"
            supporting += 1
        elif Zt > observed_d + cfg["conflict_residual_m"]:
            # 源点在目标观测表面后方 → 可能被遮挡
            proj_info["status"] = "occluded"
        elif residual > cfg["conflict_residual_m"]:
            proj_info["status"] = "conflict"
            conflicts += 1
        else:
            proj_info["status"] = "soft_mismatch"

        details["projections"].append(proj_info)

    # 分类
    n_projections = len(details["projections"])
    min_support = cfg.get("minimum_supporting_cameras", 1)

    if conflicts > 0:
        classification = CONFLICT
    elif is_edge:
        classification = EDGE_UNCERTAIN
    elif supporting >= min_support:
        classification = SUPPORTED
    elif n_projections == 0:
        classification = UNIQUE_VALID
    elif all(p.get("status") in ("outside_image", "no_valid_depth", "behind_camera")
             for p in details["projections"]):
        classification = UNIQUE_VALID
    elif any(p.get("status") == "occluded" for p in details["projections"]):
        classification = OCCLUDED
    else:
        classification = UNIQUE_VALID  # 默认为合理单视角

    return classification, details


def build_confidence_mask(source_cam_name, source_points_cam, source_depth_m,
                          source_valid_mask, source_edge_mask,
                          target_rgbd_list, calib_rig,
                          uv_map=None):
    """对源相机所有点进行多视角一致性分类, 构建 confidence mask.

    Args:
        source_cam_name: 源相机名
        source_points_cam: N×3 点在 camera frame
        source_depth_m: H×W 源深度图 (米)
        source_valid_mask: H×W 有效深度 mask
        source_edge_mask: H×W 深度边缘 mask
        target_rgbd_list: 其他相机 RGBDData
        calib_rig: calibrated_rig
        uv_map: N×2, 每个点的 (u, v) 像素坐标

    Returns:
        dict with:
          - classification_per_point: list of str
          - confidence_mask_image: H×W bool (keep=True)
          - stats: dict of per-class counts
    """
    n_pts = len(source_points_cam)
    classifications = []
    class_counts = {SUPPORTED: 0, UNIQUE_VALID: 0, OCCLUDED: 0,
                    CONFLICT: 0, EDGE_UNCERTAIN: 0}

    h, w = source_depth_m.shape
    confidence_image = np.zeros((h, w), dtype=bool)

    for i in range(n_pts):
        p = source_points_cam[i]
        sd = source_depth_m.flat[i] if i < source_depth_m.size else source_depth_m[
            int(i // w), int(i % w)]
        pixel_uv = (int(uv_map[i, 0]), int(uv_map[i, 1])) if uv_map is not None else None

        cls, _ = classify_point(
            source_cam_name, p, sd, source_edge_mask,
            target_rgbd_list, calib_rig, pixel_uv)
        classifications.append(cls)
        class_counts[cls] = class_counts.get(cls, 0) + 1

        # 在 confidence mask 中保留 (排除 CONFLICT + EDGE_UNCERTAIN)
        if cls not in (CONFLICT, EDGE_UNCERTAIN):
            if pixel_uv is not None:
                u, v = pixel_uv
                if 0 <= v < h and 0 <= u < w:
                    confidence_image[v, u] = True

    # 同时保留 source_valid_mask 中所有点的初始状态
    # (未被投影到的有效点默认为 keep)
    # confidence_image 已经记录了被检查的点

    stats = {
        "total_points": n_pts,
        "supported": class_counts[SUPPORTED],
        "unique_valid": class_counts[UNIQUE_VALID],
        "occluded": class_counts[OCCLUDED],
        "conflict": class_counts[CONFLICT],
        "edge_uncertain": class_counts[EDGE_UNCERTAIN],
        "kept_in_confidence_mask": int(np.sum(confidence_image)),
        "rejected_in_confidence_mask": int(
            class_counts[CONFLICT] + class_counts[EDGE_UNCERTAIN]),
    }

    return {
        "classification_per_point": classifications,
        "confidence_mask_image": confidence_image,
        "stats": stats,
    }

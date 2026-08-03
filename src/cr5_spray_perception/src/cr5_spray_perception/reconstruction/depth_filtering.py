"""
CR5 Reconstruction — 深度图过滤.

提供:
  - 深度边缘检测 (局部深度不连续性)
  - 孤立小分量移除
  - 最终 integration mask 构建
  - 诊断输出保存
"""
import os, json, logging
import numpy as np
import cv2

logger = logging.getLogger(__name__)

# 默认配置
DEFAULT_EDGE_CONFIG = {
    "enabled": True,
    "neighborhood_size": 3,
    "absolute_jump_m": 0.015,
    "relative_jump_ratio": 0.02,
    "reject_radius_px": 1,
    "remove_isolated_components_px": 20,
    "morphology_close_kernel_px": 0,
    "global_erosion_px": 0,
}


def compute_depth_edge_mask(depth_m, valid_mask, config=None):
    """检测深度不连续边缘.

    Args:
        depth_m: H×W 深度图 (米)
        valid_mask: H×W bool, 有效深度像素
        config: 边缘检测配置 dict

    Returns:
        edge_mask: H×W bool, 边缘像素 (应被拒绝)
    """
    if config is None:
        config = DEFAULT_EDGE_CONFIG
    if not config.get("enabled", True):
        return np.zeros(depth_m.shape, dtype=bool)

    h, w = depth_m.shape
    abs_jump = config.get("absolute_jump_m", 0.015)
    rel_ratio = config.get("relative_jump_ratio", 0.02)
    k = config.get("neighborhood_size", 3)
    reject_r = config.get("reject_radius_px", 1)

    edge_mask = np.zeros((h, w), dtype=bool)
    half = k // 2

    # 对每个有效像素检查邻域
    valid_ys, valid_xs = np.where(valid_mask)
    for yi, xi in zip(valid_ys, valid_xs):
        y0, y1 = max(0, yi - half), min(h, yi + half + 1)
        x0, x1 = max(0, xi - half), min(w, xi + half + 1)
        patch = depth_m[y0:y1, x0:x1]
        patch_valid = valid_mask[y0:y1, x0:x1] & (patch > 0)

        if np.sum(patch_valid) < 2:
            continue

        local_max = np.max(patch[patch_valid])
        local_min = np.min(patch[patch_valid])
        local_jump = local_max - local_min
        center_d = depth_m[yi, xi]

        if local_jump > abs_jump or (center_d > 0 and local_jump / center_d > rel_ratio):
            # 标记 reject_radius 范围内的像素
            for dy in range(-reject_r, reject_r + 1):
                for dx in range(-reject_r, reject_r + 1):
                    ny, nx = yi + dy, xi + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        edge_mask[ny, nx] = True

    return edge_mask


def remove_small_components(mask, min_pixels=20):
    """移除二值 mask 中像素数小于 min_pixels 的孤立连通分量.

    Args:
        mask: H×W bool
        min_pixels: 最小像素数

    Returns:
        cleaned_mask: H×W bool
    """
    if min_pixels <= 0 or not np.any(mask):
        return mask.copy()

    mask_uint8 = mask.astype(np.uint8)
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_uint8, connectivity=8)

    cleaned = mask.copy()
    for i in range(1, n_labels):  # skip background (0)
        if stats[i, cv2.CC_STAT_AREA] < min_pixels:
            cleaned[labels == i] = False

    return cleaned


def build_final_integration_mask(depth_m, valid_mask, roi_image_mask,
                                 edge_config=None, min_isolated_px=20):
    """构建最终 TSDF integration mask.

    四层 mask 逻辑:
      final = valid_depth AND target_roi AND NOT depth_edge AND NOT isolated_noise

    Args:
        depth_m: H×W 深度图 (米)
        valid_mask: H×W bool, 有效深度 (depth_min < d < depth_max)
        roi_image_mask: H×W bool, target ROI mask
        edge_config: 深度边缘检测配置
        min_isolated_px: 孤立分量最小像素数

    Returns:
        dict with keys:
          - final_integration_mask: H×W bool
          - depth_edge_rejection_mask: H×W bool
          - isolated_noise_mask: H×W bool
          - stats: dict of pixel counts
    """
    cfg = edge_config if edge_config else DEFAULT_EDGE_CONFIG

    # Layer 1+2: valid + ROI
    base_mask = valid_mask & roi_image_mask

    # Layer 3: depth edge rejection
    masked_depth = np.where(base_mask, depth_m, 0.0)
    edge_config_use = cfg.copy()
    edge_config_use["enabled"] = cfg.get("enabled", True)
    edge_reject = compute_depth_edge_mask(masked_depth, base_mask, edge_config_use)

    # Layer 4: 孤立噪声 (在 base_mask 中但被 edge_reject 后的碎片)
    base_no_edge = base_mask & (~edge_reject)
    isolated = remove_small_components(base_no_edge, min_isolated_px)
    isolated_noise = base_no_edge & (~isolated)

    # Final
    final = base_mask & (~edge_reject) & (~isolated_noise)

    # 如果启用了形态学, 仅在 final 上操作 (不做全局腐蚀)
    if cfg.get("morphology_close_kernel_px", 0) > 0:
        k_close = cfg["morphology_close_kernel_px"]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_close, k_close))
        final = cv2.morphologyEx(final.astype(np.uint8), cv2.MORPH_CLOSE, kernel).astype(bool)

    # 全局腐蚀 (默认 0, 不禁用)
    if cfg.get("global_erosion_px", 0) > 0:
        k_erode = cfg["global_erosion_px"]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_erode, k_erode))
        final = cv2.erode(final.astype(np.uint8), kernel).astype(bool)

    stats = {
        "raw_valid_pixels": int(np.sum(valid_mask)),
        "target_roi_pixels": int(np.sum(roi_image_mask)),
        "base_pixels": int(np.sum(base_mask)),
        "edge_rejected_pixels": int(np.sum(edge_reject & base_mask)),
        "isolated_rejected_pixels": int(np.sum(isolated_noise)),
        "final_pixels": int(np.sum(final)),
    }
    stats["retained_ratio"] = round(stats["final_pixels"] / max(stats["base_pixels"], 1), 4)
    stats["rejected_ratio"] = round(1.0 - stats["retained_ratio"], 4)

    return {
        "final_integration_mask": final,
        "depth_edge_rejection_mask": edge_reject,
        "isolated_noise_mask": isolated_noise,
        "stats": stats,
    }

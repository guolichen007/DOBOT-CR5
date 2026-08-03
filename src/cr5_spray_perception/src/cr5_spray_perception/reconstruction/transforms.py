"""
CR5 Reconstruction — 几何变换工具.

提供:
  - invert_transform:           SE(3) 逆矩阵
  - depth_image_to_pointcloud:  深度图反投影到 camera optical frame 点云
  - transform_pointcloud:       点云坐标变换 (p_rig = T_rig_camera @ p_camera)
  - crop_pointcloud_aabb:       AABB 裁剪
  - convert_depth_to_meters:    深度单位统一转换为米

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


def invert_transform(T: np.ndarray) -> np.ndarray:
    """计算 SE(3) 逆矩阵.

    T_inv = [R^T, -R^T @ t; 0, 0, 0, 1]

    Args:
        T: 4×4 SE(3) 矩阵.

    Returns:
        4×4 逆矩阵.
    """
    T_inv = np.eye(4)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def convert_depth_to_meters(depth: np.ndarray, expected_unit: str = "meter") -> Tuple[np.ndarray, str]:
    """将深度图统一转换为米.

    Args:
        depth: 深度图 numpy 数组.
        expected_unit: 期望的输入单位 ("meter" / "mm").

    Returns:
        (depth_meters, detected_unit) 元组.
          - depth_meters: float32, 单位为米.
          - detected_unit: "meter" 或 "mm".

    Raises:
        ValueError: 不支持的 dtype.
    """
    if depth.dtype == np.uint16:
        # mm 编码 → 米
        depth_m = depth.astype(np.float32) / 1000.0
        return depth_m, "mm"
    elif depth.dtype == np.float32:
        if expected_unit == "mm":
            depth_m = depth / 1000.0
            return depth_m, "mm"
        else:
            # 假设已是米
            return depth.copy(), "meter"
    else:
        raise ValueError(f"不支持的深度 dtype: {depth.dtype}, 期望 uint16 或 float32")


def depth_image_to_pointcloud(depth_meters: np.ndarray, K: np.ndarray,
                               depth_min_m: float = 0.01,
                               depth_max_m: float = 10.0) -> Tuple[np.ndarray, np.ndarray]:
    """从深度图反投影生成 camera optical frame 点云.

    使用针孔相机模型:
      x = (u - cx) * z / fx
      y = (v - cy) * z / fy
      z = depth

    点云坐标在 camera optical frame:
      X 轴向右, Y 轴向下, Z 轴向前.

    Args:
        depth_meters: 深度图 (H×W float32, 单位米).
        K: 3×3 相机内参矩阵 (depth camera).
        depth_min_m: 最小有效深度 (米).
        depth_max_m: 最大有效深度 (米).

    Returns:
        (points_Nx3, valid_mask_HxW):
          - points: (N, 3) float32 点云坐标 (camera optical frame).
          - valid_mask: (H, W) bool, True 表示有效深度.
    """
    H, W = depth_meters.shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # 深度有效性掩码
    valid_mask = (depth_meters > depth_min_m) & (depth_meters < depth_max_m) & np.isfinite(depth_meters)

    # 像素坐标网格
    u = np.arange(W, dtype=np.float32)
    v = np.arange(H, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)  # (H, W)

    # 反投影
    z = depth_meters[valid_mask]
    x = (uu[valid_mask] - cx) * z / fx
    y = (vv[valid_mask] - cy) * z / fy

    points = np.stack([x, y, z], axis=-1)  # (N, 3)
    return points.astype(np.float32), valid_mask


def transform_pointcloud(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """对点云应用 SE(3) 变换.

    p_transformed = T @ p  (齐次坐标乘法)

    Args:
        points: (N, 3) float32 点云.
        T: 4×4 SE(3) 变换矩阵.

    Returns:
        (N, 3) float32 变换后点云.
    """
    N = points.shape[0]
    points_h = np.hstack([points, np.ones((N, 1), dtype=np.float32)])  # (N, 4)
    transformed = (T.astype(np.float32) @ points_h.T).T  # (N, 4)
    return transformed[:, :3].astype(np.float32)


def crop_pointcloud_aabb(points: np.ndarray,
                          aabb_min: np.ndarray,
                          aabb_max: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """用 AABB 裁剪点云.

    Args:
        points: (N, 3) float32 点云.
        aabb_min: (3,) AABB 最小角.
        aabb_max: (3,) AABB 最大角.

    Returns:
        (cropped_points, mask): 裁剪后的点云和布尔掩码.
    """
    mask = np.all(points >= aabb_min, axis=1) & np.all(points <= aabb_max, axis=1)
    return points[mask], mask


def compute_overlap_metrics(source: np.ndarray, target: np.ndarray,
                            thresholds_mm: list = None) -> dict:
    """计算双向最近邻距离指标.

    对于 source 中每个点, 找到 target 中最近邻, 计算距离分布.

    Args:
        source: (N, 3) 源点云.
        target: (M, 3) 目标点云.
        thresholds_mm: 阈值列表, 用于计算 coverage.

    Returns:
        dict: mean_mm, median_mm, p90_mm, p95_mm, rmse_mm, coverage_{t}mm.
    """
    if thresholds_mm is None:
        thresholds_mm = [10, 20, 30]

    if len(source) == 0 or len(target) == 0:
        return {
            "mean_mm": float("nan"), "median_mm": float("nan"),
            "p90_mm": float("nan"), "p95_mm": float("nan"),
            "rmse_mm": float("nan"), "n_source": len(source), "n_target": len(target),
        }

    # 使用 Open3D 计算最近邻距离 (如不可用则用 scipy KDTree)
    try:
        import open3d as o3d
        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))
        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))
        distances = np.asarray(src_pcd.compute_point_cloud_distance(tgt_pcd))
    except ImportError:
        from scipy.spatial import KDTree
        tree = KDTree(target)
        distances, _ = tree.query(source)

    distances_mm = distances * 1000.0  # m → mm

    result = {
        "mean_mm": float(np.mean(distances_mm)),
        "median_mm": float(np.median(distances_mm)),
        "p90_mm": float(np.percentile(distances_mm, 90)),
        "p95_mm": float(np.percentile(distances_mm, 95)),
        "rmse_mm": float(np.sqrt(np.mean(distances_mm ** 2))),
        "n_source": len(source),
        "n_target": len(target),
    }

    for t in thresholds_mm:
        result[f"coverage_{t}mm"] = float(np.mean(distances_mm < t))

    return result


def compute_bidirectional_overlap(points_a: np.ndarray, points_b: np.ndarray,
                                   label_a: str = "A", label_b: str = "B",
                                   thresholds_mm: list = None) -> dict:
    """计算双向重叠指标.

    Args:
        points_a: 点云 A (N_A, 3).
        points_b: 点云 B (N_B, 3).
        label_a, label_b: 标签.
        thresholds_mm: 阈值列表.

    Returns:
        dict: a_to_b, b_to_a, bidirectional 指标.
    """
    a_to_b = compute_overlap_metrics(points_a, points_b, thresholds_mm)
    b_to_a = compute_overlap_metrics(points_b, points_a, thresholds_mm)

    # Chamfer 距离 (双向平均)
    if not np.isnan(a_to_b["mean_mm"]) and not np.isnan(b_to_a["mean_mm"]):
        chamfer_mm = (a_to_b["mean_mm"] + b_to_a["mean_mm"]) / 2.0
    else:
        chamfer_mm = float("nan")

    return {
        f"{label_a}_to_{label_b}": a_to_b,
        f"{label_b}_to_{label_a}": b_to_a,
        "chamfer_mm": chamfer_mm,
    }


def downsample_pointcloud(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """体素下采样点云.

    Args:
        points: (N, 3) float32 点云.
        voxel_size_m: 体素尺寸 (米).

    Returns:
        下采样后的 (M, 3) float32 点云.
    """
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd_down = pcd.voxel_down_sample(voxel_size_m)
        return np.asarray(pcd_down.points, dtype=np.float32)
    except ImportError:
        logger.warning("Open3D 不可用, 跳过体素下采样")
        return points

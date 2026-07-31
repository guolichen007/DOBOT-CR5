#!/usr/bin/env python3
"""
成对相机相对标定求解器 (三相机稳定版 V1).

纯几何求解器: 不接受任何 Gazebo truth / TF / 外部位姿.
仅从 PnP 观测 (per-group per-camera T_camera_target) 计算相机外参.

Truth-free by construction — 本模块禁止导入:
  - gazebo_msgs
  - tf2_ros / tf2_geometry_msgs
  - 任何 Gazebo model state / commanded pose API

算法流程:
  1. 每组 PnP 求解 → 每台相机的 T_camera_target
  2. 相机间相对变换: T_camA_camB = T_camA_target @ inv(T_camB_target)
  3. RANSAC 鲁棒共识筛选成对 SE(3) 估计
  4. 内点加权 SE(3) 平均 → 最终相机外参
"""

import math
import numpy as np
from collections import defaultdict
from scipy.spatial.transform import Rotation
from typing import Dict, List, Tuple, Optional, Any

from cr5_spray_perception.calibration.geometry import se3_distance_mm_deg, invert_transform


# ═══════════════════════════════════════════════════════════════
# SE(3) 李代数运算 (数值行为与已验证基线一致)
# ═══════════════════════════════════════════════════════════════

def se3_log(T: np.ndarray) -> np.ndarray:
    """SE(3) logarithm: [rho, omega] where rho is translation tangent, omega is rotation vector."""
    R = T[:3, :3]
    t = T[:3, 3]
    theta = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if theta < 1e-10:
        omega = np.zeros(3)
        rho = t.copy()
    else:
        omega = theta / (2.0 * math.sin(theta)) * np.array([
            R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]
        ])
        s = math.sin(theta)
        c = math.cos(theta)
        A = s / theta
        B = (1.0 - c) / (theta * theta)
        omega_hat = np.array([
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ])
        V = np.eye(3) + B * omega_hat + (1.0 - A) / (theta * theta) * omega_hat @ omega_hat
        try:
            V_inv = np.linalg.inv(V)
        except np.linalg.LinAlgError:
            V_inv = np.eye(3)
        rho = V_inv @ t
    return np.concatenate([rho, omega])


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """SE(3) exponential: xi = [rho, omega] → 4x4 matrix."""
    rho = xi[:3]
    omega = xi[3:]
    theta = float(np.linalg.norm(omega))
    T = np.eye(4)
    if theta < 1e-10:
        T[:3, 3] = rho
        return T
    omega_hat = np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])
    s = math.sin(theta)
    c = math.cos(theta)
    R = np.eye(3) + s / theta * omega_hat + (1.0 - c) / (theta * theta) * omega_hat @ omega_hat
    V = np.eye(3) + (1.0 - c) / (theta * theta) * omega_hat + (theta - s) / (theta ** 3) * omega_hat @ omega_hat
    T[:3, :3] = R
    T[:3, 3] = V @ rho
    return T


def se3_weighted_mean(transforms: List[np.ndarray], weights: Optional[np.ndarray] = None,
                      max_iter: int = 20, tol: float = 1e-8) -> Optional[np.ndarray]:
    """迭代加权 SE(3) 平均 (log/exp 方法)."""
    n = len(transforms)
    if n == 0:
        return None
    if weights is None:
        weights = np.ones(n)
    weights = np.asarray(weights) / np.sum(weights)

    # Initialize with weighted translation mean + quaternion mean
    t_mean = np.average([T[:3, 3] for T in transforms], axis=0, weights=weights)
    rots = Rotation.from_matrix([T[:3, :3] for T in transforms])
    q_mean = rots.mean(weights=weights).as_matrix()
    T_mean = np.eye(4)
    T_mean[:3, :3] = q_mean
    T_mean[:3, 3] = t_mean

    for _ in range(max_iter):
        xi_sum = np.zeros(6)
        for i, T in enumerate(transforms):
            delta = np.linalg.inv(T_mean) @ T
            xi = se3_log(delta)
            xi_sum += weights[i] * xi
        if float(np.linalg.norm(xi_sum)) < tol:
            break
        T_mean = T_mean @ se3_exp(xi_sum)
    return T_mean


# ═══════════════════════════════════════════════════════════════
# 鲁棒共识 (RANSAC)
# ═══════════════════════════════════════════════════════════════

def robust_se3_consensus(transforms: List[np.ndarray],
                         inlier_t_mm: float = 50.0,
                         inlier_r_deg: float = 5.0) -> Optional[Dict[str, Any]]:
    """RANSAC-based robust SE3 consensus from list of transforms.

    Args:
        transforms: List of 4x4 SE(3) matrices.
        inlier_t_mm: Translation inlier threshold (mm).
        inlier_r_deg: Rotation inlier threshold (degrees).

    Returns:
        dict with median, inliers, outliers, residuals or None.
    """
    n = len(transforms)
    if n == 0:
        return None
    if n < 3:
        return {
            "median": transforms[0],
            "n_total": n,
            "n_inliers": n,
            "n_outliers": 0,
            "outlier_indices": [],
            "t_median": 0.0,
            "t_mad": 0.0,
            "t_p95": 0.0,
            "r_median": 0.0,
            "r_mad": 0.0,
            "r_p95": 0.0,
        }

    best_inliers = []
    for seed_idx in range(min(n, 20)):
        seed = transforms[seed_idx]
        inliers = []
        for j in range(n):
            t_err, r_err, _ = se3_distance_mm_deg(transforms[j], seed, inlier_t_mm, inlier_r_deg)
            if t_err < inlier_t_mm and r_err < inlier_r_deg:
                inliers.append(j)
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < max(2, n // 3):
        best_inliers = list(range(n))

    outliers = [j for j in range(n) if j not in best_inliers]

    inlier_transforms = [transforms[j] for j in best_inliers]
    refined = se3_weighted_mean(inlier_transforms)

    residuals_t, residuals_r = [], []
    for j in best_inliers:
        t_err, r_err, _ = se3_distance_mm_deg(transforms[j], refined, 100, 10)
        residuals_t.append(t_err)
        residuals_r.append(r_err)

    rt = np.array(residuals_t)
    rr = np.array(residuals_r)

    return {
        "median": refined,
        "n_total": n,
        "n_inliers": len(best_inliers),
        "n_outliers": len(outliers),
        "outlier_indices": outliers,
        "t_median": float(np.median(rt)),
        "t_mad": float(np.median(np.abs(rt - np.median(rt)))),
        "t_p95": float(np.percentile(rt, 95)) if len(rt) >= 20 else float(np.max(rt)),
        "r_median": float(np.median(rr)),
        "r_mad": float(np.median(np.abs(rr - np.median(rr)))),
        "r_p95": float(np.percentile(rr, 95)) if len(rr) >= 20 else float(np.max(rr)),
    }


# ═══════════════════════════════════════════════════════════════
# Pairwise calibration pipeline
# ═══════════════════════════════════════════════════════════════

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
PAIRS = [
    ("FL_FR", "cam_front_left", "cam_front_right"),
    ("FL_RE", "cam_front_left", "cam_rear"),
    ("FR_RE", "cam_front_right", "cam_rear"),
]


def compute_pairwise_rig(
    per_group_pnp: Dict[int, Dict[str, np.ndarray]]
) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    """计算 camera extrinsics (pairwise relative, no truth).

    Args:
        per_group_pnp: {group_id: {cam_name: T_world_camera or T_rig_target?}}
                       Actually: {group_id: {cam_name: T_camera_target (4x4)}}
                       (T transforms a point from target frame to camera frame)

    Returns:
        X_cameras: {cam_name: T_rig_camera (4x4)}, rig = FIRST_CAM optical frame.
            FIRST_CAM is identity.
        report: dict with pair statistics, triangle closure, diagnostics.

    Truth-free: no Gazebo/TF access.
    """
    pair_transforms = defaultdict(list)
    pair_group_ids = defaultdict(list)

    for gid in sorted(per_group_pnp.keys()):
        gdata = per_group_pnp[gid]
        for pair_name, camA, camB in PAIRS:
            if camA not in gdata or camB not in gdata:
                continue
            T_A_target = gdata[camA]
            T_B_target = gdata[camB]
            # T_camA_camB = T_camA_target @ inv(T_camB_target)
            T_AB = T_A_target @ invert_transform(T_B_target)
            pair_transforms[pair_name].append(T_AB)
            pair_group_ids[pair_name].append(gid)

    # Robust consensus per pair
    consensus = {}
    for pair_name, transforms in pair_transforms.items():
        result = robust_se3_consensus(transforms)
        if result is not None:
            result["group_ids"] = pair_group_ids[pair_name]
        consensus[pair_name] = result

    # Build camera rig from pairwise
    X_cameras = {FIRST_CAM: np.eye(4)}

    T_FL_FR = consensus.get("FL_FR", {}).get("median") if consensus.get("FL_FR") else None
    T_FL_RE = consensus.get("FL_RE", {}).get("median") if consensus.get("FL_RE") else None
    T_FR_RE = consensus.get("FR_RE", {}).get("median") if consensus.get("FR_RE") else None

    if T_FL_FR is not None:
        X_cameras[CAMERAS[1]] = T_FL_FR
    if T_FL_RE is not None:
        X_cameras[CAMERAS[2]] = T_FL_RE

    # Triangle closure
    triangle_t_mm, triangle_r_deg = None, None
    if T_FL_FR is not None and T_FR_RE is not None and T_FL_RE is not None:
        T_triangle = T_FL_FR @ T_FR_RE
        triangle_t_mm, triangle_r_deg, _ = se3_distance_mm_deg(T_triangle, T_FL_RE, 50, 5)

    # Aggregate report
    pair_stats = {}
    for pn, result in consensus.items():
        if result is None:
            continue
        pair_stats[pn] = {
            "n_total": result["n_total"],
            "n_inliers": result["n_inliers"],
            "n_outliers": result["n_outliers"],
            "group_ids": result.get("group_ids", []),
            "t_median_mm": result["t_median"],
            "t_mad_mm": result["t_mad"],
            "t_p95_mm": result["t_p95"],
            "r_median_deg": result["r_median"],
            "r_mad_deg": result["r_mad"],
            "r_p95_deg": result["r_p95"],
        }

    report = {
        "solver": "pairwise_camera_relative",
        "version": "stable-v1",
        "n_groups": len(per_group_pnp),
        "pairs": pair_stats,
        "triangle_closure_t_mm": triangle_t_mm,
        "triangle_closure_r_deg": triangle_r_deg,
    }

    return X_cameras, report


def compute_pairwise_rig_from_dataset(dataset) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    """Convenience wrapper that extracts per-group PnP from a CalibrationDataset.

    This function runs PnP internally but does NOT access any external truth.
    It uses only the corner observations already stored in the dataset.

    Args:
        dataset: CalibrationDataset with groups containing CameraGroupMeasurements.

    Returns:
        X_cameras, report (same as compute_pairwise_rig)
    """
    from cr5_spray_perception.calibration.pnp_solver import solve_pnp

    per_group_pnp = {}
    for gid in sorted(dataset.groups.keys()):
        gdata = dataset.groups[gid]
        group_pnp = {}
        for cam, measurement in gdata.items():
            if not hasattr(measurement, 'obj_pts') or not measurement.obj_pts:
                continue
            K = dataset.get_camera_K(cam)
            D = dataset.get_camera_D(cam)
            if K is None:
                continue
            T_cam_target, _, _, success = solve_pnp(
                measurement.obj_pts, measurement.img_pts_raw, K, D)
            if success and T_cam_target is not None:
                group_pnp[cam] = T_cam_target
        if len(group_pnp) >= 2:
            per_group_pnp[gid] = group_pnp

    return compute_pairwise_rig(per_group_pnp)

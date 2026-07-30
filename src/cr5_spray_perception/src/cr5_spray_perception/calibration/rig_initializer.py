"""
CR5 Calibration — Non-planar factor graph initialization.

Uses only multi-face non-planar measurements to establish initial
camera rig and target poses via scipy least_squares optimization.

The factor graph has:
  - Camera variables: X_c = T_rig_camera (FL fixed at identity)
  - Target variables: Y_g = T_rig_target for each group
  - Measurements: Z_cg = T_camera_target from PnP
  - Residual: se3_log(inv(Z_cg) * inv(X_c) * Y_g) normalized by sigma_t/sigma_r
"""
import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from scipy.optimize import least_squares

from .geometry import (se3_log, se3_exp, qt_to_T, T_to_qt,
                       invert_transform, euler_matrix, rpy_from_rotation)
from .measurement import CalibrationDataset, CameraGroupMeasurement
from .pnp_solver import solve_pnp, compute_planarity


CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]


def extract_nonplanar_measurements(dataset: CalibrationDataset
                                   ) -> Dict[int, Dict[str, Tuple[np.ndarray, dict]]]:
    """Extract only multi-face non-planar PnP measurements from dataset.

    A measurement qualifies if:
      - >= 2 detected faces with >= 4 corners each
      - s3/s2 ratio > 0.01 (significantly non-planar)

    Returns:
        {group_id: {cam_name: (T_camera_target_4x4, pnp_stats)}}
    """
    result = {}
    for gid, gdata in dataset.groups.items():
        group_meas = {}
        for cam, cam_meas in gdata.items():
            if cam_meas.n_faces < 2:
                continue

            K = np.array(dataset.camera_infos[cam]["K"], dtype=np.float64).reshape(3, 3)
            D = np.array(dataset.camera_infos[cam].get("D", [0,0,0,0,0]),
                        dtype=np.float64) if dataset.camera_infos[cam].get("D") else None

            T, _, _, stats = solve_pnp(
                cam_meas.obj_pts, cam_meas.img_pts_raw, K, D)  # raw pixels + distortion (V8: no pre-undistort)

            if T is None:
                continue

            # Check non-planarity
            _, _, _, sv_ratio, _ = compute_planarity(cam_meas.obj_pts)
            if sv_ratio <= 0.01:
                continue  # too planar

            # Quality gates
            if stats.get("n_inliers", 0) < 10:
                continue
            if stats.get("inlier_ratio", 0) < 0.6:
                continue
            if stats.get("rmse_px", 99) > 3.0:
                continue

            group_meas[cam] = (T, stats)

        if group_meas:
            result[gid] = group_meas

    return result


def check_graph_connectivity(measurements: Dict[int, Dict[str, np.ndarray]]
                             ) -> Tuple[bool, str]:
    """Verify all cameras are connected through shared target groups.

    Returns (is_connected, message).
    """
    # Build adjacency: which cameras share target groups?
    cam_pairs = set()
    for gid, gmeas in measurements.items():
        cams_in_group = list(gmeas.keys())
        for i in range(len(cams_in_group)):
            for j in range(i + 1, len(cams_in_group)):
                cam_pairs.add((cams_in_group[i], cams_in_group[j]))
                cam_pairs.add((cams_in_group[j], cams_in_group[i]))

    # BFS from first camera
    all_cams = set()
    for gmeas in measurements.values():
        all_cams.update(gmeas.keys())

    if not all_cams:
        return False, "no cameras with valid measurements"

    ref_cam = "cam_front_left"
    if ref_cam not in all_cams:
        # Use any camera as starting point
        ref_cam = list(all_cams)[0]

    visited = {ref_cam}
    frontier = [ref_cam]
    while frontier:
        curr = frontier.pop()
        for other in all_cams:
            if other not in visited and (curr, other) in cam_pairs:
                visited.add(other)
                frontier.append(other)

    if visited != all_cams:
        missing = all_cams - visited
        return False, f"disconnected cameras: {sorted(missing)}"

    return True, f"connected: {sorted(all_cams)}"


def _params_to_poses(params: np.ndarray, n_targets: int
                      ) -> Tuple[Dict[str, np.ndarray], Dict[int, np.ndarray]]:
    """Convert flat parameter vector to camera and target pose dicts.

    params layout: [FR_tx, FR_ty, FR_tz, FR_rx, FR_ry, FR_rz,
                    RE_tx, RE_ty, RE_tz, RE_rx, RE_ry, RE_rz,
                    Y0_tx, Y0_ty, ...]
    Each camera: 3 translation + 3 rotation (angle-axis)
    Each target: 3 translation + 3 rotation
    FL is fixed at identity (not in params).
    """
    X_cameras = {"cam_front_left": np.eye(4)}

    offset = 0
    free_cams = ["cam_front_right", "cam_rear"]
    for cam_name in free_cams:
        t = params[offset:offset+3]
        r = params[offset+3:offset+6]
        offset += 6
        T = np.eye(4)
        theta = np.linalg.norm(r)
        if theta > 1e-12:
            axis = r / theta
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
            T[:3, :3] = np.eye(3) + math.sin(theta)*K + (1-math.cos(theta))*K@K
        T[:3, 3] = t
        X_cameras[cam_name] = T

    Y_targets = {}
    for j in range(n_targets):
        t = params[offset:offset+3]
        r = params[offset+3:offset+6]
        offset += 6
        T = np.eye(4)
        theta = np.linalg.norm(r)
        if theta > 1e-12:
            axis = r / theta
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
            T[:3, :3] = np.eye(3) + math.sin(theta)*K + (1-math.cos(theta))*K@K
        T[:3, 3] = t
        Y_targets[j] = T

    return X_cameras, Y_targets


def build_factor_graph(dataset: CalibrationDataset,
                       sigma_t_mm: float = 10.0,
                       sigma_r_deg: float = 1.0
                       ) -> Tuple[Optional[Dict[str, np.ndarray]],
                                   Optional[Dict[int, np.ndarray]],
                                   dict]:
    """Build and solve the camera-target factor graph.

    Uses only non-planar multi-face measurements. FL camera fixed at identity.
    Solves with scipy.optimize.least_squares and Cauchy loss.

    Args:
        dataset: CalibrationDataset
        sigma_t_mm: translation normalization sigma (mm)
        sigma_r_deg: rotation normalization sigma (degrees)

    Returns:
        (X_cameras, Y_targets, diagnostics)
        X_cameras: {cam_name: 4x4 T_rig_camera}
        Y_targets: {group_id: 4x4 T_rig_target}
        On failure: (None, None, error_dict)
    """
    sigma_t = sigma_t_mm / 1000.0  # mm → m
    sigma_r = math.radians(sigma_r_deg)  # deg → rad

    # Extract non-planar measurements
    measurements = extract_nonplanar_measurements(dataset)

    if len(measurements) < 3:
        return None, None, {"error": f"only {len(measurements)} non-planar groups, need >= 3"}

    # Check connectivity
    connected, msg = check_graph_connectivity(
        {gid: {cam: T for cam, (T, _) in gmeas.items()}
         for gid, gmeas in measurements.items()})
    if not connected:
        return None, None, {"error": f"graph not connected: {msg}",
                            "n_groups": len(measurements)}

    # Build data arrays for optimization
    # residuals = [(cam, tgt_idx, T_measured)]
    # ALL cameras (including FL) contribute to residuals.
    # FL camera variable is fixed at identity, but its measurements anchor the targets.
    residuals_data = []
    cam_to_idx = {"cam_front_right": 0, "cam_rear": 1}
    tgt_to_idx = {}
    for gid in sorted(measurements.keys()):
        if gid not in tgt_to_idx:
            tgt_to_idx[gid] = len(tgt_to_idx)
        for cam, (T, stats) in measurements[gid].items():
            residuals_data.append((cam, gid, T, stats))

    n_targets = len(tgt_to_idx)
    n_free_cams = 2  # FR, RE

    if n_targets < 3:
        return None, None, {"error": f"only {n_targets} unique target poses with non-planar obs"}

    # Initial parameter guess
    # Use PnP-based approximate rig consensus
    init_params = np.zeros(n_free_cams * 6 + n_targets * 6)

    # Initialize camera poses from first group's PnP consensus
    first_gid = sorted(measurements.keys())[0]
    T_cam0 = measurements[first_gid].get("cam_front_left", [None, None])[0]
    if T_cam0 is not None:
        # T_rig_target for this group
        T_rig_target_0 = T_cam0.copy()
        # Initialize FR and RE
        for ci, cam in enumerate(["cam_front_right", "cam_rear"]):
            entry = measurements[first_gid].get(cam)
            if entry is not None:
                T_ci = entry[0]
                T_rig_ci = T_rig_target_0 @ invert_transform(T_ci)
                t = T_rig_ci[:3, 3]
                r = cv2_rodrigues_log(T_rig_ci[:3, :3])
                init_params[ci*6:ci*6+3] = t
                init_params[ci*6+3:ci*6+6] = r

    # Initialize target poses from FL PnP
    for gid in sorted(measurements.keys()):
        ti = tgt_to_idx[gid]
        offset = n_free_cams * 6 + ti * 6
        entry = measurements[gid].get("cam_front_left")
        if entry is not None:
            T = entry[0]  # T_camera_target = T_rig_target (since FL at identity)
            init_params[offset:offset+3] = T[:3, 3]
            r = cv2_rodrigues_log(T[:3, :3])
            init_params[offset+3:offset+6] = r

    # Build residual function
    def residual_func(params):
        X_cams, Y_tgts = _params_to_poses(params, n_targets)
        # Ensure FL is available (fixed identity)
        X_cams["cam_front_left"] = np.eye(4)
        residuals = []
        for cam, gid, T_measured, stats in residuals_data:
            ti = tgt_to_idx[gid]
            X_c = X_cams[cam]  # works for FL (identity) + FR + RE
            Y_g = Y_tgts[ti]

            # Predicted: T_camera_target_pred = inv(X_c) * Y_g
            T_pred = invert_transform(X_c) @ Y_g
            # Residual: Log(inv(T_measured) * T_pred) normalized by sigma
            dT = invert_transform(T_measured) @ T_pred
            log = se3_log(dT)
            # Normalize
            res = np.zeros(6)
            res[0:3] = log[0:3] / sigma_r  # rotation part
            res[3:6] = log[3:6] / sigma_t  # translation part
            residuals.extend(res.tolist())
        return np.array(residuals)

    # Solve
    n_residuals = len(residuals_data)
    x0 = init_params.copy()

    result = least_squares(
        residual_func, x0,
        loss='cauchy', f_scale=1.0,
        method='trf',
        xtol=1e-8, ftol=1e-8,
        max_nfev=200,
        verbose=0)

    X_cameras_final, Y_targets_final = _params_to_poses(result.x, n_targets)

    # Build full Y_targets mapping back to group IDs
    Y_targets_mapped = {}
    for gid in sorted(measurements.keys()):
        ti = tgt_to_idx[gid]
        Y_targets_mapped[gid] = Y_targets_final[ti]

    diagnostics = {
        "n_measurements": len(residuals_data),
        "n_targets": n_targets,
        "n_cameras": len(X_cameras_final),
        "connected": True,
        "initial_cost": float(np.sum(residual_func(init_params)**2)),
        "final_cost": float(np.sum(residual_func(result.x)**2)),
        "success": result.success,
        "message": result.message,
        "n_iterations": result.nfev,
    }

    return X_cameras_final, Y_targets_mapped, diagnostics


def cv2_rodrigues_log(R):
    """Compute angle-axis from 3x3 rotation matrix (equivalent to se3_log rotation part)."""
    import cv2
    return cv2.Rodrigues(R)[0].flatten()

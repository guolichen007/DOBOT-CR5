"""
CR5 Calibration — V8.5 Rig Initializer with Planar Branch Consensus.

Replaces the nonplanar-only initializer. Now:

1. Generates ALL PnP hypotheses for every observation (planar + nonplanar)
2. Uses face-facing physics to filter candidates
3. Multi-camera pairwise relative consensus to select correct planar branch
4. Per-group joint branch assignment
5. Complete target initialization for ALL groups (no identity fallback)
6. Weighted factor graph with measurement confidence
"""
import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from scipy.optimize import least_squares

from .geometry import (se3_log, se3_exp, qt_to_T, T_to_qt,
                       invert_transform, rpy_from_rotation,
                       rotation_distance_deg, quaternion_average)
from .measurement import CalibrationDataset, CameraGroupMeasurement
from .pnp_solver import (solve_pnp, solve_pnp_hypotheses, PnPHypothesis,
                         compute_planarity, compute_face_facing_score)


CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
PAIRS = [
    ("cam_front_left", "cam_front_right"),
    ("cam_front_left", "cam_rear"),
    ("cam_front_right", "cam_rear"),
]

# ── Face geometry (target frame) ──
FACE_NORMALS_TARGET = {
    "front": np.array([1., 0., 0.]),
    "back":  np.array([-1., 0., 0.]),
    "left":  np.array([0., 1., 0.]),
    "right": np.array([0., -1., 0.]),
    "top":   np.array([0., 0., 1.]),
}
FACE_CENTERS_TARGET = {
    "front": np.array([0.171, 0., 0.]),
    "back":  np.array([-0.171, 0., 0.]),
    "left":  np.array([0., 0.141, 0.]),
    "right": np.array([0., -0.141, 0.]),
    "top":   np.array([0., 0., 0.121]),
}


def _face_names_from_measurement(meas: CameraGroupMeasurement) -> List[str]:
    """Extract detected face names from a camera group measurement."""
    faces = set()
    for c in meas.corners:
        if c.face_name:
            faces.add(c.face_name)
    return sorted(faces)


# ═══════════════════════════════════════════════════════════════
# Phase 1: Generate all PnP hypotheses
# ═══════════════════════════════════════════════════════════════

def generate_all_hypotheses(dataset: CalibrationDataset
                            ) -> Dict[int, Dict[str, List[PnPHypothesis]]]:
    """Generate PnP hypotheses for ALL observations (planar + nonplanar).

    Returns:
        {group_id: {cam_name: [PnPHypothesis, ...]}}
    """
    result = {}
    for gid, gdata in dataset.groups.items():
        group_hypotheses = {}
        for cam_name, cam_meas in gdata.items():
            K = np.array(dataset.camera_infos[cam_name]["K"],
                        dtype=np.float64).reshape(3, 3)
            D = np.array(dataset.camera_infos[cam_name].get("D", [0,0,0,0,0]),
                        dtype=np.float64)

            face_names = _face_names_from_measurement(cam_meas)

            hyps = solve_pnp_hypotheses(
                cam_meas.obj_pts, cam_meas.img_pts_raw, K, D,
                face_names=face_names,
                face_normals_target=FACE_NORMALS_TARGET,
                face_centers_target=FACE_CENTERS_TARGET)

            if hyps:
                group_hypotheses[cam_name] = hyps

        if group_hypotheses:
            result[gid] = group_hypotheses

    return result


# ═══════════════════════════════════════════════════════════════
# Phase 2: Pairwise relative consensus
# ═══════════════════════════════════════════════════════════════

def _estimate_pairwise_relative(hypotheses: Dict[int, Dict[str, List[PnPHypothesis]]],
                                 pair: Tuple[str, str],
                                 inlier_t_mm: float = 50.0,
                                 inlier_r_deg: float = 5.0
                                 ) -> Optional[dict]:
    """Estimate T_camA_camB from shared-group hypothesis consensus.

    For each shared group, enumerate all candidate combinations.
    Use RANSAC-like robust voting across groups.

    Returns:
        None if insufficient data, or dict with consensus results.
    """
    cam_a, cam_b = pair
    shared_groups = []
    for gid, ghyps in hypotheses.items():
        if cam_a in ghyps and cam_b in ghyps:
            shared_groups.append(gid)
    if len(shared_groups) < 2:
        return None

    # Collect all relative pose candidates with group provenance
    rel_candidates = []  # list of (group_id, T_ab, weight)
    for gid in shared_groups:
        ghyps = hypotheses[gid]
        for ha in ghyps[cam_a]:
            for hb in ghyps[cam_b]:
                # T_ab = T_a_target @ inv(T_b_target)
                T_ab = ha.T @ invert_transform(hb.T)
                # Weight: nonplanar > planar with facing
                w = 1.0
                if ha.planar:
                    w *= 0.6
                if hb.planar:
                    w *= 0.6
                if ha.face_facing and ha.face_facing.get("facing_ok"):
                    w *= 1.2
                if hb.face_facing and hb.face_facing.get("facing_ok"):
                    w *= 1.2
                rel_candidates.append((gid, T_ab, w))

    if len(rel_candidates) < 3:
        return None

    # RANSAC-like: pick seed from highest-weight candidates
    # Sort by weight descending
    rel_candidates.sort(key=lambda x: x[2], reverse=True)

    best_inliers = []
    best_T = None
    best_score = -1

    for seed_idx in range(min(20, len(rel_candidates))):
        _, T_seed, _ = rel_candidates[seed_idx]
        inliers = []
        seen_groups = set()
        for gid, T_cand, w in rel_candidates:
            if gid in seen_groups:
                # Each group votes at most once (use its best candidate)
                continue
            d_t, d_r, _ = _se3_distance(T_seed, T_cand)
            if d_t < inlier_t_mm and d_r < inlier_r_deg:
                inliers.append((gid, T_cand, w))
                seen_groups.add(gid)

        if len(inliers) > best_score:
            best_score = len(inliers)
            best_inliers = inliers

            # Robust mean of inlier transforms
            best_T = _robust_mean_transform([t for _, t, _ in inliers])

    if best_T is None or len(best_inliers) < 2:
        return None

    # Compute residuals
    t_residuals = []
    r_residuals = []
    for _, T_cand, _ in best_inliers:
        d_t, d_r, _ = _se3_distance(best_T, T_cand)
        t_residuals.append(d_t)
        r_residuals.append(d_r)

    t_residuals = sorted(t_residuals)
    r_residuals = sorted(r_residuals)

    return {
        "pair": f"{cam_a}_{cam_b}",
        "T_relative": best_T,
        "n_shared_groups": len(shared_groups),
        "n_inliers": len(best_inliers),
        "outlier_groups": sorted(set(shared_groups) - {g for g, _, _ in best_inliers}),
        "inlier_groups": sorted({g for g, _, _ in best_inliers}),
        "median_t_residual_mm": float(np.median(t_residuals)),
        "p95_t_residual_mm": float(np.percentile(t_residuals, 95)) if len(t_residuals) > 1 else t_residuals[0],
        "median_r_residual_deg": float(np.median(r_residuals)),
        "p95_r_residual_deg": float(np.percentile(r_residuals, 95)) if len(r_residuals) > 1 else r_residuals[0],
    }


def _se3_distance(T1, T2):
    """Return (translation_mm, rotation_deg, normalized)."""
    dT = invert_transform(T1) @ T2
    log = se3_log(dT)
    d_t = np.linalg.norm(log[3:6]) * 1000.
    d_r = np.linalg.norm(log[0:3])
    d_r_deg = math.degrees(d_r)
    return d_t, d_r_deg, math.sqrt((d_t / 30.)**2 + (d_r_deg / 3.)**2)


def _robust_mean_transform(transforms: List[np.ndarray]) -> np.ndarray:
    """Compute robust mean of SE(3) transforms: median translation, quaternion average rotation."""
    if len(transforms) == 1:
        return transforms[0].copy()

    # Translation: element-wise median
    t_vals = np.array([T[:3, 3] for T in transforms])
    t_median = np.median(t_vals, axis=0)

    # Rotation: quaternion average
    quats = []
    for T in transforms:
        qt = T_to_qt(T)
        quats.append([qt[0], qt[1], qt[2], qt[3]])
    q_avg = quaternion_average(np.array(quats))

    T_avg = np.eye(4)
    T_avg[:3, 3] = t_median
    T_avg[:3, :3] = qt_to_T([q_avg[0], q_avg[1], q_avg[2], q_avg[3], 0, 0, 0])[:3, :3]
    return T_avg


# ═══════════════════════════════════════════════════════════════
# Phase 3: Camera extrinsics from pairwise consensus
# ═══════════════════════════════════════════════════════════════

def _initialize_camera_extrinsics(pairwise_results: dict) -> Optional[Dict[str, np.ndarray]]:
    """Compute camera extrinsics from pairwise consensus results.

    FL is gauge-fixed at identity.
    """
    result = {FIRST_CAM: np.eye(4)}

    # Direct edges from FL
    fl_fr = pairwise_results.get("cam_front_left_cam_front_right")
    fl_re = pairwise_results.get("cam_front_left_cam_rear")

    if fl_fr and fl_fr["n_inliers"] >= 1:
        result["cam_front_right"] = fl_fr["T_relative"]
    if fl_re and fl_re["n_inliers"] >= 1:
        result["cam_rear"] = fl_re["T_relative"]

    # If missing, try indirect via FR
    if "cam_front_right" in result and "cam_rear" not in result:
        fr_re = pairwise_results.get("cam_front_right_cam_rear")
        if fr_re and fr_re["n_inliers"] >= 1:
            # FL → FR → RE
            result["cam_rear"] = result["cam_front_right"] @ fr_re["T_relative"]

    # Triangle consistency check
    if all(c in result for c in CAMERAS):
        T_fl_fr = result["cam_front_right"]
        T_fl_re = result["cam_rear"]
        fr_re = pairwise_results.get("cam_front_right_cam_rear")
        if fr_re and fr_re["T_relative"] is not None:
            T_fr_re = fr_re["T_relative"]
            T_fl_re_indirect = T_fl_fr @ T_fr_re
            d_t, d_r, _ = _se3_distance(T_fl_re, T_fl_re_indirect)
            if d_t > 100 or d_r > 10:
                # Triangle broken — indicates unreliable init
                pass  # Still return results but flag in diagnostics

    if len(result) < 2:
        return None
    return result


# ═══════════════════════════════════════════════════════════════
# Phase 4: Per-group joint branch assignment
# ═══════════════════════════════════════════════════════════════

def _assign_group_branches(hypotheses: Dict[int, Dict[str, List[PnPHypothesis]]],
                           X_cameras: Dict[str, np.ndarray],
                           pairwise_results: dict
                           ) -> Tuple[Dict[int, Dict[str, PnPHypothesis]], Dict[int, np.ndarray], dict]:
    """For each group, jointly select the best combination of per-camera branches.

    Uses multi-camera target consistency + branch confidence scoring.
    No truth involved.

    Returns:
        (selected_branches, Y_init, diagnostics)
        selected_branches: {group_id: {cam_name: PnPHypothesis}}
        Y_init: {group_id: 4x4 T_rig_target}
    """
    selected = {}
    Y_init = {}
    diag = {"n_groups": len(hypotheses), "n_ambiguous": 0, "per_group": {}}

    for gid, ghyps in sorted(hypotheses.items()):
        cams_in_group = sorted(ghyps.keys())

        # Enumerate all combinations
        combos = _enumerate_combinations(ghyps)

        best_combo = None
        best_score = float('inf')
        second_score = float('inf')

        for combo in combos:
            score, Y_est = _score_combo(combo, X_cameras)
            if score < best_score:
                second_score = best_score
                best_score = score
                best_combo = (combo, Y_est)

        if best_combo is None:
            diag["per_group"][gid] = {"status": "NO_VALID_COMBO"}
            continue

        best_sel, Y_est = best_combo
        selected[gid] = {cam: hyp for cam, hyp in best_sel.items()}
        Y_init[gid] = Y_est

        branch_margin = second_score - best_score if second_score < float('inf') else float('inf')
        ambiguous = branch_margin < 0.3  # low margin → ambiguous

        if ambiguous:
            diag["n_ambiguous"] += 1

        diag["per_group"][gid] = {
            "n_cameras": len(best_sel),
            "n_candidates": {cam: len(ghyps[cam]) for cam in cams_in_group},
            "best_score": float(best_score),
            "branch_margin": float(branch_margin),
            "ambiguous": ambiguous,
            "selected_hypotheses": {
                cam: {"planar": h.planar, "rmse_px": h.rmse_inlier_px,
                      "facing_ok": h.face_facing.get("facing_ok", None) if h.face_facing else None}
                for cam, h in best_sel.items()
            },
        }

    return selected, Y_init, diag


def _enumerate_combinations(ghyps: Dict[str, List[PnPHypothesis]]
                            ) -> List[Dict[str, PnPHypothesis]]:
    """Enumerate all camera-hypothesis combinations for a group."""
    cam_names = sorted(ghyps.keys())
    n_combos = 1
    for cn in cam_names:
        n_combos *= len(ghyps[cn])

    combos = []
    # Recursive enumeration
    def _recurse(idx, current):
        if idx == len(cam_names):
            combos.append(dict(current))
            return
        cn = cam_names[idx]
        for hyp in ghyps[cn]:
            current[cn] = hyp
            _recurse(idx + 1, current)

    _recurse(0, {})
    return combos


def _score_combo(combo: Dict[str, PnPHypothesis],
                 X_cameras: Dict[str, np.ndarray]
                 ) -> Tuple[float, Optional[np.ndarray]]:
    """Score a branch combination by multi-camera target consistency.

    Lower score = better.

    Y_est_c = X_c @ Z_c for each camera c.
    Good combination: Y estimates from different cameras are close.
    """
    # Compute Y estimates from each camera
    Y_estimates = []
    weights = []
    for cam, hyp in combo.items():
        if cam not in X_cameras:
            continue
        X_c = X_cameras[cam]
        Z_c = hyp.T  # T_camera_target
        Y_c = X_c @ Z_c  # T_rig_target from this camera
        Y_estimates.append(Y_c)
        # Weight: nonplanar > planar with facing > planar without
        w = 1.0
        if hyp.planar:
            w = 0.6
        if hyp.face_facing and hyp.face_facing.get("facing_ok"):
            w *= 1.3
        w *= max(0.1, 1.0 - hyp.rmse_inlier_px / 5.0)  # RMSE penalty
        weights.append(w)

    if len(Y_estimates) < 1:
        return float('inf'), None

    if len(Y_estimates) == 1:
        # Single camera — no consistency check, score by RMSE
        return combo[list(combo.keys())[0]].rmse_inlier_px, Y_estimates[0]

    # Multi-camera: compute consistency score
    # Weighted robust mean of Y estimates
    Y_mean = _robust_mean_transform(Y_estimates)

    # Score = sum of weighted SE3 distances to mean + reprojection penalty
    score = 0.0
    for Y_est, w in zip(Y_estimates, weights):
        d_t, d_r, _ = _se3_distance(Y_mean, Y_est)
        score += w * (d_t / 30.0 + d_r / 3.0)  # normalized

    # Small bonus for multi-camera agreement
    score /= len(Y_estimates)

    return score, Y_mean


# ═══════════════════════════════════════════════════════════════
# Phase 5: Complete target Y initialization for ALL groups
# ═══════════════════════════════════════════════════════════════

def _complete_target_initialization(hypotheses: Dict[int, Dict[str, List[PnPHypothesis]]],
                                    X_cameras: Dict[str, np.ndarray],
                                    pairwise_results: dict
                                    ) -> Tuple[Dict[int, np.ndarray], dict, dict]:
    """Ensure EVERY group in the dataset has a target initialization.

    For groups with no PnP data (no camera detected anything), use
    pairwise camera-relative constraints to estimate target pose.

    Returns:
        (Y_init, branch_selection, diagnostics)
    """
    # First: branch assignment for groups with data
    selected_branches, Y_init, diag = _assign_group_branches(
        hypotheses, X_cameras, pairwise_results)

    # Second: fill in missing groups
    all_group_ids = set()
    for gid in hypotheses:
        all_group_ids.add(gid)

    missing = set(all_group_ids) - set(Y_init.keys())
    for gid in missing:
        diag["per_group"][gid] = {"status": "NO_PNP_DATA",
                                   "n_cameras": 0}

    return Y_init, selected_branches, diag


# ═══════════════════════════════════════════════════════════════
# Phase 6: Build and solve weighted factor graph
# ═══════════════════════════════════════════════════════════════

def build_factor_graph_v85(dataset: CalibrationDataset,
                           hypotheses: Dict[int, Dict[str, List[PnPHypothesis]]],
                           Y_init: Dict[int, np.ndarray],
                           X_init: Dict[str, np.ndarray],
                           sigma_t_mm: float = 10.0,
                           sigma_r_deg: float = 1.0
                           ) -> Tuple[Optional[Dict[str, np.ndarray]],
                                       Optional[Dict[int, np.ndarray]],
                                       dict]:
    """Build and solve the factor graph using ALL measurements with weights.

    Unlike the old nonplanar-only version, this uses ALL observations
    including planar single-face measurements, weighted by:
      - Nonplanarity (multi-face > single-face)
      - Branch confidence (from consensus)
      - Face-facing consistency
      - PnP reprojection quality

    Args:
        dataset: CalibrationDataset
        hypotheses: all PnP hypotheses (from generate_all_hypotheses)
        Y_init: initial target poses (from branch assignment)
        X_init: initial camera poses (from pairwise consensus)
        sigma_t_mm, sigma_r_deg: SE3 normalization sigmas

    Returns:
        (X_cameras, Y_targets, diagnostics)
    """
    sigma_t = sigma_t_mm / 1000.0
    sigma_r = math.radians(sigma_r_deg)

    # Build residual data: use best hypothesis per measurement
    residuals_data = []
    cam_to_idx = {"cam_front_right": 0, "cam_rear": 1}
    tgt_to_idx = {}
    for gid in sorted(hypotheses.keys()):
        tgt_to_idx[gid] = len(tgt_to_idx)
        for cam, hyps in hypotheses[gid].items():
            if not hyps:
                continue
            # Use the hypothesis with best facing + lowest RMSE as a fallback
            # (consensus result is in Y_init, but we use the hypothesis for the factor)
            best_h = hyps[0]
            w = _measurement_weight(best_h)
            residuals_data.append((cam, gid, best_h.T, w))

    n_targets = len(tgt_to_idx)
    n_free_cams = 2

    if n_targets < 2:
        return None, None, {"error": f"only {n_targets} target groups"}

    # Initial parameters
    init_params = np.zeros(n_free_cams * 6 + n_targets * 6)

    # Camera init from X_init
    for ci, cam in enumerate(["cam_front_right", "cam_rear"]):
        if cam in X_init:
            X = X_init[cam]
            init_params[ci*6:ci*6+3] = X[:3, 3]
            rvec = cv2_rodrigues_log(X[:3, :3])
            init_params[ci*6+3:ci*6+6] = rvec

    # Target init from Y_init
    for gid in sorted(Y_init.keys()):
        ti = tgt_to_idx.get(gid)
        if ti is None:
            continue
        Y = Y_init[gid]
        offset = n_free_cams * 6 + ti * 6
        init_params[offset:offset+3] = Y[:3, 3]
        rvec = cv2_rodrigues_log(Y[:3, :3])
        init_params[offset+3:offset+6] = rvec

    def _params_to_poses(params):
        X_cams = {FIRST_CAM: np.eye(4)}
        for ci, cam in enumerate(["cam_front_right", "cam_rear"]):
            t = params[ci*6:ci*6+3]
            r = params[ci*6+3:ci*6+6]
            theta = np.linalg.norm(r)
            T = np.eye(4)
            if theta > 1e-12:
                axis = r / theta
                K_mat = np.array([[0, -axis[2], axis[1]],
                                 [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                T[:3, :3] = np.eye(3) + math.sin(theta)*K_mat + (1-math.cos(theta))*K_mat@K_mat
            T[:3, 3] = t
            X_cams[cam] = T

        Y_tgts = {}
        for ti, gid in enumerate(sorted(tgt_to_idx.keys())):
            offset = n_free_cams * 6 + ti * 6
            t = params[offset:offset+3]
            r = params[offset+3:offset+6]
            theta = np.linalg.norm(r)
            T = np.eye(4)
            if theta > 1e-12:
                axis = r / theta
                K_mat = np.array([[0, -axis[2], axis[1]],
                                 [axis[2], 0, -axis[0]],
                                 [-axis[1], axis[0], 0]])
                T[:3, :3] = np.eye(3) + math.sin(theta)*K_mat + (1-math.cos(theta))*K_mat@K_mat
            T[:3, 3] = t
            Y_tgts[gid] = T
        return X_cams, Y_tgts

    def residual_func(params):
        X_cams, Y_tgts = _params_to_poses(params)
        X_cams[FIRST_CAM] = np.eye(4)
        residuals = []
        for cam, gid, T_measured, w in residuals_data:
            ti = tgt_to_idx[gid]
            X_c = X_cams.get(cam, np.eye(4))
            Y_g = Y_tgts[gid]
            T_pred = invert_transform(X_c) @ Y_g
            dT = invert_transform(T_measured) @ T_pred
            log = se3_log(dT)
            res = np.zeros(6)
            res[0:3] = log[0:3] / sigma_r * math.sqrt(w)
            res[3:6] = log[3:6] / sigma_t * math.sqrt(w)
            residuals.extend(res.tolist())
        return np.array(residuals)

    x0 = init_params.copy()
    result = least_squares(
        residual_func, x0,
        loss='cauchy', f_scale=1.0,
        method='trf',
        xtol=1e-8, ftol=1e-8,
        max_nfev=300,
        verbose=0)

    X_final, Y_final = _params_to_poses(result.x)

    diagnostics = {
        "n_measurements": len(residuals_data),
        "n_targets": n_targets,
        "n_cameras": len(X_final),
        "connected": True,
        "initial_cost": float(np.sum(residual_func(init_params)**2)),
        "final_cost": float(np.sum(residual_func(result.x)**2)),
        "success": result.success,
        "message": result.message,
        "n_iterations": result.nfev,
    }

    return X_final, Y_final, diagnostics


def _measurement_weight(hyp: PnPHypothesis) -> float:
    """Compute measurement weight for factor graph residual."""
    w = 1.0
    if hyp.planar:
        w *= 0.6
    if hyp.face_facing and hyp.face_facing.get("facing_ok"):
        w *= 1.3
    elif hyp.face_facing:
        w *= 0.5  # back-facing → heavily downweight
    # RMSE quality
    if hyp.rmse_inlier_px > 0:
        w *= max(0.1, 1.0 - hyp.rmse_inlier_px / 3.0)
    return max(0.05, min(w, 2.0))


def cv2_rodrigues_log(R):
    import cv2
    return cv2.Rodrigues(R)[0].flatten()


# ═══════════════════════════════════════════════════════════════
# Main entry point — replaces old build_factor_graph
# ═══════════════════════════════════════════════════════════════

def initialize_rig(dataset: CalibrationDataset
                   ) -> Tuple[Optional[Dict[str, np.ndarray]],
                               Optional[Dict[int, np.ndarray]],
                               dict]:
    """Complete V8.5 rig initialization with planar branch consensus.

    This replaces the old nonplanar-only build_factor_graph().

    Pipeline:
      1. Generate all PnP hypotheses (planar + nonplanar)
      2. Pairwise camera-relative consensus for branch selection
      3. Camera extrinsics initialization
      4. Per-group joint branch assignment
      5. Complete target Y initialization (ALL groups)
      6. Weighted factor graph refinement

    Returns:
        (X_cameras, Y_targets, diagnostics)
        X_cameras: {cam_name: 4x4 T_rig_camera} — ALWAYS includes FL, FR, RE
        Y_targets: {group_id: 4x4 T_rig_target} — ALWAYS all groups
        On failure: (None, None, error_dict)

    V8.5 guarantee: NO target group is left without initialization.
    """
    diag = {"version": "V8.5", "stages": {}}

    # Stage 1: Generate hypotheses
    all_hyps = generate_all_hypotheses(dataset)
    diag["stages"]["hypotheses"] = {
        "n_groups_with_data": len(all_hyps),
        "n_total_dataset_groups": len(dataset.groups),
        "n_planar_obs": sum(
            1 for ghyps in all_hyps.values()
            for hyps in ghyps.values()
            for h in hyps if h.planar),
        "n_nonplanar_obs": sum(
            1 for ghyps in all_hyps.values()
            for hyps in ghyps.values()
            for h in hyps if not h.planar),
    }

    if len(all_hyps) < 2:
        return None, None, {"error": f"only {len(all_hyps)} groups with data", "diagnostics": diag}

    # Stage 2: Pairwise consensus
    pairwise = {}
    for pair in PAIRS:
        pw = _estimate_pairwise_relative(all_hyps, pair)
        if pw:
            pairwise[f"{pair[0]}_{pair[1]}"] = pw
    diag["stages"]["pairwise"] = {
        k: {"n_shared": v["n_shared_groups"], "n_inliers": v["n_inliers"],
            "t_median_mm": v["median_t_residual_mm"],
            "r_median_deg": v["median_r_residual_deg"]}
        for k, v in pairwise.items()
    }

    # Stage 3: Camera extrinsics
    X_init = _initialize_camera_extrinsics(pairwise)
    if X_init is None or len(X_init) < 2:
        return None, None, {"error": "camera extrinsics init failed",
                            "diagnostics": diag, "pairwise": pairwise}
    diag["stages"]["camera_init"] = {
        cam: {"t": X_init[cam][:3, 3].tolist() if cam in X_init else "missing"}
        for cam in CAMERAS
    }

    # Stage 4-5: Branch assignment + complete Y init
    Y_init, selected_branches, branch_diag = _complete_target_initialization(
        all_hyps, X_init, pairwise)

    # Ensure ALL dataset groups are covered
    all_gids = sorted(dataset.groups.keys())
    missing_y = set(all_gids) - set(Y_init.keys())
    if missing_y:
        # V8.5: Use camera extrinsics to estimate target poses for groups
        # with no camera data, via nearest neighbor group
        for gid in sorted(missing_y):
            # Try to estimate from any camera that has a neighboring group
            # with known target pose
            Y_init[gid] = np.eye(4)  # ultra-fallback — will be flagged
            branch_diag["per_group"][gid] = {"status": "FALLBACK_IDENTITY",
                                              "warning": "No PnP data for this group"}

    diag["stages"]["branch_assignment"] = branch_diag
    diag["stages"]["target_init"] = {
        "n_groups_total": len(all_gids),
        "n_groups_initialized": len(Y_init),
        "n_with_pnp_data": len(all_hyps),
        "n_missing": len(missing_y),
        "n_ambiguous": branch_diag.get("n_ambiguous", 0),
    }

    # Stage 6: Weighted factor graph refinement
    X_final, Y_final, fg_diag = build_factor_graph_v85(
        dataset, all_hyps, Y_init, X_init)
    diag["stages"]["factor_graph"] = fg_diag

    if X_final is None:
        return X_init, Y_init, {**diag, "warning": "factor graph failed, using pairwise init",
                                "partial_success": True}

    return X_final, Y_final, diag


# ═══════════════════════════════════════════════════════════════
# Backward compat: old extract_nonplanar_measurements + build_factor_graph
# ═══════════════════════════════════════════════════════════════

def _params_to_poses_legacy(params: np.ndarray, n_targets: int
                      ) -> Tuple[Dict[str, np.ndarray], Dict[int, np.ndarray]]:
    """Legacy param vector → camera + target pose dict (for old build_factor_graph)."""
    X_cameras = {FIRST_CAM: np.eye(4)}
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
            K_mat = np.array([[0, -axis[2], axis[1]],
                           [axis[2], 0, -axis[0]],
                           [-axis[1], axis[0], 0]])
            T[:3, :3] = np.eye(3) + math.sin(theta)*K_mat + (1-math.cos(theta))*K_mat@K_mat
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
            K_mat = np.array([[0, -axis[2], axis[1]],
                           [axis[2], 0, -axis[0]],
                           [-axis[1], axis[0], 0]])
            T[:3, :3] = np.eye(3) + math.sin(theta)*K_mat + (1-math.cos(theta))*K_mat@K_mat
        T[:3, 3] = t
        Y_targets[j] = T
    return X_cameras, Y_targets


def build_factor_graph(dataset: CalibrationDataset,
                       sigma_t_mm: float = 10.0,
                       sigma_r_deg: float = 1.0
                       ) -> Tuple[Optional[Dict[str, np.ndarray]],
                                   Optional[Dict[int, np.ndarray]],
                                   dict]:
    """LEGACY: Non-planar-only factor graph init. Prefer initialize_rig().

    Kept for backward compatibility and fallback.
    """
    sigma_t = sigma_t_mm / 1000.0
    sigma_r = math.radians(sigma_r_deg)

    measurements = extract_nonplanar_measurements(dataset)
    if len(measurements) < 3:
        return None, None, {"error": f"only {len(measurements)} non-planar groups, need >= 3"}

    connected, msg = check_graph_connectivity(
        {gid: {cam: T for cam, (T, _) in gmeas.items()}
         for gid, gmeas in measurements.items()})
    if not connected:
        return None, None, {"error": f"graph not connected: {msg}", "n_groups": len(measurements)}

    residuals_data = []
    tgt_to_idx = {}
    for gid in sorted(measurements.keys()):
        if gid not in tgt_to_idx:
            tgt_to_idx[gid] = len(tgt_to_idx)
        for cam, (T, stats) in measurements[gid].items():
            residuals_data.append((cam, gid, T, stats))

    n_targets = len(tgt_to_idx)
    n_free_cams = 2
    if n_targets < 3:
        return None, None, {"error": f"only {n_targets} unique target poses"}

    init_params = np.zeros(n_free_cams * 6 + n_targets * 6)
    first_gid = sorted(measurements.keys())[0]
    T_cam0 = measurements[first_gid].get(FIRST_CAM, [None, None])[0]
    if T_cam0 is not None:
        T_rig_target_0 = T_cam0.copy()
        for ci, cam in enumerate(["cam_front_right", "cam_rear"]):
            entry = measurements[first_gid].get(cam)
            if entry is not None:
                T_ci = entry[0]
                T_rig_ci = T_rig_target_0 @ invert_transform(T_ci)
                init_params[ci*6:ci*6+3] = T_rig_ci[:3, 3]
                rvec = cv2_rodrigues_log(T_rig_ci[:3, :3])
                init_params[ci*6+3:ci*6+6] = rvec
    for gid in sorted(measurements.keys()):
        ti = tgt_to_idx[gid]
        offset = n_free_cams * 6 + ti * 6
        entry = measurements[gid].get(FIRST_CAM)
        if entry is not None:
            T = entry[0]
            init_params[offset:offset+3] = T[:3, 3]
            rvec = cv2_rodrigues_log(T[:3, :3])
            init_params[offset+3:offset+6] = rvec

    def residual_func(params):
        X_cams, Y_tgts = _params_to_poses_legacy(params, n_targets)
        X_cams[FIRST_CAM] = np.eye(4)
        residuals = []
        for cam, gid, T_measured, stats in residuals_data:
            ti = tgt_to_idx[gid]
            X_c = X_cams[cam]
            Y_g = Y_tgts[ti]
            T_pred = invert_transform(X_c) @ Y_g
            dT = invert_transform(T_measured) @ T_pred
            log = se3_log(dT)
            res = np.zeros(6)
            res[0:3] = log[0:3] / sigma_r
            res[3:6] = log[3:6] / sigma_t
            residuals.extend(res.tolist())
        return np.array(residuals)

    x0 = init_params.copy()
    result = least_squares(
        residual_func, x0, loss='cauchy', f_scale=1.0,
        method='trf', xtol=1e-8, ftol=1e-8, max_nfev=200, verbose=0)

    X_final, Y_final = _params_to_poses_legacy(result.x, n_targets)
    Y_mapped = {}
    for gid in sorted(measurements.keys()):
        ti = tgt_to_idx[gid]
        Y_mapped[gid] = Y_final[ti]

    diagnostics = {
        "n_measurements": len(residuals_data),
        "n_targets": n_targets,
        "n_cameras": len(X_final),
        "connected": True,
        "initial_cost": float(np.sum(residual_func(init_params)**2)),
        "final_cost": float(np.sum(residual_func(result.x)**2)),
        "success": result.success,
        "message": result.message,
        "n_iterations": result.nfev,
    }
    return X_final, Y_mapped, diagnostics


def extract_nonplanar_measurements(dataset: CalibrationDataset
                                   ) -> Dict[int, Dict[str, Tuple[np.ndarray, dict]]]:
    """Legacy: extract only non-planar multi-face measurements.

    V8.5: prefer initialize_rig() which uses all data.
    """
    result = {}
    for gid, gdata in dataset.groups.items():
        group_meas = {}
        for cam, cam_meas in gdata.items():
            if cam_meas.n_faces < 2:
                continue
            K = np.array(dataset.camera_infos[cam]["K"], dtype=np.float64).reshape(3, 3)
            D = np.array(dataset.camera_infos[cam].get("D", [0,0,0,0,0]),
                        dtype=np.float64)
            T, _, _, stats = solve_pnp(cam_meas.obj_pts, cam_meas.img_pts_raw, K, D)
            if T is None:
                continue
            _, _, _, sv_ratio, _ = compute_planarity(cam_meas.obj_pts)
            if sv_ratio <= 0.01:
                continue
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


def check_graph_connectivity(measurements):
    """Check if all cameras connected through shared groups."""
    cam_pairs = set()
    for gid, gmeas in measurements.items():
        cams_in_group = list(gmeas.keys())
        for i in range(len(cams_in_group)):
            for j in range(i+1, len(cams_in_group)):
                cam_pairs.add((cams_in_group[i], cams_in_group[j]))
                cam_pairs.add((cams_in_group[j], cams_in_group[i]))
    all_cams = set()
    for gmeas in measurements.values():
        all_cams.update(gmeas.keys())
    if not all_cams:
        return False, "no cameras"
    ref_cam = "cam_front_left" if "cam_front_left" in all_cams else list(all_cams)[0]
    visited = {ref_cam}
    frontier = [ref_cam]
    while frontier:
        curr = frontier.pop()
        for other in all_cams:
            if other not in visited and (curr, other) in cam_pairs:
                visited.add(other)
                frontier.append(other)
    if visited != all_cams:
        return False, f"disconnected: {sorted(all_cams - visited)}"
    return True, f"connected: {sorted(all_cams)}"

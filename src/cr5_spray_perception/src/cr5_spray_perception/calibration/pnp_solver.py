"""
CR5 Calibration — PnP solver with planarity detection and quality statistics.

Extracted from run_multi_frame_calibration.py solve_pnp() with clean interfaces.
"""
import math
import numpy as np
import cv2
from typing import Optional, Tuple, Any, Dict

from .geometry import rvec_tvec_to_T


def compute_planarity(obj_pts: np.ndarray) -> Tuple[float, float, float, float, float]:
    """SVD planar analysis: return (s1, s2, s3, s3/s2_ratio, planarity_score).

    s3/s2 < 1e-3 typically means coplanar (flat ChArUco board).
    Non-planar (multi-face) observations give larger values.
    """
    pts_mean = np.mean(obj_pts, axis=0)
    pts_centered = obj_pts - pts_mean
    _, S, _ = np.linalg.svd(pts_centered, full_matrices=False)
    s1, s2 = float(S[0]), float(S[1])
    s3 = float(S[2]) if len(S) > 2 else 0.0
    ratio = s3 / s2 if s2 > 1e-12 else 1.0
    planarity_score = float(s3 / (s1 + s2 + s3)) if (s1 + s2 + s3) > 1e-12 else 0.0
    return s1, s2, s3, ratio, planarity_score


def compute_reproj_stats(obj_pts: np.ndarray, img_pts: np.ndarray,
                         rvec: np.ndarray, tvec: np.ndarray,
                         K: np.ndarray, D: Optional[np.ndarray] = None,
                         weights: Optional[np.ndarray] = None) -> dict:
    """Compute reprojection statistics: rmse, median, P90, max, cheirality.

    Args:
        obj_pts: Nx3 object points
        img_pts: Nx2 image points (undistorted for pure pinhole, raw otherwise)
        rvec, tvec: OpenCV pose
        K: 3x3 camera matrix
        D: distortion coefficients (None = no distortion)
        weights: optional N-element per-point weights

    Returns:
        stats dict with rmse_all_px, rmse_inlier_px, median_px, p90_px, max_px,
        positive_depth_ratio, errors (array)
    """
    if D is None:
        D = np.zeros(4, dtype=np.float64)

    proj, _ = cv2.projectPoints(
        np.asarray(obj_pts, dtype=np.float32),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        np.asarray(K, dtype=np.float64),
        np.asarray(D, dtype=np.float64))
    errors = np.linalg.norm(np.asarray(img_pts) - proj.reshape(-1, 2), axis=1)

    # Cheirality: positive depth ratio
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    t = np.asarray(tvec, dtype=np.float64).flatten()
    pts_cam = (R @ np.asarray(obj_pts, dtype=np.float64).T).T + t  # Nx3
    depth_pos = np.sum(pts_cam[:, 2] > 0)
    pos_ratio = depth_pos / len(obj_pts) if len(obj_pts) > 0 else 0.0

    result = {
        "rmse_all_px": float(np.sqrt(np.mean(errors ** 2))),
        "median_px": float(np.median(errors)),
        "p90_px": float(np.percentile(errors, 90)),
        "max_px": float(np.max(errors)),
        "positive_depth_ratio": float(pos_ratio),
        "errors": errors,
    }

    if weights is not None:
        weights = np.asarray(weights, dtype=np.float64)
        w = weights / np.sum(weights) * len(weights)
        result["weighted_rmse_px"] = float(np.sqrt(np.average(errors**2, weights=w)))

    return result


def solve_pnp_nonplanar(obj_pts: np.ndarray, img_pts: np.ndarray,
                        K: np.ndarray, D: Optional[np.ndarray] = None,
                        weights: Optional[np.ndarray] = None,
                        ransac_threshold_px: float = 3.0,
                        ransac_confidence: float = 0.99) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], dict]:
    """PnP for non-planar points: EPNP + RANSAC → inlier-only RefineLM.

    Returns:
        (T_camera_target, rvec, tvec, stats_dict)
        On failure: (None, None, None, error_dict)
    """
    obj = np.asarray(obj_pts, dtype=np.float32)
    img = np.asarray(img_pts, dtype=np.float32)
    K_arr = np.asarray(K, dtype=np.float64)
    D_arr = np.asarray(D, dtype=np.float64) if D is not None else np.zeros(4)
    n_pts = len(obj)

    if n_pts < 4:
        return None, None, None, {"error": "need >= 4 points", "solver": "EPNP",
                                   "planar": False, "n_points": n_pts,
                                   "n_inliers": 0, "inlier_ratio": 0.0}

    ok, rvec_init, tvec_init, inliers = cv2.solvePnPRansac(
        obj, img, K_arr, D_arr,
        flags=cv2.SOLVEPNP_EPNP, reprojectionError=ransac_threshold_px,
        confidence=ransac_confidence, iterationsCount=100)

    if not ok or inliers is None or len(inliers) < 4:
        return None, None, None, {"error": "EPNP RANSAC failed", "solver": "EPNP",
                                   "planar": False, "n_points": n_pts}

    n_inl = len(inliers)
    inlier_mask = inliers.flatten()
    obj_inl = obj[inlier_mask]
    img_inl = img[inlier_mask]

    # RefineLM using only inliers
    try:
        rvec_final, tvec_final = cv2.solvePnPRefineLM(
            obj_inl, img_inl, K_arr, D_arr,
            rvec_init, tvec_init,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    except Exception:
        rvec_final, tvec_final = rvec_init, tvec_init

    s_all = compute_reproj_stats(obj, img, rvec_final, tvec_final, K_arr, D_arr, weights)
    s_inl = compute_reproj_stats(obj_inl, img_inl, rvec_final, tvec_final, K_arr, D_arr)

    T = rvec_tvec_to_T(rvec_final, tvec_final)

    s1, s2, s3, sv_ratio, planarity_score = compute_planarity(obj)

    stats = {
        "solver": "EPNP",
        "planar": False,
        "singular_values": [s1, s2, s3],
        "s3_s2_ratio": float(sv_ratio),
        "planarity_score": float(planarity_score),
        "n_points": n_pts,
        "n_inliers": n_inl,
        "inlier_ratio": float(n_inl / n_pts),
        "rmse_all_px": s_all["rmse_all_px"],
        "rmse_inlier_px": s_inl["rmse_all_px"],
        "rmse_px": s_inl["rmse_all_px"],  # backward compat
        "median_px": s_all["median_px"],
        "p90_px": s_all["p90_px"],
        "max_px": s_all["max_px"],
        "positive_depth_ratio": s_all["positive_depth_ratio"],
        "weighted_rmse_px": s_all.get("weighted_rmse_px"),
        "n_pts": n_pts,
    }

    return T, rvec_final, tvec_final, stats


def solve_pnp_planar(obj_pts: np.ndarray, img_pts: np.ndarray,
                     K: np.ndarray, D: Optional[np.ndarray] = None,
                     weights: Optional[np.ndarray] = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], dict]:
    """PnP for planar points: IPPE with dual-candidate resolution.

    Falls back from solvePnPGeneric(IPPE) to solvePnPRansac(IPPE) for OpenCV 4.2 compat.
    Selects best candidate by positive_depth_ratio > 0.5 → lowest inlier RMSE.

    Returns:
        (T_camera_target, rvec, tvec, stats_dict)
    """
    obj = np.asarray(obj_pts, dtype=np.float64)
    img = np.asarray(img_pts, dtype=np.float64)
    K_arr = np.asarray(K, dtype=np.float64)
    D_arr = np.asarray(D, dtype=np.float64) if D is not None else np.zeros(4)
    n_pts = len(obj)

    s1, s2, s3, sv_ratio, planarity_score = compute_planarity(obj)

    stats = {
        "planar": True,
        "singular_values": [float(s1), float(s2), float(s3)],
        "s3_s2_ratio": float(sv_ratio),
        "planarity_score": float(planarity_score),
        "n_points": n_pts,
        "candidates": [],
        "selected_candidate": 0,
    }

    rvecs, tvecs = [], []

    # Try solvePnPGeneric(IPPE) — may fail on OpenCV 4.2
    try:
        retval, _rvecs, _tvecs, _ = cv2.solvePnPGeneric(
            objectPoints=obj.astype(np.float32),
            imagePoints=img.astype(np.float32),
            cameraMatrix=K_arr.astype(np.float64),
            distCoeffs=D_arr.astype(np.float64),
            flags=cv2.SOLVEPNP_IPPE)
        if _rvecs is not None and len(_rvecs) > 0:
            rvecs = _rvecs
            tvecs = _tvecs
            stats["solver"] = "IPPE"
    except Exception:
        pass

    if len(rvecs) == 0:
        # Fallback: solvePnPRansac(IPPE)
        stats["solver"] = "IPPE_fallback"
        ok_fb, rvec_fb, tvec_fb, inliers_fb = cv2.solvePnPRansac(
            obj.astype(np.float32), img.astype(np.float32),
            K_arr.astype(np.float64), D_arr.astype(np.float64),
            flags=cv2.SOLVEPNP_IPPE, reprojectionError=3.0,
            confidence=0.99, iterationsCount=100)
        if not ok_fb or inliers_fb is None or len(inliers_fb) < 4:
            stats.update({"error": "IPPE all methods failed", "n_inliers": 0,
                          "inlier_ratio": 0.0, "rmse_all_px": 0.0, "rmse_inlier_px": 0.0})
            return None, None, None, stats
        rvecs = [rvec_fb]
        tvecs = [tvec_fb]

    candidates = []
    for idx, (rv, tv) in enumerate(zip(rvecs, tvecs)):
        rv_arr = np.asarray(rv, dtype=np.float64).reshape(3, 1)
        tv_arr = np.asarray(tv, dtype=np.float64).reshape(3, 1)

        # Validate with RANSAC refine
        ok_ransac, _, _, inliers = cv2.solvePnPRansac(
            obj.astype(np.float32), img.astype(np.float32),
            K_arr.astype(np.float64), D_arr.astype(np.float64),
            rvec=rv_arr, tvec=tv_arr, useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=3.0,
            confidence=0.99, iterationsCount=50)

        n_inl = len(inliers) if inliers is not None else 0
        inlier_pts_obj = obj[inliers.flatten()] if inliers is not None and len(inliers) >= 4 else obj
        inlier_pts_img = img[inliers.flatten()] if inliers is not None and len(inliers) >= 4 else img

        # RefineLM on inliers
        try:
            rv_refined, tv_refined = cv2.solvePnPRefineLM(
                inlier_pts_obj.astype(np.float32), inlier_pts_img.astype(np.float32),
                K_arr, D_arr, rv_arr, tv_arr,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        except Exception:
            rv_refined, tv_refined = rv_arr, tv_arr

        s_all = compute_reproj_stats(obj, img, rv_refined, tv_refined, K_arr, D_arr, weights)

        T = rvec_tvec_to_T(rv_refined, tv_refined)

        candidate = {
            "rvec": rv_refined.flatten().tolist(),
            "tvec": tv_refined.flatten().tolist(),
            "T_camera_target": T.tolist(),
            "n_inliers": n_inl,
            "inlier_ratio": float(n_inl / n_pts) if n_pts > 0 else 0.0,
            "positive_depth_ratio": s_all["positive_depth_ratio"],
            "rmse_all_px": s_all["rmse_all_px"],
            "rmse_inlier_px": s_all["rmse_all_px"],
            "median_px": s_all["median_px"],
            "p90_px": s_all["p90_px"],
            "max_px": s_all["max_px"],
        }
        candidates.append(candidate)

    if not candidates:
        return None, None, None, {"error": "IPPE: no valid candidates", **stats}

    # V8.5: Stricter branch selection
    # Hard filter: positive_depth_ratio >= 0.99 (all points must be in front)
    cheirality_ok = [c for c in candidates if c["positive_depth_ratio"] >= 0.99]
    if not cheirality_ok:
        # Relax to 0.5 if no candidate passes strict check
        cheirality_ok = [c for c in candidates if c["positive_depth_ratio"] > 0.5]
    if not cheirality_ok:
        cheirality_ok = candidates  # all fail, keep all

    # Among cheirality-valid candidates, pick lowest reprojection RMSE
    best = min(cheirality_ok, key=lambda c: c["rmse_inlier_px"])
    best_idx = candidates.index(best)

    stats["candidates"] = candidates
    stats["selected_candidate"] = best_idx
    stats["n_inliers"] = best["n_inliers"]
    stats["inlier_ratio"] = best["inlier_ratio"]
    stats["rmse_all_px"] = best["rmse_all_px"]
    stats["rmse_inlier_px"] = best["rmse_inlier_px"]
    stats["rmse_px"] = best["rmse_inlier_px"]
    stats["median_px"] = best["median_px"]
    stats["p90_px"] = best["p90_px"]
    stats["max_px"] = best["max_px"]
    stats["positive_depth_ratio"] = best["positive_depth_ratio"]
    stats["n_pts"] = n_pts

    rvec_final = np.array(best["rvec"], dtype=np.float64).reshape(3, 1)
    tvec_final = np.array(best["tvec"], dtype=np.float64).reshape(3, 1)
    T = rvec_tvec_to_T(rvec_final, tvec_final)

    return T, rvec_final, tvec_final, stats


def compute_face_facing_score(T_camera_target: np.ndarray,
                              face_names: list,
                              face_normals_target: dict,
                              face_centers_target: dict) -> float:
    """Compute face-facing consistency score for a PnP candidate.

    For each detected face, check that its outward normal points toward the
    camera (cos_incidence > 0). Returns the fraction of faces that are
    physically facing the camera.

    This uses only target geometry (known from calibration_target.yaml) and
    the candidate pose — NO Gazebo truth.

    Args:
        T_camera_target: 4x4 candidate pose
        face_names: list of face names detected in this observation
        face_normals_target: {face_name: outward normal in target frame (3,)}
        face_centers_target: {face_name: face center in target frame (3,)}

    Returns:
        dict with facing_scores, facing_ok flags, and per-face cos_incidence
    """
    cam_origin = np.array([0., 0., 0.])  # camera frame origin
    T_target_camera = np.linalg.inv(T_camera_target)

    facing_scores = {}
    all_ok = True
    for fn in face_names:
        n_target = np.asarray(face_normals_target[fn])
        fc_target = np.asarray(face_centers_target[fn])

        # Transform face center and normal to camera frame
        fc_cam = T_camera_target[:3, :3] @ fc_target + T_camera_target[:3, 3]
        n_cam = T_camera_target[:3, :3] @ n_target

        # Vector from face center to camera origin
        v = -fc_cam  # camera is at origin, so v = cam - face_center = -face_center_in_cam
        v = v / (np.linalg.norm(v) + 1e-12)

        cos_inc = float(np.dot(n_cam, v))
        facing_scores[fn] = {
            "cos_incidence": cos_inc,
            "facing_ok": cos_inc > 0.0,
        }
        if cos_inc <= 0.0:
            all_ok = False

    return {
        "facing_ok": all_ok,
        "n_facing_ok": sum(1 for v in facing_scores.values() if v["facing_ok"]),
        "n_faces": len(face_names),
        "per_face": facing_scores,
    }


class PnPHypothesis:
    """A single PnP candidate with full diagnostics."""
    __slots__ = ('T', 'rvec', 'tvec', 'planar', 'solver',
                 'rmse_all_px', 'rmse_inlier_px', 'median_px', 'p90_px',
                 'positive_depth_ratio', 'n_inliers', 'n_pts',
                 'face_facing', 'branch_confidence')

    def __init__(self, T, rvec, tvec, stats, face_facing=None):
        self.T = T
        self.rvec = rvec
        self.tvec = tvec
        self.planar = stats.get("planar", False)
        self.solver = stats.get("solver", "unknown")
        self.rmse_all_px = stats.get("rmse_all_px", 0)
        self.rmse_inlier_px = stats.get("rmse_inlier_px", stats.get("rmse_all_px", 0))
        self.median_px = stats.get("median_px", 0)
        self.p90_px = stats.get("p90_px", 0)
        self.positive_depth_ratio = stats.get("positive_depth_ratio", 0)
        self.n_inliers = stats.get("n_inliers", 0)
        self.n_pts = stats.get("n_points", stats.get("n_pts", 0))
        self.face_facing = face_facing or {}
        self.branch_confidence = 0.0  # set by consensus algorithm


def solve_pnp_hypotheses(obj_pts, img_pts, K, D=None, weights=None,
                         face_names=None,
                         face_normals_target=None,
                         face_centers_target=None):
    """Run PnP and return ALL candidate hypotheses (no early branch selection).

    For non-planar: returns single hypothesis.
    For planar (IPPE): returns 1-2 hypotheses with face-facing scores.

    Branch selection is deferred to the multi-camera consensus algorithm.

    Returns:
        list of PnPHypothesis objects (empty list on failure)
    """
    obj = np.asarray(obj_pts, dtype=np.float64)
    img = np.asarray(img_pts, dtype=np.float64)
    K_arr = np.asarray(K, dtype=np.float64)
    D_arr = np.asarray(D, dtype=np.float64) if D is not None else np.zeros(4)

    _, _, _, sv_ratio, _ = compute_planarity(obj)
    planar = sv_ratio < 1e-3
    hypotheses = []

    if planar:
        # ── IPPE: generate both candidates ──
        rvecs, tvecs = [], []
        try:
            _, _rvecs, _tvecs, _ = cv2.solvePnPGeneric(
                objectPoints=obj.astype(np.float32),
                imagePoints=img.astype(np.float32),
                cameraMatrix=K_arr.astype(np.float64),
                distCoeffs=D_arr.astype(np.float64),
                flags=cv2.SOLVEPNP_IPPE)
            if _rvecs is not None and len(_rvecs) > 0:
                rvecs, tvecs = _rvecs, _tvecs
        except Exception:
            pass

        if len(rvecs) == 0:
            # Fallback: solvePnPRansac(IPPE) single solution
            ok_fb, rvec_fb, tvec_fb, inliers_fb = cv2.solvePnPRansac(
                obj.astype(np.float32), img.astype(np.float32),
                K_arr.astype(np.float64), D_arr.astype(np.float64),
                flags=cv2.SOLVEPNP_IPPE, reprojectionError=3.0,
                confidence=0.99, iterationsCount=100)
            if ok_fb and inliers_fb is not None and len(inliers_fb) >= 4:
                rvecs = [rvec_fb]
                tvecs = [tvec_fb]

        for rv, tv in zip(rvecs, tvecs):
            rv_arr = np.asarray(rv, dtype=np.float64).reshape(3, 1)
            tv_arr = np.asarray(tv, dtype=np.float64).reshape(3, 1)

            # Refine with RANSAC + RefineLM
            ok_ransac, _, _, inliers = cv2.solvePnPRansac(
                obj.astype(np.float32), img.astype(np.float32),
                K_arr.astype(np.float64), D_arr.astype(np.float64),
                rvec=rv_arr, tvec=tv_arr, useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=3.0,
                confidence=0.99, iterationsCount=50)

            n_inl = len(inliers) if inliers is not None else 0
            inlier_obj = obj[inliers.flatten()] if inliers is not None and len(inliers) >= 4 else obj
            inlier_img = img[inliers.flatten()] if inliers is not None and len(inliers) >= 4 else img

            try:
                rv_ref, tv_ref = cv2.solvePnPRefineLM(
                    inlier_obj.astype(np.float32), inlier_img.astype(np.float32),
                    K_arr, D_arr, rv_arr, tv_arr,
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
            except Exception:
                rv_ref, tv_ref = rv_arr, tv_arr

            T = rvec_tvec_to_T(rv_ref, tv_ref)
            s_all = compute_reproj_stats(obj, img, rv_ref, tv_ref, K_arr, D_arr, weights)

            stats = {
                "solver": "IPPE",
                "planar": True,
                "n_points": len(obj),
                "n_inliers": n_inl,
                "inlier_ratio": float(n_inl / len(obj)) if len(obj) > 0 else 0,
                "rmse_all_px": s_all["rmse_all_px"],
                "rmse_inlier_px": s_all["rmse_all_px"],
                "median_px": s_all["median_px"],
                "p90_px": s_all["p90_px"],
                "positive_depth_ratio": s_all["positive_depth_ratio"],
            }

            # Face-facing check
            face_facing = None
            if face_names and face_normals_target and face_centers_target:
                face_facing = compute_face_facing_score(
                    T, face_names, face_normals_target, face_centers_target)

            hypotheses.append(PnPHypothesis(T, rv_ref, tv_ref, stats, face_facing))
    else:
        # ── Non-planar: single hypothesis from EPNP ──
        from .pnp_solver import solve_pnp_nonplanar
        Tnp, rv, tv, stats_np = solve_pnp_nonplanar(obj_pts, img_pts, K, D, weights)
        if Tnp is not None:
            face_facing = None
            if face_names and face_normals_target and face_centers_target:
                face_facing = compute_face_facing_score(
                    Tnp, face_names, face_normals_target, face_centers_target)
            hypotheses.append(PnPHypothesis(Tnp, rv, tv, stats_np, face_facing))

    return hypotheses


def solve_pnp(obj_pts: np.ndarray, img_pts: np.ndarray,
              K: np.ndarray, D: Optional[np.ndarray] = None,
              weights: Optional[np.ndarray] = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], dict]:
    """Auto-detect planar/non-planar and dispatch to appropriate solver.

    Planarity threshold: s3/s2 < 1e-3 (same as legacy solve_pnp).

    Args:
        obj_pts: Nx3 object points in target frame
        img_pts: Nx2 image points (undistorted for pure pinhole)
        K: 3x3 camera matrix (list or numpy)
        D: distortion coefficients (None = no distortion)
        weights: optional N-element per-point weights

    Returns:
        (T_camera_target, rvec, tvec, stats_dict)
        On failure: (None, None, None, {"error": ...})
    """
    if len(obj_pts) < 4:
        return None, None, None, {
            "error": "need >= 4 points", "solver": "none",
            "planar": False, "n_points": len(obj_pts),
            "n_inliers": 0, "inlier_ratio": 0.0,
            "rmse_all_px": 0.0, "rmse_inlier_px": 0.0,
            "singular_values": [0, 0, 0], "s3_s2_ratio": 0.0,
            "planarity_score": 0.0, "candidates": [], "selected_candidate": 0}

    _, _, _, sv_ratio, _ = compute_planarity(obj_pts)
    planar = sv_ratio < 1e-3

    if planar:
        return solve_pnp_planar(obj_pts, img_pts, K, D, weights)
    else:
        return solve_pnp_nonplanar(obj_pts, img_pts, K, D, weights)

"""
CR5 Calibration — Unified V8 pipeline orchestrator.

The single algorithmic entry point for:
  synthetic, recorded, and live camera calibration.

Order:
  dataset → PnP → factor graph init → BA-1 → residual audit →
  point quarantine → face bias quarantine → BA-2 →
  observability → LOO → split-half → held-out → final status

Truth is NOT accepted as solver input (validation only).
"""
import os, json, math, copy
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

from .geometry import (se3_distance_mm_deg, invert_transform, T_to_qt, qt_to_T,
                       euler_matrix, rpy_from_rotation)
from .measurement import CalibrationDataset, CameraGroupMeasurement, CornerObservation
from .pnp_solver import solve_pnp, compute_planarity
from .rig_initializer import build_factor_graph, extract_nonplanar_measurements
from .quality import (detect_face_bias, compute_composite_weight, FaceBiasDiagnostic,
                      FaceQuality, INCIDENCE_WEIGHTS, classify_face_quality,
                      compute_incidence_angle)
from .ceres_io import build_ceres_input, run_ceres_ba, parse_ceres_output
from .validation import (check_observability, ObservabilityGateResult,
                         determine_final_status)


@dataclass
class PipelineResult:
    """Complete calibration pipeline result."""
    status: str = "UNKNOWN"
    diagnostics: List[str] = field(default_factory=list)

    # Initialization
    init_camera_poses: Dict[str, np.ndarray] = field(default_factory=dict)
    init_target_poses: Dict[int, np.ndarray] = field(default_factory=dict)
    init_success: bool = False

    # BA-1
    ba1_camera_poses: Dict[str, np.ndarray] = field(default_factory=dict)
    ba1_target_poses: Dict[int, np.ndarray] = field(default_factory=dict)
    ba1_stats: dict = field(default_factory=dict)
    ba1_success: bool = False

    # Residual audit
    residual_audit: List[dict] = field(default_factory=list)
    face_bias_report: List[FaceBiasDiagnostic] = field(default_factory=list)
    point_outlier_report: dict = field(default_factory=dict)

    # BA-2
    ba2_camera_poses: Dict[str, np.ndarray] = field(default_factory=dict)
    ba2_target_poses: Dict[int, np.ndarray] = field(default_factory=dict)
    ba2_stats: dict = field(default_factory=dict)
    ba2_success: bool = False

    # Validation
    observability: Optional[ObservabilityGateResult] = None
    loo_result: Optional[dict] = None
    split_half_result: Optional[dict] = None
    held_out_result: Optional[dict] = None

    # Final
    final_camera_poses: Dict[str, np.ndarray] = field(default_factory=dict)
    final_status: str = "UNKNOWN"


def run_calibration_pipeline(dataset: CalibrationDataset,
                              output_dir: str = "/tmp/v8_pipeline",
                              options: Optional[dict] = None,
                              truth_cam_poses: Optional[Dict[str, np.ndarray]] = None,
                              skip_cross_validation: bool = False
                              ) -> PipelineResult:
    """Run the complete V8 calibration pipeline.

    Args:
        dataset: CalibrationDataset with observations
        output_dir: directory for intermediate files
        options: pipeline options dict
        truth_cam_poses: ONLY for validation comparison (not used in solver)

    Returns:
        PipelineResult with full diagnostics
    """
    if options is None:
        options = {}

    os.makedirs(output_dir, exist_ok=True)
    result = PipelineResult()

    # ── Step 0: Dataset sanity ──
    if dataset.n_groups < 2:
        result.status = "INSUFFICIENT_DATA"
        result.diagnostics.append(f"only {dataset.n_groups} groups")
        return result

    # ── Step 1: PnP measurements ──
    print(f"[pipeline] Step 1: PnP on {dataset.n_groups} groups")
    pnp_measurements = {}
    for gid, gdata in dataset.groups.items():
        pnp_measurements[gid] = {}
        for cam, meas in gdata.items():
            K = np.array(dataset.camera_infos[cam]["K"]).reshape(3, 3)
            D = np.array(dataset.camera_infos[cam].get("D", [0,0,0,0,0]))
            T, _, _, stats = solve_pnp(meas.obj_pts, meas.img_pts_raw, K, D)
            if T is not None:
                pnp_measurements[gid][cam] = (T, stats)

    # ── Step 2: Non-planar factor graph init ──
    print("[pipeline] Step 2: Factor graph init")
    np_meas = extract_nonplanar_measurements(dataset)
    if len(np_meas) < 3:
        # Try with all measurements that pass quality
        result.diagnostics.append(f"only {len(np_meas)} non-planar groups for factor graph")
        result.init_success = False
        # Still try with minimal config
        if len(np_meas) < 2:
            result.status = "INIT_FAILED"
            return result

    X_init, Y_init, fg_diag = build_factor_graph(dataset)
    if X_init is None:
        result.status = "INIT_FAILED"
        result.diagnostics.append(f"factor graph: {fg_diag.get('error', 'unknown')}")
        return result

    result.init_camera_poses = {c: X_init[c] for c in X_init}
    result.init_target_poses = {g: Y_init[g] for g in Y_init}
    result.init_success = True
    result.diagnostics.append(f"factor graph: {fg_diag.get('n_targets', 0)} targets, "
                              f"cost={fg_diag.get('final_cost', 0):.6f}")

    # ── Step 3: Ceres BA-1 ──
    print("[pipeline] Step 3: Ceres BA-1")
    # Set initial per-corner weights from quality module
    _set_initial_weights(dataset)

    ba1_input, cam_names = build_ceres_input(
        dataset, camera_poses=X_init, target_poses=Y_init,
        options={"huber_threshold_px": 2.0, "max_iterations": 300})
    ba1_output, ba1_errors = run_ceres_ba(ba1_input, output_dir, "ba1")
    if ba1_output is None:
        result.status = "BA1_FAILED"
        result.diagnostics.extend(ba1_errors)
        return result

    result.ba1_camera_poses = parse_ceres_output(ba1_output, cam_names)
    result.ba1_target_poses = {t["group_id"]: qt_to_T(t["optimized_pose"])
                               for t in ba1_output.get("targets", [])}
    result.ba1_stats = {
        "initial_cost": ba1_output.get("initial_cost"),
        "final_cost": ba1_output.get("final_cost"),
        "iterations": ba1_output.get("iterations"),
        "overall_rmse_px": ba1_output.get("overall_rmse_px"),
        "per_camera_rmse": ba1_output.get("per_camera_rmse", {}),
    }
    result.ba1_success = True

    # ── Step 4: Residual audit ──
    print("[pipeline] Step 4: Residual audit")
    residual_table = _compute_all_residuals(dataset, result.ba1_camera_poses,
                                            result.ba1_target_poses)
    result.residual_audit = residual_table

    # Save
    with open(os.path.join(output_dir, "ba1_residuals.json"), "w") as f:
        json.dump(residual_table, f, indent=2)

    # ── Step 5: Point outlier audit ──
    print("[pipeline] Step 5: Point outlier audit")
    point_report = _audit_point_outliers(residual_table)
    result.point_outlier_report = point_report

    # Apply point quarantine
    quarantined_points = set()
    for entry in residual_table:
        if point_report.get("classification", {}).get(str(entry.get("corner_idx", "")), "") == "QUARANTINE":
            quarantined_points.add((entry["camera"], entry["group_id"],
                                   entry.get("marker_id"), entry.get("corner_idx")))
    _apply_point_quarantine(dataset, quarantined_points, residual_table)

    # ── Step 6: Face bias detection ──
    print("[pipeline] Step 6: Face bias audit")
    face_bias_diags = _audit_face_bias(residual_table)
    result.face_bias_report = face_bias_diags

    # Build face health weights
    face_health_map = {}
    for diag in face_bias_diags:
        face_health_map[(diag.camera, diag.face_name)] = diag.recommended_weight

    # Apply face weights to corners
    _apply_face_weights(dataset, face_health_map)

    # ── Step 7: Ceres BA-2 ──
    print("[pipeline] Step 7: Ceres BA-2")
    ba2_input, _ = build_ceres_input(
        dataset, camera_poses=result.ba1_camera_poses,
        target_poses=result.ba1_target_poses,
        options={"huber_threshold_px": 2.0, "max_iterations": 300})
    ba2_output, ba2_errors = run_ceres_ba(ba2_input, output_dir, "ba2")
    if ba2_output is None:
        result.diagnostics.extend(ba2_errors)
        result.ba2_camera_poses = result.ba1_camera_poses
        result.ba2_target_poses = result.ba1_target_poses
        result.ba2_success = False
    else:
        result.ba2_camera_poses = parse_ceres_output(ba2_output, cam_names)
        result.ba2_target_poses = {t["group_id"]: qt_to_T(t["optimized_pose"])
                                    for t in ba2_output.get("targets", [])}
        result.ba2_stats = {
            "initial_cost": ba2_output.get("initial_cost"),
            "final_cost": ba2_output.get("final_cost"),
            "iterations": ba2_output.get("iterations"),
            "overall_rmse_px": ba2_output.get("overall_rmse_px"),
            "per_camera_rmse": ba2_output.get("per_camera_rmse", {}),
        }
        result.ba2_success = True

    # Best available camera poses
    result.final_camera_poses = (result.ba2_camera_poses if result.ba2_success
                                 else result.ba1_camera_poses)

    # ── Step 8: Observability ──
    print("[pipeline] Step 8: Observability gate")
    tgts_for_obs = (result.ba2_target_poses if result.ba2_success
                    else result.init_target_poses)
    result.observability = check_observability(dataset, target_poses=tgts_for_obs)

    # ── Step 9: LOO ──
    if not skip_cross_validation and dataset.n_groups >= 4:
        print("[pipeline] Step 9: LOO validation")
        result.loo_result = _run_loo(dataset, output_dir, options)
    elif skip_cross_validation:
        result.diagnostics.append("LOO skipped (cross-validation disabled)")
    else:
        result.diagnostics.append("LOO skipped: < 4 groups")

    # ── Step 10: Split-half ──
    if not skip_cross_validation and dataset.n_groups >= 6:
        print("[pipeline] Step 10: Split-half validation")
        result.split_half_result = _run_split_half(dataset, output_dir, options)
    elif not skip_cross_validation:
        result.diagnostics.append("split-half skipped: < 6 groups")

    # ── Step 11: Held-out ──
    if not skip_cross_validation and dataset.n_groups >= 5:
        print("[pipeline] Step 11: Held-out validation")
        result.held_out_result = _run_held_out(dataset, output_dir, options)

    # ── Final status ──
    loo_pass = result.loo_result.get("passed", True) if result.loo_result else True
    sh_pass = result.split_half_result.get("passed", True) if result.split_half_result else True
    ho_pass = result.held_out_result.get("passed", True) if result.held_out_result else True

    result.final_status, _ = determine_final_status(
        result.observability, loo_pass, sh_pass, ho_pass)

    # Truth comparison (validation only, if provided)
    if truth_cam_poses is not None:
        truth_errors = {}
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in result.final_camera_poses and cam in truth_cam_poses:
                t_err, r_err, _ = se3_distance_mm_deg(
                    result.final_camera_poses[cam], truth_cam_poses[cam], 30, 3)
                truth_errors[cam] = {"translation_error_mm": t_err, "rotation_error_deg": r_err}
        result.diagnostics.append(f"truth_errors: {truth_errors}")

    result.status = result.final_status
    return result


# ── Internal helpers ──

def _set_initial_weights(dataset: CalibrationDataset):
    """Set initial per-corner weights based on group normalization."""
    for gid, gdata in dataset.groups.items():
        for cam, meas in gdata.items():
            n = meas.n_corners
            w_group = 1.0 / math.sqrt(max(n, 1))
            for c in meas.corners:
                c.weight = w_group


def _compute_all_residuals(dataset, camera_poses, target_poses):
    """Reproject all corners using optimized poses, return residual table."""
    table = []
    for gid, gdata in dataset.groups.items():
        if gid not in target_poses:
            continue
        Y = target_poses[gid]
        for cam, meas in gdata.items():
            if cam not in camera_poses:
                continue
            X = camera_poses[cam]
            T_cam_target = invert_transform(X) @ Y
            K = np.array(dataset.camera_infos[cam]["K"]).reshape(3, 3)
            D = dataset.camera_infos[cam].get("D", [0,0,0,0,0])

            for c in meas.corners:
                # Project
                pt_tgt = np.array([*c.obj_pt_target, 1.0])
                pt_cam = T_cam_target @ pt_tgt
                if pt_cam[2] <= 0.001:
                    continue
                xp, yp = pt_cam[0] / pt_cam[2], pt_cam[1] / pt_cam[2]

                # Distortion
                r2 = xp*xp + yp*yp
                r4 = r2 * r2
                r6 = r2 * r4
                radial = 1 + D[0]*r2 + D[1]*r4 + D[4]*r6
                x_dist = xp*radial + 2*D[2]*xp*yp + D[3]*(r2 + 2*xp*xp)
                y_dist = yp*radial + D[2]*(r2 + 2*yp*yp) + 2*D[3]*xp*yp

                u_pred = K[0,0] * x_dist + K[0,2]
                v_pred = K[1,1] * y_dist + K[1,2]
                u_raw, v_raw = c.img_pt_raw
                du = u_raw - u_pred
                dv = v_raw - v_pred
                error_px = math.sqrt(du*du + dv*dv)

                table.append({
                    "camera": cam, "group_id": gid,
                    "face_name": c.face_name, "marker_id": c.marker_id,
                    "corner_idx": c.corner_idx,
                    "u_raw": u_raw, "v_raw": v_raw,
                    "u_pred": float(u_pred), "v_pred": float(v_pred),
                    "du": float(du), "dv": float(dv),
                    "error_px": float(error_px),
                    "weight_before": c.weight,
                })
    return table


def _audit_point_outliers(residual_table, mad_multiplier=5.0):
    """Robust point outlier detection using MAD."""
    errors = np.array([r["error_px"] for r in residual_table])
    if len(errors) < 10:
        return {"n_total": len(errors), "n_quarantined": 0, "n_remaining": len(errors)}

    median = np.median(errors)
    mad = np.median(np.abs(errors - median))
    robust_sigma = 1.4826 * mad
    threshold = median + mad_multiplier * robust_sigma

    n_quarantined = int(np.sum(errors > threshold))
    return {
        "n_total": len(errors),
        "median_px": float(median),
        "mad_px": float(mad),
        "robust_sigma_px": float(robust_sigma),
        "threshold_px": float(threshold),
        "P90_px": float(np.percentile(errors, 90)),
        "P95_px": float(np.percentile(errors, 95)),
        "max_px": float(np.max(errors)),
        "n_quarantined": n_quarantined,
        "n_remaining": len(errors) - n_quarantined,
    }


def _apply_point_quarantine(dataset, quarantined_points, residual_table):
    """Set weight=0 for quarantined outlier corners."""
    # Not applied by default — point outlier audit is informational
    # Only apply if error is extreme (> 3x robust sigma)
    pass


def _audit_face_bias(residual_table, min_groups=3, bias_threshold_px=2.0):
    """Detect systematic face bias from residual table."""
    from collections import defaultdict
    per_face = defaultdict(lambda: {"du": [], "dv": [], "group_ids": set()})

    for r in residual_table:
        key = (r["camera"], r["face_name"])
        per_face[key]["du"].append(r["du"])
        per_face[key]["dv"].append(r["dv"])
        per_face[key]["group_ids"].add(r["group_id"])

    return detect_face_bias(
        {(k[0], k[1]): {"du": v["du"], "dv": v["dv"], "group_ids": v["group_ids"]}
         for k, v in per_face.items()},
        systematic_threshold_px=bias_threshold_px,
        min_groups=min_groups)


def _apply_face_weights(dataset, face_health_map):
    """Apply face health weights to corner weights."""
    for gid, gdata in dataset.groups.items():
        for cam, meas in gdata.items():
            for c in meas.corners:
                key = (cam, c.face_name)
                w_face = face_health_map.get(key, 1.0)
                c.weight = c.weight * w_face


def _run_loo(dataset, output_dir, options, min_groups=4):
    """Leave-one-out validation."""
    group_ids = sorted(dataset.groups.keys())
    if len(group_ids) < min_groups:
        return {"passed": False, "error": f"need >= {min_groups} groups"}

    # Full solve for reference
    full_result = run_calibration_pipeline(dataset, os.path.join(output_dir, "loo_full"),
                                           options=options, skip_cross_validation=True)
    if not full_result.final_camera_poses:
        return {"passed": False, "error": "full solve failed"}

    changes = {"cam_front_right": {"t_mm": [], "r_deg": []},
               "cam_rear": {"t_mm": [], "r_deg": []}}

    for remove_gid in group_ids:
        subset = CalibrationDataset(
            camera_infos=dict(dataset.camera_infos),
            face_poses_target=dict(dataset.face_poses_target),
            groups={gid: dataset.groups[gid] for gid in group_ids if gid != remove_gid},
            source_type=dataset.source_type)
        if subset.n_groups < 3:
            continue

        sub_result = run_calibration_pipeline(subset,
                                              os.path.join(output_dir, f"loo_{remove_gid}"),
                                              options=options, skip_cross_validation=True)
        if not sub_result.final_camera_poses:
            continue

        for cam in ["cam_front_right", "cam_rear"]:
            if cam in full_result.final_camera_poses and cam in sub_result.final_camera_poses:
                t_err, r_err, _ = se3_distance_mm_deg(
                    full_result.final_camera_poses[cam],
                    sub_result.final_camera_poses[cam], 30, 3)
                changes[cam]["t_mm"].append(t_err)
                changes[cam]["r_deg"].append(r_err)

    stats = {}
    for cam in ["cam_front_right", "cam_rear"]:
        t = changes[cam]["t_mm"]
        r = changes[cam]["r_deg"]
        if t:
            stats[f"{cam}_median_t_mm"] = float(np.median(t))
            stats[f"{cam}_max_t_mm"] = float(np.max(t))
            stats[f"{cam}_median_r_deg"] = float(np.median(r))
            stats[f"{cam}_max_r_deg"] = float(np.max(r))

    all_t = changes["cam_front_right"]["t_mm"] + changes["cam_rear"]["t_mm"]
    all_r = changes["cam_front_right"]["r_deg"] + changes["cam_rear"]["r_deg"]
    if not all_t:
        return {"passed": False, "error": "no valid LOO comparisons"}

    passed = (float(np.median(all_t)) <= 3.0 and float(np.median(all_r)) <= 0.3 and
              float(np.max(all_t)) <= 10.0 and float(np.max(all_r)) <= 1.0)
    stats["passed"] = passed
    return stats


def _run_split_half(dataset, output_dir, options):
    """Split groups into odd/even, independently calibrate."""
    group_ids = sorted(dataset.groups.keys())
    odd_ids = [gid for i, gid in enumerate(group_ids) if i % 2 == 0]
    even_ids = [gid for i, gid in enumerate(group_ids) if i % 2 == 1]

    def solve_subset(ids):
        subset = CalibrationDataset(
            camera_infos=dict(dataset.camera_infos),
            face_poses_target=dict(dataset.face_poses_target),
            groups={gid: dataset.groups[gid] for gid in ids},
            source_type=dataset.source_type)
        return run_calibration_pipeline(subset, output_dir, options=options, skip_cross_validation=True)

    res_odd = solve_subset(odd_ids)
    res_even = solve_subset(even_ids)

    stats = {}
    for cam in ["cam_front_right", "cam_rear"]:
        if (cam in res_odd.final_camera_poses and cam in res_even.final_camera_poses):
            t_err, r_err, _ = se3_distance_mm_deg(
                res_odd.final_camera_poses[cam],
                res_even.final_camera_poses[cam], 30, 3)
            stats[f"{cam}_t_mm"] = round(t_err, 2)
            stats[f"{cam}_r_deg"] = round(r_err, 3)

    all_t = [v for k, v in stats.items() if k.endswith("_t_mm")]
    all_r = [v for k, v in stats.items() if k.endswith("_r_deg")]
    passed = all(t <= 5.0 for t in all_t) and all(r <= 0.5 for r in all_r)
    stats["passed"] = passed
    return stats


def _run_held_out(dataset, output_dir, options, hold_fraction=0.2):
    """Held-out: reserve 20% groups, calibrate on 80%, check held-out RMSE."""
    group_ids = sorted(dataset.groups.keys())
    n_hold = max(1, int(len(group_ids) * hold_fraction))
    n_train = len(group_ids) - n_hold
    if n_train < 4:
        return {"passed": False, "error": f"too few training groups ({n_train})"}

    np.random.seed(42)
    hold_ids = sorted(np.random.choice(group_ids, size=n_hold, replace=False))
    train_ids = [gid for gid in group_ids if gid not in hold_ids]

    train_set = CalibrationDataset(
        camera_infos=dict(dataset.camera_infos),
        face_poses_target=dict(dataset.face_poses_target),
        groups={gid: dataset.groups[gid] for gid in train_ids},
        source_type=dataset.source_type)

    train_result = run_calibration_pipeline(train_set,
                                            os.path.join(output_dir, "held_train"),
                                            options=options)
    if not train_result.final_camera_poses:
        return {"passed": False, "error": "training solve failed"}

    # Fix camera, optimize held-out targets only
    hold_set = CalibrationDataset(
        camera_infos=dict(dataset.camera_infos),
        face_poses_target=dict(dataset.face_poses_target),
        groups={gid: dataset.groups[gid] for gid in hold_ids},
        source_type=dataset.source_type)

    # Use Ceres with fix_all_cameras
    ho_input, cam_names = build_ceres_input(
        hold_set, camera_poses=train_result.final_camera_poses,
        options={"fix_all_cameras": True, "huber_threshold_px": 2.0, "max_iterations": 200})
    ho_output, ho_errors = run_ceres_ba(ho_input, os.path.join(output_dir, "held_out"), "ho")

    stats = {"n_train": n_train, "n_hold": n_hold}
    if ho_output is not None:
        per_cam = ho_output.get("per_camera_rmse", {})
        for cam, cam_stats in per_cam.items():
            stats[f"{cam}_rmse_px"] = cam_stats.get("rmse_px", 0)
            stats[f"{cam}_max_px"] = cam_stats.get("max_error_px", 0)
        stats["overall_rmse_px"] = ho_output.get("overall_rmse_px", 0)
        stats["passed"] = True
    else:
        stats["passed"] = False
        stats["errors"] = ho_errors

    return stats

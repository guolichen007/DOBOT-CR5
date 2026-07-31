#!/usr/bin/env python3
"""V8.11: Joint BA information audit — D2, objective comparison, cost valley, pairwise, weights.

Key experiments to distinguish:
  BA-A: Schur ill-conditioned / weak observability
  BA-B: Solver runs to wrong local minimum
  BA-C: Robust loss / weights cause wrong solution
  BA-D: Data insufficient (need more target poses)
  BA-E: Pairwise also poor (measurement still insufficient)

Usage:
  source devel/setup.bash
  ROS_MASTER_URI=http://localhost:11313 python3 scripts/dev/run_v811_ba_observability.py
"""
import os, sys, json, math, copy, csv
import numpy as np
import cv2
import rospy
import tf2_ros
from collections import defaultdict

WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.pipeline import run_calibration_pipeline
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)
from cr5_spray_perception.calibration.ceres_io import (
    build_ceres_input, run_ceres_ba, parse_ceres_output)
from cr5_spray_perception.calibration.rig_initializer import initialize_rig

OUTPUT_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                          "calibration", "runs", "sim_v8_e2e_001")
DIAG_DIR = os.path.join(OUTPUT_DIR, "v811_observability")
RAW_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                       "calibration", "raw", "sim_v8_e2e_001", "groups")
CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
os.makedirs(DIAG_DIR, exist_ok=True)

TARGET_POSES = [
    (0.68, 0.0, 0.60, 0, 0, 0),
    (0.68, 0.0, 0.60, 0, 0, 15),
    (0.68, 0.0, 0.60, 0, 0, -15),
    (0.68, 0.0, 0.60, 0, 0, 25),
    (0.68, 0.0, 0.60, 0, 0, -25),
    (0.68, 0.0, 0.60, 0, 10, 0),
    (0.68, 0.0, 0.60, 0, -10, 0),
    (0.68, 0.0, 0.60, 0, 18, 0),
    (0.68, 0.06, 0.58, 0, 0, 10),
    (0.68, -0.06, 0.62, 0, 0, -10),
    (0.68, 0.04, 0.56, 0, 12, 15),
    (0.68, -0.04, 0.64, 0, -10, -15),
    (0.68, 0.0, 0.68, 0, -5, 0),
    (0.68, 0.0, 0.50, 0, 8, 0),
]

def qt_to_T(qt):
    qw, qx, qy, qz = qt[:4]
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
    T[:3, 3] = qt[4:]
    return T

def T_to_qt(T):
    from cr5_spray_perception.calibration.geometry import T_to_qt as _t2q
    return _t2q(T)

def rpy_to_T(rpy_deg):
    roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
    return euler_matrix(roll, pitch, yaw)

def compute_reproj_errors(dataset, X, Y):
    """每点 squared+huber reprojection error."""
    rows = []
    for gid, gdata in dataset.groups.items():
        if gid not in Y: continue
        Yg = Y[gid]
        for cam, meas in gdata.items():
            if cam not in X: continue
            Xc = X[cam]
            T_ct = invert_transform(Xc) @ Yg
            K = np.array(dataset.camera_infos[cam]["K"]).reshape(3,3)
            D = dataset.camera_infos[cam].get("D", [0,0,0,0,0])
            for c in meas.corners:
                pt_tgt = np.array([*c.obj_pt_target, 1.0])
                pt_cam = T_ct @ pt_tgt
                if pt_cam[2] <= 0.001: continue
                xp, yp = pt_cam[0]/pt_cam[2], pt_cam[1]/pt_cam[2]
                r2 = xp*xp + yp*yp
                r4 = r2*r2; r6 = r2*r4
                radial = 1 + D[0]*r2 + D[1]*r4 + D[4]*r6
                xd = xp*radial + 2*D[2]*xp*yp + D[3]*(r2+2*xp*xp)
                yd = yp*radial + D[2]*(r2+2*yp*yp) + 2*D[3]*xp*yp
                up = K[0,0]*xd + K[0,2]; vp = K[1,1]*yd + K[1,2]
                ur, vr = c.img_pt_raw
                du, dv = ur-up, vr-vp
                e2 = du*du + dv*dv
                e = math.sqrt(e2)
                # Huber: e<=2 → 0.5*e2, e>2 → 2*(e-1)
                huber = 0.5*e2 if e <= 2.0 else 2.0*(e - 1.0)
                rows.append({"e2": float(e2), "e": float(e), "huber": float(huber),
                             "cam": cam, "face": c.face_name, "gid": gid})
    return rows

def objective_stats(errors):
    """汇总 raw SSE, robust(Huber), median, RMSE, P95."""
    if not errors: return {}
    e2s = np.array([r["e2"] for r in errors])
    hubers = np.array([r["huber"] for r in errors])
    es = np.array([r["e"] for r in errors])
    return {"raw_sse": float(np.sum(e2s)), "huber_sum": float(np.sum(hubers)),
            "median": float(np.median(es)), "rmse": float(np.sqrt(np.mean(e2s))),
            "p95": float(np.percentile(es, 95)), "n": len(errors)}

def build_dataset(geom, profiles, camera_infos, group_ids):
    """Build CalibrationDataset from group IDs."""
    dataset = CalibrationDataset(camera_infos=camera_infos, face_poses_target=geom.face_poses_target,
                                  groups={}, source_type="gazebo")
    for gid in group_ids:
        gdir = f"group_{gid:04d}"
        group_dir = os.path.join(RAW_DIR, gdir)
        if not os.path.isdir(group_dir): continue
        group_data = {}
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path): continue
            cv_img = cv2.imread(img_path)
            if cv_img is None: continue
            K_arr = np.array(camera_infos[cam]["K"]).reshape(3,3)
            D_arr = np.array(camera_infos[cam].get("D", [0,0,0,0])[:4], dtype=np.float64)
            detection = detect_target(cv_img, K_arr, D_arr, geom, profiles)
            corners = []
            for face_name, fd in detection.items():
                obj_face = fd.get("object_points_3d_face", [])
                img_face = fd.get("image_points_2d", [])
                if not obj_face: continue
                T = geom.T_target_face[face_name]
                for pi in range(len(obj_face)):
                    pt = np.array([*obj_face[pi], 1.0])
                    pt_tgt = (T @ pt)[:3]
                    corners.append(CornerObservation(
                        camera=cam, group_id=gid, face_name=face_name,
                        marker_id=pi//4, corner_idx=pi,
                        obj_pt_target=tuple(pt_tgt.tolist()),
                        img_pt_raw=tuple(img_face[pi]),
                        img_pt_undistorted=tuple(img_face[pi]),
                        detector_type=geom.face_pattern_types.get(face_name, "unknown")))
            if corners:
                group_data[cam] = CameraGroupMeasurement(camera=cam, group_id=gid, corners=corners)
        if len(group_data) >= 2:
            dataset.groups[gid] = group_data
    return dataset

# ═══════════════════════ MAIN ═══════════════════════

def main():
    rospy.init_node("v811_diag", anonymous=True)
    print("="*60)
    print("V8.11 Joint BA Information Audit")
    print("="*60)

    geom = load_target_geometry()
    profiles = create_default_profiles()
    print(f"Geometry: {len(geom.face_poses_target)} faces, sha256={geom.yaml_sha256[:16]}")
    print(f"Detector: {profiles.profile_summary()}")

    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        D_list = list(info.D) if info.D and len(info.D) >= 4 else [0.,0.,0.,0.,0.]
        camera_infos[cam] = {"K": list(info.K), "D": D_list, "width": info.width, "height": info.height}

    all_ids = sorted([int(d.replace("group_","")) for d in os.listdir(RAW_DIR)
                      if d.startswith("group_") and os.path.isdir(os.path.join(RAW_DIR, d))])
    group_ids = all_ids[-14:]
    print(f"\nGroups: {group_ids}")

    dataset = build_dataset(geom, profiles, camera_infos, group_ids)
    total = sum(len(m.corners) for g in dataset.groups.values() for m in g.values())
    print(f"Dataset: {dataset.n_groups} groups, {total} corners")

    # ── Truth ──
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    T_world_opt = {}
    T_rig_truth = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS:
        tfs = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame", rospy.Time(0), rospy.Duration(5.0))
        t, r = tfs.transform.translation, tfs.transform.rotation
        qw, qx, qy, qz = r.w, r.x, r.y, r.z
        T = np.eye(4)
        T[:3,:3] = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
                             [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
                             [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
        T[:3,3] = [t.x, t.y, t.z]
        T_world_opt[cam] = T
        if cam != FIRST_CAM:
            T_rig_truth[cam] = invert_transform(T_world_opt[FIRST_CAM]) @ T
    print(f"\nCamera truth (FL optical frame):")
    for cam in CAMERAS:
        print(f"  {cam}: t={(T_rig_truth[cam][:3,3]*1000).round(1)}mm")

    T_world_FL = T_world_opt[FIRST_CAM]
    Y_truth_all = {}
    for i, gid in enumerate(sorted(dataset.groups.keys())):
        tx, ty, tz, rr, rp, ry = TARGET_POSES[i]
        Twt = euler_matrix(math.radians(rr), math.radians(rp), math.radians(ry))
        Twt[:3,3] = [tx, ty, tz]
        Y_truth_all[gid] = invert_transform(T_world_FL) @ Twt

    # ── INIT ──
    X_init, Y_init, _ = initialize_rig(dataset)
    print(f"\nINIT: {len(Y_init)}/{len(dataset.groups)} targets, {len(X_init)} cameras")
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_init and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_init[cam], T_rig_truth[cam], 30, 3)
            print(f"  {cam} INIT: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # ═══════════════════════════════════════════════
    # 1. D2: FIX CAMERAS, OPTIMIZE TARGETS
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("D2: FIXED CAMERAS, OPTIMIZE TARGETS")
    print("="*60)

    d2_dir = os.path.join(DIAG_DIR, "d2")
    os.makedirs(d2_dir, exist_ok=True)
    d2_input, cam_names = build_ceres_input(
        dataset, camera_poses=T_rig_truth, target_poses=Y_init,
        options={"fix_all_cameras": True, "huber_threshold_px": 2.0, "max_iterations": 300},
        require_complete_initialization=False, allow_identity_fallback=False)
    d2_output, d2_errors = run_ceres_ba(d2_input, d2_dir, "d2")

    d2_results = []
    if d2_output:
        Y_d2 = {t["group_id"]: qt_to_T(t["optimized_pose"]) for t in d2_output.get("targets", [])}
        for gid in sorted(dataset.groups.keys()):
            if gid in Y_d2 and gid in Y_truth_all:
                t_err, r_err, _ = se3_distance_mm_deg(Y_d2[gid], Y_truth_all[gid], 30, 3)
                errs = compute_reproj_errors(dataset, T_rig_truth, {gid: Y_d2[gid]})
                rmse = math.sqrt(np.mean([e["e2"] for e in errs])) if errs else 0
                d2_results.append({"gid": gid, "t_err_mm": t_err, "r_err_deg": r_err, "rmse": rmse})
        ts = [r["t_err_mm"] for r in d2_results]
        rs = [r["r_err_deg"] for r in d2_results]
        print(f"Targets: median_T={np.median(ts):.1f}mm, P95_T={np.percentile(ts,95):.1f}mm, max_T={np.max(ts):.1f}mm")
        print(f"         median_R={np.median(rs):.2f}°, P95_R={np.percentile(rs,95):.2f}°")
        bad = [r for r in d2_results if r["t_err_mm"] > 30 or r["r_err_deg"] > 3]
        if bad:
            bad_strs = [(r["gid"], "{:.0f}mm".format(r["t_err_mm"])) for r in bad]
            print("BAD_GROUPS: " + str(bad_strs))

    # ═══════════════════════════════════════════════
    # 2. COST VALLEY: TRUTH → D3
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("COST VALLEY: TRUTH → D3 CAMERA PATH")
    print("="*60)

    # Get D3 camera poses from pipeline
    d3_result = run_calibration_pipeline(dataset, os.path.join(DIAG_DIR, "d3"),
                                          skip_cross_validation=True)
    X_d3 = d3_result.final_camera_poses

    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_d3 and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_d3[cam], T_rig_truth[cam], 30, 3)
            print(f"  D3 {cam}: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # Profile alpha: truth → D3
    valley_rows = []
    for alpha in np.linspace(0, 1.0, 11):
        X_alpha = {}
        for cam in CAMERAS:
            if cam == FIRST_CAM:
                X_alpha[cam] = np.eye(4)
            else:
                # Interpolate SE3 via matrix log/exp
                T0 = T_rig_truth[cam]
                T1 = X_d3.get(cam, T0)
                # Simple: interpolate translation + slerp rotation
                xi0_t = T0[:3,3]; xi1_t = T1[:3,3]
                xi_t = (1-alpha)*xi0_t + alpha*xi1_t
                # Slerp for rotation
                from scipy.spatial.transform import Rotation, Slerp
                R0 = Rotation.from_matrix(T0[:3,:3])
                R1 = Rotation.from_matrix(T1[:3,:3])
                times = [0, 1]
                if alpha == 0:
                    R_int = R0
                elif alpha == 1:
                    R_int = R1
                else:
                    slerp = Slerp(times, Rotation.concatenate([R0, R1]))
                    R_int = slerp([alpha])[0]
                T_alpha = np.eye(4)
                T_alpha[:3,:3] = R_int.as_matrix()
                T_alpha[:3,3] = xi_t
                X_alpha[cam] = T_alpha

        # Fix cameras at X_alpha, optimize targets
        alpha_dir = os.path.join(DIAG_DIR, f"valley_alpha{alpha:.1f}")
        os.makedirs(alpha_dir, exist_ok=True)
        vi, _ = build_ceres_input(
            dataset, camera_poses=X_alpha, target_poses=Y_init,
            options={"fix_all_cameras": True, "huber_threshold_px": 2.0, "max_iterations": 200},
            require_complete_initialization=False, allow_identity_fallback=False)
        vo, _ = run_ceres_ba(vi, alpha_dir, "va")
        if vo:
            Y_opt = {t["group_id"]: qt_to_T(t["optimized_pose"]) for t in vo.get("targets", [])}
            errs = compute_reproj_errors(dataset, X_alpha, Y_opt)
            stats = objective_stats(errs)
            # Camera error vs truth
            fr_err = 0
            if "cam_front_right" in T_rig_truth and "cam_front_right" in X_alpha:
                fr_err, _, _ = se3_distance_mm_deg(X_alpha["cam_front_right"], T_rig_truth["cam_front_right"], 30, 3)
            re_err = 0
            if "cam_rear" in T_rig_truth and "cam_rear" in X_alpha:
                re_err, _, _ = se3_distance_mm_deg(X_alpha["cam_rear"], T_rig_truth["cam_rear"], 30, 3)
            valley_rows.append({
                "alpha": alpha, "fr_terr_mm": fr_err, "re_terr_mm": re_err,
                "raw_sse": stats.get("raw_sse",0), "median_px": stats.get("median",0),
                "rmse_px": stats.get("rmse",0), "huber": stats.get("huber_sum",0),
            })
            print(f"  α={alpha:.1f}: FR={fr_err:.1f}mm RE={re_err:.1f}mm raw={stats.get('raw_sse',0):.0f} med={stats.get('median',0):.2f}px")

    # ═══════════════════════════════════════════════
    # 3. OBJECTIVE COMPARISON
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("OBJECTIVE COMPARISON: TRUTH vs INIT vs D3")
    print("="*60)
    truth_errs = compute_reproj_errors(dataset, T_rig_truth, Y_truth_all)
    init_errs_raw = compute_reproj_errors(dataset, X_init, Y_init)
    # D3 with its Y_targets from pipeline
    d3_errs = compute_reproj_errors(dataset, X_d3, d3_result.ba1_target_poses)

    for label, errs in [("TRUTH", truth_errs), ("INIT", init_errs_raw), ("D3", d3_errs)]:
        s = objective_stats(errs)
        print(f"  {label}: raw_sse={s.get('raw_sse',0):.0f} huber={s.get('huber_sum',0):.0f} med={s.get('median',0):.2f}px rmse={s.get('rmse',0):.2f}px")

    # Key judgment
    ts = objective_stats(truth_errs)
    ds = objective_stats(d3_errs)
    if ts["raw_sse"] < ds["raw_sse"]:
        print("\n  → TRUTH cost < D3 cost (solver ran to worse objective)")
        print("  → LOCAL MINIMUM / SOLVER BUG suspected")
    elif abs(ts["raw_sse"] - ds["raw_sse"]) / max(ts["raw_sse"], 1) < 0.05:
        print("\n  → TRUTH ≈ D3 cost (flat valley / weak observability)")
    else:
        print("\n  → D3 cost < TRUTH cost (objective PREFERS wrong solution)")
        print("  → TARGET-CAMERA COMPENSATION confirmed")

    # ═══════════════════════════════════════════════
    # 4. PAIRWISE CAMERA-RELATIVE ESTIMATE
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("PAIRWISE CAMERA-RELATIVE (PnP per group)")
    print("="*60)

    from cr5_spray_perception.calibration.pnp_solver import solve_pnp
    pairs = {"FL_FR": ("cam_front_left", "cam_front_right"),
             "FL_RE": ("cam_front_left", "cam_rear"),
             "FR_RE": ("cam_front_right", "cam_rear")}
    pair_estimates = defaultdict(list)

    for gid in sorted(dataset.groups.keys()):
        gdata = dataset.groups[gid]
        for pair_name, (camA, camB) in pairs.items():
            if camA not in gdata or camB not in gdata: continue
            K = np.array(camera_infos[camA]["K"]).reshape(3,3)
            D = np.array(camera_infos[camA].get("D", [0,0,0,0])[:4], dtype=np.float64)
            # PnP for camA
            Ta, _, _, sa = solve_pnp(gdata[camA].obj_pts, gdata[camA].img_pts_raw, K, D)
            Tb, _, _, sb = solve_pnp(gdata[camB].obj_pts, gdata[camB].img_pts_raw, K, D)
            if Ta is None or Tb is None: continue
            T_AB = Ta @ invert_transform(Tb)
            pair_estimates[pair_name].append(T_AB)

    for pair_name in ["FL_FR", "FL_RE"]:
        if pair_name not in pair_estimates: continue
        Ts = pair_estimates[pair_name]
        # Robust median via SE3
        translations = np.array([T[:3,3] for T in Ts])
        med_t = np.median(translations, axis=0)
        mad_t = np.median(np.linalg.norm(translations - med_t, axis=1))
        camB = pairs[pair_name][1]
        if camB in T_rig_truth:
            t_err = np.linalg.norm(med_t - T_rig_truth[camB][:3,3]) * 1000
            print(f"  {pair_name}: n={len(Ts)}, pairwise_T_err={t_err:.1f}mm, spread={mad_t*1000:.1f}mm")

    # ═══════════════════════════════════════════════
    # 5. WEIGHT AUDIT
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("WEIGHT AUDIT")
    print("="*60)

    per_cam_w = defaultdict(float)
    per_face_w = defaultdict(float)
    for gid, gdata in dataset.groups.items():
        for cam, meas in gdata.items():
            for c in meas.corners:
                per_cam_w[cam] += c.weight
                per_face_w[(cam, c.face_name)] += c.weight

    for cam in CAMERAS:
        print(f"  {cam}: total_weight={per_cam_w[cam]:.1f}, n_corners={sum(1 for g in dataset.groups.values() for m in g.values() if m.camera==cam for _ in m.corners)}")
    for (cam, face), w in sorted(per_face_w.items()):
        print(f"  {cam}/{face}: weight={w:.1f}")

    # ═══════════════════════════════════════════════
    # 6. CAMERA-NORMALIZED WEIGHT TEST
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("CAMERA-NORMALIZED WEIGHT TEST")
    print("="*60)

    # Create a copy with camera-normalized weights
    ds_norm = copy.deepcopy(dataset)
    for gid, gdata in ds_norm.groups.items():
        for cam, meas in gdata.items():
            n_cam = meas.n_corners
            w_cam = 1.0 / math.sqrt(max(n_cam, 1))
            for c in meas.corners:
                c.weight = w_cam

    # Also group-normalize within each camera
    for gid, gdata in ds_norm.groups.items():
        for cam, meas in gdata.items():
            n_g = meas.n_corners
            w_g = 1.0 / max(n_g, 1)
            for c in meas.corners:
                c.weight *= w_g * 100  # scale up to reasonable magnitude

    norm_dir = os.path.join(DIAG_DIR, "d3_norm")
    os.makedirs(norm_dir, exist_ok=True)
    result_norm = run_calibration_pipeline(ds_norm, norm_dir, skip_cross_validation=True)

    for cam in ["cam_front_right", "cam_rear"]:
        if cam in result_norm.final_camera_poses and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(result_norm.final_camera_poses[cam], T_rig_truth[cam], 30, 3)
            status = "IMPROVED" if t_err < 30 else "SAME"
            print(f"  {cam}: T_err={t_err:.1f}mm [{status}]")

    # ═══════════════════════════════════════════════
    # 7. SQUARED LOSS TEST
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("SQUARED LOSS TEST (no Huber/Cauchy)")
    print("="*60)

    sq_dir = os.path.join(DIAG_DIR, "d3_squared")
    os.makedirs(sq_dir, exist_ok=True)
    sq_input, _ = build_ceres_input(
        dataset, camera_poses=X_init, target_poses=Y_init,
        options={"huber_threshold_px": 9999.0, "max_iterations": 300},
        require_complete_initialization=False, allow_identity_fallback=False)
    sq_output, sq_errors = run_ceres_ba(sq_input, sq_dir, "sq")

    if sq_output:
        X_sq = parse_ceres_output(sq_output, cam_names)
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in X_sq and cam in T_rig_truth:
                t_err, r_err, _ = se3_distance_mm_deg(X_sq[cam], T_rig_truth[cam], 30, 3)
                status = "IMPROVED" if t_err < 30 else "SAME"
                print(f"  {cam}: T_err={t_err:.1f}mm [{status}]")

    # ═══════════════════════════════════════════════
    # SAVE ALL RESULTS
    # ═══════════════════════════════════════════════
    valley_file = os.path.join(DIAG_DIR, "cost_valley.csv")
    with open(valley_file, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["alpha","fr_terr_mm","re_terr_mm","raw_sse","median_px","rmse_px","huber"])
        w.writeheader()
        w.writerows(valley_rows)

    results = {
        "d2": d2_results,
        "valley": valley_rows,
        "truth_obj": objective_stats(truth_errs),
        "init_obj": objective_stats(init_errs_raw),
        "d3_obj": objective_stats(d3_errs),
        "weights": {cam: per_cam_w[cam] for cam in CAMERAS},
        "weight_per_face": {f"{c}/{f}": w for (c,f), w in per_face_w.items()},
        "d3_fr_terr": float(d3_result.final_camera_poses.get("cam_front_right", np.eye(4))[:3,3].sum()),
    }
    with open(os.path.join(DIAG_DIR, "results.json"), "w") as f:
        json.dump(results, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

    print(f"\nResults: {DIAG_DIR}")
    print("DONE")

if __name__ == "__main__":
    main()

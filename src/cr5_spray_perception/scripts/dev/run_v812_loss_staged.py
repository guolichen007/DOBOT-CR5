#!/usr/bin/env python3
"""V8.12: Robust loss sweep + staged BA + continuation experiments.

Key experiments:
  A: Independent loss sweep (squared, d2, d4, d6, d8, d12)
  B: Continuation from squared (d12→d8→d6→d4→d2)
  C: Staged squared (target-only → camera-only → joint)
  D: Robust target-only refinement
  E: Robust joint candidate with acceptance
"""
import os, sys, json, math, copy
import numpy as np
import rospy
import tf2_ros
from collections import defaultdict

WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)
from cr5_spray_perception.calibration.ceres_io import (
    build_ceres_input, run_ceres_ba, parse_ceres_output)
from cr5_spray_perception.calibration.rig_initializer import initialize_rig

OUTPUT_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                          "calibration", "runs", "sim_v8_e2e_001")
DIAG_DIR = os.path.join(OUTPUT_DIR, "v812_loss_staged")
RAW_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                       "calibration", "raw", "sim_v8_e2e_001", "groups")
CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
os.makedirs(DIAG_DIR, exist_ok=True)

TARGET_POSES = [
    (0.68,0,0.60, 0,0,0), (0.68,0,0.60, 0,0,15), (0.68,0,0.60, 0,0,-15),
    (0.68,0,0.60, 0,0,25), (0.68,0,0.60, 0,0,-25), (0.68,0,0.60, 0,10,0),
    (0.68,0,0.60, 0,-10,0), (0.68,0,0.60, 0,18,0), (0.68,0.06,0.58, 0,0,10),
    (0.68,-0.06,0.62, 0,0,-10), (0.68,0.04,0.56, 0,12,15), (0.68,-0.04,0.64, 0,-10,-15),
    (0.68,0,0.68, 0,-5,0), (0.68,0,0.50, 0,8,0),
]

def qt_to_T(qt):
    qw,qx,qy,qz = qt[:4]
    T = np.eye(4)
    T[:3,:3] = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
                         [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
                         [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
    T[:3,3] = qt[4:]; return T

def build_dataset(group_ids, geom, profiles, camera_infos):
    dataset = CalibrationDataset(camera_infos=camera_infos, face_poses_target=geom.face_poses_target,
                                  groups={}, source_type="gazebo")
    import cv2
    for gid in group_ids:
        group_dir = os.path.join(RAW_DIR, f"group_{gid:04d}")
        if not os.path.isdir(group_dir): continue
        group_data = {}
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path): continue
            cv_img = cv2.imread(img_path)
            if cv_img is None: continue
            K_arr = np.array(camera_infos[cam]["K"]).reshape(3,3)
            D_arr = np.array(camera_infos[cam].get("D",[0,0,0,0])[:4], dtype=np.float64)
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

def setup_weights(dataset):
    """Apply group-normalized per-corner weights (same as pipeline._set_initial_weights)."""
    for gid, gdata in dataset.groups.items():
        for cam, meas in gdata.items():
            n = meas.n_corners
            w_group = 1.0 / math.sqrt(max(n, 1))
            for c in meas.corners:
                c.weight = w_group

def run_ba(dataset, X_init, Y_init, huber_delta, label, fix_cameras=False, fix_targets=False,
           max_iter=300):
    # Apply weight normalization (critical for squared loss to find correct basin)
    setup_weights(dataset)
    d = os.path.join(DIAG_DIR, label)
    os.makedirs(d, exist_ok=True)
    opts = {"huber_threshold_px": huber_delta, "max_iterations": max_iter}
    if fix_cameras: opts["fix_all_cameras"] = True
    if fix_targets: opts["fix_all_targets"] = True
    inp, names = build_ceres_input(dataset, camera_poses=X_init, target_poses=Y_init,
                                    options=opts, require_complete_initialization=False,
                                    allow_identity_fallback=False)
    out, errs = run_ceres_ba(inp, d, label)
    if out is None: return None, None, errs
    X = parse_ceres_output(out, names)
    Y = {t["group_id"]: qt_to_T(t["optimized_pose"]) for t in out.get("targets", [])}
    return X, Y, out

def main():
    rospy.init_node("v812", anonymous=True)
    print("="*60)
    print("V8.12 Robust Loss / Staged BA")
    print("="*60)

    geom = load_target_geometry()
    profiles = create_default_profiles()
    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        camera_infos[cam] = {"K": list(info.K), "D": list(info.D) if info.D else [0]*5,
                              "width": info.width, "height": info.height}

    all_ids = sorted([int(d.replace("group_","")) for d in os.listdir(RAW_DIR)
                      if d.startswith("group_") and os.path.isdir(os.path.join(RAW_DIR, d))])
    group_ids = all_ids[-14:]
    print(f"Groups: {group_ids}")

    dataset = build_dataset(group_ids, geom, profiles, camera_infos)
    total = sum(len(m.corners) for g in dataset.groups.values() for m in g.values())
    print(f"Dataset: {dataset.n_groups} groups, {total} corners")

    # INIT
    X_init, Y_init, _ = initialize_rig(dataset)
    print(f"INIT: {len(Y_init)}/{len(dataset.groups)} targets")

    # Truth
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    T_world_opt = {}
    for cam in CAMERAS:
        tfs = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame", rospy.Time(0), rospy.Duration(5.0))
        t, r = tfs.transform.translation, tfs.transform.rotation
        T = np.eye(4)
        T[:3,:3] = np.array([[1-2*r.y**2-2*r.z**2, 2*r.x*r.y-2*r.z*r.w, 2*r.x*r.z+2*r.y*r.w],
                             [2*r.x*r.y+2*r.z*r.w, 1-2*r.x**2-2*r.z**2, 2*r.y*r.z-2*r.x*r.w],
                             [2*r.x*r.z-2*r.y*r.w, 2*r.y*r.z+2*r.x*r.w, 1-2*r.x**2-2*r.y**2]])
        T[:3,3] = [t.x, t.y, t.z]
        T_world_opt[cam] = T

    T_rig_truth = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS[1:]:
        T_rig_truth[cam] = invert_transform(T_world_opt[FIRST_CAM]) @ T_world_opt[cam]

    T_world_FL = T_world_opt[FIRST_CAM]
    Y_truth_all = {}
    for i, gid in enumerate(sorted(dataset.groups.keys())):
        tx,ty,tz,rr,rp,ry = TARGET_POSES[i]
        Twt = euler_matrix(math.radians(rr), math.radians(rp), math.radians(ry))
        Twt[:3,3] = [tx, ty, tz]
        Y_truth_all[gid] = invert_transform(T_world_FL) @ Twt

    # ═══════════════════════════════════════════════
    # EXPERIMENT A: Loss Sweep (independent)
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("A: INDEPENDENT LOSS SWEEP")
    print("="*60)

    deltas = [9999, 12, 8, 6, 4, 2]
    sweep_results = {}
    for d in deltas:
        label = "squared" if d >= 9999 else f"huber{d}"
        print(f"\n  --- {label} ---")
        X, Y, out = run_ba(dataset, X_init, Y_init, d, f"sweep_{label}")
        if X:
            for cam in ["cam_front_right", "cam_rear"]:
                if cam in X and cam in T_rig_truth:
                    t_err, r_err, _ = se3_distance_mm_deg(X[cam], T_rig_truth[cam], 30, 3)
                    print(f"  {cam}: T_err={t_err:.1f}mm R_err={r_err:.2f}° [{label}]")
            sweep_results[label] = {
                "fr_terr": float(se3_distance_mm_deg(X.get("cam_front_right", np.eye(4)),
                                        T_rig_truth.get("cam_front_right", np.eye(4)), 30, 3)[0]),
                "re_terr": float(se3_distance_mm_deg(X.get("cam_rear", np.eye(4)),
                                        T_rig_truth.get("cam_rear", np.eye(4)), 30, 3)[0]),
                "rmse": out.get("overall_rmse_px", 0),
                "final_cost": out.get("final_cost", 0),
            }

    # ═══════════════════════════════════════════════
    # EXPERIMENT B: Continuation
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("B: CONTINUATION from SQUARED")
    print("="*60)

    X_curr, Y_curr, _ = run_ba(dataset, X_init, Y_init, 9999, "cont_squared")
    if X_curr:
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in X_curr and cam in T_rig_truth:
                t_err, r_err, _ = se3_distance_mm_deg(X_curr[cam], T_rig_truth[cam], 30, 3)
                print(f"  S0 squared: {cam} T_err={t_err:.1f}mm")

    for d in [12, 8, 6, 4, 2]:
        label = f"cont_d{d}"
        X_next, Y_next, _ = run_ba(dataset, X_curr, Y_curr, d, label)
        if X_next:
            for cam in ["cam_front_right", "cam_rear"]:
                if cam in X_next and cam in T_rig_truth:
                    t_err, r_err, _ = se3_distance_mm_deg(X_next[cam], T_rig_truth[cam], 30, 3)
                    shift = 0
                    if cam in X_curr:
                        shift, _, _ = se3_distance_mm_deg(X_next[cam], X_curr[cam], 30, 3)
                    print(f"  d={d}: {cam} T_err={t_err:.1f}mm shift={shift:.1f}mm")
            X_curr, Y_curr = X_next, Y_next

    # ═══════════════════════════════════════════════
    # EXPERIMENT C: Staged Squared
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("C: STAGED SQUARED")
    print("="*60)

    # C1: Fix cameras (INIT), optimize targets with squared
    print("\n  C1: TARGET-ONLY squared")
    X_c1 = dict(X_init)
    _, Y_c1, _ = run_ba(dataset, X_c1, Y_init, 9999, "stage_c1", fix_cameras=True)

    # C2: Fix targets (C1 result), optimize cameras with squared
    print("  C2: CAMERA-ONLY squared")
    X_c2, Y_c2, _ = run_ba(dataset, X_c1, Y_c1, 9999, "stage_c2", fix_targets=True)

    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_c2 and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_c2[cam], T_rig_truth[cam], 30, 3)
            print(f"  {cam} C2: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # C3: ALL FREE squared
    print("  C3: JOINT squared")
    X_c3, Y_c3, _ = run_ba(dataset, X_c2, Y_c2, 9999, "stage_c3")
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_c3 and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_c3[cam], T_rig_truth[cam], 30, 3)
            print(f"  {cam} C3: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # ═══════════════════════════════════════════════
    # EXPERIMENT D: Robust target-only
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("D: ROBUST TARGET-ONLY (from C3 cameras)")
    print("="*60)

    for d in [6, 8, 12]:
        label = f"robust_tgt_d{d}"
        _, Y_r, _ = run_ba(dataset, X_c3, Y_c3, d, label, fix_cameras=True)
        if _:
            # Compute target displacement
            shifts = []
            for gid in Y_r:
                if gid in Y_c3:
                    s, _, _ = se3_distance_mm_deg(Y_r[gid], Y_c3[gid], 30, 3)
                    shifts.append(s)
            print(f"  d={d}: target median_shift={np.median(shifts):.1f}mm max_shift={np.max(shifts):.1f}mm")

    # ═══════════════════════════════════════════════
    # SUMMARY
    # ═══════════════════════════════════════════════
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    print("\nA: Independent Sweep")
    for label in ["squared", "huber12", "huber8", "huber6", "huber4", "huber2"]:
        if label in sweep_results:
            r = sweep_results[label]
            print(f"  {label}: FR={r['fr_terr']:.1f}mm RE={r['re_terr']:.1f}mm rmse={r['rmse']:.2f}px")

    # Save
    with open(os.path.join(DIAG_DIR, "v812_results.json"), "w") as f:
        json.dump(sweep_results, f, indent=2)

    print(f"\nResults: {DIAG_DIR}")
    print("DONE")

if __name__ == "__main__":
    main()

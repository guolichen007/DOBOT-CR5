#!/usr/bin/env python3
"""V8.10: Joint-BA observability root-cause diagnostic matrix (D0-D3).

Fixed-truth experiments to separate:
  A: joint BA observability / target-camera compensation
  B: measurement/model systematic error
  C: Ceres/frame contract bug
  D: target measurement/init problem
  E: truth comparison bug

Usage:
  source devel/setup.bash
  ROS_MASTER_URI=http://localhost:11313 python3 scripts/dev/run_v810_diagnostic_matrix.py
"""
import os, sys, json, math, copy
import numpy as np
import cv2
import rospy
import tf2_ros
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR))))
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.pipeline import run_calibration_pipeline
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)
from cr5_spray_perception.calibration.ceres_io import build_ceres_input, run_ceres_ba, parse_ceres_output
from cr5_spray_perception.calibration.rig_initializer import initialize_rig

OUTPUT_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                          "calibration", "runs", "sim_v8_e2e_001")
DIAG_DIR = os.path.join(OUTPUT_DIR, "v810_diagnostic")
RAW_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                       "calibration", "raw", "sim_v8_e2e_001", "groups")
CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]

def get_truth_from_tf():
    """从 TF 获取 Gazebo 真值相机位姿 (FL optical frame)."""
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    T_world_optical = {}
    for cam in CAMERAS:
        tfs = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame",
                                          rospy.Time(0), rospy.Duration(5.0))
        t, r = tfs.transform.translation, tfs.transform.rotation
        qw, qx, qy, qz = r.w, r.x, r.y, r.z
        T = np.eye(4)
        T[:3, :3] = np.array([
            [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
        T[:3, 3] = [t.x, t.y, t.z]
        T_world_optical[cam] = T

    T_rig = {FIRST_CAM: np.eye(4)}
    T_world_FL = T_world_optical[FIRST_CAM]
    for cam in CAMERAS[1:]:
        T_rig[cam] = invert_transform(T_world_FL) @ T_world_optical[cam]

    # Target truth: compute from Gazebo model state
    from gazebo_msgs.srv import GetModelState
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    resp = get_state("simple_hanging_workpiece", "world")
    if resp.success:
        p = resp.pose.position; r = resp.pose.orientation
        qw, qx, qy, qz = r.w, r.x, r.y, r.z
        T_world_target = np.eye(4)
        T_world_target[:3, :3] = np.array([
            [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
        T_world_target[:3, 3] = [p.x, p.y, p.z]
        T_rig_target = invert_transform(T_world_FL) @ T_world_target
    else:
        T_rig_target = None

    return T_rig, T_rig_target


def build_dataset_from_groups(group_ids, geom, profiles, camera_infos):
    """从指定 group IDs 构建 CalibrationDataset."""
    dataset = CalibrationDataset(
        camera_infos=camera_infos, face_poses_target=geom.face_poses_target,
        groups={}, source_type="gazebo")

    for gid in group_ids:
        gdir = f"group_{gid:04d}"
        group_dir = os.path.join(RAW_DIR, gdir)
        if not os.path.isdir(group_dir):
            continue
        group_data = {}
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path):
                continue
            cv_img = cv2.imread(img_path)
            if cv_img is None:
                continue
            K_arr = np.array(camera_infos[cam]["K"]).reshape(3, 3)
            D_arr = np.array(camera_infos[cam].get("D", [0,0,0,0])[:4], dtype=np.float64)
            detection = detect_target(cv_img, K_arr, D_arr, geom, profiles)

            corners = []
            for face_name, fd in detection.items():
                obj_face = fd.get("object_points_3d_face", [])
                img_face = fd.get("image_points_2d", [])
                if not obj_face:
                    continue
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


def compute_reprojection_residuals(dataset, X_cameras, Y_targets):
    """计算所有观测点的 reprojection residuals."""
    table = []
    per_cam = {}
    per_face = {}

    for gid, gdata in dataset.groups.items():
        if gid not in Y_targets:
            continue
        Y = Y_targets[gid]
        for cam, meas in gdata.items():
            if cam not in X_cameras:
                continue
            X = X_cameras[cam]
            T_cam_target = invert_transform(X) @ Y
            K = np.array(dataset.camera_infos[cam]["K"]).reshape(3, 3)
            D = dataset.camera_infos[cam].get("D", [0,0,0,0,0])

            for c in meas.corners:
                pt_tgt = np.array([*c.obj_pt_target, 1.0])
                pt_cam = T_cam_target @ pt_tgt
                if pt_cam[2] <= 0.001:
                    continue
                xp, yp = pt_cam[0]/pt_cam[2], pt_cam[1]/pt_cam[2]
                r2 = xp*xp + yp*yp
                r4 = r2*r2
                r6 = r2*r4
                radial = 1 + D[0]*r2 + D[1]*r4 + D[4]*r6
                x_dist = xp*radial + 2*D[2]*xp*yp + D[3]*(r2 + 2*xp*xp)
                y_dist = yp*radial + D[2]*(r2 + 2*yp*yp) + 2*D[3]*xp*yp
                u_pred = K[0,0]*x_dist + K[0,2]
                v_pred = K[1,1]*y_dist + K[1,2]
                u_raw, v_raw = c.img_pt_raw
                du, dv = u_raw - u_pred, v_raw - v_pred
                error_px = math.sqrt(du*du + dv*dv)

                table.append({"camera": cam, "face": c.face_name, "gid": gid,
                              "error_px": error_px, "du": du, "dv": dv})

    return table


def summarize_residuals(table):
    """汇总残差统计."""
    from collections import defaultdict
    all_errs = [r["error_px"] for r in table]
    result = {"overall": _stats(np.array(all_errs), "overall")}

    per_cam = defaultdict(list)
    per_face = defaultdict(list)
    for r in table:
        per_cam[r["camera"]].append(r["error_px"])
        per_face[(r["camera"], r["face"])].append(r["error_px"])

    for cam, errs in per_cam.items():
        result[f"cam_{cam}"] = _stats(np.array(errs), cam)
    for (cam, face), errs in sorted(per_face.items()):
        result[f"face_{cam}_{face}"] = _stats(np.array(errs), f"{cam}/{face}")

    return result


def _stats(errs, label):
    if len(errs) == 0:
        return {"label": label, "n": 0}
    return {"label": label, "n": len(errs),
            "median": float(np.median(errs)), "rmse": float(np.sqrt(np.mean(errs**2))),
            "p95": float(np.percentile(errs, 95)), "max": float(np.max(errs))}


def run_d0_truth_residual(dataset, T_rig_truth, geom):
    """D0: 全部真值固定, 只计算残差."""
    print("\n" + "="*60)
    print("D0: TRUTH/TRUTH Residual Only")
    print("="*60)

    # Use truth for cameras and create identity target (target at world pose in FL frame)
    # For D0, we need per-group target truth. Use the current target pose as approximation.
    # Actually, D0 uses the FACT that cameras are at truth and target is at truth.
    # We compute: T_rig_target for each group (from Gazebo).

    table = compute_reprojection_residuals(dataset, T_rig_truth, {})
    # For D0 we need actual target poses. Let's use a different approach:
    # Re-read from the pipeline since we need Y_targets per group.

    return None  # Will be computed in main with proper target poses


def run_fixed_target_camera_ba(dataset, X_init, Y_truth, output_dir):
    """D1: 固定 target truth, 只优化 camera extrinsics."""
    # Build Ceres input with fixed targets
    ceres_input, cam_names = build_ceres_input(
        dataset, camera_poses=X_init, target_poses=Y_truth,
        options={"fix_all_targets": True, "huber_threshold_px": 2.0, "max_iterations": 300},
        require_complete_initialization=False, allow_identity_fallback=False)

    output, errors = run_ceres_ba(ceres_input, output_dir, "d1")
    if output is None:
        return None, errors

    X_final = parse_ceres_output(output, cam_names)
    return X_final, output


def run_fixed_camera_target_ba(dataset, X_truth, Y_init, output_dir):
    """D2: 固定 camera truth, 只优化 target poses."""
    ceres_input, cam_names = build_ceres_input(
        dataset, camera_poses=X_truth, target_poses=Y_init,
        options={"fix_all_cameras": True, "huber_threshold_px": 2.0, "max_iterations": 300},
        require_complete_initialization=False, allow_identity_fallback=False)

    output, errors = run_ceres_ba(ceres_input, output_dir, "d2")
    if output is None:
        return None, errors

    Y_final = {t["group_id"]: qt_to_T(t["optimized_pose"])
               for t in output.get("targets", [])}
    return Y_final, output


def qt_to_T(qt):
    """[qw,qx,qy,qz,tx,ty,tz] → 4x4 matrix."""
    qw, qx, qy, qz = qt[:4]
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
    T[:3, 3] = qt[4:]
    return T


# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    rospy.init_node("v810_diag", anonymous=True)
    os.makedirs(DIAG_DIR, exist_ok=True)

    print("="*60)
    print("V8.10 Joint-BA Root-Cause Diagnostic Matrix")
    print("="*60)

    # ── Load geometry + profiles ──
    geom = load_target_geometry()
    profiles = create_default_profiles()
    print(f"Geometry: {len(geom.face_poses_target)} faces, sha256={geom.yaml_sha256[:16]}")
    print(f"Detector: {profiles.profile_summary()}")

    # ── Camera infos from ROS ──
    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        D_list = list(info.D) if info.D and len(info.D) >= 4 else [0.,0.,0.,0.,0.]
        camera_infos[cam] = {"K": list(info.K), "D": D_list,
                              "width": info.width, "height": info.height}
        print(f"{cam}: K=[{info.K[0]:.1f},{info.K[4]:.1f}]")

    # ── Find manual14 groups ──
    all_groups = sorted([int(d.replace("group_", "")) for d in os.listdir(RAW_DIR)
                         if d.startswith("group_") and os.path.isdir(os.path.join(RAW_DIR, d))])
    # Use the last 14 groups (from the manual14 capture)
    manual14_ids = all_groups[-14:]
    print(f"\nManual14 groups: {manual14_ids}")

    # ── Build dataset ──
    dataset = build_dataset_from_groups(manual14_ids, geom, profiles, camera_infos)
    total_c = sum(len(m.corners) for g in dataset.groups.values() for m in g.values())
    print(f"Dataset: {dataset.n_groups} groups, {total_c} corners")

    # ── Get truth from TF (camera poses only) ──
    print("\nGetting truth from TF...")
    T_rig_truth, _ = get_truth_from_tf()

    for cam in CAMERAS:
        if cam in T_rig_truth:
            print(f"  {cam}: t={(T_rig_truth[cam][:3,3]*1000).round(1)}mm")

    # Per-group target truth from commanded poses
    TARGET_POSES = [
        [0.68, 0.0, 0.60],  [0, 0, 0],
        [0.68, 0.0, 0.60],  [0, 0, 15],
        [0.68, 0.0, 0.60],  [0, 0, -15],
        [0.68, 0.0, 0.60],  [0, 0, 25],
        [0.68, 0.0, 0.60],  [0, 0, -25],
        [0.68, 0.0, 0.60],  [0, 10, 0],
        [0.68, 0.0, 0.60],  [0, -10, 0],
        [0.68, 0.0, 0.60],  [0, 18, 0],
        [0.68, 0.06, 0.58], [0, 0, 10],
        [0.68, -0.06, 0.62],[0, 0, -10],
        [0.68, 0.04, 0.56], [0, 12, 15],
        [0.68, -0.04, 0.64],[0, -10, -15],
        [0.68, 0.0, 0.68],  [0, -5, 0],
        [0.68, 0.0, 0.50],  [0, 8, 0],
    ]

    # Compute T_rig_target per group from commanded poses
    # Use TF to get T_world_FL for the transform to FL frame
    T_world_FL = T_rig_truth[FIRST_CAM]  # identity, but we need world→FL
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)
    tfs = tf_buffer.lookup_transform("world", f"{FIRST_CAM}_color_optical_frame",
                                      rospy.Time(0), rospy.Duration(5.0))
    t, r = tfs.transform.translation, tfs.transform.rotation
    qw, qx, qy, qz = r.w, r.x, r.y, r.z
    T_world_FL_full = np.eye(4)
    T_world_FL_full[:3, :3] = np.array([
        [1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw, 1-2*qx**2-2*qz**2, 2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])
    T_world_FL_full[:3, 3] = [t.x, t.y, t.z]

    Y_truth_all = {}
    for i, gid in enumerate(sorted(dataset.groups.keys())):
        xyz = TARGET_POSES[i*2]
        rpy_deg = TARGET_POSES[i*2 + 1]
        roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
        T_world_target = euler_matrix(roll, pitch, yaw)
        T_world_target[:3, 3] = xyz
        T_rig_target = invert_transform(T_world_FL_full) @ T_world_target
        Y_truth_all[gid] = T_rig_target

    print(f"  Per-group target truth: {len(Y_truth_all)} groups")

    # Save truth
    truth_data = {
        "cameras": {cam: T_rig_truth[cam].tolist() for cam in T_rig_truth},
        "targets": {str(gid): T.tolist() for gid, T in Y_truth_all.items()},
    }
    with open(os.path.join(DIAG_DIR, "diagnostic_truth.json"), "w") as f:
        json.dump(truth_data, f, indent=2)

    # ── D0: Truth/Truth residual ──
    print("\n" + "="*60)
    print("D0: TRUTH/TRUTH Residual")
    print("="*60)
    d0_table = compute_reprojection_residuals(dataset, T_rig_truth, Y_truth_all)
    d0_summary = summarize_residuals(d0_table)

    for key in ["overall", "cam_cam_front_left", "cam_cam_front_right", "cam_cam_rear"]:
        if key in d0_summary:
            s = d0_summary[key]
            print(f"  {s['label']}: n={s['n']}, med={s['median']:.2f}px, rmse={s['rmse']:.2f}px, p95={s['p95']:.2f}px")

    print("\n  Per face:")
    for key, s in sorted(d0_summary.items()):
        if key.startswith("face_"):
            print(f"  {s['label']}: n={s['n']}, med={s['median']:.2f}px, p95={s['p95']:.2f}px")

    with open(os.path.join(DIAG_DIR, "D0_truth_residuals.json"), "w") as f:
        json.dump({"summary": d0_summary, "table": d0_table}, f, indent=2)

    # ── D3 baseline: Normal joint BA ──
    print("\n" + "="*60)
    print("D3: NORMAL JOINT BA (baseline)")
    print("="*60)
    result_d3 = run_calibration_pipeline(
        dataset, os.path.join(DIAG_DIR, "d3_normal"),
        skip_cross_validation=True)

    for cam in ["cam_front_right", "cam_rear"]:
        if cam in T_rig_truth and cam in result_d3.final_camera_poses:
            t_err, r_err, _ = se3_distance_mm_deg(
                result_d3.final_camera_poses[cam], T_rig_truth[cam], 30, 3)
            print(f"  {cam}: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # ── D1: Fixed targets, optimize cameras ──
    print("\n" + "="*60)
    print("D1: FIXED TARGETS, OPTIMIZE CAMERAS")
    print("="*60)

    # Init from V8.5 initializer
    X_init, Y_init, init_diag = initialize_rig(dataset)
    print(f"  Init: {len(Y_init)}/{len(dataset.groups)} targets, {len(X_init)} cameras")
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_init and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_init[cam], T_rig_truth[cam], 30, 3)
            print(f"  {cam} INIT: T_err={t_err:.1f}mm R_err={r_err:.2f}°")

    # D1A: From V8.5 init
    print("\n  D1A: from V8.5 init...")
    d1a_dir = os.path.join(DIAG_DIR, "d1a")
    os.makedirs(d1a_dir, exist_ok=True)
    X_d1a, out_d1a = run_fixed_target_camera_ba(
        dataset, X_init, Y_truth_all, d1a_dir)

    if X_d1a:
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in X_d1a and cam in T_rig_truth:
                t_err, r_err, _ = se3_distance_mm_deg(X_d1a[cam], T_rig_truth[cam], 30, 3)
                print(f"  {cam} D1A: T_err={t_err:.1f}mm R_err={r_err:.2f}°")
        if out_d1a:
            print(f"  RMSE: {out_d1a.get('overall_rmse_px', '?'):.2f}px")

    # Save results
    results = {
        "d0_summary": d0_summary,
        "d3_fr_terr": None, "d3_re_terr": None,
        "d1a_fr_terr": None, "d1a_re_terr": None,
    }
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in T_rig_truth and cam in result_d3.final_camera_poses:
            t_err, r_err, _ = se3_distance_mm_deg(
                result_d3.final_camera_poses[cam], T_rig_truth[cam], 30, 3)
            results[f"d3_{cam.split('_')[-1]}_terr"] = t_err
            results[f"d3_{cam.split('_')[-1]}_rerr"] = r_err
    if X_d1a:
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in X_d1a and cam in T_rig_truth:
                t_err, r_err, _ = se3_distance_mm_deg(X_d1a[cam], T_rig_truth[cam], 30, 3)
                results[f"d1a_{cam.split('_')[-1]}_terr"] = t_err
                results[f"d1a_{cam.split('_')[-1]}_rerr"] = r_err

    with open(os.path.join(DIAG_DIR, "results.json"), "w") as f:
        json.dump(results, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating, np.integer)) else str(x))

    # ── Summary ──
    print("\n" + "="*60)
    print("DIAGNOSTIC MATRIX SUMMARY")
    print("="*60)
    print(f"\nD0 (truth residual): overall med={d0_summary.get('overall',{}).get('median','?'):.2f}px")
    if results.get("d3_right_terr"):
        print(f"D3 (normal BA):  FR={results['d3_right_terr']:.1f}mm RE={results['d3_rear_terr']:.1f}mm")
    if results.get("d1a_right_terr"):
        print(f"D1A (fixed tgt): FR={results['d1a_right_terr']:.1f}mm RE={results['d1a_rear_terr']:.1f}mm")
        if results['d1a_right_terr'] < 15:
            print("\n→ MEASUREMENT_CAN_RECOVER_CAMERAS = YES")
            print("  Joint BA observability is the primary problem.")
        else:
            print("\n→ MEASUREMENT_SYSTEMATIC_BIAS = YES")
            print("  Even with target truth, cameras can't recover.")

    print(f"\nResults saved: {DIAG_DIR}")
    print("DONE")


if __name__ == "__main__":
    main()

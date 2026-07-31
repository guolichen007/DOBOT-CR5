#!/usr/bin/env python3
"""V8.13: Camera-Relative Calibration + Observability Analysis.

Robust pairwise SE3 aggregation from per-group PnP hypotheses.
No Gazebo truth used in solver. Truth only for diagnostic.
"""
import os, sys, json, math, copy, csv
import numpy as np
from collections import defaultdict
import cv2
import rospy
import tf2_ros

WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)
from cr5_spray_perception.calibration.pnp_solver import solve_pnp
from scipy.spatial.transform import Rotation

OUTPUT_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                          "calibration", "runs", "sim_v8_e2e_001")
DIAG_DIR = os.path.join(OUTPUT_DIR, "v813_pairwise")
RAW_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                       "calibration", "raw", "sim_v8_e2e_001", "groups")
CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
os.makedirs(DIAG_DIR, exist_ok=True)

# ═══════════════════════════════════════════════════════════════
# Robust SE3 aggregation
# ═══════════════════════════════════════════════════════════════

def se3_log(T):
    """SE(3) logarithm: [rho, omega] where rho is translation, omega is rotation vector."""
    R = T[:3, :3]
    t = T[:3, 3]
    # Rotation vector from matrix log
    theta = math.acos(max(-1, min(1, (np.trace(R) - 1) / 2)))
    if theta < 1e-10:
        omega = np.zeros(3)
        V_inv = np.eye(3)
    else:
        omega = theta / (2 * math.sin(theta)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        # V inverse
        s = math.sin(theta); c = math.cos(theta)
        A = s / theta if theta > 1e-10 else 1.0
        B = (1 - c) / (theta * theta) if theta > 1e-10 else 0.5
        omega_hat = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])
        V = np.eye(3) + B * omega_hat + (1 - A) / (theta*theta) * omega_hat @ omega_hat
        V_inv = np.linalg.inv(V) if np.linalg.cond(V) < 1e10 else np.eye(3)
    rho = V_inv @ t
    return np.concatenate([rho, omega])

def se3_exp(xi):
    """SE(3) exponential: xi = [rho, omega] → 4x4 matrix."""
    rho = xi[:3]; omega = xi[3:]
    theta = np.linalg.norm(omega)
    T = np.eye(4)
    if theta < 1e-10:
        T[:3, 3] = rho
        return T
    omega_hat = np.array([[0, -omega[2], omega[1]], [omega[2], 0, -omega[0]], [-omega[1], omega[0], 0]])
    s = math.sin(theta); c = math.cos(theta)
    R = np.eye(3) + s/theta * omega_hat + (1-c)/(theta*theta) * omega_hat @ omega_hat
    V = np.eye(3) + (1-c)/(theta*theta) * omega_hat + (theta-s)/(theta*theta*theta) * omega_hat @ omega_hat
    T[:3, :3] = R
    T[:3, 3] = V @ rho
    return T

def se3_mean(transforms, weights=None, max_iter=20, tol=1e-8):
    """Iterative SE(3) mean via log/exp averaging."""
    if len(transforms) == 0:
        return None
    if weights is None:
        weights = np.ones(len(transforms))
    weights = np.asarray(weights) / np.sum(weights)

    # Initialize with weighted translation mean + quaternion mean
    t_mean = np.average([T[:3, 3] for T in transforms], axis=0, weights=weights)
    rots = Rotation.from_matrix([T[:3, :3] for T in transforms])
    q_mean = rots.mean(weights=weights).as_matrix()
    T_mean = np.eye(4); T_mean[:3, :3] = q_mean; T_mean[:3, 3] = t_mean

    for _ in range(max_iter):
        xi_sum = np.zeros(6)
        for i, T in enumerate(transforms):
            delta = np.linalg.inv(T_mean) @ T
            xi = se3_log(delta)
            xi_sum += weights[i] * xi
        if np.linalg.norm(xi_sum) < tol:
            break
        T_mean = T_mean @ se3_exp(xi_sum)
    return T_mean

def robust_se3_consensus(transforms, inlier_t_mm=50.0, inlier_r_deg=5.0):
    """RANSAC-based robust SE3 consensus from list of transforms.

    Returns:
        dict with median, inliers, outliers, stats
    """
    if len(transforms) < 3:
        if len(transforms) >= 1:
            return {"median": transforms[0], "inliers": list(range(len(transforms))),
                    "outliers": [], "n": len(transforms)}
        return None

    n = len(transforms)
    best_inliers = []
    best_T = None

    # RANSAC: try each candidate as seed, pick one with most inliers
    for seed_idx in range(min(n, 20)):  # try up to 20 seeds
        seed = transforms[seed_idx]
        inliers = []
        for j in range(n):
            t_err, r_err, _ = se3_distance_mm_deg(transforms[j], seed, inlier_t_mm, inlier_r_deg)
            if t_err < inlier_t_mm and r_err < inlier_r_deg:
                inliers.append(j)
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < max(2, n // 3):
        # Low consensus: use all as inliers
        best_inliers = list(range(n))

    outliers = [j for j in range(n) if j not in best_inliers]

    # Refine: SE3 mean of inliers
    inlier_transforms = [transforms[j] for j in best_inliers]
    refined = se3_mean(inlier_transforms)

    # Compute residuals
    residuals_t = []
    residuals_r = []
    for j in best_inliers:
        t_err, r_err, _ = se3_distance_mm_deg(transforms[j], refined, 100, 10)
        residuals_t.append(t_err)
        residuals_r.append(r_err)

    result = {
        "median": refined,
        "n_total": n,
        "n_inliers": len(best_inliers),
        "n_outliers": len(outliers),
        "outlier_indices": outliers,
        "t_median": float(np.median(residuals_t)) if residuals_t else 0,
        "t_mad": float(np.median(np.abs(np.array(residuals_t) - np.median(residuals_t)))) if residuals_t else 0,
        "t_p95": float(np.percentile(residuals_t, 95)) if len(residuals_t) >= 20 else float(np.max(residuals_t)) if residuals_t else 0,
        "r_median": float(np.median(residuals_r)) if residuals_r else 0,
        "r_mad": float(np.median(np.abs(np.array(residuals_r) - np.median(residuals_r)))) if residuals_r else 0,
        "r_p95": float(np.percentile(residuals_r, 95)) if len(residuals_r) >= 20 else float(np.max(residuals_r)) if residuals_r else 0,
    }
    return result

# ═══════════════════════════════════════════════════════════════

def build_dataset(group_ids, geom, profiles, camera_infos):
    """Build CalibrationDataset from raw images."""
    dataset = CalibrationDataset(camera_infos=camera_infos, face_poses_target=geom.face_poses_target,
                                  groups={}, source_type="gazebo")
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
                    pt = np.array([*obj_face[pi], 1.0]); pt_tgt = (T @ pt)[:3]
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

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    rospy.init_node("v813_pairwise", anonymous=True)
    print("="*60)
    print("V8.13 Camera-Relative Calibration")
    print("="*60)

    geom = load_target_geometry(); profiles = create_default_profiles()
    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        D_list = list(info.D) if info.D and len(info.D) >= 4 else [0.,0.,0.,0.,0.]
        camera_infos[cam] = {"K": list(info.K), "D": D_list, "width": info.width, "height": info.height}

    all_ids = sorted([int(d.replace("group_","")) for d in os.listdir(RAW_DIR)
                      if d.startswith("group_") and os.path.isdir(os.path.join(RAW_DIR, d))])
    group_ids = all_ids[-14:]
    print(f"Groups: {group_ids}")

    dataset = build_dataset(group_ids, geom, profiles, camera_infos)
    total = sum(len(m.corners) for g in dataset.groups.values() for m in g.values())

    # Per-group face info
    print("\nPer-group face coverage:")
    for gid in sorted(dataset.groups.keys()):
        gdata = dataset.groups[gid]
        parts = []
        for cam in CAMERAS:
            if cam in gdata:
                faces = set(c.face_name for c in gdata[cam].corners)
                n = len(gdata[cam].corners)
                parts.append(f"{cam}:{n}cp/{'+'.join(sorted(faces))}")
        print(f"  G{gid}: " + " | ".join(parts))

    # ── Phase A: Per-group PnP → camera-relative transforms ──
    print("\n" + "="*60)
    print("Phase A: Pairwise Camera-Relative from PnP")
    print("="*60)

    pairs = [("FL_FR", "cam_front_left", "cam_front_right"),
             ("FL_RE", "cam_front_left", "cam_rear"),
             ("FR_RE", "cam_front_right", "cam_rear")]

    pair_transforms = defaultdict(list)  # pair_name → [T_camA_camB]
    pair_group_ids = defaultdict(list)

    for gid in sorted(dataset.groups.keys()):
        gdata = dataset.groups[gid]
        for pair_name, camA, camB in pairs:
            if camA not in gdata or camB not in gdata: continue
            K_A = np.array(camera_infos[camA]["K"]).reshape(3,3)
            D_A = np.array(camera_infos[camA].get("D",[0,0,0,0])[:4], dtype=np.float64)
            K_B = np.array(camera_infos[camB]["K"]).reshape(3,3)
            D_B = np.array(camera_infos[camB].get("D",[0,0,0,0])[:4], dtype=np.float64)

            T_A_target, _, _, sa = solve_pnp(gdata[camA].obj_pts, gdata[camA].img_pts_raw, K_A, D_A)
            T_B_target, _, _, sb = solve_pnp(gdata[camB].obj_pts, gdata[camB].img_pts_raw, K_B, D_B)
            if T_A_target is None or T_B_target is None: continue
            # T_camA_camB = T_camA_target @ inv(T_camB_target)
            T_AB = T_A_target @ invert_transform(T_B_target)
            pair_transforms[pair_name].append(T_AB)
            pair_group_ids[pair_name].append(gid)

    # ── Phase D: Robust SE3 aggregation ──
    print("\n" + "="*60)
    print("Phase D: Robust SE3 Consensus")
    print("="*60)

    consensus = {}
    for pair_name, transforms in pair_transforms.items():
        result = robust_se3_consensus(transforms, inlier_t_mm=50.0, inlier_r_deg=5.0)
        consensus[pair_name] = result
        if result:
            gids = pair_group_ids[pair_name]
            outliers = [gids[i] for i in result.get("outlier_indices", [])]
            print(f"\n{pair_name}: n={result['n_total']}, inliers={result['n_inliers']}, outliers={outliers}")
            print(f"  T: med={result['t_median']:.1f}mm MAD={result['t_mad']:.1f}mm P95={result['t_p95']:.1f}mm")
            print(f"  R: med={result['r_median']:.2f}° MAD={result['r_mad']:.2f}°")

    # ── Phase B: Camera Seed ──
    print("\n" + "="*60)
    print("Phase B: Camera Seed from Pairwise")
    print("="*60)

    T_FL_FR = consensus.get("FL_FR", {}).get("median")
    T_FL_RE = consensus.get("FL_RE", {}).get("median")
    T_FR_RE = consensus.get("FR_RE", {}).get("median")

    X_seed = {"cam_front_left": np.eye(4)}
    if T_FL_FR is not None:
        X_seed["cam_front_right"] = T_FL_FR
    if T_FL_RE is not None:
        X_seed["cam_front_rear"] = T_FL_RE  # note: key name
    # Remap key
    if "cam_front_rear" in X_seed:
        X_seed["cam_rear"] = X_seed.pop("cam_front_rear")

    # Triangle closure
    if T_FL_FR is not None and T_FR_RE is not None and T_FL_RE is not None:
        T_triangle = T_FL_FR @ T_FR_RE
        t_close, r_close, _ = se3_distance_mm_deg(T_triangle, T_FL_RE, 50, 5)
        print(f"\nTriangle closure FL→FR→RE vs FL→RE:")
        print(f"  T_err={t_close:.1f}mm R_err={r_close:.2f}°")

    # ── Truth diagnostic comparison ──
    tf_buffer = tf2_ros.Buffer(); tf_listener = tf2_ros.TransformListener(tf_buffer); rospy.sleep(1.0)
    T_world_opt = {}
    for cam in CAMERAS:
        tfs = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame", rospy.Time(0), rospy.Duration(5.0))
        t, r = tfs.transform.translation, tfs.transform.rotation
        T = np.eye(4)
        T[:3,:3] = np.array([[1-2*r.y**2-2*r.z**2,2*r.x*r.y-2*r.z*r.w,2*r.x*r.z+2*r.y*r.w],
                             [2*r.x*r.y+2*r.z*r.w,1-2*r.x**2-2*r.z**2,2*r.y*r.z-2*r.x*r.w],
                             [2*r.x*r.z-2*r.y*r.w,2*r.y*r.z+2*r.x*r.w,1-2*r.x**2-2*r.y**2]])
        T[:3,3] = [t.x, t.y, t.z]; T_world_opt[cam] = T
    T_rig_truth = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS[1:]:
        T_rig_truth[cam] = invert_transform(T_world_opt[FIRST_CAM]) @ T_world_opt[cam]

    print("\n" + "="*60)
    print("CAMERA SEED vs TRUTH (diagnostic only)")
    print("="*60)
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_seed and cam in T_rig_truth:
            t_err, r_err, _ = se3_distance_mm_deg(X_seed[cam], T_rig_truth[cam], 30, 3)
            status = "PASS" if t_err < 10 and r_err < 1.0 else ("CLOSE" if t_err < 20 else "FAIL")
            print(f"  {cam}: T_err={t_err:.1f}mm R_err={r_err:.2f}° [{status}]")
            print(f"    seed t={(X_seed[cam][:3,3]*1000).round(1)}mm")
            print(f"    truth t={(T_rig_truth[cam][:3,3]*1000).round(1)}mm")

    # ── Observability gate ──
    print("\n" + "="*60)
    print("OBSERVABILITY GATE")
    print("="*60)

    all_pass = True
    for cam in ["cam_front_right", "cam_rear"]:
        if cam in X_seed and cam in T_rig_truth:
            t_err, _, _ = se3_distance_mm_deg(X_seed[cam], T_rig_truth[cam], 30, 3)
            if t_err > 15:
                all_pass = False
                print(f"  {cam}: {t_err:.1f}mm > 15mm → FAIL")

    # Check spread
    for pair_name in ["FL_FR", "FL_RE"]:
        r = consensus.get(pair_name, {})
        if r.get("t_mad", 999) > 15 or r.get("n_inliers", 0) < 5:
            all_pass = False
            print(f"  {pair_name}: spread={r.get('t_mad',999):.1f}mm inliers={r.get('n_inliers',0)} → FAIL")

    if all_pass:
        print("\n  → PAIRWISE_CORE_PASS: enter refinement")
        observability = "SUFFICIENT"
    else:
        print("\n  → DATA_OBSERVABILITY_INSUFFICIENT: need more poses")
        observability = "INSUFFICIENT"

    # ── Save ──
    # Save seed matrices for refinement
    seed_matrices = {}
    for cam in CAMERAS:
        if cam in X_seed:
            seed_matrices[cam] = X_seed[cam].tolist()
    with open(os.path.join(DIAG_DIR, "v813_seed_matrices.json"), "w") as f:
        json.dump(seed_matrices, f, indent=2)

    results = {
        "observability": observability,
        "pairs": {pn: {k: v for k, v in r.items() if k != "median"}
                  for pn, r in consensus.items()},
        "seed_fr_terr": float(se3_distance_mm_deg(X_seed.get("cam_front_right", np.eye(4)),
                            T_rig_truth.get("cam_front_right", np.eye(4)), 30, 3)[0]),
        "seed_re_terr": float(se3_distance_mm_deg(X_seed.get("cam_rear", np.eye(4)),
                            T_rig_truth.get("cam_rear", np.eye(4)), 30, 3)[0]),
        "triangle_closure_t_mm": float(t_close) if T_FL_FR is not None and T_FL_RE is not None else None,
    }
    with open(os.path.join(DIAG_DIR, "v813_pairwise_results.json"), "w") as f:
        json.dump(results, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

    print(f"\nResults: {DIAG_DIR}")
    print(f"Observability: {observability}")
    print("DONE")

if __name__ == "__main__":
    main()

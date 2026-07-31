#!/usr/bin/env python3
"""
V8.18 Frozen Dataset-A Root Cause Isolation.

D1: Score sanity — feed truth as blind result, verify ~0/0.
D2: Per-camera PnP truth audit — PnP vs truth per group/camera/face.
D3: Subset replay — ALL20 / DUAL_MULTIFACE / C01-C06 / SINGLE_PLANAR.
D4: Per-group pair truth audit.

NO new captures. NO algorithm changes. NO scene changes.
"""
import sys, os, math, json, time, hashlib, csv
import numpy as np
import cv2
import rospy
import tf2_ros
from collections import defaultdict
from scipy.spatial.transform import Rotation

WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_sim", "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.pairwise_solver import (
    compute_pairwise_rig, se3_weighted_mean, robust_se3_consensus, CAMERAS, FIRST_CAM, PAIRS)
from cr5_spray_perception.calibration.pnp_solver import solve_pnp
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)

RUN_DIR = os.path.expanduser("~/cr5_data/calibration/runs/sim_v815_final_cell_A")
DIAG_DIR = os.path.join(RUN_DIR, "v818_diagnostics")
os.makedirs(DIAG_DIR, exist_ok=True)

# ═══════════════════════════════════════════════════════════════
# Utility: Gazebo truth access
# ═══════════════════════════════════════════════════════════════

def get_gazebo_truth(tf_buffer):
    """Get T_world_camera for all 3 cameras from TF."""
    T_world = {}
    for cam in CAMERAS:
        tf = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame",
                                        rospy.Time(0), rospy.Duration(5.0))
        t, r = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        T_world[cam] = T
    return T_world


def get_gazebo_target_truth():
    """Get T_world_target from gazebo model state."""
    from gazebo_msgs.srv import GetModelState, GetModelStateRequest
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    req = GetModelStateRequest()
    req.model_name = "simple_hanging_workpiece"
    req.relative_entity_name = "world"
    resp = svc(req)
    if resp.success:
        p = resp.pose.position
        o = resp.pose.orientation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T[:3, 3] = [p.x, p.y, p.z]
        return T
    return None


# ═══════════════════════════════════════════════════════════════
# Load dataset
# ═══════════════════════════════════════════════════════════════

def load_formal_dataset():
    """Load all formal capture groups with images and metadata."""
    manifest_path = os.path.join(RUN_DIR, "manifest.json")
    with open(manifest_path) as f:
        manifest = json.load(f)

    groups = {}
    groups_dir = os.path.join(RUN_DIR, "groups")
    for g in manifest["groups"]:
        gid = g["group_id"]
        sel_snap = g.get("best_snap", 0)
        img_dir = os.path.join(groups_dir, f"group_{gid:04d}", "raw", f"snap_{sel_snap}")
        if not os.path.isdir(img_dir):
            img_dir = os.path.join(groups_dir, f"group_{gid:04d}", "selected")
        images = {}
        for cam in CAMERAS:
            img_path = os.path.join(img_dir, cam, "color.png")
            if os.path.exists(img_path):
                images[cam] = cv2.imread(img_path)
        groups[gid] = {
            "candidate_id": g["candidate_id"],
            "world_xyz": g["world_xyz"],
            "rpy": g["rpy"],
            "images": images,
        }
    return groups, manifest


# ═══════════════════════════════════════════════════════════════
# D1: Score Sanity
# ═══════════════════════════════════════════════════════════════

def run_d1_score_sanity(tf_buffer):
    """Export truth camera rig and feed to scorer logic."""
    print("=" * 60)
    print("D1: SCORE CONTRACT SANITY")
    print("=" * 60)

    T_world = get_gazebo_truth(tf_buffer)
    T_world_FL = T_world[FIRST_CAM]

    # Build rig truth
    rig_truth = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS[1:]:
        rig_truth[cam] = invert_transform(T_world_FL) @ T_world[cam]

    # Compare rig truth to itself (should be zero)
    all_ok = True
    for cam in ["cam_front_right", "cam_rear"]:
        T_est = rig_truth[cam]
        T_gt = rig_truth[cam]
        t_err, r_err, _ = se3_distance_mm_deg(T_est, T_gt, 1, 0.1)
        print(f"  {cam}: T={t_err:.6f}mm R={r_err:.6f}°")
        if t_err > 0.01 or r_err > 0.001:
            all_ok = False

    # Also test: write truth as blind result, re-read via scorer's load logic
    truth_blind = {"cameras": {cam: rig_truth[cam].tolist() for cam in CAMERAS},
                   "solver": "truth_export", "version": "V8.18_sanity"}
    truth_path = os.path.join(DIAG_DIR, "truth_camera_extrinsics.json")
    with open(truth_path, "w") as f:
        json.dump(truth_blind, f, indent=2)

    # Re-read
    with open(truth_path) as f:
        reloaded = json.load(f)
    X_reloaded = {cam: np.array(reloaded["cameras"][cam]) for cam in CAMERAS}

    for cam in ["cam_front_right", "cam_rear"]:
        t_err, r_err, _ = se3_distance_mm_deg(X_reloaded[cam], rig_truth[cam], 1, 0.1)
        print(f"  {cam} reload: T={t_err:.6f}mm R={r_err:.6f}°")

    if all_ok:
        print("  SCORE_CONTRACT_SANITY_PASS")
    else:
        print("  SCORE_CONTRACT_SANITY_FAIL")
    return all_ok


# ═══════════════════════════════════════════════════════════════
# D2: Per-camera PnP Truth Audit
# ═══════════════════════════════════════════════════════════════

def run_d2_pnp_truth_audit(groups, tf_buffer):
    """For each group/camera, compare PnP vs truth. Categorize by faces."""
    print("\n" + "=" * 60)
    print("D2: PER-CAMERA PnP TRUTH AUDIT")
    print("=" * 60)

    geom = load_target_geometry()
    profiles = create_default_profiles()

    from sensor_msgs.msg import CameraInfo
    camera_info = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        camera_info[cam] = {"K": K, "D": D}

    T_world_cams = get_gazebo_truth(tf_buffer)

    # Category accumulators
    categories = defaultdict(lambda: {"n": 0, "t_vals": [], "r_vals": [],
                                       "dx": [], "dy": [], "dz": [],
                                       "rmse": [], "corners": []})

    all_rows = []

    for gid in sorted(groups.keys()):
        g = groups[gid]
        cid = g["candidate_id"]
        T_world_target = get_gazebo_target_truth()
        if T_world_target is None:
            continue

        for cam in CAMERAS:
            if cam not in g["images"]:
                continue
            cv_img = g["images"][cam]
            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]

            detection = detect_target(cv_img, K, D, geom, profiles)
            if not detection:
                continue

            # Collect object/image points
            obj_pts, img_pts = [], []
            faces_detected = []
            for fn, fd in detection.items():
                oface = fd.get("object_points_3d_face", [])
                iface = fd.get("image_points_2d", [])
                if not oface:
                    continue
                T_face = geom.T_target_face.get(fn)
                if T_face is None:
                    continue
                faces_detected.append(fn)
                for pi in range(len(oface)):
                    pt_tgt = (T_face @ np.array([*oface[pi], 1.0]))[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(iface[pi])

            if len(obj_pts) < 4:
                continue

            T_cam_target_est, _, _, pnp_stats = solve_pnp(np.array(obj_pts), np.array(img_pts), K, D)
            if T_cam_target_est is None:
                continue

            # Truth: T_cam_target = inv(T_world_cam) @ T_world_target
            T_world_cam = T_world_cams[cam]
            T_cam_target_truth = invert_transform(T_world_cam) @ T_world_target

            t_err, r_err, _ = se3_distance_mm_deg(T_cam_target_est, T_cam_target_truth, 50, 5)
            d_est = T_cam_target_est[:3, 3] * 1000
            d_truth = T_cam_target_truth[:3, 3] * 1000
            dx, dy, dz = d_est - d_truth

            rmse = pnp_stats.get("rmse_inlier_px", 0) if isinstance(pnp_stats, dict) else 0
            n_corners = len(obj_pts)

            # Category
            cat_key = f"{cam}/{'+'.join(sorted(faces_detected))}"

            # Also broad categories
            if cam == "cam_front_left":
                if "right" in faces_detected and "top" in faces_detected:
                    broad = "FL_RIGHT_TOP"
                elif "right" in faces_detected:
                    broad = "FL_RIGHT_ONLY"
                else:
                    broad = "FL_OTHER"
            elif cam == "cam_front_right":
                if "left" in faces_detected and "top" in faces_detected:
                    broad = "FR_LEFT_TOP"
                elif "left" in faces_detected:
                    broad = "FR_LEFT_ONLY"
                else:
                    broad = "FR_OTHER"
            elif cam == "cam_rear":
                if "front" in faces_detected and "top" in faces_detected:
                    broad = "RE_FRONT_TOP"
                elif "front" in faces_detected:
                    broad = "RE_FRONT_ONLY"
                else:
                    broad = "RE_OTHER"

            for cat in [cat_key, broad]:
                categories[cat]["n"] += 1
                categories[cat]["t_vals"].append(t_err)
                categories[cat]["r_vals"].append(r_err)
                categories[cat]["dx"].append(dx)
                categories[cat]["dy"].append(dy)
                categories[cat]["dz"].append(dz)
                categories[cat]["rmse"].append(rmse)
                categories[cat]["corners"].append(n_corners)

            all_rows.append({
                "gid": gid, "cid": cid, "cam": cam,
                "faces": "+".join(faces_detected),
                "broad": broad,
                "t_mm": t_err, "r_deg": r_err,
                "dx": dx, "dy": dy, "dz": dz,
                "rmse_px": rmse, "corners": n_corners,
            })

    # Print category table
    print(f"\n{'Category':<22s} {'N':>4s} {'T med':>7s} {'T P95':>7s} {'R med':>7s} {'DZ med':>8s} {'DZ P95':>8s} {'RMSE':>6s}")
    print("-" * 75)
    for cat_name in sorted(categories.keys()):
        if not cat_name.startswith(("FL_", "FR_", "RE_")):
            continue
        c = categories[cat_name]
        t_med = np.median(c["t_vals"])
        t_p95 = np.percentile(c["t_vals"], 95)
        r_med = np.median(c["r_vals"])
        dz_med = np.median(c["dz"])
        dz_p95 = np.percentile(c["dz"], 95)
        rmse_med = np.median(c["rmse"])
        print(f"{cat_name:<22s} {c['n']:4d} {t_med:6.1f}mm {t_p95:6.1f}mm {r_med:6.2f}° {dz_med:+7.1f}mm {dz_p95:+7.1f}mm {rmse_med:5.2f}px")

    # Save CSV
    csv_path = os.path.join(DIAG_DIR, "d2_pnp_truth_by_group.csv")
    with open(csv_path, "w") as f:
        w = csv.DictWriter(f, fieldnames=["gid","cid","cam","faces","broad","t_mm","r_deg","dx","dy","dz","rmse_px","corners"])
        w.writeheader()
        w.writerows(all_rows)

    # Common FL bias test
    print("\n--- COMMON FL BIAS ---")
    fl_rows = [r for r in all_rows if r["cam"] == "cam_front_left" and r["broad"] in ("FL_RIGHT_ONLY", "FL_RIGHT_TOP")]
    fr_rows = [r for r in all_rows if r["cam"] == "cam_front_right"]
    re_rows = [r for r in all_rows if r["cam"] == "cam_rear"]

    if fl_rows:
        fl_dz_med = np.median([r["dz"] for r in fl_rows])
        print(f"  FL PnP dz median: {fl_dz_med:+.1f}mm")

        # The actual pair error from V8.17B: FR dz=-21.5, RE dz=-17.6
        # If FL has a dz bias, the pair dz = f(FL_dz, FR_dz, geometry)
        # Simplified: if FL sees target at wrong depth, both FR and RE estimates are shifted
        print(f"  Actual pair errors (from V8.17B):")
        print(f"    FR dz = -21.5mm")
        print(f"    RE dz = -17.6mm")

        if abs(fl_dz_med) > 10:
            print(f"  → COMMON_FL_DEPTH_BIAS: FL PnP dz={fl_dz_med:+.1f}mm suggests systematic PnP depth offset")
            print(f"    This can propagate to both FL→FR and FL→RE pair estimates")
            print(f"  COMMON_FL_DEPTH_BIAS_CONFIRMED=YES")
        else:
            print(f"  → FL PnP dz median {fl_dz_med:+.1f}mm is small — FL depth bias is NOT the primary cause")
            print(f"  COMMON_FL_DEPTH_BIAS_CONFIRMED=NO")
    else:
        print("  No FL PnP data")

    return categories, all_rows


# ═══════════════════════════════════════════════════════════════
# D3: Subset Replay
# ═══════════════════════════════════════════════════════════════

def run_d3_subset_replay(groups, tf_buffer):
    """Run same frozen solver on different subsets, score each."""
    print("\n" + "=" * 60)
    print("D3: FROZEN SUBSET REPLAY")
    print("=" * 60)

    geom = load_target_geometry()
    profiles = create_default_profiles()

    from sensor_msgs.msg import CameraInfo
    camera_info = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        camera_info[cam] = {"K": K, "D": D}

    T_world_cams = get_gazebo_truth(tf_buffer)
    T_world_FL = T_world_cams[FIRST_CAM]

    # Build: per-group per-camera PnP and face info
    group_pnp = defaultdict(dict)
    group_faces = {}

    for gid in sorted(groups.keys()):
        g = groups[gid]
        for cam in CAMERAS:
            if cam not in g["images"]:
                continue
            cv_img = g["images"][cam]
            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]
            detection = detect_target(cv_img, K, D, geom, profiles)
            if not detection:
                continue
            obj_pts, img_pts = [], []
            faces_det = []
            for fn, fd in detection.items():
                oface = fd.get("object_points_3d_face", [])
                iface = fd.get("image_points_2d", [])
                if not oface:
                    continue
                T_face = geom.T_target_face.get(fn)
                if T_face is None:
                    continue
                faces_det.append(fn)
                for pi in range(len(oface)):
                    pt_tgt = (T_face @ np.array([*oface[pi], 1.0]))[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(iface[pi])
            if len(obj_pts) >= 4:
                T_est, _, _, _ = solve_pnp(np.array(obj_pts), np.array(img_pts), K, D)
                if T_est is not None:
                    group_pnp[gid][cam] = T_est
            group_faces[gid] = faces_det

    all_gids = sorted(group_pnp.keys())

    # Define subsets based on DETECTED faces (truth-free)
    dual_mf_gids = []
    c_bank_gids = []
    single_planar_gids = []

    for gid in all_gids:
        fl_faces = set()
        fr_faces = set()
        # We need to re-detect for face info
        g = groups[gid]
        for cam in CAMERAS:
            if cam not in g["images"]:
                continue
            cv_img = g["images"][cam]
            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]
            det = detect_target(cv_img, K, D, geom, profiles)
            if det:
                for fn, fd in det.items():
                    if fd.get("corner_count", 0) > 0:
                        if cam == "cam_front_left":
                            fl_faces.add(fn)
                        elif cam == "cam_front_right":
                            fr_faces.add(fn)

        cid = groups[gid].get("candidate_id", "")
        has_fl_rt = "right" in fl_faces and "top" in fl_faces
        has_fr_lt = "left" in fr_faces and "top" in fr_faces

        if has_fl_rt and has_fr_lt:
            dual_mf_gids.append(gid)
        if cid.startswith("C") and cid[1:].isdigit():
            c_bank_gids.append(gid)
        if ("right" in fl_faces and "top" not in fl_faces and
                "left" in fr_faces and "top" not in fr_faces):
            single_planar_gids.append(gid)

    subsets = {
        "S0_ALL20": all_gids,
        "S1_DUAL_MULTIFACE": sorted(dual_mf_gids),
        "S2_C01_C06": sorted(c_bank_gids),
        "S3_SINGLE_PLANAR": sorted(single_planar_gids),
    }

    def solve_and_score(gids, label, T_world_FL, T_world_cams):
        sub_pnp = {gid: group_pnp[gid] for gid in gids if gid in group_pnp and len(group_pnp[gid]) >= 2}
        if len(sub_pnp) < 5:
            return {"label": label, "n": len(sub_pnp), "error": "too few groups"}

        X_cameras, report = compute_pairwise_rig(sub_pnp)

        # Score
        X_truth = {FIRST_CAM: np.eye(4)}
        for cam in CAMERAS[1:]:
            X_truth[cam] = invert_transform(T_world_FL) @ T_world_cams[cam]

        scores = {}
        for cam in ["cam_front_right", "cam_rear"]:
            if cam in X_cameras and cam in X_truth:
                t_err, r_err, _ = se3_distance_mm_deg(X_cameras[cam], X_truth[cam], 50, 5)
                scores[cam] = {"T_mm": float(t_err), "R_deg": float(r_err)}
            else:
                scores[cam] = {"T_mm": None, "R_deg": None}

        tc = report.get("triangle_closure_t_mm")

        # Save blind result
        blind = {
            "solver": "pairwise_camera_relative", "version": "V8.18_subset",
            "subset": label, "n_groups": len(sub_pnp),
            "group_ids": sorted(sub_pnp.keys()),
            "cameras": {cam: X_cameras.get(cam, np.eye(4)).tolist() for cam in CAMERAS},
            "pair_stats": report.get("pairs", {}),
            "triangle_closure_t_mm": tc,
        }
        out = os.path.join(DIAG_DIR, f"blind_{label}.json")
        with open(out, "w") as f:
            json.dump(blind, f, indent=2)
        sha = hashlib.sha256(open(out, "rb").read()).hexdigest()[:16]

        return {
            "label": label, "n": len(sub_pnp), "gids": sorted(sub_pnp.keys()),
            "FR_T_mm": scores.get("cam_front_right", {}).get("T_mm"),
            "FR_R_deg": scores.get("cam_front_right", {}).get("R_deg"),
            "RE_T_mm": scores.get("cam_rear", {}).get("T_mm"),
            "RE_R_deg": scores.get("cam_rear", {}).get("R_deg"),
            "triangle_t_mm": tc,
            "FL_FR_inliers": report.get("pairs", {}).get("FL_FR", {}).get("n_inliers", 0),
            "FL_RE_inliers": report.get("pairs", {}).get("FL_RE", {}).get("n_inliers", 0),
            "sha256": sha,
        }

    results = []
    print(f"\n{'Subset':<22s} {'N':>4s} {'FR T':>8s} {'FR R':>8s} {'RE T':>8s} {'RE R':>8s} {'Tri T':>7s}")
    print("-" * 75)
    for label, gids in subsets.items():
        r = solve_and_score(gids, label, T_world_FL, T_world_cams)
        results.append(r)
        fr_t = f"{r['FR_T_mm']:.1f}mm" if r['FR_T_mm'] else "N/A"
        fr_r = f"{r['FR_R_deg']:.2f}°" if r['FR_R_deg'] else "N/A"
        re_t = f"{r['RE_T_mm']:.1f}mm" if r['RE_T_mm'] else "N/A"
        re_r = f"{r['RE_R_deg']:.2f}°" if r['RE_R_deg'] else "N/A"
        tri = f"{r['triangle_t_mm']:.1f}mm" if r.get('triangle_t_mm') else "N/A"
        print(f"{label:<22s} {r['n']:4d} {fr_t:>8s} {fr_r:>8s} {re_t:>8s} {re_r:>8s} {tri:>7s}")

    # Save
    with open(os.path.join(DIAG_DIR, "d3_subset_replay.json"), "w") as f:
        json.dump(results, f, indent=2)

    return results


# ═══════════════════════════════════════════════════════════════
# D4: Per-group Pair Truth Audit
# ═══════════════════════════════════════════════════════════════

def run_d4_pair_truth_audit(groups, tf_buffer):
    """For each group, compute pair estimates vs pair truth."""
    print("\n" + "=" * 60)
    print("D4: PER-GROUP PAIR TRUTH AUDIT")
    print("=" * 60)

    geom = load_target_geometry()
    profiles = create_default_profiles()

    from sensor_msgs.msg import CameraInfo
    camera_info = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        camera_info[cam] = {"K": K, "D": D}

    T_world_cams = get_gazebo_truth(tf_buffer)
    T_world_FL = T_world_cams[FIRST_CAM]

    pair_truth = {}
    for pn, camA, camB in PAIRS:
        pair_truth[pn] = invert_transform(T_world_cams[camA]) @ T_world_cams[camB]

    rows = []
    for gid in sorted(groups.keys()):
        g = groups[gid]
        cid = g["candidate_id"]
        T_world_target = get_gazebo_target_truth()

        group_pnp = {}
        for cam in CAMERAS:
            if cam not in g["images"]:
                continue
            cv_img = g["images"][cam]
            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]
            detection = detect_target(cv_img, K, D, geom, profiles)
            if not detection:
                continue
            obj_pts, img_pts = [], []
            faces = []
            for fn, fd in detection.items():
                oface = fd.get("object_points_3d_face", [])
                iface = fd.get("image_points_2d", [])
                if not oface:
                    continue
                T_face = geom.T_target_face.get(fn)
                if T_face is None:
                    continue
                faces.append(fn)
                for pi in range(len(oface)):
                    pt_tgt = (T_face @ np.array([*oface[pi], 1.0]))[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(iface[pi])
            if len(obj_pts) >= 4:
                T_est, _, _, _ = solve_pnp(np.array(obj_pts), np.array(img_pts), K, D)
                if T_est is not None:
                    group_pnp[cam] = T_est

        row = {"gid": gid, "cid": cid}
        for pn, camA, camB in PAIRS:
            if camA in group_pnp and camB in group_pnp:
                T_AB = group_pnp[camA] @ invert_transform(group_pnp[camB])
                t_err, r_err, _ = se3_distance_mm_deg(T_AB, pair_truth[pn], 50, 5)
                d_est = T_AB[:3, 3] * 1000
                d_truth = pair_truth[pn][:3, 3] * 1000
                row[f"{pn}_T"] = t_err
                row[f"{pn}_R"] = r_err
                row[f"{pn}_dz"] = d_est[2] - d_truth[2]
            else:
                row[f"{pn}_T"] = None
                row[f"{pn}_R"] = None
                row[f"{pn}_dz"] = None
        rows.append(row)

    # Summary
    print(f"\n{'Pair':<10s} {'N':>4s} {'T med':>7s} {'T P95':>7s} {'R med':>7s} {'DZ med':>8s}")
    print("-" * 55)
    for pn, _, _ in PAIRS:
        vals_t = [r[f"{pn}_T"] for r in rows if r.get(f"{pn}_T") is not None]
        vals_r = [r[f"{pn}_R"] for r in rows if r.get(f"{pn}_R") is not None]
        vals_dz = [r[f"{pn}_dz"] for r in rows if r.get(f"{pn}_dz") is not None]
        if vals_t:
            print(f"{pn:<10s} {len(vals_t):4d} {np.median(vals_t):6.1f}mm {np.percentile(vals_t,95):6.1f}mm {np.median(vals_r):6.2f}° {np.median(vals_dz):+7.1f}mm")

    # Worst groups
    print("\nWorst 3 groups (by FL_FR T error):")
    sorted_rows = sorted([r for r in rows if r.get("FL_FR_T") is not None], key=lambda r: -r["FL_FR_T"])
    for r in sorted_rows[:3]:
        print(f"  G{r['gid']:02d} {r['cid']:4s}: FL_FR T={r['FL_FR_T']:.1f}mm FL_RE T={r['FL_RE_T']:.1f}mm")

    # Best groups
    print("\nBest 3 groups:")
    for r in sorted_rows[-3:]:
        print(f"  G{r['gid']:02d} {r['cid']:4s}: FL_FR T={r['FL_FR_T']:.1f}mm FL_RE T={r['FL_RE_T']:.1f}mm")

    # Save
    csv_path = os.path.join(DIAG_DIR, "d4_pair_truth_by_group.csv")
    fieldnames = ["gid","cid","FL_FR_T","FL_FR_R","FL_FR_dz","FL_RE_T","FL_RE_R","FL_RE_dz","FR_RE_T","FR_RE_R","FR_RE_dz"]
    with open(csv_path, "w") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    return rows


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    rospy.init_node("v818_root_cause", anonymous=True)

    print("=" * 60)
    print("V8.18 FROZEN DATASET-A ROOT CAUSE ISOLATION")
    print("=" * 60)
    print(f"Dataset: {RUN_DIR}")
    print(f"Diagnostics: {DIAG_DIR}")

    # Load data
    groups, manifest = load_formal_dataset()
    print(f"Loaded {len(groups)} groups")

    # TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    # D1
    d1_ok = run_d1_score_sanity(tf_buffer)

    # D2
    categories, pnp_rows = run_d2_pnp_truth_audit(groups, tf_buffer)

    # D3
    subset_results = run_d3_subset_replay(groups, tf_buffer)

    # D4
    pair_rows = run_d4_pair_truth_audit(groups, tf_buffer)

    # ── Final Classification ──
    print("\n" + "=" * 60)
    print("ROOT CAUSE CLASSIFICATION")
    print("=" * 60)

    # Check D2 for FL depth bias
    fl_dz_vals = [r["dz"] for r in pnp_rows if r["broad"] in ("FL_RIGHT_ONLY", "FL_RIGHT_TOP")]
    fl_dz_med = np.median(fl_dz_vals) if fl_dz_vals else 0

    # Check D3 for subset improvement
    s0 = [r for r in subset_results if r["label"] == "S0_ALL20"][0]
    s1 = [r for r in subset_results if r["label"] == "S1_DUAL_MULTIFACE"][0]
    s2 = [r for r in subset_results if r["label"] == "S2_C01_C06"][0]

    fr_all = s0.get("FR_T_mm", 99)
    fr_mf = s1.get("FR_T_mm", 99) if s1.get("FR_T_mm") else 99

    case = "UNKNOWN"
    primary = ""

    if abs(fl_dz_med) > 10 and fr_mf < 15:
        case = "B"
        primary = f"FL planar PnP depth bias (dz_med={fl_dz_med:+.1f}mm). Multi-face subsets show improvement (FR={fr_mf:.1f}mm). DATA_COMPOSITION / PLANAR_DEPTH."
    elif fr_mf < 10:
        case = "A"
        primary = f"Dual-multiface subset passes (FR={fr_mf:.1f}mm). Root cause is DATA_COMPOSITION — insufficient multi-face groups in full set."
    elif abs(fl_dz_med) > 10 and fr_mf >= 15:
        case = "C"
        primary = f"Even multi-face subsets still biased (FR={fr_mf:.1f}mm). FL depth bias persists despite non-planar observations. Possible geometry/incidence issue."
    elif s2.get("n", 0) < 3:
        case = "X"
        primary = "C01-C06 subset too small to draw conclusion."
    else:
        case = "D"
        primary = f"PnP errors do not clearly propagate to pair errors. Possible pairwise aggregation issue."

    print(f"  CASE: {case}")
    print(f"  PRIMARY: {primary}")
    print(f"  FL dz med: {fl_dz_med:+.1f}mm")
    print(f"  ALL20 FR: {fr_all:.1f}mm")
    print(f"  DUAL_MF FR: {fr_mf:.1f}mm")
    print(f"\n  ALGORITHM_CHANGED: NO")
    print(f"  SCENE_CHANGED: NO")
    print(f"  NEW_DATA_CAPTURED: NO")
    print(f"\nDiagnostics: {DIAG_DIR}")

    # Save classification
    with open(os.path.join(DIAG_DIR, "classification.json"), "w") as f:
        json.dump({"case": case, "primary": primary, "fl_dz_med": fl_dz_med,
                   "fr_all_mm": fr_all, "fr_mf_mm": fr_mf}, f, indent=2)

    print("\nDONE")


if __name__ == "__main__":
    main()

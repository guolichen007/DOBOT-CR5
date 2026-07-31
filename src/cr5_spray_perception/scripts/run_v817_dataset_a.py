#!/usr/bin/env python3
"""
V8.17 Dataset A Blind Acceptance Runner.

End-to-end pipeline for final Gazebo cell calibration acceptance:
  --mode preview   : move target to each candidate, capture sync frames, run detector
  --mode select    : select 20-24 best groups from preview results
  --mode capture   : formal fresh capture of selected poses
  --mode solve     : blind pairwise solve (no truth)
  --mode score     : truth score (Gazebo TF)
  --mode full      : preview + select + capture + solve + score

All target poses are BASE_TARGET + delta offsets.
No old-world coordinates (0.68, 0.60) used.

Usage:
  python3 run_v817_dataset_a.py --mode full --run-id sim_v815_final_cell_A
"""
import sys, os, math, time, json, hashlib, argparse
import numpy as np
import cv2
import rospy
import rospkg
import yaml

# Path setup
WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_sim", "src"))

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState, GetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from cr5_spray_sim.scene_config import get_base_target_pose
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.pairwise_solver import compute_pairwise_rig
from cr5_spray_perception.calibration.pnp_solver import solve_pnp
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]

DATA_ROOT = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                         "calibration")

# ═══════════════════════════════════════════════════════════════
# Candidate Bank: BASE_TARGET + delta offsets
# Migrated from V8.14 OLD scene base [0.68, 0, 0.60]
# ═══════════════════════════════════════════════════════════════

CANDIDATE_DELTAS = [
    # ── Bank A: FL right+top (roll+, yaw-) ──
    {"id": "A01", "dx": -0.04, "dy": -0.02, "dz": 0.00, "rpy": [12, 0, -10], "desc": "FL right+top roll12"},
    {"id": "A02", "dx": -0.04, "dy": -0.02, "dz": 0.03, "rpy": [15, 6, -12], "desc": "FL right+top roll15 pitch6"},
    {"id": "A03", "dx": 0.00, "dy": -0.04, "dz": -0.04, "rpy": [14, -3, -12], "desc": "FL right+top combo"},
    {"id": "A04", "dx": -0.08, "dy": -0.02, "dz": 0.00, "rpy": [12, 0, -10], "desc": "FL near right+top"},
    {"id": "A05", "dx": -0.08, "dy": -0.02, "dz": 0.02, "rpy": [10, 8, -8], "desc": "FL near right+top pitch"},
    {"id": "A06", "dx": -0.04, "dy": -0.02, "dz": 0.00, "rpy": [14, -3, 10], "desc": "FL right+top yaw+10"},

    # ── Bank B: FR left+top (roll-, yaw+) ──
    {"id": "B01", "dx": -0.04, "dy": 0.02, "dz": 0.00, "rpy": [-12, 0, 10], "desc": "FR left+top roll-12"},
    {"id": "B02", "dx": -0.04, "dy": 0.02, "dz": 0.03, "rpy": [-15, 6, 12], "desc": "FR left+top roll-15 pitch6"},
    {"id": "B03", "dx": -0.08, "dy": 0.02, "dz": 0.00, "rpy": [-12, 0, 10], "desc": "FR near left+top"},
    {"id": "B04", "dx": -0.08, "dy": 0.02, "dz": 0.02, "rpy": [-10, 8, 8], "desc": "FR near left+top pitch"},

    # ── Bank C: Depth variation ──
    {"id": "C01", "dx": -0.10, "dy": 0.00, "dz": 0.00, "rpy": [0, 5, 0], "desc": "Far -100mm"},
    {"id": "C02", "dx": -0.06, "dy": 0.00, "dz": 0.00, "rpy": [0, 0, 0], "desc": "Near -60mm"},
    {"id": "C03", "dx": 0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, -5, 0], "desc": "Close +40mm"},
    {"id": "C04", "dx": 0.08, "dy": 0.00, "dz": 0.00, "rpy": [0, 0, 0], "desc": "Closest +80mm"},
    {"id": "C05", "dx": -0.04, "dy": 0.00, "dz": 0.06, "rpy": [0, -5, 0], "desc": "High +60mm"},
    {"id": "C06", "dx": -0.04, "dy": 0.00, "dz": -0.12, "rpy": [0, 8, 0], "desc": "Low -120mm"},

    # ── Bank D: Roll / yaw / pitch / xyz mixed ──
    {"id": "D01", "dx": -0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, 0, 15], "desc": "Yaw+15"},
    {"id": "D02", "dx": -0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, 0, -15], "desc": "Yaw-15"},
    {"id": "D03", "dx": -0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, 10, 0], "desc": "Pitch+10"},
    {"id": "D04", "dx": -0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, -10, 0], "desc": "Pitch-10"},
    {"id": "D05", "dx": -0.04, "dy": -0.03, "dz": -0.10, "rpy": [10, 15, -10], "desc": "Roll+pitch+yaw mixed"},
    {"id": "D06", "dx": -0.04, "dy": 0.03, "dz": 0.06, "rpy": [-10, -15, 10], "desc": "Mirror mixed"},
    {"id": "D07", "dx": -0.04, "dy": 0.00, "dz": 0.00, "rpy": [0, 18, 0], "desc": "Pitch+18"},
    {"id": "D08", "dx": -0.04, "dy": 0.05, "dz": -0.05, "rpy": [0, 0, 10], "desc": "Left combo"},
    {"id": "D09", "dx": -0.04, "dy": -0.05, "dz": -0.05, "rpy": [0, 0, -10], "desc": "Right combo"},
    {"id": "D10", "dx": 0.00, "dy": 0.05, "dz": 0.00, "rpy": [0, 0, 0], "desc": "Base+Y"},

    # Extra: more roll diversity
    {"id": "D11", "dx": -0.04, "dy": -0.03, "dz": 0.02, "rpy": [15, 0, -5], "desc": "Roll+15 right"},
    {"id": "D12", "dx": -0.04, "dy": 0.03, "dz": 0.02, "rpy": [-15, 0, 5], "desc": "Roll-15 left"},
]


def get_world_xyz(delta):
    """Convert delta to world coordinates using BASE_TARGET."""
    base = get_base_target_pose()
    return [base[0] + delta["dx"], base[1] + delta["dy"], base[2] + delta["dz"]]


# ═══════════════════════════════════════════════════════════════
# Gazebo interaction
# ═══════════════════════════════════════════════════════════════

def set_target_pose(xyz, rpy_deg):
    """Move calibration target in Gazebo."""
    q = quaternion_from_euler(*[math.radians(r) for r in rpy_deg])
    rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    req = SetModelStateRequest()
    req.model_state.model_name = "simple_hanging_workpiece"
    req.model_state.pose = Pose(position=Point(*xyz), orientation=Quaternion(*q))
    req.model_state.reference_frame = "world"
    return svc(req).success


def get_target_xyz():
    """Read current target position from Gazebo."""
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    req = GetModelStateRequest()
    req.model_name = "simple_hanging_workpiece"
    req.relative_entity_name = "world"
    resp = svc(req)
    if resp.success:
        p = resp.pose.position
        return (p.x, p.y, p.z)
    return None


def wait_settle(target_xyz=None, tol_mm=2.0, max_wait=3.0):
    """Wait for target to stop moving."""
    if target_xyz is None:
        target_xyz = get_target_xyz()
        if target_xyz is None:
            rospy.sleep(1.5)
            return True

    start = time.time()
    while time.time() - start < max_wait:
        rospy.sleep(0.3)
        cur = get_target_xyz()
        if cur is None:
            continue
        dist = math.sqrt(sum((cur[i] - target_xyz[i])**2 for i in range(3))) * 1000
        if dist < tol_mm:
            return True
    return False


# ═══════════════════════════════════════════════════════════════
# Sync capture (no capture_manager service — direct topic sync)
# ═══════════════════════════════════════════════════════════════

def capture_sync_frames():
    """Capture one synchronized frame from all 3 cameras.

    Uses approximate timestamp matching.
    Returns {cam_name: cv_img} or None if timeout.
    """
    bridge = CvBridge()
    results = {}
    stamps = {}

    for cam in CAMERAS:
        try:
            img_msg = rospy.wait_for_message(
                f"/{cam}/camera/color/image_raw", Image, timeout=3.0)
            results[cam] = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            stamps[cam] = img_msg.header.stamp
        except rospy.ROSException:
            rospy.logwarn("capture timeout for %s", cam)
            return None

    # Check sync skew
    if stamps:
        t_secs = [s.to_sec() for s in stamps.values()]
        skew_ms = (max(t_secs) - min(t_secs)) * 1000
        if skew_ms > 100:
            rospy.logwarn("Sync skew %.1f ms > 100ms", skew_ms)

    return results


def save_group(group_dir, frames):
    """Save captured frames to group directory."""
    for cam, cv_img in frames.items():
        cam_dir = os.path.join(group_dir, cam)
        os.makedirs(cam_dir, exist_ok=True)
        cv2.imwrite(os.path.join(cam_dir, "color.png"), cv_img)


# ═══════════════════════════════════════════════════════════════
# Preview
# ═══════════════════════════════════════════════════════════════

def run_preview(candidates, preview_dir, geom, profiles, camera_info):
    """Preview each candidate: move → capture → detect → score."""
    os.makedirs(preview_dir, exist_ok=True)
    results = []
    K_arrays = {cam: camera_info[cam]["K"] for cam in CAMERAS}
    D_arrays = {cam: camera_info[cam]["D"] for cam in CAMERAS}

    for i, c in enumerate(candidates):
        wxyz = get_world_xyz(c)
        rospy.loginfo("Preview %d/%d: %s → (%.3f,%.3f,%.3f) rpy=%s",
                      i+1, len(candidates), c["id"], *wxyz, c["rpy"])

        set_target_pose(wxyz, c["rpy"])
        wait_settle(wxyz)

        # Capture 3 sync snapshots, use best
        best_det = None
        best_score = -1
        for snap in range(3):
            rospy.sleep(0.1)
            frames = capture_sync_frames()
            if frames is None:
                continue

            # Save preview frames
            gdir = os.path.join(preview_dir, f"{c['id']}_snap{snap}")
            save_group(gdir, frames)

            # Run detector
            cam_det = {}
            total_corners = 0
            for cam in CAMERAS:
                if cam not in frames:
                    continue
                K = K_arrays[cam]
                D = D_arrays[cam]
                detection = detect_target(frames[cam], K, D, geom, profiles)
                if detection:
                    face_corners = {}
                    for fn, fd in detection.items():
                        nc = fd.get("corner_count", 0)
                        if nc > 0:
                            face_corners[fn] = nc
                            total_corners += nc
                    if face_corners:
                        cam_det[cam] = face_corners

            # Quality score: multi-face + corners
            has_right_top = ("right" in cam_det.get("cam_front_left", {}) and
                             "top" in cam_det.get("cam_front_left", {}))
            has_left_top = ("left" in cam_det.get("cam_front_right", {}) and
                            "top" in cam_det.get("cam_front_right", {}))
            n_cams = len(cam_det)

            snap_score = total_corners + (20 if has_right_top else 0) + (20 if has_left_top else 0) + n_cams * 10
            if snap_score > best_score:
                best_score = snap_score
                best_det = {
                    "snap": snap, "score": snap_score, "cam_det": cam_det,
                    "total_corners": total_corners, "n_cams": n_cams,
                    "has_right_top": has_right_top, "has_left_top": has_left_top,
                }

        if best_det:
            best_det["id"] = c["id"]
            best_det["delta"] = {"dx": c["dx"], "dy": c["dy"], "dz": c["dz"], "rpy": c["rpy"]}
            best_det["world_xyz"] = wxyz
            best_det["desc"] = c["desc"]
            results.append(best_det)
            rospy.loginfo("  %s: %d corners, %d cams, RT=%s LT=%s",
                          c["id"], best_det["total_corners"], best_det["n_cams"],
                          "Y" if best_det["has_right_top"] else "N",
                          "Y" if best_det["has_left_top"] else "N")
        else:
            rospy.logwarn("  %s: NO DETECTION", c["id"])

    # Save preview report
    with open(os.path.join(preview_dir, "preview_results.json"), "w") as f:
        json.dump(results, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

    return results


# ═══════════════════════════════════════════════════════════════
# Selection
# ═══════════════════════════════════════════════════════════════

def select_poses(preview_results, n_target=22):
    """Select best poses from preview results using observation diversity criteria."""
    # Sort by quality score
    scored = sorted(preview_results, key=lambda r: r.get("score", 0), reverse=True)

    # Coverage requirements
    fl_rt_count = 0  # FL right+top
    fr_lt_count = 0  # FR left+top
    depth_count = 0  # significant depth variation
    roll_count = 0   # |roll| >= 10°

    selected = []
    used_deltas = set()

    for r in scored:
        if len(selected) >= n_target:
            break

        # Avoid near-duplicate deltas
        key = (round(r["delta"]["dx"], 2), round(r["delta"]["dy"], 2), round(r["delta"]["dz"], 2),
               round(r["delta"]["rpy"][0], 0), round(r["delta"]["rpy"][1], 0), round(r["delta"]["rpy"][2], 0))
        if key in used_deltas:
            continue

        # Check diversity: at least 8mm translation or 8° rotation from all selected
        too_close = False
        wxyz_cur = np.array(r["world_xyz"])
        for s in selected:
            wxyz_s = np.array(s["world_xyz"])
            if np.linalg.norm(wxyz_cur - wxyz_s) < 0.008:
                rpy_diff = sum(abs(r["delta"]["rpy"][i] - s["delta"]["rpy"][i]) for i in range(3))
                if rpy_diff < 8:
                    too_close = True
                    break
        if too_close:
            continue

        selected.append(r)
        used_deltas.add(key)

        if r.get("has_right_top"): fl_rt_count += 1
        if r.get("has_left_top"): fr_lt_count += 1
        if abs(r["delta"]["dx"]) >= 0.06: depth_count += 1
        if abs(r["delta"]["rpy"][0]) >= 10: roll_count += 1

    # If we don't have enough coverage, add more from remaining
    if len(selected) < n_target:
        for r in scored:
            if len(selected) >= n_target:
                break
            key = (round(r["delta"]["dx"], 2), round(r["delta"]["dy"], 2), round(r["delta"]["dz"], 2),
                   round(r["delta"]["rpy"][0], 0), round(r["delta"]["rpy"][1], 0), round(r["delta"]["rpy"][2], 0))
            if key in used_deltas:
                continue
            selected.append(r)
            used_deltas.add(key)

    rospy.loginfo("Selected %d poses:", len(selected))
    rospy.loginfo("  FL right+top: %d, FR left+top: %d, depth: %d, roll>=10: %d",
                  fl_rt_count, fr_lt_count, depth_count, roll_count)
    for i, r in enumerate(selected):
        rospy.loginfo("  %2d. %s: xyz=(%.3f,%.3f,%.3f) rpy=%s score=%d %s",
                      i+1, r["id"], r["world_xyz"][0], r["world_xyz"][1], r["world_xyz"][2],
                      r["delta"]["rpy"], r["score"], r["desc"])

    return selected


# ═══════════════════════════════════════════════════════════════
# Formal Capture
# ═══════════════════════════════════════════════════════════════

def run_formal_capture(selected, run_dir, geom, profiles, camera_info):
    """Fresh capture of selected poses. Each gets 3 sync snapshots, best used."""
    groups_dir = os.path.join(run_dir, "groups")
    os.makedirs(groups_dir, exist_ok=True)

    manifest = {
        "run_id": os.path.basename(run_dir),
        "n_groups": len(selected),
        "groups": [],
        "scene_yaml_sha": "...",
        "target_yaml_sha": "...",
    }

    # Hash scene files
    rp = rospkg.RosPack()
    for fname, key in [("simulation_scene.yaml", "scene_yaml_sha"),
                        ("calibration/calibration_target.yaml", "target_yaml_sha")]:
        fpath = os.path.join(rp.get_path("cr5_spray_sim"), "config", fname)
        if os.path.isfile(fpath):
            manifest[key] = hashlib.sha256(open(fpath, "rb").read()).hexdigest()[:16]

    for i, c in enumerate(selected):
        gid = i
        wxyz = c["world_xyz"]
        rospy.loginfo("Capture %d/%d: %s", i+1, len(selected), c["id"])

        set_target_pose(wxyz, c["delta"]["rpy"])
        wait_settle(wxyz)

        # 3 sync snapshots
        best_snap = None
        best_corners = -1
        for snap in range(3):
            rospy.sleep(0.1)
            gdir = os.path.join(groups_dir, f"group_{gid:04d}_snap{snap}")
            frames = capture_sync_frames()
            if frames is None:
                continue
            save_group(gdir, frames)

            # Quick quality check
            total_c = 0
            for cam in CAMERAS:
                if cam not in frames:
                    continue
                K = camera_info[cam]["K"]
                D = camera_info[cam]["D"]
                det = detect_target(frames[cam], K, D, geom, profiles)
                if det:
                    for fd in det.values():
                        total_c += fd.get("corner_count", 0)
            if total_c > best_corners:
                best_corners = total_c
                best_snap = snap

        # Copy best snap as official group
        if best_snap is not None:
            src = os.path.join(groups_dir, f"group_{gid:04d}_snap{best_snap}")
            dst = os.path.join(groups_dir, f"group_{gid:04d}")
            if os.path.exists(dst):
                import shutil
                shutil.rmtree(dst)
            os.rename(src, dst)

            # Remove other snaps
            for snap in range(3):
                rm_dir = os.path.join(groups_dir, f"group_{gid:04d}_snap{snap}")
                if os.path.exists(rm_dir) and rm_dir != dst:
                    import shutil
                    shutil.rmtree(rm_dir)

            rospy.loginfo("  → group_%04d: %d corners (snap %d)", gid, best_corners, best_snap)
        else:
            rospy.logwarn("  → group_%04d: NO CAPTURE", gid)

        manifest["groups"].append({
            "group_id": gid,
            "candidate_id": c["id"],
            "world_xyz": c["world_xyz"],
            "rpy": c["delta"]["rpy"],
            "desc": c["desc"],
            "best_snap": best_snap,
            "best_corners": best_corners,
        })

    # Save manifest
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    return manifest


# ═══════════════════════════════════════════════════════════════
# Solve
# ═══════════════════════════════════════════════════════════════

def run_blind_solve(run_dir, camera_info):
    """Blind pairwise solve — NO Gazebo truth access."""
    groups_dir = os.path.join(run_dir, "groups")
    if not os.path.isdir(groups_dir):
        rospy.logerr("Groups directory not found: %s", groups_dir)
        return None

    geom = load_target_geometry()
    profiles = create_default_profiles()

    # Build per_group_pnp from images
    per_group_pnp = {}
    all_group_ids = sorted([
        int(d.replace("group_", "")) for d in os.listdir(groups_dir)
        if d.startswith("group_") and os.path.isdir(os.path.join(groups_dir, d))
    ])

    rospy.loginfo("Blind solve: %d groups", len(all_group_ids))

    for gid in all_group_ids:
        group_dir = os.path.join(groups_dir, f"group_{gid:04d}")
        group_pnp = {}
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path):
                continue
            cv_img = cv2.imread(img_path)
            if cv_img is None:
                continue
            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]
            detection = detect_target(cv_img, K, D, geom, profiles)
            if not detection:
                continue
            # Collect object points and image points across all faces
            obj_pts = []
            img_pts = []
            for face_name, fd in detection.items():
                obj_face = fd.get("object_points_3d_face", [])
                img_face = fd.get("image_points_2d", [])
                if not obj_face:
                    continue
                T_face = geom.T_target_face.get(face_name)
                if T_face is None:
                    continue
                for pi in range(len(obj_face)):
                    pt = np.array([*obj_face[pi], 1.0])
                    pt_tgt = (T_face @ pt)[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(img_face[pi])
            if len(obj_pts) >= 4:
                T_cam_target, _, _, success = solve_pnp(obj_pts, img_pts, K, D)
                if success and T_cam_target is not None:
                    group_pnp[cam] = T_cam_target
        if len(group_pnp) >= 2:
            per_group_pnp[gid] = group_pnp

    rospy.loginfo("Groups with 2+ camera PnP: %d", len(per_group_pnp))

    if len(per_group_pnp) < 5:
        rospy.logerr("Insufficient groups for pairwise solve: %d", len(per_group_pnp))
        return None

    # Blind solve
    X_cameras, report = compute_pairwise_rig(per_group_pnp)

    # Save blind result
    blind_result = {
        "solver": "pairwise_camera_relative",
        "version": "V8.17",
        "run_id": os.path.basename(run_dir),
        "n_groups": len(per_group_pnp),
        "group_ids": sorted(per_group_pnp.keys()),
        "cameras": {cam: X_cameras.get(cam, np.eye(4)).tolist() for cam in CAMERAS},
        "pair_stats": report.get("pairs", {}),
        "triangle_closure_t_mm": report.get("triangle_closure_t_mm"),
        "triangle_closure_r_deg": report.get("triangle_closure_r_deg"),
    }

    output_path = os.path.join(run_dir, "camera_extrinsics_blind.json")
    with open(output_path, "w") as f:
        json.dump(blind_result, f, indent=2)

    # SHA256
    blind_sha = hashlib.sha256(open(output_path, "rb").read()).hexdigest()
    rospy.loginfo("Blind result: %s", output_path)
    rospy.loginfo("SHA256: %s", blind_sha)

    # Internal report (NO truth)
    print("\n" + "="*60)
    print("BLIND PAIRWISE SOLVE (NO TRUTH)")
    print("="*60)
    for pn, ps in report.get("pairs", {}).items():
        print(f"  {pn}: n={ps['n_total']}, inliers={ps['n_inliers']}")
        print(f"    T: med={ps['t_median_mm']:.1f}mm MAD={ps['t_mad_mm']:.1f}mm P95={ps['t_p95_mm']:.1f}mm")
        print(f"    R: med={ps['r_median_deg']:.2f}° MAD={ps['r_mad_deg']:.2f}°")
    tc = report.get("triangle_closure_t_mm")
    if tc is not None:
        print(f"  Triangle closure: T={tc:.1f}mm R={report['triangle_closure_r_deg']:.2f}°")
    print(f"  BLIND_SHA256: {blind_sha}")
    print("="*60)

    return blind_result


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="V8.17 Dataset A Blind Acceptance")
    parser.add_argument("--mode", default="full",
                        choices=["preview", "select", "capture", "solve", "score", "full"],
                        help="Pipeline mode")
    parser.add_argument("--run-id", default="sim_v815_final_cell_A",
                        help="Dataset run ID")
    parser.add_argument("--num-groups", type=int, default=22,
                        help="Number of groups to select")
    args = parser.parse_args()

    rospy.init_node("v817_dataset_a", anonymous=True)

    # Directories
    raw_root = os.path.join(DATA_ROOT, "raw")
    runs_root = os.path.join(DATA_ROOT, "runs")
    preview_dir = os.path.join(raw_root, f"{args.run_id}_preview")
    run_dir = os.path.join(runs_root, args.run_id)

    # Load configs
    base = get_base_target_pose()
    rospy.loginfo("BASE_TARGET: (%.3f, %.3f, %.3f)", *base)

    geom = load_target_geometry()
    profiles = create_default_profiles()

    # Camera info
    camera_info = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        camera_info[cam] = {"K": K, "D": D, "width": info.width, "height": info.height}
        rospy.loginfo("%s: %dx%d K=[%.1f,%.1f]", cam, info.width, info.height, K[0, 0], K[1, 1])

    # Build candidate list
    candidates = list(CANDIDATE_DELTAS)
    rospy.loginfo("Candidates: %d deltas", len(candidates))

    selected = None

    # ── Preview ──
    if args.mode in ("preview", "full"):
        rospy.loginfo("=== PREVIEW ===")
        results = run_preview(candidates, preview_dir, geom, profiles, camera_info)
        rospy.loginfo("Preview done: %d candidates with detection", len(results))

    # ── Select ──
    if args.mode in ("select", "full"):
        rospy.loginfo("=== SELECT ===")
        preview_file = os.path.join(preview_dir, "preview_results.json")
        if not os.path.isfile(preview_file):
            rospy.logerr("Preview results not found: %s", preview_file)
            return
        with open(preview_file) as f:
            pr = json.load(f)
        selected = select_poses(pr, args.num_groups)
        sel_file = os.path.join(run_dir, "selected_poses.json")
        os.makedirs(run_dir, exist_ok=True)
        with open(sel_file, "w") as f:
            json.dump(selected, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

    # ── Capture ──
    if args.mode in ("capture", "full"):
        rospy.loginfo("=== FORMAL CAPTURE ===")
        if selected is None:
            sel_file = os.path.join(run_dir, "selected_poses.json")
            if not os.path.isfile(sel_file):
                rospy.logerr("Selected poses not found: %s", sel_file)
                return
            with open(sel_file) as f:
                selected = json.load(f)
        manifest = run_formal_capture(selected, run_dir, geom, profiles, camera_info)
        rospy.loginfo("Capture done: %d groups", len(manifest["groups"]))

    # ── Solve ──
    if args.mode in ("solve", "full"):
        rospy.loginfo("=== BLIND SOLVE ===")
        blind_result = run_blind_solve(run_dir, camera_info)
        if blind_result is None:
            rospy.logerr("Blind solve FAILED")
            return
        rospy.loginfo("Blind solve DONE")

    # ── Score ──
    if args.mode in ("score", "full"):
        rospy.loginfo("=== TRUTH SCORE ===")
        blind_file = os.path.join(run_dir, "camera_extrinsics_blind.json")
        if not os.path.isfile(blind_file):
            rospy.logerr("Blind result not found: %s", blind_file)
            return
        import subprocess
        score_script = os.path.join(os.path.dirname(__file__), "score_gazebo_calibration.py")
        score_out = os.path.join(run_dir, "truth_score.json")
        r = subprocess.run(["python3", score_script, blind_file, "-o", score_out],
                           capture_output=True, text=True, timeout=30,
                           env={**os.environ, "ROS_MASTER_URI": rospy.get_param("/ros_master_uri", "http://localhost:11313")})
        print(r.stdout)
        if r.returncode != 0:
            print(r.stderr)

    rospy.loginfo("V8.17 Dataset A pipeline DONE")
    print(f"\nRun dir: {run_dir}")
    print(f"Preview dir: {preview_dir}")


if __name__ == "__main__":
    main()

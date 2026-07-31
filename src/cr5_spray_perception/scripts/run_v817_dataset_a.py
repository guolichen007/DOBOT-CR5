#!/usr/bin/env python3
"""
V8.17A Dataset A Blind Acceptance Runner — with hardened sync capture.

Modes:
  --mode sync-test   : 10 sync captures, verify max skew ≤30ms
  --mode preview     : preview all candidates (move → sync capture ×3 → detect → report)
  --mode select      : select 20-24 groups from preview (with hard coverage gate)
  --mode capture     : formal fresh capture of selected poses (keep all raw snaps)
  --mode solve       : blind pairwise solve (no truth)
  --mode score       : truth score (Gazebo TF)

All target poses: BASE_TARGET + delta offsets. No old-world coords.

Usage:
  python3 run_v817_dataset_a.py --mode preview --run-id sim_v815_final_cell_A
"""
import sys, os, math, time, json, hashlib, argparse, shutil
import numpy as np
import cv2
import rospy
import rospkg
import yaml
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState, GetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

WS = "/home/ydkj/cr5_ros1_ws"
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_sim", "src"))

from cr5_spray_sim.scene_config import get_base_target_pose
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.pairwise_solver import compute_pairwise_rig
from cr5_spray_perception.calibration.pnp_solver import solve_pnp
from cr5_spray_perception.calibration.measurement import (
    CalibrationDataset, CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.geometry import se3_distance_mm_deg, invert_transform

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]
CAM_TOPICS = [f"/{c}/camera/color/image_raw" for c in CAMERAS]
SYNC_SLOP = 0.030  # 30ms
SYNC_MAX_SKEW_MS = 30.0
SYNC_TIMEOUT = 5.0

DATA_ROOT = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")), "calibration")

# ═══════════════════════════════════════════════════════════════
# Candidate Bank: BASE_TARGET + delta offsets
# ═══════════════════════════════════════════════════════════════

CANDIDATE_DELTAS = [
    # ── Bank A: FL right+top ──
    {"id": "A01", "dx": -0.04, "dy": -0.02, "dz": 0.00, "rpy": [12, 0, -10], "desc": "FL right+top roll12"},
    {"id": "A02", "dx": -0.04, "dy": -0.02, "dz": 0.03, "rpy": [15, 6, -12], "desc": "FL right+top roll15 pitch6"},
    {"id": "A03", "dx": 0.00, "dy": -0.04, "dz": -0.04, "rpy": [14, -3, -12], "desc": "FL right+top combo"},
    {"id": "A04", "dx": -0.08, "dy": -0.02, "dz": 0.00, "rpy": [12, 0, -10], "desc": "FL near right+top"},
    {"id": "A05", "dx": -0.08, "dy": -0.02, "dz": 0.02, "rpy": [10, 8, -8], "desc": "FL near right+top pitch"},
    {"id": "A06", "dx": -0.04, "dy": -0.02, "dz": 0.00, "rpy": [14, -3, 10], "desc": "FL right+top yaw+10"},
    # ── Bank B: FR left+top ──
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
    {"id": "D11", "dx": -0.04, "dy": -0.03, "dz": 0.02, "rpy": [15, 0, -5], "desc": "Roll+15 right"},
    {"id": "D12", "dx": -0.04, "dy": 0.03, "dz": 0.02, "rpy": [-15, 0, 5], "desc": "Roll-15 left"},
]


def get_world_xyz(delta):
    base = get_base_target_pose()
    return [base[0] + delta["dx"], base[1] + delta["dy"], base[2] + delta["dz"]]


# ═══════════════════════════════════════════════════════════════
# Gazebo interaction
# ═══════════════════════════════════════════════════════════════

def set_target_pose(xyz, rpy_deg):
    q = quaternion_from_euler(*[math.radians(r) for r in rpy_deg])
    rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    req = SetModelStateRequest()
    req.model_state.model_name = "simple_hanging_workpiece"
    req.model_state.pose = Pose(position=Point(*xyz), orientation=Quaternion(*q))
    req.model_state.reference_frame = "world"
    return svc(req).success


def get_target_pose_full():
    """Read current target position AND orientation from Gazebo. Returns (xyz, rpy_deg)."""
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    req = GetModelStateRequest()
    req.model_name = "simple_hanging_workpiece"
    req.relative_entity_name = "world"
    resp = svc(req)
    if resp.success:
        p = resp.pose.position
        o = resp.pose.orientation
        rpy = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return (p.x, p.y, p.z), tuple(math.degrees(r) for r in rpy)
    return None, None


def wait_settle(target_xyz, target_rpy_deg, tol_mm=2.0, tol_deg=0.1, max_wait=5.0, consec=3):
    """Wait for target to reach and stabilize at commanded pose. Checks both XYZ and RPY."""
    target_arr = np.array(target_xyz)
    target_rpy_arr = np.array(target_rpy_deg)
    stable_count = 0
    start = time.time()
    while time.time() - start < max_wait:
        rospy.sleep(0.3)
        xyz, rpy = get_target_pose_full()
        if xyz is None:
            continue
        t_err = float(np.linalg.norm(np.array(xyz) - target_arr)) * 1000
        r_err = float(np.linalg.norm(np.array(rpy) - target_rpy_arr))
        r_err = min(r_err, 360 - r_err)  # handle wrap
        if t_err < tol_mm and r_err < tol_deg:
            stable_count += 1
            if stable_count >= consec:
                return True, t_err, r_err
        else:
            stable_count = 0
            rospy.logdebug("  settle: t=%.1fmm r=%.2f° (need <%dmm/<%.1f°)", t_err, r_err, tol_mm, tol_deg)
    return False, t_err if 't_err' in dir() else 999, r_err if 'r_err' in dir() else 999


# ═══════════════════════════════════════════════════════════════
# True sync capture — message_filters.ApproximateTimeSynchronizer
# ═══════════════════════════════════════════════════════════════

class SyncCaptureResult:
    def __init__(self, images, stamps, skew_ms):
        self.images = images   # {cam: cv_img}
        self.stamps = stamps   # {cam: rospy.Time}
        self.skew_ms = skew_ms


def capture_sync_frames(timeout=SYNC_TIMEOUT):
    """Block until one synchronized triplet from all 3 cameras arrives.

    Uses ApproximateTimeSynchronizer with slop=30ms.
    Hard gate: max_timestamp_skew ≤ 30ms.
    Returns SyncCaptureResult or None on timeout/failure.
    """
    bridge = CvBridge()
    result_holder = {"result": None, "done": False}

    def callback(fl_img, fr_img, re_img):
        if result_holder["done"]:
            return
        try:
            images = {
                "cam_front_left": bridge.imgmsg_to_cv2(fl_img, "bgr8"),
                "cam_front_right": bridge.imgmsg_to_cv2(fr_img, "bgr8"),
                "cam_rear": bridge.imgmsg_to_cv2(re_img, "bgr8"),
            }
            stamps = {
                "cam_front_left": fl_img.header.stamp,
                "cam_front_right": fr_img.header.stamp,
                "cam_rear": re_img.header.stamp,
            }
            t_secs = [s.to_sec() for s in stamps.values()]
            skew_ms = (max(t_secs) - min(t_secs)) * 1000

            if skew_ms <= SYNC_MAX_SKEW_MS:
                result_holder["result"] = SyncCaptureResult(images, stamps, skew_ms)
                result_holder["done"] = True
            else:
                rospy.logwarn("Sync skew %.1f ms > %.0f ms — rejecting", skew_ms, SYNC_MAX_SKEW_MS)
        except Exception as e:
            rospy.logerr("Sync callback error: %s", e)

    # Create subscribers
    subs = []
    for i, topic in enumerate(CAM_TOPICS):
        sub = message_filters.Subscriber(topic, Image)
        subs.append(sub)

    sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=SYNC_SLOP)
    sync.registerCallback(callback)

    # Wait for result
    start = time.time()
    rate = rospy.Rate(50)
    while not result_holder["done"] and time.time() - start < timeout:
        rate.sleep()

    # Cleanup
    for sub in subs:
        sub.unregister()

    return result_holder["result"]


def run_sync_test(n=10):
    """Run N sync captures, report statistics."""
    rospy.loginfo("=== SYNC PREFLIGHT: %d captures ===", n)
    skews = []
    successes = 0
    for i in range(n):
        result = capture_sync_frames(timeout=SYNC_TIMEOUT)
        if result is not None:
            successes += 1
            skews.append(result.skew_ms)
            rospy.loginfo("  %2d: %.1f ms OK", i+1, result.skew_ms)
        else:
            rospy.logerr("  %2d: FAILED (no sync within %.0fs)", i+1, SYNC_TIMEOUT)

    if skews:
        rospy.loginfo("Sync stats: min=%.1f med=%.1f P95=%.1f max=%.1f ms",
                      np.min(skews), np.median(skews), np.percentile(skews, 95), np.max(skews))

    passed = (successes == n) and (max(skews) <= SYNC_MAX_SKEW_MS if skews else False)
    rospy.loginfo("SYNC_CAPTURE_%s: %d/%d, max skew=%.1f ms",
                  "PASS" if passed else "FAIL", successes, n,
                  max(skews) if skews else -1)
    return passed, successes, skews


# ═══════════════════════════════════════════════════════════════
# Preview
# ═══════════════════════════════════════════════════════════════

def analyze_detection(detection, geom):
    """Analyze detector output: faces, corners, per-face corner count."""
    if not detection:
        return {"faces": {}, "total_corners": 0, "n_faces_active": 0}
    faces = {}
    total = 0
    for fn, fd in detection.items():
        nc = fd.get("corner_count", 0)
        if nc > 0:
            faces[fn] = nc
            total += nc
    return {"faces": faces, "total_corners": total, "n_faces_active": len(faces)}


def run_preview(candidates, preview_dir, geom, profiles, camera_info):
    """Preview each candidate: move → settle → 3 sync captures → detect → report."""
    if os.path.isdir(preview_dir):
        backup = preview_dir.rstrip("/") + f"_backup_{int(time.time())}"
        rospy.logwarn("Preview dir exists, moving to %s", backup)
        os.rename(preview_dir, backup)
    os.makedirs(preview_dir)

    results = []
    K_arrays = {cam: camera_info[cam]["K"] for cam in CAMERAS}
    D_arrays = {cam: camera_info[cam]["D"] for cam in CAMERAS}

    for i, c in enumerate(candidates):
        wxyz = get_world_xyz(c)
        rospy.loginfo("Preview %d/%d: %s → (%.3f,%.3f,%.3f) rpy=%s",
                      i+1, len(candidates), c["id"], *wxyz, c["rpy"])

        set_target_pose(wxyz, c["rpy"])
        settled, settle_t_err, settle_r_err = wait_settle(wxyz, c["rpy"])

        # Save candidate-level directory
        cand_dir = os.path.join(preview_dir, c["id"])
        os.makedirs(cand_dir, exist_ok=True)

        snaps_data = []
        best_quality = -1
        best_idx = 0

        for snap in range(3):
            rospy.sleep(0.05)
            sync_result = capture_sync_frames(timeout=SYNC_TIMEOUT)
            snap_dir = os.path.join(cand_dir, f"snap_{snap}")
            os.makedirs(snap_dir, exist_ok=True)

            snap_info = {
                "snap": snap, "sync_ok": False, "skew_ms": None,
                "cams_detected": 0, "total_corners": 0,
                "per_cam": {}, "quality_score": 0,
            }

            if sync_result is not None:
                snap_info["sync_ok"] = True
                snap_info["skew_ms"] = sync_result.skew_ms
                # Save frames
                for cam, cv_img in sync_result.images.items():
                    cam_dir = os.path.join(snap_dir, cam)
                    os.makedirs(cam_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(cam_dir, "color.png"), cv_img)
                if sync_result.stamps:
                    snap_info["timestamps"] = {cam: s.to_sec() for cam, s in sync_result.stamps.items()}

                # Run detector on each camera
                for cam in CAMERAS:
                    if cam not in sync_result.images:
                        continue
                    det = detect_target(sync_result.images[cam], K_arrays[cam], D_arrays[cam], geom, profiles)
                    analysis = analyze_detection(det, geom)
                    snap_info["per_cam"][cam] = analysis
                    snap_info["total_corners"] += analysis["total_corners"]
                    snap_info["cams_detected"] += 1 if analysis["n_faces_active"] > 0 else 0

                # Quality: corners + multi-face bonus
                fl_faces = snap_info["per_cam"].get("cam_front_left", {}).get("faces", {})
                fr_faces = snap_info["per_cam"].get("cam_front_right", {}).get("faces", {})
                has_fl_rt = ("right" in fl_faces and "top" in fl_faces)
                has_fr_lt = ("left" in fr_faces and "top" in fr_faces)
                snap_info["has_fl_right_top"] = has_fl_rt
                snap_info["has_fr_left_top"] = has_fr_lt
                snap_info["quality_score"] = (
                    snap_info["total_corners"]
                    + (20 if has_fl_rt else 0) + (20 if has_fr_lt else 0)
                    + snap_info["cams_detected"] * 10
                )
            else:
                # Sync failed — save empty snap marker
                snap_info["sync_ok"] = False

            # Save capture meta per snap
            snap_info["settle_t_err_mm"] = settle_t_err
            snap_info["settle_r_err_deg"] = settle_r_err
            snap_info["settled"] = settled
            with open(os.path.join(snap_dir, "capture_meta.json"), "w") as f:
                json.dump(snap_info, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

            snaps_data.append(snap_info)
            if snap_info["quality_score"] > best_quality:
                best_quality = snap_info["quality_score"]
                best_idx = snap

        # Candidate summary
        best_snap = snaps_data[best_idx] if snaps_data else None
        summary = {
            "id": c["id"], "desc": c["desc"],
            "delta": {"dx": c["dx"], "dy": c["dy"], "dz": c["dz"], "rpy": c["rpy"]},
            "world_xyz": wxyz,
            "settled": settled, "settle_t_err_mm": settle_t_err, "settle_r_err_deg": settle_r_err,
            "n_snaps_ok": sum(1 for s in snaps_data if s["sync_ok"]),
            "best_snap": best_idx,
            "best_quality": best_quality,
            "best_corners": best_snap["total_corners"] if best_snap else 0,
            "best_cams": best_snap["cams_detected"] if best_snap else 0,
            "has_fl_right_top": best_snap.get("has_fl_right_top", False) if best_snap else False,
            "has_fr_left_top": best_snap.get("has_fr_left_top", False) if best_snap else False,
            "snaps": snaps_data,
            "status": "GOOD" if (settled and best_quality >= 30) else ("MARGINAL" if best_quality >= 10 else "BAD"),
        }
        results.append(summary)

        status_icon = {"GOOD": "+", "MARGINAL": "~", "BAD": "x"}.get(summary["status"], "?")
        rospy.loginfo("  [%s] %s: %d corners, %d cams, RT=%s LT=%s, settle=%s",
                      status_icon, c["id"], summary["best_corners"], summary["best_cams"],
                      "Y" if summary["has_fl_right_top"] else "N",
                      "Y" if summary["has_fr_left_top"] else "N",
                      "Y" if settled else "N")

    # Save preview report
    report = {
        "run_id": os.path.basename(preview_dir).replace("_preview", ""),
        "preview_dir": preview_dir,
        "n_candidates": len(candidates),
        "candidates": results,
        "n_good": sum(1 for r in results if r["status"] == "GOOD"),
        "n_marginal": sum(1 for r in results if r["status"] == "MARGINAL"),
        "n_bad": sum(1 for r in results if r["status"] == "BAD"),
    }
    with open(os.path.join(preview_dir, "preview_report.json"), "w") as f:
        json.dump(report, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))

    rospy.loginfo("Preview DONE: %d GOOD, %d MARGINAL, %d BAD",
                  report["n_good"], report["n_marginal"], report["n_bad"])
    return results


# ═══════════════════════════════════════════════════════════════
# Selection with hard coverage gate
# ═══════════════════════════════════════════════════════════════

SELECTION_REQUIREMENTS = {
    "FL_right_top": 4,
    "FR_left_top": 4,
    "depth": 4,       # abs(dx) >= 0.06
    "roll": 4,        # abs(roll) >= 10
    "pos_yaw": 2,     # yaw > 0
    "neg_yaw": 2,     # yaw < 0
    "pos_pitch": 2,   # pitch > 0
    "neg_pitch": 2,   # pitch < 0
    "pair_FL_FR": 8,  # groups where both FL and FR have detection
    "pair_FL_RE": 8,
    "pair_FR_RE": 6,
}


def select_poses(preview_results, n_target=22):
    """Select best poses with hard coverage gate. Returns (selected, coverage, gate_passed)."""
    # Filter to GOOD + MARGINAL
    eligible = [r for r in preview_results if r["status"] in ("GOOD", "MARGINAL")]
    rospy.loginfo("Eligible candidates: %d (from %d total)", len(eligible), len(preview_results))

    # Sort by quality
    scored = sorted(eligible, key=lambda r: r.get("best_quality", 0), reverse=True)

    selected = []
    used_ids = set()
    coverage = {k: 0 for k in SELECTION_REQUIREMENTS}

    for r in scored:
        if len(selected) >= n_target:
            break

        # Avoid near-duplicate
        too_close = False
        wxyz_cur = np.array(r["world_xyz"])
        for s in selected:
            wxyz_s = np.array(s["world_xyz"])
            t_dist = float(np.linalg.norm(wxyz_cur - wxyz_s)) * 1000
            r_cur = np.array(r["delta"]["rpy"])
            r_s = np.array(s["delta"]["rpy"])
            r_diff = float(np.linalg.norm(r_cur - r_s))
            r_diff = min(r_diff, 360 - r_diff)
            if t_dist < 30 and r_diff < 8:
                too_close = True
                break
        if too_close:
            continue

        selected.append(r)
        used_ids.add(r["id"])

        # Update coverage
        if r.get("has_fl_right_top"): coverage["FL_right_top"] += 1
        if r.get("has_fr_left_top"): coverage["FR_left_top"] += 1
        if abs(r["delta"]["dx"]) >= 0.06: coverage["depth"] += 1
        if abs(r["delta"]["rpy"][0]) >= 10: coverage["roll"] += 1
        if r["delta"]["rpy"][2] > 0: coverage["pos_yaw"] += 1
        if r["delta"]["rpy"][2] < 0: coverage["neg_yaw"] += 1
        if r["delta"]["rpy"][1] > 0: coverage["pos_pitch"] += 1
        if r["delta"]["rpy"][1] < 0: coverage["neg_pitch"] += 1

        # Pair support from snap data
        best_snap = r["snaps"][r["best_snap"]] if r.get("snaps") and r["best_snap"] < len(r.get("snaps", [])) else {}
        per_cam = best_snap.get("per_cam", {})
        fl_ok = per_cam.get("cam_front_left", {}).get("n_faces_active", 0) > 0
        fr_ok = per_cam.get("cam_front_right", {}).get("n_faces_active", 0) > 0
        re_ok = per_cam.get("cam_rear", {}).get("n_faces_active", 0) > 0
        if fl_ok and fr_ok: coverage["pair_FL_FR"] += 1
        if fl_ok and re_ok: coverage["pair_FL_RE"] += 1
        if fr_ok and re_ok: coverage["pair_FR_RE"] += 1

    # Pad if not enough
    if len(selected) < n_target:
        for r in scored:
            if len(selected) >= n_target:
                break
            if r["id"] in used_ids:
                continue
            selected.append(r)
            used_ids.add(r["id"])

    # Gate check
    gate_checks = {}
    gate_passed = True
    for key, req in SELECTION_REQUIREMENTS.items():
        ok = coverage[key] >= req
        gate_checks[key] = (coverage[key], req, ok)
        if not ok:
            gate_passed = False
            rospy.logerr("  COVERAGE FAIL: %s = %d < %d", key, coverage[key], req)

    rospy.loginfo("Selected %d poses, coverage gate: %s", len(selected), "PASS" if gate_passed else "FAIL")
    for key, (actual, req, ok) in sorted(gate_checks.items()):
        rospy.loginfo("  %s: %d/%d %s", key, actual, req, "OK" if ok else "FAIL")

    if gate_passed:
        rospy.loginfo("POSE_SELECTION_GATE_PASS")
    else:
        rospy.logerr("POSE_SELECTION_GATE_FAIL")

    return selected, coverage, gate_passed


# ═══════════════════════════════════════════════════════════════
# Formal Capture (keep ALL raw snapshots)
# ═══════════════════════════════════════════════════════════════

def run_formal_capture(selected, run_dir, geom, profiles, camera_info):
    """Fresh capture of selected poses. All 3 snaps preserved, best selected."""
    groups_dir = os.path.join(run_dir, "groups")
    os.makedirs(groups_dir, exist_ok=True)

    K_arrays = {cam: camera_info[cam]["K"] for cam in CAMERAS}
    D_arrays = {cam: camera_info[cam]["D"] for cam in CAMERAS}

    rp = rospkg.RosPack()
    scene_yaml_sha = "..."
    target_yaml_sha = "..."
    for fname, setter in [("simulation_scene.yaml", "scene"), ("calibration/calibration_target.yaml", "target")]:
        fpath = os.path.join(rp.get_path("cr5_spray_sim"), "config", fname)
        if os.path.isfile(fpath):
            sha = hashlib.sha256(open(fpath, "rb").read()).hexdigest()
            if "scene" in setter:
                scene_yaml_sha = sha
            else:
                target_yaml_sha = sha

    manifest = {
        "run_id": os.path.basename(run_dir),
        "scene_yaml_sha256": scene_yaml_sha,
        "target_yaml_sha256": target_yaml_sha,
        "n_groups": len(selected),
        "sync_max_skew_ms": SYNC_MAX_SKEW_MS,
        "groups": [],
    }

    for i, c in enumerate(selected):
        gid = i
        wxyz = c["world_xyz"]
        rospy.loginfo("Capture %d/%d: %s", i+1, len(selected), c["id"])

        set_target_pose(wxyz, c["delta"]["rpy"])
        settled, st_err, sr_err = wait_settle(wxyz, c["delta"]["rpy"])

        group_dir = os.path.join(groups_dir, f"group_{gid:04d}")
        raw_dir = os.path.join(group_dir, "raw")
        os.makedirs(raw_dir, exist_ok=True)

        snaps_info = []
        best_quality = -1
        best_snap_idx = 0

        for snap in range(3):
            rospy.sleep(0.05)
            sync_result = capture_sync_frames(timeout=SYNC_TIMEOUT)
            snap_dir = os.path.join(raw_dir, f"snap_{snap}")
            os.makedirs(snap_dir, exist_ok=True)

            snap_meta = {
                "snap": snap, "sync_ok": False, "skew_ms": None,
                "total_corners": 0, "cams_detected": 0, "per_cam": {},
            }

            if sync_result is not None:
                snap_meta["sync_ok"] = True
                snap_meta["skew_ms"] = sync_result.skew_ms
                for cam, cv_img in sync_result.images.items():
                    cam_dir = os.path.join(snap_dir, cam)
                    os.makedirs(cam_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(cam_dir, "color.png"), cv_img)
                if sync_result.stamps:
                    snap_meta["timestamps"] = {cam: s.to_sec() for cam, s in sync_result.stamps.items()}

                # Quality check
                for cam in CAMERAS:
                    if cam not in sync_result.images:
                        continue
                    det = detect_target(sync_result.images[cam], K_arrays[cam], D_arrays[cam], geom, profiles)
                    analysis = analyze_detection(det, geom)
                    snap_meta["per_cam"][cam] = analysis
                    snap_meta["total_corners"] += analysis["total_corners"]
                    snap_meta["cams_detected"] += 1 if analysis["n_faces_active"] > 0 else 0

                # Quality = corners + multi-cam weighting
                fl_faces = snap_meta["per_cam"].get("cam_front_left", {}).get("faces", {})
                fr_faces = snap_meta["per_cam"].get("cam_front_right", {}).get("faces", {})
                fl_rt = ("right" in fl_faces and "top" in fl_faces)
                fr_lt = ("left" in fr_faces and "top" in fr_faces)
                quality = snap_meta["total_corners"] + snap_meta["cams_detected"] * 10 + (20 if fl_rt else 0) + (20 if fr_lt else 0)
                snap_meta["has_fl_right_top"] = fl_rt
                snap_meta["has_fr_left_top"] = fr_lt
                snap_meta["quality_score"] = quality

                if quality > best_quality:
                    best_quality = quality
                    best_snap_idx = snap

            snap_meta["settle_t_err_mm"] = st_err
            snap_meta["settle_r_err_deg"] = sr_err
            snap_meta["settled"] = settled
            with open(os.path.join(snap_dir, "capture_meta.json"), "w") as f:
                json.dump(snap_meta, f, indent=2)
            snaps_info.append(snap_meta)

        # Create "selected" symlink or copy
        selected_dir = os.path.join(group_dir, "selected")
        best_src = os.path.join(raw_dir, f"snap_{best_snap_idx}")
        if os.path.exists(selected_dir):
            shutil.rmtree(selected_dir)
        shutil.copytree(best_src, selected_dir)

        # Write selection marker
        with open(os.path.join(group_dir, "selected_snap.txt"), "w") as f:
            f.write(str(best_snap_idx))

        rospy.loginfo("  → group_%04d: snap%d (%d corners)", gid, best_snap_idx, best_quality)

        manifest["groups"].append({
            "group_id": gid,
            "candidate_id": c["id"],
            "desc": c["desc"],
            "world_xyz": c["world_xyz"],
            "rpy": c["delta"]["rpy"],
            "best_snap": best_snap_idx,
            "best_quality": best_quality,
            "settled": settled,
            "settle_t_err_mm": st_err,
            "settle_r_err_deg": sr_err,
            "n_snaps_ok": sum(1 for s in snaps_info if s["sync_ok"]),
            "snaps": snaps_info,
        })

    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    return manifest


# ═══════════════════════════════════════════════════════════════
# Blind Solve
# ═══════════════════════════════════════════════════════════════

def run_blind_solve(run_dir, camera_info):
    """Blind pairwise solve — NO truth access."""
    groups_dir = os.path.join(run_dir, "groups")
    if not os.path.isdir(groups_dir):
        rospy.logerr("Groups directory not found: %s", groups_dir)
        return None

    geom = load_target_geometry()
    profiles = create_default_profiles()

    per_group_pnp = {}
    all_ids = sorted([int(d.replace("group_", "")) for d in os.listdir(groups_dir)
                      if d.startswith("group_") and os.path.isdir(os.path.join(groups_dir, d))])
    rospy.loginfo("Blind solve: %d groups", len(all_ids))

    for gid in all_ids:
        group_dir = os.path.join(groups_dir, f"group_{gid:04d}")
        # Read selected snap
        sel_file = os.path.join(group_dir, "selected_snap.txt")
        if os.path.isfile(sel_file):
            snap_idx = int(open(sel_file).read().strip())
            img_dir = os.path.join(group_dir, "raw", f"snap_{snap_idx}")
        else:
            img_dir = os.path.join(group_dir, "selected")

        group_pnp = {}
        for cam in CAMERAS:
            img_path = os.path.join(img_dir, cam, "color.png")
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
            obj_pts, img_pts = [], []
            for fn, fd in detection.items():
                oface = fd.get("object_points_3d_face", [])
                iface = fd.get("image_points_2d", [])
                if not oface:
                    continue
                T_face = geom.T_target_face.get(fn)
                if T_face is None:
                    continue
                for pi in range(len(oface)):
                    pt_tgt = (T_face @ np.array([*oface[pi], 1.0]))[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(iface[pi])
            if len(obj_pts) >= 4:
                T_cam_target, _, _, _ = solve_pnp(obj_pts, img_pts, K, D)
                if T_cam_target is not None:
                    group_pnp[cam] = T_cam_target
        if len(group_pnp) >= 2:
            per_group_pnp[gid] = group_pnp

    rospy.loginfo("Groups with 2+ camera PnP: %d", len(per_group_pnp))
    if len(per_group_pnp) < 5:
        rospy.logerr("Insufficient groups: %d", len(per_group_pnp))
        return None

    X_cameras, report = compute_pairwise_rig(per_group_pnp)

    blind_result = {
        "solver": "pairwise_camera_relative", "version": "V8.17",
        "run_id": os.path.basename(run_dir), "n_groups": len(per_group_pnp),
        "group_ids": sorted(per_group_pnp.keys()),
        "cameras": {cam: X_cameras.get(cam, np.eye(4)).tolist() for cam in CAMERAS},
        "pair_stats": report.get("pairs", {}),
        "triangle_closure_t_mm": report.get("triangle_closure_t_mm"),
        "triangle_closure_r_deg": report.get("triangle_closure_r_deg"),
    }

    out = os.path.join(run_dir, "camera_extrinsics_blind.json")
    with open(out, "w") as f:
        json.dump(blind_result, f, indent=2)
    blind_sha = hashlib.sha256(open(out, "rb").read()).hexdigest()
    rospy.loginfo("Blind result: %s  SHA256=%s", out, blind_sha)

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
    parser = argparse.ArgumentParser(description="V8.17A Dataset A Blind Acceptance")
    parser.add_argument("--mode", default="preview",
                        choices=["sync-test", "preview", "select", "capture", "solve", "score", "full"],
                        help="Pipeline mode (default: preview)")
    parser.add_argument("--run-id", default="sim_v815_final_cell_A", help="Dataset run ID")
    parser.add_argument("--num-groups", type=int, default=22, help="Target number of groups")
    args = parser.parse_args()

    rospy.init_node("v817_dataset_a", anonymous=True)

    raw_root = os.path.join(DATA_ROOT, "raw")
    runs_root = os.path.join(DATA_ROOT, "runs")
    preview_dir = os.path.join(raw_root, f"{args.run_id}_preview")
    run_dir = os.path.join(runs_root, args.run_id)

    base = get_base_target_pose()
    rospy.loginfo("BASE_TARGET: (%.3f, %.3f, %.3f)", *base)

    geom = load_target_geometry()
    profiles = create_default_profiles()

    camera_info = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        camera_info[cam] = {"K": K, "D": D, "width": info.width, "height": info.height}
        rospy.loginfo("%s: %dx%d K=[%.1f,%.1f]", cam, info.width, info.height, K[0, 0], K[1, 1])

    # ── Sync Test ──
    if args.mode == "sync-test":
        passed, successes, skews = run_sync_test(10)
        if not passed:
            rospy.logerr("SYNC_CAPTURE_FAIL — cannot proceed to preview")
            sys.exit(1)
        rospy.loginfo("SYNC_CAPTURE_PASS")
        return

    # ── Preview ──
    if args.mode == "preview":
        rospy.loginfo("=== PREVIEW ===")
        results = run_preview(list(CANDIDATE_DELTAS), preview_dir, geom, profiles, camera_info)
        # Print summary table
        print("\n" + "="*70)
        print("PREVIEW REPORT — STOP (no select/capture/solve/score)")
        print("="*70)
        good = [r for r in results if r["status"] == "GOOD"]
        marg = [r for r in results if r["status"] == "MARGINAL"]
        bad = [r for r in results if r["status"] == "BAD"]
        print(f"GOOD: {len(good)}, MARGINAL: {len(marg)}, BAD: {len(bad)}")
        if bad:
            print(f"BAD candidates: {[r['id'] for r in bad]}")
        print(f"FL right+top: {sum(1 for r in results if r.get('has_fl_right_top'))}")
        print(f"FR left+top:  {sum(1 for r in results if r.get('has_fr_left_top'))}")
        print(f"Preview dir: {preview_dir}")
        print("="*70)
        return

    # ── Select ──
    if args.mode == "select":
        rospy.loginfo("=== SELECT ===")
        preview_file = os.path.join(preview_dir, "preview_report.json")
        if not os.path.isfile(preview_file):
            rospy.logerr("Preview report not found: %s", preview_file)
            return
        with open(preview_file) as f:
            pr = json.load(f)
        results = pr["candidates"]
        selected, coverage, gate_passed = select_poses(results, args.num_groups)
        os.makedirs(run_dir, exist_ok=True)
        with open(os.path.join(run_dir, "selected_poses.json"), "w") as f:
            json.dump(selected, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating,)) else str(x))
        if not gate_passed:
            rospy.logerr("POSE_SELECTION_GATE_FAIL — fix coverage before capture")
            sys.exit(1)
        rospy.loginfo("POSE_SELECTION_GATE_PASS — ready for capture")
        return

    # ── Capture ──
    if args.mode == "capture":
        rospy.loginfo("=== FORMAL CAPTURE ===")
        sel_file = os.path.join(run_dir, "selected_poses.json")
        if not os.path.isfile(sel_file):
            rospy.logerr("Not found: %s", sel_file)
            return
        with open(sel_file) as f:
            selected = json.load(f)
        manifest = run_formal_capture(selected, run_dir, geom, profiles, camera_info)
        rospy.loginfo("Capture DONE: %d groups", len(manifest["groups"]))
        return

    # ── Solve ──
    if args.mode == "solve":
        rospy.loginfo("=== BLIND SOLVE ===")
        blind_result = run_blind_solve(run_dir, camera_info)
        if blind_result is None:
            rospy.logerr("Solve FAILED")
            return
        return

    # ── Score ──
    if args.mode == "score":
        rospy.loginfo("=== TRUTH SCORE ===")
        blind_file = os.path.join(run_dir, "camera_extrinsics_blind.json")
        if not os.path.isfile(blind_file):
            rospy.logerr("Not found: %s", blind_file)
            return
        import subprocess
        script = os.path.join(os.path.dirname(__file__), "score_gazebo_calibration.py")
        r = subprocess.run(["python3", script, blind_file, "-o", os.path.join(run_dir, "truth_score.json")],
                           capture_output=True, text=True, timeout=30,
                           env={**os.environ, "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI", "http://localhost:11313")})
        print(r.stdout)
        if r.returncode != 0:
            print(r.stderr)
        return

    # ── Full (NOT recommended for first run) ──
    if args.mode == "full":
        rospy.logwarn("FULL mode — not recommended for first run. Use step-by-step.")
        # (Keep for automated regression later)
        pass

    rospy.loginfo("DONE")


if __name__ == "__main__":
    main()

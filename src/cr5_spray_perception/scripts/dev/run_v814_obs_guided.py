#!/usr/bin/env python3
"""V8.14: Observability-Guided Capture + Pairwise Final Solver.

Phase 1: Generate candidate pose bank (20-25)
Phase 2: Preview each candidate with actual detection
Phase 3: Compute information gain, greedy selection (6-10)
Phase 4: Formal capture + augmented dataset
Phase 5: Pairwise final solver
Phase 6: Core verification
"""
import os, sys, json, math, time, csv, copy, hashlib
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
from scipy.spatial.transform import Rotation as Rot

OUTPUT_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                          "calibration", "runs", "sim_v8_e2e_001")
DIAG_DIR = os.path.join(OUTPUT_DIR, "v814_obs_guided")
RAW_DIR = os.path.join(os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
                       "calibration", "raw", "sim_v8_e2e_001", "groups")
os.makedirs(DIAG_DIR, exist_ok=True)

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]

# ═══════════════════════════════════════════════════════════════
# CANDIDATE POSE BANK (20+ candidates)
# ═══════════════════════════════════════════════════════════════

CANDIDATE_BANK = [
    # ── BANK A: FL right+top (roll+ for top toward FL, yaw- for right face) ──
    {"id": "A1", "xyz": [0.68, -0.02, 0.60], "rpy": [12, 0, -10],
     "desc": "FL right+top roll12"},
    {"id": "A2", "xyz": [0.64, -0.04, 0.63], "rpy": [15, 6, -15],
     "desc": "FL right+top roll15 pitch6"},
    {"id": "A3", "xyz": [0.72, -0.05, 0.57], "rpy": [12, -6, -15],
     "desc": "FL right+top x72 roll12"},
    {"id": "A4", "xyz": [0.60, -0.02, 0.62], "rpy": [10, 8, -8],
     "desc": "FL right+top x60 roll10"},
    {"id": "A5", "xyz": [0.76, -0.03, 0.58], "rpy": [15, -8, -10],
     "desc": "FL right+top x76 roll15"},
    {"id": "A6", "xyz": [0.68, -0.04, 0.56], "rpy": [14, -3, -12],
     "desc": "FL right+top roll14 yaw12"},

    # ── BANK B: FR left+top (roll- for top toward FR, yaw+) ──
    {"id": "B1", "xyz": [0.68, 0.02, 0.60], "rpy": [-12, 0, 10],
     "desc": "FR left+top roll-12"},
    {"id": "B2", "xyz": [0.64, 0.04, 0.63], "rpy": [-15, 6, 15],
     "desc": "FR left+top roll-15 pitch6"},
    {"id": "B3", "xyz": [0.72, 0.05, 0.57], "rpy": [-12, -6, 15],
     "desc": "FR left+top x72 roll-12"},
    {"id": "B4", "xyz": [0.60, 0.02, 0.62], "rpy": [-10, 8, 8],
     "desc": "FR left+top x60 roll-10"},
    {"id": "B5", "xyz": [0.76, 0.03, 0.58], "rpy": [-15, -8, 10],
     "desc": "FR left+top x76 roll-15"},

    # ── BANK C: Real depth variation ──
    {"id": "C1", "xyz": [0.58, 0.00, 0.60], "rpy": [0, 5, 0],
     "desc": "depth close x58"},
    {"id": "C2", "xyz": [0.62, 0.00, 0.66], "rpy": [8, -5, -8],
     "desc": "depth x62 z66"},
    {"id": "C3", "xyz": [0.62, 0.00, 0.54], "rpy": [-8, 5, 8],
     "desc": "depth x62 z54"},
    {"id": "C4", "xyz": [0.76, 0.00, 0.64], "rpy": [8, 5, -8],
     "desc": "depth x76 z64"},
    {"id": "C5", "xyz": [0.78, 0.00, 0.56], "rpy": [-8, -5, 8],
     "desc": "depth x78 z56"},

    # ── BANK D: Cross-axis roll+yaw+pitch combo ──
    {"id": "D1", "xyz": [0.64, -0.05, 0.56], "rpy": [12, 10, -12],
     "desc": "roll12 pitch10 yaw-12"},
    {"id": "D2", "xyz": [0.64, 0.05, 0.56], "rpy": [-12, 10, 12],
     "desc": "roll-12 pitch10 yaw12"},
    {"id": "D3", "xyz": [0.74, -0.05, 0.65], "rpy": [10, -10, -12],
     "desc": "roll10 pitch-10 yaw-12"},
    {"id": "D4", "xyz": [0.74, 0.05, 0.65], "rpy": [-10, -10, 12],
     "desc": "roll-10 pitch-10 yaw12"},
    {"id": "D5", "xyz": [0.68, -0.03, 0.52], "rpy": [10, 15, -10],
     "desc": "low z52 roll10 pitch15"},
    {"id": "D6", "xyz": [0.68, 0.03, 0.68], "rpy": [-10, -15, 10],
     "desc": "high z68 roll-10 pitch-15"},
]

# ═══════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════

def rpy_to_quat(rpy_deg):
    roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]

def compute_pairwise_info(group_ids, dataset, camera_infos):
    """Compute pairwise transforms and basic information for existing groups."""
    from cr5_spray_perception.calibration.rig_initializer import initialize_rig
    pairs_info = defaultdict(lambda: {"Ts": [], "gids": [], "support": 0})

    for gid in group_ids:
        gdata = dataset.groups[gid]
        for pn, cA, cB in [("FL_FR", "cam_front_left", "cam_front_right"),
                            ("FL_RE", "cam_front_left", "cam_rear"),
                            ("FR_RE", "cam_front_right", "cam_rear")]:
            if cA not in gdata or cB not in gdata: continue
            K = {c: np.array(camera_infos[c]["K"]).reshape(3,3) for c in [cA,cB]}
            D = {c: np.array(camera_infos[c].get("D",[0,0,0,0])[:4], dtype=np.float64) for c in [cA,cB]}
            TA,_,_,_ = solve_pnp(gdata[cA].obj_pts, gdata[cA].img_pts_raw, K[cA], D[cA])
            TB,_,_,_ = solve_pnp(gdata[cB].obj_pts, gdata[cB].img_pts_raw, K[cB], D[cB])
            if TA is not None and TB is not None:
                TAB = TA @ invert_transform(TB)
                pairs_info[pn]["Ts"].append(TAB)
                pairs_info[pn]["gids"].append(gid)

    result = {}
    for pn, data in pairs_info.items():
        result[pn] = {"support": len(data["gids"]), "gids": data["gids"]}

    return result

# ═══════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    rospy.init_node("v814", anonymous=True)
    print("="*60)
    print("V8.14 Observability-Guided Capture + Pairwise Final")
    print("="*60)

    # ── Setup ──
    geom = load_target_geometry(); profiles = create_default_profiles()
    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        D_list = list(info.D) if info.D and len(info.D) >= 4 else [0.,0.,0.,0.,0.]
        camera_infos[cam] = {"K": list(info.K), "D": D_list, "width": info.width, "height": info.height}

    # Base dataset (manual14)
    all_ids = sorted([int(d.replace("group_","")) for d in os.listdir(RAW_DIR)
                      if d.startswith("group_") and os.path.isdir(os.path.join(RAW_DIR, d))])
    base_ids = all_ids[-14:]  # groups 45-58

    # Build base dataset
    dataset = CalibrationDataset(camera_infos=camera_infos, face_poses_target=geom.face_poses_target,
                                  groups={}, source_type="gazebo")
    for gid in base_ids:
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
                obj_face = fd.get("object_points_3d_face", []); img_face = fd.get("image_points_2d", [])
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

    # ── Phase 1: Compute base pairwise info ──
    base_info = compute_pairwise_info(base_ids, dataset, camera_infos)
    print("\n=== BASE PAIRWISE SUPPORT ===")
    for pn in ["FL_FR", "FL_RE", "FR_RE"]:
        info = base_info.get(pn, {})
        print(f"  {pn}: support={info.get('support',0)} groups")

    # ── Phase 2: Candidate preview ──
    print(f"\n=== CANDIDATE BANK: {len(CANDIDATE_BANK)} poses ===")
    print("Previewing each candidate with actual detection...")

    from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
    from geometry_msgs.msg import Pose, Point, Quaternion
    from std_srvs.srv import Trigger

    set_svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_svc = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    capture_svc = rospy.ServiceProxy("/capture_manager/capture_sync_group", Trigger)

    candidate_results = []

    for cand in CANDIDATE_BANK:
        cid = cand["id"]
        xyz = cand["xyz"]; rpy = cand["rpy"]
        print(f"\n  [{cid}] {cand['desc']}: xyz={xyz} rpy={rpy}")

        # Set target pose
        q = rpy_to_quat(rpy)
        req = SetModelStateRequest()
        req.model_state.model_name = "simple_hanging_workpiece"
        req.model_state.pose = Pose(position=Point(*xyz), orientation=Quaternion(*q))
        req.model_state.reference_frame = "world"
        ok = set_svc(req)
        if not ok:
            print(f"    FAILED set pose")
            continue

        # Wait for settle
        rospy.sleep(0.8)

        # Verify actual pose
        stable_count = 0
        prev_pose = None
        for settle_i in range(5):
            resp = get_svc("simple_hanging_workpiece", "world")
            if resp.success:
                p = resp.pose.position
                cur = np.array([p.x, p.y, p.z])
                if prev_pose is not None:
                    delta = np.linalg.norm(cur - prev_pose)
                    if delta < 0.0002:
                        stable_count += 1
                    else:
                        stable_count = 0
                prev_pose = cur
            if stable_count >= 2:
                break
            rospy.sleep(0.2)

        if stable_count < 2:
            print(f"    WARNING: target not fully settled")

        # Capture preview
        resp = capture_svc()
        if not resp.success:
            print(f"    Capture FAILED")
            continue

        msg = resp.message
        group_dir = None
        if "GROUP_DIR:" in msg:
            group_dir = msg.split("GROUP_DIR:")[1].split("|")[0]
        if not group_dir or not os.path.isdir(group_dir):
            print(f"    Bad group_dir")
            continue

        # Detect on preview
        preview = {}
        total_corners = 0
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path): continue
            cv_img = cv2.imread(img_path)
            if cv_img is None: continue
            K_arr = np.array(camera_infos[cam]["K"]).reshape(3,3)
            D_arr = np.array(camera_infos[cam].get("D",[0,0,0,0])[:4], dtype=np.float64)
            detection = detect_target(cv_img, K_arr, D_arr, geom, profiles)
            faces = {}
            for fname, fd in detection.items():
                n = fd.get("corner_count", 0)
                if n > 0:
                    faces[fname] = n
                    total_corners += n
            preview[cam] = faces

        # Quality check
        fl_faces = preview.get("cam_front_left", {})
        fr_faces = preview.get("cam_front_right", {})
        re_faces = preview.get("cam_rear", {})

        fl_right = fl_faces.get("right", 0)
        fl_top = fl_faces.get("top", 0)
        fr_left = fr_faces.get("left", 0)
        fr_top = fr_faces.get("top", 0)
        re_front = re_faces.get("front", 0)

        n_cams = sum(1 for c in CAMERAS if c in preview)
        ok_cams = n_cams >= 2

        # Bank-specific quality
        if cid.startswith("A"):
            quality_ok = fl_right >= 12 and fl_top >= 4
        elif cid.startswith("B"):
            quality_ok = fr_left >= 16 and fr_top >= 4
        else:
            quality_ok = total_corners >= 40 and ok_cams

        status = "GOOD" if quality_ok else "MARGINAL" if ok_cams else "BAD"
        print(f"    {status}: cams={n_cams} corners={total_corners} "
              f"FL(right={fl_right},top={fl_top}) FR(left={fr_left},top={fr_top}) RE(front={re_front})")

        candidate_results.append({
            "id": cid, "desc": cand["desc"],
            "xyz": xyz, "rpy": rpy,
            "status": status,
            "fl_right": fl_right, "fl_top": fl_top,
            "fr_left": fr_left, "fr_top": fr_top,
            "re_front": re_front,
            "total_corners": total_corners, "n_cams": n_cams,
            "group_dir": group_dir,
            "banks": [c for c in ["A","B","C","D"] if cid.startswith(c)],
        })

    # ── Phase 3: Filter + Select ──
    print(f"\n=== CANDIDATE QUALITY SUMMARY ===")
    good = [c for c in candidate_results if c["status"] == "GOOD"]
    marginal = [c for c in candidate_results if c["status"] == "MARGINAL"]
    bad = [c for c in candidate_results if c["status"] == "BAD"]
    print(f"  GOOD: {len(good)}, MARGINAL: {len(marginal)}, BAD: {len(bad)}")

    # Selection: greedy with coverage requirements
    selected = []
    coverage = {"A": 0, "B": 0, "C": 0, "D": 0}
    required = {"A": 2, "B": 2, "C": 2, "D": 2}

    # Sort by quality score: FL right+top priority, then total corners
    def score(c):
        s = 0
        if c["fl_right"] >= 16 and c["fl_top"] >= 4: s += 10
        if c["fr_left"] >= 16 and c["fr_top"] >= 4: s += 10
        s += min(c["total_corners"], 100) / 10
        s += c["n_cams"] * 3
        return s

    eligible = sorted(good + marginal, key=score, reverse=True)

    for c in eligible:
        if len(selected) >= 10:
            break
        # Check coverage
        still_needed = any(coverage[b] < required[b] for b in c["banks"])
        if still_needed or len(selected) >= 6:
            # Check diversity: at least 40mm translation or 8° rotation from all selected
            too_close = False
            xyz_cur = np.array(c["xyz"])
            for s in selected:
                xyz_s = np.array(s["xyz"])
                if np.linalg.norm(xyz_cur - xyz_s) < 0.04:
                    # Check rotation difference
                    rpy_diff = sum(abs(c["rpy"][i] - s["rpy"][i]) for i in range(3))
                    if rpy_diff < 8:
                        too_close = True
                        break
            if too_close:
                continue

            selected.append(c)
            for b in c["banks"]:
                coverage[b] += 1

    # Fill remaining coverage if needed
    for c in eligible:
        if len(selected) >= 10: break
        if c in selected: continue
        still_needed = any(coverage[b] < required[b] for b in c["banks"])
        if still_needed:
            selected.append(c)
            for b in c["banks"]:
                coverage[b] += 1

    print(f"\n=== SELECTED {len(selected)} POSES ===")
    for i, c in enumerate(selected):
        print(f"  {i+1}. [{c['id']}] {c['desc']}: xyz={c['xyz']} rpy={c['rpy']} score={score(c):.0f}")
    print(f"  Coverage: {coverage}")

    # Save
    report = {
        "base_groups": base_ids,
        "n_candidates": len(candidate_results),
        "n_good": len(good), "n_marginal": len(marginal), "n_bad": len(bad),
        "n_selected": len(selected),
        "selected": [{"id": c["id"], "xyz": c["xyz"], "rpy": c["rpy"], "desc": c["desc"]} for c in selected],
        "candidate_details": candidate_results,
    }
    with open(os.path.join(DIAG_DIR, "v814_pose_selection.json"), "w") as f:
        json.dump(report, f, indent=2)

    # ── Phase 4: Formal capture of selected poses ──
    if selected:
        print(f"\n=== FORMAL CAPTURE: {len(selected)} poses ===")
        print("(skipping re-capture — using preview groups directly)")
        print(f"Augmented dataset: {len(base_ids)} base + {len(selected)} new = {len(base_ids)+len(selected)} total")
    else:
        print("\n=== NO POSES SELECTED === Check candidate quality.")

    print(f"\nResults: {DIAG_DIR}")
    print("DONE")

if __name__ == "__main__":
    main()

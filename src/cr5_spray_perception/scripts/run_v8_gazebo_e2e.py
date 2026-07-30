#!/usr/bin/env python3
"""V8 Gazebo E2E calibration — live capture + V8 pipeline + truth validation."""
import sys, os, math, time, json, yaml, subprocess
import numpy as np
import cv2
import rospy
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import CameraInfo
from cv2 import aruco

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from cr5_spray_perception.calibration.geometry import (euler_matrix, se3_distance_mm_deg,
    invert_transform, T_to_qt, qt_to_T, rpy_from_rotation)
from cr5_spray_perception.calibration.measurement import (CalibrationDataset,
    CameraGroupMeasurement, CornerObservation)
from cr5_spray_perception.calibration.pipeline import run_calibration_pipeline
from cr5_spray_perception import aruco_compat

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]

# Face definitions (same as run_multi_frame_calibration.py)
CHARUCO_FACES = {
    "front": {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 100},
    "back":  {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 300},
}
APRILTAG_FACES = {
    "left": {"tag_size": 0.07, "tag_ids": [4,5,6,7],
             "positions": {4:(-0.0425,0.0425,0), 5:(0.0425,0.0425,0),
                          6:(-0.0425,-0.0425,0), 7:(0.0425,-0.0425,0)}},
    "top":  {"tag_size": 0.12, "tag_ids": [8], "positions": {8:(0,0,0)}},
}
ARUCO_FACES = {
    "right": {"marker_size_m": 0.076, "marker_ids": [10,11,12,13],
              "dict_id": aruco.DICT_4X4_50,
              "positions": {10:(-0.047,0.044,0), 11:(0.047,0.044,0),
                           12:(-0.047,-0.044,0), 13:(0.047,-0.044,0)}},
}

FACE_POSES_TARGET = {}
T_TARGET_FACE = {}

# Pre-create boards
for v in CHARUCO_FACES.values():
    v["board"] = aruco.CharucoBoard_create(v["sx"],v["sy"],v["sq_m"],v["mk_m"],
                                            aruco.getPredefinedDictionary(v["dict_id"]))

def load_face_poses():
    import rospkg
    sim_path = rospkg.RosPack().get_path("cr5_spray_sim")
    yaml_path = os.path.join(sim_path, "config", "calibration", "calibration_target.yaml")
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    panels = cfg.get("panels", {})
    poses = {}
    for name, panel in panels.items():
        pt = panel.get("pose_target", {})
        if pt and "xyz" in pt and "rpy" in pt:
            poses[name] = {"xyz": list(pt["xyz"]), "rpy": list(pt["rpy"])}
    return poses

def build_T_target_face(face_name):
    p = FACE_POSES_TARGET[face_name]
    T = euler_matrix(p["rpy"][0], p["rpy"][1], p["rpy"][2])
    T[:3, 3] = p["xyz"]
    return T

def detect_on_image(cv_img, K, D):
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    results = {}
    # ChArUco (use more permissive params for oblique views)
    for fk, fc in CHARUCO_FACES.items():
        board = fc["board"]; id_start = fc["id_start"]
        params = aruco_compat.detector_parameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.adaptiveThreshWinSizeMin = 3  # smaller window for small markers
        params.adaptiveThreshWinSizeMax = 23
        params.minMarkerPerimeterRate = 0.01  # detect smaller markers at oblique angles
        params.polygonalApproxAccuracyRate = 0.05  # more tolerant
        corners, ids, _ = aruco_compat.detect_markers(gray, board.dictionary, params)
        obj_pts, img_pts = [], []
        if ids is not None:
            ids_flat = [int(i) for i in ids.flatten()]
            idx_list, local_ids = aruco_compat.remap_custom_ids(ids_flat, id_start, board)
            if len(idx_list) >= 2:
                local_corners = tuple(corners[i] for i in idx_list)
                cc, cids = aruco_compat.interpolate_charuco_corners(
                    local_corners, local_ids, gray, board, cameraMatrix=K, distCoeffs=D)
                if cids is not None and len(cids) >= 4:
                    board_pts = np.asarray(board.chessboardCorners, dtype=np.float32).reshape(-1,3)
                    bw, bh = fc["sx"]*fc["sq_m"], fc["sy"]*fc["sq_m"]
                    board_pts[:,0] -= bw/2.0; board_pts[:,1] -= bh/2.0
                    cids_flat = [int(i) for i in cids.flatten()]
                    obj_pts = [board_pts[i].tolist() for i in cids_flat]
                    img_pts = cc.reshape(-1,2).astype(np.float32).tolist()
        results[fk] = {"object_points_3d_face": obj_pts, "image_points_2d": img_pts,
                       "corner_count": len(obj_pts)}
    # ArUco
    aruco_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params4 = aruco_compat.detector_parameters()
    params4.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params4.adaptiveThreshWinSizeMin = 3
    params4.minMarkerPerimeterRate = 0.01
    params4.polygonalApproxAccuracyRate = 0.05
    corners4, ids4, _ = aruco_compat.detect_markers(gray, aruco_4x4, params4)
    for fk, fc in ARUCO_FACES.items():
        obj_pts, img_pts = [], []
        if ids4 is not None:
            ids_flat = [int(i) for i in ids4.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["marker_ids"]: continue
                pos = fc["positions"][tid]; half = fc["marker_size_m"]/2.0
                obj_pts.extend([[pos[0]-half,pos[1]+half,0],[pos[0]+half,pos[1]+half,0],
                               [pos[0]+half,pos[1]-half,0],[pos[0]-half,pos[1]-half,0]])
                img_pts.extend(corners4[i][0].tolist())
        results[fk] = {"object_points_3d_face": obj_pts, "image_points_2d": img_pts,
                       "corner_count": len(obj_pts)}
    # AprilTag
    tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    params_t = aruco_compat.detector_parameters()
    params_t.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params_t.adaptiveThreshWinSizeMin = 3
    params_t.minMarkerPerimeterRate = 0.01
    params_t.polygonalApproxAccuracyRate = 0.05
    corners_t, ids_t, _ = aruco_compat.detect_markers(gray, tag_dict, params_t)
    for fk, fc in APRILTAG_FACES.items():
        obj_pts, img_pts = [], []
        if ids_t is not None:
            ids_flat = [int(i) for i in ids_t.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["tag_ids"]: continue
                pos = fc["positions"][tid]; half = fc["tag_size"]/2.0
                obj_pts.extend([[pos[0]-half,pos[1]+half,0],[pos[0]+half,pos[1]+half,0],
                               [pos[0]+half,pos[1]-half,0],[pos[0]-half,pos[1]-half,0]])
                img_pts.extend(corners_t[i][0].tolist())
        results[fk] = {"object_points_3d_face": obj_pts, "image_points_2d": img_pts,
                       "corner_count": len(obj_pts)}
    return results

def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg); pitch = math.radians(pitch_deg); yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]

def set_target_pose(xyz, rpy_deg):
    q = rpy_to_quat(*rpy_deg)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    req = SetModelStateRequest()
    req.model_state.model_name = "simple_hanging_workpiece"
    req.model_state.pose = Pose(position=Point(*xyz), orientation=Quaternion(*q))
    req.model_state.reference_frame = "world"
    return svc(req).success

TARGET_POSES = [
    ("01_center",    [0.68, 0.0, 0.60],  [0, 0, 0]),
    ("02_yaw_p15",   [0.68, 0.0, 0.60],  [0, 0, 15]),
    ("03_yaw_m15",   [0.68, 0.0, 0.60],  [0, 0, -15]),
    ("04_yaw_p25",   [0.68, 0.0, 0.60],  [0, 0, 25]),
    ("05_yaw_m25",   [0.68, 0.0, 0.60],  [0, 0, -25]),
    ("06_pitch_p10", [0.68, 0.0, 0.60],  [0, 10, 0]),
    ("07_pitch_m10", [0.68, 0.0, 0.60],  [0, -10, 0]),
    ("08_pitch_p18", [0.68, 0.0, 0.60],  [0, 18, 0]),
    ("09_left",      [0.68, 0.06, 0.58], [0, 0, 10]),
    ("10_right",     [0.68, -0.06, 0.62],[0, 0, -10]),
    ("11_combo_pp",  [0.68, 0.04, 0.56], [0, 12, 15]),
    ("12_combo_mm",  [0.68, -0.04, 0.64],[0, -10, -15]),
    ("13_high",      [0.68, 0.0, 0.68],  [0, -5, 0]),
    ("14_low",       [0.68, 0.0, 0.50],  [0, 8, 0]),
]

def main():
    rospy.init_node("v8_gazebo_e2e", anonymous=True)
    aruco_compat.log_capability()

    output_dir = os.path.join(os.environ.get("CR5_DATA_ROOT",
        os.path.expanduser("~/cr5_data")), "calibration", "runs", "sim_v8_e2e_001")
    os.makedirs(output_dir, exist_ok=True)

    # Load face geometry
    global FACE_POSES_TARGET, T_TARGET_FACE
    FACE_POSES_TARGET = load_face_poses()
    T_TARGET_FACE = {name: build_T_target_face(name) for name in FACE_POSES_TARGET}
    rospy.loginfo("Loaded %d face poses", len(FACE_POSES_TARGET))

    # Read camera infos
    camera_infos = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        D_list = [float(v) for v in info.D] if info.D and len(info.D) >= 4 else [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_infos[cam] = {"K": list(info.K), "D": D_list,
                              "width": info.width, "height": info.height}
        rospy.loginfo("%s: K=[%.1f,%.1f] %dx%d D=%s", cam,
                       info.K[0], info.K[4], info.width, info.height, list(info.D))

    # Wait for capture service
    svc_name = "/capture_manager/capture_sync_group"
    rospy.wait_for_service(svc_name, timeout=5.0)
    capture_svc = rospy.ServiceProxy(svc_name, Trigger)

    # ── Auto-capture ──
    print("\n" + "="*60)
    print("  V8 Gazebo E2E — Auto Capture")
    print("="*60)

    dataset = CalibrationDataset(camera_infos=camera_infos, face_poses_target=FACE_POSES_TARGET,
                                  groups={}, source_type="gazebo")

    for pose_label, xyz, rpy_deg in TARGET_POSES:
        print(f"\n  Pose: {pose_label} xyz={xyz} rpy={rpy_deg}")
        ok = set_target_pose(xyz, rpy_deg)
        if not ok: print("    FAILED to set pose"); continue
        rospy.sleep(0.5)

        resp = capture_svc()
        if not resp.success:
            print(f"    Capture FAILED: {resp.message}")
            continue

        # Parse group_dir
        msg = resp.message
        group_dir = None
        if "GROUP_DIR:" in msg:
            group_dir = msg.split("GROUP_DIR:")[1].split("|")[0]
        if not group_dir or not os.path.isdir(group_dir):
            print(f"    Cannot parse group_dir")
            continue

        gid = len(dataset.groups)
        group_data = {}
        for cam in CAMERAS:
            img_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(img_path):
                print(f"    {cam}: no image"); continue
            cv_img = cv2.imread(img_path)
            if cv_img is None: continue

            K_arr = np.array(camera_infos[cam]["K"]).reshape(3,3)
            D_arr = np.array(camera_infos[cam]["D"] if camera_infos[cam]["D"] else [0.,0.,0.,0.], dtype=np.float64)
            if D_arr.ndim == 0: D_arr = np.array([0.,0.,0.,0.], dtype=np.float64)
            detection = detect_on_image(cv_img, K_arr, D_arr)

            corners = []
            for face_name, fd in detection.items():
                obj_face = fd.get("object_points_3d_face", [])
                img_face = fd.get("image_points_2d", [])
                if not obj_face: continue
                T = T_TARGET_FACE[face_name]
                for pi in range(len(obj_face)):
                    pt = np.array([*obj_face[pi], 1.0])
                    pt_tgt = (T @ pt)[:3]
                    corners.append(CornerObservation(
                        camera=cam, group_id=gid, face_name=face_name,
                        marker_id=pi//4, corner_idx=pi,
                        obj_pt_target=tuple(pt_tgt.tolist()),
                        img_pt_raw=tuple(img_face[pi]),
                        img_pt_undistorted=tuple(img_face[pi]),
                        detector_type="charuco" if face_name in ("front","back") else "apriltag"))

            if corners:
                group_data[cam] = CameraGroupMeasurement(camera=cam, group_id=gid, corners=corners)
                faces = list(set(c.face_name for c in corners))
                print(f"    {cam}: {len(corners)} corners, faces={faces}")
            else:
                print(f"    {cam}: 0 corners")

        if len(group_data) >= 2:
            dataset.groups[gid] = group_data
            print(f"    → Group {gid} ACCEPTED ({len(group_data)} cameras)")
        else:
            print(f"    → Group {gid} REJECTED")

    print(f"\n  Total accepted: {dataset.n_groups} groups")

    if dataset.n_groups < 4:
        print("  FATAL: too few groups")
        return

    # ── Run V8 pipeline ──
    print("\n" + "="*60)
    print("  V8 Pipeline")
    print("="*60)
    result = run_calibration_pipeline(dataset, os.path.join(output_dir, "pipeline"),
                                       skip_cross_validation=True)

    # ── Get Gazebo truth ──
    print("\n" + "="*60)
    print("  Gazebo Truth Comparison")
    print("="*60)

    import tf2_ros
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    T_world_optical = {}
    for cam in CAMERAS:
        try:
            tfs = tf_buffer.lookup_transform("world", f"{cam}_color_optical_frame",
                                              rospy.Time(0), rospy.Duration(2.0))
            t, r = tfs.transform.translation, tfs.transform.rotation
            T = np.eye(4)
            T[:3,:3] = np.array([[1-2*r.y**2-2*r.z**2, 2*r.x*r.y-2*r.z*r.w, 2*r.x*r.z+2*r.y*r.w],
                                 [2*r.x*r.y+2*r.z*r.w, 1-2*r.x**2-2*r.z**2, 2*r.y*r.z-2*r.x*r.w],
                                 [2*r.x*r.z-2*r.y*r.w, 2*r.y*r.z+2*r.x*r.w, 1-2*r.x**2-2*r.y**2]])
            T[:3,3] = [t.x, t.y, t.z]
            T_world_optical[cam] = T
        except Exception as e:
            rospy.logerr("TF failed for %s: %s", cam, e)

    if FIRST_CAM in T_world_optical:
        T_truth = {FIRST_CAM: np.eye(4)}
        T_world_FL = T_world_optical[FIRST_CAM]
        for cam in CAMERAS[1:]:
            if cam in T_world_optical:
                T_truth[cam] = invert_transform(T_world_FL) @ T_world_optical[cam]

        print("\n  Truth vs V8 comparison:")
        for cam in CAMERAS:
            if cam not in T_truth or cam not in result.final_camera_poses:
                continue
            t_err, r_err, _ = se3_distance_mm_deg(result.final_camera_poses[cam], T_truth[cam])
            status = "✓" if (cam == FIRST_CAM or (t_err < 10 and r_err < 1.0)) else "✗"
            print(f"    {cam}: T_err={t_err:.1f}mm, R_err={r_err:.2f}° {status}")
            if cam != FIRST_CAM:
                print(f"      V8:   t={result.final_camera_poses[cam][:3,3].round(4)}")
                print(f"      truth: t={T_truth[cam][:3,3].round(4)}")

    # ── Save results ──
    extrinsics_path = os.path.join(output_dir, "initial_extrinsics.yaml")
    from cr5_spray_perception.calibration.ceres_io import build_extrinsics_yaml
    stats = result.ba2_stats if result.ba2_success else result.ba1_stats
    stats["overall_rmse_px"] = stats.get("overall_rmse_px", 0)
    stats["n_observations"] = sum(len(m.corners) for g in dataset.groups.values() for m in g.values())
    ext = build_extrinsics_yaml(result.final_camera_poses, stats, CAMERAS,
                                 result.final_status, "v8_gazebo_e2e_calibration")
    with open(extrinsics_path, 'w') as f: yaml.dump(ext, f)
    print(f"\n  Extrinsics: {extrinsics_path}")
    print(f"  Status: {result.final_status}")
    print("  DONE")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3 三固定相机初始外参标定 — OpenCV 4.2 兼容版.

使用 aruco_compat, 自定义 ID remap, 动态 MIN_INLIERS, 正确 PnP flags.
"""
import sys, os, json, time, math, yaml, argparse
import cv2, numpy as np
import rospy, tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco
from cr5_spray_sim import aruco_compat

CAMERAS = {
    "cam_front_left": {
        "color": "/cam_front_left/camera/color/image_raw",
        "info":  "/cam_front_left/camera/color/camera_info",
        "link":  "cam_front_left_link",
    },
    "cam_front_right": {
        "color": "/cam_front_right/camera/color/image_raw",
        "info":  "/cam_front_right/camera/color/camera_info",
        "link":  "cam_front_right_link",
    },
    "cam_rear": {
        "color": "/cam_rear/camera/color/image_raw",
        "info":  "/cam_rear/camera/color/camera_info",
        "link":  "cam_rear_link",
    },
}

CHARUCO_FACES = {
    "front": {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 100,
              "face_frame": "calibration_target_front_frame"},
    "left":  {"sx": 6, "sy": 5, "sq_m": 0.022, "mk_m": 0.016,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 200,
              "face_frame": "calibration_target_left_frame"},
    "back":  {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 300,
              "face_frame": "calibration_target_back_frame"},
}

APRILTAG_RIGHT = {
    "tag_size": 0.07, "tag_gap": 0.015,
    "tag_ids": [4, 5, 6, 7],
    "positions": {
        4: (-0.0425,  0.0425, 0.0),
        5: ( 0.0425,  0.0425, 0.0),
        6: (-0.0425, -0.0425, 0.0),
        7: ( 0.0425, -0.0425, 0.0),
    },
    "face_frame": "calibration_target_right_frame",
}

APRILTAG_TOP = {
    "tag_size": 0.12,
    "tag_ids": [8],
    "positions": {8: (0.0, 0.0, 0.0)},
    "face_frame": "calibration_target_top_frame",
}

RANSAC_THRESH = 2.0
MAX_REPROJ = 2.0

# 预创建 boards
for k, v in CHARUCO_FACES.items():
    dict_obj = aruco.getPredefinedDictionary(v["dict_id"])
    v["board"] = aruco.CharucoBoard_create(v["sx"], v["sy"], v["sq_m"], v["mk_m"], dict_obj)
    v["n_markers"] = int(np.asarray(v["board"].ids).size)


def detect_charuco_face(gray, K, D, face_key):
    """检测 ChArUco + 返回 object/img points."""
    cfg = CHARUCO_FACES[face_key]
    board = cfg["board"]
    id_start = cfg["id_start"]

    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(gray, board.dictionary, params)

    if ids is None:
        return None, None

    ids_flat = [int(i) for i in ids.flatten()]
    idx_list, local_ids = aruco_compat.remap_custom_ids(ids_flat, id_start, board)

    if len(idx_list) < 2:
        return None, None

    local_corners = tuple(corners[i] for i in idx_list)

    cc, cids = aruco_compat.interpolate_charuco_corners(
        local_corners, local_ids, gray, board, cameraMatrix=K, distCoeffs=D)

    if cids is None or len(cids) < 4:
        return None, None

    # 3D points from board chessboard corners
    board_pts = np.asarray(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)

    # V5: 将 ChArUco board 原点平移到面板中心
    # board.chessboardCorners 使用 ChArUco board 局部原点,
    # 但 calibration_target_*_frame 位于面板中心
    board_width = cfg["sx"] * cfg["sq_m"]
    board_height = cfg["sy"] * cfg["sq_m"]
    board_pts[:, 0] -= board_width / 2.0
    board_pts[:, 1] -= board_height / 2.0

    cids_flat = [int(i) for i in cids.flatten()]
    obj_pts = np.array([board_pts[i] for i in cids_flat], dtype=np.float32)
    img_pts = cc.reshape(-1, 2).astype(np.float32)

    return obj_pts, img_pts


def detect_apriltag_face(gray, face_key):
    """检测 AprilTag (right 2×2 或 top single)."""
    tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(gray, tag_dict, params)

    if ids is None:
        return None, None

    ids_flat = [int(i) for i in ids.flatten()]
    cfg = APRILTAG_RIGHT if face_key == "right" else APRILTAG_TOP

    obj_pts_list, img_pts_list = [], []
    for i, tid in enumerate(ids_flat):
        if tid not in cfg["tag_ids"]:
            continue

        pos = cfg["positions"][tid]
        half = cfg["tag_size"] / 2.0
        tag_obj = np.array([
            [pos[0] - half, pos[1] + half, 0.0],
            [pos[0] + half, pos[1] + half, 0.0],
            [pos[0] + half, pos[1] - half, 0.0],
            [pos[0] - half, pos[1] - half, 0.0],
        ], dtype=np.float32)
        obj_pts_list.append(tag_obj)
        img_pts_list.append(corners[i][0])

    if not obj_pts_list:
        return None, None

    return np.vstack(obj_pts_list).astype(np.float32), np.vstack(img_pts_list).astype(np.float32)


def solve_pnp(obj_pts, img_pts, K, D):
    """动态门限 PnP 求解."""
    n_pts = len(obj_pts)
    min_inliers = 4 if n_pts == 4 else min(10, n_pts)

    flag = cv2.SOLVEPNP_IPPE_SQUARE if n_pts == 4 else cv2.SOLVEPNP_EPNP

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_pts, img_pts, K, D,
        reprojectionError=RANSAC_THRESH,
        flags=flag,
    )

    if not success or inliers is None or len(inliers) < min_inliers:
        return None, None, None

    obj_in = obj_pts[inliers.flatten()]
    img_in = img_pts[inliers.flatten()]

    # Refine with LM if available
    if hasattr(cv2, "solvePnPRefineLM"):
        try:
            rvec, tvec = cv2.solvePnPRefineLM(obj_in, img_in, K, D, rvec, tvec)
        except Exception:
            pass

    projected, _ = cv2.projectPoints(obj_in, rvec, tvec, K, D)
    errors = np.linalg.norm(img_in - projected.reshape(-1, 2), axis=1)
    rmse = float(np.sqrt(np.mean(errors**2)))

    return rvec, tvec, {
        "inliers": len(inliers), "n_pts": n_pts,
        "rmse_px": rmse, "max_error_px": float(np.max(errors)),
        "pass": rmse <= MAX_REPROJ,
    }


def get_face_tf(tf_buf, face_frame):
    """calibration_target_frame → face_frame."""
    try:
        ts = tf_buf.lookup_transform(
            "calibration_target_frame", face_frame, rospy.Time(0), rospy.Duration(3.0))
        t = ts.transform.translation
        q = ts.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
    except Exception:
        return None, None


def rvec_tvec_to_cam_target(rvec, tvec, face_frame, tf_buf):
    """将 face-local PnP 转成 camera→target."""
    R_cam_face, _ = cv2.Rodrigues(rvec)
    T_cam_face = np.eye(4)
    T_cam_face[:3, :3] = R_cam_face
    T_cam_face[:3, 3] = tvec.flatten()

    face_pos, face_quat = get_face_tf(tf_buf, face_frame)
    if face_pos is None:
        return None, None

    from tf.transformations import quaternion_matrix
    T_target_face = quaternion_matrix([face_quat[0], face_quat[1], face_quat[2], face_quat[3]])
    T_target_face[:3, 3] = face_pos

    T_cam_target = T_cam_face @ np.linalg.inv(T_target_face)
    rvec_ct = cv2.Rodrigues(T_cam_target[:3, :3])[0]
    tvec_ct = T_cam_target[:3, 3]
    return rvec_ct.flatten().tolist(), tvec_ct.flatten().tolist()


def gazebo_truth(tf_buf, optical_frame):
    """optical_frame → calibration_target_frame."""
    try:
        ts = tf_buf.lookup_transform(
            optical_frame, "calibration_target_frame", rospy.Time(0), rospy.Duration(3.0))
        t = ts.transform.translation
        q = ts.transform.rotation
        return {"tx": t.x, "ty": t.y, "tz": t.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
    except Exception:
        return None


def process_camera(cam_name, tf_buf, output_dir):
    """处理单台相机."""
    rospy.loginfo("=== %s ===", cam_name)
    topics = CAMERAS[cam_name]
    bridge = CvBridge()

    try:
        color_msg = rospy.wait_for_message(topics["color"], Image, timeout=12.0)
        info_msg  = rospy.wait_for_message(topics["info"],  CameraInfo, timeout=6.0)
    except rospy.ROSException:
        return {"camera": cam_name, "error": "capture_timeout"}

    try:
        cv_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    except Exception as e:
        return {"camera": cam_name, "error": str(e)}

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    K = np.array(info_msg.K).reshape(3, 3)
    D = np.array(info_msg.D) if info_msg.D is not None else np.zeros(4)
    optical_frame = info_msg.header.frame_id

    if not optical_frame.endswith("_color_optical_frame"):
        rospy.logwarn("%s: optical_frame=%s (expected *_color_optical_frame)", cam_name, optical_frame)

    results = {"camera": cam_name, "optical_frame": optical_frame,
               "K": K.tolist(), "D": D.tolist(), "solutions": []}

    # ChArUco faces
    for fk in ["front", "left", "back"]:
        obj_pts, img_pts = detect_charuco_face(gray, K, D, fk)
        if obj_pts is None:
            continue
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            ff = CHARUCO_FACES[fk]["face_frame"]
            rvec_ct, tvec_ct = rvec_tvec_to_cam_target(rvec, tvec, ff, tf_buf)
            results["solutions"].append({
                "panel": fk, "type": "charuco",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                "rvec_camera_target": rvec_ct,
                "tvec_camera_target": tvec_ct,
                **stats,
            })

    # AprilTag right
    obj_pts, img_pts = detect_apriltag_face(gray, "right")
    if obj_pts is not None:
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            rvec_ct, tvec_ct = rvec_tvec_to_cam_target(rvec, tvec, APRILTAG_RIGHT["face_frame"], tf_buf)
            results["solutions"].append({
                "panel": "right", "type": "apriltag",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                "rvec_camera_target": rvec_ct,
                "tvec_camera_target": tvec_ct,
                **stats,
            })

    # AprilTag top
    obj_pts, img_pts = detect_apriltag_face(gray, "top")
    if obj_pts is not None:
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            rvec_ct, tvec_ct = rvec_tvec_to_cam_target(rvec, tvec, APRILTAG_TOP["face_frame"], tf_buf)
            results["solutions"].append({
                "panel": "top", "type": "apriltag",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                "rvec_camera_target": rvec_ct,
                "tvec_camera_target": tvec_ct,
                **stats,
            })

    # Gazebo 真值对比
    truth = gazebo_truth(tf_buf, optical_frame)
    results["gazebo_truth"] = truth
    if truth and results["solutions"]:
        best = min(results["solutions"], key=lambda s: s.get("rmse_px", 99))
        if best.get("tvec_camera_target"):
            t_sol = np.array(best["tvec_camera_target"])
            t_truth = np.array([truth["tx"], truth["ty"], truth["tz"]])
            results["translation_error_m"] = float(np.linalg.norm(t_sol - t_truth))
            # 旋转误差
            from tf.transformations import quaternion_matrix
            if best.get("rvec_camera_target"):
                R_sol, _ = cv2.Rodrigues(np.array(best["rvec_camera_target"]))
                T_truth = quaternion_matrix([truth["qx"], truth["qy"], truth["qz"], truth["qw"]])
                R_diff = R_sol.T @ T_truth[:3, :3]
                trace = np.trace(R_diff)
                angle = math.acos(max(-1.0, min(1.0, (trace - 1) / 2.0)))
                results["rotation_error_deg"] = round(float(angle * 180 / math.pi), 4)

    # 保存
    cv2.imwrite(os.path.join(output_dir, f"{cam_name}.png"), cv_img)
    with open(os.path.join(output_dir, f"{cam_name}_result.json"), "w") as f:
        json.dump(results, f, indent=2, default=str)

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/calibration_target")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("calibrate_three_fixed_cameras", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    os.makedirs(args.output, exist_ok=True)

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.sleep(1.5)

    all_results = {"opencv_capability": aruco_compat.get_opencv_info()}
    for cam_name in sorted(CAMERAS.keys()):
        all_results[cam_name] = process_camera(cam_name, tf_buf, args.output)

    any_sol = any(len(r.get("solutions", [])) > 0 for r in all_results.values()
                  if isinstance(r, dict))
    all_results["any_solution"] = any_sol

    import yaml as _yaml
    with open(os.path.join(args.output, "initial_extrinsics.yaml"), "w") as f:
        _yaml.dump(all_results, f, default_flow_style=False)

    reproj = {}
    for cn, cr in all_results.items():
        if isinstance(cr, dict) and "solutions" in cr:
            reproj[cn] = [{"panel": s["panel"], "rmse_px": s.get("rmse_px", -1),
                           "inliers": s.get("inliers", 0), "pass": s.get("pass", False)}
                          for s in cr["solutions"]]
    with open(os.path.join(args.output, "reprojection_report.json"), "w") as f:
        json.dump(reproj, f, indent=2)

    truth_d = {cn: cr.get("gazebo_truth") for cn, cr in all_results.items()
               if isinstance(cr, dict)}
    with open(os.path.join(args.output, "gazebo_truth_comparison.json"), "w") as f:
        json.dump(truth_d, f, indent=2)

    rospy.loginfo("Calibration done. Solutions: %s", any_sol)


if __name__ == "__main__":
    main()

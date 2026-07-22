#!/usr/bin/env python3
"""
V2 三固定相机初始外参标定 (P1 修复版).

P1 修复:
- 读取 CameraInfo D (畸变系数) + header.frame_id (optical frame)
- ChArUco: 自定义 ID board (100-123, 200-214, 300-323) + 手动匹配
- AprilTag Right: 2×2 真实网格位置 (85mm 间距)
- PnP: 单方形用 IPPE_SQUARE, 多点用 IPPE/EPNP + RefineLM
- TF: T_camera_target = T_camera_face × inv(T_target_face)
- Gazebo 真值: optical_frame ← calibration_target_frame
"""
import sys, os, json, time, math, yaml
import cv2, numpy as np
import rospy, rospkg, tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco
from geometry_msgs.msg import TransformStamped

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

# === ChArUco 面板定义 ===
CHARUCO_PANELS = {
    "front": {
        "board": None, "id_start": 100, "sx": 8, "sy": 6,
        "sq_m": 0.027, "mk_m": 0.020, "dict_id": aruco.DICT_5X5_1000,
        "face_frame": "calibration_target_front_frame",
        "panel_normal": np.array([1.0, 0.0, 0.0]),
    },
    "left": {
        "board": None, "id_start": 200, "sx": 6, "sy": 5,
        "sq_m": 0.022, "mk_m": 0.016, "dict_id": aruco.DICT_5X5_1000,
        "face_frame": "calibration_target_left_frame",
        "panel_normal": np.array([0.0, 1.0, 0.0]),
    },
    "back": {
        "board": None, "id_start": 300, "sx": 8, "sy": 6,
        "sq_m": 0.027, "mk_m": 0.020, "dict_id": aruco.DICT_5X5_1000,
        "face_frame": "calibration_target_back_frame",
        "panel_normal": np.array([-1.0, 0.0, 0.0]),
    },
}

# AprilTag 定义
APRILTAG_RIGHT = {
    "tag_size": 0.07, "tag_gap": 0.015, "center_dist": 0.085,
    "tag_ids": [4, 5, 6, 7],
    "positions": {
        4: (-0.0425,  0.0425, 0.0),   # TL
        5: ( 0.0425,  0.0425, 0.0),   # TR
        6: (-0.0425, -0.0425, 0.0),   # BL
        7: ( 0.0425, -0.0425, 0.0),   # BR
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
MIN_INLIERS = 10
MAX_REPROJ = 2.0

# 初始化 ChArUco boards
for k, p in CHARUCO_PANELS.items():
    dict_obj = aruco.getPredefinedDictionary(p["dict_id"])
    p["board"] = aruco.CharucoBoard_create(p["sx"], p["sy"], p["sq_m"], p["mk_m"], dict_obj)
    p["expected_ids"] = set(range(p["id_start"], p["id_start"] + p["sx"] * p["sy"]))


class CameraCalibrator:
    def __init__(self, output_dir):
        self.bridge = CvBridge()
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        rospy.sleep(1.5)

    def capture(self, cam_name):
        topics = CAMERAS[cam_name]
        try:
            color_msg = rospy.wait_for_message(topics["color"], Image, timeout=10.0)
            info_msg  = rospy.wait_for_message(topics["info"],  CameraInfo, timeout=10.0)
        except rospy.ROSException:
            return None, None, None, None

        cv_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        K = np.array(info_msg.K).reshape(3, 3)
        D = np.array(info_msg.D) if info_msg.D else np.zeros(4)
        optical_frame = info_msg.header.frame_id
        return cv_img, K, D, optical_frame

    def detect_charuco(self, gray, panel_key):
        """检测单个 ChArUco 面板."""
        panel = CHARUCO_PANELS[panel_key]
        board = panel["board"]
        dict_obj = board.dictionary

        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejected = aruco.detectMarkers(gray, dict_obj, parameters=params)

        if ids is None:
            return None, None

        ids_flat = [int(i) for i in ids.flatten()]
        matched = [(i, ids_flat[i]) for i in range(len(ids_flat))
                   if ids_flat[i] in panel["expected_ids"]]

        if len(matched) < 2:
            return None, None

        matched_idx = [m[0] for m in matched]
        matched_ids_arr = np.array([[ids_flat[i]] for i in matched_idx], dtype=np.int32)
        matched_corners = tuple([corners[i] for i in matched_idx])

        # ChArUco corner interpolation
        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            matched_corners, matched_ids_arr, gray, board)
        if charuco_ids is None or len(charuco_ids) < 4:
            return None, None

        # 3D 点: 直接用 board 的 chessboard corners (objPoints)
        board_pts = board.chessboardCorners  # shape: (48, 1, 3) for 8×6
        charuco_ids_flat = [int(i) for i in charuco_ids.flatten()]
        obj_pts = np.array([board_pts[i][0] for i in charuco_ids_flat], dtype=np.float32)
        img_pts = charuco_corners.reshape(-1, 2).astype(np.float32)

        return obj_pts, img_pts

    def detect_apriltag_right(self, gray):
        """检测 Right AprilTag 2×2 grid."""
        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        detector = aruco.ArucoDetector(tag_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is None:
            return None, None

        ids_flat = [int(i) for i in ids.flatten()]
        obj_pts_list = []
        img_pts_list = []

        for i, tid in enumerate(ids_flat):
            if tid not in APRILTAG_RIGHT["tag_ids"]:
                continue

            pos = APRILTAG_RIGHT["positions"][tid]
            half = APRILTAG_RIGHT["tag_size"] / 2.0
            # Tag 四角 (centered at panel origin, Z=0, counter-clockwise from top-left)
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

    def detect_apriltag_top(self, gray):
        """检测 Top AprilTag single."""
        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        detector = aruco.ArucoDetector(tag_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is None:
            return None, None

        ids_flat = [int(i) for i in ids.flatten()]
        if 8 not in ids_flat:
            return None, None

        idx = ids_flat.index(8)
        half = APRILTAG_TOP["tag_size"] / 2.0
        tag_obj = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        return tag_obj, corners[idx][0].astype(np.float32)

    def solve_pnp(self, obj_pts, img_pts, K, D):
        """PnP 求解: 多点用 IPPE/EPNP, 单 tag 用 IPPE_SQUARE."""
        n_pts = len(obj_pts)
        if n_pts < 4:
            return None, None, None

        if n_pts == 4:
            flag = cv2.SOLVEPNP_IPPE_SQUARE
        else:
            flag = cv2.SOLVEPNP_IPPE

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts, K, D,
            reprojectionError=RANSAC_THRESH,
            flags=flag,
        )

        if not success or inliers is None or len(inliers) < MIN_INLIERS:
            return None, None, None

        # Refine with LM
        obj_in = obj_pts[inliers.flatten()]
        img_in = img_pts[inliers.flatten()]
        rvec2, tvec2 = cv2.solvePnPRefineLM(obj_in, img_in, K, D, rvec, tvec)

        # 重投影误差
        projected, _ = cv2.projectPoints(obj_in, rvec2, tvec2, K, D)
        errors = np.linalg.norm(img_in - projected.reshape(-1, 2), axis=1)

        return rvec2, tvec2, {"inliers": len(inliers), "n_pts": n_pts,
                               "rmse_px": float(np.sqrt(np.mean(errors**2))),
                               "max_error_px": float(np.max(errors)),
                               "pass": np.sqrt(np.mean(errors**2)) <= MAX_REPROJ}

    def get_target_face_tf(self, face_frame):
        """从 TF 获取 calibration_target_frame → face_frame 的变换."""
        try:
            ts = self.tf_buf.lookup_transform(
                "calibration_target_frame", face_frame,
                rospy.Time(0), rospy.Duration(5.0))
            t = ts.transform.translation
            q = ts.transform.rotation
            return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
        except Exception as e:
            rospy.logwarn("TF %s→%s: %s", "calibration_target_frame", face_frame, e)
            return None, None

    def get_gazebo_truth(self, optical_frame):
        """Gazebo 真值: optical_frame → calibration_target_frame."""
        try:
            ts = self.tf_buf.lookup_transform(
                optical_frame, "calibration_target_frame",
                rospy.Time(0), rospy.Duration(5.0))
            t = ts.transform.translation
            q = ts.transform.rotation
            return {"tx": t.x, "ty": t.y, "tz": t.z,
                    "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        except Exception as e:
            return None

    def rvec_tvec_to_cam_target(self, rvec, tvec, face_frame):
        """将 face-local PnP 结果转换为 camera→target 位姿."""
        R_cam_face, _ = cv2.Rodrigues(rvec)
        T_cam_face = np.eye(4)
        T_cam_face[:3, :3] = R_cam_face
        T_cam_face[:3, 3] = tvec.flatten()

        face_pos, face_quat = self.get_target_face_tf(face_frame)
        if face_pos is None:
            return None, None

        from tf.transformations import quaternion_matrix
        T_target_face = quaternion_matrix([
            face_quat[0], face_quat[1], face_quat[2], face_quat[3]])
        T_target_face[:3, 3] = face_pos

        # T_camera_target = T_camera_face × inv(T_target_face)
        T_camera_target = T_cam_face @ np.linalg.inv(T_target_face)

        rvec_ct = cv2.Rodrigues(T_camera_target[:3, :3])[0]
        tvec_ct = T_camera_target[:3, 3]
        return rvec_ct.flatten().tolist(), tvec_ct.flatten().tolist()

    def process_camera(self, cam_name):
        """处理单台相机."""
        rospy.loginfo("=== %s ===", cam_name)
        cv_img, K, D, optical_frame = self.capture(cam_name)

        if cv_img is None:
            return {"camera": cam_name, "error": "capture_timeout",
                    "optical_frame": optical_frame}

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        results = {"camera": cam_name, "optical_frame": optical_frame,
                   "K": K.tolist(), "D": D.tolist(), "solutions": []}

        # ChArUco: front, left, back
        for face_key in ["front", "left", "back"]:
            obj_pts, img_pts = self.detect_charuco(gray, face_key)
            if obj_pts is None:
                continue

            rvec, tvec, stats = self.solve_pnp(obj_pts, img_pts, K, D)
            if rvec is not None:
                face_frame = CHARUCO_PANELS[face_key]["face_frame"]
                rvec_ct, tvec_ct = self.rvec_tvec_to_cam_target(rvec, tvec, face_frame)
                results["solutions"].append({
                    "panel": face_key, "type": "charuco",
                    "rvec_camera_face": rvec.flatten().tolist(),
                    "tvec_camera_face": tvec.flatten().tolist(),
                    "rvec_camera_target": rvec_ct,
                    "tvec_camera_target": tvec_ct,
                    **stats,
                })

        # AprilTag: right
        obj_pts, img_pts = self.detect_apriltag_right(gray)
        if obj_pts is not None:
            rvec, tvec, stats = self.solve_pnp(obj_pts, img_pts, K, D)
            if rvec is not None:
                face_frame = APRILTAG_RIGHT["face_frame"]
                rvec_ct, tvec_ct = self.rvec_tvec_to_cam_target(rvec, tvec, face_frame)
                results["solutions"].append({
                    "panel": "right", "type": "apriltag",
                    "rvec_camera_face": rvec.flatten().tolist(),
                    "tvec_camera_face": tvec.flatten().tolist(),
                    "rvec_camera_target": rvec_ct,
                    "tvec_camera_target": tvec_ct,
                    **stats,
                })

        # AprilTag: top
        obj_pts, img_pts = self.detect_apriltag_top(gray)
        if obj_pts is not None:
            rvec, tvec, stats = self.solve_pnp(obj_pts, img_pts, K, D)
            if rvec is not None:
                face_frame = APRILTAG_TOP["face_frame"]
                rvec_ct, tvec_ct = self.rvec_tvec_to_cam_target(rvec, tvec, face_frame)
                results["solutions"].append({
                    "panel": "top", "type": "apriltag",
                    "rvec_camera_face": rvec.flatten().tolist(),
                    "tvec_camera_face": tvec.flatten().tolist(),
                    "rvec_camera_target": rvec_ct,
                    "tvec_camera_target": tvec_ct,
                    **stats,
                })

        # Gazebo truth comparison
        truth = self.get_gazebo_truth(optical_frame)
        results["gazebo_truth"] = truth
        if truth and results["solutions"]:
            best = min(results["solutions"], key=lambda s: s.get("rmse_px", 99))
            if best.get("tvec_camera_target"):
                t_sol = np.array(best["tvec_camera_target"])
                t_truth = np.array([truth["tx"], truth["ty"], truth["tz"]])
                results["translation_error_m"] = float(np.linalg.norm(t_sol - t_truth))

        # 保存图
        img_path = os.path.join(self.output_dir, f"{cam_name}.png")
        cv2.imwrite(img_path, cv_img)
        obs_path = os.path.join(self.output_dir, f"{cam_name}_result.json")
        with open(obs_path, "w") as f:
            json.dump(results, f, indent=2, default=str)

        return results


def main():
    rospy.init_node("calibrate_three_fixed_cameras", anonymous=True, log_level=rospy.WARN)

    output_dir = "artifacts/calibration_target_v1"
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]

    calibrator = CameraCalibrator(output_dir)
    all_results = {}

    for cam_name in sorted(CAMERAS.keys()):
        result = calibrator.process_camera(cam_name)
        all_results[cam_name] = result

    # 汇总
    any_solution = any(len(r.get("solutions", [])) > 0 for r in all_results.values())
    summary = {"timestamp": time.time(), "results": all_results, "any_solution": any_solution}

    with open(os.path.join(output_dir, "initial_extrinsics.yaml"), "w") as f:
        yaml.dump(summary, f, default_flow_style=False)

    # 重投影报告
    reproj = {}
    for cn, cr in all_results.items():
        reproj[cn] = [{"panel": s["panel"], "rmse_px": s.get("rmse_px", -1),
                        "inliers": s.get("inliers", 0), "pass": s.get("pass", False)}
                      for s in cr.get("solutions", [])]
    with open(os.path.join(output_dir, "reprojection_report.json"), "w") as f:
        json.dump(reproj, f, indent=2, default=str)

    truth_data = {cn: cr.get("gazebo_truth") for cn, cr in all_results.items()}
    with open(os.path.join(output_dir, "gazebo_truth_comparison.json"), "w") as f:
        json.dump(truth_data, f, indent=2, default=str)

    rospy.loginfo("Calibration complete. Solutions: %s", any_solution)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V1 三固定相机初始外参标定 — 骨架 (Stage 1).

从标定目标的多面 ChArUco/AprilTag 建立 2D-3D 对应，solvePnPRansac 求解
每台相机相对 calibration_target_frame 的位姿，与 Gazebo 真值 TF 对比验证。

仅建立骨架：
- 不覆盖正式 TF
- 不推进 bundle adjustment
- 不更改现有相机配置

用法:
  rosrun cr5_spray_sim calibrate_three_fixed_cameras.py \
    --config config/calibration_target_v1.yaml \
    --output artifacts/calibration_target_v1/

输出:
  artifacts/calibration_target_v1/
  ├── observations/          # 每台相机原始检测
  ├── initial_extrinsics.yaml
  ├── reprojection_report.json
  └── gazebo_truth_comparison.json
"""
import sys
import os
import json
import math
import time
import yaml
import numpy as np
import cv2
import rospy
import rospkg
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco
from geometry_msgs.msg import TransformStamped


CAMERAS = {
    "cam_front_left": {
        "color": "/cam_front_left/camera/color/image_raw",
        "info":  "/cam_front_left/camera/color/camera_info",
        "frame": "cam_front_left_link",
    },
    "cam_front_right": {
        "color": "/cam_front_right/camera/color/image_raw",
        "info":  "/cam_front_right/camera/color/camera_info",
        "frame": "cam_front_right_link",
    },
    "cam_rear": {
        "color": "/cam_rear/camera/color/image_raw",
        "info":  "/cam_rear/camera/color/camera_info",
        "frame": "cam_rear_link",
    },
}

# 标定板物理参数 (引用 calibration_target_v1.yaml)
CHARUCO_FRONT = {
    "squares_x": 8, "squares_y": 6,
    "square_length": 0.027, "marker_length": 0.020,
    "dict": aruco.DICT_5X5_1000,
    "marker_ids": list(range(100, 124)),
    # 面板中心在 calibration_target_frame 中的位姿
    "panel_center_xyz": [0.1515, 0.0, 0.0],  # 正面板外表面
    "panel_normal": [1.0, 0.0, 0.0],          # +X
}

CHARUCO_LEFT = {
    "squares_x": 6, "squares_y": 5,
    "square_length": 0.022, "marker_length": 0.016,
    "dict": aruco.DICT_5X5_1000,
    "marker_ids": list(range(200, 215)),
    "panel_center_xyz": [0.0, 0.1115, 0.0],
    "panel_normal": [0.0, 1.0, 0.0],
}

APRILTAG_RIGHT = {
    "tag_size": 0.07, "tag_gap": 0.015,
    "dict": aruco.DICT_APRILTAG_36h11,
    "tag_ids": [4, 5, 6, 7],
    "layout": [[4, 5], [6, 7]],  # 2x2 grid
    "panel_center_xyz": [0.0, -0.1115, 0.0],
    "panel_normal": [0.0, -1.0, 0.0],
}

APRILTAG_TOP = {
    "tag_size": 0.12,
    "dict": aruco.DICT_APRILTAG_36h11,
    "tag_ids": [8],
    "panel_center_xyz": [0.0, 0.0, 0.0915],
    "panel_normal": [0.0, 0.0, 1.0],
}

# PnP 参数
RANSAC_THRESH = 2.0     # px
MIN_INLIERS = 10
TARGET_REPROJ_ERROR = 1.0  # px
MAX_REPROJ_ERROR = 2.0     # px — 超过标记为失败


class CameraCalibrator:
    def __init__(self, config_path, output_dir):
        self.bridge = CvBridge()
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "observations"), exist_ok=True)

        # Load YAML config
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        # TF buffer for Gazebo truth
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        rospy.sleep(1.0)

        # ChArUco boards for front and left
        self.front_board = aruco.CharucoBoard(
            (CHARUCO_FRONT["squares_x"], CHARUCO_FRONT["squares_y"]),
            CHARUCO_FRONT["square_length"],
            CHARUCO_FRONT["marker_length"],
            aruco.getPredefinedDictionary(CHARUCO_FRONT["dict"]),
        )
        self.left_board = aruco.CharucoBoard(
            (CHARUCO_LEFT["squares_x"], CHARUCO_LEFT["squares_y"]),
            CHARUCO_LEFT["square_length"],
            CHARUCO_LEFT["marker_length"],
            aruco.getPredefinedDictionary(CHARUCO_LEFT["dict"]),
        )

        # AprilTag detectors
        self.right_tag_dict = aruco.getPredefinedDictionary(APRILTAG_RIGHT["dict"])
        self.top_tag_dict = aruco.getPredefinedDictionary(APRILTAG_TOP["dict"])
        det_params = aruco.DetectorParameters()
        self.right_detector = aruco.ArucoDetector(self.right_tag_dict, det_params)
        self.top_detector = aruco.ArucoDetector(self.top_tag_dict, det_params)

        self.all_results = {}

    def get_board_object_points(self, board_type, marker_ids):
        """获取标定板上 Markers 对应的 3D 物体点."""
        if board_type == "front":
            board = self.front_board
            ids_to_use = [i for i in marker_ids if i in CHARUCO_FRONT["marker_ids"]]
        else:
            board = self.left_board
            ids_to_use = [i for i in marker_ids if i in CHARUCO_LEFT["marker_ids"]]

        if not ids_to_use:
            return None, None

        # 获取每个 marker 的四角 3D 坐标
        obj_points = []
        for mid in ids_to_use:
            marker_corners = board.getObjPoints()[mid - min(board.ids.flatten())]
            if marker_corners is not None:
                obj_points.extend(marker_corners.tolist())

        return np.array(obj_points, dtype=np.float32), ids_to_use

    def get_apriltag_object_points(self, panel_type, tag_ids):
        """获取 AprilTag 的 3D 物体点 (tag 四角)."""
        config = APRILTAG_RIGHT if panel_type == "right" else APRILTAG_TOP
        tag_size = config["tag_size"]
        half = tag_size / 2.0

        obj_points = []
        valid_ids = []

        for tid in tag_ids:
            if tid not in config["tag_ids"]:
                continue

            if panel_type == "right":
                # 计算在 2x2 网格中的位置
                layout = config["layout"]
                for r, row in enumerate(layout):
                    for c, val in enumerate(row):
                        if val == tid:
                            cx = (c - 0.5) * (tag_size + config["tag_gap"])
                            cy = (0.5 - r) * (tag_size + config["tag_gap"])
                            obj_points.append([cx - half, cy + half, 0.0])
                            obj_points.append([cx + half, cy + half, 0.0])
                            obj_points.append([cx + half, cy - half, 0.0])
                            obj_points.append([cx - half, cy - half, 0.0])
                            valid_ids.append(tid)
            else:  # top: single tag
                obj_points.append([-half, half, 0.0])
                obj_points.append([half, half, 0.0])
                obj_points.append([half, -half, 0.0])
                obj_points.append([-half, -half, 0.0])
                valid_ids.append(tid)

        return np.array(obj_points, dtype=np.float32), valid_ids

    def detect_and_solve(self, cam_name, cv_img, K):
        """对单帧执行检测和 PnP 求解."""
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        result = {
            "camera": cam_name,
            "stamp": rospy.get_time(),
            "solutions": [],
        }

        # === ChArUco detection for front and left ===
        aruco_dict = aruco.getPredefinedDictionary(CHARUCO_FRONT["dict"])
        aruco_params = aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            ids_flat = [int(i) for i in ids.flatten()]

            # 尝试正面 ChArUco
            front_ids = [i for i in ids_flat if i in CHARUCO_FRONT["marker_ids"]]
            if front_ids:
                self._solve_charuco_pnp(
                    result, gray, corners, ids, "front", front_ids, K,
                    CHARUCO_FRONT["panel_center_xyz"], CHARUCO_FRONT["panel_normal"])

            # 尝试左面 ChArUco
            left_ids = [i for i in ids_flat if i in CHARUCO_LEFT["marker_ids"]]
            if left_ids:
                self._solve_charuco_pnp(
                    result, gray, corners, ids, "left", left_ids, K,
                    CHARUCO_LEFT["panel_center_xyz"], CHARUCO_LEFT["panel_normal"])

        # === AprilTag detection for right and top ===
        right_corners, right_ids, _ = self.right_detector.detectMarkers(gray)
        if right_ids is not None:
            rids = [int(i) for i in right_ids.flatten() if i in APRILTAG_RIGHT["tag_ids"]]
            if rids:
                self._solve_apriltag_pnp(
                    result, right_corners, right_ids, "right", rids, K,
                    APRILTAG_RIGHT["panel_center_xyz"], APRILTAG_RIGHT["panel_normal"])

        top_corners, top_ids, _ = self.top_detector.detectMarkers(gray)
        if top_ids is not None:
            tids = [int(i) for i in top_ids.flatten() if i in APRILTAG_TOP["tag_ids"]]
            if tids:
                self._solve_apriltag_pnp(
                    result, top_corners, top_ids, "top", tids, K,
                    APRILTAG_TOP["panel_center_xyz"], APRILTAG_TOP["panel_normal"])

        return result

    def _solve_charuco_pnp(self, result, gray, all_corners, all_ids,
                           panel_name, marker_ids, K, panel_center, panel_normal):
        """ChArUco PnP 求解."""
        # 只取对应面的 markers
        valid_indices = []
        valid_marker_ids = []
        for i, mid in enumerate(all_ids.flatten()):
            if int(mid) in marker_ids:
                valid_indices.append(i)
                valid_marker_ids.append(int(mid))

        if len(valid_indices) < 2:
            return

        valid_corners = [all_corners[i] for i in valid_indices]

        # Interpolate ChArUco corners
        if panel_name == "front":
            board = self.front_board
        else:
            board = self.left_board

        # 构建针对面的 board points
        # 简单近似：使用 marker corners 直接匹配
        obj_pts_list = []
        img_pts_list = []

        for idx, mid in zip(valid_indices, valid_marker_ids):
            marker_obj = board.getObjPoints()[mid - min(board.ids.flatten())]
            marker_img = all_corners[idx][0]
            obj_pts_list.append(marker_obj)
            img_pts_list.append(marker_img)

        if not obj_pts_list:
            return

        obj_pts = np.vstack(obj_pts_list).astype(np.float32)
        img_pts = np.vstack(img_pts_list).astype(np.float32)

        if len(obj_pts) < 4:
            return

        # PnP without panel offset (solve directly)
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        try:
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_pts, img_pts, K, dist_coeffs,
                reprojectionError=RANSAC_THRESH,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except Exception as e:
            rospy.logwarn("%s PnP failed: %s", panel_name, e)
            return

        if not success or inliers is None:
            return

        # 重投影误差
        obj_pts_inlier = obj_pts[inliers.flatten()]
        img_pts_inlier = img_pts[inliers.flatten()]
        projected, _ = cv2.projectPoints(obj_pts_inlier, rvec, tvec, K, dist_coeffs)
        reproj_errors = np.linalg.norm(img_pts_inlier - projected.reshape(-1, 2), axis=1)
        mean_error = np.mean(reproj_errors)

        solution = {
            "panel": panel_name,
            "type": "charuco",
            "marker_ids": valid_marker_ids,
            "inliers": len(inliers),
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
            "mean_reprojection_error_px": round(float(mean_error), 4),
            "pass": mean_error <= MAX_REPROJ_ERROR and len(inliers) >= MIN_INLIERS,
        }
        result["solutions"].append(solution)

        rospy.loginfo("%s: %s PnP — inliers=%d error=%.3fpx %s",
                      result["camera"], panel_name,
                      len(inliers), mean_error,
                      "PASS" if solution["pass"] else "FAIL")

    def _solve_apriltag_pnp(self, result, tag_corners, tag_ids,
                            panel_name, matched_ids, K, panel_center, panel_normal):
        """AprilTag PnP 求解."""
        obj_pts_list = []
        img_pts_list = []

        for i, (corner, tid) in enumerate(zip(tag_corners, tag_ids.flatten())):
            if int(tid) not in matched_ids:
                continue

            half = (APRILTAG_RIGHT["tag_size"] if panel_name == "right"
                    else APRILTAG_TOP["tag_size"]) / 2.0

            # Tag corners in object space (planar, Z=0)
            tag_obj = np.array([
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ], dtype=np.float32)

            obj_pts_list.append(tag_obj)
            img_pts_list.append(corner[0])

        if not obj_pts_list:
            return

        obj_pts = np.vstack(obj_pts_list).astype(np.float32)
        img_pts = np.vstack(img_pts_list).astype(np.float32)

        if len(obj_pts) < 4:
            return

        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        try:
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_pts, img_pts, K, dist_coeffs,
                reprojectionError=RANSAC_THRESH,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except Exception as e:
            rospy.logwarn("%s AprilTag PnP failed: %s", panel_name, e)
            return

        if not success or inliers is None:
            return

        obj_pts_inlier = obj_pts[inliers.flatten()]
        img_pts_inlier = img_pts[inliers.flatten()]
        projected, _ = cv2.projectPoints(obj_pts_inlier, rvec, tvec, K, dist_coeffs)
        reproj_errors = np.linalg.norm(img_pts_inlier - projected.reshape(-1, 2), axis=1)
        mean_error = np.mean(reproj_errors)

        solution = {
            "panel": panel_name,
            "type": "apriltag",
            "tag_ids": matched_ids,
            "inliers": len(inliers),
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
            "mean_reprojection_error_px": round(float(mean_error), 4),
            "pass": mean_error <= MAX_REPROJ_ERROR and len(inliers) >= MIN_INLIERS,
        }
        result["solutions"].append(solution)

    def get_gazebo_truth(self, cam_name):
        """获取 Gazebo 真值: 相机 frame → calibration_target_frame."""
        try:
            ts = self.tf_buf.lookup_transform(
                "calibration_target_frame",
                CAMERAS[cam_name]["frame"],
                rospy.Time(0),
                rospy.Duration(5.0),
            )
            t = ts.transform.translation
            q = ts.transform.rotation
            return {
                "tx": t.x, "ty": t.y, "tz": t.z,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
            }
        except Exception as e:
            rospy.logwarn("TF truth for %s: %s", cam_name, e)
            return None

    def compare_with_truth(self, tvec, rvec, truth):
        """比较 PnP 结果与 Gazebo 真值."""
        if truth is None:
            return None

        t_diff = np.linalg.norm([
            tvec[0] - truth["tx"],
            tvec[1] - truth["ty"],
            tvec[2] - truth["tz"],
        ])

        return {
            "translation_diff_m": round(float(t_diff), 6),
        }


def main():
    rospy.init_node("calibrate_three_fixed_cameras", anonymous=True,
                    log_level=rospy.WARN)

    config_path = None
    output_dir = "artifacts/calibration_target_v1"

    for i, arg in enumerate(sys.argv):
        if arg == "--config" and i + 1 < len(sys.argv):
            config_path = sys.argv[i + 1]
        if arg == "--output" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]

    if config_path is None:
        try:
            rp = rospkg.RosPack()
            config_path = os.path.join(
                rp.get_path("cr5_spray_sim"),
                "config", "calibration_target_v1.yaml")
        except Exception:
            rospy.logerr("Cannot find config file")
            sys.exit(1)

    calibrator = CameraCalibrator(config_path, output_dir)
    all_results = {}
    truth_data = {}

    for cam_name, topics in sorted(CAMERAS.items()):
        rospy.loginfo("=== Processing %s ===", cam_name)

        # Capture
        try:
            color_msg = rospy.wait_for_message(topics["color"], Image, timeout=10.0)
            info_msg = rospy.wait_for_message(topics["info"], CameraInfo, timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("%s: capture timeout", cam_name)
            all_results[cam_name] = {"error": "capture_timeout"}
            continue

        try:
            cv_img = calibrator.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("%s: bridge failed: %s", cam_name, e)
            all_results[cam_name] = {"error": str(e)}
            continue

        K = np.array(info_msg.K).reshape(3, 3)

        # Detect + PnP
        result = calibrator.detect_and_solve(cam_name, cv_img, K)
        all_results[cam_name] = result

        # Gazebo truth
        truth = calibrator.get_gazebo_truth(cam_name)
        truth_data[cam_name] = truth

        # Save observation
        obs_dir = os.path.join(output_dir, "observations")
        os.makedirs(obs_dir, exist_ok=True)
        obs_path = os.path.join(obs_dir, "{}.json".format(cam_name))
        with open(obs_path, "w") as f:
            json.dump(result, f, indent=2, default=str)

        # Save image sample
        img_path = os.path.join(obs_dir, "{}.png".format(cam_name))
        cv2.imwrite(img_path, cv_img)

    # 汇总
    summary = {
        "config": config_path,
        "timestamp": time.time(),
        "results": all_results,
        "gazebo_truth": truth_data,
        "any_solution": any(
            len(r.get("solutions", [])) > 0 for r in all_results.values()
        ),
    }

    # Save outputs
    with open(os.path.join(output_dir, "initial_extrinsics.yaml"), "w") as f:
        yaml.dump(summary, f, default_flow_style=False)

    reproj_report = {
        "target_reprojection_error_px": TARGET_REPROJ_ERROR,
        "max_reprojection_error_px": MAX_REPROJ_ERROR,
        "results": {},
    }
    for cam_name, result in all_results.items():
        cam_errors = []
        for sol in result.get("solutions", []):
            cam_errors.append({
                "panel": sol["panel"],
                "error_px": sol["mean_reprojection_error_px"],
                "inliers": sol["inliers"],
                "pass": sol["pass"],
            })
        reproj_report["results"][cam_name] = cam_errors

    with open(os.path.join(output_dir, "reprojection_report.json"), "w") as f:
        json.dump(reproj_report, f, indent=2, default=str)

    with open(os.path.join(output_dir, "gazebo_truth_comparison.json"), "w") as f:
        json.dump(truth_data, f, indent=2, default=str)

    rospy.loginfo("Calibration skeleton complete. Output: %s", output_dir)
    rospy.loginfo("Solutions found: %s",
                  summary["any_solution"])


if __name__ == "__main__":
    main()

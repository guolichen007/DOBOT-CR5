#!/usr/bin/env python3
"""
标定目标可见性定量诊断.

对每台相机和每个面板计算:
- 面板是否在相机前方 (视线点积)
- 面板法向与相机视线夹角
- 四角理论像素坐标
- 投影面积 (px²)
- 图像内面积比例
- 面板中心理论深度
- 实际检测 Marker/Tag 数量

根因分类:
  OUT_OF_FRAME        — 全部角点都在画面外
  BACK_FACING         — 面板法向背离相机
  TOO_SMALL           — 投影面积不足
  TOO_OBLIQUE         — 视线夹角过大
  PARTIALLY_VISIBLE   — 部分角点在画面内但不够
  RENDERED_BUT_NOT_DETECTED — 理论可见但未检测到
  DETECTED            — 检测成功

用法:
  rosrun cr5_spray_sim diagnose_calibration_visibility.py --output <dir>
"""
import sys
import os
import json
import math
import yaml
import argparse
import cv2
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco
from cr5_spray_perception import aruco_compat


# ── 面板定义 (从标定目标 + 感知脚本提取) ──
CHARUCO_FACES = {
    "front": {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 100,
              "face_frame": "calibration_target_front_frame",
              "board_size_m": (0.216, 0.162)},
    "left":  {"sx": 6, "sy": 5, "sq_m": 0.022, "mk_m": 0.016,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 200,
              "face_frame": "calibration_target_left_frame",
              "board_size_m": (0.132, 0.110)},
    "back":  {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 300,
              "face_frame": "calibration_target_back_frame",
              "board_size_m": (0.216, 0.162)},
}

APRILTAG_FACES = {
    "right": {"face_frame": "calibration_target_right_frame",
              "dict_id": aruco.DICT_APRILTAG_36h11,
              "tag_ids": {4, 5, 6, 7},
              "board_size_m": (0.100, 0.100)},
    "top":   {"face_frame": "calibration_target_top_frame",
              "dict_id": aruco.DICT_APRILTAG_36h11,
              "tag_ids": {8},
              "board_size_m": (0.120, 0.120)},
}

CAMERAS = {
    "cam_front_left": {
        "color": "/cam_front_left/camera/color/image_raw",
        "info":  "/cam_front_left/camera/color/camera_info",
    },
    "cam_front_right": {
        "color": "/cam_front_right/camera/color/image_raw",
        "info":  "/cam_front_right/camera/color/camera_info",
    },
    "cam_rear": {
        "color": "/cam_rear/camera/color/image_raw",
        "info":  "/cam_rear/camera/color/camera_info",
    },
}

# 阈值
MIN_PROJECTED_AREA_PX2 = 400    # 最小投影面积 (px²)
MAX_VIEW_ANGLE_DEG = 75.0       # 最大视线夹角
MIN_IN_FRAME_CORNERS = 2        # 最少在画面内的角点


def compute_face_3d_corners(face_frame, board_size_m, tf_buf):
    """查询面板的 3D 四角坐标 (world frame).

    face_frame 位于面板中心, X=右, Y=上, Z=外法向.
    """
    try:
        ts = tf_buf.lookup_transform(
            "world", face_frame, rospy.Time(0), rospy.Duration(3.0))
    except Exception:
        return None

    t = ts.transform.translation
    q = ts.transform.rotation
    from tf.transformations import quaternion_matrix
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [t.x, t.y, t.z]

    w, h = board_size_m
    # 面板四角 (X=水平, Y=垂直, 面板平面 Z=0)
    corners_local = np.array([
        [-w/2,  h/2, 0, 1],
        [ w/2,  h/2, 0, 1],
        [ w/2, -h/2, 0, 1],
        [-w/2, -h/2, 0, 1],
    ], dtype=float)
    corners_world = (T @ corners_local.T).T[:, :3]
    return corners_world, T


def project_points(points_3d, K, R_cam_world, t_cam_world):
    """将 3D 点投影到像素坐标."""
    pts_cam = (R_cam_world @ points_3d.T).T + t_cam_world.reshape(1, 3)
    # 检查是否在相机前方 (Z > 0)
    in_front = pts_cam[:, 2] > 0.01
    if not in_front.any():
        return None, in_front
    pts_img = (K @ pts_cam.T).T
    pts_img[:, :2] /= pts_img[:, 2:3]
    return pts_img[:, :2], in_front


def compute_camera_pose(tf_buf, cam_name):
    """获取相机的 world 位姿 (通过 optical_frame)."""
    optical_frame = "{}_color_optical_frame".format(cam_name)
    try:
        ts = tf_buf.lookup_transform(
            "world", optical_frame, rospy.Time(0), rospy.Duration(3.0))
    except Exception:
        return None
    t = ts.transform.translation
    q = ts.transform.rotation
    from tf.transformations import quaternion_matrix
    T = quaternion_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def detect_faces_on_image(gray, cam_name):
    """对图像运行 Charuco + AprilTag 检测."""
    detected = {}
    K_dummy = np.eye(3)

    for fk, fc in CHARUCO_FACES.items():
        board = aruco.CharucoBoard_create(fc["sx"], fc["sy"], fc["sq_m"],
                                          fc["mk_m"],
                                          aruco.getPredefinedDictionary(fc["dict_id"]))
        params = aruco_compat.detector_parameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejected = aruco_compat.detect_markers(
            gray, board.dictionary, params)

        if ids is None:
            detected[fk] = {"marker_ids": [], "corner_count": 0}
            continue

        ids_flat = [int(i) for i in ids.flatten()]
        idx_list, local_ids = aruco_compat.remap_custom_ids(
            ids_flat, fc["id_start"], board)

        cc_count = 0
        if len(idx_list) >= 2:
            local_corners = tuple(corners[i] for i in idx_list)
            cc, cids = aruco_compat.interpolate_charuco_corners(
                local_corners, local_ids, gray, board)
            if cids is not None:
                cc_count = len(cids)

        detected[fk] = {
            "marker_ids": [ids_flat[i] for i in idx_list],
            "corner_count": cc_count,
        }

    # AprilTag
    for fk, fc in APRILTAG_FACES.items():
        tag_dict = aruco.getPredefinedDictionary(fc["dict_id"])
        params = aruco_compat.detector_parameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejected = aruco_compat.detect_markers(
            gray, tag_dict, params)

        if ids is None:
            detected[fk] = {"tag_ids": [], "tag_count": 0}
            continue

        ids_flat = [int(i) for i in ids.flatten()]
        matched = [mid for mid in ids_flat if mid in fc["tag_ids"]]
        detected[fk] = {"tag_ids": matched, "tag_count": len(matched)}

    return detected


def classify_face(proj_corners, in_front, board_size_m, detected_info, is_charuco):
    """分类面板可见性."""
    if proj_corners is None or not in_front.any():
        return "BACK_FACING"

    img_w, img_h = 640, 480  # 会被覆盖
    in_frame = ((proj_corners[:, 0] >= -10) & (proj_corners[:, 0] < 650) &
                (proj_corners[:, 1] >= -10) & (proj_corners[:, 1] < 490))
    n_in_frame = int(in_frame.sum())

    if n_in_frame < MIN_IN_FRAME_CORNERS:
        return "OUT_OF_FRAME"

    # 投影面积
    if n_in_frame >= 3 and proj_corners is not None:
        from cv2 import contourArea
        area_px = abs(contourArea(proj_corners.astype(np.float32)))
        if area_px < MIN_PROJECTED_AREA_PX2:
            return "TOO_SMALL"

    if detected_info.get("corner_count", 0) >= 12:
        return "DETECTED"
    elif detected_info.get("tag_count", 0) >= 1:
        return "DETECTED"
    elif len(detected_info.get("marker_ids", [])) >= 2:
        return "DETECTED"
    else:
        return "RENDERED_BUT_NOT_DETECTED"


def draw_diagnostic_overlay(cv_img, cam_name, face_results, proj_data):
    """在图像上叠加理论投影和检测."""
    annotated = cv_img.copy()
    face_colors = {
        "front": (0, 255, 0),      # 绿
        "left":  (255, 0, 0),      # 蓝
        "back":  (0, 255, 255),    # 黄
        "right": (255, 0, 255),    # 紫
        "top":   (0, 165, 255),    # 橙
    }
    classification_colors = {
        "DETECTED": (0, 255, 0),
        "RENDERED_BUT_NOT_DETECTED": (0, 165, 255),
        "TOO_SMALL": (0, 255, 255),
        "OUT_OF_FRAME": (128, 128, 128),
        "BACK_FACING": (128, 128, 128),
        "TOO_OBLIQUE": (0, 255, 255),
    }

    for face_key, result in face_results.items():
        color = classification_colors.get(result.get("classification", ""),
                                          face_colors.get(face_key, (255, 255, 255)))
        # 绘制理论投影
        proj = proj_data.get(face_key, {}).get("projected_corners_px")
        if proj is not None and len(proj) > 0:
            pts = np.array(proj, dtype=np.int32)
            cv2.polylines(annotated, [pts], True, color, 2)
            label = "EXPECTED_{}:{}".format(face_key.upper(), result.get("classification", "?"))
            cv2.putText(annotated, label,
                       (max(0, pts[0][0]), max(15, pts[0][1] - 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        if result.get("classification") == "DETECTED":
            status = "DETECTED_{}".format(face_key.upper())
            cv2.putText(annotated, status, (10, 30 + 20 * list(face_results.keys()).index(face_key)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return annotated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="output artifact directory")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("diagnose_calibration_visibility", anonymous=True, log_level=rospy.WARN)
    bridge = CvBridge()

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.sleep(2.0)

    os.makedirs(args.output, exist_ok=True)

    all_results = {}

    for cam_name in sorted(CAMERAS.keys()):
        rospy.loginfo("=== %s ===", cam_name)
        topics = CAMERAS[cam_name]

        try:
            color_msg = rospy.wait_for_message(topics["color"], Image, timeout=12.0)
            info_msg = rospy.wait_for_message(topics["info"], CameraInfo, timeout=6.0)
        except rospy.ROSException:
            all_results[cam_name] = {"error": "capture_timeout"}
            continue

        cv_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        K = np.array(info_msg.K).reshape(3, 3)

        # 相机位姿
        T_world_cam = compute_camera_pose(tf_buf, cam_name)
        if T_world_cam is None:
            all_results[cam_name] = {"error": "no_camera_tf"}
            continue

        R_cam_world = T_world_cam[:3, :3].T  # world→cam 旋转
        t_cam_world = -R_cam_world @ T_world_cam[:3, 3]

        cam_result = {"camera": cam_name, "faces": {}, "image_shape": list(cv_img.shape)}
        proj_data = {}
        detection = detect_faces_on_image(gray, cam_name)

        # 处理 Charuco 面
        for fk, fc in CHARUCO_FACES.items():
            result = {"face": fk, "type": "charuco"}
            corners, T = compute_face_3d_corners(
                fc["face_frame"], fc["board_size_m"], tf_buf)

            if corners is None:
                result["classification"] = "TF_MISSING"
                cam_result["faces"][fk] = result
                continue

            proj, in_front = project_points(corners, K, R_cam_world, t_cam_world)
            if proj is not None:
                proj_data[fk] = {"projected_corners_px": proj.tolist()}
                result["projected_corners_px"] = proj.tolist()
                result["corners_in_front"] = int(in_front.sum())

            # 面板法向
            if T is not None:
                face_normal_world = T[:3, 2]
                cam_to_face = corners.mean(axis=0) - T_world_cam[:3, 3]
                cam_to_face_norm = np.linalg.norm(cam_to_face)
                if cam_to_face_norm > 0.001:
                    cam_to_face_dir = cam_to_face / cam_to_face_norm
                    cos_angle = float(np.dot(cam_to_face_dir, face_normal_world))
                    result["face_normal_dot_view"] = round(cos_angle, 4)
                    result["view_angle_deg"] = round(
                        math.degrees(math.acos(max(-1, min(1, abs(cos_angle))))), 1)
                    result["depth_m"] = round(cam_to_face_norm, 3)

            # 面积
            if proj is not None and in_front.sum() >= 3:
                area_px = abs(cv2.contourArea(proj.astype(np.float32)))
                result["projected_area_px2"] = round(area_px, 1)
                result["projected_area_pct"] = round(
                    area_px / (cv_img.shape[1] * cv_img.shape[0]) * 100, 2)

            det = detection.get(fk, {})
            result.update({k: det.get(k) for k in ("marker_ids", "corner_count")
                          if k in det})

            result["classification"] = classify_face(
                proj, in_front, fc["board_size_m"], det, is_charuco=True)
            cam_result["faces"][fk] = result

        # 处理 AprilTag 面
        for fk, fc in APRILTAG_FACES.items():
            result = {"face": fk, "type": "apriltag"}
            corners, T = compute_face_3d_corners(
                fc["face_frame"], fc["board_size_m"], tf_buf)

            if corners is None:
                result["classification"] = "TF_MISSING"
                cam_result["faces"][fk] = result
                continue

            proj, in_front = project_points(corners, K, R_cam_world, t_cam_world)
            if proj is not None:
                proj_data[fk] = {"projected_corners_px": proj.tolist()}
                result["projected_corners_px"] = proj.tolist()

            if T is not None:
                face_normal_world = T[:3, 2]
                cam_to_face = corners.mean(axis=0) - T_world_cam[:3, 3]
                cam_to_face_norm = np.linalg.norm(cam_to_face)
                if cam_to_face_norm > 0.001:
                    cam_to_face_dir = cam_to_face / cam_to_face_norm
                    cos_angle = float(np.dot(cam_to_face_dir, face_normal_world))
                    result["face_normal_dot_view"] = round(cos_angle, 4)
                    result["view_angle_deg"] = round(
                        math.degrees(math.acos(max(-1, min(1, abs(cos_angle))))), 1)
                    result["depth_m"] = round(cam_to_face_norm, 3)

            if proj is not None and in_front.sum() >= 3:
                area_px = abs(cv2.contourArea(proj.astype(np.float32)))
                result["projected_area_px2"] = round(area_px, 1)
                result["projected_area_pct"] = round(
                    area_px / (cv_img.shape[1] * cv_img.shape[0]) * 100, 2)

            det = detection.get(fk, {})
            result.update({k: det.get(k) for k in ("tag_ids", "tag_count") if k in det})

            result["classification"] = classify_face(
                proj, in_front, fc["board_size_m"], det, is_charuco=False)
            cam_result["faces"][fk] = result

        # 生成诊断图
        annotated = draw_diagnostic_overlay(cv_img, cam_name, cam_result["faces"], proj_data)
        cv2.imwrite(os.path.join(args.output, "{}_diagnostic.png".format(cam_name)), annotated)
        cv2.imwrite(os.path.join(args.output, "{}_original.png".format(cam_name)), cv_img)

        # 统计
        detected = [fk for fk, r in cam_result["faces"].items()
                   if r.get("classification") == "DETECTED"]
        cam_result["detected_faces"] = detected
        cam_result["pass"] = len(detected) >= 1
        all_results[cam_name] = cam_result

        rospy.loginfo("%s: detected=%s pass=%s", cam_name, detected, cam_result["pass"])

    # 汇总
    all_pass = all(r.get("pass", False) for r in all_results.values()
                   if isinstance(r, dict) and "pass" in r)
    all_results["all_cameras_visible"] = all_pass

    with open(os.path.join(args.output, "visibility_diagnosis.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)

    # 打印摘要表
    print("\nVisibility Diagnosis Summary:")
    print("Camera           Front      Left       Back       Right      Top        PASS")
    print("-" * 85)
    for cam_name in sorted(CAMERAS.keys()):
        r = all_results.get(cam_name, {})
        if not isinstance(r, dict): continue
        faces = r.get("faces", {})
        parts = [cam_name]
        for fk in ["front", "left", "back", "right", "top"]:
            cls = faces.get(fk, {}).get("classification", "?")
            parts.append(cls[:12])
        parts.append("✓" if r.get("pass") else "✗")
        print("  ".join(parts))

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()

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
from cr5_spray_perception import aruco_compat

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


def compose_T_camera_target_from_face_pnp(rvec, tvec, face_frame, tf_buf):
    """
    将 face-local PnP 结果组合为 T_camera_target.

    数学关系:
      p_camera = T_camera_face @ p_face
      p_face = inv(T_target_face) @ p_target
      ∴ p_camera = T_camera_face @ inv(T_target_face) @ p_target
      ∴ T_camera_target = T_camera_face @ inv(T_target_face)

    Returns:
      (T_camera_target_4x4, T_target_camera_4x4) 或 (None, None)
    """
    R_cam_face, _ = cv2.Rodrigues(rvec)
    T_cam_face = np.eye(4)
    T_cam_face[:3, :3] = R_cam_face
    T_cam_face[:3, 3] = tvec.flatten()

    face_pos, face_quat = get_face_tf(tf_buf, face_frame)
    if face_pos is None:
        return None, None

    from tf.transformations import quaternion_matrix
    T_target_face = quaternion_matrix([face_quat[0], face_quat[1],
                                        face_quat[2], face_quat[3]])
    T_target_face[:3, 3] = face_pos

    # T_camera_target: 将 target 坐标系中的点转到 camera 坐标系
    T_camera_target = T_cam_face @ np.linalg.inv(T_target_face)
    # T_target_camera: 将 camera 坐标系中的点转到 target 坐标系
    T_target_camera = np.linalg.inv(T_camera_target)

    return T_camera_target, T_target_camera


def invert_transform(T):
    """计算 4x4 逆矩阵."""
    return np.linalg.inv(T)


def rvec_tvec_from_44(T):
    """从 4x4 矩阵提取 rvec, tvec."""
    rvec = cv2.Rodrigues(T[:3, :3])[0]
    tvec = T[:3, 3]
    return rvec.flatten().tolist(), tvec.flatten().tolist()


def gazebo_truth(tf_buf, optical_frame):
    """
    查询 Gazebo TF truth.

    lookup_transform(optical_frame, "calibration_target_frame") 返回:
      T_camera_target (source→target 方向: optical_frame→target)

    Returns:
      {
        "T_camera_target_tx/ty/tz/qx/qy/qz/qw": ...,
        "T_target_camera_4x4": ...
      }
    """
    try:
        ts = tf_buf.lookup_transform(
            optical_frame, "calibration_target_frame",
            rospy.Time(0), rospy.Duration(3.0))
        t = ts.transform.translation
        q = ts.transform.rotation

        from tf.transformations import quaternion_matrix
        T_cam_target = quaternion_matrix([q.x, q.y, q.z, q.w])
        T_cam_target[:3, 3] = [t.x, t.y, t.z]

        T_target_cam = np.linalg.inv(T_cam_target)
        rvec_tc, tvec_tc = rvec_tvec_from_44(T_target_cam)

        return {
            # T_camera_target (optical_frame → target)
            "T_camera_target_tx": t.x, "T_camera_target_ty": t.y,
            "T_camera_target_tz": t.z,
            "T_camera_target_qx": q.x, "T_camera_target_qy": q.y,
            "T_camera_target_qz": q.z, "T_camera_target_qw": q.w,
            "T_camera_target_4x4": T_cam_target.tolist(),
            # T_target_camera (target → optical_frame)
            "T_target_camera_tvec": tvec_tc,
            "T_target_camera_rvec": rvec_tc,
            "T_target_camera_4x4": T_target_cam.tolist(),
            # 坐标方向说明
            "transform_contract": {
                "T_camera_target_equation": "p_camera = T_camera_target @ p_target",
                "T_target_camera_equation": "p_target = T_target_camera @ p_camera",
                "T_target_camera_equals_inv_T_camera_target": True,
            },
        }
    except Exception:
        return None


def select_best_solution(solutions):
    """
    从多候选解中选择最优 PnP 初值.

    优先级:
      1. stats.pass == True (RMSE <= MAX_REPROJ)
      2. 逐候选 inlier 门限 (4 点→min 4, 其他→max(4, min(10, ceil(n_pts*0.6))))
      3. reprojection RMSE 最小
      4. RMSE 接近时优先 inlier_ratio 更高, 然后 point_count 更高

    Returns:
      (best_solution, selection_reason, all_candidates_ranked)
      all_candidates_ranked 包含所有候选 (含失败), 每个带 elimination_reason.
    """
    if not solutions:
        return None, "no solutions", []

    def _candidate_has_sufficient_inliers(sol):
        n_pts = sol.get("n_pts", 0)
        inliers = sol.get("inliers", 0)
        if n_pts <= 4:
            required = 4
        else:
            required = max(4, min(10, int(math.ceil(n_pts * 0.6))))
        return inliers >= required

    # 对所有候选进行排名 (含失败原因)
    all_ranked = []
    for s in solutions:
        entry = {
            "panel": s.get("panel"),
            "type": s.get("type"),
            "rmse_px": s.get("rmse_px"),
            "inliers": s.get("inliers"),
            "n_pts": s.get("n_pts"),
            "pass": s.get("pass", False),
            "has_valid_transform": s.get("T_camera_target_4x4") is not None,
        }

        elim = []
        if not entry["pass"]:
            elim.append("reprojection_failed")
        if not entry["has_valid_transform"]:
            elim.append("missing_face_tf")
        if entry["pass"] and entry["has_valid_transform"]:
            if not _candidate_has_sufficient_inliers(s):
                elim.append("insufficient_inliers")
        entry["elimination_reason"] = elim if elim else None
        all_ranked.append(entry)

    # 1. 只考虑 pass=True
    passing = [s for s in solutions if s.get("pass", False)]
    if not passing:
        ranked = sorted(solutions, key=lambda s: s.get("rmse_px", 999))
        return ranked[0], "no pass — fallback to min RMSE", all_ranked

    # 2. 逐候选 inlier 门限
    sufficient = [s for s in passing if _candidate_has_sufficient_inliers(s)]
    if not sufficient:
        sufficient = passing  # 放宽门限

    # 3. 按 RMSE 升序 → inlier_ratio 降序 → point_count 降序
    ranked = sorted(sufficient, key=lambda s: (
        s.get("rmse_px", 999),
        -(s.get("inliers", 0) / max(s.get("n_pts", 1), 1)),
        -s.get("n_pts", 0),
    ))

    # 4. RMSE 接近时看 inlier_ratio
    best = ranked[0]
    best_rmse = best.get("rmse_px", 999)
    close_thresh = best_rmse * 1.2  # 20% 范围内视为接近
    close_candidates = [s for s in ranked if s.get("rmse_px", 999) <= close_thresh]

    if len(close_candidates) > 1:
        best = max(close_candidates, key=lambda s: (
            s.get("inliers", 0) / max(s.get("n_pts", 1), 1)))

    reason = (
        f"selected from {len(solutions)} candidates "
        f"({len(passing)} pass, {len(sufficient)} sufficient inliers), "
        f"best RMSE={best.get('rmse_px', -1):.4f}px, "
        f"panel={best.get('panel', 'unknown')}"
    )
    return best, reason, all_ranked


def compute_transform_errors(T_camera_target_est, T_camera_target_truth):
    """
    在同一方向下计算平移和旋转误差.

    Args:
      T_camera_target_est: 4x4 估计值 (PnP)
      T_camera_target_truth: 4x4 真值 (Gazebo TF)

    Returns:
      {
        translation_error_m, translation_error_mm,
        rotation_error_rad, rotation_error_deg,
      }
    """
    t_est = T_camera_target_est[:3, 3]
    t_truth = T_camera_target_truth[:3, 3]
    trans_err_m = float(np.linalg.norm(t_est - t_truth))

    R_est = T_camera_target_est[:3, :3]
    R_truth = T_camera_target_truth[:3, :3]
    R_diff = R_est @ R_truth.T
    trace = np.trace(R_diff)
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))

    return {
        "translation_error_m": round(trans_err_m, 6),
        "translation_error_mm": round(trans_err_m * 1000.0, 3),
        "rotation_error_rad": round(float(angle), 6),
        "rotation_error_deg": round(float(angle * 180.0 / math.pi), 4),
        "condition": "||t_est - t_truth||; R_diff = R_est @ R_truth.T, "
                      "angle = acos((trace(R_diff)-1)/2)",
    }


def process_camera(cam_name, tf_buf, output_dir, truth_source="none"):
    """处理单台相机 — V4 双方向外参 + 多候选解选择."""
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
        rospy.logwarn("%s: optical_frame=%s (expected *_color_optical_frame)",
                      cam_name, optical_frame)

    results = {
        "camera": cam_name,
        "optical_frame": optical_frame,
        "K": K.tolist(),
        "D": D.tolist(),
        "solutions": [],
        "transform_contract": {
            "T_camera_target_equation": "p_camera = T_camera_target @ p_target",
            "T_target_camera_equation": "p_target = T_target_camera @ p_camera",
            "T_target_camera_equals_inv_T_camera_target": True,
        },
    }

    # ── ChArUco faces ──
    for fk in ["front", "left", "back"]:
        obj_pts, img_pts = detect_charuco_face(gray, K, D, fk)
        if obj_pts is None:
            continue
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            ff = CHARUCO_FACES[fk]["face_frame"]
            T_ct, T_tc = compose_T_camera_target_from_face_pnp(
                rvec, tvec, ff, tf_buf)
            sol = {
                "panel": fk, "type": "charuco",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                **stats,
            }
            if T_ct is not None:
                rvec_ct, tvec_ct = rvec_tvec_from_44(T_ct)
                rvec_tc, tvec_tc = rvec_tvec_from_44(T_tc)
                sol.update({
                    "T_camera_target_rvec": rvec_ct,
                    "T_camera_target_tvec": tvec_ct,
                    "T_camera_target_4x4": T_ct.tolist(),
                    "T_target_camera_rvec": rvec_tc,
                    "T_target_camera_tvec": tvec_tc,
                    "T_target_camera_4x4": T_tc.tolist(),
                })
            results["solutions"].append(sol)

    # ── AprilTag right ──
    obj_pts, img_pts = detect_apriltag_face(gray, "right")
    if obj_pts is not None:
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            T_ct, T_tc = compose_T_camera_target_from_face_pnp(
                rvec, tvec, APRILTAG_RIGHT["face_frame"], tf_buf)
            sol = {
                "panel": "right", "type": "apriltag",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                **stats,
            }
            if T_ct is not None:
                rvec_ct, tvec_ct = rvec_tvec_from_44(T_ct)
                rvec_tc, tvec_tc = rvec_tvec_from_44(T_tc)
                sol.update({
                    "T_camera_target_rvec": rvec_ct,
                    "T_camera_target_tvec": tvec_ct,
                    "T_camera_target_4x4": T_ct.tolist(),
                    "T_target_camera_rvec": rvec_tc,
                    "T_target_camera_tvec": tvec_tc,
                    "T_target_camera_4x4": T_tc.tolist(),
                })
            results["solutions"].append(sol)

    # ── AprilTag top ──
    obj_pts, img_pts = detect_apriltag_face(gray, "top")
    if obj_pts is not None:
        rvec, tvec, stats = solve_pnp(obj_pts, img_pts, K, D)
        if rvec is not None:
            T_ct, T_tc = compose_T_camera_target_from_face_pnp(
                rvec, tvec, APRILTAG_TOP["face_frame"], tf_buf)
            sol = {
                "panel": "top", "type": "apriltag",
                "rvec_camera_face": rvec.flatten().tolist(),
                "tvec_camera_face": tvec.flatten().tolist(),
                **stats,
            }
            if T_ct is not None:
                rvec_ct, tvec_ct = rvec_tvec_from_44(T_ct)
                rvec_tc, tvec_tc = rvec_tvec_from_44(T_tc)
                sol.update({
                    "T_camera_target_rvec": rvec_ct,
                    "T_camera_target_tvec": tvec_ct,
                    "T_camera_target_4x4": T_ct.tolist(),
                    "T_target_camera_rvec": rvec_tc,
                    "T_target_camera_tvec": tvec_tc,
                    "T_target_camera_4x4": T_tc.tolist(),
                })
            results["solutions"].append(sol)

    # ── 多候选解选择 ──
    best, reason, ranked = select_best_solution(results["solutions"])
    if best is not None:
        results["selected_solution"] = best
        results["selection_reason"] = reason
        results["all_solutions_ranked"] = [
            {"panel": s.get("panel"), "rmse_px": s.get("rmse_px"),
             "inliers": s.get("inliers"), "pass": s.get("pass")}
            for s in ranked
        ]

    # ── Gazebo 真值对比 (同方向 T_camera_target) ──
    if truth_source == "gazebo":
        truth = gazebo_truth(tf_buf, optical_frame)
        results["gazebo_truth"] = truth

        if truth and best is not None and best.get("T_camera_target_4x4"):
            T_ct_est = np.array(best["T_camera_target_4x4"])
            T_ct_truth = np.array(truth["T_camera_target_4x4"])

            errors = compute_transform_errors(T_ct_est, T_ct_truth)
            results["truth_errors"] = errors

            # 分级评定
            mm = errors["translation_error_mm"]
            deg = errors["rotation_error_deg"]
            if mm <= 10 and deg <= 0.5:
                grade = "PASS"
            elif mm <= 20 and deg <= 1.0:
                grade = "WARN"
            else:
                grade = "FAIL"
            results["truth_grade"] = grade

    # 保存
    cv2.imwrite(os.path.join(output_dir, f"{cam_name}.png"), cv_img)
    with open(os.path.join(output_dir, f"{cam_name}_result.json"), "w") as f:
        json.dump(results, f, indent=2, default=str)

    return results


def _build_truth_comparison(all_results, per_camera_pass):
    """
    生成完整的 truth 对比报告 (estimate + truth + errors + grade).

    与 Commit 4 不同, 此版本同时输出每台相机的 estimate/truth/errors/grade.
    """
    comparison = {
        "truth_source": "gazebo",
        "transform_direction": "T_camera_target",
        "thresholds": {
            "translation_pass_mm": 10.0,
            "translation_warn_mm": 20.0,
            "rotation_pass_deg": 0.5,
            "rotation_warn_deg": 1.0,
        },
        "cameras": {},
    }

    grades = []
    for cam_name in sorted(CAMERAS.keys()):
        cr = all_results.get(cam_name, {})
        if not isinstance(cr, dict):
            continue

        entry = {}
        selected = cr.get("selected_solution", {})
        truth = cr.get("gazebo_truth", {})

        if selected and selected.get("T_camera_target_4x4"):
            entry["selected_panel"] = selected.get("panel", "unknown")
            entry["estimate"] = {
                "T_camera_target": selected["T_camera_target_4x4"],
            }
        else:
            entry["selected_panel"] = "none"
            entry["estimate"] = None

        if truth and truth.get("T_camera_target_4x4"):
            entry["truth"] = {
                "T_camera_target": truth["T_camera_target_4x4"],
            }
        else:
            entry["truth"] = None

        if cr.get("truth_errors"):
            entry["errors"] = cr["truth_errors"]
        if cr.get("truth_grade"):
            entry["grade"] = cr["truth_grade"]
            grades.append(cr["truth_grade"])

        comparison["cameras"][cam_name] = entry

    # 整体判定
    if not grades:
        comparison["overall_result"] = "UNKNOWN"
    elif "FAIL" in grades:
        comparison["overall_result"] = "FAIL"
    elif "WARN" in grades:
        comparison["overall_result"] = "WARN"
    else:
        comparison["overall_result"] = "PASS"

    return comparison


def build_standard_extrinsics_yaml(all_results, per_camera_pass):
    """
    构建符合 CaptureManager 消费端 schema 的 initial_extrinsics.yaml.

    结构:
      schema_version: 1
      calibration_id: ...
      status: PASS | FAIL
      target_frame: calibration_target_frame
      transform_contract: {...}
      cameras:
        cam_front_left:
          optical_frame: ...
          selected_panel: ...
          selected_type: ...
          point_count: ...
          inlier_count: ...
          inlier_ratio: ...
          reprojection_rmse_px: ...
          T_camera_target: 4x4
          T_target_camera: 4x4
          truth_errors: {...}
          truth_grade: PASS|WARN|FAIL
    """
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    all_pass = all(per_camera_pass.values())

    doc = {
        "schema_version": 1,
        "calibration_id": f"three_camera_pnp_{timestamp}",
        "generated_at": timestamp,
        "method": "multi_face_pnp_gazebo_validation",
        "status": "PASS" if all_pass else "FAIL",
        "target_frame": "calibration_target_frame",
        "transform_contract": {
            "primary_transform": "T_target_camera",
            "inverse_transform": "T_camera_target",
            "primary_equation": "p_target = T_target_camera @ p_camera",
            "inverse_equation": "p_camera = T_camera_target @ p_target",
        },
        "cameras": {},
    }

    for cam_name in sorted(CAMERAS.keys()):
        cr = all_results.get(cam_name, {})
        if not isinstance(cr, dict):
            continue

        selected = cr.get("selected_solution", {})
        cam_entry = {
            "optical_frame": cr.get("optical_frame", "unknown"),
            "selected_panel": selected.get("panel", "none"),
            "selected_type": selected.get("type", "none"),
            "point_count": selected.get("n_pts", 0),
            "inlier_count": selected.get("inliers", 0),
            "inlier_ratio": round(
                selected.get("inliers", 0) / max(selected.get("n_pts", 1), 1), 4),
            "reprojection_rmse_px": selected.get("rmse_px", -1.0),
            "T_camera_target": selected.get("T_camera_target_4x4", []),
            "T_target_camera": selected.get("T_target_camera_4x4", []),
        }

        # 附加 truth 信息 (如果有)
        if cr.get("truth_errors"):
            cam_entry["truth_errors"] = cr["truth_errors"]
        if cr.get("truth_grade"):
            cam_entry["truth_grade"] = cr["truth_grade"]

        doc["cameras"][cam_name] = cam_entry

    return doc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/calibration_target")
    parser.add_argument("--truth-source", default="none",
                        choices=["none", "gazebo"],
                        help="none: PnP only (default); gazebo: also compare with Gazebo TF truth")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("calibrate_three_fixed_cameras", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    os.makedirs(args.output, exist_ok=True)

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.sleep(1.5)

    all_results = {
        "opencv_capability": aruco_compat.get_opencv_info(),
        "capture_mode": "online_debug",
        "capture_mode_note": "非严格同步采集, color+CameraInfo 独立 wait_for_message. "
                            "正式外参估计建议使用 CaptureManager 批量采集数据.",
    }
    per_camera_pass = {}
    for cam_name in sorted(CAMERAS.keys()):
        cr = process_camera(cam_name, tf_buf, args.output, args.truth_source)
        all_results[cam_name] = cr
        # Per-camera: must have at least one passing solution
        solutions = cr.get("solutions", []) if isinstance(cr, dict) else []
        per_camera_pass[cam_name] = any(
            s.get("pass", False) and s.get("T_camera_target_tvec") is not None
            for s in solutions
        )

    all_cameras_have_solution = all(per_camera_pass.values())
    all_results["per_camera_pass"] = per_camera_pass
    all_results["all_cameras_have_solution"] = all_cameras_have_solution

    # ── 诊断报告 (保留完整信息, 供人工审查) ──
    import yaml as _yaml

    # 1. initial_extrinsics.yaml: 标准化可消费外参 (CaptureManager 输入)
    extrinsics_doc = build_standard_extrinsics_yaml(all_results, per_camera_pass)
    with open(os.path.join(args.output, "initial_extrinsics.yaml"), "w") as f:
        _yaml.dump(extrinsics_doc, f, default_flow_style=False)

    # 2. all_candidates.json: 所有候选解 (诊断)
    with open(os.path.join(args.output, "all_candidates.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)

    # 3. reprojection_report.json: 重投影汇总
    reproj = {}
    for cn, cr in all_results.items():
        if isinstance(cr, dict) and "solutions" in cr:
            reproj[cn] = [{"panel": s["panel"], "rmse_px": s.get("rmse_px", -1),
                           "inliers": s.get("inliers", 0), "pass": s.get("pass", False)}
                          for s in cr["solutions"]]
    with open(os.path.join(args.output, "reprojection_report.json"), "w") as f:
        json.dump(reproj, f, indent=2)

    # 4. gazebo_truth_comparison.json: 完整 truth 对比 (Commit 7 增强)
    if args.truth_source == "gazebo":
        all_results["truth_source"] = "gazebo"
        truth_comparison = _build_truth_comparison(all_results, per_camera_pass)
        with open(os.path.join(args.output, "gazebo_truth_comparison.json"), "w") as f:
            json.dump(truth_comparison, f, indent=2)
    else:
        all_results["truth_source"] = "none"
        all_results["truth_comparison"] = "unavailable"

    # 5. calibration_summary.json
    summary = {
        "per_camera_pass": per_camera_pass,
        "all_cameras_have_solution": all_cameras_have_solution,
        "overall_status": extrinsics_doc["status"],
        "git_sha": os.environ.get("CR5_SPRAY_GIT_SHA", "unknown"),
    }
    with open(os.path.join(args.output, "calibration_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    if all_cameras_have_solution:
        rospy.loginfo("Calibration PASS: all cameras have valid solutions")
        sys.exit(0)
    else:
        rospy.logerr("Calibration FAIL: %s", per_camera_pass)
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
多帧标定采集 + PnP + Bundle Adjustment 工作流.

修复 (P0-1~P0-7):
  P0-1: 读取 JointCaptureManager 保存的同步帧组图像 (非 rospy.wait_for_message)
  P0-2: 合并各面检测结果为相机级 obj_pts/img_pts (BA 兼容格式)
  P0-3: 使用整数 group ID
  P0-4: 将面板局部坐标转换到 calibration_target_frame
  P0-5: 每帧每相机运行 solvePnP 计算 T_camera_target 初值
  P0-6: 恒等四元数 [1,0,0,0]
  P0-7: 输出 T_rig_camera (rig=第一台相机 optical frame)

用法:
  rosrun cr5_spray_perception run_multi_frame_calibration.py \
    --num-groups 10 --output artifacts/calibration
"""
import sys, os, json, time, math, argparse, glob
import yaml
import numpy as np
import cv2
from cv2 import aruco
import rospy
from std_srvs.srv import Trigger
from datetime import datetime

from cr5_spray_perception import aruco_compat

# ── 面板定义 ──
CHARUCO_FACES = {
    "front": {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 100,
              "face_frame": "calibration_target_front_frame"},
    "back":  {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
              "dict_id": aruco.DICT_5X5_1000, "id_start": 300,
              "face_frame": "calibration_target_back_frame"},
}

APRILTAG_FACES = {
    "left": {"tag_size": 0.07, "tag_ids": [4,5,6,7],
             "face_frame": "calibration_target_left_frame",
             "positions": {4:(-0.0425,0.0425,0), 5:(0.0425,0.0425,0),
                          6:(-0.0425,-0.0425,0), 7:(0.0425,-0.0425,0)}},
    "top":  {"tag_size": 0.12, "tag_ids": [8],
             "face_frame": "calibration_target_top_frame",
             "positions": {8:(0,0,0)}},
}

# V3: YAML 为几何真值来源, 此处仅保留 import 阶段 fallback.
# 正式运行时 main() 从 calibration_target.yaml 覆盖.
ARUCO_FACES = {
    "right": {"marker_size_m": 0.076, "marker_ids": [10, 11, 12, 13],
              "dict_id": aruco.DICT_4X4_50,
              "face_frame": "calibration_target_right_frame",
              "positions": {10: (-0.047, 0.044, 0), 11: (0.047, 0.044, 0),
                           12: (-0.047, -0.044, 0), 13: (0.047, -0.044, 0)}},
}

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]

# ── 从 calibration_target.yaml 加载面板位姿 ──
# 硬编码值仅做 fallback (与仓库中 YAML 保持一致)
_FALLBACK_FACE_POSES = {
    "front": {"xyz": [0.171, 0.0, 0.0],     "rpy": [0.0,  math.pi/2, 0.0]},
    "left":  {"xyz": [0.0, 0.141, 0.0],     "rpy": [-math.pi/2, 0.0, 0.0]},
    "right": {"xyz": [0.0, -0.141, 0.0],    "rpy": [math.pi/2, 0.0, 0.0]},
    "top":   {"xyz": [0.0, 0.0, 0.121],     "rpy": [0.0, 0.0, 0.0]},
    "back":  {"xyz": [-0.171, 0.0, 0.0],    "rpy": [0.0, -math.pi/2, 0.0]},
}


def _load_face_poses_from_yaml():
    """从 calibration_target.yaml 读取面板位姿.

    确保单一真值来源. YAML 不可用时返回 None (由调用者决定是否 fallback).
    """
    search_paths = []
    # 通过 rospack 查找
    try:
        import rospkg
        rp = rospkg.RosPack()
        sim_path = rp.get_path("cr5_spray_sim")
        search_paths.append(os.path.join(
            sim_path, "config", "calibration", "calibration_target.yaml"))
    except Exception:
        pass
    # 相对于本脚本的路径
    search_paths.append(os.path.join(
        os.path.dirname(__file__), "..", "..", "cr5_spray_sim",
        "config", "calibration", "calibration_target.yaml"))

    for p in search_paths:
        if os.path.isfile(p):
            try:
                with open(p, "r") as f:
                    cfg = yaml.safe_load(f)
                panels = cfg.get("panels", {})
                poses = {}
                for name, panel in panels.items():
                    pt = panel.get("pose_target", {})
                    if pt and "xyz" in pt and "rpy" in pt:
                        poses[name] = {
                            "xyz": list(pt["xyz"]),
                            "rpy": list(pt["rpy"]),
                        }
                if len(poses) >= 5:
                    rospy.loginfo("Loaded %d face poses from %s",
                                  len(poses), p)
                    return poses
            except Exception as e:
                rospy.logwarn("Failed to load face poses from %s: %s", p, e)

    # YAML 不可用 → 返回 None, 由调用者决定是否 fallback
    rospy.logwarn("calibration_target.yaml not found — returning None")
    return None


# P0-2 修复: 使用 fallback 作为模块顶层默认值, 避免 import 时迭代 None.
# 正式运行时 main() 会用 calibration_target.yaml 覆盖.
FACE_POSES_TARGET = dict(_FALLBACK_FACE_POSES)


def _load_full_yaml():
    """加载完整 calibration_target.yaml (含面板级 marker center 定义)."""
    search_paths = []
    try:
        import rospkg
        rp = rospkg.RosPack()
        sim_path = rp.get_path("cr5_spray_sim")
        search_paths.append(os.path.join(
            sim_path, "config", "calibration", "calibration_target.yaml"))
    except Exception:
        pass
    search_paths.append(os.path.join(
        os.path.dirname(__file__), "..", "..", "cr5_spray_sim",
        "config", "calibration", "calibration_target.yaml"))
    for p in search_paths:
        if os.path.isfile(p):
            try:
                with open(p, "r") as f:
                    return yaml.safe_load(f)
            except Exception as e:
                rospy.logwarn("Failed to load full YAML from %s: %s", p, e)
    return None


def _euler_matrix(ai, aj, ak):
    """tf.transformations.euler_matrix 等价实现, 避免 ROS tf 依赖."""
    from math import cos, sin
    Rx = np.array([[1, 0, 0], [0, cos(ai), -sin(ai)], [0, sin(ai), cos(ai)]])
    Ry = np.array([[cos(aj), 0, sin(aj)], [0, 1, 0], [-sin(aj), 0, cos(aj)]])
    Rz = np.array([[cos(ak), -sin(ak), 0], [sin(ak), cos(ak), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    return T


def _quaternion_from_matrix(T):
    """从 4x4 旋转矩阵提取四元数 [x,y,z,w]."""
    R = np.asarray(T[:3, :3], dtype=np.float64)
    q = np.empty(4)
    t = R.trace()
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2,1] - R[1,2]) * s
        q[1] = (R[0,2] - R[2,0]) * s
        q[2] = (R[1,0] - R[0,1]) * s
    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            q[3] = (R[2,1] - R[1,2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0,1] + R[1,0]) / s
            q[2] = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            q[3] = (R[0,2] - R[2,0]) / s
            q[0] = (R[0,1] + R[1,0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            q[3] = (R[1,0] - R[0,1]) / s
            q[0] = (R[0,2] + R[2,0]) / s
            q[1] = (R[1,2] + R[2,1]) / s
            q[2] = 0.25 * s
    return [float(v) for v in q]


def build_T_target_face(face_name):
    """构建 T_target_face 4x4 矩阵."""
    p = FACE_POSES_TARGET[face_name]
    T = _euler_matrix(p["rpy"][0], p["rpy"][1], p["rpy"][2])
    T[:3, 3] = p["xyz"]
    return T


# 预构建
T_TARGET_FACE = {name: build_T_target_face(name) for name in FACE_POSES_TARGET}

# 预创建 Charuco boards
for v in CHARUCO_FACES.values():
    v["board"] = aruco.CharucoBoard_create(
        v["sx"], v["sy"], v["sq_m"], v["mk_m"],
        aruco.getPredefinedDictionary(v["dict_id"]))


# ═══════════════════════════════════════════════════════════════
# 检测
# ═══════════════════════════════════════════════════════════════

def detect_on_image(cv_img, K, D):
    """检测所有 ChArUco/AprilTag 面, 返回 face-keyed 检测结果.

    返回格式: {face_name: {object_points_3d_face, image_points_2d, corner_count}}
    object_points_3d_face: 面板局部坐标系 (z=0 平面)
    """
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    results = {}

    # ── ChArUco 面 ──
    for fk, fc in CHARUCO_FACES.items():
        board = fc["board"]
        id_start = fc["id_start"]
        params = aruco_compat.detector_parameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejected = aruco_compat.detect_markers(
            gray, board.dictionary, params)

        obj_pts_face, img_pts_face = [], []
        if ids is not None:
            ids_flat = [int(i) for i in ids.flatten()]
            idx_list, local_ids = aruco_compat.remap_custom_ids(
                ids_flat, id_start, board)
            if len(idx_list) >= 2:
                local_corners = tuple(corners[i] for i in idx_list)
                cc, cids = aruco_compat.interpolate_charuco_corners(
                    local_corners, local_ids, gray, board,
                    cameraMatrix=K, distCoeffs=D)
                if cids is not None and len(cids) >= 4:
                    board_pts = np.asarray(board.chessboardCorners,
                                          dtype=np.float32).reshape(-1, 3)
                    bw = fc["sx"] * fc["sq_m"]
                    bh = fc["sy"] * fc["sq_m"]
                    board_pts[:, 0] -= bw / 2.0
                    board_pts[:, 1] -= bh / 2.0
                    cids_flat = [int(i) for i in cids.flatten()]
                    obj_pts_face = [board_pts[i].tolist() for i in cids_flat]
                    img_pts_face = cc.reshape(-1, 2).astype(np.float32).tolist()

        results[fk] = {
            "object_points_3d_face": obj_pts_face,
            "image_points_2d": img_pts_face,
            "corner_count": len(obj_pts_face),
        }

    # ── ArUco 面 (DICT_4X4_50, 右面) ──
    aruco_dict_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params_4x4 = aruco_compat.detector_parameters()
    params_4x4.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners_4x4, ids_4x4, _ = aruco_compat.detect_markers(
        gray, aruco_dict_4x4, params_4x4)

    for fk, fc in ARUCO_FACES.items():
        obj_pts_face, img_pts_face = [], []
        if ids_4x4 is not None:
            ids_flat = [int(i) for i in ids_4x4.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["marker_ids"]:
                    continue
                pos = fc["positions"][tid]
                half = fc["marker_size_m"] / 2.0
                marker_obj = [
                    [pos[0]-half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]-half, 0],
                    [pos[0]-half, pos[1]-half, 0],
                ]
                obj_pts_face.extend(marker_obj)
                img_pts_face.extend(corners_4x4[i][0].tolist())

        results[fk] = {
            "object_points_3d_face": obj_pts_face,
            "image_points_2d": img_pts_face,
            "corner_count": len(obj_pts_face),
        }

    # ── AprilTag 面 ──
    tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(
        gray, tag_dict, params)

    for fk, fc in APRILTAG_FACES.items():
        obj_pts_face, img_pts_face = [], []
        if ids is not None:
            ids_flat = [int(i) for i in ids.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["tag_ids"]:
                    continue
                pos = fc["positions"][tid]
                half = fc["tag_size"] / 2.0
                tag_obj = [
                    [pos[0]-half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]-half, 0],
                    [pos[0]-half, pos[1]-half, 0],
                ]
                obj_pts_face.extend(tag_obj)
                # P1-2: 展平 - corners[i][0] 是 4 个角点 [[u,v],...], 需要 expand
                img_pts_face.extend(corners[i][0].tolist())

        results[fk] = {
            "object_points_3d_face": obj_pts_face,
            "image_points_2d": img_pts_face,
            "corner_count": len(obj_pts_face),
        }

    return results


# ═══════════════════════════════════════════════════════════════
# 坐标转换 + PnP
# ═══════════════════════════════════════════════════════════════

def transform_points_to_target(obj_pts_face, face_name):
    """将面板局部 3D 点转换到 calibration_target_frame."""
    if not obj_pts_face:
        return []
    T = T_TARGET_FACE[face_name]
    result = []
    for pt in obj_pts_face:
        p_h = np.array([pt[0], pt[1], pt[2], 1.0])
        p_t = T @ p_h
        result.append([float(p_t[0]), float(p_t[1]), float(p_t[2])])
    return result


def merge_face_detections_to_target(detection):
    """合并所有面的检测结果到统一的 calibration_target_frame.

    Returns:
        obj_pts_target: 所有面板点在 target 坐标系中的 3D 坐标
        img_pts: 对应的 2D 像素坐标
        face_counts: {face_name: corner_count}
    """
    all_obj = []
    all_img = []
    face_counts = {}
    for fk, fd in detection.items():
        obj_face = fd.get("object_points_3d_face", [])
        img_face = fd.get("image_points_2d", [])
        if not obj_face:
            continue
        # P0-4: 转换到 target 坐标系
        obj_target = transform_points_to_target(obj_face, fk)
        all_obj.extend(obj_target)
        all_img.extend(img_face)
        face_counts[fk] = len(obj_target)
    return all_obj, all_img, face_counts


def undistort_points(img_pts, K, D):
    """去畸变像素坐标, 使 Ceres BA 可以使用纯针孔模型."""
    if not img_pts or D is None or np.all(np.array(D) == 0):
        return img_pts
    pts = np.array(img_pts, dtype=np.float32).reshape(-1, 1, 2)
    K_arr = np.array(K, dtype=np.float64).reshape(3, 3)
    D_arr = np.array(D, dtype=np.float64).reshape(-1)
    # undistortPoints + 还原到像素坐标
    undistorted = cv2.undistortPoints(pts, K_arr, D_arr, P=K_arr)
    return undistorted.reshape(-1, 2).tolist()


def _compute_reproj_stats(obj_pts, img_pts, rvec, tvec, K_arr, D_arr):
    """计算重投影统计: rmse_all, rmse_inlier, median, P90, max, positive_depth_ratio."""
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K_arr, D_arr)
    errors = np.linalg.norm(img_pts - proj.reshape(-1, 2), axis=1)

    # Cheirality: positive depth ratio
    R, _ = cv2.Rodrigues(rvec)
    t = np.asarray(tvec).flatten()
    pts_cam = (R @ obj_pts.T).T + t  # Nx3
    depth_pos = np.sum(pts_cam[:, 2] > 0)
    pos_ratio = depth_pos / len(obj_pts) if len(obj_pts) > 0 else 0.0

    return {
        "rmse_all_px": float(np.sqrt(np.mean(errors ** 2))),
        "median_px": float(np.median(errors)),
        "p90_px": float(np.percentile(errors, 90)),
        "max_px": float(np.max(errors)),
        "positive_depth_ratio": float(pos_ratio),
        "errors": errors,
    }


def _compute_planarity(obj_pts):
    """SVD 平面分析: 返回 s1,s2,s3, s3/s2 ratio, planarity_score.

    s3/s2 < 1e-3 通常表示共面 (ChArUco board),
    非共面 (多面观测) 会给出更大值.
    """
    pts_mean = np.mean(obj_pts, axis=0)
    pts_centered = obj_pts - pts_mean
    _, S, _ = np.linalg.svd(pts_centered, full_matrices=False)
    s1, s2, s3 = float(S[0]), float(S[1]), float(S[2]) if len(S) > 2 else 0.0
    ratio = s3 / s2 if s2 > 1e-12 else 1.0
    planarity_score = float(s3 / (s1 + s2 + s3)) if (s1 + s2 + s3) > 1e-12 else 0.0
    return s1, s2, s3, ratio, planarity_score


def _rvec_tvec_to_T(rvec, tvec):
    """rvec, tvec → 4x4 T matrix."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec).flatten()
    return T


def solve_pnp(obj_pts, img_pts, K, D):
    """PnP: 计算 T_camera_target (相机在 target 坐标系中的位姿). V2: 新增 planarity/IPPE/统计.

    OpenCV solvePnP 返回的 rvec/tvec 满足:
      p_camera = R * p_target + t
    即 T_camera_target: 将 target 系 3D 点变换到 camera 系

    Args:
        obj_pts: Nx3 目标坐标 (calibration_target_frame)
        img_pts: Nx2 像素坐标 (去畸变后)
        K: 3x3 内参矩阵
        D: 畸变系数 (可选, 已去畸变传 None)

    Returns:
        T_cam_target: 4x4 变换矩阵
        rvec, tvec: OpenCV 格式
        stats: dict {
            solver, planar, singular_values, planarity_score,
            n_points, n_inliers, inlier_ratio,
            rmse_all_px, rmse_inlier_px, median_px, p90_px, max_px,
            candidates: [候选列表],
            selected_candidate: int,  # 选中的 candidate 索引
        }
        失败返回 (None, None, None, {"error": ...})
    """
    if len(obj_pts) < 4:
        return None, None, None, {"error": "need >= 4 points", "solver": "none",
                                   "planar": False, "n_points": len(obj_pts),
                                   "n_inliers": 0, "inlier_ratio": 0.0,
                                   "rmse_all_px": 0.0, "rmse_inlier_px": 0.0,
                                   "singular_values": [0,0,0], "s3_s2_ratio": 0.0,
                                   "planarity_score": 0.0, "candidates": [],
                                   "selected_candidate": 0}

    obj = np.array(obj_pts, dtype=np.float64).reshape(-1, 3)
    img = np.array(img_pts, dtype=np.float64).reshape(-1, 2)
    K_arr = np.array(K, dtype=np.float64).reshape(3, 3)
    D_arr = np.array(D, dtype=np.float64).reshape(-1) if D is not None else np.zeros(4)

    n_pts = len(obj)

    # ── A. 计算 SVD 平面分析 ──
    s1, s2, s3, sv_ratio, planarity_score = _compute_planarity(obj)
    planar = sv_ratio < 1e-3  # 共面阈值

    # ── B. 统计基础 ──
    stats = {
        "planar": bool(planar),
        "singular_values": [s1, s2, s3],
        "s3_s2_ratio": float(sv_ratio),
        "planarity_score": float(planarity_score),
        "n_points": n_pts,
        "candidates": [],
        "selected_candidate": 0,
    }

    rvec_final, tvec_final = None, None

    if planar:
        # ── C. 共面: 尝试 solvePnPGeneric(IPPE) 获取双解; 失败则 fallback solvePnPRansac(IPPE) ──
        stats["solver"] = "IPPE"
        candidates = []

        def _try_solvePnPGeneric_IPPE():
            """OpenCV 4.2 solvePnPGeneric 有 dtype bug. 用 named args 避开."""
            try:
                retval, rvecs, tvecs, _reproj = cv2.solvePnPGeneric(
                    objectPoints=obj.astype(np.float32),
                    imagePoints=img.astype(np.float32),
                    cameraMatrix=K_arr.astype(np.float64),
                    distCoeffs=D_arr.astype(np.float64),
                    flags=cv2.SOLVEPNP_IPPE)
                return retval, rvecs, tvecs
            except Exception:
                return None, None, None

        retval, rvecs, tvecs = _try_solvePnPGeneric_IPPE()

        if rvecs is None or len(rvecs) == 0:
            # Fallback: 使用 solvePnPRansac(IPPE) 单解
            stats["solver"] = "IPPE_fallback"
            ok_fb, rvec_fb, tvec_fb, inliers_fb = cv2.solvePnPRansac(
                obj.astype(np.float32), img.astype(np.float32),
                K_arr.astype(np.float64), D_arr.astype(np.float64),
                flags=cv2.SOLVEPNP_IPPE, reprojectionError=3.0,
                confidence=0.99, iterationsCount=100)
            if not ok_fb or inliers_fb is None or len(inliers_fb) < 4:
                return None, None, None, {
                    "error": "IPPE all methods failed", "solver": "IPPE_fallback",
                    "planar": True, "n_points": n_pts, "n_inliers": 0,
                    "inlier_ratio": 0.0, "rmse_all_px": 0.0, "rmse_inlier_px": 0.0,
                    "singular_values": [s1, s2, s3], "s3_s2_ratio": float(sv_ratio),
                    "planarity_score": float(planarity_score), "candidates": [],
                    "selected_candidate": 0}
            rvecs = [rvec_fb]
            tvecs = [tvec_fb]

        candidates = []
        for idx, (rv, tv) in enumerate(zip(rvecs, tvecs)):
            rv_arr = np.asarray(rv, dtype=np.float64).reshape(3, 1)
            tv_arr = np.asarray(tv, dtype=np.float64).reshape(3, 1)

            # 使用 RANSAC 提取 inliers (IPPE 候选需要验证)
            ok_ransac, _, _, inliers = cv2.solvePnPRansac(
                obj.astype(np.float32), img.astype(np.float32),
                K_arr.astype(np.float64), D_arr.astype(np.float64),
                rvec=rv_arr, tvec=tv_arr, useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=3.0,
                confidence=0.99, iterationsCount=50)

            n_inl = len(inliers) if inliers is not None else 0
            inlier_pts = (obj[inliers.flatten()], img[inliers.flatten()]) if inliers is not None and len(inliers) >= 4 else (obj, img)

            # RefineLM 仅 inliers
            try:
                rv_refined, tv_refined = cv2.solvePnPRefineLM(
                    inlier_pts[0].astype(np.float32), inlier_pts[1].astype(np.float32),
                    K_arr.astype(np.float64), D_arr.astype(np.float64),
                    rv_arr, tv_arr,
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
            except Exception:
                rv_refined, tv_refined = rv_arr, tv_arr

            # 统计
            s_all = _compute_reproj_stats(
                obj.astype(np.float32), img.astype(np.float32),
                rv_refined, tv_refined, K_arr, D_arr)
            s_inl = _compute_reproj_stats(
                inlier_pts[0].astype(np.float32), inlier_pts[1].astype(np.float32),
                rv_refined, tv_refined, K_arr, D_arr)

            T = _rvec_tvec_to_T(rv_refined, tv_refined)

            candidates.append({
                "rvec": rv_refined.flatten().tolist(),
                "tvec": tv_refined.flatten().tolist(),
                "T_camera_target": T.tolist(),
                "n_inliers": n_inl,
                "inlier_ratio": float(n_inl / n_pts) if n_pts > 0 else 0.0,
                "positive_depth_ratio": s_all["positive_depth_ratio"],
                "rmse_all_px": s_all["rmse_all_px"],
                "rmse_inlier_px": s_inl["rmse_all_px"],
                "median_px": s_all["median_px"],
                "p90_px": s_all["p90_px"],
                "max_px": s_all["max_px"],
            })

        if not candidates:
            return None, None, None, {"error": "IPPE: no valid candidates"}

        # 选候选: (1) positive depth > 0.5, (2) 最低 inlier RMSE
        valid = [c for c in candidates if c["positive_depth_ratio"] > 0.5]
        if not valid:
            valid = candidates  # 全部无效就都保留
        best = min(valid, key=lambda c: c["rmse_inlier_px"])
        best_idx = candidates.index(best)

        stats["candidates"] = candidates
        stats["selected_candidate"] = best_idx
        stats["n_inliers"] = best["n_inliers"]
        stats["inlier_ratio"] = best["inlier_ratio"]
        stats["rmse_all_px"] = best["rmse_all_px"]
        stats["rmse_inlier_px"] = best["rmse_inlier_px"]
        stats["median_px"] = best["median_px"]
        stats["p90_px"] = best["p90_px"]
        stats["max_px"] = best["max_px"]
        stats["positive_depth_ratio"] = best["positive_depth_ratio"]

        rvec_final = np.array(best["rvec"], dtype=np.float64).reshape(3, 1)
        tvec_final = np.array(best["tvec"], dtype=np.float64).reshape(3, 1)

    else:
        # ── D. 非共面: EPNP + RANSAC → inlier-only RefineLM ──
        stats["solver"] = "EPNP"

        ok, rvec_init, tvec_init, inliers = cv2.solvePnPRansac(
            obj.astype(np.float32), img.astype(np.float32),
            K_arr.astype(np.float64), D_arr.astype(np.float64),
            flags=cv2.SOLVEPNP_EPNP, reprojectionError=3.0,
            confidence=0.99, iterationsCount=100)

        if not ok or inliers is None or len(inliers) < 4:
            return None, None, None, {"error": "EPNP RANSAC failed", "solver": "EPNP"}

        n_inl = len(inliers)
        inlier_mask = inliers.flatten()
        obj_inl = obj[inlier_mask].astype(np.float32)
        img_inl = img[inlier_mask].astype(np.float32)

        # RefineLM 仅使用 inliers
        try:
            rvec_final, tvec_final = cv2.solvePnPRefineLM(
                obj_inl, img_inl, K_arr.astype(np.float64), D_arr.astype(np.float64),
                rvec_init, tvec_init,
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        except Exception:
            rvec_final, tvec_final = rvec_init, tvec_init

        # 统计
        s_all = _compute_reproj_stats(
            obj.astype(np.float32), img.astype(np.float32),
            rvec_final, tvec_final, K_arr, D_arr)
        s_inl = _compute_reproj_stats(
            obj_inl, img_inl, rvec_final, tvec_final, K_arr, D_arr)

        stats["n_inliers"] = n_inl
        stats["inlier_ratio"] = float(n_inl / n_pts)
        stats["rmse_all_px"] = s_all["rmse_all_px"]
        stats["rmse_inlier_px"] = s_inl["rmse_all_px"]
        stats["median_px"] = s_all["median_px"]
        stats["p90_px"] = s_all["p90_px"]
        stats["max_px"] = s_all["max_px"]
        stats["positive_depth_ratio"] = s_all["positive_depth_ratio"]

    # ── E. 构造 T 矩阵 ──
    T = _rvec_tvec_to_T(rvec_final, tvec_final)

    # 兼容旧接口: rmse_px = rmse_inlier_px (inlier 上的 RMSE)
    stats["rmse_px"] = stats.get("rmse_inlier_px", stats.get("rmse_all_px", 0.0))
    stats["n_pts"] = n_pts

    return T, rvec_final, tvec_final, stats


def T_to_quat_trans(T):
    """从 4x4 矩阵提取 [qw,qx,qy,qz,tx,ty,tz] (Ceres 格式)."""
    q = _quaternion_from_matrix(T)
    return [q[3], q[0], q[1], q[2],
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]


def T_to_rvec_tvec(T):
    """从 4x4 矩阵提取 rvec, tvec."""
    rvec = cv2.Rodrigues(T[:3, :3])[0].flatten().tolist()
    tvec = T[:3, 3].flatten().tolist()
    return rvec, tvec


def compute_rig_poses(pnp_results):
    """从各相机的 T_camera_target 计算 T_rig_camera.

    rig = 第一台相机 (CAMERAS[0]) 的 optical frame.
    T_rig_cam0 = I (规范固定).
    T_rig_target = T_cam0_target (第一帧时 rig=target 的相对位姿从 cam0 的 PnP 获得).
    T_rig_cami = T_rig_target @ inv(T_cami_target).

    Returns:
        T_rig_cameras: {cam_name: 4x4}
        T_rig_target: 4x4
    """
    T_cam0_target = pnp_results.get(CAMERAS[0])
    if T_cam0_target is None:
        return None, None

    T_rig_cam0 = np.eye(4)
    T_rig_target = T_cam0_target.copy()  # 当 rig=cam0: T_rig_target = T_cam0_target

    T_rig_cameras = {CAMERAS[0]: T_rig_cam0}

    for cam in CAMERAS[1:]:
        T_cami_target = pnp_results.get(cam)
        if T_cami_target is None:
            continue
        T_rig_cami = T_rig_target @ np.linalg.inv(T_cami_target)
        T_rig_cameras[cam] = T_rig_cami

    return T_rig_cameras, T_rig_target


# ═══════════════════════════════════════════════════════════════
# V6: 运行清单 (数据来源追踪)
# ═══════════════════════════════════════════════════════════════

def _save_run_manifest(output_dir, n_groups):
    """保存 run_manifest.yaml — 记录本次标定运行的数据来源."""
    import hashlib
    import subprocess as _subprocess

    manifest = {
        "schema_version": 1,
        "run_id": os.path.basename(output_dir),
        "created_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "n_groups_captured": n_groups,
        "cameras": list(CAMERAS),
    }

    # Git SHA
    try:
        result = _subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True, cwd=os.path.dirname(__file__))
        if result.returncode == 0:
            manifest["git_sha"] = result.stdout.strip()
    except Exception:
        pass

    # 场景配置文件 SHA256
    scene_files = {}
    search_paths = []
    try:
        import rospkg
        rp = rospkg.RosPack()
        sim_path = rp.get_path("cr5_spray_sim")
        search_paths.append(os.path.join(
            sim_path, "config", "simulation_scene.yaml"))
        search_paths.append(os.path.join(
            sim_path, "config", "calibration", "calibration_target.yaml"))
    except Exception:
        pass

    for sp in search_paths:
        if os.path.isfile(sp):
            key = os.path.basename(sp)
            try:
                with open(sp, "rb") as f:
                    sha = hashlib.sha256(f.read()).hexdigest()
                scene_files[key] = {"path": sp, "sha256": sha}
            except Exception:
                pass

    if scene_files:
        manifest["scene_files"] = scene_files

    manifest_path = os.path.join(output_dir, "run_manifest.yaml")
    with open(manifest_path, "w") as f:
        yaml.dump(manifest, f, default_flow_style=False)
    rospy.loginfo("Manifest saved: %s", manifest_path)


# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Multi-frame calibration with PnP + Ceres BA")
    parser.add_argument("--num-groups", type=int, default=10,
                        help="number of sync frame groups to capture")
    parser.add_argument("--output", default="",
                        help="output directory (default: $CR5_DATA_ROOT/calibration/runs/<timestamp>)")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("multi_frame_calibration", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    # 默认输出路径: $CR5_DATA_ROOT/calibration/runs/<timestamp>
    if not args.output:
        data_root = os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data"))
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = os.path.join(data_root, "calibration", "runs", ts)
    os.makedirs(args.output, exist_ok=True)

    # P0-2/P1-3: 从 calibration_target.yaml 加载面板位姿 (权威来源)
    # fallback 仅用于 import 阶段; 正式标定 YAML 缺失直接 FAIL
    global FACE_POSES_TARGET, T_TARGET_FACE
    yaml_poses = _load_face_poses_from_yaml()
    if yaml_poses is None or len(yaml_poses) < 5:
        rospy.logerr("FATAL: calibration_target.yaml not found or incomplete. "
                     "Cannot run formal calibration with fallback poses.")
        sys.exit(1)
    FACE_POSES_TARGET = yaml_poses
    T_TARGET_FACE = {name: build_T_target_face(name) for name in FACE_POSES_TARGET}
    rospy.loginfo("Face poses loaded from YAML: %d faces", len(FACE_POSES_TARGET))

    # ── 从 YAML 覆盖面定义 (schema v2: tag_centers_face_m / marker_centers_face_m) ──
    # 加载完整 YAML 读取面级几何真值
    yaml_full = _load_full_yaml()
    if yaml_full:
        panels = yaml_full.get("panels", {})
        global APRILTAG_FACES, ARUCO_FACES
        # 左面 (AprilTag)
        left_cfg = panels.get("left", {})
        if left_cfg.get("tag_centers_face_m"):
            APRILTAG_FACES["left"] = {
                "tag_size": left_cfg["tag_size_m"],
                "tag_ids": left_cfg["tag_ids"],
                "face_frame": left_cfg["frame"],
                "positions": {int(k): tuple(v) for k, v
                             in left_cfg["tag_centers_face_m"].items()},
            }
            rospy.loginfo("Left face loaded from YAML: %d tags",
                          len(APRILTAG_FACES["left"]["tag_ids"]))
        # 顶面 (AprilTag)
        top_cfg = panels.get("top", {})
        if top_cfg.get("tag_centers_face_m"):
            APRILTAG_FACES["top"] = {
                "tag_size": top_cfg["tag_size_m"],
                "tag_ids": top_cfg["tag_ids"],
                "face_frame": top_cfg["frame"],
                "positions": {int(k): tuple(v) for k, v
                             in top_cfg["tag_centers_face_m"].items()},
            }
        # 右面 (ArUco)
        right_cfg = panels.get("right", {})
        if right_cfg.get("marker_centers_face_m"):
            ARUCO_FACES["right"] = {
                "marker_size_m": right_cfg["marker_size_m"],
                "marker_ids": right_cfg["tag_ids"],
                "dict_id": aruco.DICT_4X4_50,
                "face_frame": right_cfg["frame"],
                "positions": {int(k): tuple(v) for k, v
                             in right_cfg["marker_centers_face_m"].items()},
            }
            rospy.loginfo("Right face loaded from YAML: markers at %s",
                          {k: (v[0], v[1]) for k, v
                           in ARUCO_FACES["right"]["positions"].items()})

    # ── 等待 joint_capture_manager 服务 ──
    svc_name = "/joint_capture_manager/capture_sync_group"
    rospy.loginfo("Waiting for %s ...", svc_name)
    try:
        rospy.wait_for_service(svc_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Service %s not available. Start joint_capture_manager first.", svc_name)
        sys.exit(1)

    capture_svc = rospy.ServiceProxy(svc_name, Trigger)

    # ── 读取相机内参 ──
    from sensor_msgs.msg import CameraInfo
    camera_infos = {}
    for cam in CAMERAS:
        try:
            info = rospy.wait_for_message(
                "/{}/camera/color/camera_info".format(cam), CameraInfo, timeout=5.0)
            K = np.array(info.K).reshape(3, 3)
            D = np.array(info.D) if info.D else np.zeros(4)
            camera_infos[cam] = {"K": K.tolist(), "D": D.tolist(),
                                  "width": info.width, "height": info.height}
            rospy.loginfo("%s: K=[%.1f, %.1f] %dx%d",
                          cam, K[0, 0], K[1, 1], info.width, info.height)
        except Exception as e:
            rospy.logerr("%s CameraInfo failed: %s", cam, e)
            sys.exit(1)

    # ── 多帧采集循环 ──
    print("\n" + "=" * 60)
    print("  Multi-Frame Calibration Capture (Fixed P0 issues)")
    print("  Target: {} sync frame groups".format(args.num_groups))
    print("  Cameras: {}".format(CAMERAS))
    print("=" * 60 + "\n")

    accumulated = {
        "cameras": {},
        "observations": {},
    }

    # 相机初始位姿 (跨帧累积, 第一帧后固定)
    camera_initial_poses = {}
    rig_frame = "{}_color_optical_frame".format(CAMERAS[0])

    for group_idx in range(args.num_groups):
        print("\n--- Group {}/{} ---".format(group_idx + 1, args.num_groups))
        print("Move calibration target to a new position, then press ENTER...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            break

        # ── P0-1: 调用采集服务, 解析 group_dir ──
        print("Capturing sync group...")
        resp = capture_svc()
        if not resp.success:
            print("  FAIL: {}".format(resp.message))
            retry = input("  Retry? (y/n): ")
            if retry.lower() == 'y':
                resp = capture_svc()
            if not resp.success:
                print("  Skipping group {}".format(group_idx))
                continue

        # 从 response message 解析 group_dir
        group_dir = None
        msg = resp.message
        if msg.startswith("GROUP_DIR:"):
            parts = msg.split("|", 1)
            group_dir = parts[0].replace("GROUP_DIR:", "")
            clean_msg = parts[1] if len(parts) > 1 else msg
        else:
            clean_msg = msg
        print("  OK: {}".format(clean_msg))

        if group_dir is None or not os.path.isdir(group_dir):
            print("  ERROR: cannot determine group_dir from service response")
            continue

        # ── P0-1: 从保存的同步组读取图像 ──
        group_data = {}
        group_pass = True
        pnp_results = {}  # {cam: T_camera_target}

        for cam in CAMERAS:
            color_path = os.path.join(group_dir, cam, "color.png")
            if not os.path.exists(color_path):
                print("  {}: color.png not found at {}".format(cam, color_path))
                group_pass = False
                continue

            cv_img = cv2.imread(color_path)
            if cv_img is None:
                print("  {}: cv2.imread failed".format(cam))
                group_pass = False
                continue

            K = camera_infos[cam]["K"]
            D = camera_infos[cam]["D"]

            # 检测
            detection = detect_on_image(cv_img, K, D)

            # ── P0-4: 面板局部坐标 → calibration_target_frame → 合并 ──
            obj_pts_target, img_pts_raw, face_counts = \
                merge_face_detections_to_target(detection)

            total_corners = sum(face_counts.values())
            detected_faces = [k for k, v in face_counts.items() if v >= 4]
            face_str = ",".join(detected_faces) if detected_faces else "none"

            if total_corners < 4:
                print("  {}: {} corners (<4), faces=[{}] ✗".format(
                    cam, total_corners, face_str))
                group_pass = False
                continue

            # P0-5: 去畸变 → PnP → T_camera_target
            img_pts_undist = undistort_points(img_pts_raw, K, D)

            # P1-2: 强制验证 obj/img 点数一致
            if len(obj_pts_target) != len(img_pts_undist):
                print("  {}: obj/img count mismatch ({} vs {}), faces=[{}]".format(
                    cam, len(obj_pts_target), len(img_pts_undist), face_str))
                group_pass = False
                continue

            T_cam_target, rvec, tvec, pnp_stats = solve_pnp(
                obj_pts_target, img_pts_undist, K, None)  # D=None (已去畸变)

            if T_cam_target is None:
                print("  {}: {} corners, faces=[{}] — PnP FAIL: {}".format(
                    cam, total_corners, face_str, pnp_stats.get("error", "unknown")))
                group_pass = False
                continue

            pnp_results[cam] = T_cam_target

            status = "✓" if detected_faces else "✗"
            print("  {}: {} corners, faces=[{}], PnP RMSE={:.2f}px {} {}".format(
                cam, total_corners, face_str,
                pnp_stats.get("rmse_px", 99), status,
                "(undistorted)" if D is not None and np.any(np.array(D) != 0) else ""))

            # ── P0-2: 存储合并后的相机级观测 ──
            group_data[cam] = {
                "object_points_3d": obj_pts_target,
                "image_points_2d": img_pts_undist,  # 去畸变后
                "corner_count": total_corners,
                "face_counts": face_counts,
                "pnp_stats": pnp_stats,
            }

        if not group_pass:
            print("  Group {}: FAIL (not all cameras passed)".format(group_idx + 1))
            continue

        # ── P0-5: 计算 T_rig_camera 初值 ──
        T_rig_cameras, T_rig_target = compute_rig_poses(pnp_results)
        if T_rig_cameras is None:
            print("  Group {}: FAIL (cannot compute rig poses)".format(group_idx + 1))
            continue

        # P1-1: 第一个有效组初始化相机 (而非仅 group_idx==0)
        if not camera_initial_poses:
            for cam in CAMERAS:
                T_rc = T_rig_cameras.get(cam)
                if T_rc is not None:
                    rv, tv = T_to_rvec_tvec(T_rc)
                    camera_initial_poses[cam] = {
                        "initial_pose": T_to_quat_trans(T_rc),
                        "rvec_init": rv,
                        "tvec_init": tv,
                    }

        # 存储目标初始位姿 (T_rig_target)
        group_data["target_initial_pose"] = T_to_quat_trans(T_rig_target)

        # ── P0-3: 使用整数 group ID ──
        accumulated["observations"][group_idx] = group_data
        print("  Group {}: PASS ({} cameras, target pose initialized)".format(
            group_idx + 1, len(pnp_results)))

    # ── 填写相机信息 (含初值) ──
    for cam in CAMERAS:
        cam_entry = {
            "K": camera_infos[cam]["K"],
            "D": camera_infos[cam]["D"],
            "width": camera_infos[cam]["width"],
            "height": camera_infos[cam]["height"],
        }
        if cam in camera_initial_poses:
            cam_entry.update(camera_initial_poses[cam])
        else:
            # fallback: 恒等初值
            cam_entry["initial_pose"] = [1, 0, 0, 0, 0, 0, 0]
            cam_entry["rvec_init"] = [0, 0, 0]
            cam_entry["tvec_init"] = [0, 0, 0]
        accumulated["cameras"][cam] = cam_entry

    # ── 保存累积观测 ──
    obs_path = os.path.join(args.output, "accumulated_observations.yaml")
    with open(obs_path, "w") as f:
        yaml.dump(accumulated, f, default_flow_style=False)
    print("\nObservations saved: {} ({} groups)".format(
        obs_path, len(accumulated["observations"])))

    # ── V6: 保存 run_manifest.yaml (数据来源追踪) ──
    _save_run_manifest(args.output, n_groups)

    # ── 运行 Bundle Adjustment ──
    n_groups = len(accumulated["observations"])
    if n_groups < 2:
        print("Only {} groups with valid detections. Need >= 2 for BA.".format(n_groups))
        sys.exit(1)

    print("\nRunning Bundle Adjustment ({} groups)...".format(n_groups))

    import subprocess
    ba_script = os.path.join(os.path.dirname(__file__), "bundle_adjustment.py")
    ba_dir = os.path.join(args.output, "ba_extrinsics")
    result = subprocess.run(
        [sys.executable, ba_script,
         "--observations", obs_path,
         "--output", ba_dir],
        capture_output=False, timeout=120)

    if result.returncode == 0:
        print("\n" + "=" * 60)
        print("  CALIBRATION COMPLETE")
        print("  Rig frame: {}".format(rig_frame))
        print("  Extrinsics: {}".format(
            os.path.join(ba_dir, "initial_extrinsics.yaml")))
        print("=" * 60)
    else:
        print("\nBA failed with code {}".format(result.returncode))


if __name__ == "__main__":
    main()

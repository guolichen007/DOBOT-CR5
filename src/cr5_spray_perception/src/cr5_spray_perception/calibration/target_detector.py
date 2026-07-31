#!/usr/bin/env python3
"""统一 target detector — 所有标定脚本的唯一检测实现.

提供按 pattern 类型 (ChArUco / ArUco / AprilTag) 分离的 detector profile,
以及统一的 detect_target() 接口.

Profile 选择规则 (不使用 Gazebo truth):
  - 检测稳定 (per-marker consistency)
  - 角点 temporal jitter 小
  - reprojection consistency 好
  - multi-view consistency 好

Gazebo truth 仅用于最后验证 profile 确实解决了系统偏差.

使用:
    from cr5_spray_perception.calibration.target_detector import (
        DetectorProfiles, detect_target, create_default_profiles,
    )
"""

import numpy as np
import cv2
from cv2 import aruco

from cr5_spray_perception import aruco_compat


# ── Detector Profile 定义 ──

class DetectorProfiles:
    """按 pattern 类型分离的 detector 参数."""

    __slots__ = (
        "charuco",    # dict — ChArUco 参数
        "aruco",      # dict — ArUco 参数
        "apriltag",   # dict — AprilTag 参数
    )

    def __init__(self, charuco=None, aruco=None, apriltag=None):
        self.charuco = charuco or {}
        self.aruco = aruco or {}
        self.apriltag = apriltag or {}

    def to_dict(self):
        return {
            "charuco": dict(self.charuco),
            "aruco": dict(self.aruco),
            "apriltag": dict(self.apriltag),
        }

    def profile_summary(self):
        """人类可读的 profile 摘要 (用于 manifest)."""
        lines = []
        for pattern in ("charuco", "aruco", "apriltag"):
            p = getattr(self, pattern)
            ref = p.get("corner_refinement", "SUBPIX")
            lines.append(f"{pattern}: refinement={ref}")
        return "; ".join(lines)


def create_default_profiles() -> DetectorProfiles:
    """创建默认 detector profiles (Gazebo 当前最佳已知配置).

    ChArUco: 保留 SUBPIX (棋盘格角点 refine 行为与 ArUco 不同).
    ArUco: 使用 NONE (V8.8 因果证明 — SUBPIX 产生 4-7% 边缘尺度膨胀).
    AprilTag: 保留 SUBPIX (当前策略不变).

    实机部署时必须重新比较 refinement 选项.
    """
    return DetectorProfiles(
        charuco={
            "corner_refinement": aruco.CORNER_REFINE_SUBPIX,
            "corner_refinement_name": "SUBPIX",
            # Relaxed params for oblique ChArUco detection
            "adaptiveThreshWinSizeMin": 3,
            "adaptiveThreshWinSizeMax": 23,
            "minMarkerPerimeterRate": 0.01,
            "polygonalApproxAccuracyRate": 0.05,
        },
        aruco={
            # V8.8/V8.9: NONE eliminates 4-7% edge scale expansion
            # on Gazebo-resolution ArUco markers (~30px edges).
            "corner_refinement": aruco.CORNER_REFINE_NONE,
            "corner_refinement_name": "NONE",
            "adaptiveThreshWinSizeMin": 3,
            "minMarkerPerimeterRate": 0.01,
            "polygonalApproxAccuracyRate": 0.05,
        },
        apriltag={
            "corner_refinement": aruco.CORNER_REFINE_SUBPIX,
            "corner_refinement_name": "SUBPIX",
            "adaptiveThreshWinSizeMin": 3,
            "minMarkerPerimeterRate": 0.01,
            "polygonalApproxAccuracyRate": 0.05,
        },
    )


# ── _make_detector_params ──

def _make_detector_params(profile):
    """从 profile dict 构造 aruco.DetectorParameters."""
    params = aruco_compat.detector_parameters()
    if "corner_refinement" in profile:
        params.cornerRefinementMethod = profile["corner_refinement"]
    if "adaptiveThreshWinSizeMin" in profile:
        params.adaptiveThreshWinSizeMin = profile["adaptiveThreshWinSizeMin"]
    if "adaptiveThreshWinSizeMax" in profile:
        params.adaptiveThreshWinSizeMax = profile["adaptiveThreshWinSizeMax"]
    if "minMarkerPerimeterRate" in profile:
        params.minMarkerPerimeterRate = profile["minMarkerPerimeterRate"]
    if "polygonalApproxAccuracyRate" in profile:
        params.polygonalApproxAccuracyRate = profile["polygonalApproxAccuracyRate"]
    return params


# ── 各 pattern 检测函数 ──

def _detect_charuco(gray_img, K, D, charuco_faces, profile):
    """检测所有 ChArUco 面板.

    Args:
        gray_img: 灰度图像.
        K: 相机内参 (3x3).
        D: 畸变系数.
        charuco_faces: {face_name: dict} — 来自 TargetGeometry.charuco_faces.
        profile: dict — ChArUco detector profile.

    Returns:
        {face_name: {"object_points_3d_face": [...], "image_points_2d": [...],
                      "corner_count": int}}
    """
    results = {}
    if not charuco_faces:
        return results

    params = _make_detector_params(profile)

    # 单个 dictionary 检测所有 ChArUco markers
    # 使用第一个 face 的 dictionary (所有 ChArUco faces 用相同 dict)
    first_face = next(iter(charuco_faces.values()))
    board_ref = first_face["board"]

    corners, ids, _ = aruco_compat.detect_markers(gray_img, board_ref.dictionary, params)

    for fk, fc in charuco_faces.items():
        board = fc["board"]
        id_start = fc["id_start"]
        obj_pts_face, img_pts_face = [], []

        if ids is not None and len(ids) > 0:
            ids_flat = [int(i) for i in ids.flatten()]
            idx_list, local_ids = aruco_compat.remap_custom_ids(ids_flat, id_start, board)
            if len(idx_list) >= 2:
                local_corners = tuple(corners[i] for i in idx_list)
                cc, cids = aruco_compat.interpolate_charuco_corners(
                    local_corners, local_ids, gray_img, board,
                    cameraMatrix=K, distCoeffs=D)
                if cids is not None and len(cids) >= 4:
                    board_pts = np.asarray(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)
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

    return results


def _detect_aruco(gray_img, aruco_faces, profile):
    """检测所有 ArUco 面板.

    Args:
        gray_img: 灰度图像.
        aruco_faces: {face_name: dict} — 来自 TargetGeometry.aruco_faces.
        profile: dict — ArUco detector profile.

    Returns:
        {face_name: {"object_points_3d_face": [...], "image_points_2d": [...],
                      "corner_count": int}}
    """
    results = {}
    if not aruco_faces:
        return results

    params = _make_detector_params(profile)

    # 收集所有 ArUco face 用的 dictionary (按 dict_id 分组)
    dict_groups = {}
    for fk, fc in aruco_faces.items():
        dict_id = fc.get("dict_id", aruco.DICT_4X4_50)
        if dict_id not in dict_groups:
            dict_groups[dict_id] = {"faces": {}, "dictionary": aruco.getPredefinedDictionary(dict_id)}
        dict_groups[dict_id]["faces"][fk] = fc

    for dict_id, dg in dict_groups.items():
        corners, ids, _ = aruco_compat.detect_markers(gray_img, dg["dictionary"], params)

        for fk, fc in dg["faces"].items():
            obj_pts_face, img_pts_face = [], []
            if ids is not None and len(ids) > 0:
                ids_flat = [int(i) for i in ids.flatten()]
                for i, tid in enumerate(ids_flat):
                    if tid not in fc["marker_ids"]:
                        continue
                    pos = fc["positions"][tid]
                    half = fc["marker_size_m"] / 2.0
                    marker_obj = [
                        [pos[0] - half, pos[1] + half, 0],
                        [pos[0] + half, pos[1] + half, 0],
                        [pos[0] + half, pos[1] - half, 0],
                        [pos[0] - half, pos[1] - half, 0],
                    ]
                    obj_pts_face.extend(marker_obj)
                    img_pts_face.extend(corners[i][0].tolist())

            results[fk] = {
                "object_points_3d_face": obj_pts_face,
                "image_points_2d": img_pts_face,
                "corner_count": len(obj_pts_face),
            }

    return results


def _detect_apriltag(gray_img, apriltag_faces, profile):
    """检测所有 AprilTag 面板.

    Args:
        gray_img: 灰度图像.
        apriltag_faces: {face_name: dict} — 来自 TargetGeometry.apriltag_faces.
        profile: dict — AprilTag detector profile.

    Returns:
        {face_name: {"object_points_3d_face": [...], "image_points_2d": [...],
                      "corner_count": int}}
    """
    results = {}
    if not apriltag_faces:
        return results

    params = _make_detector_params(profile)
    tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    corners, ids, _ = aruco_compat.detect_markers(gray_img, tag_dict, params)

    for fk, fc in apriltag_faces.items():
        obj_pts_face, img_pts_face = [], []
        if ids is not None and len(ids) > 0:
            ids_flat = [int(i) for i in ids.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["tag_ids"]:
                    continue
                pos = fc["positions"][tid]
                half = fc["tag_size"] / 2.0
                tag_obj = [
                    [pos[0] - half, pos[1] + half, 0],
                    [pos[0] + half, pos[1] + half, 0],
                    [pos[0] + half, pos[1] - half, 0],
                    [pos[0] - half, pos[1] - half, 0],
                ]
                obj_pts_face.extend(tag_obj)
                img_pts_face.extend(corners[i][0].tolist())

        results[fk] = {
            "object_points_3d_face": obj_pts_face,
            "image_points_2d": img_pts_face,
            "corner_count": len(obj_pts_face),
        }

    return results


# ── 统一检测入口 ──

def detect_target(cv_img, K, D, target_geometry, profiles=None):
    """对单张图像检测所有标定靶面板.

    Args:
        cv_img: BGR 彩色图像 (np.ndarray, H×W×3).
        K: 相机内参 (3×3 np.ndarray).
        D: 畸变系数 (np.ndarray, N).
        target_geometry: TargetGeometry 实例.
        profiles: DetectorProfiles 实例. None 时使用默认.

    Returns:
        {face_name: {"object_points_3d_face": [...], "image_points_2d": [...],
                      "corner_count": int}}
    """
    if profiles is None:
        profiles = create_default_profiles()

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    results = {}

    # ChArUco
    charuco_results = _detect_charuco(gray, K, D, target_geometry.charuco_faces,
                                      profiles.charuco)
    results.update(charuco_results)

    # ArUco
    aruco_results = _detect_aruco(gray, target_geometry.aruco_faces, profiles.aruco)
    results.update(aruco_results)

    # AprilTag
    apriltag_results = _detect_apriltag(gray, target_geometry.apriltag_faces,
                                        profiles.apriltag)
    results.update(apriltag_results)

    return results

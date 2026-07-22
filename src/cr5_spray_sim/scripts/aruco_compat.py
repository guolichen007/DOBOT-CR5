#!/usr/bin/env python3
"""
OpenCV 4.2 / 新 OpenCV ArUco API 双兼容层.

Ubuntu 20.04 / ROS Noetic 使用 OpenCV 4.2，不提供:
- cv2.aruco.ArucoDetector
- cv2.aruco.CharucoDetector

新版本 OpenCV (≥4.7) 提供这些类。

本模块统一封装，脚本只需调用本模块的函数。
"""
import cv2
import numpy as np
from cv2 import aruco

_OPENCV_VERSION = cv2.__version__
_HAS_ARUCO_DETECTOR = hasattr(aruco, "ArucoDetector")
_HAS_CHARUCO_DETECTOR = hasattr(aruco, "CharucoDetector")


def get_opencv_info():
    return {
        "version": _OPENCV_VERSION,
        "has_aruco_detector": _HAS_ARUCO_DETECTOR,
        "has_charuco_detector": _HAS_CHARUCO_DETECTOR,
    }


def detector_parameters():
    """返回兼容的 DetectorParameters."""
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def detect_markers(image, dictionary, parameters=None):
    """
    检测 ArUco markers (兼容新旧 API).

    返回: (corners, ids, rejected)
    """
    params = parameters or detector_parameters()

    if _HAS_ARUCO_DETECTOR:
        detector = aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(image)

    return aruco.detectMarkers(image, dictionary, parameters=params)


def interpolate_charuco_corners(marker_corners, marker_ids, image, board,
                                cameraMatrix=None, distCoeffs=None):
    """
    ChArUco 角点插值 (兼容新旧 API).

    marker_ids: 必须是 board 的 local ID (0-based), 不是自定义 ID.
    """
    if _HAS_CHARUCO_DETECTOR:
        detector = aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
        return charuco_corners, charuco_ids

    # Legacy API
    kwargs = {}
    if cameraMatrix is not None:
        kwargs["cameraMatrix"] = cameraMatrix
    if distCoeffs is not None:
        kwargs["distCoeffs"] = distCoeffs

    retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, image, board, **kwargs)
    return charuco_corners, charuco_ids


def draw_detected_markers(image, corners, ids, border_color=(0, 255, 0)):
    """绘制检测到的 markers."""
    if corners is None or ids is None:
        return image
    return aruco.drawDetectedMarkers(image.copy(), corners, ids, border_color)


def draw_charuco_corners(image, charuco_corners, charuco_ids,
                         corner_color=(255, 0, 0)):
    """绘制 ChArUco corners."""
    if charuco_corners is None or charuco_ids is None:
        return image
    return aruco.drawDetectedCornersCharuco(
        image.copy(), charuco_corners, charuco_ids, corner_color)


def remap_custom_ids(raw_ids_flat, id_start, board):
    """
    将自定义 Marker ID 映射为 board local ID.

    raw_ids_flat: 检测到的所有 marker ID (如 [100, 101, ..., 123])
    id_start: 此面板的起始 ID (100, 200, 或 300)
    board: CharucoBoard 对象 (local IDs 0-based)

    返回: (selected_indices, local_ids_array, matched_corners_indices)
    """
    board_n_markers = int(np.asarray(board.ids).size)
    id_end = id_start + board_n_markers - 1

    selected = []
    for i, raw_id in enumerate(raw_ids_flat):
        if id_start <= raw_id <= id_end:
            selected.append((i, raw_id))

    if not selected:
        return [], None, []

    idx_list = [s[0] for s in selected]
    local_ids = np.array(
        [[s[1] - id_start] for s in selected],
        dtype=np.int32,
    )
    return idx_list, local_ids


def log_capability():
    """打印 OpenCV ArUco 能力."""
    info = get_opencv_info()
    import rospy
    rospy.loginfo("OpenCV version: %s", info["version"])
    rospy.loginfo("ArucoDetector: %s", info["has_aruco_detector"])
    rospy.loginfo("CharucoDetector: %s", info["has_charuco_detector"])

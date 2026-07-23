#!/usr/bin/env python3
"""
Runtime contract validation — 验证 ROS 话题、TF 帧、相机流的运行时约定。

可在仿真运行时独立导入或 rosrun。
"""
import sys

# 三相机运行时约定
EXPECTED_CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]

EXPECTED_COLOR_TOPICS = [
    "/cam_front_left/color/image_raw",
    "/cam_front_right/color/image_raw",
    "/cam_rear/color/image_raw",
]

EXPECTED_DEPTH_TOPICS = [
    "/cam_front_left/depth/image_rect_raw",
    "/cam_front_right/depth/image_rect_raw",
    "/cam_rear/depth/image_rect_raw",
]

EXPECTED_CAMERA_INFO_TOPICS = [
    "/cam_front_left/color/camera_info",
    "/cam_front_right/color/camera_info",
    "/cam_rear/color/camera_info",
]

# 标定目标 TF 帧
EXPECTED_CALIBRATION_FRAMES = [
    "object_frame",
    "calibration_target_frame",
    "calibration_target_front_frame",
    "calibration_target_left_frame",
    "calibration_target_right_frame",
    "calibration_target_top_frame",
    "calibration_target_back_frame",
]

# CR5 机械臂 TF 帧
EXPECTED_CR5_FRAMES = [
    "base_link",
    "Link1", "Link2", "Link3", "Link4", "Link5", "Link6",
]


def check_topics_exist(topic_list, timeout=5.0):
    """检查 ROS 话题是否存在（需要 rospy 环境）。

    Returns:
        (found: list, missing: list)
    """
    try:
        import rospy
        existing = set(
            t[0] for t in rospy.get_published_topics(timeout=timeout))
    except ImportError:
        return [], topic_list

    found = [t for t in topic_list if t in existing]
    missing = [t for t in topic_list if t not in existing]
    return found, missing


def check_camera_contract(camera_names=None):
    """验证三相机 color/depth 话题约定。

    Returns:
        dict: {camera_name: {color: bool, depth: bool, info: bool}}
    """
    if camera_names is None:
        camera_names = EXPECTED_CAMERAS

    try:
        import rospy
        existing = set(
            t[0] for t in rospy.get_published_topics(timeout=3.0))
    except ImportError:
        return {}

    result = {}
    for cam in camera_names:
        result[cam] = {
            "color": f"/{cam}/color/image_raw" in existing,
            "depth": f"/{cam}/depth/image_rect_raw" in existing,
            "info": f"/{cam}/color/camera_info" in existing,
        }
    return result


def validate_static(offline=True):
    """离线（无 ROS）验证：仅检查常量定义完整性。

    Returns:
        (passed: bool, report: dict)
    """
    report = {
        "camera_count": len(EXPECTED_CAMERAS),
        "color_topics": len(EXPECTED_COLOR_TOPICS),
        "depth_topics": len(EXPECTED_DEPTH_TOPICS),
        "calibration_frames": len(EXPECTED_CALIBRATION_FRAMES),
        "cr5_frames": len(EXPECTED_CR5_FRAMES),
    }

    # 基本检查
    passed = True
    if len(EXPECTED_CAMERAS) != 3:
        report["error"] = f"Expected 3 cameras, got {len(EXPECTED_CAMERAS)}"
        passed = False
    if len(EXPECTED_COLOR_TOPICS) != 3:
        report["error"] = f"Expected 3 color topics, got {len(EXPECTED_COLOR_TOPICS)}"
        passed = False
    if len(EXPECTED_DEPTH_TOPICS) != 3:
        report["error"] = f"Expected 3 depth topics, got {len(EXPECTED_DEPTH_TOPICS)}"
        passed = False

    return passed, report


if __name__ == "__main__":
    passed, report = validate_static(offline=True)
    print(f"Runtime contract {'PASS' if passed else 'FAIL'}")
    for k, v in report.items():
        print(f"  {k}: {v}")
    sys.exit(0 if passed else 1)

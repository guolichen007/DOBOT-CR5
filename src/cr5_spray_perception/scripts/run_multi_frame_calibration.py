#!/usr/bin/env python3
"""
多帧标定采集 + PnP + Bundle Adjustment 工作流.

交互式引导: 移动标定目标 → 采集同步帧组 → 检测 ChArUco/AprilTag → PnP.
累计 N 组后自动运行 Ceres Bundle Adjustment, 输出精化外参.

用法:
  rosrun cr5_spray_perception run_multi_frame_calibration.py \
    --num-groups 10 --output artifacts/calibration
"""
import sys, os, json, time, math, argparse
import yaml, numpy as np
import rospy
from std_srvs.srv import Trigger
from datetime import datetime

# 复用 estimate_camera_extrinsics 的检测逻辑
_est_path = os.path.join(os.path.dirname(__file__),
                         "estimate_camera_extrinsics.py")

# 直接导入检测函数 (避免重复定义)
import cv2
from cv2 import aruco
from cr5_spray_perception import aruco_compat

# 面板定义 (与 estimate_camera_extrinsics.py 保持一致)
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

APRILTAG_FACES = {
    "right": {"tag_size": 0.07, "tag_ids": [4,5,6,7],
              "face_frame": "calibration_target_right_frame",
              "positions": {4:(-0.0425,0.0425,0), 5:(0.0425,0.0425,0),
                           6:(-0.0425,-0.0425,0), 7:(0.0425,-0.0425,0)}},
    "top":   {"tag_size": 0.12, "tag_ids": [8],
              "face_frame": "calibration_target_top_frame",
              "positions": {8:(0,0,0)}},
}

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]

# 预创建 Charuco boards
for v in CHARUCO_FACES.values():
    v["board"] = aruco.CharucoBoard_create(
        v["sx"], v["sy"], v["sq_m"], v["mk_m"],
        aruco.getPredefinedDictionary(v["dict_id"]))


def detect_on_image(cv_img, K, D):
    """对单张图像检测所有 ChArUco/AprilTag 面."""
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    results = {}

    for fk, fc in CHARUCO_FACES.items():
        board = fc["board"]
        id_start = fc["id_start"]
        params = aruco_compat.detector_parameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejected = aruco_compat.detect_markers(
            gray, board.dictionary, params)

        obj_pts_list, img_pts_list = [], []
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
                    obj_pts = np.array([board_pts[i] for i in cids_flat],
                                       dtype=np.float32)
                    img_pts = cc.reshape(-1, 2).astype(np.float32)
                    obj_pts_list = obj_pts.tolist()
                    img_pts_list = img_pts.tolist()

        results[fk] = {
            "object_points_3d": obj_pts_list,
            "image_points_2d": img_pts_list,
            "corner_count": len(obj_pts_list),
        }

    # AprilTag
    tag_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(
        gray, tag_dict, params)

    for fk, fc in APRILTAG_FACES.items():
        obj_pts_list, img_pts_list = [], []
        if ids is not None:
            ids_flat = [int(i) for i in ids.flatten()]
            for i, tid in enumerate(ids_flat):
                if tid not in fc["tag_ids"]:
                    continue
                pos = fc["positions"][tid]
                half = fc["tag_size"] / 2.0
                tag_obj = np.array([
                    [pos[0]-half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]+half, 0],
                    [pos[0]+half, pos[1]-half, 0],
                    [pos[0]-half, pos[1]-half, 0],
                ], dtype=np.float32)
                obj_pts_list.extend(tag_obj.tolist())
                img_pts_list.extend(corners[i][0].tolist())

        results[fk] = {
            "object_points_3d": obj_pts_list,
            "image_points_2d": img_pts_list,
            "corner_count": len(obj_pts_list),
        }

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-groups", type=int, default=10,
                        help="number of frame groups to capture")
    parser.add_argument("--output", default="artifacts/calibration",
                        help="output directory")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("multi_frame_calibration", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    os.makedirs(args.output, exist_ok=True)

    # 等待服务
    svc_name = "/joint_capture_manager/capture_sync_group"
    rospy.loginfo("Waiting for %s ...", svc_name)
    try:
        rospy.wait_for_service(svc_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Service %s not available. Start joint_capture_manager first.", svc_name)
        sys.exit(1)

    capture_svc = rospy.ServiceProxy(svc_name, Trigger)

    # 累积观测
    accumulated = {
        "cameras": {},
        "observations": {},
    }

    # 初始化相机内参
    for cam in CAMERAS:
        try:
            from sensor_msgs.msg import CameraInfo
            info = rospy.wait_for_message(
                "/{}/camera/color/camera_info".format(cam), CameraInfo, timeout=5.0)
            K = np.array(info.K).reshape(3, 3)
            D = np.array(info.D) if info.D else np.zeros(4)
            accumulated["cameras"][cam] = {
                "K": K.tolist(),
                "D": D.tolist(),
                "rvec_init": [0, 0, 0],
                "tvec_init": [0, 0, 0],
            }
            rospy.loginfo("%s: K=[%.1f, %.1f] %dx%d",
                          cam, K[0,0], K[1,1], info.width, info.height)
        except Exception as e:
            rospy.logerr("%s CameraInfo failed: %s", cam, e)
            sys.exit(1)

    # ── 多帧采集循环 ──
    print("\n" + "=" * 60)
    print("  Multi-Frame Calibration Capture")
    print("  Target: {} sync frame groups".format(args.num_groups))
    print("  Cameras: {}".format(CAMERAS))
    print("=" * 60 + "\n")

    for group_idx in range(args.num_groups):
        print("\n--- Group {}/{} ---".format(group_idx + 1, args.num_groups))
        print("Move calibration target to a new position, then press ENTER...")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            break

        # 采集同步帧组
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
        print("  OK: {}".format(resp.message))

        # PnP 检测 (从已保存的图像中检测)
        group_data = {}
        group_pass = True

        for cam in CAMERAS:
            # 读取刚保存的图像
            import glob
            group_dirs = sorted(glob.glob(
                os.path.join(os.path.expanduser("~/cr5_spray_data"),
                             "*", "groups", "group_*")))
            # 直接用 rostopic 获取最新图像
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            bridge = CvBridge()
            try:
                color_msg = rospy.wait_for_message(
                    "/{}/camera/color/image_raw".format(cam), Image, timeout=5.0)
                cv_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
            except Exception as e:
                print("  {}: image failed - {}".format(cam, e))
                group_pass = False
                continue

            K = np.array(accumulated["cameras"][cam]["K"]).reshape(3, 3)
            D = np.array(accumulated["cameras"][cam].get("D", [0,0,0,0]))

            detection = detect_on_image(cv_img, K, D)

            total_corners = sum(
                d.get("corner_count", 0) for d in detection.values())
            detected_faces = [k for k, d in detection.items()
                            if d.get("corner_count", 0) >= 4]
            group_data[cam] = detection
            face_str = ",".join(detected_faces) if detected_faces else "none"
            status = "✓" if detected_faces else "✗"
            print("  {}: {} corners, faces=[{}] {}".format(
                cam, total_corners, face_str, status))
            if not detected_faces:
                group_pass = False

        # 保存组数据
        if group_pass:
            group_id = "group_{:04d}".format(group_idx)
            accumulated["observations"][group_id] = group_data
            print("  Group {}: PASS".format(group_idx + 1))
        else:
            print("  Group {}: FAIL (not all cameras detected)".format(group_idx + 1))

    # ── 保存累积观测 ──
    obs_path = os.path.join(args.output, "accumulated_observations.yaml")
    with open(obs_path, "w") as f:
        yaml.dump(accumulated, f, default_flow_style=False)
    print("\nObservations saved: {}".format(obs_path))

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
        print("  Extrinsics: {}".format(
            os.path.join(ba_dir, "initial_extrinsics.yaml")))
        print("=" * 60)
    else:
        print("\nBA failed with code {}".format(result.returncode))


if __name__ == "__main__":
    main()

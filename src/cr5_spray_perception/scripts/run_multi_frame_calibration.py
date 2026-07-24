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

    确保单一真值来源. 如果 YAML 不可用则退回硬编码值.
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
                    rospy.loginfo("Loaded %d face poses from %s (SHA: ...)",
                                  len(poses), p)
                    return poses
            except Exception as e:
                rospy.logwarn("Failed to load face poses from %s: %s", p, e)

    rospy.logwarn("Using fallback hardcoded face poses (YAML not found)")
    return dict(_FALLBACK_FACE_POSES)


FACE_POSES_TARGET = None  # 延迟加载


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


def solve_pnp(obj_pts, img_pts, K, D):
    """PnP: 计算 T_camera_target (相机在 target 坐标系中的位姿).

    OpenCV solvePnP 返回的 rvec/tvec 满足:
      p_camera = R * p_target + t
    即 T_camera_target: 将 target 系 3D 点变换到 camera 系

    Returns:
        T_cam_target: 4x4 变换矩阵 (p_camera = T_cam_target @ p_target)
        rvec, tvec: OpenCV 格式
        stats: dict with inliers, rmse_px
        None 如果失败
    """
    if len(obj_pts) < 4:
        return None, None, None, None

    obj = np.array(obj_pts, dtype=np.float32).reshape(-1, 3)
    img = np.array(img_pts, dtype=np.float32).reshape(-1, 2)
    K_arr = np.array(K, dtype=np.float64).reshape(3, 3)
    D_arr = np.array(D, dtype=np.float64).reshape(-1) if D is not None else np.zeros(4)

    # EPNP + RANSAC
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj, img, K_arr, D_arr,
        flags=cv2.SOLVEPNP_EPNP, reprojectionError=3.0,
        confidence=0.99, iterationsCount=100)
    if not ok or inliers is None or len(inliers) < 4:
        return None, None, None, {"error": "PnP RANSAC failed"}

    # LM 精化
    rvec, tvec = cv2.solvePnPRefineLM(
        obj, img, K_arr, D_arr, rvec, tvec,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))

    # 重投影误差
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K_arr, D_arr)
    errors = np.linalg.norm(img - proj.reshape(-1, 2), axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    return T, rvec, tvec, {
        "inliers": int(len(inliers)), "n_pts": len(obj_pts),
        "rmse_px": rmse, "max_error_px": float(np.max(errors)),
    }


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
# 主流程
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Multi-frame calibration with PnP + Ceres BA")
    parser.add_argument("--num-groups", type=int, default=10,
                        help="number of sync frame groups to capture")
    parser.add_argument("--output", default="artifacts/calibration",
                        help="output directory")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("multi_frame_calibration", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    os.makedirs(args.output, exist_ok=True)

    # P1-3: 从 calibration_target.yaml 加载面板位姿 (替代硬编码)
    global FACE_POSES_TARGET, T_TARGET_FACE
    FACE_POSES_TARGET = _load_face_poses_from_yaml()
    T_TARGET_FACE = {name: build_T_target_face(name) for name in FACE_POSES_TARGET}

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

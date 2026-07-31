#!/usr/bin/env python3
"""
三相机标定 — 正式算法入口 (稳定版 V1).

从已采集的标定数据集计算三相机外参:
  1. 标定靶检测 (ChArUco / ArUco / AprilTag)
  2. 单相机 PnP 求解 (T_camera_target)
  3. 成对相机相对变换 (pairwise T_camA_camB)
  4. RANSAC 鲁棒共识
  5. SE(3) 加权平均 → 最终外参

前左相机 (cam_front_left) 固定为 rig 基准 (gauge), 输出 FL→FR 和 FL→RE 的变换.

本模块禁止导入 Gazebo / TF truth, 仅接受标定图像和配置.
"""
import sys, os, math, json, hashlib, argparse
import numpy as np
import cv2

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.calibration.target_geometry import load_target_geometry
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.pairwise_solver import (
    compute_pairwise_rig, CAMERAS, FIRST_CAM)
from cr5_spray_perception.calibration.pnp_solver import solve_pnp

DATA_ROOT = os.path.join(os.environ.get("CR5_DATA_ROOT",
    os.path.expanduser("~/cr5_data")), "calibration")


def build_pnp_from_dataset(dataset_dir, geom, profiles, camera_info):
    """从数据集目录构建 per-group PnP 结果.

    Args:
        dataset_dir: 包含 group_XXXX/ 子目录的路径.
        geom: TargetGeometry 实例.
        profiles: DetectorProfiles 实例.
        camera_info: {cam_name: {"K": ndarray, "D": ndarray}}.

    Returns:
        {group_id: {cam_name: 4x4 T_camera_target}}
    """
    per_group_pnp = {}
    group_dirs = sorted([
        d for d in os.listdir(dataset_dir)
        if d.startswith("group_") and os.path.isdir(os.path.join(dataset_dir, d))
    ])

    for gdir_name in group_dirs:
        gid = int(gdir_name.split("_")[1])
        group_dir = os.path.join(dataset_dir, gdir_name)

        # 确定图像来源: 优先 selected/, 回退 raw/snap_0/
        sel_file = os.path.join(group_dir, "selected_snap.txt")
        if os.path.isfile(sel_file):
            snap_idx = int(open(sel_file).read().strip())
            img_dir = os.path.join(group_dir, "raw", f"snap_{snap_idx}")
        else:
            img_dir = os.path.join(group_dir, "selected")
        if not os.path.isdir(img_dir):
            img_dir = group_dir

        group_pnp = {}
        for cam in CAMERAS:
            img_path = os.path.join(img_dir, cam, "color.png")
            if not os.path.exists(img_path):
                continue
            cv_img = cv2.imread(img_path)
            if cv_img is None:
                continue

            K = camera_info[cam]["K"]
            D = camera_info[cam]["D"]
            detection = detect_target(cv_img, K, D, geom, profiles)
            if not detection:
                continue

            obj_pts, img_pts = [], []
            for face_name, face_data in detection.items():
                oface = face_data.get("object_points_3d_face", [])
                iface = face_data.get("image_points_2d", [])
                if not oface:
                    continue
                T_face = geom.T_target_face.get(face_name)
                if T_face is None:
                    continue
                for pi in range(len(oface)):
                    pt_tgt = (T_face @ np.array([*oface[pi], 1.0]))[:3]
                    obj_pts.append(pt_tgt)
                    img_pts.append(iface[pi])

            if len(obj_pts) >= 4:
                T_est, _, _, _ = solve_pnp(
                    np.array(obj_pts), np.array(img_pts), K, D)
                if T_est is not None:
                    group_pnp[cam] = T_est

        if len(group_pnp) >= 2:
            per_group_pnp[gid] = group_pnp

    return per_group_pnp


def calibrate(dataset_dir, camera_info, output_path=None):
    """执行三相机标定.

    Args:
        dataset_dir: 数据集目录路径.
        camera_info: {cam_name: {"K": ndarray, "D": ndarray}}.
        output_path: 可选的输出 JSON 路径.

    Returns:
        dict: 包含 cameras, pair_stats, triangle_closure 的结果.
    """
    geom = load_target_geometry()
    profiles = create_default_profiles()

    per_group_pnp = build_pnp_from_dataset(
        dataset_dir, geom, profiles, camera_info)

    if len(per_group_pnp) < 5:
        return {
            "error": "数据集组数不足 (需要 >=5, 实际 {})".format(len(per_group_pnp)),
            "status": "INSUFFICIENT_DATA",
        }

    X_cameras, report = compute_pairwise_rig(per_group_pnp)

    # 有效性检查: 禁止静默填充 identity
    missing_cameras = [cam for cam in CAMERAS if cam not in X_cameras]
    if missing_cameras:
        return {
            "error": "相机外参缺失: {}".format(missing_cameras),
            "status": "CALIBRATION_NOT_RELIABLE",
        }

    # 检查矩阵有效性
    for cam, T in X_cameras.items():
        if not np.all(np.isfinite(T)):
            return {
                "error": "{} 外参矩阵含非有限值".format(cam),
                "status": "CALIBRATION_NOT_RELIABLE",
            }
        R = T[:3, :3]
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.01:
            return {
                "error": "{} 旋转矩阵 det={:.4f}, 期望约 1.0".format(cam, det),
                "status": "CALIBRATION_NOT_RELIABLE",
            }

    # 检查共识可靠性
    pair_stats = report.get("pairs", {})
    for pn in ["FL_FR", "FL_RE"]:
        ps = pair_stats.get(pn, {})
        if ps.get("n_inliers", 0) < 3:
            return {
                "error": "{} 共识内点不足 ({} < 3)".format(pn, ps.get("n_inliers", 0)),
                "status": "CALIBRATION_NOT_RELIABLE",
            }

    result = {
        "solver": "pairwise_camera_relative",
        "version": "stable-v1",
        "n_groups": len(per_group_pnp),
        "group_ids": sorted(per_group_pnp.keys()),
        "cameras": {
            cam: X_cameras[cam].tolist()
            for cam in CAMERAS if cam in X_cameras
        },
        "pair_stats": report.get("pairs", {}),
        "triangle_closure_t_mm": report.get("triangle_closure_t_mm"),
        "triangle_closure_r_deg": report.get("triangle_closure_r_deg"),
        "status": "CALIBRATION_SUCCESS",
    }

    if output_path:
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(result, f, indent=2)
        sha = hashlib.sha256(open(output_path, "rb").read()).hexdigest()
        print("结果已保存: {}".format(output_path))
        print("SHA256: {}".format(sha))

    return result


def main():
    parser = argparse.ArgumentParser(
        description="三相机标定 — 从采集数据计算外参 (稳定版 V1)")
    parser.add_argument("dataset_dir",
                        help="数据集目录 (包含 group_XXXX/ 子目录)")
    parser.add_argument("-o", "--output",
                        default=None, help="输出 JSON 路径")
    parser.add_argument("--camera-k", nargs=9, type=float,
                        default=[462.1, 0, 320, 0, 462.1, 240, 0, 0, 1],
                        help="相机内参 K (3x3, 按行, 默认 Realsense)")
    args = parser.parse_args()

    if not os.path.isdir(args.dataset_dir):
        print("错误: 数据集目录不存在: {}".format(args.dataset_dir))
        sys.exit(1)

    K = np.array(args.camera_k).reshape(3, 3)
    D = np.zeros(5)
    camera_info = {
        cam: {"K": K.copy(), "D": D.copy()}
        for cam in CAMERAS
    }

    result = calibrate(args.dataset_dir, camera_info, args.output)

    if result.get("status") == "CALIBRATION_SUCCESS":
        print("\n=== 标定完成 ===")
        for pn, ps in result.get("pair_stats", {}).items():
            print("  {}: {} 对, {} 内点".format(
                pn, ps["n_total"], ps["n_inliers"]))
        tc = result.get("triangle_closure_t_mm")
        if tc is not None:
            print("  三角形闭合: T={:.1f}mm R={:.2f}°".format(
                tc, result["triangle_closure_r_deg"]))
    else:
        print("标定失败: {}".format(result.get("error", "未知错误")))
        sys.exit(1)


if __name__ == "__main__":
    main()

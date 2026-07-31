#!/usr/bin/env python3
"""
Gazebo Truth Scorer — 完全独立于求解器。

读取 blind solve 输出的 camera_extrinsics_blind.json,
查询 Gazebo TF truth, 输出 FR/RE 的 T/R 误差.

Truth-free guarantee: 此脚本与 solver 完全独立运行,
solver 进程不读取任何 Gazebo truth.

用法:
  python3 score_gazebo_calibration.py <blind_result.json> [--output report.json]
"""
import sys, os, json, math, argparse
import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation

# 路径设置
WS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
sys.path.insert(0, os.path.join(WS, "src", "cr5_spray_perception", "src"))
from cr5_spray_perception.calibration.geometry import (
    se3_distance_mm_deg, invert_transform, euler_matrix)

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]


def get_gazebo_truth_camera_rig(tf_buffer, timeout=5.0):
    """从 TF 树读取当前 Gazebo camera truth.

    注意: 读取的是 optical frame 之间的相对变换.
    T_rig_cam = inv(T_world_rig) @ T_world_cam, rig = FL optical frame.

    Returns:
        T_rig_truth: {cam_name: 4x4 T_rig_camera}
    """
    # 等待 TF 树稳定
    rospy.sleep(0.5)

    optical_frames = {cam: f"{cam}_color_optical_frame" for cam in CAMERAS}

    T_world_cam = {}
    for cam, frame in optical_frames.items():
        tf = tf_buffer.lookup_transform("world", frame, rospy.Time(0), rospy.Duration(timeout))
        t, r = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        T_world_cam[cam] = T

    T_world_FL = T_world_cam.get(FIRST_CAM)
    if T_world_FL is None:
        raise RuntimeError(f"Cannot get truth for {FIRST_CAM}")

    T_rig_truth = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS[1:]:
        if cam in T_world_cam:
            T_rig_truth[cam] = invert_transform(T_world_FL) @ T_world_cam[cam]

    return T_rig_truth


def load_blind_result(path):
    """加载 blind solve 输出的 camera extrinsics."""
    with open(path, "r") as f:
        data = json.load(f)
    # Format: {"cameras": {"cam_front_left": [[...],...], ...}}
    result = {}
    cam_data = data.get("cameras", data)
    for cam_name, T_list in cam_data.items():
        result[cam_name] = np.array(T_list)
    return result, data


def score(blind_result_path, output_path=None):
    """主评分逻辑."""
    rospy.init_node("score_gazebo_calibration", anonymous=True)

    # Load blind result (solver output, no truth used)
    X_blind, raw_data = load_blind_result(blind_result_path)

    # SHA256 of input file
    import hashlib
    with open(blind_result_path, "rb") as f:
        blind_sha = hashlib.sha256(f.read()).hexdigest()

    # Query Gazebo truth (now, not before)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)  # let TF buffer fill

    try:
        X_truth = get_gazebo_truth_camera_rig(tf_buffer)
    except Exception as e:
        rospy.logerr(f"Failed to get Gazebo truth: {e}")
        sys.exit(1)

    # Score each camera
    scores = {}
    all_pass = True
    for cam in ["cam_front_right", "cam_rear"]:
        if cam not in X_blind or cam not in X_truth:
            rospy.logwarn(f"Cannot score {cam}: missing in blind or truth")
            continue

        T_est = np.array(X_blind[cam])
        T_gt = np.array(X_truth[cam])
        t_err, r_err, detail = se3_distance_mm_deg(T_est, T_gt, 50, 5)

        t_pass = t_err < 10.0
        r_pass = r_err < 1.0
        status = "PASS" if (t_pass and r_pass) else "FAIL"

        scores[cam] = {
            "T_mm": float(t_err),
            "R_deg": float(r_err),
            "T_pass": t_pass,
            "R_pass": r_pass,
            "status": status,
        }
        if not (t_pass and r_pass):
            all_pass = False

        rospy.loginfo(f"  {cam}: T={t_err:.2f}mm ({'PASS' if t_pass else 'FAIL'}), "
                      f"R={r_err:.3f}° ({'PASS' if r_pass else 'FAIL'}) [{status}]")

        # Detail
        rospy.loginfo(f"    blind t={(T_est[:3,3]*1000).round(1)}mm")
        rospy.loginfo(f"    truth t={(T_gt[:3,3]*1000).round(1)}mm")

    # Triangle closure check
    if ("cam_front_right" in X_blind and "cam_rear" in X_blind
            and "cam_front_right" in X_truth and "cam_rear" in X_truth):
        T_FR = X_blind["cam_front_right"]
        T_RE = X_blind["cam_rear"]
        T_FR_RE = invert_transform(T_FR) @ T_RE
        T_tri_gt = invert_transform(X_truth["cam_front_right"]) @ X_truth["cam_rear"]
        tri_t, tri_r, _ = se3_distance_mm_deg(T_FR_RE, T_tri_gt, 50, 5)
        rospy.loginfo(f"  Triangle (FR→RE): T={tri_t:.2f}mm R={tri_r:.3f}°")

    # Report
    report = {
        "blind_result_path": blind_result_path,
        "blind_result_sha256": blind_sha,
        "scores": scores,
        "core_pass": all_pass,
        "cameras_tested": list(scores.keys()),
    }

    if output_path:
        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(report, f, indent=2, default=lambda x: float(x) if isinstance(x, (np.floating, np.ndarray)) else str(x))

    print(f"\n{'='*60}")
    print(f"GAZEBO TRUTH SCORE")
    print(f"{'='*60}")
    for cam, s in scores.items():
        print(f"  {cam}: T={s['T_mm']:.2f}mm R={s['R_deg']:.3f}° [{s['status']}]")
    print(f"\n  CORE_PASS: {'YES' if all_pass else 'NO'}")
    print(f"  BLIND_SHA256: {blind_sha[:16]}...")
    print(f"{'='*60}")

    return all_pass


def main():
    parser = argparse.ArgumentParser(description="Gazebo Truth Scorer for blind calibration results")
    parser.add_argument("blind_result", help="Path to camera_extrinsics_blind.json")
    parser.add_argument("--output", "-o", default=None, help="Output report path")
    args = parser.parse_args()

    if not os.path.isfile(args.blind_result):
        print(f"ERROR: blind result not found: {args.blind_result}")
        sys.exit(1)

    ok = score(args.blind_result, args.output)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

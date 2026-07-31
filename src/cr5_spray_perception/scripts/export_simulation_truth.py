#!/usr/bin/env python3
"""
SIMULATION TRUTH FALLBACK — 从场景配置导出 Gazebo 等价外参.

不依赖 ROS runtime, 纯几何计算.
TF chain: world → camera_link (look-at) → camera_optical_frame (rpy=-π/2,0,-π/2)

用法:
  python3 export_simulation_truth.py [output_dir]
"""
import sys, os, math, hashlib, subprocess
from datetime import datetime
import yaml
import numpy as np


def _euler_matrix(ai, aj, ak):
    from math import cos, sin
    Rx = np.array([[1, 0, 0], [0, cos(ai), -sin(ai)], [0, sin(ai), cos(ai)]])
    Ry = np.array([[cos(aj), 0, sin(aj)], [0, 1, 0], [-sin(aj), 0, cos(aj)]])
    Rz = np.array([[cos(ak), -sin(ak), 0], [sin(ak), cos(ak), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rpy_from_rotation(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _look_at_rotation(cam_pos, target_pos):
    """Gazebo link convention: link +X = camera forward."""
    cam = np.array(cam_pos, dtype=np.float64)
    tgt = np.array(target_pos, dtype=np.float64)
    d_raw = tgt - cam
    dist = float(np.linalg.norm(d_raw))
    if dist < 1e-9:
        raise ValueError(f"Camera at target position: {cam_pos}")
    d = d_raw / dist
    world_z = np.array([0.0, 0.0, 1.0])
    cam_x = d
    cam_y = np.cross(world_z, cam_x)
    cam_y_norm = float(np.linalg.norm(cam_y))
    if cam_y_norm < 1e-9:
        cam_y = np.array([0.0, 1.0, 0.0])
    else:
        cam_y = cam_y / cam_y_norm
    cam_z = np.cross(cam_x, cam_y)
    cam_z_norm = float(np.linalg.norm(cam_z))
    if cam_z_norm > 1e-9:
        cam_z = cam_z / cam_z_norm
    R = np.column_stack([cam_x, cam_y, cam_z])
    return R, d, dist


def compute_camera_look_at(cam_pos, target_pos, roll_offset_deg=0.0):
    R, d, dist = _look_at_rotation(cam_pos, target_pos)
    roll, pitch, yaw = _rpy_from_rotation(R)
    roll += math.radians(roll_offset_deg)
    return roll, pitch, yaw


# Link → optical frame rotation (from fixed_rgbd_camera.urdf.xacro: rpy="-1.5708 0 -1.5708")
LINK_TO_OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
FIRST_CAM = CAMERAS[0]


def _quaternion_from_matrix(T):
    R = np.asarray(T[:3, :3], dtype=np.float64)
    q = np.empty(4)
    t = R.trace()
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return [float(v) for v in q]


def T_to_qt(T):
    q = _quaternion_from_matrix(T)
    return [q[3], q[0], q[1], q[2],
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]


def compute_nominal_camera_poses(scene_config):
    profiles = scene_config.get("cameras", {})
    cam_cfg = profiles.get("cameras", [])
    target = profiles.get("target", {"x": 0.72, "y": 0.0, "z": 0.62})
    tgt = [target["x"], target["y"], target["z"]]

    R_link_optical = _euler_matrix(*LINK_TO_OPTICAL_RPY)[:3, :3]
    T_world_optical = {}

    for cam in cam_cfg:
        name = cam["name"]
        pos = [cam["position"]["x"], cam["position"]["y"], cam["position"]["z"]]
        roll_off = cam.get("roll_offset_deg", 0.0)
        roll, pitch, yaw = compute_camera_look_at(pos, tgt, roll_offset_deg=roll_off)

        R_wl = _euler_matrix(roll, pitch, yaw)
        T_wl = np.eye(4)
        T_wl[:3, :3] = R_wl
        T_wl[:3, 3] = pos
        T_lo = np.eye(4)
        T_lo[:3, :3] = R_link_optical
        T_wo = T_wl @ T_lo
        T_world_optical[name] = T_wo

    T_world_FL = T_world_optical[FIRST_CAM]
    T_rig_cameras = {FIRST_CAM: np.eye(4)}
    for cam in CAMERAS[1:]:
        T_world_cam = T_world_optical[cam]
        T_rig_cam = np.linalg.inv(T_world_FL) @ T_world_cam
        T_rig_cameras[cam] = T_rig_cam

    return T_rig_cameras


def main():
    # Find scene config
    search_paths = [
        os.path.join(os.path.dirname(__file__), "..", "..", "cr5_spray_sim",
                     "config", "simulation_scene.yaml"),
    ]
    try:
        import rospkg
        sim_path = rospkg.RosPack().get_path("cr5_spray_sim")
        search_paths.insert(0, os.path.join(sim_path, "config", "simulation_scene.yaml"))
    except Exception:
        pass

    scene_config = None
    for sp in search_paths:
        if os.path.isfile(sp):
            with open(sp) as f:
                scene_config = yaml.safe_load(f)
            break

    if scene_config is None:
        print("FATAL: Cannot find simulation_scene.yaml")
        sys.exit(1)

    # Compute nominal poses
    T_rig_cameras = compute_nominal_camera_poses(scene_config)

    print("=" * 70)
    print("  SIMULATION TRUTH EXPORT")
    print("  Using scene geometry (link→optical: rpy={})".format(
        tuple(round(math.degrees(r), 1) for r in LINK_TO_OPTICAL_RPY)))
    print("=" * 70)

    for cam in CAMERAS:
        T = T_rig_cameras[cam]
        qt = T_to_qt(T)
        r = _rpy_from_rotation(T[:3, :3])
        print("  {}:".format(cam))
        print("    q=[{:.6f},{:.6f},{:.6f},{:.6f}]".format(*qt[:4]))
        print("    t=[{:.6f},{:.6f},{:.6f}] m".format(*qt[4:]))
        print("    rpy=[{:.3f},{:.3f},{:.3f}]°".format(
            math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2])))
        print("    translation_norm={:.4f} m".format(np.linalg.norm(T[:3, 3])))

    # Determine output dir
    output_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.environ.get("CR5_DATA_ROOT", os.path.expanduser("~/cr5_data")),
        "calibration", "runs", "sim_v7_truth_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(output_dir, exist_ok=True)

    RUN_ID = os.path.basename(output_dir)

    # Build cameras dict (native Python types)
    import cv2
    cameras_dict = {}
    for cam in CAMERAS:
        T = T_rig_cameras[cam]
        T_native = [[float(v) for v in row] for row in T.tolist()]
        T_cam_rig = np.linalg.inv(T)
        T_cam_rig_native = [[float(v) for v in row] for row in T_cam_rig.tolist()]
        rvec = [float(v) for v in cv2.Rodrigues(T[:3, :3])[0].flatten().tolist()]
        qt = [float(v) for v in T_to_qt(T)]
        tvec = [float(v) for v in T[:3, 3]]

        cameras_dict[cam] = {
            "optical_frame": "{}_color_optical_frame".format(cam),
            "T_rig_camera": T_native,
            "T_camera_rig": T_cam_rig_native,
            "T_rig_camera_rvec": rvec,
            "T_rig_camera_tvec": tvec,
            "T_rig_camera_qt": qt,
        }

    rig_frame = "{}_color_optical_frame".format(FIRST_CAM)

    extrinsics = {
        "schema_version": 2,
        "calibration_id": "sim_truth_" + datetime.now().strftime("%Y%m%d_%H%M%S"),
        "generated_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "method": "gazebo_ground_truth_export",
        "status": "SIMULATION_TRUTH_FALLBACK",
        "note": "Direct export from simulation_scene.yaml geometry. "
                "NOT a blind calibration result. "
                "Realsense plugin crash prevented live calibration capture (SIGSEGV on 2nd camera load).",
        "rig_frame": rig_frame,
        "rig_definition": "first camera ({}) color optical frame, gauge-fixed at identity".format(FIRST_CAM),
        "transform_contract": {
            "primary_transform": "T_rig_camera",
            "inverse_transform": "T_camera_rig",
            "primary_equation": "p_rig = T_rig_camera @ p_camera",
            "inverse_equation": "p_camera = T_camera_rig @ p_rig",
            "note": "For TSDF fusion, transform each camera's points to rig_frame using T_rig_camera",
        },
        "camera_prior_source": {
            "type": "simulation_scene_nominal_mount",
            "equivalent_real_system": "CAD_mount_pose",
            "tf_chain": "world → camera_link (look-at) → camera_optical_frame (rpy=-π/2,0,-π/2)",
        },
        "cameras": cameras_dict,
    }

    # Write extrinsics
    extrinsics_path = os.path.join(output_dir, "initial_extrinsics.yaml")
    with open(extrinsics_path, "w") as f:
        yaml.dump(extrinsics, f, default_flow_style=False)
    print("\n  Wrote: {}".format(extrinsics_path))

    # Also save truth extrinsics copy
    extrinsics_truth_path = os.path.join(output_dir, "initial_extrinsics_truth.yaml")
    with open(extrinsics_truth_path, "w") as f:
        yaml.dump(extrinsics, f, default_flow_style=False)

    # Save run manifest
    manifest = {
        "schema_version": 1,
        "run_id": RUN_ID,
        "created_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "cameras": list(CAMERAS),
        "method": "simulation_truth_from_scene_geometry",
        "status": "SIMULATION_TRUTH_FALLBACK",
        "reason": "Gazebo RealSense plugin SIGSEGV on 2nd camera load; scene geometry used as ground truth",
        "camera_prior_source": {
            "type": "simulation_scene_nominal_mount",
            "equivalent_real_system": "CAD_mount_pose",
        },
    }

    # Git SHA
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True,
            cwd=os.path.dirname(__file__))
        if result.returncode == 0:
            manifest["git_sha"] = result.stdout.strip()
    except Exception:
        pass

    # Scene file hash
    scene_config_path = None
    for sp in search_paths:
        if os.path.isfile(sp):
            scene_config_path = sp
            break
    if scene_config_path:
        with open(scene_config_path, "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        manifest["scene_files"] = {"simulation_scene.yaml": sha}

    manifest_path = os.path.join(output_dir, "run_manifest.yaml")
    with open(manifest_path, "w") as f:
        yaml.dump(manifest, f, default_flow_style=False)
    print("  Wrote: {}".format(manifest_path))

    # Final report
    print("\n" + "=" * 70)
    print("  CR5 FINAL CALIBRATION — RESULT")
    print("=" * 70)

    try:
        git_sha = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True,
            cwd=os.path.dirname(__file__)).stdout.strip()[:8]
    except Exception:
        git_sha = "unknown"

    print("  INPUT_SHA:   {}".format(git_sha))
    print("  OUTPUT_SHA:  {}".format(git_sha))
    print("  RUN_ID:      {}".format(RUN_ID))
    print("  Method:      simulation-truth (from scene geometry)")
    print("  Status:      SIMULATION_TRUTH_FALLBACK")
    print("")
    print("  Truth Camera Poses (T_rig_camera):")
    for cam in CAMERAS:
        T = T_rig_cameras[cam]
        t_norm = np.linalg.norm(T[:3, 3])
        r = _rpy_from_rotation(T[:3, :3])
        if cam == FIRST_CAM:
            print("    {}: identity (fixed)".format(cam))
        else:
            print("    {}: t=[{:.4f},{:.4f},{:.4f}]m |t|={:.4f}m rpy=[{:.2f},{:.2f},{:.2f}]°".format(
                cam, T[0,3], T[1,3], T[2,3], t_norm,
                math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2])))

    print("")
    print("  ✓ 仿真外参已导出。后续 TSDF/喷涂 可立即使用。")
    print("  ⚠ 这不是盲标定结果。标定算法鲁棒性后续单独优化。")
    print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())

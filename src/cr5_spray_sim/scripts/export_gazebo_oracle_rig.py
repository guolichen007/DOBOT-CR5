#!/usr/bin/env python3
"""
CR5 Simulation — Gazebo Oracle 外参导出.

从 simulation_scene.yaml (场景唯一真值) 计算 Oracle T_rig_camera,
完全独立于 Stable V1 pairwise_solver.

Oracle schema: cr5_gazebo_oracle_rig_v1
输出: oracle_rig.yaml + provenance.json

生产 reconstruction 模块禁止导入此脚本或读取 Oracle 输出.
Oracle 仅用于独立基线评价.
"""
import os, sys, math, json, yaml, hashlib, argparse, datetime
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.join(SCRIPT_DIR, "..")
sys.path.insert(0, os.path.join(PKG_DIR, "src"))

from cr5_spray_sim.camera_geometry import compute_camera_look_at


# ── 常量 ──
# link → optical 固定变换 (来自 fixed_rgbd_camera.urdf.xacro)
# rpy = (-π/2, 0, -π/2)
LINK_TO_OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
LINK_TO_OPTICAL_R = Rotation.from_euler('xyz', LINK_TO_OPTICAL_RPY).as_matrix()
RIG_FRAME = "cam_front_left_color_optical_frame"

REQUIRED_CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]


def _rpy_to_matrix(roll, pitch, yaw):
    """RPY (fixed-axis XYZ) → 旋转矩阵."""
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()


def _matrix_to_se3(R, t):
    """构建 4×4 SE(3) 矩阵."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _invert_se3(T):
    """SE(3) 逆矩阵."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def _format_matrix_list(T):
    """将 4×4 矩阵转为嵌套列表 (YAML 兼容)."""
    return [[float(T[i, j]) for j in range(4)] for i in range(4)]


def _sha256_file(path):
    if not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _sha256_str(s):
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def _get_git_sha(pkg_dir):
    try:
        import subprocess
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=pkg_dir, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def compute_oracle_rig(scene_config):
    """从 simulation_scene.yaml 计算 Oracle T_rig_camera.

    变换链:
      T_world_link = [R_look_at | pos]
      T_world_optical = T_world_link @ T_link_optical
      T_world_rig = T_world_optical_FL
      T_rig_camera = inv(T_world_rig) @ T_world_optical_camera

    Returns:
        dict: oracle rig 数据
    """
    profiles = scene_config.get("cameras", {})
    cam_cfg_list = profiles.get("cameras", [])
    target = profiles.get("target", {"x": 0.72, "y": 0, "z": 0.62})
    tgt = [target["x"], target["y"], target["z"]]

    cam_cfgs = {c["name"]: c for c in cam_cfg_list}

    # 计算每台相机的 T_world_optical
    T_world_optical = {}
    pose_details = {}

    for cam_name in REQUIRED_CAMERAS:
        if cam_name not in cam_cfgs:
            raise ValueError(f"simulation_scene.yaml 缺失相机: {cam_name}")

        cfg = cam_cfgs[cam_name]
        pos = [cfg["position"]["x"], cfg["position"]["y"], cfg["position"]["z"]]
        roll_off = cfg.get("roll_offset_deg", 0.0)

        # 与 publish_fixed_camera_frames.py 完全相同的 look-at 计算
        rpy_data = compute_camera_look_at(pos, tgt, roll_offset_deg=roll_off)
        R_link = rpy_data["R"]  # world → link 旋转
        t_link = np.array(pos)

        # T_world_link
        T_wl = _matrix_to_se3(R_link, t_link)

        # T_link_optical (从 URDF: rpy = -π/2, 0, -π/2, 无平移)
        T_lo = _matrix_to_se3(LINK_TO_OPTICAL_R, np.zeros(3))

        # T_world_optical
        T_wo = T_wl @ T_lo
        T_world_optical[cam_name] = T_wo

        pose_details[cam_name] = {
            "position_world": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
            "target_world": {"x": float(tgt[0]), "y": float(tgt[1]), "z": float(tgt[2])},
            "distance_m": rpy_data["distance_m"],
            "link_rpy_deg": {
                "roll": round(math.degrees(rpy_data["roll"]), 4),
                "pitch": round(math.degrees(rpy_data["pitch"]), 4),
                "yaw": round(math.degrees(rpy_data["yaw"]), 4),
            },
            "optical_z_error_deg": rpy_data["optical_z_angle_error_deg"],
            "image_up_error_deg": rpy_data["image_up_vs_world_up_deg"],
        }

    # Rig frame = FL optical frame
    T_world_rig = T_world_optical["cam_front_left"]

    # 计算 T_rig_camera = inv(T_world_rig) @ T_world_optical
    T_rig_inv = _invert_se3(T_world_rig)
    T_rig_camera = {}
    T_camera_rig = {}

    for cam_name in REQUIRED_CAMERAS:
        Trc = T_rig_inv @ T_world_optical[cam_name]
        Tcr = _invert_se3(Trc)
        T_rig_camera[cam_name] = Trc
        T_camera_rig[cam_name] = Tcr

    # ── 验证 ──
    # 1. FL 的 T_rig_camera 应为 identity
    T_fl = T_rig_camera["cam_front_left"]
    id_error = np.max(np.abs(T_fl - np.eye(4)))
    if id_error > 1e-10:
        raise RuntimeError(f"FL T_rig_camera 非 identity: max|T-I|={id_error:.2e}")

    # 2. T_rig_camera = inv(T_camera_rig)
    for cam_name in REQUIRED_CAMERAS:
        Trc = T_rig_camera[cam_name]
        Tcr = T_camera_rig[cam_name]
        product = Trc @ Tcr
        id_err = np.max(np.abs(product - np.eye(4)))
        if id_err > 1e-10:
            raise RuntimeError(f"{cam_name}: T_rig @ T_cam ≠ I, error={id_err:.2e}")

    # 3. T_rig_camera = inv(T_world_rig) @ T_world_camera
    for cam_name in REQUIRED_CAMERAS:
        Trc = T_rig_camera[cam_name]
        computed = T_rig_inv @ T_world_optical[cam_name]
        err = np.max(np.abs(Trc - computed))
        if err > 1e-10:
            raise RuntimeError(f"{cam_name}: Oracle 定义不一致, error={err:.2e}")

    return {
        "T_world_optical": T_world_optical,
        "T_rig_camera": T_rig_camera,
        "T_camera_rig": T_camera_rig,
        "T_world_rig": T_world_rig,
        "rig_inv": T_rig_inv,
        "pose_details": pose_details,
    }


def main():
    parser = argparse.ArgumentParser(description="导出 Gazebo Oracle 外参 (场景真值)")
    parser.add_argument("--scene-config", default=None,
                        help="simulation_scene.yaml 路径")
    parser.add_argument("--output-dir", "-o", required=True,
                        help="输出目录 (oracle_rig.yaml + provenance.json)")
    args = parser.parse_args()

    # 加载场景配置
    if args.scene_config:
        scene_path = args.scene_config
    else:
        scene_path = os.path.join(PKG_DIR, "config", "simulation_scene.yaml")

    if not os.path.isfile(scene_path):
        print(f"ERROR: 场景配置文件不存在: {scene_path}")
        sys.exit(1)

    with open(scene_path, "r") as f:
        scene_config = yaml.safe_load(f)

    scene_sha = _sha256_file(scene_path)
    git_sha = _get_git_sha(PKG_DIR)
    timestamp = datetime.datetime.utcnow().isoformat() + "Z"

    # 计算 Oracle
    oracle = compute_oracle_rig(scene_config)

    # 输出目录
    os.makedirs(args.output_dir, exist_ok=True)

    # ── oracle_rig.yaml ──
    oracle_yaml = {
        "schema": "cr5_gazebo_oracle_rig_v1",
        "source": "gazebo_tf_truth",
        "query_timestamp": timestamp,
        "world_frame": "world",
        "rig_frame": RIG_FRAME,
        "transform_contract": "p_rig = T_rig_camera @ p_camera",
        "cameras": {},
    }

    for cam_name in REQUIRED_CAMERAS:
        Trc = oracle["T_rig_camera"][cam_name]
        Tcr = oracle["T_camera_rig"][cam_name]
        details = oracle["pose_details"][cam_name]

        oracle_yaml["cameras"][cam_name] = {
            "optical_frame": f"{cam_name}_color_optical_frame",
            "position_world": details["position_world"],
            "target_world": details["target_world"],
            "distance_m": details["distance_m"],
            "T_rig_camera": _format_matrix_list(Trc),
            "T_camera_rig": _format_matrix_list(Tcr),
        }

    oracle_yaml["validation"] = {
        "FL_T_rig_camera_is_identity": True,
        "T_rig_camera_equals_inv_T_world_rig_times_T_world_camera": True,
        "T_rig_camera_equals_inv_T_camera_rig": True,
    }

    oracle_path = os.path.join(args.output_dir, "oracle_rig.yaml")
    with open(oracle_path, "w") as f:
        yaml.dump(oracle_yaml, f, default_flow_style=None, sort_keys=False, width=120)
    oracle_sha = _sha256_file(oracle_path)

    # ── oracle_calibrated_rig.yaml (融合兼容格式) ──
    # 使用 cr5_calibrated_rig_v1 schema, 但 source=oracle (不伪装 Stable V1)
    rig_compat = {
        "schema_version": "cr5_calibrated_rig_v1",
        "status": "PASS",
        "units": "meter",
        "rig_frame": RIG_FRAME,
        "source_calibration": {
            "file": oracle_path,
            "sha256": oracle_sha,
            "solver": "gazebo_oracle_truth",
            "version": "oracle-v1",
            "n_cameras": 3,
        },
        "transform_contract": {
            "primary": "T_rig_camera",
            "equation": "p_rig = T_rig_camera @ p_camera",
            "inverse": "T_camera_rig = inverse(T_rig_camera)",
        },
        "cameras": {},
    }
    for cam_name in REQUIRED_CAMERAS:
        Trc = oracle["T_rig_camera"][cam_name]
        Tcr = oracle["T_camera_rig"][cam_name]
        rig_compat["cameras"][cam_name] = {
            "optical_frame": f"{cam_name}_color_optical_frame",
            "T_rig_camera": _format_matrix_list(Trc),
            "T_camera_rig": _format_matrix_list(Tcr),
        }

    rig_compat_path = os.path.join(args.output_dir, "oracle_calibrated_rig.yaml")
    with open(rig_compat_path, "w") as f:
        yaml.dump(rig_compat, f, default_flow_style=None, sort_keys=False, width=120)

    # ── provenance.json ──
    provenance = {
        "schema": "cr5_gazebo_oracle_rig_v1",
        "source": "gazebo_tf_truth",
        "scene_config_path": os.path.abspath(scene_path),
        "scene_config_sha256": scene_sha,
        "oracle_rig_sha256": oracle_sha,
        "git_sha": git_sha,
        "timestamp": timestamp,
        "transform_equations": [
            "T_world_link = [R_look_at(cam, target) | cam_pos]",
            "T_link_optical = T(rpy=-π/2, 0, -π/2, t=0)",
            "T_world_optical = T_world_link @ T_link_optical",
            "T_world_rig = T_world_optical_FL",
            "T_rig_camera = inv(T_world_rig) @ T_world_optical",
            "T_camera_rig = inv(T_rig_camera)",
            "p_rig = T_rig_camera @ p_camera",
        ],
        "validation_passed": True,
        "cameras": {},
    }

    for cam_name in REQUIRED_CAMERAS:
        provenance["cameras"][cam_name] = {
            "optical_frame": f"{cam_name}_color_optical_frame",
            "pose_details": oracle["pose_details"][cam_name],
            "T_rig_camera_translation_m": oracle["T_rig_camera"][cam_name][:3, 3].tolist(),
        }

    prov_path = os.path.join(args.output_dir, "provenance.json")
    with open(prov_path, "w") as f:
        json.dump(provenance, f, indent=2, default=str)

    # ── 控制台输出 ──
    print(f"\n{'='*60}")
    print("Oracle 外参导出完成")
    print(f"{'='*60}")
    print(f"场景配置: {os.path.abspath(scene_path)}")
    print(f"场景 SHA: {scene_sha}")
    print(f"Git SHA:  {git_sha}")
    print(f"Rig frame: {RIG_FRAME}")
    print()

    for cam_name in REQUIRED_CAMERAS:
        t = oracle["T_rig_camera"][cam_name][:3, 3]
        r = Rotation.from_matrix(oracle["T_rig_camera"][cam_name][:3, :3])
        rpy = r.as_euler('xyz', degrees=True)
        dist = oracle["pose_details"][cam_name]["distance_m"]
        print(f"  {cam_name}:")
        print(f"    世界位置: ({oracle['pose_details'][cam_name]['position_world']['x']:.3f}, "
              f"{oracle['pose_details'][cam_name]['position_world']['y']:.3f}, "
              f"{oracle['pose_details'][cam_name]['position_world']['z']:.3f})")
        print(f"    距目标: {dist:.3f}m")
        print(f"    T_rig_camera 平移: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
        print(f"    T_rig_camera RPY:   [{rpy[0]:.2f}°, {rpy[1]:.2f}°, {rpy[2]:.2f}°]")

    print(f"\n✅ 验证通过: FL identity + 互逆一致")
    print(f"📁 Oracle rig:  {oracle_path}")
    print(f"📁 Provenance:  {prov_path}")
    print()

    sys.exit(0)


if __name__ == "__main__":
    main()

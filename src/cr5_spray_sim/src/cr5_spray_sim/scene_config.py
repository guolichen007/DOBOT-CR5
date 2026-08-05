#!/usr/bin/env python3
"""
CR5 Simulation Scene Config — 单一真值源读取工具.

所有 world-level pose 应从 simulation_scene.yaml 读取，禁止在其他位置硬编码场景坐标。

用法:
  from cr5_spray_sim.scene_config import load_scene_config, get_base_target_pose

  scene = load_scene_config()
  tx, ty, tz = get_base_target_pose()
"""

import os
import yaml


def _find_scene_yaml():
    """定位 simulation_scene.yaml 的绝对路径."""
    # 优先从 rosparam 获取
    try:
        import rospy
        config_path = rospy.get_param("/scene_config/path", "")
        if config_path and os.path.isfile(config_path):
            return config_path
    except Exception:
        pass

    # 通过 rospkg
    try:
        import rospkg
        return os.path.join(
            rospkg.RosPack().get_path("cr5_spray_sim"),
            "config", "simulation_scene.yaml")
    except Exception:
        pass

    # 相对路径回退
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "config", "simulation_scene.yaml"),
        os.path.join(os.path.dirname(__file__), "..", "config", "simulation_scene.yaml"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return os.path.abspath(p)

    raise FileNotFoundError("Cannot locate simulation_scene.yaml")


def load_scene_config():
    """加载并返回 simulation_scene.yaml 的完整字典."""
    path = _find_scene_yaml()
    with open(path, "r") as f:
        return yaml.safe_load(f)


def get_base_target_pose():
    """返回 base target 位置 (x, y, z).

    对应 simulation_scene.yaml 中 simple_hanging_workpiece.position.
    """
    scene = load_scene_config()
    wp = scene.get("simple_hanging_workpiece", {})
    pos = wp.get("position", {"x": 0.72, "y": 0.0, "z": 0.62})
    return (float(pos["x"]), float(pos["y"]), float(pos["z"]))


def get_camera_lookat_target():
    """返回相机 look-at 目标点 (x, y, z).

    对应 simulation_scene.yaml 中 cameras.target.
    """
    scene = load_scene_config()
    cams = scene.get("cameras", {})
    tgt = cams.get("target", {"x": 0.72, "y": 0.0, "z": 0.62})
    return (float(tgt["x"]), float(tgt["y"]), float(tgt["z"]))


def get_camera_configs():
    """返回相机配置列表.

    每项包含: name, position {x,y,z}, roll_offset_deg, keep_horizon, description.
    """
    scene = load_scene_config()
    return list(scene.get("cameras", {}).get("cameras", []))


def get_pedestal_configs():
    """返回 pedestal 配置列表.

    每项包含: name, base {x,y,z}, height, arm_length.
    """
    scene = load_scene_config()
    return list(scene.get("pedestals", []))


def get_goalpost_config():
    """返回门架配置字典.

    包含: center_x, post_y, height, top_beam_length_y, profile_size,
          base_plate, color_frame.
    """
    scene = load_scene_config()
    return dict(scene.get("simple_goalpost_frame", {}))


def get_cr5_base():
    """返回 CR5 基座位置 (x, y, z)."""
    scene = load_scene_config()
    base = scene.get("cr5_base", {}).get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
    return (float(base["x"]), float(base["y"]), float(base["z"]))


def load_pose_evidence_model_pose(evidence_path):
    """从 pose_evidence.json 提取模型位姿 (T_world_object).

    供 generate_visible_gt.py 和 evaluate_reconstruction_gazebo.py 共用.

    读取 actual_pose_before_capture (新 schema), 兼容 actual_pose (旧).

    Returns: (T_world_object_4x4, evidence_dict)
    Raises: FileNotFoundError, ValueError, KeyError
    """
    import os, json, math, numpy as np
    from scipy.spatial.transform import Rotation

    if not os.path.isfile(evidence_path):
        raise FileNotFoundError(f"pose_evidence.json not found: {evidence_path}")

    with open(evidence_path) as f:
        evidence = json.load(f)

    # 新 schema 优先: actual_pose_before_capture
    actual = evidence.get("actual_pose_before_capture") or evidence.get("actual_pose")
    if not actual:
        raise KeyError("pose_evidence missing actual_pose_before_capture (and actual_pose fallback)")

    pos = actual.get("position_xyz") or actual.get("position")
    quat = actual.get("orientation_xyzw") or actual.get("orientation")
    if not pos or len(pos) != 3:
        raise ValueError(f"invalid position in pose_evidence: {pos}")
    if not quat or len(quat) != 4:
        raise ValueError(f"invalid orientation in pose_evidence: {quat}")

    # 验证数值
    for v in pos + quat:
        if not math.isfinite(v):
            raise ValueError(f"non-finite value in pose_evidence: {v}")

    # quaternion 范数检查
    norm = math.sqrt(sum(q*q for q in quat))
    if abs(norm - 1.0) > 0.01:
        raise ValueError(f"quaternion norm={norm:.4f} != 1.0")

    # settle 状态检查
    settle = evidence.get("settle", {})
    if settle.get("status") != "STABLE":
        # 不阻止, 但记录 (兼容无 settle 的旧数据)
        pass

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T, evidence

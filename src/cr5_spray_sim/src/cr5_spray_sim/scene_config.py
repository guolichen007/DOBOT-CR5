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

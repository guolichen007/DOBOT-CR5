#!/usr/bin/env python3
"""test_scene_config.py — 场景 YAML 配置验证测试"""

import os
import sys
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def load_config(name):
    path = os.path.join(CONFIG_DIR, name)
    if not os.path.isfile(path):
        return None
    with open(path, "r") as f:
        return yaml.safe_load(f)


def test_simulation_scene_exists():
    """simulation_scene.yaml 必须存在且包含 cameras key."""
    cfg = load_config("simulation_scene.yaml")
    assert cfg is not None, "simulation_scene.yaml not found"
    assert "cameras" in cfg, "Missing cameras key"


def test_simulation_scene_three_cameras():
    """simulation_scene.yaml 必须定义 3 台固定相机."""
    cfg = load_config("simulation_scene.yaml")
    assert cfg is not None
    cameras = cfg["cameras"]["cameras"]
    assert len(cameras) == 3, f"Expected 3 cameras, got {len(cameras)}"
    names = [c["name"] for c in cameras]
    for expected in ["cam_front_left", "cam_front_right", "cam_rear"]:
        assert expected in names, f"Camera {expected} not in {names}"


def test_simulation_scene_target_defined():
    """simulation_scene.yaml 必须定义标定目标位姿."""
    cfg = load_config("simulation_scene.yaml")
    assert cfg is not None
    target = cfg["cameras"]["target"]
    for axis in ["x", "y", "z"]:
        assert axis in target, f"target missing {axis}"


def test_simulation_scene_profiles():
    """simulation_scene.yaml 必须定义 vm_profile 和 quality_profile."""
    cfg = load_config("simulation_scene.yaml")
    assert cfg is not None
    c = cfg["cameras"]
    assert "vm_profile" in c, "Missing vm_profile"
    assert "quality_profile" in c, "Missing quality_profile"
    assert c["quality_profile"]["color_width"] >= 640
    assert c["quality_profile"]["color_height"] >= 480


def test_camera_layout_ideal_exists():
    cfg = load_config("camera_layout_ideal.yaml")
    assert cfg is not None, "camera_layout_ideal.yaml not found"
    assert "cameras" in cfg
    assert len(cfg["cameras"]) >= 3


def test_camera_layout_realistic_exists():
    cfg = load_config("camera_layout_realistic.yaml")
    assert cfg is not None, "camera_layout_realistic.yaml not found"
    assert "cameras" in cfg


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS: {t.__name__}")
        except Exception as e:
            print(f"FAIL: {t.__name__} — {e}")
            failed += 1
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)

#!/usr/bin/env python3
"""
test_hanging_attachment_geometry.py — 悬挂几何接触验证

离线验证标定目标与门架横梁的物理连接关系:

1. Launch 文件未硬编码过时坐标 (x≠0.56)
2. goalpost 与 target XY 对齐 (误差 ≤ 1mm)
3. spreader 顶面贴合门架底面 (误差 ≤ 1mm)
4. spreader 不穿透门架横梁 (穿透 ≤ 1mm)
5. 吊索端点接触: bottom=body_top, top=spreader_bottom
6. SDF 与权威 YAML 的 spreader/cable 参数一致

几何真值来源:
  - simulation_scene.yaml → world pose
  - config/calibration/calibration_target.yaml → 目标内部几何
  - models/calibration_target/model.sdf → Gazebo 实际渲染几何

用法:
  python3 src/cr5_spray_sim/tests/test_hanging_attachment_geometry.py

退出码:
  0 = 全部通过
  1 = 至少一项失败
"""
import os
import sys
import re
import math
import yaml

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
CONFIG_DIR = os.path.join(PKG_DIR, "config")
MODEL_DIR = os.path.join(PKG_DIR, "models", "calibration_target")
LAUNCH_FILE = os.path.join(PKG_DIR, "launch", "spray_simulation.launch")
SCENE_YAML = os.path.join(CONFIG_DIR, "simulation_scene.yaml")
TARGET_YAML = os.path.join(CONFIG_DIR, "calibration", "calibration_target.yaml")
SDF_FILE = os.path.join(MODEL_DIR, "model.sdf")

# 容差 (米)
XY_TOLERANCE_M = 0.001
Z_CONTACT_TOLERANCE_M = 0.001
Z_PENETRATION_TOLERANCE_M = 0.001

# ── helpers ──────────────────────────────────────────────────

def _load_scene():
    with open(SCENE_YAML, "r") as f:
        return yaml.safe_load(f)

def _load_target_yaml():
    with open(TARGET_YAML, "r") as f:
        return yaml.safe_load(f)

def _load_sdf_text():
    with open(SDF_FILE, "r") as f:
        return f.read()

def _load_launch_text():
    with open(LAUNCH_FILE, "r") as f:
        return f.read()

def _extract_sdf_pose(link_name, sdf_text):
    """提取 SDF 中指定 link 的 pose (x y z roll pitch yaw)."""
    pattern = rf'<link name="{link_name}">(.*?)</link>'
    link_match = re.search(pattern, sdf_text, re.DOTALL)
    if not link_match:
        return None
    link_content = link_match.group(1)
    pose_match = re.search(r'<pose>\s*([0-9.e+\-\s]+)\s*</pose>', link_content)
    if not pose_match:
        return None
    parts = [float(x) for x in pose_match.group(1).split()]
    return parts  # [x, y, z, roll, pitch, yaw]

def _extract_sdf_box_size(link_name, sdf_text):
    """提取 SDF 中指定 link 的 box size."""
    pattern = rf'<link name="{link_name}">(.*?)</link>'
    link_match = re.search(pattern, sdf_text, re.DOTALL)
    if not link_match:
        return None
    link_content = link_match.group(1)
    size_match = re.search(r'<size>\s*([0-9.e+\-\s]+)\s*</size>', link_content)
    if not size_match:
        return None
    return [float(x) for x in size_match.group(1).split()]

def _extract_sdf_cylinder_params(link_name, sdf_text):
    """提取 SDF 中指定 link 的 cylinder radius 和 length."""
    pattern = rf'<link name="{link_name}">(.*?)</link>'
    link_match = re.search(pattern, sdf_text, re.DOTALL)
    if not link_match:
        return None, None
    link_content = link_match.group(1)
    radius_match = re.search(r'<radius>\s*([0-9.e+\-]+)\s*</radius>', link_content)
    length_match = re.search(r'<length>\s*([0-9.e+\-]+)\s*</length>', link_content)
    radius = float(radius_match.group(1)) if radius_match else None
    length = float(length_match.group(1)) if length_match else None
    return radius, length

# ── tests ────────────────────────────────────────────────────

def test_launch_target_pose_not_hardcoded_stale():
    """Launch 文件中 target X 不得为旧值 0.56."""
    launch_text = _load_launch_text()
    # 找到 spawn_calibration_target 相关行
    pattern = r'spawn_calibration_target.*?-x\s+([0-9.]+)'
    matches = re.findall(pattern, launch_text, re.DOTALL)
    assert matches, "Could not find spawn_calibration_target -x in launch file"

    for x_val in matches:
        x = float(x_val)
        assert abs(x - 0.56) > 0.001, \
            f"STALE X: launch still uses -x {x_val} (should be 0.68)"
        assert abs(x - 0.68) <= 0.001, \
            f"Launch target X={x_val}, expected 0.68 ± 0.001"

    launch_text_clean = launch_text.replace('\n', ' ')
    z_match = re.search(r'spawn_calibration_target.*?-z\s+([0-9.]+)', launch_text_clean)
    if z_match:
        z = float(z_match.group(1))
        assert abs(z - 0.98) <= 0.001, \
            f"Launch target Z={z}, expected 0.98 ± 0.001"


def test_goalpost_target_xy_alignment():
    """门架与目标 XY 对齐 (误差 ≤ 1mm)."""
    scene = _load_scene()

    # goalpost center from scene config
    gp_cfg = scene.get("simple_goalpost_frame", {})
    gp_x = gp_cfg.get("center_x")
    assert gp_x is not None, "goalpost center_x missing in simulation_scene.yaml"
    # goalpost Y=0 (从 launch 的 -y 0 确认)
    gp_y = 0.0

    # target position from scene config
    target_cfg = scene.get("simple_hanging_workpiece", {})
    target_pos = target_cfg.get("position", {})
    target_x = target_pos.get("x")
    target_y = target_pos.get("y", 0.0)
    assert target_x is not None, "target position.x missing in simulation_scene.yaml"

    dx = abs(target_x - gp_x)
    dy = abs(target_y - gp_y)

    assert dx <= XY_TOLERANCE_M, \
        f"TARGET_GOALPOST_XY_ALIGNMENT_FAIL: dx={dx:.4f}m > {XY_TOLERANCE_M*1000:.0f}mm"
    assert dy <= XY_TOLERANCE_M, \
        f"TARGET_GOALPOST_XY_ALIGNMENT_FAIL: dy={dy:.4f}m > {XY_TOLERANCE_M*1000:.0f}mm"


def test_spreader_touches_beam_underside():
    """spreader 顶面贴合门架底面 (接触误差 ≤ 1mm).

    beam_bottom_z = goalpost_z + height - profile_size = 0 + 1.65 - 0.05 = 1.60
    spreader_top_world_z = target_world_z + spreader_center_local_z + spreader_size_z/2
    """
    scene = _load_scene()
    sdf_text = _load_sdf_text()

    # goalpost dimensions
    gp_cfg = scene.get("simple_goalpost_frame", {})
    height = gp_cfg.get("height", 1.65)
    profile = gp_cfg.get("profile_size", 0.05)
    beam_bottom_z = height - profile  # goalpost at z=0

    # target world z
    target_cfg = scene.get("simple_hanging_workpiece", {})
    target_world_z = target_cfg.get("position", {}).get("z", 0.98)

    # spreader from SDF
    spreader_pose = _extract_sdf_pose("spreader_bar", sdf_text)
    assert spreader_pose is not None, "spreader_bar pose not found in SDF"
    spreader_local_z = spreader_pose[2]

    spreader_size = _extract_sdf_box_size("spreader_bar", sdf_text)
    assert spreader_size is not None, "spreader_bar box size not found in SDF"
    spreader_size_z = spreader_size[2]

    spreader_top_world_z = target_world_z + spreader_local_z + spreader_size_z / 2.0
    gap_z = spreader_top_world_z - beam_bottom_z

    assert abs(gap_z) <= Z_CONTACT_TOLERANCE_M, \
        f"SPREADER_BEAM_CONTACT_FAIL: gap={gap_z*1000:.2f}mm " \
        f"(spreader_top_world={spreader_top_world_z:.4f}, beam_bottom={beam_bottom_z:.4f})"


def test_spreader_does_not_penetrate_beam():
    """spreader 不穿透门架横梁 (穿透 ≤ 1mm).

    penetration_z = spreader_top_world_z - beam_bottom_z (正值 = 穿透)
    """
    scene = _load_scene()
    sdf_text = _load_sdf_text()

    gp_cfg = scene.get("simple_goalpost_frame", {})
    height = gp_cfg.get("height", 1.65)
    profile = gp_cfg.get("profile_size", 0.05)
    beam_bottom_z = height - profile

    target_world_z = scene.get("simple_hanging_workpiece", {}).get("position", {}).get("z", 0.98)

    spreader_pose = _extract_sdf_pose("spreader_bar", sdf_text)
    spreader_local_z = spreader_pose[2]
    spreader_size = _extract_sdf_box_size("spreader_bar", sdf_text)
    spreader_size_z = spreader_size[2]

    spreader_top_world_z = target_world_z + spreader_local_z + spreader_size_z / 2.0
    penetration = spreader_top_world_z - beam_bottom_z

    assert penetration <= Z_PENETRATION_TOLERANCE_M, \
        f"SPREADER_BEAM_PENETRATION_FAIL: penetration={penetration*1000:.2f}mm " \
        f"> {Z_PENETRATION_TOLERANCE_M*1000:.0f}mm " \
        f"(spreader_top={spreader_top_world_z:.4f}, beam_bottom={beam_bottom_z:.4f})"


def test_cables_touch_body_and_spreader():
    """吊索端点接触: cable_bottom = body_top, cable_top = spreader_bottom.

    验证:
    - cable_bottom_z = body_top_z = body_size_z / 2 = 0.12
    - cable_top_z = spreader_bottom_z = spreader_center_z - spreader_size_z / 2
    - cable_length = cable_top - cable_bottom
    """
    sdf_text = _load_sdf_text()

    # body top = body_center(0) + body_size_z/2 = 0.12
    body_size = _extract_sdf_box_size("main_body", sdf_text)
    assert body_size is not None, "main_body box size not found in SDF"
    body_top_z = body_size[2] / 2.0  # link center at origin

    # spreader bottom
    spreader_pose = _extract_sdf_pose("spreader_bar", sdf_text)
    spreader_local_z = spreader_pose[2]
    spreader_size = _extract_sdf_box_size("spreader_bar", sdf_text)
    spreader_size_z = spreader_size[2]
    spreader_bottom_z = spreader_local_z - spreader_size_z / 2.0

    # left cable
    left_pose = _extract_sdf_pose("left_cable", sdf_text)
    assert left_pose is not None, "left_cable pose not found in SDF"
    left_center_z = left_pose[2]
    _, left_length = _extract_sdf_cylinder_params("left_cable", sdf_text)
    assert left_length is not None, "left_cable length not found in SDF"
    left_bottom_z = left_center_z - left_length / 2.0
    left_top_z = left_center_z + left_length / 2.0

    # right cable
    right_pose = _extract_sdf_pose("right_cable", sdf_text)
    assert right_pose is not None, "right_cable pose not found in SDF"
    right_center_z = right_pose[2]
    _, right_length = _extract_sdf_cylinder_params("right_cable", sdf_text)
    assert right_length is not None, "right_cable length not found in SDF"
    right_bottom_z = right_center_z - right_length / 2.0
    right_top_z = right_center_z + right_length / 2.0

    # check cables bottom = body top
    assert abs(left_bottom_z - body_top_z) <= Z_CONTACT_TOLERANCE_M, \
        f"CABLE_BODY_CONTACT_FAIL (left): cable_bottom={left_bottom_z:.4f}, " \
        f"body_top={body_top_z:.4f}, gap={abs(left_bottom_z-body_top_z)*1000:.2f}mm"
    assert abs(right_bottom_z - body_top_z) <= Z_CONTACT_TOLERANCE_M, \
        f"CABLE_BODY_CONTACT_FAIL (right): cable_bottom={right_bottom_z:.4f}, " \
        f"body_top={body_top_z:.4f}, gap={abs(right_bottom_z-body_top_z)*1000:.2f}mm"

    # check cables top = spreader bottom
    assert abs(left_top_z - spreader_bottom_z) <= Z_CONTACT_TOLERANCE_M, \
        f"CABLE_SPREADER_CONTACT_FAIL (left): cable_top={left_top_z:.4f}, " \
        f"spreader_bottom={spreader_bottom_z:.4f}, gap={abs(left_top_z-spreader_bottom_z)*1000:.2f}mm"
    assert abs(right_top_z - spreader_bottom_z) <= Z_CONTACT_TOLERANCE_M, \
        f"CABLE_SPREADER_CONTACT_FAIL (right): cable_top={right_top_z:.4f}, " \
        f"spreader_bottom={spreader_bottom_z:.4f}, gap={abs(right_top_z-spreader_bottom_z)*1000:.2f}mm"


def test_sdf_matches_authoritative_yaml():
    """SDF 与 config/calibration/calibration_target.yaml 的 spreader/cable 参数一致."""
    sdf_text = _load_sdf_text()
    target_yaml = _load_target_yaml()

    target = target_yaml.get("target", {})

    # spreader center z
    yaml_spreader_z = target.get("spreader_center_z_m")
    assert yaml_spreader_z is not None, "spreader_center_z_m missing in YAML"
    spreader_pose = _extract_sdf_pose("spreader_bar", sdf_text)
    sdf_spreader_z = spreader_pose[2]
    assert abs(sdf_spreader_z - yaml_spreader_z) <= 0.001, \
        f"SDF spreader z={sdf_spreader_z:.3f} != YAML spreader_center_z_m={yaml_spreader_z:.3f}"

    # spreader size
    yaml_spreader_size = target.get("spreader_size_m", [])
    assert len(yaml_spreader_size) == 3, "spreader_size_m invalid in YAML"
    sdf_spreader_size = _extract_sdf_box_size("spreader_bar", sdf_text)
    for i, axis in enumerate(["x", "y", "z"]):
        assert abs(sdf_spreader_size[i] - yaml_spreader_size[i]) <= 0.001, \
            f"SDF spreader size[{axis}]={sdf_spreader_size[i]:.3f} != YAML={yaml_spreader_size[i]:.3f}"

    # cable values
    yaml_cable_bottom = target.get("cable_bottom_z_m")
    yaml_cable_top = target.get("cable_top_z_m")
    yaml_cable_length = target.get("cable_length_m")
    yaml_cable_center = target.get("cable_center_z_m")

    for cable_name in ["left_cable", "right_cable"]:
        cable_pose = _extract_sdf_pose(cable_name, sdf_text)
        sdf_center_z = cable_pose[2]
        _, sdf_length = _extract_sdf_cylinder_params(cable_name, sdf_text)
        sdf_bottom = sdf_center_z - sdf_length / 2
        sdf_top = sdf_center_z + sdf_length / 2

        assert abs(sdf_bottom - yaml_cable_bottom) <= 0.001, \
            f"SDF {cable_name} bottom_z={sdf_bottom:.3f} != YAML={yaml_cable_bottom:.3f}"
        assert abs(sdf_top - yaml_cable_top) <= 0.001, \
            f"SDF {cable_name} top_z={sdf_top:.3f} != YAML={yaml_cable_top:.3f}"
        assert abs(sdf_length - yaml_cable_length) <= 0.001, \
            f"SDF {cable_name} length={sdf_length:.3f} != YAML={yaml_cable_length:.3f}"
        assert abs(sdf_center_z - yaml_cable_center) <= 0.001, \
            f"SDF {cable_name} center_z={sdf_center_z:.3f} != YAML={yaml_cable_center:.3f}"

    # hanger y
    yaml_hanger_y = target.get("hanger_y_m")
    left_pose = _extract_sdf_pose("left_cable", sdf_text)
    right_pose = _extract_sdf_pose("right_cable", sdf_text)
    assert abs(left_pose[1] - (-yaml_hanger_y)) <= 0.001, \
        f"SDF left_cable y={left_pose[1]:.3f} != -hanger_y_m={-yaml_hanger_y:.3f}"
    assert abs(right_pose[1] - yaml_hanger_y) <= 0.001, \
        f"SDF right_cable y={right_pose[1]:.3f} != hanger_y_m={yaml_hanger_y:.3f}"


def test_scene_yaml_target_has_expected_pose():
    """simulation_scene.yaml 中目标位姿与 launch 预期一致."""
    scene = _load_scene()
    target_pos = scene.get("simple_hanging_workpiece", {}).get("position", {})

    assert abs(target_pos.get("x", 0) - 0.68) <= 0.001, \
        f"Scene YAML target x={target_pos['x']} != 0.68"
    assert abs(target_pos.get("y", 0) - 0.0) <= 0.001, \
        f"Scene YAML target y={target_pos['y']} != 0.0"
    assert abs(target_pos.get("z", 0) - 0.98) <= 0.001, \
        f"Scene YAML target z={target_pos['z']} != 0.98"


def test_camera_target_matches_actual_target():
    """相机 target 与实际目标位姿一致."""
    scene = _load_scene()
    target_pos = scene.get("simple_hanging_workpiece", {}).get("position", {})
    cam_target = scene.get("cameras", {}).get("target", {})

    for axis in ["x", "y", "z"]:
        assert abs(cam_target.get(axis, 0) - target_pos.get(axis, 0)) <= 0.001, \
            f"Camera target {axis}={cam_target[axis]} != target {axis}={target_pos[axis]}"


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
    if failed == 0:
        print("TARGET_GOALPOST_XY_ALIGNMENT_PASS")
        print("SPREADER_BEAM_CONTACT_PASS")
        print("SPREADER_BEAM_NO_PENETRATION_PASS")
        print("CABLE_BODY_CONTACT_PASS")
        print("CABLE_SPREADER_CONTACT_PASS")
    sys.exit(1 if failed else 0)

#!/usr/bin/env python3
"""统一 target geometry loader — calibration_target.yaml 为唯一权威来源.

所有脚本 (run_multi_frame_calibration.py, run_v8_gazebo_e2e.py,
pnp_truth_audit.py, gazebo_visibility_audit.py) 必须使用此模块,
不得硬编码面板几何参数.

使用:
    from cr5_spray_perception.calibration.target_geometry import (
        load_target_geometry, TargetGeometry,
        build_charuco_faces, build_aruco_faces, build_apriltag_faces,
        build_face_poses_target, build_T_target_face,
    )
"""

import os
import math
import hashlib
import yaml
import numpy as np


class TargetGeometry:
    """从 calibration_target.yaml 解析的完整标定靶几何."""

    __slots__ = (
        "yaml_path", "yaml_sha256",
        "panels",                # raw dict
        "face_poses_target",     # {face_name: {xyz:[3], rpy:[3]}}
        "T_target_face",         # {face_name: np.ndarray(4,4)}
        "charuco_faces",         # {face_name: dict}  — ChArUco pattern faces
        "aruco_faces",           # {face_name: dict}  — ArUco pattern faces
        "apriltag_faces",        # {face_name: dict}  — AprilTag pattern faces
        "face_names",            # [str]
        "face_pattern_types",    # {face_name: "charuco"|"aruco"|"apriltag"}
    )

    def to_manifest(self):
        return {
            "yaml_path": self.yaml_path,
            "yaml_sha256": self.yaml_sha256,
            "face_names": self.face_names,
            "face_pattern_types": self.face_pattern_types,
        }


def _compute_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _find_yaml_path():
    """查找 calibration_target.yaml.

    搜索顺序:
      1. 显式环境变量 CR5_CALIBRATION_TARGET_YAML
      2. rospkg cr5_spray_sim
      3. 相对本脚本位置推断 (适用于已安装的包)
      4. 相对 CWD 的 workspace 结构 (适用于开发环境)
      5. FAIL — 生产脚本不允许静默 fallback
    """
    searched = []

    # 1: 环境变量 (最高优先级)
    env_p = os.environ.get("CR5_CALIBRATION_TARGET_YAML", "")
    if env_p:
        searched.append(f"env:CR5_CALIBRATION_TARGET_YAML={env_p}")
        if os.path.exists(env_p):
            return env_p

    # 2: rospkg
    try:
        import rospkg
        rp = rospkg.RosPack()
        sim_path = rp.get_path("cr5_spray_sim")
        p = os.path.join(sim_path, "config", "calibration", "calibration_target.yaml")
        searched.append(f"rospkg:{p}")
        if os.path.exists(p):
            return p
    except Exception:
        pass

    # 3: 相对本脚本推断
    _dir = os.path.abspath(os.path.dirname(__file__))
    # .../cr5_spray_perception/src/cr5_spray_perception/calibration → .../src
    for _ in range(4):
        _dir = os.path.dirname(_dir)
    p = os.path.join(_dir, "cr5_spray_sim", "config", "calibration", "calibration_target.yaml")
    searched.append(f"script-relative:{p}")
    if os.path.exists(p):
        return p

    # 4: 从 CWD 查找 workspace 结构
    cwd = os.getcwd()
    for candidate_dir in [cwd] + list(_parent_dirs(cwd, max_depth=5)):
        p = os.path.join(candidate_dir, "src", "cr5_spray_sim", "config",
                         "calibration", "calibration_target.yaml")
        searched.append(f"cwd:{p}")
        if os.path.exists(p):
            return p

    raise FileNotFoundError(
        "Cannot locate calibration_target.yaml. Searched:\n  " +
        "\n  ".join(searched) +
        "\nInstall cr5_spray_sim or set CR5_CALIBRATION_TARGET_YAML env var."
    )


def _parent_dirs(start_path, max_depth=5):
    """向上遍历目录, 返回 [parent, grandparent, ...]."""
    dirs = []
    current = os.path.abspath(start_path)
    for _ in range(max_depth):
        parent = os.path.dirname(current)
        if parent == current:
            break
        dirs.append(parent)
        current = parent
    return dirs


def load_target_geometry(yaml_path=None) -> TargetGeometry:
    """加载标定靶几何 (calibration_target.yaml).

    Args:
        yaml_path: YAML 路径. None 时自动查找.

    Returns:
        TargetGeometry 实例.

    Raises:
        FileNotFoundError: YAML 文件未找到.
        KeyError: YAML 缺少必需字段.
    """
    if yaml_path is None:
        yaml_path = _find_yaml_path()

    yaml_sha256 = _compute_sha256(yaml_path)

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    panels = cfg.get("panels", {})
    if not panels:
        raise KeyError("calibration_target.yaml: missing 'panels' section")

    geom = TargetGeometry()
    geom.yaml_path = yaml_path
    geom.yaml_sha256 = yaml_sha256
    geom.panels = panels

    # face_poses_target
    geom.face_poses_target = {}
    for name, panel in panels.items():
        pt = panel.get("pose_target", {})
        if pt and "xyz" in pt and "rpy" in pt:
            geom.face_poses_target[name] = {
                "xyz": list(pt["xyz"]),
                "rpy": list(pt["rpy"]),
            }

    # T_target_face
    geom.T_target_face = {}
    for name, fp in geom.face_poses_target.items():
        T = _euler_matrix(fp["rpy"][0], fp["rpy"][1], fp["rpy"][2])
        T[:3, 3] = fp["xyz"]
        geom.T_target_face[name] = T

    # Pattern face builders
    geom.charuco_faces = build_charuco_faces(panels)
    geom.aruco_faces = build_aruco_faces(panels)
    geom.apriltag_faces = build_apriltag_faces(panels)

    # face_names
    geom.face_names = sorted(panels.keys())

    # face_pattern_types
    geom.face_pattern_types = {}
    for name in geom.face_names:
        if name in geom.charuco_faces:
            geom.face_pattern_types[name] = "charuco"
        elif name in geom.aruco_faces:
            geom.face_pattern_types[name] = "aruco"
        elif name in geom.apriltag_faces:
            geom.face_pattern_types[name] = "apriltag"

    return geom


def build_charuco_faces(panels):
    """从 YAML panels 构造 ChArUco face 定义."""
    from cv2 import aruco
    faces = {}
    for name, panel in panels.items():
        if panel.get("pattern") != "charuco":
            continue
        board_cfg = panel.get("board", {})
        if not board_cfg:
            continue
        sx = board_cfg["squares_x"]
        sy = board_cfg["squares_y"]
        sq_m = board_cfg["square_length_m"]
        mk_m = board_cfg["marker_length_m"]
        dict_name = board_cfg.get("dictionary", "DICT_5X5_1000")
        dict_id = getattr(aruco, dict_name, aruco.DICT_5X5_1000)
        first_id = int(board_cfg["marker_ids"][0])

        board = aruco.CharucoBoard_create(sx, sy, sq_m, mk_m,
                                          aruco.getPredefinedDictionary(dict_id))
        faces[name] = {
            "sx": sx, "sy": sy, "sq_m": sq_m, "mk_m": mk_m,
            "dict_id": dict_id, "id_start": first_id,
            "board": board,
            "dict_name": dict_name,
        }
    return faces


def build_aruco_faces(panels):
    """从 YAML panels 构造 ArUco face 定义."""
    from cv2 import aruco
    faces = {}
    for name, panel in panels.items():
        if panel.get("pattern") != "aruco":
            continue
        dict_name = panel.get("dictionary", "DICT_4X4_50")
        dict_id = getattr(aruco, dict_name, aruco.DICT_4X4_50)
        tag_ids = list(panel.get("tag_ids", []))
        marker_size_m = float(panel.get("marker_size_m", 0.076))

        # centers: 优先 tag_centers_face_m, fallback marker_centers_face_m
        centers_raw = panel.get("tag_centers_face_m",
                                panel.get("marker_centers_face_m", {}))
        positions = {}
        for tid, pos in centers_raw.items():
            positions[int(tid)] = tuple(pos)

        faces[name] = {
            "marker_size_m": marker_size_m,
            "marker_ids": tag_ids,
            "dict_id": dict_id,
            "dict_name": dict_name,
            "positions": positions,
        }
    return faces


def build_apriltag_faces(panels):
    """从 YAML panels 构造 AprilTag face 定义."""
    faces = {}
    for name, panel in panels.items():
        if panel.get("pattern") not in ("apriltag_grid", "apriltag_single", "apriltag"):
            continue
        tag_ids = list(panel.get("tag_ids", []))
        tag_size_m = float(panel.get("tag_size_m", 0.07))
        centers_raw = panel.get("tag_centers_face_m", {})
        positions = {}
        for tid, pos in centers_raw.items():
            positions[int(tid)] = tuple(pos)

        faces[name] = {
            "tag_size": tag_size_m,
            "tag_ids": tag_ids,
            "positions": positions,
        }
    return faces


def _euler_matrix(roll, pitch, yaw):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[0, 0] = cy * cp
    T[0, 1] = cy * sp * sr - sy * cr
    T[0, 2] = cy * sp * cr + sy * sr
    T[1, 0] = sy * cp
    T[1, 1] = sy * sp * sr + cy * cr
    T[1, 2] = sy * sp * cr - cy * sr
    T[2, 0] = -sp
    T[2, 1] = cp * sr
    T[2, 2] = cp * cr
    return T

"""
Calibration Target GT 几何 — 从 model.sdf 精确构建.

Scope:
  CALIBRATION_TARGET_BODY: main_body + top_offset_block + 5 panels
  FULL_CALIBRATION_FIXTURE: BODY + spreader_bar + cables

禁止:
  - 读取 motor_housing_cylinder
  - 读取 simple_hanging_workpiece.default_type
  - 用圆柱近似面板
"""
import os
import hashlib
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SDF_PATH = os.path.join(os.path.dirname(__file__), "..", "..",
                        "models", "calibration_target", "model.sdf")

# ── model.sdf 解析出的常量 ──
# 所有尺寸和位姿直接来自 V5 model.sdf

MAIN_BODY = {
    "name": "main_body",
    "geometry": "box",
    "size": [0.34, 0.28, 0.24],  # x, y, z
    "pose": [0, 0, 0, 0, 0, 0],  # xyz rpy
    "in_body_scope": True,
    "in_fixture_scope": True,
}

TOP_OFFSET_BLOCK = {
    "name": "top_offset_block",
    "geometry": "box",
    "size": [0.06, 0.05, 0.04],
    "pose": [0.13, 0.11, 0.14, 0, 0, 0],
    "in_body_scope": True,
    "in_fixture_scope": True,
}

PANELS = [
    {"name": "front_panel", "geometry": "box", "size": [0.001, 0.18, 0.24],
     "pose": [0.171, 0, 0, 0, 1.570796, 0],
     "in_body_scope": True, "in_fixture_scope": True},
    {"name": "back_panel", "geometry": "box", "size": [0.001, 0.18, 0.24],
     "pose": [-0.171, 0, 0, 0, -1.570796, 0],
     "in_body_scope": True, "in_fixture_scope": True},
    {"name": "left_panel", "geometry": "box", "size": [0.22, 0.001, 0.18],
     "pose": [0, 0.141, 0, -1.570796, 0, 0],
     "in_body_scope": True, "in_fixture_scope": True},
    {"name": "right_panel", "geometry": "box", "size": [0.22, 0.001, 0.18],
     "pose": [0, -0.141, 0, 1.570796, 0, 0],
     "in_body_scope": True, "in_fixture_scope": True},
    {"name": "top_panel", "geometry": "box", "size": [0.16, 0.16, 0.001],
     "pose": [0, 0, 0.121, 0, 0, 0],
     "in_body_scope": True, "in_fixture_scope": True},
]

FIXTURE_ONLY = [
    {"name": "spreader_bar", "geometry": "box", "size": [0.03, 0.30, 0.03],
     "pose": [0, 0, 0.605, 0, 0, 0],
     "in_body_scope": False, "in_fixture_scope": True},
    {"name": "left_cable", "geometry": "cylinder", "radius": 0.003, "length": 0.470,
     "pose": [0, -0.115, 0.355, 0, 0, 0],
     "in_body_scope": False, "in_fixture_scope": True},
    {"name": "right_cable", "geometry": "cylinder", "radius": 0.003, "length": 0.470,
     "pose": [0, 0.115, 0.355, 0, 0, 0],
     "in_body_scope": False, "in_fixture_scope": True},
]

ALL_LINKS = [MAIN_BODY, TOP_OFFSET_BLOCK] + PANELS + FIXTURE_ONLY


def _pose_to_matrix(pose_xyzrpy):
    """SDF pose (x y z roll pitch yaw) → 4×4 SE(3)."""
    x, y, z, roll, pitch, yaw = pose_xyzrpy
    R = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def sample_box(size, pose_matrix, n_points):
    """在 box 表面采样点 (SDF frame)."""
    sx, sy, sz = size
    # 6 个面,每个面 n/6 点
    n_face = max(1, n_points // 6)
    np.random.seed(42)
    all_pts = []
    half = np.array([sx / 2, sy / 2, sz / 2])
    # ±X faces
    for sign in [-1, 1]:
        u = np.random.uniform(-half[1], half[1], n_face)
        v = np.random.uniform(-half[2], half[2], n_face)
        pts = np.column_stack([np.full(n_face, sign * half[0]), u, v])
        all_pts.append(pts)
    # ±Y faces
    for sign in [-1, 1]:
        u = np.random.uniform(-half[0], half[0], n_face)
        v = np.random.uniform(-half[2], half[2], n_face)
        pts = np.column_stack([u, np.full(n_face, sign * half[1]), v])
        all_pts.append(pts)
    # ±Z faces
    for sign in [-1, 1]:
        u = np.random.uniform(-half[0], half[0], n_face)
        v = np.random.uniform(-half[1], half[1], n_face)
        pts = np.column_stack([u, v, np.full(n_face, sign * half[2])])
        all_pts.append(pts)
    pts = np.vstack(all_pts)
    # transform by pose
    pts_h = (pose_matrix[:3, :3] @ pts.T + pose_matrix[:3, 3:4]).T
    return pts_h


def sample_cylinder(radius, length, pose_matrix, n_points):
    """在圆柱面采样点."""
    np.random.seed(42)
    n_side = max(1, int(n_points * 0.85))
    theta = np.random.uniform(0, 2 * np.pi, n_side)
    z = np.random.uniform(-length / 2, length / 2, n_side)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    pts = np.column_stack([x, y, z])

    n_cap = max(1, n_points - n_side)
    r_cap = np.sqrt(np.random.uniform(0, radius**2, n_cap // 2 + 1))
    theta_cap = np.random.uniform(0, 2 * np.pi, n_cap // 2 + 1)
    xc = r_cap * np.cos(theta_cap)
    yc = r_cap * np.sin(theta_cap)
    caps = np.vstack([
        np.column_stack([xc, yc, np.full(len(xc), length / 2)]),
        np.column_stack([xc, yc, np.full(len(xc), -length / 2)]),
    ])
    pts = np.vstack([pts, caps])
    pts_h = (pose_matrix[:3, :3] @ pts.T + pose_matrix[:3, 3:4]).T
    return pts_h


def build_gt_pointcloud(scope="CALIBRATION_TARGET_BODY", total_points=60000):
    """构建 GT 点云 (在 SDF object frame).

    Args:
        scope: "CALIBRATION_TARGET_BODY" | "FULL_CALIBRATION_FIXTURE"
        total_points: 总采样点数

    Returns:
        (points_Nx3, link_labels: list of str per point,
         manifest: dict)
    """
    links = []
    for link in ALL_LINKS:
        if scope == "CALIBRATION_TARGET_BODY" and link["in_body_scope"]:
            links.append(link)
        elif scope == "FULL_CALIBRATION_FIXTURE" and link["in_fixture_scope"]:
            links.append(link)

    # 按体积分配点数
    volumes = []
    for link in links:
        if link["geometry"] == "box":
            sx, sy, sz = link["size"]
            volumes.append(sx * sy * sz)
        elif link["geometry"] == "cylinder":
            volumes.append(np.pi * link["radius"]**2 * link["length"])
    total_vol = sum(volumes)
    n_per_link = [max(100, int(total_points * v / total_vol)) for v in volumes]

    all_pts = []
    all_labels = []

    for link, n in zip(links, n_per_link):
        T = _pose_to_matrix(link["pose"])
        if link["geometry"] == "box":
            pts = sample_box(link["size"], T, n)
        elif link["geometry"] == "cylinder":
            pts = sample_cylinder(link["radius"], link["length"], T, n)
        else:
            continue
        all_pts.append(pts)
        all_labels.extend([link["name"]] * len(pts))

    points = np.vstack(all_pts)

    # 构建 manifest
    manifest = {
        "schema": "cr5_calibration_target_gt_v1",
        "scope": scope,
        "source_sdf": os.path.abspath(SDF_PATH),
        "source_sdf_sha256": hashlib.sha256(
            open(SDF_PATH, "rb").read()).hexdigest(),
        "coordinate_frame": "SDF object frame (+X=front, +Y=left, +Z=up)",
        "total_points": int(len(points)),
        "links": [],
    }
    for link in links:
        entry = {
            "name": link["name"],
            "geometry": link["geometry"],
        }
        if link["geometry"] == "box":
            entry["size"] = link["size"]
        elif link["geometry"] == "cylinder":
            entry["radius"] = link["radius"]
            entry["length"] = link["length"]
        entry["pose_xyzrpy"] = link["pose"]
        entry["in_body_scope"] = link["in_body_scope"]
        entry["in_fixture_scope"] = link["in_fixture_scope"]
        manifest["links"].append(entry)

    return points, all_labels, manifest


def get_gt_aabb_sdf_frame():
    """计算 CALIBRATION_TARGET_BODY 在 SDF frame 的 AABB."""
    body_links = [l for l in ALL_LINKS if l["in_body_scope"]]
    all_corners = []
    for link in body_links:
        T = _pose_to_matrix(link["pose"])
        if link["geometry"] == "box":
            sx, sy, sz = link["size"]
            for dx in [-sx / 2, sx / 2]:
                for dy in [-sy / 2, sy / 2]:
                    for dz in [-sz / 2, sz / 2]:
                        p = np.array([dx, dy, dz, 1.0])
                        all_corners.append(T @ p)
        elif link["geometry"] == "cylinder":
            r = link["radius"]
            l = link["length"]
            for dx in [-r, r]:
                for dy in [-r, r]:
                    for dz in [-l / 2, l / 2]:
                        p = np.array([dx, dy, dz, 1.0])
                        all_corners.append(T @ p)
    corners = np.array(all_corners)
    return {"min": corners[:, :3].min(axis=0).tolist(),
            "max": corners[:, :3].max(axis=0).tolist()}

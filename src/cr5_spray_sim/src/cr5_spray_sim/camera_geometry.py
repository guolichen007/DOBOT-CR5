"""
Camera geometry utilities — look-at, FOV coverage, projection.

从 spawn_fixed_cameras.py 提取的几何计算，独立为可测试模块。
"""
import math
import numpy as np


def _rpy_from_rotation(R):
    """从 3×3 旋转矩阵提取 RPY (ZYX convention)."""
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
    """计算相机 Gazebo link 朝向目标的旋转矩阵.

    Gazebo 约定: camera link +X = 镜头方向 (指向目标),
    link +Z 尽量接近世界 +Z (保持水平).

    Returns:
        (R_3x3, direction_norm): 旋转矩阵 [cam_x|cam_y|cam_z] (列向量) + 单位视线方向
    """
    cam = np.array(cam_pos, dtype=np.float64)
    tgt = np.array(target_pos, dtype=np.float64)
    d_raw = tgt - cam  # 从相机指向目标
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
    """计算相机朝向目标的姿态 (Gazebo camera link convention).

    Args:
        cam_pos: [x, y, z] 相机位置 (world)
        target_pos: [x, y, z] 目标位置 (world)
        roll_offset_deg: roll 偏移角度 (绕视线旋转)

    Returns:
        dict: {roll, pitch, yaw (rad), distance_m, R (3×3 matrix),
               optical_z_angle_error_deg, image_up_vs_world_up_deg}
    """
    R, d, dist = _look_at_rotation(cam_pos, target_pos)
    roll, pitch, yaw = _rpy_from_rotation(R)

    # 应用 roll 偏移
    roll += math.radians(roll_offset_deg)

    # 光学 Z 轴误差 (光学 +Z = 拍摄方向)
    R_opt_to_link = np.array([
        [0, 1, 0],
        [0, 0, -1],
        [-1, 0, 0],
    ], dtype=np.float64)
    opt_z_world = R @ R_opt_to_link @ np.array([0, 0, 1])
    cos_angle = float(np.dot(opt_z_world, d))
    cos_angle = max(-1.0, min(1.0, cos_angle))
    opt_err = float(math.degrees(math.acos(cos_angle)))

    # 图像上方向 vs 世界上方向误差
    opt_y_world = R @ R_opt_to_link @ np.array([0, 1, 0])
    img_up = -opt_y_world
    world_z = np.array([0.0, 0.0, 1.0])
    cos_up = float(np.dot(img_up, world_z))
    cos_up = max(-1.0, min(1.0, cos_up))
    up_err = float(math.degrees(math.acos(abs(cos_up))))

    return {
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "distance_m": dist,
        "R": R,
        "optical_z_angle_error_deg": opt_err,
        "image_up_vs_world_up_deg": up_err,
    }


def compute_distance(pos_a, pos_b):
    """两点间欧氏距离。"""
    a = np.array(pos_a, dtype=np.float64)
    b = np.array(pos_b, dtype=np.float64)
    return float(np.linalg.norm(a - b))


def estimate_fov_coverage(cam_pos, target_center, target_size_xy,
                          hfov_deg=69.4, vfov_deg=None):
    """估算目标在相机图像中的覆盖率。

    Args:
        cam_pos: 相机位置 [x, y, z]
        target_center: 目标中心 [x, y, z]
        target_size_xy: 目标宽高 (width, height) 米
        hfov_deg: 水平视场角 (度)
        vfov_deg: 垂直视场角，默认由宽高比计算

    Returns:
        dict: {horizontal_fill_pct, vertical_fill_pct, distance_m}
    """
    distance = compute_distance(cam_pos, target_center)
    if distance < 1e-9:
        return {"horizontal_fill_pct": 100.0, "vertical_fill_pct": 100.0,
                "distance_m": 0.0}

    hfov_rad = math.radians(hfov_deg)
    # 水平覆盖范围
    h_coverage = 2.0 * distance * math.tan(hfov_rad / 2.0)
    h_fill_pct = (target_size_xy[0] / h_coverage) * 100.0

    if vfov_deg is None:
        # 默认 4:3 宽高比
        vfov_rad = hfov_rad * (3.0 / 4.0)
    else:
        vfov_rad = math.radians(vfov_deg)
    v_coverage = 2.0 * distance * math.tan(vfov_rad / 2.0)
    v_fill_pct = (target_size_xy[1] / v_coverage) * 100.0

    return {
        "horizontal_fill_pct": round(h_fill_pct, 1),
        "vertical_fill_pct": round(v_fill_pct, 1),
        "distance_m": round(distance, 3),
    }


def validate_camera_acceptance(metrics, thresholds):
    """验证相机覆盖是否满足验收阈值。

    Args:
        metrics: estimate_fov_coverage 返回的 dict
        thresholds: dict with min/max_*_fill_pct keys

    Returns:
        (passed: bool, violations: list[str])
    """
    violations = []

    min_h = thresholds.get("min_horizontal_fill_pct", 10)
    max_h = thresholds.get("max_horizontal_fill_pct", 60)
    min_v = thresholds.get("min_vertical_fill_pct", 10)
    max_v = thresholds.get("max_vertical_fill_pct", 70)

    if metrics["horizontal_fill_pct"] < min_h:
        violations.append(
            f"Horizontal fill {metrics['horizontal_fill_pct']}% < {min_h}%")
    if metrics["horizontal_fill_pct"] > max_h:
        violations.append(
            f"Horizontal fill {metrics['horizontal_fill_pct']}% > {max_h}%")
    if metrics["vertical_fill_pct"] < min_v:
        violations.append(
            f"Vertical fill {metrics['vertical_fill_pct']}% < {min_v}%")
    if metrics["vertical_fill_pct"] > max_v:
        violations.append(
            f"Vertical fill {metrics['vertical_fill_pct']}% > {max_v}%")

    return len(violations) == 0, violations

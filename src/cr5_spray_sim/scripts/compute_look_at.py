#!/usr/bin/env python3
"""
Compute look-at pose for Gazebo camera.
光轴 = camera optical frame +Z direction (ROS convention: +X right, +Y down, +Z forward).
Gazebo camera default axis: +X right, +Y up, +Z backward (looking -Z).
ROS optical frame: +X right, +Y down, +Z forward.

Transform: Gazebo camera → ROS optical frame = RPY(-PI/2, 0, -PI/2)

This script computes the Gazebo camera pose that makes the optical +Z
point toward the target.
"""
import sys
import math
import numpy as np
import yaml


def compute_look_at(cam_pos, target_pos, roll_offset_deg=0.0):
    """
    Compute RPY for Gazebo camera to look at target with stable horizon.

    Strategy (prevents upside-down images):
    1. optical +Z = direction to target (fixed)
    2. optical +Y (image down) = project world -Z onto plane ⟂ optical_z
       → ensures image "down" generally points world-down
    3. optical +X (image right) = cross(opt_y, opt_z)  — right-handed
    4. Apply optional roll_offset_deg around optical axis for fine-tuning.

    Returns dict with roll, pitch, yaw (rad), distance_m,
    optical_z_angle_error_deg, image_up_vs_world_up_deg.
    """
    cam = np.array(cam_pos, dtype=float)
    tgt = np.array(target_pos, dtype=float)

    # Direction from camera to target
    direction = tgt - cam
    dist = np.linalg.norm(direction)
    if dist < 1e-6:
        raise ValueError("Camera at same position as target")
    direction /= dist

    # ── ROS optical frame axes ──
    # +Z forward (toward target)
    optical_z = direction

    # +Y down in image — we want this to point generally world-down
    world_down = np.array([0.0, 0.0, -1.0])
    # Project world_down onto plane perpendicular to optical_z
    optical_y = world_down - np.dot(world_down, optical_z) * optical_z
    ny = np.linalg.norm(optical_y)

    if ny < 1e-6:
        # Camera looking straight down/up: optical_z ∥ world Z
        # Fall back: use world +Y (forward) as image-down reference
        optical_y = np.array([0.0, -1.0, 0.0])  # south = down in image
        ny = 1.0
    optical_y /= ny

    # +X right in image — complete right-handed frame
    optical_x = np.cross(optical_y, optical_z)
    nx = np.linalg.norm(optical_x)
    if nx < 1e-6:
        # Degenerate: force orthogonal
        optical_x = np.cross(optical_z, np.array([0.0, 1.0, 0.0]))
        nx = np.linalg.norm(optical_x)
    optical_x /= nx

    # Re-orthogonalize: ensure optical_y ⟂ optical_z and ⟂ optical_x
    optical_y = np.cross(optical_z, optical_x)
    optical_y /= np.linalg.norm(optical_y)

    # ── Optional roll around optical axis ──
    if abs(roll_offset_deg) > 1e-9:
        roll_rad = math.radians(roll_offset_deg)
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        ox = optical_x * cr + optical_y * sr
        oy = -optical_x * sr + optical_y * cr
        optical_x = ox
        optical_y = oy

    # Build rotation matrix: columns = optical frame axes in world coords
    R_optical_in_world = np.column_stack([optical_x, optical_y, optical_z])

    # ── Gazebo camera ↔ ROS optical correction ──
    # Gazebo default camera: +X right, +Y up, +Z backward (looks -Z)
    # ROS optical:           +X right, +Y down, +Z forward
    # R_corr maps Gazebo-cam axes → ROS-optical axes:
    #   rot_x(-π/2) · rot_z(-π/2)
    roll_corr = -math.pi / 2
    pitch_corr = 0.0
    yaw_corr = -math.pi / 2

    R_corr = R_from_rpy(roll_corr, pitch_corr, yaw_corr)
    R_gazebo = R_optical_in_world @ R_corr.T

    # Extract RPY from Gazebo rotation
    rpy = rpy_from_R(R_gazebo)

    # ── Quality metrics ──
    # Optical +Z vs target direction error
    optical_z_actual = R_gazebo @ R_corr @ np.array([0.0, 0.0, 1.0])
    cos_err = np.dot(optical_z_actual, direction)
    cos_err = np.clip(cos_err, -1.0, 1.0)
    angle_err_deg = math.degrees(math.acos(cos_err))

    # Image "up" vs world "up" deviation
    # Image up = -optical_y (optical +Y is down, so -Y is up)
    image_up = -R_optical_in_world[:, 1]  # second column = optical_y
    world_up = np.array([0.0, 0.0, 1.0])
    cos_up = np.dot(image_up, world_up)
    cos_up = np.clip(cos_up, -1.0, 1.0)
    image_up_err_deg = math.degrees(math.acos(cos_up))

    return {
        "roll": float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw": float(rpy[2]),
        "distance_m": float(dist),
        "optical_z_angle_error_deg": float(angle_err_deg),
        "image_up_vs_world_up_deg": float(image_up_err_deg),
    }


def R_from_rpy(r, p, y):
    """RPY (intrinsic ZYX) to rotation matrix."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    return Rz @ Ry @ Rx


def rpy_from_R(R):
    """Rotation matrix to RPY (intrinsic ZYX)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        r = math.atan2(R[2, 1], R[2, 2])
        p = math.atan2(-R[2, 0], sy)
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        p = math.atan2(-R[2, 0], sy)
        y = 0

    return (r, p, y)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam-x", type=float, required=True)
    parser.add_argument("--cam-y", type=float, required=True)
    parser.add_argument("--cam-z", type=float, required=True)
    parser.add_argument("--target-x", type=float, default=0.72)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.88)
    parser.add_argument("--roll-offset-deg", type=float, default=0.0,
                        help="Optional roll around optical axis (degrees)")
    parser.add_argument("--yaml", action="store_true",
                        help="Output as YAML")
    args = parser.parse_args()

    result = compute_look_at(
        [args.cam_x, args.cam_y, args.cam_z],
        [args.target_x, args.target_y, args.target_z],
        roll_offset_deg=args.roll_offset_deg)

    if args.yaml:
        print(yaml.dump(result, default_flow_style=False))
    else:
        for k, v in result.items():
            print(f"{k}: {v}")

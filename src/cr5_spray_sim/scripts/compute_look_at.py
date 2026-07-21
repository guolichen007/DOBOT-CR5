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


def compute_look_at(cam_pos, target_pos):
    """
    Compute RPY for Gazebo camera to look at target.
    Returns (roll, pitch, yaw) in radians.
    """
    cam = np.array(cam_pos, dtype=float)
    tgt = np.array(target_pos, dtype=float)

    # Direction from camera to target
    direction = tgt - cam
    dist = np.linalg.norm(direction)
    if dist < 1e-6:
        raise ValueError("Camera at same position as target")
    direction /= dist

    # ROS optical frame: +Z forward (toward target)
    optical_z = direction

    # ROS optical frame: +X right
    # Use world +Z (up) to derive right vector
    world_up = np.array([0, 0, 1])
    optical_x = np.cross(world_up, optical_z)
    nx = np.linalg.norm(optical_x)

    if nx < 1e-6:
        # Vertical look: optical_z is parallel to world_up
        # Use world +Y as alternative reference
        world_y = np.array([0, 1, 0])
        optical_x = np.cross(optical_z, world_y)
        nx = np.linalg.norm(optical_x)
    optical_x /= nx

    # ROS optical frame: +Y down
    optical_y = np.cross(optical_z, optical_x)

    # Build rotation matrix: columns are optical frame axes in world coords
    R_optical_in_world = np.column_stack([optical_x, optical_y, optical_z])

    # Gazebo camera pose = R_optical * R_correction
    # where R_correction converts from Gazebo camera frame to ROS optical frame
    # Gazebo camera: +X right, +Y up, +Z backward
    # ROS optical: +X right, +Y down, +Z forward
    # R_correction = R_optical_in_gazebo
    #   = rot_x(-PI/2) * rot_z(-PI/2)
    roll_corr = -math.pi / 2
    pitch_corr = 0
    yaw_corr = -math.pi / 2

    R_corr = R_from_rpy(roll_corr, pitch_corr, yaw_corr)
    R_gazebo = R_optical_in_world @ R_corr.T

    # Extract RPY from Gazebo rotation
    rpy = rpy_from_R(R_gazebo)

    # Compute optical +Z vs target direction angle error
    optical_z_actual = R_gazebo @ R_corr @ np.array([0, 0, 1])
    cos_err = np.dot(optical_z_actual, direction)
    cos_err = np.clip(cos_err, -1, 1)
    angle_err_deg = math.degrees(math.acos(cos_err))

    return {
        "roll": float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw": float(rpy[2]),
        "distance_m": float(dist),
        "optical_z_angle_error_deg": float(angle_err_deg),
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
    parser.add_argument("--yaml", action="store_true",
                        help="Output as YAML")
    args = parser.parse_args()

    result = compute_look_at(
        [args.cam_x, args.cam_y, args.cam_z],
        [args.target_x, args.target_y, args.target_z])

    if args.yaml:
        print(yaml.dump(result, default_flow_style=False))
    else:
        for k, v in result.items():
            print(f"{k}: {v}")

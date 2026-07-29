#!/usr/bin/env python3
"""
Calibration target pose setter — simulation-only tool.

Moves the calibration target (simple_hanging_workpiece) to predefined poses
for multi-frame calibration data collection.

Each pose is a small perturbation from the default position.
Avoids collisions and keeps at least one face visible to each camera.

Usage:
  rosrun cr5_spray_sim set_calibration_target_pose.py --pose center
  rosrun cr5_spray_sim set_calibration_target_pose.py --pose left
  rosrun cr5_spray_sim set_calibration_target_pose.py --list
"""
import sys
import math
import argparse
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Point, Quaternion


# ── V5: 目标降低至 z=0.60, 相机靠近至 ~0.85m ──
POSE_PRESETS = {
    "center": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 0, 0],
        "desc": "Default centered pose z=0.60",
    },
    "left": {
        "xyz": [0.68, 0.08, 0.60],
        "rpy_deg": [0, 0, 0],
        "desc": "Shifted left",
    },
    "right": {
        "xyz": [0.68, -0.08, 0.60],
        "rpy_deg": [0, 0, 0],
        "desc": "Shifted right",
    },
    "up": {
        "xyz": [0.68, 0.0, 0.72],
        "rpy_deg": [0, 0, 0],
        "desc": "Raised to z=0.72",
    },
    "down": {
        "xyz": [0.68, 0.0, 0.48],
        "rpy_deg": [0, 0, 0],
        "desc": "Lowered to z=0.48",
    },
    "yaw_p15": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 0, 15],
        "desc": "Yaw +15deg",
    },
    "yaw_m15": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 0, -15],
        "desc": "Yaw -15deg",
    },
    "yaw_p25": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 0, 25],
        "desc": "Yaw +25deg (展示侧面)",
    },
    "yaw_m25": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 0, -25],
        "desc": "Yaw -25deg (展示侧面)",
    },
    "pitch_p10": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 10, 0],
        "desc": "Pitch +10deg (顶面可见)",
    },
    "pitch_m10": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, -10, 0],
        "desc": "Pitch -10deg (底面方向)",
    },
    "pitch_p20": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, 20, 0],
        "desc": "Pitch +20deg (顶面大面积可见)",
    },
    "pitch_m20": {
        "xyz": [0.68, 0.0, 0.60],
        "rpy_deg": [0, -20, 0],
        "desc": "Pitch -20deg",
    },
    "combo_yp": {
        "xyz": [0.68, 0.05, 0.55],
        "rpy_deg": [0, 12, 15],
        "desc": "Yaw+15 Pitch+12 组合",
    },
    "combo_ym": {
        "xyz": [0.68, -0.05, 0.65],
        "rpy_deg": [0, -12, -15],
        "desc": "Yaw-15 Pitch-12 组合",
    },
}


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Convert roll/pitch/yaw (degrees) to quaternion [x,y,z,w]."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def main():
    parser = argparse.ArgumentParser(description="Set calibration target pose")
    parser.add_argument("--pose", default="center", help="Pose preset name")
    parser.add_argument("--list", action="store_true", help="List available presets")
    parser.add_argument("--model", default="simple_hanging_workpiece",
                        help="Gazebo model name")
    args = parser.parse_args()

    if args.list:
        print("Available calibration target pose presets:")
        for name, p in POSE_PRESETS.items():
            print(f"  {name:12s}  xyz={p['xyz']}  rpy={p['rpy_deg']}deg  {p['desc']}")
        return

    preset = POSE_PRESETS.get(args.pose)
    if preset is None:
        print(f"Unknown preset: {args.pose}")
        print("Use --list to see available presets")
        sys.exit(1)

    rospy.init_node("set_calibration_target_pose", anonymous=True)

    rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    q = rpy_to_quat(*preset["rpy_deg"])

    req = SetModelStateRequest()
    req.model_state.model_name = args.model
    req.model_state.pose = Pose(
        position=Point(*preset["xyz"]),
        orientation=Quaternion(*q),
    )
    req.model_state.reference_frame = "world"

    try:
        resp = svc(req)
        if resp.success:
            rospy.loginfo("Target pose set: %s → %s", args.pose, preset["desc"])
            print(f"POSE_SET: {args.pose} → xyz={preset['xyz']} rpy={preset['rpy_deg']}deg")
        else:
            rospy.logerr("Failed to set pose: %s", resp.status_message)
            print(f"POSE_SET_FAILED: {resp.status_message}")
            sys.exit(1)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()

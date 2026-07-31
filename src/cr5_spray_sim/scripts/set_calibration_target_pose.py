#!/usr/bin/env python3
"""
Calibration target pose setter — simulation-only tool.

Moves the calibration target (simple_hanging_workpiece) to predefined poses
for multi-frame calibration data collection.

V8.15: 所有预设改为相对于 BASE_TARGET (从 simulation_scene.yaml 读取) 的偏移量。
不再硬编码世界坐标 (0.68, 0.60 等)。

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
from cr5_spray_sim.scene_config import get_base_target_pose


# ── V8.15: 相对于 BASE_TARGET 的偏移量, 不再硬编码世界坐标 ──
# base 从 simulation_scene.yaml 的 simple_hanging_workpiece.position 读取
POSE_DELTAS = {
    "center": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 0, 0],
        "desc": "Base centered pose",
    },
    "left": {
        "dx": 0.0, "dy": 0.08, "dz": 0.0,
        "rp_deg": [0, 0, 0],
        "desc": "Shifted left (+Y)",
    },
    "right": {
        "dx": 0.0, "dy": -0.08, "dz": 0.0,
        "rp_deg": [0, 0, 0],
        "desc": "Shifted right (-Y)",
    },
    "up": {
        "dx": 0.0, "dy": 0.0, "dz": 0.10,
        "rp_deg": [0, 0, 0],
        "desc": "Raised +0.10m",
    },
    "down": {
        "dx": 0.0, "dy": 0.0, "dz": -0.14,
        "rp_deg": [0, 0, 0],
        "desc": "Lowered -0.14m",
    },
    "yaw_p15": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 0, 15],
        "desc": "Yaw +15deg",
    },
    "yaw_m15": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 0, -15],
        "desc": "Yaw -15deg",
    },
    "yaw_p25": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 0, 25],
        "desc": "Yaw +25deg",
    },
    "yaw_m25": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 0, -25],
        "desc": "Yaw -25deg",
    },
    "pitch_p10": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 10, 0],
        "desc": "Pitch +10deg",
    },
    "pitch_m10": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, -10, 0],
        "desc": "Pitch -10deg",
    },
    "pitch_p20": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, 20, 0],
        "desc": "Pitch +20deg",
    },
    "pitch_m20": {
        "dx": 0.0, "dy": 0.0, "dz": 0.0,
        "rp_deg": [0, -20, 0],
        "desc": "Pitch -20deg",
    },
    "combo_yp": {
        "dx": 0.0, "dy": 0.05, "dz": -0.07,
        "rp_deg": [0, 12, 15],
        "desc": "Yaw+15 Pitch+12 combo",
    },
    "combo_ym": {
        "dx": 0.0, "dy": -0.05, "dz": 0.03,
        "rp_deg": [0, -12, -15],
        "desc": "Yaw-15 Pitch-12 combo",
    },
}


def get_base_xyz():
    """获取 BASE_TARGET 世界坐标."""
    try:
        return get_base_target_pose()
    except Exception as e:
        rospy.logwarn("Cannot read YAML base target: %s, using default", e)
        return (0.72, 0.0, 0.62)


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
    parser = argparse.ArgumentParser(description="Set calibration target pose (relative offsets)")
    parser.add_argument("--pose", default="center", help="Pose preset name")
    parser.add_argument("--list", action="store_true", help="List available presets")
    parser.add_argument("--model", default="simple_hanging_workpiece",
                        help="Gazebo model name")
    args = parser.parse_args()

    if args.list:
        base = get_base_xyz()
        print("BASE_TARGET (from YAML): x={:.3f}, y={:.3f}, z={:.3f}".format(*base))
        print("Available calibration target pose presets (offsets from BASE_TARGET):")
        for name, d in POSE_DELTAS.items():
            ax = base[0] + d["dx"]
            ay = base[1] + d["dy"]
            az = base[2] + d["dz"]
            print("  {:12s}  world=({:.3f},{:.3f},{:.3f})  rpy={}deg  {}".format(
                name, ax, ay, az, d["rp_deg"], d["desc"]))
        return

    delta = POSE_DELTAS.get(args.pose)
    if delta is None:
        print("Unknown preset: {}".format(args.pose))
        print("Use --list to see available presets")
        sys.exit(1)

    rospy.init_node("set_calibration_target_pose", anonymous=True)
    base = get_base_xyz()

    rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
    svc = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # 计算实际世界坐标 = base + delta
    world_xyz = [
        base[0] + delta["dx"],
        base[1] + delta["dy"],
        base[2] + delta["dz"],
    ]

    q = rpy_to_quat(*delta["rp_deg"])

    req = SetModelStateRequest()
    req.model_state.model_name = args.model
    req.model_state.pose = Pose(
        position=Point(*world_xyz),
        orientation=Quaternion(*q),
    )
    req.model_state.reference_frame = "world"

    try:
        resp = svc(req)
        if resp.success:
            rospy.loginfo("Target pose set: %s → world=%s %s",
                          args.pose, world_xyz, delta["desc"])
            print("POSE_SET: {} → world=({:.3f},{:.3f},{:.3f}) rpy={}deg ({})".format(
                args.pose, world_xyz[0], world_xyz[1], world_xyz[2],
                delta["rp_deg"], delta["desc"]))
        else:
            rospy.logerr("Failed to set pose: %s", resp.status_message)
            print("POSE_SET_FAILED: {}".format(resp.status_message))
            sys.exit(1)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3.3.6 Model Pose Stability Check.

使用正确的 Gazebo service request 对象采样模型姿态。
两次采样间隔 1 秒，验证静态模型稳定性和 CR5 Link6 高度。

用法:
  rosrun cr5_spray_sim check_model_poses_v336.py

输出到 stderr:
  MODEL_POSES_STABLE
  MODEL_POSES_UNSTABLE

退出码:
  0 = MODEL_POSES_STABLE
  1 = 服务不可用
  2 = 姿态不稳定或异常
"""
import sys
import math
import time
import rospy
from gazebo_msgs.srv import (
    GetModelState, GetModelStateRequest,
    GetLinkState, GetLinkStateRequest,
)

# 需要检查的模型及其类型
STATIC_MODELS = {
    "simple_goalpost_frame",
    "simple_hanging_workpiece",
    "pedestal_fl",
    "pedestal_fr",
    "pedestal_rear",
    "cam_front_left",
    "cam_front_right",
    "cam_rear",
    "ground_plane",
}
CR5_MODEL = "cr5_robot"
LINK6_NAME = "cr5_robot::Link6"
MIN_LINK6_Z = 0.80  # 米

# 静态模型容差
MAX_TRANSLATION_DRIFT_MM = 0.5
MAX_ROTATION_DRIFT_DEG = 0.05


def _to_xyz_rpy(pose_dict):
    """从 pose dict 提取 xyz 和 rpy."""
    from tf.transformations import euler_from_quaternion
    x = pose_dict["x"]
    y = pose_dict["y"]
    z = pose_dict["z"]
    qx = pose_dict["qx"]
    qy = pose_dict["qy"]
    qz = pose_dict["qz"]
    qw = pose_dict["qw"]
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return (x, y, z, roll, pitch, yaw)


def _check_finite(label, x, y, z):
    """验证坐标 finite + 合理范围."""
    for val, axis in [(x, "x"), (y, "y"), (z, "z")]:
        if not math.isfinite(val):
            rospy.logerr("%s: %s=%s non-finite", label, axis, val)
            return False
        if abs(val) > 50.0:  # 不合理的大坐标
            rospy.logerr("%s: %s=%s out of bounds", label, axis, val)
            return False
    return True


def _get_model_pose(model_name):
    """正确调用 GetModelState service."""
    rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
    srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    req = GetModelStateRequest()
    req.model_name = model_name
    req.relative_entity_name = "world"
    resp = srv(req)
    if not resp.success:
        rospy.logerr("get_model_state(%s) failed: %s", model_name, resp.status_message)
        return None
    p = resp.pose.position
    o = resp.pose.orientation
    return {"x": p.x, "y": p.y, "z": p.z,
            "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w}


def _get_link_state(link_name):
    """正确调用 GetLinkState service."""
    rospy.wait_for_service("/gazebo/get_link_state", timeout=5.0)
    srv = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    req = GetLinkStateRequest()
    req.link_name = link_name
    req.reference_frame = "world"
    resp = srv(req)
    if not resp.success:
        rospy.logerr("get_link_state(%s) failed: %s", link_name, resp.status_message)
        return None
    p = resp.link_state.pose.position
    o = resp.link_state.pose.orientation
    return {"x": p.x, "y": p.y, "z": p.z,
            "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w}


def main():
    rospy.init_node("check_model_poses_v336", anonymous=True,
                    log_level=rospy.WARN)

    # ====== 第一轮采样 ======
    rospy.loginfo("Sampling model poses (round 1)...")
    round1 = {}

    # 静态模型
    for name in sorted(STATIC_MODELS):
        pose = _get_model_pose(name)
        if pose is None:
            rospy.logerr("round 1: model %s pose is None", name)
            sys.stderr.write("MODEL_POSES_UNSTABLE\n")
            sys.stderr.flush()
            sys.exit(2)
        if not _check_finite(name, pose["x"], pose["y"], pose["z"]):
            sys.stderr.write("MODEL_POSES_UNSTABLE\n")
            sys.stderr.flush()
            sys.exit(2)
        round1[name] = pose

    # CR5 root
    cr5_pose1 = _get_model_pose(CR5_MODEL)
    if cr5_pose1 is None:
        rospy.logerr("round 1: %s pose is None", CR5_MODEL)
        sys.stderr.write("MODEL_POSES_UNSTABLE\n")
        sys.stderr.flush()
        sys.exit(2)
    if not _check_finite(CR5_MODEL, cr5_pose1["x"], cr5_pose1["y"], cr5_pose1["z"]):
        sys.stderr.write("MODEL_POSES_UNSTABLE\n")
        sys.stderr.flush()
        sys.exit(2)

    # CR5 Link6
    link6_1 = _get_link_state(LINK6_NAME)
    if link6_1 is None:
        rospy.logerr("round 1: %s pose is None", LINK6_NAME)
        sys.stderr.write("MODEL_POSES_UNSTABLE\n")
        sys.stderr.flush()
        sys.exit(2)
    link6_z1 = link6_1["z"]

    # ====== 等待 1 秒 ======
    rospy.loginfo("Waiting 1s for stability check...")
    time.sleep(1.0)

    # ====== 第二轮采样 ======
    rospy.loginfo("Sampling model poses (round 2)...")
    all_stable = True

    for name in sorted(STATIC_MODELS):
        pose = _get_model_pose(name)
        if pose is None:
            rospy.logerr("round 2: model %s pose is None", name)
            all_stable = False
            continue
        if not _check_finite(name, pose["x"], pose["y"], pose["z"]):
            all_stable = False
            continue

        p1 = round1[name]
        dx = abs(pose["x"] - p1["x"]) * 1000  # mm
        dy = abs(pose["y"] - p1["y"]) * 1000
        dz = abs(pose["z"] - p1["z"]) * 1000

        # 姿态变化 (简单用四元数差的模)
        from tf.transformations import euler_from_quaternion
        _, _, yaw1 = euler_from_quaternion(
            [p1["qx"], p1["qy"], p1["qz"], p1["qw"]])
        _, _, yaw2 = euler_from_quaternion(
            [pose["qx"], pose["qy"], pose["qz"], pose["qw"]])
        dyaw_deg = abs(yaw2 - yaw1) * 180 / math.pi
        if dyaw_deg > math.pi:
            dyaw_deg = 360 - dyaw_deg

        if dx > MAX_TRANSLATION_DRIFT_MM or dy > MAX_TRANSLATION_DRIFT_MM or \
           dz > MAX_TRANSLATION_DRIFT_MM:
            rospy.logerr("%s drifted: dx=%.3fmm dy=%.3fmm dz=%.3fmm",
                         name, dx, dy, dz)
            all_stable = False
        if dyaw_deg > MAX_ROTATION_DRIFT_DEG:
            rospy.logerr("%s rotated: dyaw=%.4f deg", name, dyaw_deg)
            all_stable = False

    # CR5 root 稳定性
    cr5_pose2 = _get_model_pose(CR5_MODEL)
    if cr5_pose2 is not None and _check_finite(CR5_MODEL, cr5_pose2["x"],
                                                cr5_pose2["y"], cr5_pose2["z"]):
        dx = abs(cr5_pose2["x"] - cr5_pose1["x"]) * 1000
        dy = abs(cr5_pose2["y"] - cr5_pose1["y"]) * 1000
        dz = abs(cr5_pose2["z"] - cr5_pose1["z"]) * 1000
        rospy.loginfo("CR5 root drift: dx=%.3fmm dy=%.3fmm dz=%.3fmm", dx, dy, dz)

    # Link6 高度
    link6_2 = _get_link_state(LINK6_NAME)
    if link6_2 is not None:
        link6_z2 = link6_2["z"]
        rospy.loginfo("Link6 z1=%.4f z2=%.4f (min=%.2f)", link6_z1, link6_z2, MIN_LINK6_Z)
        if link6_z2 < MIN_LINK6_Z:
            rospy.logerr("Link6.z=%.4f below minimum %.2f", link6_z2, MIN_LINK6_Z)
            all_stable = False
    else:
        rospy.logerr("round 2: %s pose is None", LINK6_NAME)
        all_stable = False

    if all_stable:
        rospy.loginfo("all model poses stable")
        sys.stderr.write("MODEL_POSES_STABLE\n")
        sys.stderr.flush()
        sys.exit(0)
    else:
        rospy.logerr("model poses unstable")
        sys.stderr.write("MODEL_POSES_UNSTABLE\n")
        sys.stderr.flush()
        sys.exit(2)


if __name__ == "__main__":
    main()

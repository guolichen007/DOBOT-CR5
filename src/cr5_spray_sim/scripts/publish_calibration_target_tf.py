#!/usr/bin/env python3
"""
V4 标定目标内部 TF 发布 — 使用 StaticTransformBroadcaster.

替代原来的 calibration_target_state_publisher (robot_state_publisher),
只发布标定目标内部固定 frame, 不发布 CR5 Link1~Link6.

TF 树:
  object_frame → calibration_target_frame
    ├── calibration_target_front_frame
    ├── calibration_target_left_frame
    ├── calibration_target_right_frame
    ├── calibration_target_top_frame
    └── calibration_target_back_frame

world → object_frame 继续由 object_pose_tf_v31.py 根据 Gazebo pose 发布.

帧位姿来源于 calibration_target_v1.xacro 几何常量.
"""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


# 面板参数 (与 calibration_target_v1.xacro 一致)
BODY_SX = 0.30
BODY_SY = 0.22
BODY_SZ = 0.18
PANEL_THICK = 0.003

FRONT_X = BODY_SX / 2 + PANEL_THICK / 2   # 0.1515
BACK_X = -BODY_SX / 2 - PANEL_THICK / 2   # -0.1515
LEFT_Y = BODY_SY / 2 + PANEL_THICK / 2    # 0.1115
RIGHT_Y = -BODY_SY / 2 - PANEL_THICK / 2  # -0.1115
TOP_Z = BODY_SZ / 2 + PANEL_THICK / 2     # 0.0915

# 面 frame 定义: (child_frame, x, y, z, roll, pitch, yaw)
# 所有面 frame 保持 identity 方向 (与 xacro 中 identity joint 一致)
FACE_DEFS = [
    ("calibration_target_front_frame", FRONT_X, 0.0, 0.0, 0.0, 0.0, 0.0),
    ("calibration_target_left_frame", 0.0, LEFT_Y, 0.0, 0.0, 0.0, 0.0),
    ("calibration_target_right_frame", 0.0, RIGHT_Y, 0.0, 0.0, 0.0, 0.0),
    ("calibration_target_top_frame", 0.0, 0.0, TOP_Z, 0.0, 0.0, 0.0),
    ("calibration_target_back_frame", BACK_X, 0.0, 0.0, 0.0, 0.0, 0.0),
]


def euler_to_quaternion(roll, pitch, yaw):
    """将 Euler 角转为 quaternion (xyzw)."""
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
    return (qx, qy, qz, qw)


def make_static_transform(parent, child, x, y, z, roll, pitch, yaw):
    """构建 TransformStamped 消息."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def main():
    rospy.init_node("publish_calibration_target_tf", anonymous=True,
                    log_level=rospy.INFO)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    # object_frame → calibration_target_frame (identity)
    transforms.append(make_static_transform(
        "object_frame", "calibration_target_frame",
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    # calibration_target_frame → 5 个面 frame
    for child, x, y, z, r, p, yw in FACE_DEFS:
        transforms.append(make_static_transform(
            "calibration_target_frame", child, x, y, z, r, p, yw))

    broadcaster.sendTransform(transforms)
    rospy.loginfo("Published %d static transforms for calibration target",
                  len(transforms))
    rospy.loginfo("object_frame → calibration_target_frame → [front, left, right, top, back]")

    # 保持节点存活 (不订阅任何话题, 只 spin 等待退出)
    rospy.spin()


if __name__ == "__main__":
    main()

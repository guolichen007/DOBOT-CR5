#!/usr/bin/env python3
"""
V2 Scene TF Broadcaster: publish static transforms for gantry, object, cameras.
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def make_tf(parent, child, x, y, z, r, p, yaw):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(r, p, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def main():
    rospy.init_node("scene_tf_v2")
    br = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(1)

    # Camera look-at poses (pre-computed, angle error 0.0°)
    # cam_front_left:  rpy=(-PI, 0.4559, 0.9908)
    # cam_front_right: rpy=(-PI, 0.4559, -0.9908)
    # cam_rear:        rpy=(-PI, 0.4049, -PI)
    cameras = [
        ("cam_front_left",  0.34, -0.58, 1.22, -3.14159, 0.45589,  0.99079),
        ("cam_front_right", 0.34,  0.58, 1.22, -3.14159, 0.45589, -0.99079),
        ("cam_rear",        1.28,  0.00, 1.12, -3.14159, 0.40489, -3.14159),
    ]

    tfs = []

    # Gantry TF
    tfs.append(make_tf("world", "gantry_v2_base", 0.78, 0, 0, 0, 0, 0))

    # Object TF
    tfs.append(make_tf("world", "object_frame", 0.72, 0, 0.88, 0, 0, 0))

    # Camera TFs (link frames and optical frames)
    for name, x, y, z, r, p, yaw in cameras:
        tfs.append(make_tf("world", name + "_link", x, y, z, r, p, yaw))
        tfs.append(make_tf(name + "_link", name + "_color_optical_frame",
                           0, 0, 0, -1.5708, 0, -1.5708))
        tfs.append(make_tf(name + "_link", name + "_depth_optical_frame",
                           0, 0, 0, -1.5708, 0, -1.5708))

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for t in tfs:
            t.header.stamp = now
        br.sendTransform(tfs)
        rate.sleep()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Preflight check for the reviewed CR5 A4 demo."""

from __future__ import print_function

import json
import os
import sys

import rosnode
import rosservice
import rospy
import tf
from dobot_bringup.msg import RobotStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def report(label, ok, detail=""):
    if ok:
        rospy.loginfo("[OK]   %s%s", label, (" — " + detail) if detail else "")
    else:
        rospy.logerr("[FAIL] %s%s", label, (" — " + detail) if detail else "")
    return ok


def main():
    rospy.init_node("a4_spray_quick_check", anonymous=False)
    eef_link = rospy.get_param("~eef_link", "Tool_end")
    ok = True

    nodes = rosnode.get_node_names()
    ok &= report("DOBOT driver node", any("cr5_robot" in n for n in nodes), str(nodes))
    ok &= report("MoveIt move_group", "/move_group" in nodes)
    ok &= report("robot_state_publisher", "/robot_state_publisher" in nodes)

    try:
        joint = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
        names_ok = joint.name[:6] == [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]
        positions_ok = len(joint.position) >= 6
        ok &= report("Fresh /joint_states", True)
        ok &= report("Joint names/order", names_ok, str(joint.name))
        ok &= report("Six joint positions", positions_ok, "count=%d" % len(joint.position))
    except rospy.ROSException:
        ok &= report("Fresh /joint_states", False, "no message within 3 s")

    try:
        status = rospy.wait_for_message(
            "/dobot_bringup/msg/RobotStatus", RobotStatus, timeout=3.0
        )
        ok &= report("DOBOT TCP connection", bool(status.is_connected))
        report("Robot enabled", bool(status.is_enable), "required only before execution")
    except rospy.ROSException:
        ok &= report("RobotStatus feedback", False, "no message within 3 s")

    try:
        feed = rospy.wait_for_message(
            "/dobot_bringup/msg/FeedInfo", String, timeout=3.0
        )
        data = json.loads(feed.data)
        ok &= report("Realtime ErrorStatus=0", int(data.get("ErrorStatus", 1)) == 0, str(data))
    except (rospy.ROSException, ValueError) as exc:
        ok &= report("Realtime FeedInfo", False, str(exc))

    services = rosservice.get_service_list()
    for name in [
        "/dobot_bringup/srv/EnableRobot",
        "/dobot_bringup/srv/DisableRobot",
        "/dobot_bringup/srv/ClearError",
        "/dobot_bringup/srv/SpeedFactor",
        "/dobot_bringup/srv/EmergencyStop",
    ]:
        ok &= report("Service " + name, name in services)

    topics = [name for name, _ in rospy.get_published_topics()]
    action_goal = "/cr5_robot/joint_controller/follow_joint_trajectory/goal"
    ok &= report("FollowJointTrajectory action", action_goal in topics, action_goal)

    listener = tf.TransformListener()
    try:
        listener.waitForTransform(
            "base_link", eef_link, rospy.Time(0), rospy.Duration(3.0)
        )
        trans, rot = listener.lookupTransform("base_link", eef_link, rospy.Time(0))
        ok &= report(
            "TF base_link -> " + eef_link,
            True,
            "xyz=(%.3f, %.3f, %.3f)" % tuple(trans),
        )
    except Exception as exc:
        ok &= report("TF base_link -> " + eef_link, False, str(exc))

    dobot_type = os.environ.get("DOBOT_TYPE", "")
    ok &= report("DOBOT_TYPE=cr5", dobot_type == "cr5", "got '%s'" % dobot_type)

    if ok:
        rospy.loginfo("ALL REQUIRED PREFLIGHT CHECKS PASSED.")
        return 0
    rospy.logerr("PREFLIGHT FAILED. Do not execute the A4 trajectory.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)

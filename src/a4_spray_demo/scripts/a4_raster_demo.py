#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""DOBOT CR5 A4 raster coverage demo for ROS 1 Noetic + MoveIt 1.

The current end-effector pose is treated as the first inner corner of the paper.
The raster is generated in the selected end-effector's LOCAL XY plane:
  local +X: A4 long-edge direction
  local +Y: A4 short-edge direction
  local +Z: paper normal / standoff direction

Plan-only is the default. Real execution requires both execute=true and the
confirmation token CR5_A4_EXECUTE.
"""

from __future__ import print_function

import json
import math
import sys
from copy import deepcopy

import moveit_commander
import rospy
import tf.transformations as tft
from dobot_bringup.msg import RobotStatus
from geometry_msgs.msg import Point, Pose
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def get_param(name, default):
    return rospy.get_param("~" + name, default)


def local_offset_pose(origin, dx, dy, dz=0.0):
    """Translate a pose along its own local axes while retaining orientation."""
    q = [
        origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w,
    ]
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise ValueError("The start-pose quaternion is invalid.")
    q = [v / norm for v in q]
    rot = tft.quaternion_matrix(q)

    pose = deepcopy(origin)
    pose.position.x += rot[0, 0] * dx + rot[0, 1] * dy + rot[0, 2] * dz
    pose.position.y += rot[1, 0] * dx + rot[1, 1] * dy + rot[1, 2] * dz
    pose.position.z += rot[2, 0] * dx + rot[2, 1] * dy + rot[2, 2] * dz
    return pose


def build_raster(start_pose, length_m, width_m, num_lines):
    """Build a continuous serpentine raster including short row connectors."""
    if length_m <= 0.0 or width_m < 0.0:
        raise ValueError("Effective paper dimensions must be positive.")
    if num_lines < 1:
        raise ValueError("num_lines must be at least 1.")
    if num_lines == 1:
        actual_spacing = 0.0
    else:
        actual_spacing = width_m / float(num_lines - 1)

    waypoints = [deepcopy(start_pose)]
    for row in range(num_lines):
        y = row * actual_spacing
        row_start_x = 0.0 if row % 2 == 0 else length_m
        row_end_x = length_m if row % 2 == 0 else 0.0

        if row > 0:
            # Move along the short edge at the same long-edge endpoint.
            waypoints.append(local_offset_pose(start_pose, row_start_x, y, 0.0))

        # Long simulated spray stroke.
        waypoints.append(local_offset_pose(start_pose, row_end_x, y, 0.0))

    return waypoints, actual_spacing


def publish_path_marker(pub, frame_id, waypoints):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "a4_raster"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.004
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)
    for pose in waypoints:
        point = Point()
        point.x = pose.position.x
        point.y = pose.position.y
        point.z = pose.position.z
        marker.points.append(point)
    pub.publish(marker)


def compute_cartesian_path_compat(arm, waypoints, eef_step, jump_threshold, avoid_collisions):
    """Support the common MoveIt 1 Python signatures used by this workspace."""
    try:
        return arm.compute_cartesian_path(
            waypoints,
            eef_step,
            jump_threshold,
            avoid_collisions,
        )
    except TypeError:
        # Compatibility fallback for the existing local examples.
        return arm.compute_cartesian_path(
            waypoints=waypoints,
            eef_step=eef_step,
            avoid_collisions=avoid_collisions,
            path_constraints=None,
        )


def read_robot_health(timeout_s=3.0):
    """Return (ok, detail) from the existing DOBOT V3 feedback topics."""
    try:
        status = rospy.wait_for_message(
            "/dobot_bringup/msg/RobotStatus", RobotStatus, timeout=timeout_s
        )
    except rospy.ROSException:
        return False, "RobotStatus has no recent message."

    if not status.is_connected:
        return False, "DOBOT driver does not report dashboard/motion connection."
    if not status.is_enable:
        return False, "Robot is not enabled."

    try:
        feed = rospy.wait_for_message(
            "/dobot_bringup/msg/FeedInfo", String, timeout=timeout_s
        )
        data = json.loads(feed.data)
    except (rospy.ROSException, ValueError) as exc:
        return False, "FeedInfo is unavailable or invalid: %s" % exc

    if int(data.get("EnableStatus", 0)) == 0:
        return False, "Realtime feedback reports EnableStatus=0."
    if int(data.get("ErrorStatus", 0)) != 0:
        return False, "Realtime feedback reports ErrorStatus=%s." % data.get("ErrorStatus")

    return True, "connected, enabled, no realtime error"


def pose_position_error_mm(actual, expected):
    dx = actual.position.x - expected.position.x
    dy = actual.position.y - expected.position.y
    dz = actual.position.z - expected.position.z
    return 1000.0 * math.sqrt(dx * dx + dy * dy + dz * dz)


def pose_orientation_error_deg(actual, expected):
    qa = [
        actual.orientation.x,
        actual.orientation.y,
        actual.orientation.z,
        actual.orientation.w,
    ]
    qe = [
        expected.orientation.x,
        expected.orientation.y,
        expected.orientation.z,
        expected.orientation.w,
    ]
    na = math.sqrt(sum(v * v for v in qa))
    ne = math.sqrt(sum(v * v for v in qe))
    if na < 1e-9 or ne < 1e-9:
        return float("nan")
    qa = [v / na for v in qa]
    qe = [v / ne for v in qe]
    dot = abs(sum(a * e for a, e in zip(qa, qe)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def main():
    rospy.init_node("a4_raster_demo", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    group_name = str(get_param("group_name", "cr5_arm"))
    reference_frame = str(get_param("reference_frame", "base_link"))
    eef_link = str(get_param("eef_link", "Tool_end"))

    paper_long_mm = float(get_param("paper_long_mm", 297.0))
    paper_short_mm = float(get_param("paper_short_mm", 210.0))
    margin_long_mm = float(get_param("margin_long_mm", 20.0))
    margin_short_mm = float(get_param("margin_short_mm", 20.0))
    num_lines = int(get_param("num_lines", 7))

    eef_step = float(get_param("eef_step_m", 0.005))
    jump_threshold = float(get_param("jump_threshold", 0.0))
    avoid_collisions = bool(get_param("avoid_collisions", True))
    fraction_threshold = float(get_param("fraction_threshold", 0.999))
    planning_attempts = int(get_param("planning_attempts", 3))

    velocity_scaling = float(get_param("velocity_scaling", 0.05))
    acceleration_scaling = float(get_param("acceleration_scaling", 0.05))
    max_allowed_execution_scaling = float(
        get_param("max_allowed_execution_scaling", 0.05)
    )
    planning_time = float(get_param("planning_time_s", 30.0))
    goal_position_tolerance = float(get_param("goal_position_tolerance_m", 0.005))
    goal_orientation_tolerance = float(
        get_param("goal_orientation_tolerance_rad", 0.05)
    )

    execute = bool(get_param("execute", False))
    confirmation = str(get_param("confirmation", ""))
    required_confirmation = str(
        get_param("required_confirmation", "CR5_A4_EXECUTE")
    )
    countdown_seconds = int(get_param("countdown_seconds", 5))
    require_robot_health = bool(get_param("require_robot_health", True))
    preview_hold_seconds = float(get_param("preview_hold_seconds", 15.0))

    display_topic = str(
        get_param("display_trajectory_topic", "/move_group/display_planned_path")
    )
    marker_topic = str(get_param("path_marker_topic", "/a4_spray_demo/path_marker"))

    effective_long_mm = paper_long_mm - 2.0 * margin_long_mm
    effective_short_mm = paper_short_mm - 2.0 * margin_short_mm
    if effective_long_mm <= 0.0 or effective_short_mm < 0.0:
        rospy.logfatal("Margins leave a non-positive effective paper area.")
        return 2
    if eef_step <= 0.0:
        rospy.logfatal("eef_step_m must be positive.")
        return 2
    if num_lines < 1:
        rospy.logfatal("num_lines must be at least 1.")
        return 2
    if not (0.0 < fraction_threshold <= 1.0):
        rospy.logfatal("fraction_threshold must be in (0, 1].")
        return 2
    if velocity_scaling <= 0.0 or acceleration_scaling <= 0.0:
        rospy.logfatal("Velocity/acceleration scaling must be positive.")
        return 2
    if execute and (
        velocity_scaling > max_allowed_execution_scaling
        or acceleration_scaling > max_allowed_execution_scaling
    ):
        rospy.logfatal(
            "Execution blocked: scaling exceeds configured maximum %.3f.",
            max_allowed_execution_scaling,
        )
        return 3

    robot = RobotCommander()
    arm = MoveGroupCommander(group_name)

    if eef_link:
        if eef_link not in robot.get_link_names():
            rospy.logfatal("End-effector link '%s' is not in robot_description.", eef_link)
            return 4
        arm.set_end_effector_link(eef_link)

    arm.set_pose_reference_frame(reference_frame)
    arm.allow_replanning(True)
    arm.set_goal_position_tolerance(goal_position_tolerance)
    arm.set_goal_orientation_tolerance(goal_orientation_tolerance)
    arm.set_max_velocity_scaling_factor(velocity_scaling)
    arm.set_max_acceleration_scaling_factor(acceleration_scaling)
    arm.set_planning_time(planning_time)
    arm.set_num_planning_attempts(max(1, planning_attempts))
    arm.set_start_state_to_current_state()

    rospy.loginfo("=" * 68)
    rospy.loginfo("CR5 A4 raster demo: %s", "EXECUTE" if execute else "PLAN ONLY")
    rospy.loginfo("Planning group: %s", group_name)
    rospy.loginfo("Planning frame: %s", arm.get_planning_frame())
    rospy.loginfo("Selected end-effector: %s", arm.get_end_effector_link())
    rospy.loginfo(
        "Effective area: %.1f x %.1f mm, lines=%d",
        effective_long_mm,
        effective_short_mm,
        num_lines,
    )
    rospy.logwarn(
        "Path is generated in the selected end-effector LOCAL XY plane, not base_link XY."
    )
    rospy.logwarn(
        "MoveIt scaling does not guarantee real speed because the existing DOBOT bridge "
        "sends ServoJ positions at a fixed period."
    )
    rospy.loginfo("=" * 68)

    start_pose = arm.get_current_pose(eef_link).pose
    waypoints, actual_spacing = build_raster(
        start_pose,
        effective_long_mm / 1000.0,
        effective_short_mm / 1000.0,
        num_lines,
    )
    rospy.loginfo(
        "Generated %d waypoints; actual row spacing %.2f mm.",
        len(waypoints),
        actual_spacing * 1000.0,
    )

    marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1, latch=True)
    display_pub = rospy.Publisher(
        display_topic, DisplayTrajectory, queue_size=1, latch=True
    )
    rospy.sleep(0.5)
    publish_path_marker(marker_pub, reference_frame, waypoints)

    best_plan = None
    best_fraction = -1.0
    for attempt in range(max(1, planning_attempts)):
        arm.set_start_state_to_current_state()
        plan, fraction = compute_cartesian_path_compat(
            arm, waypoints, eef_step, jump_threshold, avoid_collisions
        )
        rospy.loginfo(
            "Cartesian attempt %d/%d: %.3f%%",
            attempt + 1,
            max(1, planning_attempts),
            fraction * 100.0,
        )
        if fraction > best_fraction:
            best_plan = plan
            best_fraction = fraction
        if fraction >= fraction_threshold:
            break

    plan = best_plan
    fraction = best_fraction
    point_count = (
        len(plan.joint_trajectory.points)
        if plan is not None and hasattr(plan, "joint_trajectory")
        else 0
    )
    rospy.loginfo("Best path fraction: %.3f%%", fraction * 100.0)
    rospy.loginfo("Joint trajectory points: %d", point_count)

    if plan is None or point_count == 0:
        rospy.logerr("No executable JointTrajectory was produced.")
        return 5

    display = DisplayTrajectory()
    display.trajectory_start = robot.get_current_state()
    display.trajectory.append(plan)
    display_pub.publish(display)

    if fraction < fraction_threshold:
        rospy.logerr(
            "Path fraction %.5f is below required %.5f. Execution is blocked.",
            fraction,
            fraction_threshold,
        )
        rospy.sleep(preview_hold_seconds)
        return 6

    if not execute:
        rospy.loginfo("Plan-only mode: no physical motion command was sent.")
        rospy.loginfo("RViz marker topic: %s", marker_topic)
        rospy.loginfo("RViz trajectory topic: %s", display_topic)
        rospy.sleep(preview_hold_seconds)
        return 0

    if confirmation != required_confirmation:
        rospy.logerr("Execution blocked: confirmation token is missing or incorrect.")
        return 7

    if require_robot_health:
        health_ok, detail = read_robot_health()
        if not health_ok:
            rospy.logerr("Execution blocked by robot-health check: %s", detail)
            return 8
        rospy.loginfo("Robot-health check passed: %s", detail)

    for remaining in range(max(0, countdown_seconds), 0, -1):
        rospy.logwarn(
            "REAL CR5 MOTION STARTS IN %d s — keep the physical E-stop ready.",
            remaining,
        )
        rospy.sleep(1.0)

    rospy.logwarn("Executing A4 raster trajectory.")
    success = arm.execute(plan, wait=True)
    arm.stop()

    final_pose = arm.get_current_pose(eef_link).pose
    expected_pose = waypoints[-1]
    position_error = pose_position_error_mm(final_pose, expected_pose)
    orientation_error = pose_orientation_error_deg(final_pose, expected_pose)

    rospy.loginfo("MoveIt execute result: %s", success)
    rospy.loginfo("Final position error: %.2f mm", position_error)
    rospy.loginfo("Final orientation error: %.2f deg", orientation_error)

    if require_robot_health:
        health_ok, detail = read_robot_health()
        if not health_ok:
            rospy.logerr("Post-execution robot-health check failed: %s", detail)
            return 9

    if not success:
        return 10

    rospy.logwarn(
        "Geometric coverage is complete. This is NOT a coating-speed or film-uniformity validation."
    )
    return 0


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except rospy.ROSInterruptException:
        exit_code = 130
    except Exception as exc:
        rospy.logfatal("Unhandled demo error: %s", exc)
        exit_code = 99
    finally:
        moveit_commander.roscpp_shutdown()
    sys.exit(exit_code)

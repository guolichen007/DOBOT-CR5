#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MoveIt plan/execute node for a dry-run book-cover spray path.

Default behavior is PLAN ONLY. Physical execution requires all of:
  1) ~allow_execution:=true
  2) a valid locked book pose
  3) a successful, recent plan
  4) rosparam /book_demo/confirm_execute set to the exact token
  5) an explicit call to the execute service

This node does not operate any spray valve or material source.
"""

import copy
import math
import threading

import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray


class BookSprayPlanner:
    def __init__(self):
        self.lock = threading.RLock()
        moveit_commander.roscpp_initialize([])

        self.group_name = rospy.get_param("~planning_group", "cr5_arm")
        self.eef_link = rospy.get_param("~eef_link", "Tool_end")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.locked_pose_topic = rospy.get_param(
            "~locked_pose_topic", "/book_demo/estimator/locked_pose"
        )
        self.locked_size_topic = rospy.get_param(
            "~locked_size_topic", "/book_demo/estimator/locked_size"
        )
        self.target_locked_topic = rospy.get_param(
            "~target_locked_topic", "/book_demo/estimator/target_locked"
        )

        # Path dimensions and behavior.
        self.path_mode = rospy.get_param("~path_mode", "single_stroke")
        self.use_detected_size = bool(rospy.get_param("~use_detected_size", False))
        self.measured_length_m = float(rospy.get_param("~book_length_m", 0.260))
        self.measured_width_m = float(rospy.get_param("~book_width_m", 0.190))
        self.margin_long_m = float(rospy.get_param("~margin_long_m", 0.025))
        self.margin_short_m = float(rospy.get_param("~margin_short_m", 0.025))
        self.pass_spacing_m = float(rospy.get_param("~pass_spacing_m", 0.035))
        self.serpentine = bool(rospy.get_param("~serpentine", True))
        self.standoff_m = float(rospy.get_param("~standoff_m", 0.100))
        self.approach_clearance_m = float(
            rospy.get_param("~approach_clearance_m", 0.080)
        )
        self.retreat_clearance_m = float(
            rospy.get_param("~retreat_clearance_m", 0.080)
        )
        self.cartesian_step_m = float(rospy.get_param("~cartesian_step_m", 0.005))
        self.jump_threshold = float(rospy.get_param("~jump_threshold", 0.0))
        self.min_fraction = float(rospy.get_param("~min_fraction", 0.995))

        # Orientation. align_to_book assumes spray_tcp +Z points out of the nozzle
        # toward the target. First tests should use keep_current until TCP is verified.
        self.orientation_mode = rospy.get_param("~orientation_mode", "keep_current")
        self.roll_offset_deg = float(rospy.get_param("~roll_offset_deg", 0.0))

        # MoveIt and execution safety.
        self.planning_time_s = float(rospy.get_param("~planning_time_s", 8.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 8))
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 0.03))
        self.acceleration_scale = float(rospy.get_param("~acceleration_scale", 0.03))
        self.allow_execution = bool(rospy.get_param("~allow_execution", False))
        self.execution_token = rospy.get_param(
            "~execution_token", "CR5_BOOK_DRY_RUN_EXECUTE"
        )
        self.confirm_param = rospy.get_param(
            "~execute_confirmation_param", "/book_demo/confirm_execute"
        )
        self.max_plan_age_s = float(rospy.get_param("~max_plan_age_s", 120.0))
        self.max_target_shift_m = float(rospy.get_param("~max_target_shift_m", 0.003))
        self.max_target_angle_deg = float(
            rospy.get_param("~max_target_angle_deg", 0.8)
        )

        # Optional collision scene based on the detected book plane.
        self.add_collision_scene = bool(rospy.get_param("~add_collision_scene", False))
        self.book_thickness_m = float(rospy.get_param("~book_thickness_m", 0.055))
        self.table_length_m = float(rospy.get_param("~table_length_m", 1.50))
        self.table_width_m = float(rospy.get_param("~table_width_m", 0.55))
        self.table_thickness_m = float(rospy.get_param("~table_thickness_m", 0.05))

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_pose_reference_frame(self.base_frame)
        self.group.set_end_effector_link(self.eef_link)
        self.group.set_planning_time(self.planning_time_s)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.allow_replanning(True)
        self.group.set_max_velocity_scaling_factor(self.velocity_scale)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scale)

        self.display_pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher("~path_markers", MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber(self.locked_pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber(self.locked_size_topic, Vector3Stamped, self.size_callback, queue_size=1)
        rospy.Subscriber(self.target_locked_topic, Bool, self.lock_callback, queue_size=1)

        self.plan_srv = rospy.Service("~plan_path", Trigger, self.handle_plan)
        self.execute_srv = rospy.Service("~execute_path", Trigger, self.handle_execute)
        self.clear_srv = rospy.Service("~clear_plan", Trigger, self.handle_clear_plan)

        self.locked_pose = None
        self.locked_size = None
        self.target_locked = False
        self.approach_plan = None
        self.cartesian_plan = None
        self.plan_time = None
        self.planned_target_pose = None
        self.planned_waypoints = []
        self.plan_fraction = 0.0

        rospy.loginfo(
            "book_spray_planner ready: group=%s eef=%s mode=%s orientation=%s execution=%s",
            self.group_name,
            self.eef_link,
            self.path_mode,
            self.orientation_mode,
            self.allow_execution,
        )
        if self.orientation_mode == "align_to_book":
            rospy.logwarn(
                "align_to_book assumes %s +Z is the physical spray direction. Verify the TCP in RViz before execution.",
                self.eef_link,
            )

    def pose_callback(self, msg):
        with self.lock:
            self.locked_pose = copy.deepcopy(msg)

    def size_callback(self, msg):
        with self.lock:
            self.locked_size = copy.deepcopy(msg)

    def lock_callback(self, msg):
        with self.lock:
            self.target_locked = bool(msg.data)

    @staticmethod
    def pose_rotation(pose_stamped):
        q = pose_stamped.pose.orientation
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    @staticmethod
    def pose_position(pose_stamped):
        p = pose_stamped.pose.position
        return np.array([p.x, p.y, p.z], dtype=np.float64)

    @staticmethod
    def quaternion_angle_deg(pose_a, pose_b):
        qa = pose_a.pose.orientation
        qb = pose_b.pose.orientation
        a = np.array([qa.x, qa.y, qa.z, qa.w], dtype=np.float64)
        b = np.array([qb.x, qb.y, qb.z, qb.w], dtype=np.float64)
        a /= max(np.linalg.norm(a), 1e-12)
        b /= max(np.linalg.norm(b), 1e-12)
        dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
        return math.degrees(2.0 * math.acos(dot))

    def selected_size(self):
        if self.use_detected_size:
            if self.locked_size is None:
                raise RuntimeError("No locked size available")
            return float(self.locked_size.vector.x), float(self.locked_size.vector.y)
        return self.measured_length_m, self.measured_width_m

    def desired_orientation(self, book_pose):
        if self.orientation_mode == "keep_current":
            return copy.deepcopy(self.group.get_current_pose(self.eef_link).pose.orientation)
        if self.orientation_mode != "align_to_book":
            raise RuntimeError("Unknown orientation_mode: %s" % self.orientation_mode)

        r_book = self.pose_rotation(book_pose)
        roll = math.radians(self.roll_offset_deg)
        r_roll = np.array(
            [
                [math.cos(roll), -math.sin(roll), 0.0],
                [math.sin(roll), math.cos(roll), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        # Book axes after optional in-plane roll.
        r_book_rolled = r_book @ r_roll
        # Desired spray_tcp axes: +X follows book +X, +Z points into the cover.
        r_flip = np.diag([1.0, -1.0, -1.0])
        r_eef = r_book_rolled @ r_flip
        matrix = np.eye(4)
        matrix[:3, :3] = r_eef
        q = quaternion_from_matrix(matrix)
        orientation = copy.deepcopy(book_pose.pose.orientation)
        orientation.x, orientation.y, orientation.z, orientation.w = map(float, q)
        return orientation

    def local_to_pose(self, center, rotation, local_xyz, orientation):
        position = center + rotation @ np.asarray(local_xyz, dtype=np.float64)
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation = copy.deepcopy(orientation)
        return pose

    def generate_work_waypoints(self, book_pose, length, width):
        usable_length = length - 2.0 * self.margin_long_m
        usable_width = width - 2.0 * self.margin_short_m
        if usable_length <= 0.03 or usable_width <= 0.02:
            raise RuntimeError(
                "Margins leave no usable area: length %.3f width %.3f" % (usable_length, usable_width)
            )

        center = self.pose_position(book_pose)
        rotation = self.pose_rotation(book_pose)
        orientation = self.desired_orientation(book_pose)
        top_x = -0.5 * usable_length
        bottom_x = 0.5 * usable_length

        work_local = []
        if self.path_mode == "single_stroke":
            work_local = [
                (top_x, 0.0, self.standoff_m),
                (bottom_x, 0.0, self.standoff_m),
            ]
        elif self.path_mode == "raster":
            passes = max(2, int(math.ceil(usable_width / self.pass_spacing_m)) + 1)
            y_values = np.linspace(-0.5 * usable_width, 0.5 * usable_width, passes)
            for index, y_value in enumerate(y_values):
                if self.serpentine and index % 2 == 1:
                    xs = (bottom_x, top_x)
                else:
                    xs = (top_x, bottom_x)
                work_local.append((xs[0], float(y_value), self.standoff_m))
                work_local.append((xs[1], float(y_value), self.standoff_m))
        else:
            raise RuntimeError("Unknown path_mode: %s" % self.path_mode)

        first = work_local[0]
        last = work_local[-1]
        approach_local = (
            first[0],
            first[1],
            self.standoff_m + self.approach_clearance_m,
        )
        retreat_local = (
            last[0],
            last[1],
            self.standoff_m + self.retreat_clearance_m,
        )
        approach_pose = self.local_to_pose(center, rotation, approach_local, orientation)
        work_poses = [self.local_to_pose(center, rotation, p, orientation) for p in work_local]
        # Cartesian section descends, traverses the cover, then retreats.
        cartesian_poses = work_poses + [self.local_to_pose(center, rotation, retreat_local, orientation)]
        return approach_pose, cartesian_poses, usable_length, usable_width

    @staticmethod
    def unpack_plan_result(result):
        if isinstance(result, tuple):
            if len(result) >= 2:
                return bool(result[0]), result[1]
            return False, None
        trajectory = result
        success = bool(
            trajectory is not None
            and hasattr(trajectory, "joint_trajectory")
            and len(trajectory.joint_trajectory.points) > 0
        )
        return success, trajectory

    def state_after_plan(self, start_state, trajectory):
        if trajectory is None or not trajectory.joint_trajectory.points:
            raise RuntimeError("Approach trajectory has no points")
        state = copy.deepcopy(start_state)
        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)
        final = trajectory.joint_trajectory.points[-1]
        for joint_name, value in zip(trajectory.joint_trajectory.joint_names, final.positions):
            if joint_name not in names:
                names.append(joint_name)
                positions.append(float(value))
            else:
                positions[names.index(joint_name)] = float(value)
        state.joint_state.name = names
        state.joint_state.position = positions
        state.joint_state.header.stamp = rospy.Time.now()
        return state

    def add_scene(self, book_pose, length, width):
        if not self.add_collision_scene:
            return
        r_book = self.pose_rotation(book_pose)
        top = self.pose_position(book_pose)

        book_center = top - r_book[:, 2] * (0.5 * self.book_thickness_m)
        book_box = copy.deepcopy(book_pose)
        book_box.header.stamp = rospy.Time.now()
        book_box.pose.position.x = float(book_center[0])
        book_box.pose.position.y = float(book_center[1])
        book_box.pose.position.z = float(book_center[2])
        self.scene.add_box(
            "detected_book",
            book_box,
            size=(float(length), float(width), self.book_thickness_m),
        )

        table_center = top - r_book[:, 2] * (
            self.book_thickness_m + 0.5 * self.table_thickness_m
        )
        table_box = copy.deepcopy(book_pose)
        table_box.header.stamp = rospy.Time.now()
        table_box.pose.position.x = float(table_center[0])
        table_box.pose.position.y = float(table_center[1])
        table_box.pose.position.z = float(table_center[2])
        self.scene.add_box(
            "book_conveyor_surface",
            table_box,
            size=(self.table_length_m, self.table_width_m, self.table_thickness_m),
        )
        rospy.sleep(0.8)

    def publish_markers(self, book_pose, waypoints, usable_length, usable_width):
        markers = MarkerArray()
        stamp = rospy.Time.now()

        cover = Marker()
        cover.header.frame_id = self.base_frame
        cover.header.stamp = stamp
        cover.ns = "book_spray_path"
        cover.id = 0
        cover.type = Marker.CUBE
        cover.action = Marker.ADD
        cover.pose = copy.deepcopy(book_pose.pose)
        cover.scale.x = usable_length
        cover.scale.y = usable_width
        cover.scale.z = 0.004
        cover.color.r, cover.color.g, cover.color.b, cover.color.a = 0.1, 0.7, 0.9, 0.25
        markers.markers.append(cover)

        line = Marker()
        line.header = cover.header
        line.ns = cover.ns
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.008
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.3, 0.1, 1.0
        for pose in waypoints:
            line.points.append(copy.deepcopy(pose.position))
        markers.markers.append(line)

        points = Marker()
        points.header = cover.header
        points.ns = cover.ns
        points.id = 2
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.scale.x = points.scale.y = points.scale.z = 0.015
        points.color.r, points.color.g, points.color.b, points.color.a = 1.0, 0.9, 0.1, 1.0
        for pose in waypoints:
            points.points.append(copy.deepcopy(pose.position))
        markers.markers.append(points)
        self.marker_pub.publish(markers)

    def publish_display_trajectory(self, start_state, approach_plan, cartesian_plan):
        display = DisplayTrajectory()
        display.model_id = ""
        display.trajectory_start = start_state
        display.trajectory = [approach_plan, cartesian_plan]
        self.display_pub.publish(display)

    def clear_cached_plan(self):
        self.approach_plan = None
        self.cartesian_plan = None
        self.plan_time = None
        self.planned_target_pose = None
        self.planned_waypoints = []
        self.plan_fraction = 0.0

    def handle_clear_plan(self, _request):
        with self.lock:
            self.clear_cached_plan()
        self.group.stop()
        self.group.clear_pose_targets()
        return TriggerResponse(success=True, message="Cached plan cleared")

    def handle_plan(self, _request):
        with self.lock:
            locked = self.target_locked
            book_pose = copy.deepcopy(self.locked_pose)
            book_size = copy.deepcopy(self.locked_size)
            self.clear_cached_plan()

        if not locked or book_pose is None:
            return TriggerResponse(success=False, message="No locked book target")
        if self.use_detected_size and book_size is None:
            return TriggerResponse(success=False, message="No locked book size")

        try:
            length, width = self.selected_size()
            approach_pose, cartesian_poses, usable_length, usable_width = (
                self.generate_work_waypoints(book_pose, length, width)
            )
            self.add_scene(book_pose, length, width)

            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(approach_pose, self.eef_link)
            approach_result = self.group.plan()
            self.group.clear_pose_targets()
            approach_ok, approach_plan = self.unpack_plan_result(approach_result)
            if not approach_ok:
                return TriggerResponse(success=False, message="MoveIt could not plan approach pose")

            start_state = self.robot.get_current_state()
            cart_start_state = self.state_after_plan(start_state, approach_plan)
            self.group.set_start_state(cart_start_state)
            cartesian_plan, fraction = self.group.compute_cartesian_path(
                cartesian_poses,
                self.cartesian_step_m,
                self.jump_threshold,
                True,
            )
            self.group.set_start_state_to_current_state()

            if fraction < self.min_fraction:
                return TriggerResponse(
                    success=False,
                    message="Cartesian fraction %.3f below required %.3f"
                    % (fraction, self.min_fraction),
                )
            if not cartesian_plan.joint_trajectory.points:
                return TriggerResponse(success=False, message="Cartesian trajectory is empty")

            try:
                cartesian_plan = self.group.retime_trajectory(
                    cart_start_state,
                    cartesian_plan,
                    self.velocity_scale,
                    self.acceleration_scale,
                    "iterative_time_parameterization",
                )
            except Exception as exc:
                rospy.logwarn("Trajectory retiming unavailable; keeping original timing: %s", exc)

            with self.lock:
                self.approach_plan = approach_plan
                self.cartesian_plan = cartesian_plan
                self.plan_time = rospy.Time.now()
                self.planned_target_pose = copy.deepcopy(book_pose)
                self.planned_waypoints = copy.deepcopy(cartesian_poses)
                self.plan_fraction = float(fraction)

            self.publish_display_trajectory(start_state, approach_plan, cartesian_plan)
            self.publish_markers(book_pose, cartesian_poses, usable_length, usable_width)
            rospy.loginfo(
                "PLAN ONLY complete: mode=%s, work waypoints=%d, fraction=%.3f, area=%.0fx%.0f mm",
                self.path_mode,
                len(cartesian_poses),
                fraction,
                usable_length * 1000.0,
                usable_width * 1000.0,
            )
            return TriggerResponse(
                success=True,
                message="PLAN ONLY: fraction %.3f, %d Cartesian waypoints. Review RViz before execution."
                % (fraction, len(cartesian_poses)),
            )
        except Exception as exc:
            self.group.set_start_state_to_current_state()
            self.group.clear_pose_targets()
            rospy.logerr("Book path planning failed: %s", exc)
            return TriggerResponse(success=False, message="Planning failed: %s" % exc)

    def target_still_matches_plan(self):
        if self.locked_pose is None or self.planned_target_pose is None:
            return False, "Missing current/planned target pose"
        shift = float(
            np.linalg.norm(
                self.pose_position(self.locked_pose)
                - self.pose_position(self.planned_target_pose)
            )
        )
        angle = self.quaternion_angle_deg(self.locked_pose, self.planned_target_pose)
        if shift > self.max_target_shift_m:
            return False, "Locked target shifted %.1f mm" % (shift * 1000.0)
        if angle > self.max_target_angle_deg:
            return False, "Locked target rotated %.2f deg" % angle
        return True, "Target unchanged"

    def handle_execute(self, _request):
        with self.lock:
            approach_plan = self.approach_plan
            cartesian_plan = self.cartesian_plan
            plan_time = self.plan_time
            target_locked = self.target_locked

        if not self.allow_execution:
            return TriggerResponse(
                success=False,
                message="Execution disabled. Relaunch with allow_execution:=true only after PLAN review.",
            )
        if not target_locked:
            return TriggerResponse(success=False, message="Book target is not locked")
        if approach_plan is None or cartesian_plan is None or plan_time is None:
            return TriggerResponse(success=False, message="No successful cached plan")
        plan_age = (rospy.Time.now() - plan_time).to_sec()
        if plan_age > self.max_plan_age_s:
            return TriggerResponse(
                success=False,
                message="Plan is stale (%.1f s > %.1f s); re-plan" % (plan_age, self.max_plan_age_s),
            )
        target_ok, target_message = self.target_still_matches_plan()
        if not target_ok:
            return TriggerResponse(success=False, message=target_message + "; re-plan")

        supplied_token = rospy.get_param(self.confirm_param, "")
        if supplied_token != self.execution_token:
            return TriggerResponse(
                success=False,
                message="Confirmation missing. Set %s to the exact token after RViz review."
                % self.confirm_param,
            )

        # One-shot confirmation: clear before any physical command is sent.
        rospy.set_param(self.confirm_param, "")
        rospy.logwarn("PHYSICAL DRY-RUN EXECUTION starts in 5 seconds. No spray/air/material may be connected.")
        for remaining in range(5, 0, -1):
            rospy.logwarn("Executing in %d...", remaining)
            rospy.sleep(1.0)
            if rospy.is_shutdown():
                return TriggerResponse(success=False, message="ROS shutdown during countdown")

        try:
            approach_ok = self.group.execute(approach_plan, wait=True)
            self.group.stop()
            if not approach_ok:
                return TriggerResponse(success=False, message="Approach execution failed")
            process_ok = self.group.execute(cartesian_plan, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            if not process_ok:
                return TriggerResponse(success=False, message="Cartesian dry-run execution failed")
            return TriggerResponse(success=True, message="Dry-run book path executed successfully")
        except Exception as exc:
            self.group.stop()
            self.group.clear_pose_targets()
            rospy.logerr("Execution failed: %s", exc)
            return TriggerResponse(success=False, message="Execution exception: %s" % exc)


def main():
    rospy.init_node("book_spray_planner")
    BookSprayPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()

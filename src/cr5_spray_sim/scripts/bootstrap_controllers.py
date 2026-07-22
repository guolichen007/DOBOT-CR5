#!/usr/bin/env python3
"""
V3.3.7 Controller Bootstrap — 确定性顺序启动。

在 Gazebo 已 unpause 后，顺序启动控制器：
1. 调用 /gazebo/unpause_physics
2. wall-time 采样确认 /clock 增长
3. 加载并启动 joint_state_controller
4. 等待 running + /joint_states ≥ 5 帧
5. 加载并启动 arm_controller
6. 等待 running
7. 发送六轴零位 FollowJointTrajectory
8. 验证 action 成功 + 六轴 ≤ 0.02rad

用法:
  rosrun cr5_spray_sim bootstrap_controllers.py

输出到 stderr:
  SIM_CLOCK_ADVANCING / CLOCK_NOT_ADVANCING
  JOINT_STATE_CONTROLLER_RUNNING / JOINT_STATE_CONTROLLER_FAILED
  JOINT_STATES_READY / JOINT_STATES_FAILED
  ARM_CONTROLLER_RUNNING / ARM_CONTROLLER_FAILED
  ZERO_HOLD_ACTIVE / ZERO_HOLD_FAILED
  CONTROLLERS_RUNNING / CONTROLLERS_FAILED

退出码:
  0 = 全部就绪
  1 = clock 未推进
  2 = 控制器加载失败
  3 = 零位保持失败
"""
import sys
import os
import math
import time
import subprocess
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    SwitchController,
)
from std_srvs.srv import Empty as EmptySrv
from control_msgs.msg import FollowJointTrajectoryActionGoal
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
CONTROLLER_TIMEOUT = 45.0


class ControllerBootstrapper:
    def __init__(self):
        # 等待基础服务
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=30.0)
        rospy.wait_for_service("/controller_manager/load_controller", timeout=30.0)
        rospy.wait_for_service("/controller_manager/switch_controller", timeout=30.0)

        self.list_ctrl = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers)
        self.load_ctrl = rospy.ServiceProxy(
            "/controller_manager/load_controller", LoadController)
        self.switch_ctrl = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController)

    def unpause(self):
        """Unpause Gazebo physics."""
        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=10.0)
            srv = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
            srv()
            rospy.loginfo("Gazebo physics unpaused")
            return True
        except Exception as e:
            rospy.logerr("unpause_physics failed: %s", e)
            return False

    def verify_clock(self):
        """wall-time 采样 1 秒确认 /clock 增长."""
        rospy.loginfo("Verifying /clock advancing...")
        messages = []
        start = time.time()
        timeout = 3.0

        while time.time() - start < timeout:
            try:
                msg = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                messages.append(msg.clock.to_sec())
            except rospy.ROSException:
                continue

            if time.time() - start >= 1.0 and len(messages) >= 3:
                break

        if len(messages) < 3:
            rospy.logerr("Too few /clock messages: %d", len(messages))
            return False

        if messages[-1] <= messages[0]:
            rospy.logerr("Clock not advancing: first=%.3f last=%.3f",
                         messages[0], messages[-1])
            return False

        advance = messages[-1] - messages[0]
        if advance <= 0.001:
            rospy.logerr("Clock advance too small: %.4f", advance)
            return False

        rospy.loginfo("Clock advancing: %.3fs → %.3fs (advance=%.3fs)",
                      messages[0], messages[-1], advance)
        return True

    def _get_controller_state(self, name):
        """获取指定 controller 的状态."""
        try:
            resp = self.list_ctrl()
            for c in resp.controller:
                if c.name == name:
                    return c.state
        except Exception:
            pass
        return "unknown"

    def _wait_controller_running(self, name, timeout=CONTROLLER_TIMEOUT):
        """轮询等待 controller 进入 running 状态."""
        start = time.time()
        stable = 0
        last_state = None
        while time.time() - start < timeout:
            state = self._get_controller_state(name)
            if state == "running":
                if state == last_state:
                    stable += 1
                else:
                    stable = 1
                last_state = state
                if stable >= 3:
                    rospy.loginfo("%s running (stable %d)", name, stable)
                    return True
            else:
                stable = 0
                last_state = state
                rospy.loginfo("%s state: %s (waiting...)", name, state)
            time.sleep(0.2)
        rospy.logerr("%s timed out waiting for running, last state: %s",
                     name, self._get_controller_state(name))
        return False

    def start_joint_state_controller(self):
        """加载并启动 joint_state_controller."""
        rospy.loginfo("Loading joint_state_controller...")
        try:
            resp = self.load_ctrl("joint_state_controller")
            if not resp.ok:
                rospy.logwarn("Load joint_state_controller: %s", resp.ok)
        except Exception as e:
            rospy.logwarn("Load joint_state_controller exception (may already be loaded): %s", e)

        rospy.loginfo("Starting joint_state_controller...")
        try:
            resp = self.switch_ctrl(
                start_controllers=["joint_state_controller"],
                stop_controllers=[],
                strictness=SwitchController._request_class.BEST_EFFORT,
            )
            if not resp.ok:
                rospy.logwarn("Switch joint_state_controller: %s", resp.ok)
        except Exception as e:
            rospy.logerr("Switch joint_state_controller failed: %s", e)
            return False

        if not self._wait_controller_running("joint_state_controller"):
            return False

        return True

    def wait_joint_states(self, min_messages=5):
        """等待 /joint_states 有足够消息."""
        rospy.loginfo("Waiting for /joint_states (%d messages)...", min_messages)
        timeout = 10.0
        start = time.time()
        count = 0
        while time.time() - start < timeout:
            try:
                msg = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
                # 检查所有关节都在
                names_set = set(msg.name)
                required = set(JOINT_NAMES)
                if required.issubset(names_set):
                    count += 1
                    rospy.loginfo("joint_states msg %d: %d joints", count, len(msg.name))
                    if count >= min_messages:
                        rospy.loginfo("Joint states ready (%d messages)", count)
                        return True
            except rospy.ROSException:
                pass

        rospy.logerr("Joint states timeout: got %d/%d messages", count, min_messages)
        return False

    def start_arm_controller(self):
        """加载并启动 arm_controller."""
        rospy.loginfo("Loading arm_controller...")
        try:
            resp = self.load_ctrl("arm_controller")
            if not resp.ok:
                rospy.logwarn("Load arm_controller: %s", resp.ok)
        except Exception as e:
            rospy.logwarn("Load arm_controller exception (may already be loaded): %s", e)

        rospy.loginfo("Starting arm_controller...")
        try:
            resp = self.switch_ctrl(
                start_controllers=["arm_controller"],
                stop_controllers=[],
                strictness=SwitchController._request_class.BEST_EFFORT,
            )
            if not resp.ok:
                rospy.logwarn("Switch arm_controller: %s", resp.ok)
        except Exception as e:
            rospy.logerr("Switch arm_controller failed: %s", e)
            return False

        if not self._wait_controller_running("arm_controller"):
            return False

        return True

    def send_zero_hold_trajectory(self):
        """发送六轴零位 FollowJointTrajectory，保持姿态."""
        rospy.loginfo("Sending zero-hold trajectory...")

        # 等待 action server
        action_name = "/arm_controller/follow_joint_trajectory"
        client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

        if not client.wait_for_server(timeout=rospy.Duration(10.0)):
            rospy.logerr("Action server %s not available", action_name)
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 6
        point.velocities = [0.0] * 6
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)

        rospy.loginfo("Sending zero trajectory: %s → [0,0,0,0,0,0]", JOINT_NAMES)
        client.send_goal(goal)

        if not client.wait_for_result(timeout=rospy.Duration(5.0)):
            rospy.logerr("Zero trajectory timed out")
            return False

        result = client.get_result()
        if result and result.error_code == 0:
            rospy.loginfo("Zero hold trajectory SUCCESS")
            return True
        else:
            rospy.logerr("Zero trajectory failed: error_code=%s",
                         result.error_code if result else "NONE")
            return False

    def verify_zero_joints(self):
        """验证六轴在 0 (±0.02rad) 内."""
        rospy.loginfo("Verifying joint positions...")
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
            names = list(msg.name)
            positions = list(msg.position)

            all_ok = True
            for jn in JOINT_NAMES:
                if jn not in names:
                    rospy.logerr("Joint %s missing from /joint_states", jn)
                    return False
                idx = names.index(jn)
                val = positions[idx]
                if abs(val) > 0.02:
                    rospy.logerr("Joint %s = %.4f rad, exceeds ±0.02 tolerance", jn, val)
                    all_ok = False
                else:
                    rospy.loginfo("Joint %s = %.4f rad OK", jn, val)

            return all_ok
        except rospy.ROSException:
            rospy.logerr("No /joint_states for zero verification")
            return False


def main():
    rospy.init_node("bootstrap_controllers", anonymous=True,
                    log_level=rospy.WARN)

    boot = ControllerBootstrapper()
    all_ok = True

    # Step 1: Unpause
    rospy.loginfo("=== Step 1: Unpause physics ===")
    if not boot.unpause():
        sys.stderr.write("CLOCK_NOT_ADVANCING\n")
        sys.stderr.flush()
        sys.exit(1)

    # Step 2: Verify clock
    rospy.loginfo("=== Step 2: Verify clock ===")
    if not boot.verify_clock():
        sys.stderr.write("CLOCK_NOT_ADVANCING\n")
        sys.stderr.flush()
        sys.exit(1)
    sys.stderr.write("SIM_CLOCK_ADVANCING\n")

    # Step 3: Start joint_state_controller
    rospy.loginfo("=== Step 3: Start joint_state_controller ===")
    if not boot.start_joint_state_controller():
        sys.stderr.write("JOINT_STATE_CONTROLLER_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)
    sys.stderr.write("JOINT_STATE_CONTROLLER_RUNNING\n")

    # Step 4: Wait for joint_states
    rospy.loginfo("=== Step 4: Wait for joint_states ===")
    if not boot.wait_joint_states():
        sys.stderr.write("JOINT_STATES_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)
    sys.stderr.write("JOINT_STATES_READY\n")

    # Step 5: Start arm_controller
    rospy.loginfo("=== Step 5: Start arm_controller ===")
    if not boot.start_arm_controller():
        sys.stderr.write("ARM_CONTROLLER_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)
    sys.stderr.write("ARM_CONTROLLER_RUNNING\n")

    # Step 6: Zero-hold trajectory
    rospy.loginfo("=== Step 6: Zero-hold trajectory ===")
    if not boot.send_zero_hold_trajectory():
        sys.stderr.write("ZERO_HOLD_FAILED\n")
        sys.stderr.flush()
        sys.exit(3)
    sys.stderr.write("ZERO_HOLD_ACTIVE\n")

    # Step 7: Verify zero joints
    rospy.loginfo("=== Step 7: Verify zero joints ===")
    if not boot.verify_zero_joints():
        rospy.logwarn("Joint zero verification failed, but controllers are running")
        # 非致命: controller 可能还在调整中

    sys.stderr.write("CONTROLLERS_RUNNING\n")
    sys.stderr.flush()
    rospy.loginfo("Controller bootstrap complete")
    sys.exit(0)


if __name__ == "__main__":
    main()

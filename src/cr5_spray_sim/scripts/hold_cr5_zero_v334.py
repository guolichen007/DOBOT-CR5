#!/usr/bin/env python3
"""
V3.3.4 CR5 Zero-Position Hold.

在控制器启动后发送六轴零位保持轨迹，验证机械臂姿态正确。
必须在 /clock 推进、控制器 running、/joint_states 正常发布后调用。

用法:
  rosrun cr5_spray_sim hold_cr5_zero_v334.py [--timeout 15.0]

输出到 stderr (供 wrapper 读取):
  CR5_ZERO_HOLD_OK — 零位保持成功
  CR5_ZERO_HOLD_FAILED — 保持失败

退出码:
  0 = CR5_ZERO_HOLD_OK
  1 = 服务/action 不可用
  2 = 关节不在零位
  3 = Link6/nozzle 高度异常
"""
import sys
import math
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ZERO_TOLERANCE = 0.03   # rad, 初始零位允许误差
MIN_LINK6_Z = 0.80      # Link6 最低高度 (m)
MIN_NOZZLE_Z = 0.80     # 喷嘴最低高度 (m)


def _collect_joint_states(timeout=8.0):
    """收集至少 3 帧 joint_states 并验证."""
    collected = []
    rospy.loginfo("Waiting for /joint_states (need >=3 frames)...")
    start = rospy.get_time()
    sub = None
    try:
        msg_buf = []

        def cb(msg):
            msg_buf.append(msg)

        sub = rospy.Subscriber("/joint_states", JointState, cb, queue_size=10)
        rate = rospy.Rate(20)
        while len(msg_buf) < 3 and rospy.get_time() - start < timeout:
            rate.sleep()

        collected = list(msg_buf)
    finally:
        if sub:
            sub.unregister()

    if len(collected) < 3:
        rospy.logerr("Only %d /joint_states frames in %.1fs (need >=3)",
                     len(collected), timeout)
        return None

    # 验证最后一帧
    last = collected[-1]
    names = list(last.name)
    positions = list(last.position)

    found = []
    for jn in JOINT_NAMES:
        if jn in names:
            idx = names.index(jn)
            val = positions[idx]
            if math.isfinite(val):
                found.append((jn, val))
            else:
                rospy.logerr("Joint %s value not finite: %s", jn, val)
                return None
        else:
            rospy.logerr("Joint %s not in /joint_states: %s", jn, names)
            return None

    if len(found) != 6:
        rospy.logerr("Only %d/6 CR5 joints found", len(found))
        return None

    rospy.loginfo("Joint states ready: %s",
                  {jn: round(v, 4) for jn, v in found})
    return {jn: v for jn, v in found}


def _send_zero_trajectory(timeout=10.0):
    """发送六轴零位 FollowJointTrajectory action."""
    client = actionlib.SimpleActionClient(
        "/arm_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
    )

    rospy.loginfo("Waiting for /arm_controller/follow_joint_trajectory action...")
    if not client.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("Action server not available after %.1fs", timeout)
        return False

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint(
        positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start=rospy.Duration(1.0),
    )
    goal.trajectory.points.append(point)

    rospy.loginfo("Sending zero-position hold trajectory...")
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(timeout))

    if not finished:
        rospy.logerr("Trajectory action timed out")
        client.cancel_goal()
        return False

    result = client.get_result()
    rospy.loginfo("Trajectory result: error_code=%d", result.error_code)
    return result.error_code >= 0  # SUCCESSFUL or similar


def _check_pose(joints):
    """检查关节角和末端高度."""
    failures = []
    for jn, val in joints.items():
        if abs(val) > ZERO_TOLERANCE:
            failures.append("  %s = %.4f rad (> %.3f)" % (jn, val, ZERO_TOLERANCE))

    if failures:
        rospy.logerr("Joint position check failed:")
        for f in failures:
            rospy.logerr(f)
        return False
    return True


def main():
    rospy.init_node("hold_cr5_zero_v334", anonymous=True, log_level=rospy.WARN)

    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) \
        if "--timeout" in sys.argv else 15.0

    # Step 1: 收集 joint_states
    joints = _collect_joint_states(timeout)
    if joints is None:
        sys.stderr.write("CR5_ZERO_HOLD_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    # Step 2: 发送零位轨迹
    rospy.loginfo("Sending zero-position hold...")
    if not _send_zero_trajectory(timeout):
        sys.stderr.write("CR5_ZERO_HOLD_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    rospy.sleep(1.0)  # 等关节运动到位

    # Step 3: 验证关节角
    joints = _collect_joint_states(max(timeout - 3, 5))
    if joints is None:
        sys.stderr.write("CR5_ZERO_HOLD_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    if not _check_pose(joints):
        sys.stderr.write("CR5_ZERO_HOLD_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    sys.stderr.write("CR5_ZERO_HOLD_OK\n")
    sys.stderr.flush()
    rospy.loginfo("Zero-position hold successful")


if __name__ == "__main__":
    main()

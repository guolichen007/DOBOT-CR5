#!/usr/bin/env python3
"""
V3.3.5 CR5 Controller Sequential Starter.

在 /clock 推进后分步启动控制器:
1. 先启动 joint_state_controller → 等待 running → 验证 /joint_states
2. 再启动 arm_controller → 等待 running

接受 initialized/stopped 作为合法初始状态。

用法:
  rosrun cr5_spray_sim start_cr5_controllers_v335.py [--timeout 20.0]

输出到 stderr (供 wrapper 读取):
  JOINT_STATE_CONTROLLER_RUNNING
  JOINT_STATES_READY
  ARM_CONTROLLER_RUNNING
  CONTROLLERS_RUNNING
  CONTROLLERS_FAILED

退出码:
  0 = CONTROLLERS_RUNNING
  1 = 服务不可用
  2 = 控制器启动失败
"""
import sys
import math
import rospy
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers, SwitchController

NOT_RUNNING_STATES = {"initialized", "stopped"}


def _list_controllers():
    """获取所有控制器状态."""
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=10.0)
    srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    resp = srv()
    return {c.name: c.state for c in resp.controller}


def _switch_controller(start_list, timeout=15.0):
    """显式切换控制器到 running."""
    rospy.wait_for_service("/controller_manager/switch_controller", timeout=10.0)
    srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    resp = srv(
        start_controllers=list(start_list),
        stop_controllers=[],
        strictness=SwitchController._request_class.STRICT,
        start_asap=True,
        timeout=timeout,
    )
    return resp.ok


def _wait_running(name, timeout=10.0):
    """等待某个控制器变为 running."""
    rate = rospy.Rate(10)
    waited = 0.0
    while waited < timeout:
        states = _list_controllers()
        if states.get(name) == "running":
            return True
        rate.sleep()
        waited += 0.1
    return False


def _collect_joint_states(need_frames=3, timeout=8.0):
    """收集 N 帧 /joint_states."""
    frames = []
    start = rospy.get_time()
    buf = []

    def cb(msg):
        buf.append(msg)

    sub = rospy.Subscriber("/joint_states", JointState, cb, queue_size=10)
    rate = rospy.Rate(20)
    try:
        while len(buf) < need_frames and rospy.get_time() - start < timeout:
            rate.sleep()
        frames = list(buf)
    finally:
        sub.unregister()

    if len(frames) < need_frames:
        return None, "only %d/%d frames" % (len(frames), need_frames)

    # 验证最后一帧
    last = frames[-1]
    names = list(last.name)
    positions = list(last.position)
    for jn in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
        if jn not in names:
            return None, "joint %s missing" % jn
        idx = names.index(jn)
        if not math.isfinite(positions[idx]):
            return None, "joint %s non-finite: %s" % (jn, positions[idx])

    return frames, None


def main():
    rospy.init_node("start_cr5_controllers_v335", anonymous=True,
                    log_level=rospy.WARN)

    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) \
        if "--timeout" in sys.argv else 20.0

    # 检查当前状态
    states = _list_controllers()
    rospy.loginfo("Initial controller states: %s", states)

    jsc_state = states.get("joint_state_controller", "MISSING")
    ac_state = states.get("arm_controller", "MISSING")

    # V3.3.5: 接受 initialized/stopped 作为合法初始状态
    if jsc_state not in NOT_RUNNING_STATES:
        rospy.logerr("joint_state_controller state='%s' (expected: %s)",
                     jsc_state, NOT_RUNNING_STATES)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)
    if ac_state not in NOT_RUNNING_STATES:
        rospy.logerr("arm_controller state='%s' (expected: %s)",
                     ac_state, NOT_RUNNING_STATES)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    # Step 1: 启动 joint_state_controller
    rospy.loginfo("Step 1: starting joint_state_controller...")
    ok = _switch_controller(["joint_state_controller"], timeout=timeout)
    if not ok:
        rospy.logerr("switch_controller(joint_state_controller) failed")
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    if not _wait_running("joint_state_controller", timeout):
        rospy.logerr("joint_state_controller not running after %.1fs", timeout)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    sys.stderr.write("JOINT_STATE_CONTROLLER_RUNNING\n")
    sys.stderr.flush()
    rospy.loginfo("joint_state_controller → running")

    # Step 2: 等待 /joint_states
    rospy.loginfo("Step 2: waiting for /joint_states...")
    _, err = _collect_joint_states(need_frames=3, timeout=8.0)
    if err:
        rospy.logerr("joint_states not ready: %s", err)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    sys.stderr.write("JOINT_STATES_READY\n")
    sys.stderr.flush()
    rospy.loginfo("/joint_states ready")

    # Step 3: 启动 arm_controller
    rospy.loginfo("Step 3: starting arm_controller...")
    ok = _switch_controller(["arm_controller"], timeout=timeout)
    if not ok:
        rospy.logerr("switch_controller(arm_controller) failed")
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    if not _wait_running("arm_controller", timeout):
        rospy.logerr("arm_controller not running after %.1fs", timeout)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    sys.stderr.write("ARM_CONTROLLER_RUNNING\n")
    sys.stderr.flush()
    rospy.loginfo("arm_controller → running")

    # 最终确认
    final_states = _list_controllers()
    rospy.loginfo("Final controller states: %s", final_states)
    sys.stderr.write("CONTROLLERS_RUNNING\n")
    sys.stderr.flush()


if __name__ == "__main__":
    main()

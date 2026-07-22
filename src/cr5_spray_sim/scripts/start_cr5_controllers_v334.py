#!/usr/bin/env python3
"""
V3.3.4 CR5 Controller Starter.

在 /clock 推进后显式启动 joint_state_controller 和 arm_controller。
必须在 Gazebo unpause 且 /clock 确认推进后调用。

用法:
  rosrun cr5_spray_sim start_cr5_controllers_v334.py [--timeout 15.0]

输出到 stderr (供 wrapper 读取):
  CONTROLLERS_RUNNING  — 全部成功
  CONTROLLERS_FAILED   — 启动失败

退出码:
  0 = CONTROLLERS_RUNNING
  1 = 服务不可用或超时
  2 = 控制器未能全部切换到 running
"""
import sys
import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController


EXPECTED_CONTROLLERS = {"joint_state_controller", "arm_controller"}


def _list_controllers():
    """获取所有控制器状态."""
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=10.0)
    srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    resp = srv()
    states = {}
    for c in resp.controller:
        states[c.name] = c.state
    return states


def _switch_controllers(start_list, timeout=10.0):
    """显式切换控制器到 running 状态."""
    rospy.wait_for_service("/controller_manager/switch_controller", timeout=10.0)
    srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

    resp = srv(
        start_controllers=start_list,
        stop_controllers=[],
        strictness=SwitchController._request_class.STRICT,
        start_asap=True,
        timeout=timeout,
    )
    return resp.ok


def main():
    rospy.init_node("start_cr5_controllers_v334", anonymous=True, log_level=rospy.WARN)

    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) \
        if "--timeout" in sys.argv else 15.0

    rospy.loginfo("Starting CR5 controllers (timeout=%.1fs)", timeout)

    # Step 1: 检查当前状态
    states = _list_controllers()
    rospy.loginfo("Current controller states: %s", states)

    # 确认两个控制器都存在且为 stopped
    for name in EXPECTED_CONTROLLERS:
        if name not in states:
            rospy.logerr("Controller '%s' not found in list_controllers", name)
            sys.stderr.write("CONTROLLERS_FAILED\n")
            sys.stderr.flush()
            sys.exit(2)
        if states[name] != "stopped":
            rospy.logwarn("Controller '%s' state is '%s' (expected 'stopped')",
                          name, states[name])

    # Step 2: 显式切换到 running
    ok = _switch_controllers(list(EXPECTED_CONTROLLERS), timeout=timeout)
    if not ok:
        rospy.logerr("switch_controller returned ok=false")
        # 打印当前状态帮助诊断
        final_states = _list_controllers()
        rospy.logerr("Final controller states: %s", final_states)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)

    # Step 3: 验证两个都 running
    rate = rospy.Rate(10)
    waited = 0.0
    while waited < timeout:
        states = _list_controllers()
        both_running = all(
            states.get(name) == "running"
            for name in EXPECTED_CONTROLLERS
        )
        if both_running:
            break
        rate.sleep()
        waited += 0.1

    states = _list_controllers()
    if all(states.get(name) == "running" for name in EXPECTED_CONTROLLERS):
        sys.stderr.write("CONTROLLERS_RUNNING\n")
        sys.stderr.flush()
        rospy.loginfo("Both controllers running: %s", states)
        return

    # 失败：输出诊断
    rospy.logerr("Controllers not running after %.1fs: %s", waited, states)
    sys.stderr.write("CONTROLLERS_FAILED\n")
    sys.stderr.flush()
    sys.exit(2)


if __name__ == "__main__":
    main()

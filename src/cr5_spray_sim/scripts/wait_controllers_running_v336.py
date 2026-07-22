#!/usr/bin/env python3
"""
V3.3.6 Controller Running Waiter.

轮询 /controller_manager/list_controllers，等待两个控制器都变为 running。
接受中间状态（空列表/initialized/stopped），继续等待。

用法:
  rosrun cr5_spray_sim wait_controllers_running_v336.py [--timeout 45.0]

输出到 stderr:
  CONTROLLERS_RUNNING
  CONTROLLERS_FAILED

退出码:
  0 = CONTROLLERS_RUNNING
  1 = 服务不可用
  2 = 超时
"""
import sys
import rospy
from controller_manager_msgs.srv import ListControllers

EXPECTED = {"joint_state_controller", "arm_controller"}
STABLE_COUNT = 5
POLL_INTERVAL = 0.2


def _list_controllers():
    """获取控制器状态字典."""
    srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    resp = srv()
    return {c.name: c.state for c in resp.controller}


def main():
    rospy.init_node("wait_controllers_running_v336", anonymous=True,
                    log_level=rospy.WARN)

    timeout = 45.0
    for i, arg in enumerate(sys.argv):
        if arg == "--timeout" and i + 1 < len(sys.argv):
            timeout = float(sys.argv[i + 1])

    # 等待服务
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=30.0)
    except rospy.ROSException:
        rospy.logerr("list_controllers service not available after 30s")
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    stable = 0
    rate = rospy.Rate(1.0 / POLL_INTERVAL)
    start = rospy.get_time()

    while rospy.get_time() - start < timeout:
        try:
            states = _list_controllers()
        except Exception as e:
            rospy.logwarn("list_controllers failed: %s", e)
            rate.sleep()
            continue

        jsc = states.get("joint_state_controller", "MISSING")
        ac = states.get("arm_controller", "MISSING")

        if jsc == "running" and ac == "running":
            stable += 1
            if stable >= STABLE_COUNT:
                elapsed = rospy.get_time() - start
                rospy.loginfo("both controllers running after %.1fs (stable %d)",
                              elapsed, stable)
                sys.stderr.write("CONTROLLERS_RUNNING\n")
                sys.stderr.flush()
                sys.exit(0)
        else:
            stable = 0
            rospy.loginfo("waiting controllers: jsc=%s, ac=%s", jsc, ac)

        rate.sleep()

    # 超时
    try:
        final = _list_controllers()
    except Exception:
        final = {}
    rospy.logerr("timeout after %.1fs: states=%s", timeout,
                 {k: final.get(k, "MISSING") for k in EXPECTED})
    sys.stderr.write("CONTROLLERS_FAILED\n")
    sys.stderr.flush()
    sys.exit(2)


if __name__ == "__main__":
    main()

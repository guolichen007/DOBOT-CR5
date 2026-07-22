#!/usr/bin/env python3
"""
V3.3.5 Controller Loaded Check.

通过 /controller_manager/list_controllers 服务直接获取控制器状态。
接受 initialized / stopped 作为"已加载、未运行"状态。
不再使用 shell grep/awk 解析字符串。

用法:
  rosrun cr5_spray_sim check_controllers_loaded_v335.py [--timeout 30.0]

输出到 stdout (JSON):
  {"joint_state_controller": "initialized", "arm_controller": "stopped"}

输出到 stderr (供 wrapper 读取):
  CONTROLLERS_LOADED_NOT_RUNNING  — 两个控制器都 loaded 且未 running
  CONTROLLERS_FAILED              — 服务不可用、控制器缺失或状态异常

退出码:
  0 = CONTROLLERS_LOADED_NOT_RUNNING
  1 = 服务不可用或超时
  2 = 控制器缺失、已 running 或状态异常
"""
import sys
import json
import rospy
from controller_manager_msgs.srv import ListControllers

EXPECTED_CONTROLLERS = {"joint_state_controller", "arm_controller"}
# V3.3.5: 合法的"已加载未运行"状态
NOT_RUNNING_STATES = {"initialized", "stopped"}


def main():
    rospy.init_node("check_controllers_loaded_v335", anonymous=True,
                    log_level=rospy.WARN)

    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) \
        if "--timeout" in sys.argv else 30.0

    rospy.loginfo("Checking controller load state (timeout=%.1fs)...", timeout)

    # 等待服务可用
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("list_controllers service not available after %.1fs", timeout)
        sys.stderr.write("CONTROLLERS_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    resp = srv()
    states = {c.name: c.state for c in resp.controller}

    # 输出 JSON 到 stdout 供 wrapper/forensics 使用
    print(json.dumps(states))
    sys.stdout.flush()

    rospy.loginfo("Controller states: %s", states)

    # 验证两个控制器存在且处于合法未运行状态
    for name in EXPECTED_CONTROLLERS:
        if name not in states:
            rospy.logerr("Controller '%s' not found. Available: %s",
                         name, list(states.keys()))
            sys.stderr.write("CONTROLLERS_FAILED\n")
            sys.stderr.flush()
            sys.exit(2)

        state = states[name]
        if state not in NOT_RUNNING_STATES:
            rospy.logerr("Controller '%s' state='%s' (expected: %s)",
                         name, state, NOT_RUNNING_STATES)
            sys.stderr.write("CONTROLLERS_FAILED\n")
            sys.stderr.flush()
            sys.exit(2)

    sys.stderr.write("CONTROLLERS_LOADED_NOT_RUNNING\n")
    sys.stderr.flush()
    rospy.loginfo("Both controllers loaded, not running: %s", states)


if __name__ == "__main__":
    main()

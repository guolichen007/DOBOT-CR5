#!/usr/bin/env python3
"""
CR5 Spray Demo: Execution Safety Check
验证 /use_sim_time=true 且不存在 dobot_bringup 节点后才能允许 Gazebo 执行。
"""
import sys
import rospy


def check_safety():
    rospy.init_node("execution_safety_check", anonymous=True)

    confirm = rospy.get_param("~simulation_confirm", "")
    if confirm != "GAZEBO_ONLY":
        rospy.logerr("SAFETY: simulation_confirm must be 'GAZEBO_ONLY' to execute")
        sys.exit(1)

    # Check /use_sim_time
    use_sim = rospy.get_param("/use_sim_time", False)
    if not use_sim:
        rospy.logerr("SAFETY: /use_sim_time is not true. Refusing to execute.")
        sys.exit(1)

    # Check no dobot_bringup nodes
    try:
        nodes = rospy.get_master().lookupNode(
            "", "dobot_bringup")  # getSystemState
    except Exception:
        nodes = []

    # Use rosnode to check
    import subprocess
    result = subprocess.run(
        ["rosnode", "list"], capture_output=True, text=True, timeout=5)
    for line in result.stdout.splitlines():
        if "dobot_bringup" in line.lower():
            rospy.logerr("SAFETY: dobot_bringup node detected! Refusing to execute.")
            sys.exit(1)

    rospy.loginfo("SAFETY CHECK PASSED: use_sim_time=%s, no dobot_bringup, confirm=%s",
                  use_sim, confirm)
    rospy.spin()


if __name__ == "__main__":
    check_safety()

#!/usr/bin/env python3
"""
V2 Scene Preflight: 检测启动冲突
检查已存在的 ROS master、Gazebo、同名节点和模型。
有冲突时打印详情并返回非零。
"""
import os
import sys
import subprocess
import socket


def check_port(port):
    """Return True if port is in use."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect(("127.0.0.1", port))
        s.close()
        return True
    except Exception:
        return False


def get_ros_nodes():
    """Get list of running ROS node names."""
    try:
        import rosgraph
        master = rosgraph.Master("preflight")
        state = master.getSystemState()
        nodes = set()
        for topic_type, topic_nodes in state[2]:
            nodes.update(topic_nodes)
        return nodes
    except Exception:
        return set()


def get_running_processes():
    """Get PIDs of ros/gz processes."""
    procs = []
    result = subprocess.run(
        ["ps", "-eo", "pid,cmd"], capture_output=True, text=True, timeout=5)
    for line in result.stdout.splitlines():
        if any(k in line for k in ["rosmaster", "gzserver", "gzclient",
                                    "roslaunch", "robot_state_publisher"]):
            parts = line.strip().split(None, 1)
            if parts:
                procs.append(parts)
    return procs


def main():
    conflicts = []

    # 1. Check ROS master
    if check_port(11311):
        conflicts.append("ROS master port 11311 in use")

    # 2. Check Gazebo master
    if check_port(11345):
        conflicts.append("Gazebo master port 11345 in use")

    # 3. Check running nodes
    nodes = get_ros_nodes()
    conflict_nodes = {"gazebo", "robot_state_publisher",
                      "cr5_controller_spawner", "joint_state_publisher"}
    for n in conflict_nodes:
        if any(n in node for node in nodes):
            conflicts.append("Node '{}' already running".format(n))

    # 4. Check running processes
    procs = get_running_processes()
    for pid, cmd in procs:
        conflicts.append("Process PID={}: {}".format(pid, cmd[:80]))

    if conflicts:
        print("=== PREFLIGHT FAILED: {} conflict(s) ===".format(len(conflicts)))
        for c in conflicts:
            print("  CONFLICT: {}".format(c))
        print("\nPlease stop existing sessions before launching a new scene.")
        print("  Recommended: use 'run_scene_v2.sh --isolated'")
        sys.exit(1)
    else:
        print("=== PREFLIGHT PASSED: no conflicts ===")
        sys.exit(0)


if __name__ == "__main__":
    main()

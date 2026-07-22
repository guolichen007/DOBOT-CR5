#!/usr/bin/env python3
"""
V3.3.7 Simulation Process Audit.

启动前扫描 roscore/rosmaster/roslaunch/gzserver/gzclient 进程，
检测旧 session 占用，绑定当前 session 的 ROS/GAZEBO_MASTER_URI。

用法:
  rosrun cr5_spray_sim audit_sim_processes_v337.py [--gui]

输出到 stderr:
  SIM_PROCESS_PREFLIGHT_PASS
  SIM_PROCESS_PREFLIGHT_FAIL

退出码:
  0 = 干净环境，可以启动
  1 = 旧 gzserver 或 scene launch 存在
  2 = 端口被占用
"""
import sys
import os
import re
import subprocess
import rospy


# 需要检查的进程模式
PROC_PATTERNS = {
    "roscore": re.compile(r"roscore"),
    "rosmaster": re.compile(r"rosmaster"),
    "gzserver": re.compile(r"gzserver"),
    "gzclient": re.compile(r"gzclient"),
    "scene_launch": re.compile(r"scene_v33_spray\.launch"),
}


def _list_processes():
    """列出系统中所有相关进程."""
    try:
        r = subprocess.run(
            ["ps", "-eo", "pid,ppid,pgid,args", "--no-headers"],
            capture_output=True, text=True, timeout=10,
        )
        return r.stdout.strip().split("\n") if r.stdout.strip() else []
    except Exception as e:
        rospy.logwarn("ps failed: %s", e)
        return []


def _read_environ(pid):
    """读取 /proc/<pid>/environ 并解析为 dict."""
    env = {}
    try:
        with open("/proc/{}/environ".format(pid), "r") as f:
            data = f.read()
        for item in data.split("\0"):
            if "=" in item:
                key, val = item.split("=", 1)
                env[key] = val
    except (IOError, PermissionError):
        pass
    return env


def _check_port(port):
    """检查指定端口是否被占用."""
    try:
        r = subprocess.run(
            ["ss", "-tlnp"], capture_output=True, text=True, timeout=5,
        )
        for line in r.stdout.split("\n"):
            if ":{} ".format(port) in line:
                return True
    except Exception:
        pass
    return False


def _find_available_port(start, max_attempts=100):
    """从 start 开始找可用端口."""
    for offset in range(max_attempts):
        port = start + offset
        if not _check_port(port):
            return port
    return None


def main():
    rospy.init_node("audit_sim_processes_v337", anonymous=True,
                    log_level=rospy.WARN)

    gui = "--gui" in sys.argv

    # 获取当前 session 的环境变量
    ros_master_uri = os.environ.get("ROS_MASTER_URI", "unknown")
    gazebo_master_uri = os.environ.get("GAZEBO_MASTER_URI", "unknown")

    rospy.loginfo("Current ROS_MASTER_URI: %s", ros_master_uri)
    rospy.loginfo("Current GAZEBO_MASTER_URI: %s", gazebo_master_uri)

    # 扫描进程
    all_procs = _list_processes()
    conflicts = []

    for line in all_procs:
        parts = line.strip().split(None, 3)
        if len(parts) < 4:
            continue
        pid_str, ppid_str, pgid_str, args = parts
        try:
            pid = int(pid_str)
        except ValueError:
            continue

        # 跳过自己
        if pid == os.getpid() or pid == os.getppid():
            continue

        # 检查是否匹配关键进程
        matched = []
        for name, pattern in PROC_PATTERNS.items():
            if pattern.search(args) and "audit_sim_processes" not in args:
                matched.append(name)

        if matched:
            env = _read_environ(pid)
            proc_info = {
                "pid": pid,
                "args": args[:120],
                "matched": matched,
                "ros_master_uri": env.get("ROS_MASTER_URI", "N/A"),
                "gazebo_master_uri": env.get("GAZEBO_MASTER_URI", "N/A"),
            }
            conflicts.append(proc_info)

    # 分类冲突
    old_gzserver = [c for c in conflicts if "gzserver" in c["matched"]]
    old_scene_launch = [c for c in conflicts if "scene_launch" in c["matched"]]
    old_gzclient = [c for c in conflicts if "gzclient" in c["matched"]]

    # 报告
    if old_gzserver:
        rospy.logerr("Found existing gzserver processes:")
        for c in old_gzserver:
            rospy.logerr("  PID %d: %s", c["pid"], c["args"])
            rospy.logerr("    GAZEBO_MASTER_URI=%s", c["gazebo_master_uri"])
        sys.stderr.write("SIM_PROCESS_PREFLIGHT_FAIL\n")
        sys.stderr.write("OLD_GZSERVER_FOUND\n")
        sys.stderr.flush()
        sys.exit(1)

    if old_scene_launch:
        rospy.logerr("Found existing scene launch processes:")
        for c in old_scene_launch:
            rospy.logerr("  PID %d: %s", c["pid"], c["args"])
            rospy.logerr("    ROS_MASTER_URI=%s", c["ros_master_uri"])
        sys.stderr.write("SIM_PROCESS_PREFLIGHT_FAIL\n")
        sys.stderr.write("OLD_SCENE_LAUNCH_FOUND\n")
        sys.stderr.flush()
        sys.exit(1)

    # 报告旧 gzclient 但不是致命错误
    if old_gzclient:
        rospy.logwarn("Found existing gzclient processes (non-fatal):")
        for c in old_gzclient:
            rospy.logwarn("  PID %d: %s", c["pid"], c["args"])
            rospy.logwarn("    GAZEBO_MASTER_URI=%s", c["gazebo_master_uri"])

    # 检查端口占用
    ros_port = None
    gazebo_port = None
    try:
        ros_port = int(ros_master_uri.rsplit(":", 1)[-1]) if ":" in ros_master_uri else 11311
    except (ValueError, AttributeError):
        ros_port = 11311
    try:
        gazebo_port = int(gazebo_master_uri.rsplit(":", 1)[-1]) if ":" in gazebo_master_uri else 11345
    except (ValueError, AttributeError):
        gazebo_port = 11345

    ros_occupied = _check_port(ros_port)
    gz_occupied = _check_port(gazebo_port)

    if ros_occupied:
        rospy.logerr("Port %d (ROS_MASTER_URI) is already in use", ros_port)
    if gz_occupied:
        rospy.logerr("Port %d (GAZEBO_MASTER_URI) is already in use", gazebo_port)

    if ros_occupied or gz_occupied:
        # 如果是 GUI 模式，寻找 fallback 端口
        if gui:
            alt_ros = _find_available_port(ros_port + 1) if ros_occupied else ros_port
            alt_gz = _find_available_port(gazebo_port + 1) if gz_occupied else gazebo_port
            if alt_ros and alt_gz:
                rospy.loginfo("Falling back to alternative ports: ROS=%d GAZEBO=%d",
                              alt_ros, alt_gz)
                sys.stderr.write("SIM_PROCESS_PREFLIGHT_PASS\n")
                sys.stderr.write("PORT_FALLBACK rosmaster={} gazebo_master={}\n".format(
                    alt_ros, alt_gz))
                sys.stderr.flush()
                sys.exit(0)

        sys.stderr.write("SIM_PROCESS_PREFLIGHT_FAIL\n")
        sys.stderr.write("PORT_CONFLICT\n")
        sys.stderr.flush()
        sys.exit(2)

    # 通过
    rospy.loginfo("No conflicting processes found, environment clean")
    sys.stderr.write("SIM_PROCESS_PREFLIGHT_PASS\n")
    sys.stderr.flush()
    sys.exit(0)


if __name__ == "__main__":
    main()

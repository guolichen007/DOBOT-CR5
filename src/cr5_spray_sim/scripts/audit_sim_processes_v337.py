#!/usr/bin/env python3
"""
V4 Simulation Process Audit — 独立进程扫描 (不依赖 ROS).

V4 修复:
- 去掉 rospy 依赖, 在 roscore 启动前运行
- 只检查旧 gzserver / gzclient / scene launch 进程
- 不再检查端口 (端口分配由 run_scene_v337.sh 唯一负责)

用法:
  python3 audit_sim_processes_v337.py [--gui]

退出码:
  0 = 干净环境
  1 = 旧 gzserver 或 scene launch 存在
  2 = 旧 gzclient (GUI 模式下) 存在
"""
import sys
import os
import re
import subprocess


PROC_PATTERNS = {
    "gzserver": re.compile(r"gzserver"),
    "gzclient": re.compile(r"gzclient"),
    "scene_launch": re.compile(r"scene_v33_spray"),
}


def _list_processes():
    """列出所有相关进程."""
    try:
        r = subprocess.run(
            ["ps", "-eo", "pid,ppid,pgid,args", "--no-headers"],
            capture_output=True, text=True, timeout=10,
        )
        return r.stdout.strip().split("\n") if r.stdout.strip() else []
    except Exception as e:
        print(f"WARN: ps failed: {e}", file=sys.stderr)
        return []


def _read_environ(pid):
    """读取 /proc/<pid>/environ."""
    env = {}
    try:
        with open(f"/proc/{pid}/environ", "r") as f:
            data = f.read()
        for item in data.split("\0"):
            if "=" in item:
                key, val = item.split("=", 1)
                env[key] = val
    except (IOError, PermissionError):
        pass
    return env


def main():
    gui = "--gui" in sys.argv
    my_pid = os.getpid()
    my_ppid = os.getppid()

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

        # 跳过自己和父进程
        if pid in (my_pid, my_ppid):
            continue
        # 跳过本脚本
        if "audit_sim_processes" in args:
            continue

        matched = [name for name, pattern in PROC_PATTERNS.items()
                   if pattern.search(args)]
        if matched:
            env = _read_environ(pid)
            conflicts.append({
                "pid": pid,
                "args": args[:150],
                "matched": matched,
                "ros_master_uri": env.get("ROS_MASTER_URI", "N/A"),
                "gazebo_master_uri": env.get("GAZEBO_MASTER_URI", "N/A"),
            })

    old_gzserver = [c for c in conflicts if "gzserver" in c["matched"]]
    old_scene_launch = [c for c in conflicts if "scene_launch" in c["matched"]]
    old_gzclient = [c for c in conflicts if "gzclient" in c["matched"]]

    exit_code = 0

    if old_gzserver:
        print("ERROR: Found existing gzserver processes:", file=sys.stderr)
        for c in old_gzserver:
            print(f"  PID {c['pid']}: {c['args']}", file=sys.stderr)
            print(f"    GAZEBO_MASTER_URI={c['gazebo_master_uri']}", file=sys.stderr)
        exit_code = 1

    if old_scene_launch:
        print("ERROR: Found existing scene launch processes:", file=sys.stderr)
        for c in old_scene_launch:
            print(f"  PID {c['pid']}: {c['args']}", file=sys.stderr)
            print(f"    ROS_MASTER_URI={c['ros_master_uri']}", file=sys.stderr)
        exit_code = 1

    if old_gzclient:
        if gui:
            print("ERROR: Found existing gzclient processes — fatal in GUI mode:", file=sys.stderr)
            for c in old_gzclient:
                print(f"  PID {c['pid']}: {c['args']}", file=sys.stderr)
                print(f"    GAZEBO_MASTER_URI={c['gazebo_master_uri']}", file=sys.stderr)
            exit_code = 2
        else:
            print("WARN: Found existing gzclient processes (headless, non-fatal):", file=sys.stderr)
            for c in old_gzclient:
                print(f"  PID {c['pid']}: {c['args']}", file=sys.stderr)

    if exit_code != 0:
        print("SIM_PROCESS_PREFLIGHT_FAIL", file=sys.stderr)
        if old_gzserver:
            print("OLD_GZSERVER_FOUND", file=sys.stderr)
        if old_scene_launch:
            print("OLD_SCENE_LAUNCH_FOUND", file=sys.stderr)
        if old_gzclient and gui:
            print("OLD_GZCLIENT_FOUND", file=sys.stderr)
    else:
        print("SIM_PROCESS_PREFLIGHT_PASS", file=sys.stderr)

    sys.stderr.flush()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

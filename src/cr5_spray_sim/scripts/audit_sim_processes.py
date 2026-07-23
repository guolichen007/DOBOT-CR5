#!/usr/bin/env python3
"""
Calibration Simulation Process Audit — 独立进程扫描 + 清理 (不依赖 ROS).

Calibration 修复:
- 去掉 rospy 依赖, 在 roscore 启动前运行
- 只检查旧 gzserver / gzclient / scene launch 进程
- 不再检查端口 (端口分配由 run_simulation.sh 唯一负责)
- --cleanup 模式自动杀掉检测到的旧进程

用法:
  python3 audit_sim_processes.py [--gui] [--cleanup]

退出码:
  0 = 干净环境
  1 = 旧 gzserver 或 scene launch 存在
  2 = 旧 gzclient (GUI 模式下) 存在
"""
import sys
import os
import re
import signal
import subprocess


PROC_PATTERNS = {
    "gzserver": re.compile(r"gzserver"),
    "gzclient": re.compile(r"gzclient"),
    "scene_launch": re.compile(r"spray_simulation"),
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


def _kill_process(pid):
    """安全杀掉一个进程 (先 SIGTERM, 等 1s, 再 SIGKILL)."""
    try:
        os.kill(pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        return False
    import time
    time.sleep(0.3)
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass  # already dead
    except PermissionError:
        return False
    return True


def cleanup_old_processes(gui=False):
    """查找并杀掉旧仿真进程。返回清理报告."""
    my_pid = os.getpid()
    my_ppid = os.getppid()
    all_procs = _list_processes()

    killed = {"gzserver": [], "gzclient": [], "scene_launch": []}

    for line in all_procs:
        parts = line.strip().split(None, 3)
        if len(parts) < 4:
            continue
        pid_str, ppid_str, pgid_str, args = parts
        try:
            pid = int(pid_str)
        except ValueError:
            continue

        if pid in (my_pid, my_ppid):
            continue
        if "audit_sim_processes" in args:
            continue

        for name, pattern in PROC_PATTERNS.items():
            if pattern.search(args):
                # headless 模式不删 gzclient
                if name == "gzclient" and not gui:
                    continue
                if _kill_process(pid):
                    killed[name].append(pid)

    # 最后删除锁文件和残留环境文件
    for f in ["/tmp/cr5_spray_demo.lock", "/tmp/cr5_spray_simulation.env"]:
        try:
            os.remove(f)
        except (IOError, OSError):
            pass

    return killed


def main():
    gui = "--gui" in sys.argv
    cleanup = "--cleanup" in sys.argv

    # cleanup 模式：先清理再扫描确认
    if cleanup:
        killed = cleanup_old_processes(gui=gui)
        total = sum(len(v) for v in killed.values())
        if total > 0:
            parts = []
            if killed["gzserver"]:
                parts.append(f"gzserver({','.join(map(str, killed['gzserver']))})")
            if killed["gzclient"]:
                parts.append(f"gzclient({','.join(map(str, killed['gzclient']))})")
            if killed["scene_launch"]:
                parts.append(f"launch({','.join(map(str, killed['scene_launch']))})")
            print(f"CLEANUP: killed {total} old processes: {', '.join(parts)}",
                  file=sys.stderr)
        else:
            print("CLEANUP: no old processes found", file=sys.stderr)
        # 清理后短暂等待
        import time
        time.sleep(1.0)

    # 扫描
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

        if pid in (my_pid, my_ppid):
            continue
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
            print("ERROR: Found existing gzclient processes — fatal in GUI mode:",
                  file=sys.stderr)
            for c in old_gzclient:
                print(f"  PID {c['pid']}: {c['args']}", file=sys.stderr)
                print(f"    GAZEBO_MASTER_URI={c['gazebo_master_uri']}", file=sys.stderr)
            exit_code = 2
        else:
            print("WARN: Found existing gzclient processes (headless, non-fatal):",
                  file=sys.stderr)
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

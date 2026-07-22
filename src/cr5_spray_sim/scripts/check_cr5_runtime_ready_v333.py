#!/usr/bin/env python3
"""
V3.3.3 CR5 Runtime Readiness Check (wall-time based).

与 V3.3.2 的区别:
- 使用 wall clock (time.monotonic()) 做所有超时，不依赖 sim time
- 验证控制器为 running 状态 (不允许 stopped/initialized/aborted)
- 要求连续收到 >=3 帧 joint_states，stamp 非倒退
- 验证六轴全部 finite 且 |q| <= 0.03 rad

退出码:
  0 = CR5_RUNTIME_READY
  1 = CONTROLLERS_NOT_RUNNING
  2 = JOINT_STATES_TIMEOUT
  3 = JOINT_STATES_INVALID
"""
import sys
import math
import time
import rospy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MAX_JOINT_ERR_RAD = 0.03
WALL_TIMEOUT_S = 15.0
MIN_CONSECUTIVE_FRAMES = 3


def check_controllers_running():
    """Verify both controllers are in 'running' state."""
    print("  Checking controllers...")
    wall_start = time.monotonic()
    while time.monotonic() - wall_start < 10.0:
        try:
            rospy.wait_for_service("/controller_manager/list_controllers", timeout=3.0)
            lc = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
            resp = lc()
            running = {}
            for c in resp.controller:
                if c.name in ["joint_state_controller", "arm_controller"]:
                    running[c.name] = c.state
                    print(f"    {c.name}: {c.state}")

            for name in ["joint_state_controller", "arm_controller"]:
                if name not in running:
                    print(f"    [FAIL] {name}: not found in controller list")
                    return False
                if running[name] != "running":
                    print(f"    [FAIL] {name}: state={running[name]}, expected=running")
                    return False
            print("    [OK] Both controllers running")
            return True
        except Exception as e:
            print(f"    Waiting for controller_manager... ({e})")
            time.sleep(1.0)

    print("    [FATAL] controller_manager/list_controllers not available")
    return False


def check_joint_states():
    """Wait for valid joint_states using wall clock."""
    print("  Checking joint_states (wall-time)...")
    wall_start = time.monotonic()

    msg_count = 0
    last_stamp = None
    positions_history = []

    # Subscribe manually to track consecutive messages
    msgs = []

    def callback(msg):
        msgs.append(msg)

    sub = rospy.Subscriber("/joint_states", JointState, callback, queue_size=10)

    while time.monotonic() - wall_start < WALL_TIMEOUT_S:
        if len(msgs) >= MIN_CONSECUTIVE_FRAMES:
            break
        time.sleep(0.1)

    sub.unregister()

    if len(msgs) < MIN_CONSECUTIVE_FRAMES:
        print(f"    [FAIL] Only {len(msgs)} joint_states messages in {WALL_TIMEOUT_S}s "
              f"(need >={MIN_CONSECUTIVE_FRAMES})")
        return False

    print(f"    Received {len(msgs)} joint_states messages in "
          f"{time.monotonic() - wall_start:.1f}s")

    # Verify stamp non-decreasing
    stamps_ok = True
    for i in range(1, len(msgs)):
        t_prev = msgs[i-1].header.stamp
        t_curr = msgs[i].header.stamp
        if t_curr < t_prev:
            print(f"    [FAIL] stamp倒退: msg[{i}] {t_curr.to_sec():.3f} < msg[{i-1}] {t_prev.to_sec():.3f}")
            stamps_ok = False
    if stamps_ok:
        print(f"    [OK] Stamp monotonic: {msgs[0].header.stamp.to_sec():.3f} → "
              f"{msgs[-1].header.stamp.to_sec():.3f}")

    # Use the last message for joint verification
    msg = msgs[-1]
    positions = dict(zip(msg.name, msg.position))

    all_ok = True
    for name in JOINT_NAMES:
        actual = positions.get(name)
        if actual is None:
            print(f"    [FAIL] {name}: missing from joint_states")
            all_ok = False
        elif not math.isfinite(actual):
            print(f"    [FAIL] {name}: NaN/Inf")
            all_ok = False
        elif abs(actual) > MAX_JOINT_ERR_RAD:
            print(f"    [FAIL] {name}: {actual:.4f} rad (limit={MAX_JOINT_ERR_RAD})")
            all_ok = False

    if all_ok:
        angle_str = ", ".join(f"{name}={positions.get(name, 0):.4f}" for name in JOINT_NAMES)
        print(f"    [OK] All joints near zero: {angle_str}")

    return all_ok and stamps_ok


def main():
    rospy.init_node("check_cr5_runtime_ready_v333", anonymous=True, disable_signals=True)

    wall_start = time.monotonic()

    # 1. Controllers must be running
    if not check_controllers_running():
        print("\nFATAL: CONTROLLERS_NOT_RUNNING")
        sys.exit(1)

    # 2. Joint states must be valid
    if not check_joint_states():
        print("\nFATAL: JOINT_STATES_INVALID")
        sys.exit(2)

    elapsed = time.monotonic() - wall_start
    print(f"\nCR5_RUNTIME_READY ({elapsed:.1f}s wall-time)")
    sys.exit(0)


if __name__ == "__main__":
    main()

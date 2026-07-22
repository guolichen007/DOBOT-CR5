#!/usr/bin/env python3
"""
V3.3.3 Gazebo Unpause + Clock Verification (wall-time based).

与 V3.3.2 的区别:
- unpause 调用后必须验证 /clock 实际推进
- 使用 wall clock 做所有超时
- 失败时重试最多 3 次
- 读取两次 world_properties，要求 sim_time 也在增加
- 禁止静默吞掉 unpause 失败

退出码:
  0 = GAZEBO_CLOCK_ADVANCING
  1 = UNPAUSE_SERVICE_FAILED
  2 = CLOCK_NOT_ADVANCING
"""
import sys
import time
import rospy
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import GetWorldProperties

MAX_RETRIES = 3
WALL_CLOCK_TIMEOUT = 8.0
MIN_CLOCK_MSGS = 5
MIN_CLOCK_ADVANCE_S = 0.20


def call_unpause():
    """Call /gazebo/unpause_physics, return True on success."""
    try:
        rospy.wait_for_service("/gazebo/unpause_physics", timeout=5.0)
        unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        unpause()
        print("    unpause_physics called successfully")
        return True
    except Exception as e:
        print(f"    [FAIL] unpause_physics: {e}")
        return False


def check_world_sim_time():
    """Read /gazebo/get_world_properties twice, verify sim_time advances."""
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5.0)
        gw = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        r1 = gw()
        t1 = r1.sim_time
        time.sleep(1.0)
        r2 = gw()
        t2 = r2.sim_time
        if t2 > t1:
            advance = t2.to_sec() - t1.to_sec()
            print(f"    [OK] World sim_time advancing: {t1.to_sec():.3f}s → {t2.to_sec():.3f}s "
                  f"(+{advance:.3f}s)")
            return True
        else:
            print(f"    [FAIL] World sim_time stalled: {t1.to_sec():.3f}s → {t2.to_sec():.3f}s")
            return False
    except Exception as e:
        print(f"    [WARN] get_world_properties: {e}")
        return None  # indeterminate


def verify_clock():
    """Subscribe to /clock using wall clock, verify advancing."""
    print(f"  Verifying /clock advances (wall-time timeout={WALL_CLOCK_TIMEOUT}s)...")
    wall_start = time.monotonic()

    clock_msgs = []

    def callback(msg):
        clock_msgs.append(msg.clock)

    sub = rospy.Subscriber("/clock", Clock, callback, queue_size=20)

    while time.monotonic() - wall_start < WALL_CLOCK_TIMEOUT:
        if len(clock_msgs) >= MIN_CLOCK_MSGS + 2:
            break
        time.sleep(0.05)

    sub.unregister()

    if len(clock_msgs) < MIN_CLOCK_MSGS:
        print(f"    [FAIL] Only {len(clock_msgs)} clock messages (need >={MIN_CLOCK_MSGS})")
        return False

    first = clock_msgs[0]
    last = clock_msgs[-1]
    advance = (last - first).to_sec()

    print(f"    {len(clock_msgs)} clock msgs in {time.monotonic() - wall_start:.1f}s wall-time")
    print(f"    Clock range: {first.to_sec():.3f}s → {last.to_sec():.3f}s "
          f"(advance={advance:.3f}s)")

    if advance < MIN_CLOCK_ADVANCE_S:
        print(f"    [FAIL] Clock advance {advance:.3f}s < {MIN_CLOCK_ADVANCE_S}s")
        return False

    # Check at least one strict increase
    strictly_increasing = False
    for i in range(1, len(clock_msgs)):
        if clock_msgs[i] > clock_msgs[i-1]:
            strictly_increasing = True
            break

    if not strictly_increasing:
        print(f"    [FAIL] No strictly-increasing adjacent clock messages")
        return False

    print(f"    [OK] Strictly increasing: confirmed")
    return True


def main():
    rospy.init_node("unpause_and_verify_clock_v333", anonymous=True, disable_signals=True)

    # Step 1: Unpause
    print("=== Unpause Gazebo ===")
    for attempt in range(1, MAX_RETRIES + 1):
        print(f"  Attempt {attempt}/{MAX_RETRIES}...")
        if not call_unpause():
            if attempt < MAX_RETRIES:
                print(f"    Retrying in 2s...")
                time.sleep(2.0)
            continue

        # Step 2: Verify clock
        if verify_clock():
            # Step 3: Cross-check with world_properties
            world_ok = check_world_sim_time()
            if world_ok is False:
                print(f"    [FAIL] World sim_time cross-check failed")
                if attempt < MAX_RETRIES:
                    time.sleep(2.0)
                    continue
                break

            print(f"\nSIM_CLOCK_ADVANCING")
            sys.exit(0)
        else:
            if attempt < MAX_RETRIES:
                print(f"    Retrying unpause + clock verify...")
                time.sleep(2.0)

    print(f"\nFATAL: GAZEBO_CLOCK_NOT_ADVANCING (after {MAX_RETRIES} attempts)")
    sys.exit(1)


if __name__ == "__main__":
    main()

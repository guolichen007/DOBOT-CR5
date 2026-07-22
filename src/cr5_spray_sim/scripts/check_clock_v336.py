#!/usr/bin/env python3
"""
V3.3.6 Clock Health Check.

固定采样 1.0 秒 wall-time 的 /clock 消息。
不调用 unpause——假设 Gazebo 已直接 unpaused 启动。

用法:
  rosrun cr5_spray_sim check_clock_v336.py

输出到 stderr:
  SIM_CLOCK_ADVANCING
  CLOCK_NOT_ADVANCING

退出码:
  0 = SIM_CLOCK_ADVANCING
  1 = 没有消息
  2 = 时钟未推进
"""
import sys
import time
import rospy
from rosgraph_msgs.msg import Clock


def main():
    rospy.init_node("check_clock_v336", anonymous=True,
                    log_level=rospy.WARN)

    SAMPLE_DURATION = 1.0  # 固定采样 1 秒
    MIN_MSGS = 10
    MIN_ADVANCE = 0.001  # 秒

    clock_msgs = []

    def cb(msg):
        clock_msgs.append(msg.clock.to_sec())

    sub = rospy.Subscriber("/clock", Clock, cb, queue_size=50)

    wall_start = time.monotonic()
    rate = rospy.Rate(50)
    while time.monotonic() - wall_start < SAMPLE_DURATION:
        rate.sleep()

    sub.unregister()

    count = len(clock_msgs)
    if count < MIN_MSGS:
        rospy.logerr("only %d /clock messages in %.1fs (need >= %d)",
                     count, SAMPLE_DURATION, MIN_MSGS)
        sys.stderr.write("CLOCK_NOT_ADVANCING\n")
        sys.stderr.flush()
        sys.exit(1)

    first = clock_msgs[0]
    last = clock_msgs[-1]
    advance = last - first

    # 检查严格递增
    increasing = all(clock_msgs[i] > clock_msgs[i - 1]
                     for i in range(1, len(clock_msgs)))

    rospy.loginfo("/clock: %d msgs, %.3fs → %.3fs (adv=%.4fs, increasing=%s)",
                  count, first, last, advance, increasing)

    if advance >= MIN_ADVANCE and increasing:
        sys.stderr.write("SIM_CLOCK_ADVANCING\n")
        sys.stderr.flush()
        sys.exit(0)
    else:
        rospy.logerr("clock not advancing: advance=%.4fs, increasing=%s",
                     advance, increasing)
        sys.stderr.write("CLOCK_NOT_ADVANCING\n")
        sys.stderr.flush()
        sys.exit(2)


if __name__ == "__main__":
    main()

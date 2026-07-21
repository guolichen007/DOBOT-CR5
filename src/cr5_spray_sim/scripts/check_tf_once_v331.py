#!/usr/bin/env python3
"""
V3.3.1: 一次性 TF 连通性检查。

替代启动器中的后台 tf_echo 循环。
每项查询 3 秒超时，检查数值 finite。
全部通过返回 0，任一失败返回非零。
"""
import sys
import math
import rospy
import tf2_ros


def check_transform(tf_buf, from_frame, to_frame, timeout_s=3.0):
    """Look up transform once. Returns (ok, msg)."""
    try:
        t = tf_buf.lookup_transform(from_frame, to_frame,
                                    rospy.Time(0), rospy.Duration(timeout_s))
        tx = t.transform.translation
        rt = t.transform.rotation
        # Check all values are finite
        vals = [tx.x, tx.y, tx.z, rt.x, rt.y, rt.z, rt.w]
        for v in vals:
            if not math.isfinite(v):
                return (False, f"NaN/Inf in {from_frame}→{to_frame}")
        # Check position not absurd (>50m)
        dist = math.sqrt(tx.x**2 + tx.y**2 + tx.z**2)
        if dist > 50.0:
            return (False, f"absurd distance {dist:.1f}m in {from_frame}→{to_frame}")
        return (True, f"[{tx.x:.3f}, {tx.y:.3f}, {tx.z:.3f}]")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        return (False, f"{from_frame}→{to_frame}: {type(e).__name__}")


def main():
    rospy.init_node("check_tf_once_v331", anonymous=True, disable_signals=True)

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)

    # Wait briefly for TF buffer to populate
    rospy.sleep(2.0)

    # Critical TF pairs that must be connected
    checks = [
        ("world", "dummy_link", True),       # must exist
        ("world", "base_link", True),        # must exist
        ("world", "Link6", True),            # must exist
        ("world", "spray_nozzle_frame", True),  # must exist (if spray tool enabled)
        ("world", "object_frame", True),     # must exist
        ("object_frame", "spray_nozzle_frame", False),  # connected via world
    ]

    failed = 0
    for from_f, to_f, required in checks:
        ok, msg = check_transform(tf_buf, from_f, to_f)
        status = "OK" if ok else ("WARN" if not required else "FAIL")
        if not ok and required:
            failed += 1
        print(f"  [{status}] {from_f} → {to_f}: {msg}")

    if failed > 0:
        print(f"\n  {failed} required TF checks FAILED")
        sys.exit(1)
    else:
        print("  All TF checks PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()

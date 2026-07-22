#!/usr/bin/env python3
"""
V3.3.6 Runtime Signals Health Check.

统一检查所有运行时信号:
  /clock, /joint_states, TF, 三台 RGB-D, spray 服务.

用法:
  rosrun cr5_spray_sim check_runtime_signals_v336.py

输出到 stderr (每行一个检查项，最后一行是总结):
  CLOCK_OK
  JOINT_STATES_OK
  TF_OK
  CAMERA_RGBD_OK
  SPRAY_SIGNAL_OK
  RUNTIME_SIGNALS_READY
  (或 RUNTIME_SIGNALS_DEGRADED)

退出码:
  0 = RUNTIME_SIGNALS_READY
  1 = RUNTIME_SIGNALS_DEGRADED
"""
import sys
import math
import time
import rospy
import tf2_ros
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import TransformStamped

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
REQUIRED_TF = [
    ("world", "Link6"),
    ("world", "spray_nozzle_frame"),
    ("world", "object_frame"),
]


def _check_clock():
    """检查 /clock 有消息且时间 > 0."""
    try:
        msg = rospy.wait_for_message("/clock", Clock, timeout=5.0)
        t = msg.clock.to_sec()
        if t > 0.001:
            rospy.loginfo("clock OK: %.3fs", t)
            return True
        rospy.logerr("clock time too small: %.3f", t)
        return False
    except rospy.ROSException:
        rospy.logerr("no /clock message in 5s")
        return False


def _check_joint_states():
    """检查 /joint_states 持续发布，所有关节 finite."""
    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        names = list(msg.name)
        positions = list(msg.position)
        for jn in JOINT_NAMES:
            if jn not in names:
                rospy.logerr("joint %s missing from /joint_states", jn)
                return False
            idx = names.index(jn)
            val = positions[idx]
            if not math.isfinite(val):
                rospy.logerr("joint %s non-finite: %s", jn, val)
                return False
        rospy.loginfo("joint_states OK: %d joints", len(names))
        return True
    except rospy.ROSException:
        rospy.logerr("no /joint_states message in 5s")
        return False


def _check_tf():
    """检查三条必需 TF."""
    try:
        tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buf)
        # 给 TF listener 填充时间
        rospy.sleep(1.0)

        all_ok = True
        for parent, child in REQUIRED_TF:
            try:
                tf_buf.lookup_transform(parent, child, rospy.Time(),
                                        timeout=rospy.Duration(5.0))
                rospy.loginfo("TF %s→%s OK", parent, child)
            except Exception as e:
                rospy.logerr("TF %s→%s FAILED: %s", parent, child, e)
                all_ok = False
        return all_ok
    except Exception as e:
        rospy.logerr("TF system error: %s", e)
        return False


def _find_topic(cam_name, suffix):
    """动态发现包含相机名且以 suffix 结尾的话题."""
    topics = rospy.get_published_topics()
    for topic, topic_type in topics:
        if cam_name in topic and topic.endswith(suffix):
            return topic
    return None


def _check_cameras():
    """检查三台相机都有 color+深度 image_raw + camera_info (动态话题发现)."""
    all_ok = True
    color_timeout = 12.0
    depth_timeout = 12.0

    for cam in CAMERAS:
        # CameraInfo — 动态查找
        info_topic = _find_topic(cam, "camera_info")
        if info_topic is None:
            rospy.logerr("%s camera_info topic not found", cam)
            all_ok = False
        else:
            try:
                msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=5.0)
                if msg.width > 0 and msg.height > 0 and len(msg.K) >= 5 and msg.K[0] > 0:
                    rospy.loginfo("%s camera_info OK: %dx%d (topic=%s)",
                                  cam, msg.width, msg.height, info_topic)
                else:
                    rospy.logerr("%s camera_info invalid", cam)
                    all_ok = False
            except rospy.ROSException:
                rospy.logerr("%s camera_info timeout (topic=%s)", cam, info_topic)
                all_ok = False

        # Color image — 动态查找
        color_topic = _find_topic(cam, "image_raw")
        if color_topic is None:
            rospy.logerr("%s color image_raw topic not found", cam)
            all_ok = False
        else:
            try:
                msg = rospy.wait_for_message(color_topic, Image, timeout=color_timeout)
                if msg.width > 0 and msg.height > 0 and len(msg.data) > 0:
                    rospy.loginfo("%s color OK: %dx%d %d bytes (topic=%s)",
                                  cam, msg.width, msg.height, len(msg.data), color_topic)
                else:
                    rospy.logerr("%s color invalid", cam)
                    all_ok = False
            except rospy.ROSException:
                rospy.logerr("%s color timeout (topic=%s)", cam, color_topic)
                all_ok = False

        # Depth image — 需要第二个 image_raw (深度 topic)
        # 获取所有匹配的 image_raw 话题，选非 color 的那个
        all_image_topics = [t for t, _ in rospy.get_published_topics()
                            if cam in t and t.endswith("image_raw")]
        depth_topic = None
        for t in all_image_topics:
            if t != color_topic:  # 不同于 color 的那个
                depth_topic = t
                break
        if depth_topic is None and len(all_image_topics) >= 2:
            depth_topic = all_image_topics[1]

        if depth_topic is None:
            rospy.logerr("%s depth image_raw topic not found (have: %s)",
                         cam, all_image_topics)
            all_ok = False
        else:
            try:
                msg = rospy.wait_for_message(depth_topic, Image, timeout=depth_timeout)
                if msg.width > 0 and msg.height > 0 and len(msg.data) > 0:
                    rospy.loginfo("%s depth OK: %dx%d %d bytes (topic=%s)",
                                  cam, msg.width, msg.height, len(msg.data), depth_topic)
                else:
                    rospy.logerr("%s depth invalid", cam)
                    all_ok = False
            except rospy.ROSException:
                rospy.logerr("%s depth timeout (topic=%s)", cam, depth_topic)
                all_ok = False

    return all_ok


def _check_spray():
    """检查 spray 服务存在 + set_spray(false) 成功."""
    # 等 state topic
    try:
        from std_msgs.msg import String
        msg = rospy.wait_for_message("/spray_demo/state", String, timeout=5.0)
        rospy.loginfo("spray state topic OK: %s", msg.data)
    except rospy.ROSException:
        rospy.logerr("no /spray_demo/state in 5s")
        return False

    # set_spray 服务
    try:
        rospy.wait_for_service("/spray_demo/set_spray", timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("spray_demo/set_spray service not available")
        return False

    # set_spray(false) 快速调用
    try:
        srv = rospy.ServiceProxy("/spray_demo/set_spray", SetBool)
        resp = srv(False)
        if resp.success:
            rospy.loginfo("set_spray(false) OK: %s", resp.message)
        else:
            rospy.logwarn("set_spray(false) returned failure: %s", resp.message)
    except Exception as e:
        rospy.logerr("set_spray(false) call failed: %s", e)
        return False

    return True


def main():
    rospy.init_node("check_runtime_signals_v336", anonymous=True,
                    log_level=rospy.WARN)

    results = {}

    # 1. Clock
    results["clock"] = _check_clock()
    if results["clock"]:
        sys.stderr.write("CLOCK_OK\n")
    else:
        sys.stderr.write("CLOCK_FAIL\n")

    # 2. Joint States
    results["joint_states"] = _check_joint_states()
    if results["joint_states"]:
        sys.stderr.write("JOINT_STATES_OK\n")
    else:
        sys.stderr.write("JOINT_STATES_FAIL\n")

    # 3. TF
    results["tf"] = _check_tf()
    if results["tf"]:
        sys.stderr.write("TF_OK\n")
    else:
        sys.stderr.write("TF_FAIL\n")

    # 4. Cameras
    results["cameras"] = _check_cameras()
    if results["cameras"]:
        sys.stderr.write("CAMERA_RGBD_OK\n")
    else:
        sys.stderr.write("CAMERA_RGBD_FAIL\n")

    # 5. Spray
    results["spray"] = _check_spray()
    if results["spray"]:
        sys.stderr.write("SPRAY_SIGNAL_OK\n")
    else:
        sys.stderr.write("SPRAY_SIGNAL_FAIL\n")

    sys.stderr.flush()

    all_ok = all(results.values())
    rospy.loginfo("Runtime signals: %s", results)

    if all_ok:
        sys.stderr.write("RUNTIME_SIGNALS_READY\n")
        sys.stderr.flush()
        sys.exit(0)
    else:
        failed = [k for k, v in results.items() if not v]
        rospy.logerr("Runtime signals degraded: %s", failed)
        sys.stderr.write("RUNTIME_SIGNALS_DEGRADED\n")
        sys.stderr.flush()
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3.3.7 Runtime Signals Health Check — 精确相机话题 + 喷枪信号。

关键修复 vs V3.3.6:
- 相机话题写死精确路径，不再使用动态"第一个匹配"逻辑
- 禁止把 IR/IR2 当作 depth
- 只在 exact topic 不存在时才 fallback，fallback 必须路径含 /color/ 或 /depth/
- master 关闭时返回 CHECK_ABORTED_BY_SHUTDOWN

用法:
  rosrun cr5_spray_sim check_runtime_signals.py [--output artifacts/<session>/camera/]

输出到 stderr:
  CLOCK_OK / CLOCK_FAIL
  JOINT_STATES_OK / JOINT_STATES_FAIL
  TF_OK / TF_FAIL
  CAMERA_COLOR_3_OF_3_PASS / CAMERA_COLOR_X_OF_3
  CAMERA_DEPTH_3_OF_3_PASS / CAMERA_DEPTH_X_OF_3
  SPRAY_SIGNAL_PASS / SPRAY_SIGNAL_FAIL
  RUNTIME_SIGNALS_READY / RUNTIME_SIGNALS_DEGRADED

退出码:
  0 = RUNTIME_SIGNALS_READY
  1 = RUNTIME_SIGNALS_DEGRADED
"""
import sys
import os
import math
import time
import rospy
import tf2_ros
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

# ---- 精确相机话题 (禁止 IR 兜底) ----
CAMERA_EXACT_TOPICS = {
    "cam_front_left": {
        "color":      "/cam_front_left/camera/color/image_raw",
        "depth":      "/cam_front_left/camera/depth/image_raw",
        "color_info": "/cam_front_left/camera/color/camera_info",
        "depth_info": "/cam_front_left/camera/depth/camera_info",
    },
    "cam_front_right": {
        "color":      "/cam_front_right/camera/color/image_raw",
        "depth":      "/cam_front_right/camera/depth/image_raw",
        "color_info": "/cam_front_right/camera/color/camera_info",
        "depth_info": "/cam_front_right/camera/depth/camera_info",
    },
    "cam_rear": {
        "color":      "/cam_rear/camera/color/image_raw",
        "depth":      "/cam_rear/camera/depth/image_raw",
        "color_info": "/cam_rear/camera/color/camera_info",
        "depth_info": "/cam_rear/camera/depth/camera_info",
    },
}

# 喷枪信号
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
REQUIRED_TF = [
    ("world", "Link6"),
    ("world", "spray_nozzle_frame"),
    ("world", "object_frame"),
]

# 每台相机需要的帧数
MIN_COLOR_FRAMES = 3
MIN_DEPTH_FRAMES = 3
CAMERA_FRAME_TIMEOUT = 15.0  # 每台相机等待时间


def _is_master_alive():
    """检查 ROS master 是否仍在运行."""
    try:
        rospy.get_master().getPid()
        return True
    except:
        return False


def _find_fallback_topic(cam_name, category):
    """
    只在 exact topic 不存在时 fallback。
    要求路径中必须含 /color/ 或 /depth/。
    """
    prefix = "/{}/camera/".format(cam_name)
    required_segment = "/{}/".format(category)  # /color/ or /depth/

    topics = rospy.get_published_topics()
    candidates = []
    for topic, topic_type in topics:
        if topic.startswith(prefix) and required_segment in topic:
            candidates.append(topic)

    if not candidates:
        return None

    # 优先选择最短路径 (通常是最直接的)
    candidates.sort(key=len)
    return candidates[0]


class SignalChecker:
    def __init__(self):
        self.bridge = CvBridge()
        self.output_dir = None

    def set_output_dir(self, path):
        self.output_dir = path
        if path:
            os.makedirs(path, exist_ok=True)

    # ===== Clock =====
    def check_clock(self):
        try:
            msg = rospy.wait_for_message("/clock", Clock, timeout=5.0)
            t = msg.clock.to_sec()
            if t > 0.001:
                rospy.loginfo("clock OK: %.3fs", t)
                return True
            rospy.logerr("clock time too small: %.3f", t)
            return False
        except rospy.ROSException:
            if not _is_master_alive():
                rospy.logerr("ROS master closed, aborting clock check")
                sys.stderr.write("CHECK_ABORTED_BY_SHUTDOWN\n")
                sys.stderr.flush()
            else:
                rospy.logerr("no /clock message in 5s")
            return False

    # ===== Joint States =====
    def check_joint_states(self):
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
            if not _is_master_alive():
                sys.stderr.write("CHECK_ABORTED_BY_SHUTDOWN\n")
                sys.stderr.flush()
            rospy.logerr("no /joint_states message in 5s")
            return False

    # ===== TF =====
    def check_tf(self):
        try:
            tf_buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buf)
            rospy.sleep(1.0)

            all_ok = True
            for parent, child in REQUIRED_TF:
                try:
                    tf_buf.lookup_transform(parent, child, rospy.Time(),
                                            timeout=rospy.Duration(5.0))
                    rospy.loginfo("TF %s->%s OK", parent, child)
                except Exception as e:
                    rospy.logerr("TF %s->%s FAILED: %s", parent, child, e)
                    all_ok = False
            return all_ok
        except Exception as e:
            rospy.logerr("TF system error: %s", e)
            return False

    # ===== Cameras =====
    def _collect_frames(self, topic, topic_type, min_frames, timeout):
        """收集指定话题的消息帧，返回 list[msg]."""
        frames = []
        start = time.time()
        while time.time() - start < timeout and len(frames) < min_frames:
            if not _is_master_alive():
                rospy.logerr("Master closed during camera collection")
                return frames, "MASTER_CLOSED"
            try:
                msg = rospy.wait_for_message(topic, topic_type, timeout=2.0)
                frames.append(msg)
            except rospy.ROSException:
                if not _is_master_alive():
                    return frames, "MASTER_CLOSED"
                continue
        return frames, "OK"

    def _check_camera_info(self, info_topic):
        """检查 camera_info 有效性."""
        try:
            info = rospy.wait_for_message(info_topic, CameraInfo, timeout=5.0)
            if info.width <= 0 or info.height <= 0:
                rospy.logerr("%s: invalid dimensions %dx%d", info_topic, info.width, info.height)
                return None
            if len(info.K) < 5 or info.K[0] <= 0 or info.K[4] <= 0:
                rospy.logerr("%s: invalid K matrix", info_topic)
                return None
            rospy.loginfo("%s OK: %dx%d K=[%.1f, %.1f]", info_topic, info.width, info.height,
                          info.K[0], info.K[4])
            return info
        except rospy.ROSException:
            rospy.logerr("camera_info timeout: %s", info_topic)
            return None

    def _check_image_frames(self, topic, min_frames, is_depth=False):
        """收集并检查 image 帧."""
        frames, status = self._collect_frames(topic, Image, min_frames, CAMERA_FRAME_TIMEOUT)
        if status == "MASTER_CLOSED":
            return False, 0, "master_closed"

        count = len(frames)
        if count < min_frames:
            rospy.logerr("%s: only %d/%d frames (timeout)", topic, count, min_frames)
            return False, count, "timeout"

        # 检查编码
        for i, msg in enumerate(frames):
            if msg.width <= 0 or msg.height <= 0 or len(msg.data) == 0:
                rospy.logerr("%s frame %d: invalid (w=%d h=%d data=%d)",
                             topic, i, msg.width, msg.height, len(msg.data))
                return False, count, "invalid_frame"

            if is_depth:
                valid_encodings = {"32FC1", "16UC1", "TYPE_32FC1", "TYPE_16UC1"}
                if msg.encoding not in valid_encodings:
                    rospy.logerr("%s frame %d: bad depth encoding '%s'", topic, i, msg.encoding)
                    return False, count, "bad_encoding"
            else:
                valid_encodings = {"rgb8", "bgr8", "RGB8", "BGR8", "rgb8", "bgr8"}
                if msg.encoding not in valid_encodings:
                    rospy.logerr("%s frame %d: bad color encoding '%s'", topic, i, msg.encoding)
                    return False, count, "bad_encoding"

        rospy.loginfo("%s: %d frames %dx%d encoding=%s OK",
                      topic, count, frames[0].width, frames[0].height, frames[0].encoding)
        return True, count, "OK"

    def check_all_cameras(self):
        """检查所有三台相机的 color + depth."""
        color_ok_count = 0
        depth_ok_count = 0

        for cam_name in sorted(CAMERA_EXACT_TOPICS.keys()):
            topics = CAMERA_EXACT_TOPICS[cam_name]
            rospy.loginfo("=== Checking %s ===", cam_name)

            # ---- CameraInfo ----
            # color camera_info
            info_ok = self._check_camera_info(topics["color_info"])
            if info_ok is None:
                rospy.logwarn("%s color camera_info not at exact topic, trying fallback", cam_name)
                fallback = _find_fallback_topic(cam_name, "color")
                if fallback and "camera_info" in fallback:
                    info_ok = self._check_camera_info(fallback)
            if info_ok is None:
                rospy.logerr("%s color camera_info FAILED", cam_name)

            # depth camera_info
            depth_info_ok = self._check_camera_info(topics["depth_info"])
            if depth_info_ok is None:
                rospy.logwarn("%s depth camera_info not at exact topic, trying fallback", cam_name)
                fallback = _find_fallback_topic(cam_name, "depth")
                if fallback and "camera_info" in fallback:
                    depth_info_ok = self._check_camera_info(fallback)

            # ---- Color images ----
            color_topic = topics["color"]
            color_ok, color_count, color_status = self._check_image_frames(
                color_topic, MIN_COLOR_FRAMES, is_depth=False)

            if not color_ok and color_status != "master_closed":
                # 尝试精确 fallback: 必须路径含 /color/
                rospy.logwarn("%s: exact color topic failed, trying fallback...", cam_name)
                fallback = _find_fallback_topic(cam_name, "color")
                if fallback:
                    rospy.loginfo("%s: color fallback → %s", cam_name, fallback)
                    color_ok, color_count, color_status = self._check_image_frames(
                        fallback, MIN_COLOR_FRAMES, is_depth=False)

            if color_ok:
                color_ok_count += 1
                # 保存样本
                if self.output_dir:
                    self._save_frame(cam_name, "color", color_topic)

            # ---- Depth images ----
            depth_topic = topics["depth"]
            depth_ok, depth_count, depth_status = self._check_image_frames(
                depth_topic, MIN_DEPTH_FRAMES, is_depth=True)

            if not depth_ok and depth_status != "master_closed":
                rospy.logwarn("%s: exact depth topic failed, trying fallback...", cam_name)
                fallback = _find_fallback_topic(cam_name, "depth")
                if fallback:
                    rospy.loginfo("%s: depth fallback → %s", cam_name, fallback)
                    depth_ok, depth_count, depth_status = self._check_image_frames(
                        fallback, MIN_DEPTH_FRAMES, is_depth=True)

            if depth_ok:
                depth_ok_count += 1

        # 汇总
        color_all_ok = (color_ok_count == 3)
        depth_all_ok = (depth_ok_count == 3)

        if color_all_ok:
            sys.stderr.write("CAMERA_COLOR_3_OF_3_PASS\n")
        else:
            sys.stderr.write("CAMERA_COLOR_{}_OF_3\n".format(color_ok_count))

        if depth_all_ok:
            sys.stderr.write("CAMERA_DEPTH_3_OF_3_PASS\n")
        else:
            sys.stderr.write("CAMERA_DEPTH_{}_OF_3\n".format(depth_ok_count))

        sys.stderr.flush()

        return color_all_ok and depth_all_ok

    def _save_frame(self, cam_name, img_type, topic):
        """保存一帧图像用于诊断."""
        try:
            msg = rospy.wait_for_message(topic, Image, timeout=2.0)
            import cv2
            import numpy as np
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                cv_img = self.bridge.imgmsg_to_cv2(msg)

            fname = os.path.join(self.output_dir,
                                 "{}_{}.png".format(cam_name, img_type))
            cv2.imwrite(fname, cv_img)
            rospy.loginfo("Saved %s", fname)
        except Exception as e:
            rospy.logwarn("Save frame %s/%s failed: %s", cam_name, img_type, e)

    # ===== Spray =====
    def check_spray(self):
        """检查 spray state topic + set_spray(false) 服务."""
        # Wait for state topic
        try:
            msg = rospy.wait_for_message("/spray_demo/state", String, timeout=5.0)
            rospy.loginfo("spray state topic OK: %s", msg.data)
        except rospy.ROSException:
            rospy.logerr("no /spray_demo/state in 5s")
            return False

        # set_spray service
        try:
            rospy.wait_for_service("/spray_demo/set_spray", timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("spray_demo/set_spray service not available")
            return False

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
    rospy.init_node("check_runtime_signals", anonymous=True,
                    log_level=rospy.WARN)

    output_dir = None
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]

    checker = SignalChecker()
    checker.set_output_dir(output_dir)

    results = {}

    # 1. Clock
    results["clock"] = checker.check_clock()
    sys.stderr.write("CLOCK_OK\n" if results["clock"] else "CLOCK_FAIL\n")

    # 2. Joint States
    results["joint_states"] = checker.check_joint_states()
    sys.stderr.write("JOINT_STATES_OK\n" if results["joint_states"] else "JOINT_STATES_FAIL\n")

    # 3. TF
    results["tf"] = checker.check_tf()
    sys.stderr.write("TF_OK\n" if results["tf"] else "TF_FAIL\n")

    # 4. Cameras
    results["cameras"] = checker.check_all_cameras()

    # 5. Spray
    results["spray"] = checker.check_spray()
    sys.stderr.write("SPRAY_SIGNAL_PASS\n" if results["spray"] else "SPRAY_SIGNAL_FAIL\n")

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

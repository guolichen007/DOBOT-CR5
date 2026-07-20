#!/usr/bin/env python3
"""
D455-like RGB-D Simulator Smoke Test
验证相机话题、CameraInfo、frame_id 和数据有效性。
失败时返回非零退出码。

Usage: d455_like_smoke_test.py [camera_name]
  Default camera_name: test_camera
"""
import sys
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo

CAMERA_NAME = sys.argv[1] if len(sys.argv) > 1 else "test_camera"
CAMERA_NS = "/" + CAMERA_NAME

REQUIRED_COLOR_TOPIC = CAMERA_NS + "/camera/color/image_raw"
REQUIRED_COLOR_INFO = CAMERA_NS + "/camera/color/camera_info"
REQUIRED_DEPTH_TOPIC = CAMERA_NS + "/camera/depth/image_raw"
REQUIRED_DEPTH_INFO = CAMERA_NS + "/camera/depth/camera_info"

TIMEOUT_S = 30
MIN_HZ = 1.0


class SmokeTest:
    def __init__(self):
        self.failures = []
        self.color_msgs = []
        self.color_info_msgs = []
        self.depth_msgs = []
        self.depth_info_msgs = []

    def color_cb(self, msg):
        self.color_msgs.append(msg)

    def color_info_cb(self, msg):
        self.color_info_msgs.append(msg)

    def depth_cb(self, msg):
        self.depth_msgs.append(msg)

    def depth_info_cb(self, msg):
        self.depth_info_msgs.append(msg)

    def check(self, condition, desc):
        if not condition:
            self.failures.append(desc)
            rospy.logerr("FAIL: %s", desc)
        else:
            rospy.loginfo("PASS: %s", desc)

    def run(self):
        rospy.init_node("d455_like_smoke_test", anonymous=True)
        rospy.loginfo("D455-like smoke test starting...")

        # Subscribe
        rospy.Subscriber(REQUIRED_COLOR_TOPIC, Image, self.color_cb)
        rospy.Subscriber(REQUIRED_COLOR_INFO, CameraInfo, self.color_info_cb)
        rospy.Subscriber(REQUIRED_DEPTH_TOPIC, Image, self.depth_cb)
        rospy.Subscriber(REQUIRED_DEPTH_INFO, CameraInfo, self.depth_info_cb)

        # Wait for data
        start = time.time()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if (len(self.color_msgs) >= 3 and len(self.depth_msgs) >= 3
                    and len(self.color_info_msgs) >= 1 and len(self.depth_info_msgs) >= 1):
                break
            if time.time() - start > TIMEOUT_S:
                break
            rate.sleep()

        elapsed = time.time() - start

        # --- Checks ---
        rospy.loginfo("=== D455-like Smoke Test Results ===")

        # 1. Color image topic
        self.check(len(self.color_msgs) > 0,
                   "Color image topic {}".format(REQUIRED_COLOR_TOPIC))

        # 2. Color CameraInfo topic
        self.check(len(self.color_info_msgs) > 0,
                   "Color CameraInfo topic {}".format(REQUIRED_COLOR_INFO))

        # 3. Depth image topic
        self.check(len(self.depth_msgs) > 0,
                   "Depth image topic {}".format(REQUIRED_DEPTH_TOPIC))

        # 4. Depth CameraInfo topic
        self.check(len(self.depth_info_msgs) > 0,
                   "Depth CameraInfo topic {}".format(REQUIRED_DEPTH_INFO))

        expected_frame = CAMERA_NAME + "color"
        # 5. Color frame_id
        if self.color_msgs:
            fid = self.color_msgs[0].header.frame_id
            self.check(fid == expected_frame,
                       "Color frame_id is '{}' (expected '{}')".format(fid, expected_frame))

        # 6. Depth frame_id (plugin uses COLOR camera name for depth too)
        if self.depth_msgs:
            fid = self.depth_msgs[0].header.frame_id
            self.check(fid == expected_frame,
                       "Depth frame_id is '{}' (expected '{}')".format(fid, expected_frame))

        # 7. Color image has valid dimensions
        if self.color_msgs:
            msg = self.color_msgs[0]
            self.check(msg.width > 0 and msg.height > 0,
                       "Color image dimensions {}x{}".format(msg.width, msg.height))
            self.check(msg.encoding == "rgb8",
                       "Color encoding is '{}' (expected 'rgb8')".format(msg.encoding))

        # 8. Depth image has valid dimensions and encoding
        if self.depth_msgs:
            msg = self.depth_msgs[0]
            self.check(msg.width > 0 and msg.height > 0,
                       "Depth image dimensions {}x{}".format(msg.width, msg.height))
            self.check(msg.encoding == "16UC1",
                       "Depth encoding is '{}' (expected '16UC1')".format(msg.encoding))
            # Check for finite depth values
            if msg.encoding == "16UC1":
                import struct
                data = msg.data
                n_zeros = sum(1 for i in range(0, len(data), 2)
                              if struct.unpack_from('<H', data, i)[0] == 0)
                n_total = len(data) // 2
                valid_ratio = 1.0 - n_zeros / max(n_total, 1)
                self.check(valid_ratio > 0.0,
                           "Depth valid pixel ratio {:.2%} > 0%".format(valid_ratio))
                rospy.loginfo("  Depth: {} total px, {} zero (invalid), {:.1%} valid"
                              .format(n_total, n_zeros, valid_ratio))

        # 9. CameraInfo K matrix is non-zero
        if self.color_info_msgs:
            K = self.color_info_msgs[0].K
            self.check(K[0] > 0.0 and K[4] > 0.0,
                       "Color CameraInfo K[0]={:.1f}, K[4]={:.1f} (both > 0)".format(K[0], K[4]))
            self.check(K[2] > 0.0 and K[5] > 0.0,
                       "Color CameraInfo cx={:.1f}, cy={:.1f} (both > 0)".format(K[2], K[5]))

        # 10. Frequency check (approximate)
        if self.color_msgs and len(self.color_msgs) >= 2:
            t0 = self.color_msgs[0].header.stamp.to_sec()
            t1 = self.color_msgs[-1].header.stamp.to_sec()
            if t1 > t0:
                hz = (len(self.color_msgs) - 1) / (t1 - t0)
                self.check(hz >= MIN_HZ,
                           "Color Hz ~{:.1f} >= {:.1f}".format(hz, MIN_HZ))

        if self.depth_msgs and len(self.depth_msgs) >= 2:
            t0 = self.depth_msgs[0].header.stamp.to_sec()
            t1 = self.depth_msgs[-1].header.stamp.to_sec()
            if t1 > t0:
                hz = (len(self.depth_msgs) - 1) / (t1 - t0)
                self.check(hz >= MIN_HZ,
                           "Depth Hz ~{:.1f} >= {:.1f}".format(hz, MIN_HZ))

        # Summary
        rospy.loginfo("=== Summary: {} failures ===".format(len(self.failures)))
        for f in self.failures:
            rospy.logerr("  FAIL: %s", f)

        if self.failures:
            rospy.logerr("SMOKE TEST FAILED with {} failure(s)".format(len(self.failures)))
            sys.exit(1)
        else:
            rospy.loginfo("SMOKE TEST PASSED")
            sys.exit(0)


if __name__ == "__main__":
    SmokeTest().run()

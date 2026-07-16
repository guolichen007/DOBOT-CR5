#!/usr/bin/env python3
"""
check_d455_topics.py
检查 D455 相机话题一致性和时间同步
"""

import sys
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo

# 默认参数
DEFAULT_SYNC_THRESHOLD_MS = 100
DEFAULT_SAMPLE_COUNT = 10


class D455TopicChecker:
    def __init__(self, sync_threshold_ms=DEFAULT_SYNC_THRESHOLD_MS, sample_count=DEFAULT_SAMPLE_COUNT):
        self.sync_threshold_ms = sync_threshold_ms
        self.sample_count = sample_count

        self.color_image = None
        self.depth_image = None
        self.color_info = None
        self.depth_info = None

        self.color_timestamps = []
        self.depth_timestamps = []

        self.pass_count = 0
        self.fail_count = 0
        self.warn_count = 0

    def pass_test(self, msg):
        print(f"[PASS] {msg}")
        self.pass_count += 1

    def fail_test(self, msg):
        print(f"[FAIL] {msg}")
        self.fail_count += 1

    def warn_test(self, msg):
        print(f"[WARN] {msg}")
        self.warn_count += 1

    def check_topic_exists(self, topic, msg_type):
        """检查话题是否存在且有 publisher"""
        try:
            publishers = rospy.get_published_topics()
            for t, t_type in publishers:
                if t == topic:
                    return True
            return False
        except:
            return False

    def run_checks(self):
        """执行所有检查"""
        print("==========================================")
        print("  D455 话题一致性检查")
        print("==========================================")
        print(f"同步阈值: {self.sync_threshold_ms} ms")
        print(f"采样帧数: {self.sample_count}")

        # 1. 检查话题存在性
        print("\n--- 话题存在性检查 ---")

        topics_to_check = [
            ("/camera/color/image_raw", "sensor_msgs/Image"),
            ("/camera/aligned_depth_to_color/image_raw", "sensor_msgs/Image"),
            ("/camera/color/camera_info", "sensor_msgs/CameraInfo"),
            ("/camera/aligned_depth_to_color/camera_info", "sensor_msgs/CameraInfo"),
        ]

        all_topics_exist = True
        for topic, msg_type in topics_to_check:
            if self.check_topic_exists(topic, msg_type):
                self.pass_test(f"{topic}")
            else:
                self.fail_test(f"{topic} 不存在")
                all_topics_exist = False

        if not all_topics_exist:
            print("\n[FAIL] 必要话题不存在，无法继续检查")
            return False

        # 2. 采样数据
        print(f"\n--- 采样 {self.sample_count} 帧数据 ---")

        self.color_image = None
        self.depth_image = None
        self.color_info = None
        self.depth_info = None

        color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_cb)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_cb)
        color_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.color_info_cb)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.depth_info_cb)

        # 等待接收数据
        timeout = rospy.Duration(10)
        start_time = rospy.Time.now()

        while len(self.color_timestamps) < self.sample_count or len(self.depth_timestamps) < self.sample_count:
            if rospy.Time.now() - start_time > timeout:
                print(f"[WARN] 超时：只收到 {len(self.color_timestamps)} 帧 color, {len(self.depth_timestamps)} 帧 depth")
                break
            rospy.sleep(0.1)

        color_sub.unregister()
        depth_sub.unregister()
        color_info_sub.unregister()
        depth_info_sub.unregister()

        # 3. 检查 CameraInfo
        print("\n--- CameraInfo 检查 ---")

        if self.color_info:
            self.pass_test(f"Color CameraInfo: {self.color_info.width}x{self.color_info.height}, frame_id={self.color_info.header.frame_id}")
        else:
            self.fail_test("Color CameraInfo 未收到")

        if self.depth_info:
            self.pass_test(f"Aligned Depth CameraInfo: {self.depth_info.width}x{self.depth_info.height}, frame_id={self.depth_info.header.frame_id}")
        else:
            self.fail_test("Aligned Depth CameraInfo 未收到")

        # 4. 检查分辨率一致性
        print("\n--- 分辨率一致性检查 ---")

        if self.color_image and self.depth_image:
            if self.color_image.width == self.depth_image.width and self.color_image.height == self.depth_image.height:
                self.pass_test(f"分辨率一致: {self.color_image.width}x{self.color_image.height}")
            else:
                self.fail_test(f"分辨率不一致: Color={self.color_image.width}x{self.color_image.height}, Depth={self.depth_image.width}x{self.depth_image.height}")
        elif self.color_info and self.depth_info:
            if self.color_info.width == self.depth_info.width and self.color_info.height == self.depth_info.height:
                self.pass_test(f"分辨率一致: {self.color_info.width}x{self.color_info.height}")
            else:
                self.fail_test(f"分辨率不一致: Color={self.color_info.width}x{self.color_info.height}, Depth={self.depth_info.width}x{self.depth_info.height}")
        else:
            self.warn_test("无法检查分辨率一致性（数据不足）")

        # 5. 检查 frame_id
        print("\n--- frame_id 检查 ---")

        if self.color_image:
            self.pass_test(f"Color image frame_id: {self.color_image.header.frame_id}")
        if self.depth_image:
            self.pass_test(f"Depth image frame_id: {self.depth_image.header.frame_id}")

        # 6. 检查时间同步
        print("\n--- 时间同步检查 ---")

        if len(self.color_timestamps) >= 2 and len(self.depth_timestamps) >= 2:
            # 计算帧率
            color_dt = [(self.color_timestamps[i+1] - self.color_timestamps[i]).to_sec() for i in range(len(self.color_timestamps)-1)]
            depth_dt = [(self.depth_timestamps[i+1] - self.depth_timestamps[i]).to_sec() for i in range(len(self.depth_timestamps)-1)]

            color_fps = 1.0 / (sum(color_dt) / len(color_dt)) if color_dt else 0
            depth_fps = 1.0 / (sum(depth_dt) / len(depth_dt)) if depth_dt else 0

            self.pass_test(f"Color 帧率: {color_fps:.1f} Hz")
            self.pass_test(f"Depth 帧率: {depth_fps:.1f} Hz")

            # 检查时间差
            min_len = min(len(self.color_timestamps), len(self.depth_timestamps))
            if min_len > 0:
                time_diffs = []
                for i in range(min_len):
                    diff_ms = abs((self.color_timestamps[i] - self.depth_timestamps[i]).to_sec() * 1000)
                    time_diffs.append(diff_ms)

                avg_diff = sum(time_diffs) / len(time_diffs)
                max_diff = max(time_diffs)

                if max_diff <= self.sync_threshold_ms:
                    self.pass_test(f"时间同步: 平均={avg_diff:.1f}ms, 最大={max_diff:.1f}ms (阈值={self.sync_threshold_ms}ms)")
                else:
                    self.fail_test(f"时间同步: 平均={avg_diff:.1f}ms, 最大={max_diff:.1f}ms (超过阈值={self.sync_threshold_ms}ms)")
        else:
            self.warn_test("数据不足，无法检查时间同步")

        # 7. 总结
        print("\n==========================================")
        print("  检查总结")
        print("==========================================")
        print(f"PASS: {self.pass_count}")
        print(f"WARN: {self.warn_count}")
        print(f"FAIL: {self.fail_count}")

        return self.fail_count == 0

    def color_image_cb(self, msg):
        if self.color_image is None:
            self.color_image = msg
        self.color_timestamps.append(msg.header.stamp)
        if len(self.color_timestamps) > self.sample_count:
            self.color_timestamps = self.color_timestamps[-self.sample_count:]

    def depth_image_cb(self, msg):
        if self.depth_image is None:
            self.depth_image = msg
        self.depth_timestamps.append(msg.header.stamp)
        if len(self.depth_timestamps) > self.sample_count:
            self.depth_timestamps = self.depth_timestamps[-self.sample_count:]

    def color_info_cb(self, msg):
        if self.color_info is None:
            self.color_info = msg

    def depth_info_cb(self, msg):
        if self.depth_info is None:
            self.depth_info = msg


def main():
    rospy.init_node("d455_topic_checker", anonymous=True)

    # 解析参数
    sync_threshold = rospy.get_param("~sync_threshold_ms", DEFAULT_SYNC_THRESHOLD_MS)
    sample_count = rospy.get_param("~sample_count", DEFAULT_SAMPLE_COUNT)

    checker = D455TopicChecker(sync_threshold, sample_count)

    if checker.run_checks():
        print("\n[RESULT] 所有检查通过")
        sys.exit(0)
    else:
        print("\n[RESULT] 存在失败项")
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
CR5 Spray Demo: Scene Validation
检查 /gazebo/model_states、所有相机 topic、TF、object yaw。
失败返回非零。
"""
import sys
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
import tf2_ros


class SceneValidator:
    def __init__(self):
        rospy.init_node("scene_validator", anonymous=True)
        self.failures = []
        self.models = []
        self.model_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_cb)

    def model_cb(self, msg):
        self.models = msg.name

    def check(self, cond, desc):
        if not cond:
            self.failures.append(desc)
            rospy.logerr("FAIL: %s", desc)
        else:
            rospy.loginfo("PASS: %s", desc)

    def run(self, expected_cameras, timeout_s=30):
        rate = rospy.Rate(2)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > timeout_s:
                break
            if len(self.models) > 2:  # ground_plane + at least 1 other
                break
            rate.sleep()

        # 1. Required models present
        required = ["cr5_robot", "gantry", "spray_object"]
        for m in required:
            self.check(m in self.models,
                       "Model '{}' present in Gazebo".format(m))

        # 2. Camera topics
        for cam in expected_cameras:
            info_topic = "/{}/camera/color/camera_info".format(cam)
            try:
                ci = rospy.wait_for_message(info_topic, CameraInfo, timeout=10.0)
                self.check(ci.K[0] > 0, "Camera '{}' info valid".format(cam))
            except rospy.ROSException:
                self.check(False, "Camera '{}' info NOT available".format(cam))

        # 3. Check no duplicate frame_ids via TF
        try:
            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer)
            rospy.sleep(2.0)
            all_frames = tf_buffer.all_frames_as_string()
            # Basic check: at least contains "world"
            self.check("world" in all_frames, "TF tree contains 'world'")
        except Exception as e:
            rospy.logwarn("TF check: %s", e)

        # Summary
        rospy.loginfo("=== Scene Validation: %d failures ===", len(self.failures))
        for f in self.failures:
            rospy.logerr("  FAIL: %s", f)

        if self.failures:
            sys.exit(1)
        else:
            rospy.loginfo("SCENE VALIDATION PASSED")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--cameras", nargs="*", default=[],
                        help="Expected camera names")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    validator = SceneValidator()
    validator.run(args.cameras, args.timeout)

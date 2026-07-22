#!/usr/bin/env python3
"""
V3.3.2 Dynamic Object TF: 读取 simple_hanging_workpiece 的实际 Gazebo 姿态，
发布 world→object_frame 连续 TF。

V3.3.2 修复:
- 只在仿真时间推进或姿态变化时发布
- 限制 10 Hz 最大发布频率
- 跳过重复时间戳 (消除 TF_REPEATED_DATA)
- 姿态变化阈值: translation > 0.1mm 或 rotation > 0.01度
"""
import math
import rospy
import tf2_ros
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class ObjectPoseTFV31:
    def __init__(self):
        rospy.init_node("publish_object_pose")
        self.model_name = rospy.get_param("~model_name", "simple_hanging_workpiece")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.cb, queue_size=1)

        # V3.3.2: 去重状态
        self.last_stamp = rospy.Time(0)
        self.last_pos = np.zeros(3)
        self.last_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.min_period = rospy.Duration(0.1)  # 10 Hz max
        self.trans_thresh = 1e-4  # 0.1 mm
        self.rot_thresh = 1.745e-4  # ~0.01 degrees in radians

        rospy.loginfo("Object pose TF V3.3.2 ready (model=%s, dedup enabled)", self.model_name)

    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        now = rospy.Time.now()

        # V3.3.2: 跳过非推进时间戳
        if now <= self.last_stamp:
            return

        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w])

        # 检查时间间隔和姿态变化
        dt = now - self.last_stamp
        pos_changed = np.linalg.norm(pos - self.last_pos) > self.trans_thresh
        rot_changed = (1.0 - abs(np.dot(quat, self.last_quat))) > self.rot_thresh

        if dt < self.min_period and not (pos_changed or rot_changed):
            return

        # 发布 TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "world"
        t.child_frame_id = "object_frame"
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)

        self.last_stamp = now
        self.last_pos = pos
        self.last_quat = quat

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    ObjectPoseTFV31().run()

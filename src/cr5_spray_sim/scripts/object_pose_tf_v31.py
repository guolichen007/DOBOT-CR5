#!/usr/bin/env python3
"""
V3.1 Dynamic Object TF: 读取 simple_hanging_workpiece 的实际 Gazebo 姿态，
发布 world→object_frame 连续 TF。跟随 yaw 旋转。
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class ObjectPoseTFV31:
    def __init__(self):
        rospy.init_node("object_pose_tf_v31")
        self.model_name = rospy.get_param("~model_name", "simple_hanging_workpiece")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.cb, queue_size=1)
        rospy.loginfo("Object pose TF V3.1 ready (model=%s)", self.model_name)

    def cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        pose = msg.pose[idx]
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "object_frame"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    ObjectPoseTFV31().run()

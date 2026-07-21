#!/usr/bin/env python3
"""
V2 Dynamic Object TF: reads spray_object pose from /gazebo/model_states,
publishes world→object_frame continuously. Follows yaw changes.
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class ObjectPoseTF:
    def __init__(self):
        rospy.init_node("object_pose_tf_v2")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.cb, queue_size=1)
        rospy.loginfo("Object pose TF publisher ready")

    def cb(self, msg):
        try:
            idx = msg.name.index("spray_object")
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
    ObjectPoseTF().run()

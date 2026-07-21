#!/usr/bin/env python3
"""
V2 Object Yaw: use set_model_state (not ros_control!) to rotate object.
"""
import math
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class ObjectYawV2:
    def __init__(self):
        rospy.init_node("object_yaw_v2")
        self.current_yaw_rad = 0.0
        self.obj_pos = rospy.get_param("~object_position",
                                       {"x": 0.72, "y": 0.0, "z": 0.88})

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.yaw_pub = rospy.Publisher("/spray_demo/object_yaw", Float64, queue_size=1)
        rospy.Service("/spray_demo/set_object_yaw", Trigger, self.handle)

        rospy.loginfo("Object yaw V2 ready (via set_model_state)")

    def handle(self, req):
        new_yaw = self.current_yaw_rad
        if abs(new_yaw) < 0.01:
            new_yaw = math.pi  # toggle 0→180
        else:
            new_yaw = 0.0

        q = quaternion_from_euler(0, 0, new_yaw)
        state = ModelState()
        state.model_name = "spray_object"
        state.pose = Pose(
            position=Point(x=self.obj_pos["x"], y=self.obj_pos["y"], z=self.obj_pos["z"]),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        state.reference_frame = "world"

        try:
            self.set_state(state)
            self.current_yaw_rad = new_yaw
            self.yaw_pub.publish(Float64(data=new_yaw))
            msg = "Object yaw set to {:.1f} deg".format(math.degrees(new_yaw))
            rospy.loginfo(msg)
            return TriggerResponse(success=True, message=msg)
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    ObjectYawV2().run()

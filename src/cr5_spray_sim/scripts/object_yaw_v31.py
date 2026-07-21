#!/usr/bin/env python3
"""
V3.1 Object Yaw: 使用 set_model_state 旋转整个吊件模型。
模型名: simple_hanging_workpiece
"""
import math
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class ObjectYawV31:
    def __init__(self):
        rospy.init_node("object_yaw_v31")

        self.model_name = rospy.get_param("~model_name", "simple_hanging_workpiece")
        self.obj_pos = rospy.get_param("~object_position",
                                       {"x": 0.56, "y": 0.0, "z": 0.98})
        self.current_yaw_rad = 0.0

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.yaw_pub = rospy.Publisher("/spray_demo/object_yaw", Float64, queue_size=1)
        rospy.Service("/spray_demo/set_object_yaw", Trigger, self.handle)

        rospy.loginfo("Object yaw V3.1 ready (model=%s, via set_model_state)",
                      self.model_name)

    def handle(self, req):
        new_yaw = self.current_yaw_rad
        if abs(new_yaw) < 0.01:
            new_yaw = math.pi  # toggle 0→180
        else:
            new_yaw = 0.0

        q = quaternion_from_euler(0, 0, new_yaw)
        state = ModelState()
        state.model_name = self.model_name
        state.pose = Pose(
            position=Point(x=self.obj_pos["x"], y=self.obj_pos["y"],
                           z=self.obj_pos["z"]),
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
    ObjectYawV31().run()

#!/usr/bin/env python3
"""
CR5 Spray Demo: Object Yaw Controller
提供服务 /spray_demo/set_object_yaw 来旋转吊挂工件。
发布 /spray_demo/object_yaw 当前角度。
"""
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelConfiguration, GetModelState


class ObjectYawController:
    def __init__(self):
        rospy.init_node("object_yaw_controller")
        self.model_name = rospy.get_param("~object_model", "spray_object")
        self.joint_name = rospy.get_param("~yaw_joint",
                                          "spray_object_yaw_joint")
        self.current_yaw = 0.0

        # Publisher
        self.yaw_pub = rospy.Publisher("/spray_demo/object_yaw", Float64,
                                       queue_size=1)

        # Wait for Gazebo services
        rospy.wait_for_service("/gazebo/set_model_configuration", timeout=30.0)
        self.set_config = rospy.ServiceProxy(
            "/gazebo/set_model_configuration", SetModelConfiguration)

        # Service
        rospy.Service("/spray_demo/set_object_yaw", Trigger,
                      self.handle_set_yaw)
        rospy.loginfo("Object yaw controller ready for model '%s'", self.model_name)

    def handle_set_yaw(self, req):
        """Toggle between 0 and 180 degrees, or set via request comment."""
        try:
            msg = req.comment if hasattr(req, 'comment') and req.comment else ""
            # Parse explicit angle or toggle
            new_yaw = self.current_yaw
            if msg:
                try:
                    new_yaw = float(msg)
                except ValueError:
                    pass

            if new_yaw == self.current_yaw:
                new_yaw = 3.14159 if abs(self.current_yaw) < 0.01 else 0.0

            # Set joint position in Gazebo
            self.set_config(
                self.model_name, "", [self.joint_name], [new_yaw])
            self.current_yaw = new_yaw

            # Publish
            self.yaw_pub.publish(Float64(data=new_yaw))

            msg_out = "Object yaw set to {:.1f} deg".format(
                new_yaw * 180.0 / 3.14159)
            rospy.loginfo(msg_out)
            return TriggerResponse(
                success=True, message=msg_out)

        except Exception as e:
            msg_err = "Failed to set yaw: {}".format(e)
            rospy.logerr(msg_err)
            return TriggerResponse(success=False, message=msg_err)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    ObjectYawController().run()

from __future__ import print_function
from __future__ import division
# from tf.transformations import quaternion_multiply, quaternion_from_euler
# from geometry_msgs.msg import Quaternion
import moveit_commander
from moveit_commander import MoveGroupCommander
import rospy
import time

import sys
from geometry_msgs.msg import PoseStamped, Pose


# import numpy as np
# from itertools import chain
# try:
#     from itertools import izip as zip
# except ImportError: # will be 3.x series
#     pass
from copy import deepcopy
# import math
# 在获取位姿前强制更新规划场景
from moveit_commander import PlanningSceneInterface
  
# 初始化ROS节点
rospy.init_node('test_demo')

 
mgc = MoveGroupCommander("cr5_arm")
mgc.set_planner_id("RRT")  # TODO: this is only needed for the UR5
mgc.set_max_velocity_scaling_factor(0.5)
mgc.set_max_acceleration_scaling_factor(0.2)
# mgc.set_pose_reference_frame("dummy_link")


print("\n active_joints, ", mgc.get_active_joints(), "\n")

# # 获取位姿前更新状态
# mgc.stop()  # 停止任何可能的活动
rospy.sleep(0.1)  # 短暂等待

# end_effector_link="Tool_end"
# # 现在获取位姿
# start_pose = mgc.get_current_pose(end_effector_link)
# print("start_pose:", start_pose)
 

# start_pose: header: 
#   seq: 0
#   stamp: 
#     secs: 1759124346
#     nsecs:  67908763
#   frame_id: "dummy_link"
# pose: 
#   position: 
#     x: 0.4538559809831526
#     y: 0.4018474859053568
#     z: 0.563549474881319
#   orientation: 
#     x: -0.48658840781803137
#     y: -0.47724787332831725
#     z: -0.5522789965808504
#     w: 0.4800563495219715


# start_pose: header: 
#   seq: 0
#   stamp: 
#     secs: 1759124601
#     nsecs: 168486833
#   frame_id: "dummy_link"
# pose: 
#   position: 
#     x: -0.23001662058665168
#     y: 0.40410049232815365
#     z: 0.5421637997982908
#   orientation: 
#     x: -0.5379378475317707
#     y: -0.5009208199673323
#     z: -0.5049781258539238
#     w: 0.4524359587004552


all_p = [[-0.325, 0.787, 0.77, 0.677, 0.022, -0.033, -0.735], [-0.887, 0.748, 0.69, 0.677, 0.022, -0.033, -0.735], 
         [-0.667, 0.744, 0.688, 0.677, 0.022, -0.033, -0.735], [-0.506, 0.738, 0.681, 0.677, 0.022, -0.033, -0.735], 
         [-0.792, 0.746, 0.688, 0.677, 0.022, -0.033, -0.735], [-0.33, 0.733, 0.682, 0.677, 0.022, -0.033, -0.735], 
         [-0.122, 0.731, 0.682, 0.677, 0.022, -0.033, -0.735], [0.133, 0.73, 0.672, 0.677, 0.022, -0.033, -0.735], 
         [0.542, 0.725, 0.657, 0.677, 0.022, -0.033, -0.735], [0.318, 0.73, 0.661, 0.677, 0.022, -0.033, -0.735]]

# all_p = [[-0.325, 0.787, 0.77, 0.677, 0.022, -0.033, -0.735]]

pose_in_base_list = []
for item in all_p:
    pose1 = PoseStamped()
    pose1.header.frame_id = "base_link"
    pose1.header.stamp = rospy.Time.now()     
    pose1.pose.position.x = item[0]
    pose1.pose.position.y = item[1]-0.14
    pose1.pose.position.z = item[2]
    pose1.pose.orientation.x = item[3]
    pose1.pose.orientation.y = item[4]
    pose1.pose.orientation.z = item[5]
    pose1.pose.orientation.w = item[6]
    pose_in_base_list.append(pose1)


for pi in range(len(pose_in_base_list)):
    rospy.loginfo("\n++++++++++++++++++++++++++++++++++++++++++++++")
    rospy.loginfo(str(len(pose_in_base_list)) + "个有效目标，正在规划第" + str(pi)+"个目标的路径")
    try:
        # 设置目标位姿
        mgc.set_start_state_to_current_state()
        mgc.set_pose_target(pose_in_base_list[pi].pose, "Tool_end")

        # 进行规划
        rospy.loginfo("开始规划到目标位姿...")
        plan = mgc.plan()
        
        if plan[0]:
            rospy.loginfo("规划成功，执行运动...")
            mgc.execute(plan[1], wait=True)
            rospy.loginfo("运动完成")
            flag = True
            time.sleep(2)
        else:
            rospy.logwarn("规划失败")
            
    except Exception as e:
        rospy.logerr("规划错误: {}".format(str(e)))
 
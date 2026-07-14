# from __future__ import print_function
# from __future__ import division
# # from tf.transformations import quaternion_multiply, quaternion_from_euler
# # from geometry_msgs.msg import Quaternion
# import moveit_commander
# from moveit_commander import MoveGroupCommander
# import rospy
# # import numpy as np
# # from itertools import chain
# # try:
# #     from itertools import izip as zip
# # except ImportError: # will be 3.x series
# #     pass
# from copy import deepcopy
# # import math
# # 在获取位姿前强制更新规划场景
# from moveit_commander import PlanningSceneInterface
  
# # 初始化ROS节点
# rospy.init_node('test_demo')

 
# mgc = MoveGroupCommander("cr5_arm")
# mgc.set_planner_id("RRT")  # TODO: this is only needed for the UR5
# mgc.set_max_velocity_scaling_factor(0.5)
# mgc.set_max_acceleration_scaling_factor(0.2)
# # mgc.set_pose_reference_frame("dummy_link")


# print("\n active_joints, ", mgc.get_active_joints(), "\n")

# # # 获取位姿前更新状态
# # mgc.stop()  # 停止任何可能的活动
# rospy.sleep(0.1)  # 短暂等待

  
# # 现在获取位姿
# start_pose = mgc.get_current_pose()
# print("start_pose:", start_pose)
 

# fp = deepcopy(start_pose)
# fp.pose.position.x = start_pose.pose.position.x + 0.1
# # fp.pose.position.y = 0
# # fp.pose.position.z = 0
# # fp.pose.orientation.x = 0
# # fp.pose.orientation.y = 0
# # fp.pose.orientation.z = 0
# # fp.pose.orientation.w = 0

# # # pose: 
# # #   position: 
# # #     x: 2.828370763275107e-06
# # #     y: -0.2459999999991972
# # #     z: 1.0470003856814356
# # #   orientation: 
# # #     x: 1.2986741187292023e-06
# # #     y: -0.7071067811805847
# # #     z: 0.707106781180585
# # #     w: 3.896022356026386e-06
# # print("\nstart_pose", start_pose)
# # print("\ntarget_pose", fp)

# # print("\n")
# end_effector_link = mgc.get_end_effector_link()
# mgc.set_pose_target(fp, end_effector_link)
# ret = mgc.plan()
# if type(ret) is tuple:
#     # noetic
#     success, plan1, planning_time, error_code = ret
    

# if len(plan1.joint_trajectory.points) == 0:
#     print(False)
# else:
#     print(True)
#     mgc.execute(ret)



# moveit_commander.roscpp_shutdown()
# moveit_commander.os._exit(0)

 



#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('cr5_arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('home')
        # arm.go()
        # rospy.sleep(1)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.2593
        target_pose.pose.position.y = 0.0636
        target_pose.pose.position.z = 0.1787
        target_pose.pose.orientation.x = 0.70692
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.70729
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    




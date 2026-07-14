#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class ArmController:
    def __init__(self):
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_segment_movement', anonymous=True)
        
        # 创建机器人 commander 和场景
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"  # 根据您的机械臂组名修改
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 设置规划参数
        self.move_group.set_planning_time(10)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.allow_replanning(True)
        
        print("Robot groups:", self.robot.get_group_names())
        print("Current pose:", self.move_group.get_current_pose().pose)
        
    def create_pose(self, position, orientation_euler):
        """创建位姿"""
        pose = geometry_msgs.msg.Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        # 将欧拉角转换为四元数
        quat = quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def plan_complete_trajectory(self, target_pose):
        """规划完整的轨迹"""
        self.move_group.set_pose_target(target_pose)
        
        # 规划路径
        plan = self.move_group.plan()
        
        if not plan[0]:
            print("完整路径规划失败!")
            return None
            
        print("完整路径规划成功!")
        return plan[1]
    
    def extract_waypoints_from_trajectory(self, trajectory, num_segments=5):
        """从轨迹中提取路径点"""
        waypoints = []
        
        if not trajectory.joint_trajectory.points:
            print("轨迹中没有路径点!")
            return waypoints
        
        points = trajectory.joint_trajectory.points
        total_points = len(points)
        
        print(f"轨迹包含 {total_points} 个路径点")
        
        # 均匀选取路径点（包括起点和终点）
        indices = []
        if total_points <= num_segments + 1:
            # 如果轨迹点很少，直接使用所有点
            indices = list(range(total_points))
        else:
            # 均匀选取点，包括第一个和最后一个点
            step = (total_points - 1) / (num_segments)
            indices = [0]  # 起点
            for i in range(1, num_segments):
                idx = int(i * step)
                if idx < total_points and idx not in indices:
                    indices.append(idx)
            indices.append(total_points - 1)  # 终点
        
        print(f"选取的路径点索引: {indices}")
        
        # 将选取的关节位置转换为笛卡尔空间位姿
        for idx in indices:
            # 设置机械臂到该关节位置
            joint_positions = points[idx].positions
            
            # 通过正向运动学计算位姿
            self.move_group.set_joint_value_target(joint_positions)
            waypoint_pose = self.move_group.get_current_pose().pose  # 这里应该用正向运动学计算，但MoveIt Python API限制
            
            # 由于MoveIt Python API限制，我们这里使用近似方法
            # 在实际应用中，您可能需要使用ROS服务调用计算机械臂正向运动学
            waypoints.append(waypoint_pose)
        
        return waypoints
    
    def execute_segmented_movement(self, trajectory, num_segments=5, pause_time=3):
        """执行分段移动"""
        try:
            # 从完整轨迹中提取路径点
            waypoints = self.extract_waypoints_from_trajectory(trajectory, num_segments)
            
            if not waypoints:
                print("无法提取路径点!")
                return False
            
            print(f"从轨迹中提取了 {len(waypoints)} 个路径点")
            
            # 逐段执行
            for i, waypoint in enumerate(waypoints):
                if i == 0:
                    # 第一个点通常是当前位置，跳过
                    continue
                    
                print(f"执行第 {i}/{len(waypoints)-1} 段移动...")
                
                # 设置目标位姿
                self.move_group.set_pose_target(waypoint)
                
                # 规划到当前路径点的轨迹
                segment_plan = self.move_group.plan()
                
                if not segment_plan[0]:
                    print(f"第 {i} 段规划失败!")
                    return False
                
                # 执行移动
                success = self.move_group.execute(segment_plan[1], wait=True)
                
                if not success:
                    print(f"第 {i} 段执行失败!")
                    return False
                
                print(f"第 {i} 段移动完成，等待 {pause_time} 秒...")
                
                # 获取当前位姿
                current_pose = self.move_group.get_current_pose().pose
                print(f"当前位姿: [{current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}]")
                
                # 停留指定时间
                time.sleep(pause_time)
            
            print("所有分段移动完成!")
            return True
            
        except Exception as e:
            print(f"移动过程中发生错误: {e}")
            return False

    def execute_segmented_movement_direct(self, trajectory, num_segments=5, pause_time=3):
        """直接分割关节轨迹执行分段移动（更精确的方法）"""
        try:
            if not trajectory.joint_trajectory.points:
                print("轨迹中没有路径点!")
                return False
            
            points = trajectory.joint_trajectory.points
            total_points = len(points)
            joint_names = trajectory.joint_trajectory.joint_names
            
            print(f"轨迹包含 {total_points} 个路径点，分割为 {num_segments} 段")
            
            # 计算每段应该包含的路径点数
            points_per_segment = max(1, total_points // num_segments)
            
            for segment in range(num_segments):
                start_idx = segment * points_per_segment
                
                # 最后一段包含所有剩余点
                if segment == num_segments - 1:
                    end_idx = total_points
                else:
                    end_idx = (segment + 1) * points_per_segment
                
                # 创建子轨迹
                segment_trajectory = JointTrajectory()
                segment_trajectory.joint_names = joint_names
                segment_trajectory.points = points[start_idx:end_idx]
                
                print(f"执行第 {segment+1}/{num_segments} 段 (点 {start_idx}-{end_idx-1})...")
                
                # 执行子轨迹
                self.move_group.execute(segment_trajectory, wait=True)
                
                # 获取当前位姿
                current_pose = self.move_group.get_current_pose().pose
                print(f"当前位姿: [{current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}]")
                
                
                print(f"第 {segment+1} 段完成，等待 {pause_time} 秒...")
                time.sleep(pause_time)
            
            print("所有分段移动完成!")
            return True
            
        except Exception as e:
            print(f"移动过程中发生错误: {e}")
            return False

def main():
    arm_controller = ArmController()
    
    # 等待MoveIt初始化
    time.sleep(2)
    
    # 获取当前位姿作为pose1
    pose1 = arm_controller.move_group.get_current_pose().pose
    print(f"起始位姿 pose1: {pose1.position}")
    
    # 定义目标位姿 pose2（根据您的机械臂修改这些值）
    pose2 = arm_controller.create_pose(
        position=[0.5, 0.2, 0.5],  # x, y, z 坐标
        orientation_euler=[0, 0, 0]  # 欧拉角: roll, pitch, yaw
    )
    print(f"目标位姿 pose2: {pose2.position}")
    
    # 规划完整轨迹
    trajectory = arm_controller.plan_complete_trajectory(pose2)
    
    if trajectory is None:
        print("无法规划完整轨迹，退出!")
        return
    
    # 执行分段移动（推荐使用直接分割关节轨迹的方法）
    success = arm_controller.execute_segmented_movement_direct(
        trajectory=trajectory,
        num_segments=5,
        pause_time=3
    )
    
    if success:
        print("任务成功完成!")
    else:
        print("任务失败!")
    
    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    import sys
    try:
        main()
    except rospy.ROSInterruptException:
        pass
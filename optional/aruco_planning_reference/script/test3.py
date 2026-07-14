import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from std_msgs.msg import Bool  # 导入Bool消息类型
import geometry_msgs.msg
import math

# youdao@service:~/cr5_ws/src$ /usr/bin/python3 /home/youdao/cr5_ws/src/TCP-IP-ROS-6AXis/aruco_planning/script/test3.py
# 让机械臂绕一圈，配合slam程序，构建环境点云
class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo', anonymous=True)
        # 创建一个Publisher，发布名为'bool_topic'的话题，消息类型为Bool，队列大小10
        self.bool_pub = rospy.Publisher('bool_topic', Bool, queue_size=10)
        # 创建并填充消息
        bool_msg = Bool()
        bool_msg.data = False
        # 发布消息
        self.bool_pub.publish(bool_msg)


        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('cr5_arm')
                
        # 获取终端link的名称
        self.end_effector_link = "Tool_end"
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
       
        # self.arm.set_goal_joint_tolerance(0.02)
        # self.arm.set_goal_position_tolerance(0.02)
        # self.arm.set_goal_orientation_tolerance(0.05)


        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
               
 
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        pose1 = PoseStamped()
        pose1.header.frame_id = self.reference_frame
        pose1.header.stamp = rospy.Time.now()     
        pose1.pose.position.x = 0.4538559809831526
        pose1.pose.position.y = 0.4018474859053568
        pose1.pose.position.z = 0.563549474881319
        pose1.pose.orientation.x = -0.48658840781803137
        pose1.pose.orientation.y = -0.47724787332831725
        pose1.pose.orientation.z = -0.5522789965808504
        pose1.pose.orientation.w = 0.4800563495219715
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(pose1, self.end_effector_link)
        
        # 规划运动路径
        traj = self.arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj[1])
        rospy.sleep(1)

 
        pose2 = PoseStamped()
        pose2.header.frame_id = self.reference_frame
        pose2.header.stamp = rospy.Time.now()     
        pose2.pose.position.x = -0.23001662058665168
        pose2.pose.position.y = 0.40410049232815365
        pose2.pose.position.z = 0.5421637997982908
        pose2.pose.orientation.x = -0.5379378475317707
        pose2.pose.orientation.y = -0.5009208199673323
        pose2.pose.orientation.z = -0.5049781258539238
        pose2.pose.orientation.w = 0.4524359587004552
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(pose2, self.end_effector_link)
        
        # 规划运动路径
        traj = self.arm.plan()
 

        # 执行分段移动（推荐使用直接分割关节轨迹的方法）
        success = self.execute_segmented_movement_robust(traj[1], 18)
 

        if success:
            print("任务成功完成!")
        else:
            print("任务失败!")
 
        # # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)
    
    def calculate_euclidean_distance(self, pose1, pose2):
        """计算两个位姿之间的欧几里得距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def analyze_trajectory_distance(self, trajectory, split_num):
        """分析轨迹并计算总距离，不移动机械臂"""
        if trajectory is None:
            return 0.0, []
        
        # 获取轨迹点
        joint_trajectory = trajectory.joint_trajectory
        points = joint_trajectory.points
        
        print(f"轨迹包含 {len(points)} 个路径点")
        
        # 存储笛卡尔坐标点
        cartesian_points = []
         
        all_distance = [0]
        # 对每个路径点计算正向运动学
        for i, point in enumerate(points):
            # 使用正向运动学计算末端位姿 - 这不会移动实际机器人
            fk_result = self.calculate_forward_kinematics(point.positions)
            if fk_result:
                cartesian_points.append(fk_result)
                
                # 计算与前一点的距离（从第二个点开始）
                if i > 0:
                    segment_distance = self.calculate_euclidean_distance(
                        cartesian_points[i-1], cartesian_points[i]
                    )
                    all_distance.append(segment_distance)
            else:
                raise ValueError("第"+str(i)+"点无法规划！")
            
        print("\nall_distance: ", all_distance)
        mean_distance = sum(all_distance)/split_num
        print("mean_distance: ", mean_distance)

        pause_ix = []
        temp_sum = 0.0
        for i in range(1, len(all_distance)):
            temp_sum = temp_sum + all_distance[i]
            if temp_sum > mean_distance:
                print("temp_sum : ", temp_sum)
                if len(pause_ix)==0:
                    if i==1:
                        pause_ix.append(i)
                    else:
                        pause_ix.append(i-1)  
                else:
                    if i == (pause_ix[-1]+1):
                        pause_ix.append(i)
                    else:
                        pause_ix.append(i-1)
                temp_sum = 0  

            if i==(len(all_distance)-1):
                if len(pause_ix)==0:
                    pause_ix.append(i)
                else:
                    if pause_ix[-1]!=i:
                        pause_ix.append(i)
        return pause_ix
    
    def calculate_forward_kinematics(self, joint_positions):
        """计算正向运动学获取末端位姿，不移动机械臂"""
        try:
            # 设置关节状态（虚拟的，不移动实际机器人）
            self.arm.set_joint_value_target(joint_positions)
            
            # 获取正向运动学结果 - 这只是计算，不移动机器人
            fk_pose = self.arm.get_current_pose(self.end_effector_link).pose
            
            return fk_pose
        except Exception as e:
            print(f"正向运动学计算错误: {e}")
            return None
        
    
    def wait_for_stabilization(self, timeout=2.0):
        """等待机械臂稳定"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            # 检查机械臂是否停止运动
            current_velocities = self.arm.get_current_joint_values()
            if all(abs(v) < 0.01 for v in current_velocities):
                break
            rate.sleep()
    
    def execute_segmented_movement_robust(self, trajectory, split_num):
        """更鲁棒的分段移动执行方法"""
        try:
            if not trajectory.joint_trajectory.points:
                print("轨迹中没有路径点!")
                return False
            
            pause_ix = self.analyze_trajectory_distance(trajectory, split_num)

 
            points = trajectory.joint_trajectory.points
            total_points = len(points)
            joint_names = trajectory.joint_trajectory.joint_names
            
            print(f"轨迹包含 {total_points} 个路径点")
            print("pause_ix: ", pause_ix)

            # 第一次发布
            bool_msg = Bool()
            bool_msg.data = True
            # 发布消息
            self.bool_pub.publish(bool_msg)
            time.sleep(1.5)
            bool_msg.data = False
            # 发布消息
            self.bool_pub.publish(bool_msg)

            for segment in range(len(pause_ix)):
  
                print(f"\n执行第 {segment+1}/{len(pause_ix)}")
    
                # 获取这段轨迹的最后一个点作为目标
                target_joint_positions = points[pause_ix[segment]].positions
                
                # 设置关节目标
                self.arm.set_joint_value_target(target_joint_positions)
                
                # 重新规划
                plan = self.arm.plan()
                
                if plan[0]:
                    # 执行重新规划的轨迹
                    self.arm.execute(plan[1], wait=True)
                    print(f"第 {segment+1} 段重新规划并执行成功")
                else:
                    print(f"第 {segment+1} 段重新规划失败!")
                    return False
                
                # 等待机械臂稳定
                self.wait_for_stabilization()
                
                # 获取当前位姿
                current_pose = self.arm.get_current_pose().pose
                current_joints = self.arm.get_current_joint_values()
                print(f"当前位姿: [{current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}]")
                print(f"当前关节: {[f'{j:.3f}' for j in current_joints]}")
                
 
                print(f"第 {segment+1} 段完成，等待 4 秒...")
                time.sleep(2)
                bool_msg.data = True
                # 发布消息
                self.bool_pub.publish(bool_msg)
                time.sleep(2)
                bool_msg.data = False
                # 发布消息
                self.bool_pub.publish(bool_msg)

            
            print("所有分段移动完成!")
            return True
            
        except Exception as e:
            print(f"移动过程中发生错误: {e}")
            return False

if __name__ == "__main__":
    MoveItIkDemo()
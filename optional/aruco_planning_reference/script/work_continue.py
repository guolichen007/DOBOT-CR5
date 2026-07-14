#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf
import tf.transformations as tf_trans
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from moveit_commander import MoveGroupCommander, RobotCommander
import moveit_commander
import sys
import math
import time 

class ArucoDetector:
    def __init__(self):
        # 设置参数
        self.aruco_dict_type = aruco.DICT_4X4_50
        self.target_marker_id = 0
        self.camera_frame = "camera_color_optical_frame"
        self.marker_size = 0.095  # 10cm的标记
        self.offset_distance = 0.05  # 在标记前方10cm
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # CV桥接
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 检测状态
        self.marker_detected = False
        self.target_pose_camera = None
        self.marker_pose_camera = None
        self.target_pose_camera_list = []
        



        # 订阅相机话题
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        
        rospy.loginfo("Aruco检测器已初始化")

    def camera_info_callback(self, msg):
        """获取相机内参"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(msg.D)
            rospy.loginfo("已获取相机内参")

    def image_callback(self, msg):
        """图像回调函数，检测ArUco标记"""
        try:
            if self.marker_detected:
                return

            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
 
            # 检测ArUco标记
            corners, ids, rejected = self.detect_aruco_markers(cv_image)
            
            if ids is not None and self.target_marker_id in ids:
                # 找到目标标记
                for ci in range(len(corners)):
                    if ids[ci] != self.target_marker_id:
                        continue
                    target_corners = corners[ci]
                    
                    # 估计标记位姿
                    success, rvec, tvec = self.estimate_marker_pose(target_corners)
                    
                    if success:
                        # 转换到相机坐标系中的位姿
                        self.marker_pose_camera = self.get_marker_pose_from_rvec_tvec(rvec, tvec)
                        
                        if self.marker_pose_camera is not None:
                            # 计算目标位姿（在标记前方一定距离）
                            self.target_pose_camera = self.calculate_target_pose(self.marker_pose_camera)
                            
                            if self.target_pose_camera is not None:
                                self.target_pose_camera_list.append(self.target_pose_camera)
                                self.target_pose_camera = None
                            self.marker_pose_camera = None
                if len(self.target_pose_camera_list) > 0:
                    self.marker_detected = True
                    rospy.logerr("找到{}个目标!!!!!!!!!!!!!!!!!!1 ".format(str(self.target_pose_camera_list)))


            # 可视化
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                # # 绘制坐标轴
                # if success:
                #     cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
            
            cv2.imshow('Aruco Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr("图像处理错误: {}".format(str(e)))

    def detect_aruco_markers(self, image):
        """检测ArUco标记"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检查OpenCV版本并选择相应的API
        if cv2.__version__ >= '4.7.0':
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)
        
        return corners, ids, rejected

    def estimate_marker_pose(self, corners):
        """估计标记位姿"""
        if self.camera_matrix is None:
            rospy.logwarn("相机内参未初始化")
            return False, None, None

        try:
            # 定义标记的3D对象点 (基于标记大小)
            marker_points = np.array([[
                [-self.marker_size / 2, self.marker_size / 2, 0],   # 左上角
                [self.marker_size / 2, self.marker_size / 2, 0],    # 右上角
                [self.marker_size / 2, -self.marker_size / 2, 0],   # 右下角
                [-self.marker_size / 2, -self.marker_size / 2, 0]   # 左下角
            ]], dtype=np.float32)

            # 使用solvePnP来估计姿态
            success, rvec, tvec = cv2.solvePnP(
                marker_points, 
                corners.astype(np.float32),
                self.camera_matrix, 
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # 验证位姿的合理性
                distance = np.linalg.norm(tvec)
                
                if distance > 2.0:  # 如果距离太远，可能是误检测
                    rospy.logwarn("检测到的标记距离过远: {:.3f}m，可能是误检测".format(distance))
                    return False, None, None
                    
                return True, rvec, tvec
            else:
                rospy.logwarn("solvePnP失败")
                return False, None, None
                
        except Exception as e:
            rospy.logwarn("估计位姿时发生异常: {}".format(str(e)))
            return False, None, None

    def get_marker_pose_from_rvec_tvec(self, rvec, tvec):
        """从rvec和tvec获取位姿"""
        pose = Pose()
        
        try:
            # 转换旋转向量到旋转矩阵
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            
            # 将旋转矩阵转换为四元数
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            quaternion = tf_trans.quaternion_from_matrix(transform_matrix)
            
            # 设置位置
            tvec_flat = tvec.flatten()
            pose.position.x = tvec_flat[0]
            pose.position.y = tvec_flat[1]
            pose.position.z = tvec_flat[2]
            
            # 设置方向（四元数）
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            
            return pose
            
        except Exception as e:
            rospy.logerr("转换rvec/tvec到位姿时出错: {}".format(str(e)))
            return None
    
    def rotate_180_degrees_x(self, quat):
        """
        绕X轴旋转180度（以当前姿态为原点）
        参数:
            quat: geometry_msgs.msg.Quaternion 输入四元数
        返回:
            geometry_msgs.msg.Quaternion 旋转后的四元数
        """
        from tf.transformations import quaternion_multiply, quaternion_about_axis
        
        # 创建绕X轴旋转180度的四元数
        flip_x_quat = quaternion_about_axis(np.pi, [1, 0, 0])
        
        # 将旋转应用到当前四元数（四元数乘法顺序很重要）
        new_quat_array = quaternion_multiply(
            [quat.x, quat.y, quat.z, quat.w],  # 当前姿态
            [flip_x_quat[0], flip_x_quat[1], flip_x_quat[2], flip_x_quat[3]]  # 旋转
        )
        
        # 创建新的四元数消息
        new_orientation = geometry_msgs.msg.Quaternion()
        new_orientation.x = new_quat_array[0]
        new_orientation.y = new_quat_array[1]
        new_orientation.z = new_quat_array[2]
        new_orientation.w = new_quat_array[3]
        
        return new_orientation

    def calculate_target_pose(self, marker_pose):
        """计算目标位姿（在标记前方一定距离）"""
        try:
            target_pose = Pose()
            
            # 位置：在标记前方一定距离（沿着相机Z轴方向）
            target_pose.position.x = marker_pose.position.x
            target_pose.position.y = marker_pose.position.y
            target_pose.position.z = marker_pose.position.z - self.offset_distance  # 沿着相机Z轴前进
            
            target_pose.orientation = self.rotate_180_degrees_x(marker_pose.orientation)

            return target_pose
            
        except Exception as e:
            rospy.logerr("计算目标位姿失败: {}".format(str(e)))
            return None

    def publish_marker_tf(self, pose_stamped, frame_id):
        """发布标记位姿的TF"""
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = pose_stamped.header.frame_id
            transform.child_frame_id = frame_id
            
            transform.transform.translation.x = pose_stamped.pose.position.x
            transform.transform.translation.y = pose_stamped.pose.position.y
            transform.transform.translation.z = pose_stamped.pose.position.z
            
            transform.transform.rotation.x = pose_stamped.pose.orientation.x
            transform.transform.rotation.y = pose_stamped.pose.orientation.y
            transform.transform.rotation.z = pose_stamped.pose.orientation.z
            transform.transform.rotation.w = pose_stamped.pose.orientation.w
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logwarn("发布TF失败: {}".format(str(e)))

    def get_target_pose(self):
        """获取目标位姿"""
        return self.target_pose_camera_list, self.marker_detected

class MoveItController:
    def __init__(self, aruco_detector):
        # 初始化MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 设置参数
        self.robot_base_frame = "base_link"
        self.ee_frame = "Link6"        # 
        self.tool_frame = "Tool_end"
        self.camera_frame = "camera_color_optical_frame"
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Aruco检测器
        self.aruco_detector = aruco_detector
        
        # 初始化MoveGroup
        try:
            self.move_group = MoveGroupCommander("cr5_arm")
            self.robot = RobotCommander()
            
            # 设置规划参数
            self.move_group.set_planning_time(5)
            self.move_group.set_num_planning_attempts(10)
            self.move_group.set_goal_position_tolerance(0.01)
            self.move_group.set_goal_orientation_tolerance(0.1)
            self.move_group.set_planner_id("RRTConnect") 
            self.move_group.set_max_velocity_scaling_factor(0.5)
            self.move_group.set_max_acceleration_scaling_factor(0.2)
            
            # 设置末端执行器参考坐标系
            self.move_group.set_pose_reference_frame(self.robot_base_frame)
            # self.move_group.set_pose_reference_frame(self.ee_frame)
            
            rospy.loginfo("MoveIt控制器初始化成功")
        except Exception as e:
            rospy.logerr("MoveIt初始化失败: {}".format(str(e)))
            return

    def pose_to_matrix(self, pose):
        """
        将 geometry_msgs/Pose 转换为 4x4 齐次变换矩阵
        """
        q = pose.orientation
        t = pose.position
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z
        return T

    def matrix_to_pose(self, T):
        """
        将 4x4 齐次变换矩阵转换为 geometry_msgs/Pose
        """
        pose = Pose()
        # 提取位置
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]
        # 提取旋转（四元数）
        q = tf.transformations.quaternion_from_matrix(T)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

    def get_transform(self, target_frame, source_frame):
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            
            # 将 TransformStamped 转换为 4x4 变换矩阵
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # 创建齐次变换矩阵
            T = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            T[0, 3] = trans.x
            T[1, 3] = trans.y
            T[2, 3] = trans.z
            
            return T
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed: %s" % str(e))
            return None

    def transform_pose_to_base(self, target_pose_camera):
        """将相机坐标系中的位姿转换到基坐标系"""
        try:
            # 获取从相机到末端执行器的变换
            T_cam_to_tool = self.get_transform(self.ee_frame, self.camera_frame)
            if T_cam_to_tool is None:
                rospy.logerr("无法获取相机到末端执行器的变换")
                return None
            
            # 获取目标在相机坐标系中的位姿矩阵
            T_obj_in_cam = self.pose_to_matrix(target_pose_camera)
            
            # 计算目标在末端执行器坐标系中的位姿
            T_obj_in_tool = np.dot(T_cam_to_tool, T_obj_in_cam)
            
            # 获取当前末端执行器在基坐标系中的位姿
            current_robot_pose = self.move_group.get_current_pose().pose
            T_tool_to_base = self.pose_to_matrix(current_robot_pose)
            
            # 计算目标在机器人基坐标系下的位姿
            T_obj_in_base = np.dot(T_tool_to_base, T_obj_in_tool)
            
            # 转换回Pose消息
            target_pose_in_base = self.matrix_to_pose(T_obj_in_base)

            # target_pose_in_base = self.matrix_to_pose(T_obj_in_tool)
            
            # 创建PoseStamped消息
            pose_in_base = PoseStamped()
            pose_in_base.header.stamp = rospy.Time.now()
            # pose_in_base.header.frame_id = self.ee_frame
            pose_in_base.header.frame_id = self.robot_base_frame
            pose_in_base.pose = target_pose_in_base
            
            # 发布TF用于可视化
            self.publish_marker_tf(pose_in_base, "target_pose_base")
            
            return pose_in_base
            
        except Exception as e:
            rospy.logerr("位姿转换失败: {}".format(str(e)))
            return None

    def publish_marker_tf(self, pose_stamped, frame_id):
        """发布标记位姿的TF"""
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = pose_stamped.header.frame_id
            transform.child_frame_id = frame_id
            
            transform.transform.translation.x = pose_stamped.pose.position.x
            transform.transform.translation.y = pose_stamped.pose.position.y
            transform.transform.translation.z = pose_stamped.pose.position.z
            
            transform.transform.rotation.x = pose_stamped.pose.orientation.x
            transform.transform.rotation.y = pose_stamped.pose.orientation.y
            transform.transform.rotation.z = pose_stamped.pose.orientation.z
            transform.transform.rotation.w = pose_stamped.pose.orientation.w
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logwarn("发布TF失败: {}".format(str(e)))

    def plan_to_marker(self):
        """规划机械臂移动到标记位置"""
        # 获取目标位姿
        target_pose_camera_list, marker_detected = self.aruco_detector.get_target_pose()
        
        if not marker_detected:
            rospy.logwarn("未检测到标记或目标位姿无效")
            return False
        
        pose_in_base_list = []
        rospy.loginfo("\n----------------------------------------------------")
        for ti in range(len(target_pose_camera_list)):
            rospy.loginfo(str(len(target_pose_camera_list)) + "个目标，正在转换第" + str(ti)+"个目标到base_link坐标")

            try:
                # 转换位姿到基坐标系
                pose_in_base = self.transform_pose_to_base(target_pose_camera_list[ti])
                
                if pose_in_base is None:
                    rospy.logerr("位姿转换失败")
                    continue
                pose_in_base_list.append(pose_in_base)
            except Exception as e:
                rospy.logerr("转换错误: {}".format(str(e)))
                continue
            
        if len(pose_in_base_list)==0:
            rospy.loginfo("\n没有目标的坐标转换成功!!!")
            return False

        flag = False
        for pi in range(len(pose_in_base_list)):
            rospy.loginfo("\n++++++++++++++++++++++++++++++++++++++++++++++")
            rospy.loginfo(str(len(pose_in_base_list)) + "个有效目标，正在规划第" + str(pi)+"个目标的路径")
            try:
                # 设置目标位姿
                self.move_group.set_start_state_to_current_state()
                self.move_group.set_pose_target(pose_in_base_list[pi].pose, self.tool_frame)

                # 进行规划
                rospy.loginfo("开始规划到目标位姿...")
                plan = self.move_group.plan()
                
                if plan[0]:
                    rospy.loginfo("规划成功，执行运动...")
                    self.move_group.execute(plan[1], wait=True)
                    rospy.loginfo("运动完成")
                    flag = True
                    time.sleep(2)
                else:
                    rospy.logwarn("规划失败")
                    
            except Exception as e:
                rospy.logerr("规划错误: {}".format(str(e)))
        self.aruco_detector.marker_detected = False
        if flag:
            return True
        else:
            return False
        


    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz
        plan_once = False
        
        while not rospy.is_shutdown() and not plan_once:
            # 检测到标记，开始规划
            success = self.plan_to_marker()
            
            if success:
                rospy.loginfo("成功移动到标记位置")
                plan_once = True
            else:
                rospy.logwarn("移动失败，等待重新检测")
            
            rate.sleep()

class ArucoDetectorAndPlanner:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('aruco_detector_planner', anonymous=True)
        
        # 初始化检测器和控制器
        self.aruco_detector = ArucoDetector()
        self.moveit_controller = MoveItController(self.aruco_detector)
        
        rospy.loginfo("Aruco检测和规划节点已初始化")

    def run(self):
        """运行主程序"""
        rospy.sleep(2)  # 等待初始化完成
        self.moveit_controller.run()

if __name__ == '__main__':
    try:
        detector_planner = ArucoDetectorAndPlanner()
        detector_planner.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("程序错误: {}".format(str(e)))
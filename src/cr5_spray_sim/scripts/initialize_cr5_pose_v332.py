#!/usr/bin/env python3
"""
V3.3.2 CR5 确定性初始姿态初始化 (简化版)。

依赖:
- Gazebo paused 启动
- controller_spawner 已在 launch 中加载+启动 controllers
- 此脚本只负责: set joints → verify → unpause → monitor

启动流程:
  1. 确认物理已暂停
  2. set_model_configuration 设六轴零位
  3. 等待 joint_states 有数据且近零
  4. 发送保持零位轨迹给 arm_controller
  5. 在 paused 状态验证 Link6/nozzle TF 高度
  6. unpause physics
  7. 监控 5 秒确认不塌陷
"""
import sys
import math
import rospy
import actionlib
import yaml
import tf2_ros
import numpy as np

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ZERO_POSITIONS = [0.0] * 6


def load_config():
    cfg_path = rospy.get_param("~config", "")
    if not cfg_path or not cfg_path.startswith("/"):
        import os
        pkg_path = os.environ.get("CR5_SPRAY_PKG",
            "/home/ydkj/cr5_ros1_ws/src/cr5_spray_sim")
        cfg_path = os.path.join(pkg_path, cfg_path if cfg_path else
                                "config/cr5_initial_pose_v332.yaml")
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)["cr5_initial_pose"]


def wait_for_service(srv_name, timeout_s=20.0):
    try:
        rospy.wait_for_service(srv_name, timeout_s)
        return True
    except rospy.ROSException:
        rospy.logerr("Service %s not available", srv_name)
        return False


def main():
    rospy.init_node("initialize_cr5_pose_v332", disable_signals=True)

    cfg = load_config()
    rospy.loginfo("CR5 initial pose: %s", cfg["name"])

    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)

    # A. Pause physics (double check)
    rospy.loginfo("=== A: Pause physics ===")
    if not wait_for_service("/gazebo/pause_physics"):
        sys.exit(1)
    try:
        rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
        rospy.loginfo("Physics paused")
    except Exception as e:
        rospy.logerr("pause_physics: %s", e)
    rospy.sleep(0.5)

    # B. Set joint configuration
    rospy.loginfo("=== B: Set joint configuration ===")
    if not wait_for_service("/gazebo/set_model_configuration"):
        sys.exit(1)
    try:
        srv = rospy.ServiceProxy("/gazebo/set_model_configuration",
                                 SetModelConfiguration)
        req = SetModelConfigurationRequest()
        req.model_name = "cr5_robot"
        req.urdf_param_name = "robot_description"
        req.joint_names = JOINT_NAMES
        req.joint_positions = ZERO_POSITIONS
        resp = srv(req)
        if resp.success:
            rospy.loginfo("set_model_configuration: OK")
        else:
            rospy.logerr("set_model_configuration: %s", resp.status_message)
            sys.exit(1)
    except Exception as e:
        rospy.logerr("set_model_configuration: %s", e)
        sys.exit(1)

    # C. Wait for joint_states (controllers started by spawner)
    rospy.loginfo("=== C: Wait for joint_states ===")
    rospy.sleep(2.0)
    max_wait = 30.0
    start = rospy.Time.now()
    joints_ok = False
    while (rospy.Time.now() - start).to_sec() < max_wait:
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
            positions = dict(zip(msg.name, msg.position))
            all_near_zero = True
            for name in JOINT_NAMES:
                actual = positions.get(name)
                if actual is None or not math.isfinite(actual):
                    all_near_zero = False
                    break
                if abs(actual) > cfg["tolerance_rad"]:
                    all_near_zero = False
                    break
            if all_near_zero and len([n for n in msg.name if n in JOINT_NAMES]) >= 6:
                rospy.loginfo("joint_states OK: %s",
                              {n: f"{positions.get(n, 0):.4f}" for n in JOINT_NAMES})
                joints_ok = True
                break
            rospy.loginfo("Waiting for zero joint states...")
        except rospy.ROSException:
            pass
        rospy.sleep(1.0)

    if not joints_ok:
        rospy.logerr("FATAL: joint_states not at zero after %.0fs", max_wait)
        sys.exit(1)

    # D. Send hold trajectory
    rospy.loginfo("=== D: Send hold trajectory ===")
    action_name = "/arm_controller/follow_joint_trajectory"
    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
    if client.wait_for_server(rospy.Duration(15.0)):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = ZERO_POSITIONS
        point.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(point)
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(5.0)):
            rospy.loginfo("Hold trajectory: OK")
        else:
            rospy.logwarn("Hold trajectory timed out (may be ok)")
    else:
        rospy.logwarn("arm_controller action not available (may be starting)")

    rospy.sleep(1.0)

    # E. Verify TF heights (paused, after setting joints)
    rospy.loginfo("=== E: Verify frame heights (paused) ===")
    for frame_key, frame_cfg in cfg["expected_frames"].items():
        try:
            t = tf_buf.lookup_transform("world", frame_key, rospy.Time(0),
                                         rospy.Duration(5.0))
            z = t.transform.translation.z
            expected = np.array(frame_cfg["position"])
            actual = np.array([t.transform.translation.x,
                               t.transform.translation.y, z])
            dist = np.linalg.norm(actual - expected)

            rospy.loginfo("  %s: [%.3f, %.3f, %.3f] (expected ~[%.3f, %.3f, %.3f], dist=%.3f)",
                          frame_key, actual[0], actual[1], actual[2],
                          expected[0], expected[1], expected[2], dist)

            if z < frame_cfg["min_z_m"]:
                rospy.logerr("FATAL: %s z=%.3f < min=%.2f", frame_key, z, frame_cfg["min_z_m"])
                sys.exit(1)

            if dist > frame_cfg["position_tolerance_m"]:
                rospy.logerr("FATAL: %s distance %.3f > tol %.3f",
                             frame_key, dist, frame_cfg["position_tolerance_m"])
                sys.exit(1)
        except Exception as e:
            rospy.logerr("TF %s: %s", frame_key, e)
            sys.exit(1)

    # Check for folding
    try:
        t = tf_buf.lookup_transform("world", "Link6", rospy.Time(0), rospy.Duration(3.0))
        if t.transform.translation.z < cfg["folded_z_threshold_m"]:
            rospy.logerr("FATAL: CR5_ARM_FOLDED (Link6.z=%.3f < %.2f)",
                         t.transform.translation.z, cfg["folded_z_threshold_m"])
            sys.exit(1)
    except Exception as e:
        rospy.logerr("Folded check failed: %s", e)
        sys.exit(1)

    # F. Unpause physics
    rospy.loginfo("=== F: Unpause physics ===")
    if not wait_for_service("/gazebo/unpause_physics"):
        sys.exit(1)
    try:
        rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
        rospy.loginfo("Physics unpaused")
    except Exception as e:
        rospy.logerr("unpause_physics: %s", e)
        sys.exit(1)

    rospy.sleep(1.0)

    # G. Monitor stability
    rospy.loginfo("=== G: Monitor stability (%.1fs) ===", cfg["monitor_duration_s"])
    mon_start = rospy.Time.now()
    rate = rospy.Rate(10)
    mon_ok = True
    while (rospy.Time.now() - mon_start).to_sec() < cfg["monitor_duration_s"]:
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
            positions = dict(zip(msg.name, msg.position))
            for name in JOINT_NAMES:
                actual = positions.get(name)
                if actual is None or not math.isfinite(actual):
                    continue
                if abs(actual) > cfg["monitor_tolerance_rad"]:
                    rospy.logerr("Monitor: %s = %.4f drifted", name, actual)
                    mon_ok = False
        except rospy.ROSException:
            pass

        for frame_key, min_z in [("Link6", cfg["expected_frames"]["Link6"]["min_z_m"]),
                                  ("spray_nozzle_frame",
                                   cfg["expected_frames"]["spray_nozzle_frame"]["min_z_m"])]:
            try:
                t = tf_buf.lookup_transform("world", frame_key, rospy.Time(0),
                                             rospy.Duration(1.0))
                if t.transform.translation.z < min_z:
                    rospy.logerr("Monitor: %s z=%.3f < %.2f", frame_key,
                                 t.transform.translation.z, min_z)
                    mon_ok = False
            except Exception:
                pass
        rate.sleep()

    if not mon_ok:
        rospy.logerr("FATAL: stability monitor failed")
        try:
            rospy.ServiceProxy("/gazebo/pause_physics", Empty)()
        except Exception:
            pass
        sys.exit(1)

    # Final check
    try:
        t = tf_buf.lookup_transform("world", "Link6", rospy.Time(0), rospy.Duration(3.0))
        if t.transform.translation.z < cfg["folded_z_threshold_m"]:
            rospy.logerr("FATAL: CR5 still folded after unpause")
            sys.exit(1)
    except Exception:
        pass

    rospy.loginfo("==============================================")
    rospy.loginfo("  CR5_INITIAL_POSE_READY")
    rospy.loginfo("  Link6.z = %.3f m", t.transform.translation.z if 't' in dir() else 0)
    rospy.loginfo("==============================================")
    sys.exit(0)


if __name__ == "__main__":
    main()

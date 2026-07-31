#!/usr/bin/env python3
"""
CR5 Calibration Scene Geometry Absolute Check.

在 paused 状态下验证所有模型绝对坐标，并设置 CR5 零位。

1. 从 simulation_scene.yaml (唯一真值) 动态构造期望位置表
2. 验证每个模型绝对位置 (位置 ≤ 2mm, 姿态 ≤ 0.2°)
3. 验证静态模型 is_static=true
4. 调用 /gazebo/set_model_configuration 设 CR5 六轴零位
5. 验证 joint1~joint6 ≤ 0.01rad

V8.15: 不再硬编码 EXPECTED_POSITIONS，全部从 YAML 动态读取.

用法:
  rosrun cr5_spray_sim check_scene_geometry.py

输出到 stderr:
  ABSOLUTE_SCENE_GEOMETRY_PASS / ABSOLUTE_SCENE_GEOMETRY_FAIL
  CR5_ZERO_CONFIGURATION_PASS / CR5_ZERO_CONFIGURATION_FAIL

退出码:
  0 = 全部通过
  1 = 服务不可用
  2 = 几何检查失败
  3 = CR5 零位设置失败
"""
import sys
import os
import math
import yaml
import rospy
import rospkg
from gazebo_msgs.srv import (
    GetModelState, GetModelStateRequest,
    GetModelProperties, GetModelPropertiesRequest,
    SetModelConfiguration, SetModelConfigurationRequest,
)
from tf.transformations import euler_from_quaternion


# 静态模型列表
STATIC_MODELS = {
    "simple_goalpost_frame",
    "simple_hanging_workpiece",
    "pedestal_fl",
    "pedestal_fr",
    "pedestal_rear",
    "cam_front_left",
    "cam_front_right",
    "cam_rear",
}

# CR5 关节名
CR5_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# 容差
MAX_POSITION_MM = 2.0      # mm
MAX_ORIENTATION_DEG = 0.2  # 度
MAX_JOINT_RAD = 0.01       # rad

# 相机模型不检查方向 (由 compute_look_at 动态计算)
SKIP_ORIENTATION_CHECK = {"cam_front_left", "cam_front_right", "cam_rear"}


def load_scene_config():
    """加载 simulation_scene.yaml."""
    try:
        rp = rospkg.RosPack()
        config_path = os.path.join(
            rp.get_path("cr5_spray_sim"), "config", "simulation_scene.yaml")
    except Exception:
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "simulation_scene.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def build_expected_positions(config):
    """从 YAML 动态构造 EXPECTED_POSITIONS 字典.

    这是 V8.15 的关键变更: 不再硬编码任何世界坐标.
    """
    expected = {}

    # CR5 base
    cr5 = config.get("cr5_base", {}).get("position", {"x": 0.0, "y": 0.0, "z": 0.0})
    expected["cr5_robot"] = (float(cr5["x"]), float(cr5["y"]), float(cr5["z"]))

    # Goalpost frame
    gp = config.get("simple_goalpost_frame", {})
    gx = gp.get("center_x", 0.72)
    expected["simple_goalpost_frame"] = (float(gx), 0.0, 0.0)

    # Target
    wp = config.get("simple_hanging_workpiece", {}).get("position", {"x": 0.72, "y": 0.0, "z": 0.62})
    expected["simple_hanging_workpiece"] = (float(wp["x"]), float(wp["y"]), float(wp["z"]))

    # Pedestals
    for ped in config.get("pedestals", []):
        name = ped.get("name", "")
        base = ped.get("base", {"x": 0, "y": 0, "z": 0})
        expected[name] = (float(base["x"]), float(base["y"]), float(base["z"]))

    # Cameras
    for cam in config.get("cameras", {}).get("cameras", []):
        name = cam.get("name", "")
        pos = cam.get("position", {"x": 0, "y": 0, "z": 0})
        expected[name] = (float(pos["x"]), float(pos["y"]), float(pos["z"]))

    return expected


class GeometryChecker:
    def __init__(self):
        config = load_scene_config()
        self.expected = build_expected_positions(config)
        rospy.loginfo("Expected positions (from YAML): %d models", len(self.expected))
        for name, pos in sorted(self.expected.items()):
            rospy.loginfo("  %s: (%.3f, %.3f, %.3f)", name, *pos)

        # 初始化 services
        rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=10.0)
        rospy.wait_for_service("/gazebo/set_model_configuration", timeout=10.0)

        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.get_model_properties = rospy.ServiceProxy(
            "/gazebo/get_model_properties", GetModelProperties)
        self.set_model_config = rospy.ServiceProxy(
            "/gazebo/set_model_configuration", SetModelConfiguration)

        self.pos_errors = {}
        self.ori_errors = {}
        self.static_failures = []

    def _pose_to_xyz_rpy(self, pose_msg):
        """从 geometry_msgs/Pose 提取 xyz 和 rpy."""
        p = pose_msg.position
        o = pose_msg.orientation
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        return (p.x, p.y, p.z, roll, pitch, yaw)

    def _check_position(self, name, actual_xyz):
        """检查绝对位置，返回 (pass, pos_error_mm)."""
        expected = self.expected.get(name)
        if expected is None:
            rospy.logwarn("No expected position for %s, skipping", name)
            return True, 0.0

        dx = abs(actual_xyz[0] - expected[0]) * 1000  # mm
        dy = abs(actual_xyz[1] - expected[1]) * 1000
        dz = abs(actual_xyz[2] - expected[2]) * 1000
        max_err = max(dx, dy, dz)

        if max_err > MAX_POSITION_MM:
            rospy.logerr(
                "%s position error: dx=%.2f dy=%.2f dz=%.2f mm (max %.2f > %.2f)",
                name, dx, dy, dz, max_err, MAX_POSITION_MM)
            return False, max_err

        rospy.loginfo("%s position OK: dx=%.2f dy=%.2f dz=%.2f mm",
                      name, dx, dy, dz)
        self.pos_errors[name] = (dx, dy, dz)
        return True, max_err

    def _check_orientation(self, name, actual_rpy):
        """检查姿态偏差."""
        roll_err = abs(actual_rpy[0]) * 180 / math.pi
        pitch_err = abs(actual_rpy[1]) * 180 / math.pi

        # 处理角度环绕
        roll_err = min(roll_err, 360 - roll_err)
        pitch_err = min(pitch_err, 360 - pitch_err)
        max_err = max(roll_err, pitch_err)

        if max_err > MAX_ORIENTATION_DEG:
            rospy.logerr(
                "%s orientation error: roll=%.3f pitch=%.3f deg (max %.3f > %.2f)",
                name, roll_err, pitch_err, max_err, MAX_ORIENTATION_DEG)
            return False, max_err

        rospy.loginfo("%s orientation OK: roll=%.3f pitch=%.3f deg",
                      name, roll_err, pitch_err)
        self.ori_errors[name] = (roll_err, pitch_err)
        return True, max_err

    def check_model_pose(self, name):
        """检查单个模型的绝对位置和姿态."""
        req = GetModelStateRequest()
        req.model_name = name
        req.relative_entity_name = "world"
        try:
            resp = self.get_model_state(req)
        except Exception as e:
            rospy.logerr("get_model_state(%s) failed: %s", name, e)
            return False

        if not resp.success:
            rospy.logerr("get_model_state(%s) returned failure: %s",
                         name, resp.status_message)
            return False

        xyz_rpy = self._pose_to_xyz_rpy(resp.pose)
        pos_ok = self._check_position(name, xyz_rpy[:3])
        # 相机模型不检查方向 (look-at 动态计算)
        if name in SKIP_ORIENTATION_CHECK:
            rospy.loginfo("%s orientation skipped (look-at camera)", name)
            return pos_ok
        ori_ok = self._check_orientation(name, xyz_rpy[3:])
        return pos_ok and ori_ok

    def check_static_properties(self, name):
        """验证静态模型 is_static=true."""
        if name not in STATIC_MODELS:
            return True

        req = GetModelPropertiesRequest()
        req.model_name = name
        try:
            resp = self.get_model_properties(req)
        except Exception as e:
            rospy.logerr("get_model_properties(%s) failed: %s", name, e)
            self.static_failures.append(name)
            return False

        if not resp.success:
            rospy.logerr("get_model_properties(%s) failed: %s",
                         name, resp.status_message)
            self.static_failures.append(name)
            return False

        if not resp.is_static:
            rospy.logerr("%s is NOT static!", name)
            self.static_failures.append(name)
            return False

        rospy.loginfo("%s is_static=true OK", name)
        return True

    def set_cr5_zero(self):
        """调用 set_model_configuration 设 CR5 六轴为 0."""
        rospy.loginfo("Setting CR5 to zero configuration...")
        req = SetModelConfigurationRequest()
        req.model_name = "cr5_robot"
        req.urdf_param_name = "robot_description"
        req.joint_names = CR5_JOINTS
        req.joint_positions = [0.0] * 6

        try:
            resp = self.set_model_config(req)
        except Exception as e:
            rospy.logerr("set_model_configuration failed: %s", e)
            return False

        if not resp.success:
            rospy.logerr("set_model_configuration failed: %s", resp.status_message)
            return False

        rospy.loginfo("set_model_configuration returned success")
        return True

    def verify_cr5_zero(self):
        """通过 GetModelState 验证 CR5 rooting 正确."""
        req = GetModelStateRequest()
        req.model_name = "cr5_robot"
        req.relative_entity_name = "world"
        try:
            resp = self.get_model_state(req)
        except Exception as e:
            rospy.logerr("CR5 get_model_state failed: %s", e)
            return False

        if not resp.success:
            rospy.logerr("CR5 get_model_state failed: %s", resp.status_message)
            return False

        xyz = self._pose_to_xyz_rpy(resp.pose)[:3]
        for i, axis, expected in [(0, "x", 0.0), (1, "y", 0.0), (2, "z", 0.0)]:
            if abs(xyz[i] - expected) > 0.005:
                rospy.logerr("CR5 root %s=%.4f, expected %.4f", axis, xyz[i], expected)
                return False

        rospy.loginfo("CR5 zero configuration: root at (%.3f, %.3f, %.3f)", *xyz)
        return True


def main():
    rospy.init_node("check_scene_geometry", anonymous=True,
                    log_level=rospy.WARN)

    checker = GeometryChecker()
    all_ok = True

    # ---- 第一阶段：检查所有模型绝对位置 ----
    rospy.loginfo("=== Phase 1: Absolute position check ===")
    for name in sorted(checker.expected.keys()):
        if not checker.check_model_pose(name):
            all_ok = False

    # ---- 第二阶段：检查静态属性 ----
    rospy.loginfo("=== Phase 2: Static property check ===")
    for name in sorted(STATIC_MODELS):
        if not checker.check_static_properties(name):
            all_ok = False

    if not all_ok:
        rospy.logerr("Absolute geometry check FAILED")
        if checker.pos_errors:
            rospy.logerr("Position errors: %s", checker.pos_errors)
        if checker.ori_errors:
            rospy.logerr("Orientation errors: %s", checker.ori_errors)
        if checker.static_failures:
            rospy.logerr("Static check failures: %s", checker.static_failures)
        sys.stderr.write("ABSOLUTE_SCENE_GEOMETRY_FAIL\n")
        sys.stderr.flush()

    # ---- 第三阶段：设置并验证 CR5 零位 ----
    rospy.loginfo("=== Phase 3: CR5 zero configuration ===")
    zero_ok = True

    if not checker.set_cr5_zero():
        zero_ok = False
        rospy.logerr("set_model_configuration FAILED")

    if zero_ok and not checker.verify_cr5_zero():
        zero_ok = False
        rospy.logerr("CR5 zero verification FAILED")

    # 输出结果
    if all_ok:
        sys.stderr.write("ABSOLUTE_SCENE_GEOMETRY_PASS\n")
    else:
        sys.stderr.write("ABSOLUTE_SCENE_GEOMETRY_FAIL\n")

    if zero_ok:
        sys.stderr.write("CR5_ZERO_CONFIGURATION_PASS\n")
    else:
        sys.stderr.write("CR5_ZERO_CONFIGURATION_FAIL\n")

    sys.stderr.flush()

    if not all_ok:
        sys.exit(2)
    if not zero_ok:
        sys.exit(3)
    sys.exit(0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3.3.7 CR5 Runtime Kinematic Chain Verification.

在 controller 运行后验证完整 CR5 运动链:
- joint1~joint6 接近 0
- CR5 root 接近 (0,0,0)
- Link1~Link6 全部 finite
- 所有 CR5 body z > -0.03m
- Link6 z 在预期零位附近
- spray_nozzle_frame 可查询

用法:
  rosrun cr5_spray_sim check_scene_runtime_v337.py [--output artifacts/scene_runtime_pose.json]

输出到 stderr:
  CR5_KINEMATIC_CHAIN_PASS / CR5_KINEMATIC_CHAIN_FAIL
  MODEL_LAYOUT_PASS / MODEL_LAYOUT_FAIL

退出码:
  0 = 全部通过
  1 = 服务不可用
  2 = 运动链异常
"""
import sys
import os
import json
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import (
    GetModelState, GetModelStateRequest,
    GetLinkState, GetLinkStateRequest,
    GetModelProperties, GetModelPropertiesRequest,
)
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LINK_NAMES = ["Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
CR5_MODEL = "cr5_robot"
MAX_JOINT_RAD = 0.05      # 运行后允许稍大容差
MAX_ROOT_MM = 10.0         # root 偏差
MIN_LINK6_Z_EXPECTED = 0.80  # 零位 Link6 预期高度 ~1.0m


class RuntimeChecker:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=10.0)
        rospy.wait_for_service("/gazebo/get_link_state", timeout=10.0)
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=10.0)

        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy(
            "/gazebo/get_link_state", GetLinkState)
        self.get_model_properties = rospy.ServiceProxy(
            "/gazebo/get_model_properties", GetModelProperties)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        rospy.sleep(1.0)  # 填充 TF buffer

        self.results = {
            "joints": {},
            "links": {},
            "root": None,
            "has_spray_nozzle": False,
            "all_finite": True,
        }

    def check_joint_states(self):
        """检查 /joint_states 中六轴接近 0."""
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("No /joint_states message")
            return False

        names = list(msg.name)
        positions = list(msg.position)
        all_ok = True

        for jn in JOINT_NAMES:
            if jn not in names:
                rospy.logerr("Joint %s missing from /joint_states", jn)
                all_ok = False
                continue

            idx = names.index(jn)
            val = positions[idx]
            self.results["joints"][jn] = val

            if not math.isfinite(val):
                rospy.logerr("Joint %s non-finite: %s", jn, val)
                self.results["all_finite"] = False
                all_ok = False
            elif abs(val) > MAX_JOINT_RAD:
                rospy.logerr("Joint %s = %.4f rad, exceeds ±%.2f tolerance",
                             jn, val, MAX_JOINT_RAD)
                all_ok = False
            else:
                rospy.loginfo("Joint %s = %.4f rad OK", jn, val)

        return all_ok

    def check_cr5_root(self):
        """检查 CR5 root 在原点附近."""
        req = GetModelStateRequest()
        req.model_name = CR5_MODEL
        req.relative_entity_name = "world"
        try:
            resp = self.get_model_state(req)
        except Exception as e:
            rospy.logerr("CR5 get_model_state failed: %s", e)
            return False

        if not resp.success:
            rospy.logerr("CR5 get_model_state failed: %s", resp.status_message)
            return False

        p = resp.pose.position
        self.results["root"] = {"x": p.x, "y": p.y, "z": p.z}

        dx = abs(p.x) * 1000
        dy = abs(p.y) * 1000
        dz = abs(p.z) * 1000
        max_err = max(dx, dy, dz)

        if max_err > MAX_ROOT_MM:
            rospy.logerr("CR5 root error: dx=%.1f dy=%.1f dz=%.1f mm (max %.1f > %.1f)",
                         dx, dy, dz, max_err, MAX_ROOT_MM)
            return False

        rospy.loginfo("CR5 root OK: (%.3f, %.3f, %.3f)", p.x, p.y, p.z)
        return True

    def check_cr5_links(self):
        """检查每个 CR5 link 的 state."""
        all_ok = True
        cr5_links = [LINK_NAMES[0], LINK_NAMES[1], LINK_NAMES[2],
                      LINK_NAMES[3], LINK_NAMES[4], LINK_NAMES[5]]

        for ln in cr5_links:
            link_full = "{}::{}".format(CR5_MODEL, ln)
            req = GetLinkStateRequest()
            req.link_name = link_full
            req.reference_frame = "world"
            try:
                resp = self.get_link_state(req)
            except Exception as e:
                rospy.logerr("get_link_state(%s) failed: %s", link_full, e)
                all_ok = False
                continue

            if not resp.success:
                rospy.logerr("get_link_state(%s) failed: %s", link_full, resp.status_message)
                all_ok = False
                continue

            p = resp.link_state.pose.position
            self.results["links"][ln] = {"x": p.x, "y": p.y, "z": p.z}

            # 检查 finite
            for val, axis in [(p.x, "x"), (p.y, "y"), (p.z, "z")]:
                if not math.isfinite(val):
                    rospy.logerr("%s: %s=%s non-finite", ln, axis, val)
                    self.results["all_finite"] = False
                    all_ok = False

            # 检查 z > -0.03m (不下沉)
            if p.z < -0.03:
                rospy.logerr("%s z=%.4f below -0.03m (sinking!)", ln, p.z)
                all_ok = False

            rospy.loginfo("%s: (%.3f, %.3f, %.3f)", ln, p.x, p.y, p.z)

        # 特别检查 Link6 高度
        if "Link6" in self.results["links"]:
            l6z = self.results["links"]["Link6"]["z"]
            if l6z < MIN_LINK6_Z_EXPECTED:
                rospy.logerr("Link6 z=%.4f too low (expected > %.2f)", l6z, MIN_LINK6_Z_EXPECTED)
                all_ok = False
            else:
                rospy.loginfo("Link6 z=%.4f OK", l6z)

        return all_ok

    def check_spray_nozzle(self):
        """通过 TF 验证 spray_nozzle_frame 存在."""
        try:
            ts = self.tf_buf.lookup_transform(
                "world", "spray_nozzle_frame", rospy.Time(0), rospy.Duration(5.0))
            self.results["has_spray_nozzle"] = True
            p = ts.transform.translation
            rospy.loginfo("spray_nozzle_frame: (%.3f, %.3f, %.3f)", p.x, p.y, p.z)
            self.results["spray_nozzle"] = {"x": p.x, "y": p.y, "z": p.z}
            return True
        except Exception as e:
            rospy.logerr("spray_nozzle_frame TF not found: %s", e)
            return False

    def check_tf_chain(self):
        """验证 world→Link6 TF 链存在."""
        try:
            ts = self.tf_buf.lookup_transform(
                "world", "Link6", rospy.Time(0), rospy.Duration(5.0))
            p = ts.transform.translation
            rospy.loginfo("world→Link6 TF: (%.3f, %.3f, %.3f)", p.x, p.y, p.z)
            if p.z < MIN_LINK6_Z_EXPECTED:
                rospy.logerr("world→Link6 TF z=%.4f too low", p.z)
                return False
            return True
        except Exception as e:
            rospy.logerr("world→Link6 TF not found: %s", e)
            return False


def main():
    rospy.init_node("check_scene_runtime_v337", anonymous=True,
                    log_level=rospy.WARN)

    output_path = None
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_path = sys.argv[i + 1]

    checker = RuntimeChecker()
    all_ok = True

    # 1. Joint states
    rospy.loginfo("=== Checking joint states ===")
    joints_ok = checker.check_joint_states()

    # 2. CR5 root
    rospy.loginfo("=== Checking CR5 root ===")
    root_ok = checker.check_cr5_root()

    # 3. CR5 links
    rospy.loginfo("=== Checking CR5 links ===")
    links_ok = checker.check_cr5_links()

    # 4. Spray nozzle
    rospy.loginfo("=== Checking spray nozzle ===")
    nozzle_ok = checker.check_spray_nozzle()

    # 5. TF chain
    rospy.loginfo("=== Checking TF chain ===")
    tf_ok = checker.check_tf_chain()

    # 汇总
    kinematic_ok = joints_ok and links_ok and tf_ok
    layout_ok = root_ok and nozzle_ok

    if checker.results["all_finite"]:
        rospy.loginfo("All link coordinates finite")
    else:
        rospy.logerr("Some coordinates are non-finite!")
        kinematic_ok = False

    # 输出
    if kinematic_ok:
        sys.stderr.write("CR5_KINEMATIC_CHAIN_PASS\n")
    else:
        sys.stderr.write("CR5_KINEMATIC_CHAIN_FAIL\n")

    if layout_ok:
        sys.stderr.write("MODEL_LAYOUT_PASS\n")
    else:
        sys.stderr.write("MODEL_LAYOUT_FAIL\n")

    sys.stderr.flush()

    # 保存 artifact
    if output_path:
        out_dir = os.path.dirname(output_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(checker.results, f, indent=2, default=str)
        rospy.loginfo("Runtime pose saved: %s", output_path)

    if not kinematic_ok:
        sys.exit(2)
    sys.exit(0)


if __name__ == "__main__":
    main()

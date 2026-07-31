#!/usr/bin/env python3
"""
统一场景 Spawner: 从 simulation_scene.yaml 读取所有 world-level pose,
生成并 spawn goalpost + target + pedestals。

这是所有场景 world-level pose 的唯一执行入口。
禁止在其他脚本中硬编码场景坐标。

V8.15: 新增 beam-spreader contract, camera-pedestal contract 验证。
"""
import os
import sys
import math
import yaml
import rospy
import subprocess
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel


def load_scene_config():
    """加载 simulation_scene.yaml."""
    try:
        rp = rospkg.RosPack()
        config_path = os.path.join(
            rp.get_path("cr5_spray_sim"), "config", "simulation_scene.yaml")
    except Exception:
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "simulation_scene.yaml")
    config_path = os.path.abspath(config_path)
    with open(config_path) as f:
        return yaml.safe_load(f)


def get_xacro_path(filename):
    """返回 urdf 目录下 xacro 文件的绝对路径."""
    try:
        rp = rospkg.RosPack()
        return os.path.join(rp.get_path("cr5_spray_sim"), "urdf", filename)
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "urdf", filename)


def get_sdf_path(model_name, sdf_filename="model.sdf"):
    """返回 models 目录下 SDF 文件的绝对路径."""
    try:
        rp = rospkg.RosPack()
        return os.path.join(
            rp.get_path("cr5_spray_sim"), "models", model_name, sdf_filename)
    except Exception:
        return os.path.join(
            os.path.dirname(__file__), "..", "models", model_name, sdf_filename)


def run_xacro(template_path, args_dict):
    """运行 xacro 并返回生成的 URDF 字符串."""
    cmd = ["xacro", template_path]
    for k, v in args_dict.items():
        cmd.append("{}:={}".format(k, v))
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=15,
                       env={**os.environ})
    if r.returncode != 0:
        raise RuntimeError("xacro failed: {}".format(r.stderr))
    return r.stdout


def make_pose(x, y, z):
    """创建 identity 姿态的 Pose 消息."""
    return Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                orientation=Quaternion(x=0, y=0, z=0, w=1))


class SceneSpawner:
    def __init__(self):
        rospy.init_node("spawn_calibration_scene")
        self.scene = load_scene_config()
        self.failed = []
        self.spawned = []

        # 等待 Gazebo spawn services
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=30)
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30)
        self.spawn_urdf = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.spawn_sdf = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    def _spawn_urdf(self, name, model_xml, pose):
        """Spawn URDF model, return True on success."""
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = model_xml
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = "world"
        try:
            resp = self.spawn_urdf(req)
            if resp.success:
                rospy.loginfo("Spawned %s OK", name)
                self.spawned.append(name)
                return True
            else:
                rospy.logerr("Spawn %s failed: %s", name, resp.status_message)
                self.failed.append(name)
                return False
        except Exception as e:
            rospy.logerr("Spawn %s exception: %s", name, e)
            self.failed.append(name)
            return False

    def _spawn_sdf(self, name, sdf_path, pose):
        """Spawn SDF model from file path, return True on success."""
        rospy.loginfo("Spawning %s from %s at (%.2f, %.2f, %.2f)",
                      name, sdf_path,
                      pose.position.x, pose.position.y, pose.position.z)
        req = SpawnModelRequest()
        req.model_name = name
        req.model_xml = open(sdf_path).read()
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = "world"
        try:
            resp = self.spawn_sdf(req)
            if resp.success:
                rospy.loginfo("Spawned %s OK", name)
                self.spawned.append(name)
                return True
            else:
                rospy.logerr("Spawn %s failed: %s", name, resp.status_message)
                self.failed.append(name)
                return False
        except Exception as e:
            rospy.logerr("Spawn %s exception: %s", name, e)
            self.failed.append(name)
            return False

    def spawn_goalpost(self):
        """从 YAML 生成并 spawn 门架."""
        gp = self.scene.get("simple_goalpost_frame", {})
        center_x = gp.get("center_x", 0.72)
        post_y = gp.get("post_y", [-0.62, 0.62])
        height = gp.get("height", 1.29)
        beam_len_y = gp.get("top_beam_length_y", 1.28)
        profile = gp.get("profile_size", 0.05)
        base_plate = gp.get("base_plate", [0.24, 0.18, 0.025])

        rospy.loginfo("Goalpost: x=%.2f height=%.2f post_y=±%.2f beam=%.2f",
                      center_x, height, abs(post_y[0]), beam_len_y)

        xacro_args = {
            "post_y": abs(post_y[0]),
            "height": height,
            "beam_len_y": beam_len_y,
            "profile": profile,
            "base_sx": base_plate[0],
            "base_sy": base_plate[1],
            "base_sz": base_plate[2],
        }
        model_xml = run_xacro(get_xacro_path("goalpost_frame.xacro"), xacro_args)
        pose = make_pose(center_x, 0, 0)
        return self._spawn_urdf("simple_goalpost_frame", model_xml, pose)

    def spawn_target(self):
        """从 YAML 生成并 spawn 标定靶."""
        wp = self.scene.get("simple_hanging_workpiece", {})
        pos = wp.get("position", {"x": 0.72, "y": 0.0, "z": 0.62})
        tx, ty, tz = float(pos["x"]), float(pos["y"]), float(pos["z"])

        rospy.loginfo("Target: (%.2f, %.2f, %.2f)", tx, ty, tz)

        sdf_path = get_sdf_path("calibration_target")
        if not os.path.isfile(sdf_path):
            rospy.logerr("Target SDF not found: %s", sdf_path)
            self.failed.append("simple_hanging_workpiece")
            return False

        # SDF 内部有 <pose>0 0 0 0 0 0</pose>，所以 spawn 时 pose 直接设置世界坐标
        pose = make_pose(tx, ty, tz)
        return self._spawn_sdf("simple_hanging_workpiece", sdf_path, pose)

    def spawn_pedestals(self):
        """从 YAML 生成并 spawn 所有相机支架."""
        pedestals = self.scene.get("pedestals", [])
        xacro_template = get_xacro_path("camera_pedestal.xacro")
        all_ok = True

        for ped in pedestals:
            name = ped.get("name", "pedestal_unknown")
            base = ped.get("base", {"x": 0, "y": 0, "z": 0})
            height = ped.get("height", 1.15)
            arm_len = ped.get("arm_length", 0.10)

            rospy.loginfo("Pedestal %s: base=(%.2f,%.2f) h=%.2f arm=%.2f",
                          name, base["x"], base["y"], height, arm_len)

            xacro_args = {
                "pedestal_height": height,
                "arm_length": arm_len,
            }
            model_xml = run_xacro(xacro_template, xacro_args)
            pose = make_pose(base["x"], base["y"], base["z"])
            if not self._spawn_urdf(name, model_xml, pose):
                all_ok = False

        return all_ok

    def verify_contract(self):
        """验证场景 geometry contract (非阻塞, 仅输出诊断)."""
        rospy.loginfo("=== Scene Contract Diagnostic ===")

        # 1. beam-spreader 匹配
        gp = self.scene.get("simple_goalpost_frame", {})
        height = gp.get("height", 1.29)
        profile = gp.get("profile_size", 0.05)
        # beam bottom: top beam center at height - profile/2, box half = profile/2
        # beam_bottom = center - half = (height - profile/2) - profile/2 = height - profile
        beam_bottom_z = height - profile  # 门架局部坐标, world_z = spawn_z + this

        wp = self.scene.get("simple_hanging_workpiece", {})
        pos = wp.get("position", {"x": 0.72, "y": 0.0, "z": 0.62})
        target_z = float(pos["z"])
        spreader_top_z = target_z + 0.620  # 从 model.sdf: spreader_center=0.605, half_height=0.015

        delta = abs(beam_bottom_z - spreader_top_z) * 1000  # mm
        rospy.loginfo("Beam bottom: %.3f m, Spreader top: %.3f m, delta: %.1f mm",
                      beam_bottom_z, spreader_top_z, delta)
        if delta <= 5.0:
            rospy.loginfo("BEAM_SPREADER_CONTRACT_PASS (%.1f mm <= 5mm)", delta)
        else:
            rospy.logwarn("BEAM_SPREADER_CONTRACT_WARN: delta=%.1f mm > 5mm", delta)

        # 2. camera-pedestal 安装匹配
        cameras = self.scene.get("cameras", {}).get("cameras", [])
        pedestals = self.scene.get("pedestals", [])
        ped_map = {p["name"]: p for p in pedestals}

        # 映射: camera name → pedestal name
        cam_to_ped = {
            "cam_front_left": "pedestal_fl",
            "cam_front_right": "pedestal_fr",
            "cam_rear": "pedestal_rear",
        }

        for cam in cameras:
            cname = cam["name"]
            cpos = cam["position"]
            ped_name = cam_to_ped.get(cname)
            if ped_name is None or ped_name not in ped_map:
                rospy.logwarn("No pedestal mapping for %s", cname)
                continue

            ped = ped_map[ped_name]
            base = ped["base"]
            height = ped.get("height", 1.15)
            arm_len = ped.get("arm_length", 0.10)
            post_size = 0.04  # from camera_pedestal.xacro default

            # bracket tip (世界坐标)
            bx = base["x"] + arm_len
            by = base["y"]
            bz = height - post_size / 2.0

            cx, cy, cz = float(cpos["x"]), float(cpos["y"]), float(cpos["z"])
            dist = math.sqrt((cx - bx)**2 + (cy - by)**2 + (cz - bz)**2) * 1000

            rospy.loginfo("%s: camera(%.2f,%.2f,%.2f) bracket(%.2f,%.2f,%.2f) dist=%.1f mm",
                          cname, cx, cy, cz, bx, by, bz, dist)
            if dist <= 20.0:
                rospy.loginfo("CAMERA_MOUNT_PASS: %s (%.1f mm <= 20mm)", cname, dist)
            else:
                rospy.logwarn("CAMERA_MOUNT_WARN: %s dist=%.1f mm > 20mm", cname, dist)

        rospy.loginfo("=== Scene Contract Diagnostic Done ===")

    def run(self):
        rospy.loginfo("=== Spawning Calibration Scene from YAML ===")

        if not self.spawn_goalpost():
            rospy.logerr("Goalpost spawn FAILED")
        if not self.spawn_target():
            rospy.logerr("Target spawn FAILED")
        if not self.spawn_pedestals():
            rospy.logerr("Pedestal spawn FAILED")

        self.verify_contract()

        if self.failed:
            rospy.logerr("SCENE_SPAWN_FAIL: %s", self.failed)
            sys.exit(1)
        else:
            rospy.loginfo("SCENE_SPAWN_PASS: %s", self.spawned)


if __name__ == "__main__":
    spawner = SceneSpawner()
    spawner.run()
    rospy.spin()

#!/usr/bin/env python3
"""
Fixed Camera Spawner: 从 scene_v31.yaml 读取相机坐标，计算 look-at 并生成。

相机坐标轴约定:
  - Gazebo camera link: +X 向前 (镜头方向), +Y 向左, +Z 向上
  - ROS optical frame: +Z 向前 (拍摄方向), +X 向右, -Y 向下
  - link → optical: rpy = (-π/2, 0, -π/2)
"""
import os
import sys
import math
import yaml
import rospy
import subprocess
import tempfile
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel


def load_scene_config():
    config_path = rospy.get_param("~scene_config", "")
    if not config_path:
        try:
            import rospkg
            config_path = os.path.join(
                rospkg.RosPack().get_path("cr5_spray_sim"),
                "config", "scene_v31.yaml")
        except Exception:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "config", "scene_v31.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def compute_camera_look_at(cam_pos, target_pos, roll_offset_deg=0.0):
    """
    纯 Python 实现: 计算相机 look-at 的 roll/pitch/yaw.

    返回 dict 包含:
      roll, pitch, yaw: Gazebo camera link 姿态 (rad)
      distance_m: 相机到目标距离
      optical_z_angle_error_deg: 光学 +Z 偏离目标的角度
      image_up_vs_world_up_deg: 图像上方向偏离世界 +Z 的角度
    """
    cam = np.array(cam_pos, dtype=np.float64)
    tgt = np.array(target_pos, dtype=np.float64)

    # Direction from camera to target
    d = tgt - cam
    dist = float(np.linalg.norm(d))
    if dist < 1e-6:
        return None
    d = d / dist

    # Camera link X should point at the target
    cam_x = d
    # Camera link Z should be as close to world Z as possible
    world_z = np.array([0.0, 0.0, 1.0])
    # cam_y = world_z × cam_x (normalized)
    cam_y = np.cross(world_z, cam_x)
    cam_y_norm = float(np.linalg.norm(cam_y))
    if cam_y_norm < 1e-9:
        cam_y = np.array([0.0, 1.0, 0.0])
    else:
        cam_y = cam_y / cam_y_norm
    # cam_z = cam_x × cam_y
    cam_z = np.cross(cam_x, cam_y)
    cam_z_norm = float(np.linalg.norm(cam_z))
    if cam_z_norm > 1e-9:
        cam_z = cam_z / cam_z_norm

    # Build rotation matrix R = [cam_x | cam_y | cam_z] (columns)
    R = np.column_stack([cam_x, cam_y, cam_z])

    # Extract Euler angles (ZYX convention used by Gazebo)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # roll:  around X
    # pitch: around Y
    # yaw:   around Z
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    # Apply roll offset (around camera X axis)
    roll += math.radians(roll_offset_deg)

    # Compute optical Z angle error (how well optical +Z points to target)
    # Optical +Z in world frame = R * optical_Z_local
    # optical_Z_local = [0, 0, 1] in optical frame
    # link → optical: rpy = (-π/2, 0, -π/2)
    # R_optical_to_link rotates optical frame vectors to link frame
    R_opt_to_link = np.array([
        [0, 1, 0],
        [0, 0, -1],
        [-1, 0, 0],
    ], dtype=np.float64)
    # optical +Z in world = R * R_opt_to_link * [0,0,1]
    opt_z_world = R @ R_opt_to_link @ np.array([0, 0, 1])
    # angle between optical +Z and target direction
    cos_angle = float(np.dot(opt_z_world, d))
    cos_angle = max(-1.0, min(1.0, cos_angle))
    opt_err = float(math.degrees(math.acos(cos_angle)))

    # Image up direction error
    # optical +Y (image up = -optical_Y) vs world +Z
    opt_y_world = R @ R_opt_to_link @ np.array([0, 1, 0])
    # image up = -optical_y
    img_up = -opt_y_world
    cos_up = float(np.dot(img_up, world_z))
    cos_up = max(-1.0, min(1.0, cos_up))
    up_err = float(math.degrees(math.acos(abs(cos_up))))

    return {
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
        "distance_m": dist,
        "optical_z_angle_error_deg": opt_err,
        "image_up_vs_world_up_deg": up_err,
    }


def make_camera_xacro(name, profile):
    """Generate D455-like camera URDF (same as V2)."""
    p = profile
    return f'''<?xml version="1.0"?>
<robot name="{name}" xmlns:xacro="http://ros.org/wiki/xacro">
  <material name="{name}_body"><color rgba="0.25 0.25 0.25 1"/></material>
  <material name="{name}_lens"><color rgba="0.05 0.05 0.05 1"/></material>
  <material name="{name}_axis"><color rgba="1 0.15 0.15 1"/></material>
  <material name="{name}_up_mark"><color rgba="0.15 0.7 0.15 1"/></material>

  <link name="{name}_link">
    <inertial><mass value="0.05"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
    <visual name="body"><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.03 0.09 0.025"/></geometry><material name="{name}_body"/></visual>
    <visual name="lens"><origin xyz="0.015 0 0" rpy="0 1.5708 0"/><geometry><cylinder radius="0.010" length="0.004"/></geometry><material name="{name}_lens"/></visual>
    <visual name="axis"><origin xyz="0.025 0 0" rpy="0 1.5708 0"/><geometry><cylinder radius="0.002" length="0.05"/></geometry><material name="{name}_axis"/></visual>
    <visual name="up_dot"><origin xyz="0 0 0.013" rpy="0 0 0"/><geometry><cylinder radius="0.004" length="0.003"/></geometry><material name="{name}_up_mark"/></visual>
  </link>

  <link name="{name}_color_optical_frame"/>
  <joint name="{name}_color_optical_joint" type="fixed">
    <parent link="{name}_link"/><child link="{name}_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <link name="{name}_depth_optical_frame"/>
  <joint name="{name}_depth_optical_joint" type="fixed">
    <parent link="{name}_link"/><child link="{name}_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <gazebo reference="{name}_link">
    <self_collide>0</self_collide>
    <sensor name="{name}color" type="camera">
      <camera name="{name}"><horizontal_fov>1.2112585</horizontal_fov>
        <image><width>{p["color_width"]}</width><height>{p["color_height"]}</height><format>RGB_INT8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
      </camera><always_on>1</always_on><update_rate>{p["fps"]}</update_rate><visualize>false</visualize>
    </sensor>
    <sensor name="{name}depth" type="depth">
      <camera name="{name}"><horizontal_fov>1.2112585</horizontal_fov>
        <image><width>{p["depth_width"]}</width><height>{p["depth_height"]}</height><format>R_FLOAT32</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
      </camera><always_on>1</always_on><update_rate>{p["fps"]}</update_rate><visualize>false</visualize>
    </sensor>
    <sensor name="{name}ired1" type="camera">
      <camera name="{name}"><horizontal_fov>1.48702</horizontal_fov>
        <image><width>160</width><height>120</height><format>L_INT8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
      </camera><always_on>1</always_on><update_rate>2</update_rate><visualize>false</visualize>
    </sensor>
    <sensor name="{name}ired2" type="camera">
      <camera name="{name}"><horizontal_fov>1.48702</horizontal_fov>
        <image><width>160</width><height>120</height><format>L_INT8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
      </camera><always_on>1</always_on><update_rate>2</update_rate><visualize>false</visualize>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="{name}" filename="librealsense_gazebo_plugin.so">
      <prefix>{name}</prefix>
      <colorFrameName>{name}_color_optical_frame</colorFrameName>
      <depthFrameName>{name}_depth_optical_frame</depthFrameName>
    </plugin>
  </gazebo>

  <gazebo><static>true</static></gazebo>
</robot>'''


class FixedCameraSpawner:
    def __init__(self):
        rospy.init_node("spawn_fixed_cameras")
        self.scene = load_scene_config()
        self.failed = 0

        profile_name = rospy.get_param("~camera_profile", "vm")
        profiles = self.scene.get("cameras_v31", {})
        self.profile = profiles.get(
            "vm_profile" if profile_name == "vm" else "quality_profile",
            {"color_width": 424, "color_height": 240, "depth_width": 424,
             "depth_height": 240, "fps": 5})

        cam_cfg = profiles.get("cameras", [])
        if not cam_cfg:
            rospy.logerr("No cameras in scene_v31.yaml!")
            return

        self.target = profiles.get("target", {"x": 0.56, "y": 0, "z": 0.98})
        tgt = [self.target["x"], self.target["y"], self.target["z"]]

        rospy.loginfo("Camera target: (%.3f, %.3f, %.3f)", *tgt)

        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=30)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        rospy.loginfo("Camera profile: %s (%dx%d@%dHz), %d cameras",
                      profile_name, self.profile["color_width"],
                      self.profile["color_height"], self.profile["fps"],
                      len(cam_cfg))

        for cam in cam_cfg:
            name = cam["name"]
            pos = [cam["position"]["x"], cam["position"]["y"], cam["position"]["z"]]
            roll_off = cam.get("roll_offset_deg", 0.0)
            self.spawn_one(name, pos, tgt, roll_offset_deg=roll_off)

        if self.failed > 0:
            rospy.logerr("FIXED_CAMERAS_SPAWN_FAIL: %d/%d cameras failed",
                         self.failed, len(cam_cfg))
            sys.exit(1)
        else:
            rospy.loginfo("FIXED_CAMERAS_SPAWN_PASS: %d cameras", len(cam_cfg))

    def spawn_one(self, name, pos, tgt, roll_offset_deg=0.0):
        rpy_data = compute_camera_look_at(pos, tgt, roll_offset_deg=roll_offset_deg)
        if not rpy_data:
            rospy.logerr("look-at failed for %s", name)
            self.failed += 1; return

        roll, pitch, yaw = rpy_data["roll"], rpy_data["pitch"], rpy_data["yaw"]
        err = rpy_data["optical_z_angle_error_deg"]
        up_err = rpy_data.get("image_up_vs_world_up_deg", 999)
        if err > 1.0:
            rospy.logwarn("%s: look-at error %.4f deg > 1.0 (tolerated)", name, err)

        xacro = make_camera_xacro(name, self.profile)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False) as f:
            f.write(xacro); xf = f.name
        try:
            r = subprocess.run(["xacro", xf], capture_output=True, text=True, timeout=10,
                               env={**os.environ})
            if r.returncode != 0:
                rospy.logerr("xacro failed for %s: %s", name, r.stderr)
                self.failed += 1; return
            model_xml = r.stdout
        finally:
            os.unlink(xf)

        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        try:
            resp = self.spawn(name, model_xml, "", pose, "world")
            if resp.success:
                rospy.loginfo(
                    "Spawned %s: dist=%.2fm look-err=%.4f° up-err=%.1f° "
                    "rpy=(%.3f,%.3f,%.3f)",
                    name, rpy_data["distance_m"], err, up_err,
                    roll, pitch, yaw)
            else:
                rospy.logerr("Spawn %s failed: %s", name, resp.status_message)
                self.failed += 1
        except Exception as e:
            rospy.logerr("Spawn %s error: %s", e)
            self.failed += 1


if __name__ == "__main__":
    FixedCameraSpawner()
    rospy.spin()

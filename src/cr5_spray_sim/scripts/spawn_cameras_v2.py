#!/usr/bin/env python3
"""
V2 Camera Spawner: 3 static D455-like cameras with look-at poses.
Each camera is a static Gazebo model; pose applied only once via spawn.
"""
import os
import sys
import yaml
import rospy
import subprocess
import tempfile
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import SpawnModel

# Camera positions from scene_v2.yaml
CAMERAS = [
    {"name": "cam_front_left",  "x": 0.34, "y": -0.58, "z": 1.22},
    {"name": "cam_front_right", "x": 0.34, "y":  0.58, "z": 1.22},
    {"name": "cam_rear",        "x": 1.28, "y":  0.00, "z": 1.12},
]
TARGET = {"x": 0.72, "y": 0.0, "z": 0.88}

VM_PROFILE = {"color_width": 424, "color_height": 240, "depth_width": 424,
              "depth_height": 240, "fps": 5}
QUALITY_PROFILE = {"color_width": 640, "color_height": 480, "depth_width": 640,
                   "depth_height": 480, "fps": 10}


def compute_look_at(cam_pos, target_pos):
    """Compute RPY so optical +Z points to target. Uses compute_look_at module."""
    script = os.path.join(os.path.dirname(__file__), "compute_look_at.py")
    cmd = [
        sys.executable, script,
        "--cam-x", str(cam_pos[0]),
        "--cam-y", str(cam_pos[1]),
        "--cam-z", str(cam_pos[2]),
        "--target-x", str(target_pos[0]),
        "--target-y", str(target_pos[1]),
        "--target-z", str(target_pos[2]),
        "--yaml",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    if result.returncode != 0:
        rospy.logerr("look-at failed: %s", result.stderr)
        return None
    return yaml.safe_load(result.stdout)


class CameraSpawnerV2:
    def __init__(self):
        rospy.init_node("spawn_cameras_v2")
        profile_name = rospy.get_param("~camera_profile", "vm")
        self.profile = VM_PROFILE if profile_name == "vm" else QUALITY_PROFILE
        rospy.loginfo("Camera profile: %s (%dx%d@%dHz)",
                      profile_name, self.profile["color_width"],
                      self.profile["color_height"], self.profile["fps"])

        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=30.0)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        self.spawn_all()

    def spawn_all(self):
        for cam in CAMERAS:
            self.spawn_one(cam)

    def spawn_one(self, cam):
        name = cam["name"]
        pos = [cam["x"], cam["y"], cam["z"]]
        tgt = [TARGET["x"], TARGET["y"], TARGET["z"]]

        rpy_data = compute_look_at(pos, tgt)
        if rpy_data is None:
            rospy.logerr("Skipping %s: look-at failed", name)
            return

        roll, pitch, yaw = rpy_data["roll"], rpy_data["pitch"], rpy_data["yaw"]
        rospy.loginfo("%s: dist=%.2fm, angle_err=%.4f deg, rpy=(%.3f,%.3f,%.3f)",
                      name, rpy_data["distance_m"],
                      rpy_data["optical_z_angle_error_deg"],
                      roll, pitch, yaw)

        # Generate camera xacro: sensor origin=0, static model
        xacro = self._make_camera_xacro(name)
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False) as f:
            f.write(xacro)
            xacro_file = f.name

        try:
            result = subprocess.run(
                ["xacro", xacro_file],
                capture_output=True, text=True, timeout=10,
                env={**os.environ,
                     "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", "")})
            if result.returncode != 0:
                rospy.logerr("xacro failed for %s: %s", name, result.stderr)
                return
            model_xml = result.stdout
        finally:
            os.unlink(xacro_file)

        # Pose
        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose(position=Point(x=cam["x"], y=cam["y"], z=cam["z"]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

        try:
            resp = self.spawn(name, model_xml, "", pose, "world")
            if resp.success:
                rospy.loginfo("Spawned: %s", name)
            else:
                rospy.logerr("Spawn failed for %s: %s", name, resp.status_message)
        except Exception as e:
            rospy.logerr("Spawn error for %s: %s", name, e)

    def _make_camera_xacro(self, name):
        p = self.profile
        return f'''<?xml version="1.0"?>
<robot name="{name}" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find realsense_gazebo_description)/urdf/_d455_like.urdf.xacro"/>
  <xacro:include filename="$(find realsense_gazebo_description)/urdf/_d455_like.gazebo.xacro"/>

  <!-- Sensor origin = 0 (pose from spawn_model only) -->
  <link name="{name}_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Optical frame chain -->
  <link name="{name}_color_frame"/>
  <joint name="{name}_color_joint" type="fixed">
    <parent link="{name}_link"/>
    <child link="{name}_color_frame"/>
  </joint>

  <link name="{name}_color_optical_frame"/>
  <joint name="{name}_color_optical_joint" type="fixed">
    <parent link="{name}_color_frame"/>
    <child link="{name}_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <link name="{name}_depth_optical_frame"/>
  <joint name="{name}_depth_optical_joint" type="fixed">
    <parent link="{name}_color_frame"/>
    <child link="{name}_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  </joint>

  <!-- Gazebo sensors (color + depth + IR required by plugin) -->
  <gazebo reference="{name}_link">
    <self_collide>0</self_collide>
    <sensor name="{name}color" type="camera">
      <camera name="{name}">
        <horizontal_fov>1.2112585</horizontal_fov>
        <image>
          <width>{p["color_width"]}</width>
          <height>{p["color_height"]}</height>
          <format>RGB_INT8</format>
        </image>
        <clip><near>0.15</near><far>10.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
      </camera>
      <always_on>1</always_on>
      <update_rate>{p["fps"]}</update_rate>
      <visualize>false</visualize>
    </sensor>

    <sensor name="{name}depth" type="depth">
      <camera name="{name}">
        <horizontal_fov>1.2112585</horizontal_fov>
        <image>
          <width>{p["depth_width"]}</width>
          <height>{p["depth_height"]}</height>
          <format>R_FLOAT32</format>
        </image>
        <clip><near>0.15</near><far>10.0</far></clip>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.005</stddev></noise>
      </camera>
      <always_on>1</always_on>
      <update_rate>{p["fps"]}</update_rate>
      <visualize>false</visualize>
    </sensor>

    <sensor name="{name}ired1" type="camera">
      <camera name="{name}">
        <horizontal_fov>1.48702</horizontal_fov>
        <image><width>160</width><height>120</height><format>L_INT8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>2</update_rate>
      <visualize>false</visualize>
    </sensor>

    <sensor name="{name}ired2" type="camera">
      <camera name="{name}">
        <horizontal_fov>1.48702</horizontal_fov>
        <image><width>160</width><height>120</height><format>L_INT8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>2</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- Plugin at model level -->
  <gazebo>
    <plugin name="{name}" filename="librealsense_gazebo_plugin.so">
      <prefix>{name}</prefix>
    </plugin>
  </gazebo>

  <!-- Static: no gravity -->
  <gazebo>
    <static>true</static>
  </gazebo>
</robot>'''


if __name__ == "__main__":
    CameraSpawnerV2()
    rospy.spin()

#!/usr/bin/env python3
"""
CR5 Spray Demo: Scene Camera Spawner
从 YAML 读取相机布局，生成每台相机的 URDF 并 spawn 到 Gazebo。
每台相机具有唯一的 model name / namespace / TF frame / topic。
"""
import os
import sys
import yaml
import rospy
import subprocess
import tempfile
from gazebo_msgs.srv import SpawnModel, DeleteModel


class CameraSpawner:
    def __init__(self):
        rospy.init_node("camera_spawner")
        self.layout_file = rospy.get_param("~camera_layout", "")
        if not self.layout_file:
            rospy.logerr("camera_spawner: no camera_layout param")
            sys.exit(1)

        with open(self.layout_file) as f:
            self.layout = yaml.safe_load(f)

        self.cameras = self.layout.get("cameras", [])
        if not self.cameras:
            rospy.logwarn("camera_spawner: no cameras in layout")
            return

        # Wait for spawn service
        rospy.loginfo("Camera spawner: waiting for /gazebo/spawn_urdf_model...")
        rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=30.0)
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        self.delete_srv = None
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
            self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        except Exception:
            pass

        self.spawned_models = []
        self.spawn_all()

        # Keep alive for re-spawn or cleanup
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo("Camera spawner: %d cameras ready", len(self.spawned_models))
        rospy.spin()

    def spawn_all(self):
        for cam in self.cameras:
            if not cam.get("enabled", True):
                continue
            self.spawn_camera(cam)

    def spawn_camera(self, cam):
        """Generate D455-like URDF for one camera and spawn it."""
        name = cam["name"]
        parent = cam.get("parent", "world")
        pos = cam["position"]
        ori = cam.get("orientation", {"r": 0, "p": 0, "y": 0})

        # Generate URDF via xacro
        xacro_content = self._make_camera_xacro(name, parent, pos, ori, cam)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(xacro_content)
            urdf_file = f.name

        try:
            # Call xacro to process
            result = subprocess.run(
                ["xacro", urdf_file],
                capture_output=True, text=True, timeout=10,
                env={**os.environ, "ROS_PACKAGE_PATH": os.environ.get("ROS_PACKAGE_PATH", "")}
            )
            if result.returncode != 0:
                rospy.logerr("xacro failed for %s: %s", name, result.stderr)
                return False
            model_xml = result.stdout
        except Exception as e:
            rospy.logerr("xacro error for %s: %s", name, e)
            os.unlink(urdf_file)
            return False
        finally:
            os.unlink(urdf_file)

        # Spawn in Gazebo
        try:
            from geometry_msgs.msg import Pose, Point, Quaternion
            from tf.transformations import quaternion_from_euler
            pose = Pose()
            pose.position = Point(x=pos["x"], y=pos["y"], z=pos["z"])
            q = quaternion_from_euler(ori["r"], ori["p"], ori["y"])
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            resp = self.spawn_srv(name, model_xml, "",
                                  pose, "world")
            if resp.success:
                rospy.loginfo("Spawned camera: %s", name)
                self.spawned_models.append(name)
                return True
            else:
                rospy.logerr("Spawn failed for %s: %s", name, resp.status_message)
                return False
        except Exception as e:
            rospy.logerr("Spawn error for %s: %s", name, e)
            return False

    def _make_camera_xacro(self, name, parent, pos, ori, cam):
        """Generate a standalone D455-like camera xacro file content."""
        cw = cam.get("color_width", 640)
        ch = cam.get("color_height", 480)
        dw = cam.get("depth_width", 640)
        dh = cam.get("depth_height", 480)
        fps = cam.get("color_fps", 10)
        r, p, y = ori.get("r", 0), ori.get("p", 0), ori.get("y", 0)

        return f'''<?xml version="1.0"?>
<robot name="{name}" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find realsense_gazebo_description)/urdf/_d455_like.urdf.xacro"/>
  <link name="base_link"/>
  <xacro:sensor_d455_like parent="base_link" name="{name}" topics_ns="{name}"
      color_width="{cw}" color_height="{ch}" color_fps="{fps}"
      depth_width="{dw}" depth_height="{dh}" depth_fps="{fps}"
      enable_pointcloud="false" visualize="false">
    <origin xyz="{pos['x']} {pos['y']} {pos['z']}" rpy="{r} {p} {y}"/>
  </xacro:sensor_d455_like>
  <!-- Frame matching plugin frame_id convention -->
  <joint name="{name}color_joint" type="fixed">
    <parent link="{name}_color_optical_frame"/>
    <child link="{name}color"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="{name}color"/>
</robot>'''

    def cleanup(self):
        if self.delete_srv:
            for model in self.spawned_models:
                try:
                    self.delete_srv(model)
                except Exception:
                    pass


if __name__ == "__main__":
    CameraSpawner()

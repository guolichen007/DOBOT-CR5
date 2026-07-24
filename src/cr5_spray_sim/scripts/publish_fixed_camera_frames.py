#!/usr/bin/env python3
"""
固定相机 TF 发布器.

从 simulation_scene.yaml 读取相机位置和 look-at 方向,
计算与 spawn_fixed_cameras.py 完全相同的 link 位姿,
发布 world → camera_link → optical_frame 的静态 TF.

TF 树:
  world
  └── <camera>_link
      ├── <camera>_color_optical_frame  (rpy=-π/2, 0, -π/2)
      └── <camera>_depth_optical_frame  (rpy=-π/2, 0, -π/2)

这是 CaptureManager (T_world_camera) 和 PnP Gazebo truth 的必需基础设施.
"""
import os
import sys
import math
import yaml
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from cr5_spray_sim.camera_geometry import compute_camera_look_at


def load_scene_config():
    """与 spawn_fixed_cameras.py 使用相同的配置加载逻辑."""
    config_path = rospy.get_param("~scene_config", "")
    if not config_path:
        try:
            import rospkg
            config_path = os.path.join(
                rospkg.RosPack().get_path("cr5_spray_sim"),
                "config", "simulation_scene.yaml")
        except Exception:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "config",
                "simulation_scene.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def publish_camera_frames():
    """发布所有固定相机的 TF 树."""
    rospy.init_node("publish_fixed_camera_frames")

    scene = load_scene_config()
    profiles = scene.get("cameras", {})
    cam_cfg = profiles.get("cameras", [])
    target = profiles.get("target", {"x": 0.68, "y": 0, "z": 0.98})
    tgt = [target["x"], target["y"], target["z"]]

    if not cam_cfg:
        rospy.logerr("No cameras in simulation_scene.yaml!")
        sys.exit(1)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    # link → optical 固定旋转 (来自 fixed_rgbd_camera.urdf.xacro)
    # rpy = (-π/2, 0, -π/2)
    LINK_TO_OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
    link_to_optical_q = quaternion_from_euler(*LINK_TO_OPTICAL_RPY)

    for cam in cam_cfg:
        name = cam["name"]
        pos = [cam["position"]["x"], cam["position"]["y"], cam["position"]["z"]]
        roll_off = cam.get("roll_offset_deg", 0.0)

        # 使用与 spawn 完全相同的 look-at 计算
        rpy_data = compute_camera_look_at(pos, tgt, roll_offset_deg=roll_off)
        if not rpy_data:
            rospy.logerr("look-at failed for %s", name)
            sys.exit(1)

        roll, pitch, yaw = rpy_data["roll"], rpy_data["pitch"], rpy_data["yaw"]
        link_q = quaternion_from_euler(roll, pitch, yaw)

        link_frame = "{}_link".format(name)
        now = rospy.Time.now()

        # 1. world → camera_link
        t_wl = TransformStamped()
        t_wl.header.stamp = now
        t_wl.header.frame_id = "world"
        t_wl.child_frame_id = link_frame
        t_wl.transform.translation.x = pos[0]
        t_wl.transform.translation.y = pos[1]
        t_wl.transform.translation.z = pos[2]
        t_wl.transform.rotation.x = link_q[0]
        t_wl.transform.rotation.y = link_q[1]
        t_wl.transform.rotation.z = link_q[2]
        t_wl.transform.rotation.w = link_q[3]
        transforms.append(t_wl)
        rospy.loginfo("%s: world → %s pos=(%.3f,%.3f,%.3f) "
                      "rpy=(%.3f°,%.3f°,%.3f°) dist=%.2fm "
                      "look-err=%.4f° up-err=%.1f°",
                      name, link_frame, pos[0], pos[1], pos[2],
                      math.degrees(roll), math.degrees(pitch),
                      math.degrees(yaw),
                      rpy_data["distance_m"],
                      rpy_data["optical_z_angle_error_deg"],
                      rpy_data.get("image_up_vs_world_up_deg", 999))

        # 2. camera_link → color_optical_frame
        t_lco = TransformStamped()
        t_lco.header.stamp = now
        t_lco.header.frame_id = link_frame
        t_lco.child_frame_id = "{}_color_optical_frame".format(name)
        t_lco.transform.translation.x = 0.0
        t_lco.transform.translation.y = 0.0
        t_lco.transform.translation.z = 0.0
        t_lco.transform.rotation.x = link_to_optical_q[0]
        t_lco.transform.rotation.y = link_to_optical_q[1]
        t_lco.transform.rotation.z = link_to_optical_q[2]
        t_lco.transform.rotation.w = link_to_optical_q[3]
        transforms.append(t_lco)

        # 3. camera_link → depth_optical_frame
        t_ldo = TransformStamped()
        t_ldo.header.stamp = now
        t_ldo.header.frame_id = link_frame
        t_ldo.child_frame_id = "{}_depth_optical_frame".format(name)
        t_ldo.transform.translation.x = 0.0
        t_ldo.transform.translation.y = 0.0
        t_ldo.transform.translation.z = 0.0
        t_ldo.transform.rotation.x = link_to_optical_q[0]
        t_ldo.transform.rotation.y = link_to_optical_q[1]
        t_ldo.transform.rotation.z = link_to_optical_q[2]
        t_ldo.transform.rotation.w = link_to_optical_q[3]
        transforms.append(t_ldo)

    broadcaster.sendTransform(transforms)
    rospy.loginfo("FIXED_CAMERA_TF_PUBLISH_PASS: %d cameras, %d transforms",
                  len(cam_cfg), len(transforms))

    # 保持节点存活 (StaticTransformBroadcaster 需要 latch)
    rospy.spin()


if __name__ == "__main__":
    publish_camera_frames()

#!/usr/bin/env python3
"""
V2.2 Scene TF Broadcaster: publish static transforms for gantry, object, cameras.
Camera poses computed dynamically via compute_look_at (stable-horizon algorithm).
"""
import os
import sys
import yaml
import rospy
import subprocess
import tf2_ros
from geometry_msgs.msg import TransformStamped


def compute_look_at(cam_pos, target_pos, roll_offset_deg=0.0):
    """Call compute_look_at.py subprocess, return RPY dict."""
    script = os.path.join(os.path.dirname(__file__), "compute_look_at.py")
    cmd = [sys.executable, script,
           "--cam-x", str(cam_pos[0]), "--cam-y", str(cam_pos[1]),
           "--cam-z", str(cam_pos[2]),
           "--target-x", str(target_pos[0]), "--target-y", str(target_pos[1]),
           "--target-z", str(target_pos[2]),
           "--roll-offset-deg", str(roll_offset_deg),
           "--yaml"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    if r.returncode != 0:
        return None
    return yaml.safe_load(r.stdout)


def make_tf(parent, child, x, y, z, r, p, yaw):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(r, p, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def main():
    rospy.init_node("scene_tf_v2")
    br = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(1)

    # Load config
    try:
        import rospkg
        config_path = os.path.join(
            rospkg.RosPack().get_path("cr5_spray_sim"),
            "config", "scene_v2.yaml")
    except Exception:
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "scene_v2.yaml")
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    cam_cfg = cfg.get("cameras_v2", {})
    cameras = cam_cfg.get("cameras", [])
    target = cam_cfg.get("target", {"x": 0.72, "y": 0.0, "z": 0.92})
    tgt = [target["x"], target["y"], target["z"]]

    gantry_x = cfg.get("gantry_v2", {}).get("center_x", 0.78)
    obj_pos = cfg.get("object_v2", {}).get("position", {"x": 0.72, "y": 0.0, "z": 0.88})

    tfs = []

    # Gantry TF
    tfs.append(make_tf("world", "gantry_v2_base", gantry_x, 0, 0, 0, 0, 0))

    # Object TF
    tfs.append(make_tf("world", "object_frame",
                       obj_pos["x"], obj_pos["y"], obj_pos["z"], 0, 0, 0))

    # Camera TFs (computed dynamically)
    rospy.loginfo("Computing camera poses (stable-horizon algorithm)...")
    for cam in cameras:
        name = cam["name"]
        pos = [cam["position"]["x"], cam["position"]["y"], cam["position"]["z"]]
        roll_off = cam.get("roll_offset_deg", 0.0)

        rpy = compute_look_at(pos, tgt, roll_offset_deg=roll_off)
        if not rpy:
            rospy.logerr("look-at failed for %s", name)
            continue

        err = rpy["optical_z_angle_error_deg"]
        up_err = rpy.get("image_up_vs_world_up_deg", 999)
        if err > 0.5:
            rospy.logerr("%s: look-at error %.4f deg > 0.5!", name, err)
            continue

        roll, pitch, yaw = rpy["roll"], rpy["pitch"], rpy["yaw"]
        rospy.loginfo("%s: dist=%.2fm look-err=%.4f° up-err=%.1f° "
                      "rpy=(%.3f,%.3f,%.3f)",
                      name, rpy["distance_m"], err, up_err, roll, pitch, yaw)

        tfs.append(make_tf("world", name + "_link", pos[0], pos[1], pos[2],
                           roll, pitch, yaw))
        tfs.append(make_tf(name + "_link", name + "_color_optical_frame",
                           0, 0, 0, -1.5708, 0, -1.5708))
        tfs.append(make_tf(name + "_link", name + "_depth_optical_frame",
                           0, 0, 0, -1.5708, 0, -1.5708))

    rospy.loginfo("Publishing %d static transforms at 1 Hz", len(tfs))
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for t in tfs:
            t.header.stamp = now
        br.sendTransform(tfs)
        rate.sleep()


if __name__ == "__main__":
    main()

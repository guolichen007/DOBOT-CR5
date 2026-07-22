#!/usr/bin/env python3
"""
V5 标定目标内部 TF 发布 — 使用 StaticTransformBroadcaster.

替代原来的 calibration_target_state_publisher (robot_state_publisher),
只发布标定目标内部固定 frame, 不发布 CR5 Link1~Link6.

TF 树:
  object_frame → calibration_target_frame
    ├── calibration_target_front_frame
    ├── calibration_target_left_frame
    ├── calibration_target_right_frame
    ├── calibration_target_top_frame
    └── calibration_target_back_frame

world → object_frame 继续由 publish_object_pose.py 根据 Gazebo pose 发布.

V5: 从 YAML 读取面板 pose, 不再硬编码 BODY_SX 等.
    每个 face frame 的 local +Z 等于该面向外法向.
"""
import math
import os
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped


# YAML 中面板 key → frame 名映射
PANEL_FACE_MAP = {
    "front": "calibration_target_front_frame",
    "left":  "calibration_target_left_frame",
    "right": "calibration_target_right_frame",
    "top":   "calibration_target_top_frame",
    "back":  "calibration_target_back_frame",
}


def load_yaml_config():
    """从 config/calibration/calibration_target.yaml 加载面板 pose."""
    # 尝试多种路径: rospack, 相对路径
    config_path = None
    candidates = []

    # rospack 查找
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("cr5_spray_sim")
        candidates.append(os.path.join(pkg_path, "config", "calibration/calibration_target.yaml"))
    except Exception:
        pass

    # 相对于脚本路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(script_dir, "..", "config", "calibration/calibration_target.yaml"))

    # 工作目录
    candidates.append("src/cr5_spray_sim/config/calibration/calibration_target.yaml")

    for p in candidates:
        if os.path.isfile(p):
            config_path = p
            break

    if config_path is None:
        rospy.logerr("Cannot find calibration/calibration_target.yaml, tried: %s", candidates)
        raise FileNotFoundError("calibration/calibration_target.yaml not found")

    rospy.loginfo("Loading config: %s", config_path)
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def build_face_defs(config):
    """从 YAML 解析面板 pose, 返回 FACE_DEFS 列表."""
    panels = config.get("panels", {})
    face_defs = []

    for panel_key, frame_name in PANEL_FACE_MAP.items():
        panel = panels.get(panel_key, {})
        pose = panel.get("pose_target", {})
        if not pose:
            rospy.logerr("Missing pose_target for panel '%s' in YAML", panel_key)
            raise ValueError(f"Missing pose_target for panel '{panel_key}'")

        xyz = pose.get("xyz", [0, 0, 0])
        rpy = pose.get("rpy", [0, 0, 0])

        face_defs.append((frame_name,
                          float(xyz[0]), float(xyz[1]), float(xyz[2]),
                          float(rpy[0]), float(rpy[1]), float(rpy[2])))

    return face_defs


def euler_to_quaternion(roll, pitch, yaw):
    """将 Euler 角转为 quaternion (xyzw)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def make_static_transform(parent, child, x, y, z, roll, pitch, yaw):
    """构建 TransformStamped 消息."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def main():
    rospy.init_node("publish_calibration_target_tf", anonymous=True,
                    log_level=rospy.INFO)

    # 从 YAML 加载面板 pose
    config = load_yaml_config()
    face_defs = build_face_defs(config)
    rospy.loginfo("Loaded %d face poses from YAML", len(face_defs))

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []

    # object_frame → calibration_target_frame (identity)
    transforms.append(make_static_transform(
        "object_frame", "calibration_target_frame",
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    # calibration_target_frame → 5 个面 frame (从 YAML 读取 pose)
    for child, x, y, z, r, p, yw in face_defs:
        transforms.append(make_static_transform(
            "calibration_target_frame", child, x, y, z, r, p, yw))
        rospy.loginfo("  %s: pos=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)",
                      child, x, y, z, r, p, yw)

    broadcaster.sendTransform(transforms)
    rospy.loginfo("Published %d static transforms for calibration target",
                  len(transforms))
    rospy.loginfo("object_frame → calibration_target_frame → [front, left, right, top, back]")

    # 保持节点存活 (不订阅任何话题, 只 spin 等待退出)
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3.3.4 CR5 Gravity Guard — 临时关闭/恢复 CR5 各 link 的重力。

在 Gazebo paused 状态下加载控制器后、unpause 前，临时关闭 CR5
所有运动 link 的 gravity_mode，防止控制器接管前因重力折叠机械臂。

用法:
  rosrun cr5_spray_sim cr5_gravity_guard_v334.py disable   # 关闭重力
  rosrun cr5_spray_sim cr5_gravity_guard_v334.py restore   # 恢复重力
  rosrun cr5_spray_sim cr5_gravity_guard_v334.py status    # 查看状态

输出到 stderr 供 wrapper 读取:
  CR5_GRAVITY_DISABLED
  CR5_GRAVITY_RESTORED
  CR5_GRAVITY_STATUS
"""
import sys
import os
import json
import rospy
from gazebo_msgs.srv import GetModelProperties, GetLinkProperties, SetLinkProperties
from geometry_msgs.msg import Pose, Point, Quaternion

MODEL_NAME = "cr5_robot"
SAVE_DIR = os.environ.get("CR5_SPRAY_LOG_DIR", "/tmp")
SAVE_FILE = os.path.join(SAVE_DIR, "cr5_link_properties_before.json")

# CR5 link 名称白名单 (前缀匹配). 只保护运动链上的 link，
# 不修改静态物体 (门架、底座、相机等)
CR5_LINK_PREFIXES = (
    "cr5_",
    "Link",
    "joint",
    "base_link",
    "dummy_link",
    "spray_nozzle",
    "tool",
)


def _is_cr5_link(name):
    """判断是否是 CR5 的运动 link (非静态环境物体)."""
    for prefix in CR5_LINK_PREFIXES:
        if name.startswith(prefix):
            return True
    return False


def _get_body_names():
    """获取 CR5 模型的所有 body 名称."""
    rospy.wait_for_service("/gazebo/get_model_properties", timeout=15.0)
    srv = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
    resp = srv(MODEL_NAME)
    if not resp.success:
        rospy.logerr("Cannot get model properties: %s", resp.status_message)
        return []
    return resp.body_names


def _get_link_props(link_name):
    """获取单个 link 的属性."""
    rospy.wait_for_service("/gazebo/get_link_properties", timeout=5.0)
    srv = rospy.ServiceProxy("/gazebo/get_link_properties", GetLinkProperties)
    resp = srv(link_name)
    if not resp.success:
        rospy.logwarn("Cannot get link properties for %s: %s",
                      link_name, resp.status_message)
        return None
    return {
        "link_name": link_name,
        "mass": resp.mass,
        "com": {
            "x": resp.com.position.x,
            "y": resp.com.position.y,
            "z": resp.com.position.z,
        },
        "com_orientation": {
            "x": resp.com.orientation.x,
            "y": resp.com.orientation.y,
            "z": resp.com.orientation.z,
            "w": resp.com.orientation.w,
        },
        "ixx": resp.ixx, "ixy": resp.ixy, "ixz": resp.ixz,
        "iyy": resp.iyy, "iyz": resp.iyz, "izz": resp.izz,
        "gravity_mode": resp.gravity_mode,
    }


def _set_link_props(props, gravity_mode):
    """设置单个 link 属性 (保留质量、COM、惯量，只改 gravity_mode)."""
    rospy.wait_for_service("/gazebo/set_link_properties", timeout=5.0)
    srv = rospy.ServiceProxy("/gazebo/set_link_properties", SetLinkProperties)

    com = Pose(
        position=Point(
            x=props["com"]["x"],
            y=props["com"]["y"],
            z=props["com"]["z"],
        ),
        orientation=Quaternion(
            x=props["com_orientation"]["x"],
            y=props["com_orientation"]["y"],
            z=props["com_orientation"]["z"],
            w=props["com_orientation"]["w"],
        ),
    )

    resp = srv(
        link_name=props["link_name"],
        com=com,
        gravity_mode=gravity_mode,
        mass=props["mass"],
        ixx=props["ixx"], ixy=props["ixy"], ixz=props["ixz"],
        iyy=props["iyy"], iyz=props["iyz"], izz=props["izz"],
    )
    return resp.success, resp.status_message


def cmd_disable():
    """保存当前属性并关闭 CR5 所有 link 的重力."""
    body_names = _get_body_names()
    if not body_names:
        rospy.logerr("No body names found for model '%s'", MODEL_NAME)
        sys.exit(1)

    cr5_links = [n for n in body_names if _is_cr5_link(n)]
    rospy.loginfo("CR5 links found: %s", cr5_links)

    if not cr5_links:
        rospy.logerr("No CR5 links identified in model body_names")
        sys.exit(1)

    # 保存每个 link 的属性
    saved = []
    for link_name in cr5_links:
        props = _get_link_props(link_name)
        if props is None:
            rospy.logerr("Failed to get properties for %s, aborting", link_name)
            sys.exit(1)
        saved.append(props)

    # 写入 JSON 备份
    os.makedirs(SAVE_DIR, exist_ok=True)
    with open(SAVE_FILE, "w") as f:
        json.dump(saved, f, indent=2)
    rospy.loginfo("Saved %d link properties to %s", len(saved), SAVE_FILE)

    # 关闭重力
    failed = []
    for props in saved:
        ok, msg = _set_link_props(props, False)
        if not ok:
            failed.append((props["link_name"], msg))
            rospy.logerr("FAILED to disable gravity for %s: %s",
                         props["link_name"], msg)
        else:
            rospy.loginfo("  gravity OFF: %s (was %s)",
                          props["link_name"], props["gravity_mode"])

    if failed:
        rospy.logerr("Gravity disable failed for %d link(s): %s",
                     len(failed), [f[0] for f in failed])
        sys.exit(1)

    sys.stderr.write("CR5_GRAVITY_DISABLED\n")
    sys.stderr.flush()
    rospy.loginfo("CR5 gravity disabled on %d links", len(saved))


def cmd_restore():
    """从备份文件恢复 CR5 所有 link 的重力."""
    if not os.path.exists(SAVE_FILE):
        rospy.logerr("No backup file found: %s (run disable first)", SAVE_FILE)
        sys.exit(1)

    with open(SAVE_FILE, "r") as f:
        saved = json.load(f)

    rospy.loginfo("Loaded %d link properties from %s", len(saved), SAVE_FILE)

    failed = []
    for props in saved:
        ok, msg = _set_link_props(props, props["gravity_mode"])
        if not ok:
            failed.append((props["link_name"], msg))
            rospy.logerr("FAILED to restore gravity for %s: %s",
                         props["link_name"], msg)
        else:
            rospy.loginfo("  gravity RESTORED: %s → %s",
                          props["link_name"], props["gravity_mode"])

    if failed:
        rospy.logerr("Gravity restore failed for %d link(s): %s",
                     len(failed), [f[0] for f in failed])
        sys.exit(1)

    sys.stderr.write("CR5_GRAVITY_RESTORED\n")
    sys.stderr.flush()
    rospy.loginfo("CR5 gravity restored on %d links", len(saved))


def cmd_status():
    """输出每个 CR5 link 当前的重力状态."""
    body_names = _get_body_names()
    if not body_names:
        rospy.logerr("No body names found for model '%s'", MODEL_NAME)
        sys.exit(1)

    cr5_links = [n for n in body_names if _is_cr5_link(n)]
    if not cr5_links:
        rospy.logerr("No CR5 links identified")
        sys.exit(1)

    has_backup = os.path.exists(SAVE_FILE)

    sys.stderr.write("CR5_GRAVITY_STATUS\n")
    sys.stderr.flush()

    for link_name in sorted(cr5_links):
        props = _get_link_props(link_name)
        if props is None:
            print("  %-35s UNKNOWN" % link_name)
            continue
        print("  %-35s gravity=%s  mass=%.3f" % (
            link_name,
            "ON " if props["gravity_mode"] else "OFF",
            props["mass"]))

    print("  backup: %s" % (SAVE_FILE if has_backup else "(none)"))


def main():
    rospy.init_node("cr5_gravity_guard_v334", anonymous=True, log_level=rospy.WARN)

    if len(sys.argv) < 2:
        print("Usage: cr5_gravity_guard_v334.py <disable|restore|status>", file=sys.stderr)
        sys.exit(1)

    cmd = sys.argv[1].lower()
    if cmd == "disable":
        cmd_disable()
    elif cmd == "restore":
        cmd_restore()
    elif cmd == "status":
        cmd_status()
    else:
        print("Unknown command: %s (expected: disable, restore, status)" % cmd, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

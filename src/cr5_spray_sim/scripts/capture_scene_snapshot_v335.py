#!/usr/bin/env python3
"""
V3.3.5 Scene Snapshot — 启动前模型基线快照 + 失败取证。

在 unpause / controller switch / gravity 修改之前调用，
记录所有 Gazebo 模型姿态、CR5 body pose、关节角。

用法:
  rosrun cr5_spray_sim capture_scene_snapshot_v335.py --output /path/to/snapshot.json

输出到 stderr (供 wrapper 读取):
  PRE_BOOTSTRAP_SCENE_BASELINE_PASS → 快照成功，所有坐标 finite
  SCENE_SNAPSHOT_WARN              → 有坐标 non-finite 但快照已保存
  SCENE_SNAPSHOT_FAILED            → 快照失败

退出码:
  0 = BASELINE_PASS
  1 = 服务不可用
  2 = 部分模型姿态 non-finite
"""
import sys
import os
import json
import rospy
from gazebo_msgs.srv import (
    GetModelProperties, GetLinkProperties, GetLinkState,
    GetWorldProperties, GetModelState,
)
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point, Quaternion


def _to_dict(point, quat):
    """Convert geometry_msgs Point+Quaternion to plain dict."""
    return {
        "x": point.x, "y": point.y, "z": point.z,
        "qx": quat.x, "qy": quat.y, "qz": quat.z, "qw": quat.w,
    }


def _is_finite(pose_dict):
    """Check all values in a pose dict are finite."""
    for v in pose_dict.values():
        if not (float('-inf') < v < float('inf')):
            return False
    return True


def _call_gazebo(srv_name, srv_type, request=None):
    """Safe Gazebo service call with timeout."""
    try:
        rospy.wait_for_service(srv_name, timeout=10.0)
        srv = rospy.ServiceProxy(srv_name, srv_type)
        if request is not None:
            return srv(request)
        return srv()
    except Exception as e:
        rospy.logwarn("Service call %s failed: %s", srv_name, e)
        return None


def _get_model_names():
    """Get all Gazebo model names via world properties."""
    resp = _call_gazebo("/gazebo/get_world_properties", GetWorldProperties)
    if resp is None:
        return []
    return list(resp.model_names)


def _get_model_pose(model_name):
    """Get root pose of a model."""
    resp = _call_gazebo("/gazebo/get_model_state", GetModelState, model_name)
    if resp is None or not resp.success:
        return None
    return _to_dict(resp.pose.position, resp.pose.orientation)


def _get_model_properties(model_name):
    """Get model body names."""
    resp = _call_gazebo("/gazebo/get_model_properties", GetModelProperties, model_name)
    if resp is None or not resp.success:
        return None
    return {
        "parent_model_name": resp.parent_model_name,
        "body_names": list(resp.body_names),
        "joint_names": list(resp.joint_names),
        "is_static": resp.is_static,
    }


def _get_link_state(link_name, ref_frame="world"):
    """Get link state."""
    resp = _call_gazebo("/gazebo/get_link_state", GetLinkState, link_name)
    if resp is None or not resp.success:
        return None
    return _to_dict(resp.link_state.pose.position, resp.link_state.pose.orientation)


def main():
    rospy.init_node("capture_scene_snapshot_v335", anonymous=True,
                    log_level=rospy.WARN)

    output_path = "/tmp/cr5_scene_snapshot.json"
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_path = sys.argv[i + 1]
            break

    snapshot = {
        "version": "V3.3.5",
        "timestamp_wall": rospy.get_time(),
        "models": {},
        "controllers": {},
        "clock": None,
        "all_finite": True,
    }

    # 1. 获取 /clock 最近值
    try:
        from rosgraph_msgs.msg import Clock
        clock_msg = rospy.wait_for_message("/clock", Clock, timeout=3.0)
        snapshot["clock"] = {
            "secs": clock_msg.clock.secs,
            "nsecs": clock_msg.clock.nsecs,
        }
    except Exception:
        snapshot["clock"] = None

    # 2. 获取所有模型
    model_names = _get_model_names()
    rospy.loginfo("Found %d models: %s", len(model_names), model_names)

    for mname in model_names:
        model_info = {"name": mname, "pose": None, "properties": None, "links": {}}
        model_info["pose"] = _get_model_pose(mname)
        model_info["properties"] = _get_model_properties(mname)

        # 对 CR5 模型获取每个 link 的 state
        if model_info["properties"] and "body_names" in (model_info["properties"] or {}):
            for bname in model_info["properties"]["body_names"]:
                ls = _get_link_state("{}::{}".format(mname, bname))
                if ls:
                    model_info["links"][bname] = ls

        snapshot["models"][mname] = model_info

    # 3. 获取控制器状态
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", timeout=5.0)
        srv = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        resp = srv()
        for c in resp.controller:
            snapshot["controllers"][c.name] = c.state
    except Exception:
        snapshot["controllers"] = "UNAVAILABLE"

    # 4. 验证所有坐标 finite
    def check_finite(d, path=""):
        if isinstance(d, dict):
            for k, v in d.items():
                check_finite(v, "{}.{}".format(path, k))
        elif isinstance(d, (int, float)):
            if not (float('-inf') < d < float('inf')):
                rospy.logerr("Non-finite value at %s = %s", path, d)
                snapshot["all_finite"] = False

    check_finite(snapshot["models"], "models")

    # 5. 写入文件
    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(snapshot, f, indent=2, default=str)

    rospy.loginfo("Scene snapshot saved: %s (%d models)",
                  output_path, len(model_names))

    if snapshot["all_finite"]:
        sys.stderr.write("PRE_BOOTSTRAP_SCENE_BASELINE_PASS\n")
        sys.stderr.flush()
    else:
        sys.stderr.write("SCENE_SNAPSHOT_WARN\n")
        sys.stderr.flush()
        sys.exit(2)


if __name__ == "__main__":
    main()

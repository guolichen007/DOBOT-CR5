#!/usr/bin/env python3
"""
V3.3.5 Scene Snapshot — 启动前模型基线快照 + 失败取证。
V3.3.6: 修复 service 调用使用正确的 request 对象，None pose → FAIL.

在 unpause / controller switch / gravity 修改之前调用，
记录所有 Gazebo 模型姿态、CR5 body pose、关节角。

用法:
  rosrun cr5_spray_sim capture_scene_snapshot_v335.py --output /path/to/snapshot.json

输出到 stderr (供 wrapper 读取):
  SCENE_SNAPSHOT_VALID   → 快照成功，所有必需模型坐标 finite
  SCENE_SNAPSHOT_FAILED  → 快照失败或模型缺失

退出码:
  0 = VALID
  1 = 服务不可用
  2 = 模型缺失或姿态 non-finite
"""
import sys
import os
import json
import math
import rospy
from gazebo_msgs.srv import (
    GetModelProperties, GetModelPropertiesRequest,
    GetLinkState, GetLinkStateRequest,
    GetWorldProperties,
    GetModelState, GetModelStateRequest,
)
from controller_manager_msgs.srv import ListControllers

# 必需模型 — 任何一个姿势为 None 都是 FAIL
REQUIRED_MODELS = [
    "ground_plane",
    "cr5_robot",
    "simple_goalpost_frame",
    "simple_hanging_workpiece",
    "pedestal_fl",
    "pedestal_fr",
    "pedestal_rear",
    "cam_front_left",
    "cam_front_right",
    "cam_rear",
]


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
    """Get root pose of a model — 正确的 GetModelStateRequest."""
    req = GetModelStateRequest()
    req.model_name = model_name
    req.relative_entity_name = "world"
    resp = _call_gazebo("/gazebo/get_model_state", GetModelState, req)
    if resp is None or not resp.success:
        return None
    return _to_dict(resp.pose.position, resp.pose.orientation)


def _get_model_properties(model_name):
    """Get model body names — 正确的 GetModelPropertiesRequest."""
    req = GetModelPropertiesRequest()
    req.model_name = model_name
    resp = _call_gazebo("/gazebo/get_model_properties", GetModelProperties, req)
    if resp is None or not resp.success:
        return None
    return {
        "parent_model_name": resp.parent_model_name,
        "body_names": list(resp.body_names),
        "joint_names": list(resp.joint_names),
        "is_static": resp.is_static,
    }


def _get_link_state(link_name, ref_frame="world"):
    """Get link state — 正确的 GetLinkStateRequest."""
    req = GetLinkStateRequest()
    req.link_name = link_name
    req.reference_frame = ref_frame
    resp = _call_gazebo("/gazebo/get_link_state", GetLinkState, req)
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
        "missing_models": [],
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

    # 检查必需模型
    model_set = set(model_names)
    missing_required = [m for m in REQUIRED_MODELS if m not in model_set]
    if missing_required:
        rospy.logerr("Missing required models: %s", missing_required)
        snapshot["missing_models"] = missing_required
        # 继续处理已存在的模型用于诊断

    for mname in model_names:
        model_info = {"name": mname, "pose": None, "properties": None, "links": {}}
        model_info["pose"] = _get_model_pose(mname)
        model_info["properties"] = _get_model_properties(mname)

        # 获取每个 link 的 state
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

    # 4. V3.3.6: 检查必需模型的 pose 是否为 None
    missing_poses = []
    for mname in REQUIRED_MODELS:
        if mname not in snapshot["models"]:
            missing_poses.append(mname)
            snapshot["all_finite"] = False
        else:
            pose = snapshot["models"][mname]["pose"]
            if pose is None:
                rospy.logerr("Required model %s pose is None", mname)
                missing_poses.append(mname)
                snapshot["all_finite"] = False
            elif not _is_finite(pose):
                rospy.logerr("Required model %s has non-finite pose: %s", mname, pose)
                missing_poses.append(mname)
                snapshot["all_finite"] = False

    if missing_poses:
        rospy.logerr("Models with invalid pose: %s", missing_poses)
        snapshot["missing_models"] = list(set(snapshot["missing_models"] + missing_poses))

    # 5. 验证所有坐标 finite (递归检查)
    def check_finite(d, path=""):
        if isinstance(d, dict):
            for k, v in d.items():
                check_finite(v, "{}.{}".format(path, k))
        elif isinstance(d, (int, float)):
            if not (float('-inf') < d < float('inf')):
                rospy.logerr("Non-finite value at %s = %s", path, d)
                snapshot["all_finite"] = False
        elif isinstance(d, list):
            for i, v in enumerate(d):
                check_finite(v, "{}[{}]".format(path, i))

    check_finite(snapshot["models"], "models")

    # 6. 写入文件
    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(snapshot, f, indent=2, default=str)

    rospy.loginfo("Scene snapshot saved: %s (%d models)",
                  output_path, len(model_names))

    if snapshot["all_finite"] and not missing_poses:
        sys.stderr.write("SCENE_SNAPSHOT_VALID\n")
        sys.stderr.flush()
        sys.exit(0)
    else:
        sys.stderr.write("SCENE_SNAPSHOT_FAILED\n")
        sys.stderr.flush()
        sys.exit(2)


if __name__ == "__main__":
    main()

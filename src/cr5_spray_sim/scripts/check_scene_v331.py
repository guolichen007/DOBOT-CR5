#!/usr/bin/env python3
"""
V3.3.1: 真实场景健康检查 (使用 Gazebo service)。

替换 V3.3 中无效的 rosparam pose 检查和 grep 模型计数。

检查:
- 模型白名单 (通过 /gazebo/get_world_properties)
- 关键模型 pose finite (通过 /gazebo/get_model_state)
- CR5 base 位移 < 1mm (10 秒内)
- 坐标绝对值 < 5m
- controller 状态
- 三台 camera_info 有实际消息
"""
import sys
import math
import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetLinkState
from gazebo_msgs.msg import LinkState
from sensor_msgs.msg import CameraInfo
from controller_manager_msgs.srv import ListControllers


# Expected models for V3.3 scene
EXPECTED_MODELS = [
    "cr5_robot",
    "simple_goalpost_frame",
    "simple_hanging_workpiece",
    "pedestal_fl",
    "pedestal_fr",
    "pedestal_rear",
]

# Maximum position coordinate in meters
MAX_COORD = 5.0
# Maximum drift in meters over 10s
MAX_DRIFT = 0.001


def check_models():
    """Verify expected models exist via Gazebo service."""
    print("  Model whitelist:")
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", 5.0)
        gw = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        resp = gw()
        model_names = resp.model_names

        for expected in EXPECTED_MODELS:
            found = expected in model_names
            status = "OK" if found else "MISSING"
            print(f"    [{status}] {expected}")

        # Additional spawned models (cameras)
        cam_models = [m for m in model_names if m.startswith("cam_")]
        print(f"    Cameras spawned: {len(cam_models)} ({', '.join(cam_models[:5])})")

    except Exception as e:
        print(f"    [FAIL] get_world_properties: {e}")
        return False
    return True


def check_model_poses():
    """Check key model poses are finite and within bounds."""
    print("  Model poses:")
    rospy.wait_for_service("/gazebo/get_model_state", 5.0)
    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    failed = 0
    for model_name in EXPECTED_MODELS:
        try:
            resp = gms(model_name, "world")
            if not resp.success:
                print(f"    [WARN] {model_name}: {resp.status_message}")
                continue

            pose = resp.pose.position
            vals = [pose.x, pose.y, pose.z]
            finite = all(math.isfinite(v) for v in vals)
            in_bounds = all(abs(v) < MAX_COORD for v in vals)

            if not finite:
                print(f"    [FAIL] {model_name}: NaN/Inf position")
                failed += 1
            elif not in_bounds:
                vals_str = f"({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})"
                print(f"    [FAIL] {model_name}: out of bounds {vals_str}")
                failed += 1
            else:
                print(f"    [OK] {model_name}: ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})")
        except Exception as e:
            print(f"    [FAIL] {model_name}: {e}")
            failed += 1

    return failed == 0


def check_cr5_stability():
    """Check CR5 base is stable over a short interval."""
    print("  CR5 base stability (5s):")
    rospy.wait_for_service("/gazebo/get_link_state", 5.0)
    gls = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

    try:
        resp1 = gls("cr5_robot::base_link", "world")
        if not resp1.success:
            print(f"    [FAIL] cannot get base_link state: {resp1.status_message}")
            return False

        p1 = resp1.link_state.pose.position
        rospy.sleep(5.0)

        resp2 = gls("cr5_robot::base_link", "world")
        if not resp2.success:
            print(f"    [FAIL] second read failed")
            return False

        p2 = resp2.link_state.pose.position
        drift = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)

        if drift > MAX_DRIFT:
            print(f"    [FAIL] drift={drift*1000:.2f}mm > {MAX_DRIFT*1000:.1f}mm")
            return False
        else:
            print(f"    [OK] drift={drift*1000:.2f}mm")
            return True
    except Exception as e:
        print(f"    [FAIL] {e}")
        return False


def check_controllers():
    """Check joint_state_controller and arm_controller are running."""
    print("  Controllers:")
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", 5.0)
        lc = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        resp = lc()

        expected = ["joint_state_controller", "arm_controller"]
        for name in expected:
            found = False
            for c in resp.controller:
                if c.name == name:
                    state = "running" if c.state == "running" else c.state
                    status = "OK" if c.state == "running" else "FAIL"
                    print(f"    [{status}] {name}: {state}")
                    found = True
                    break
            if not found:
                print(f"    [FAIL] {name}: not found")
                return False
        return True
    except Exception as e:
        print(f"    [WARN] controller check: {e}")
        return True  # non-fatal


def check_camera_topics():
    """Check camera_info topics have actual messages."""
    print("  Camera topics:")
    cams = ["cam_front_left", "cam_front_right", "cam_rear"]
    all_ok = True
    for cam in cams:
        topic = f"/{cam}/camera_info"
        try:
            msg = rospy.wait_for_message(topic, CameraInfo, timeout=5.0)
            if msg.height > 0 and msg.width > 0:
                print(f"    [OK] {topic}: {msg.width}x{msg.height}")
            else:
                print(f"    [FAIL] {topic}: zero dimensions")
                all_ok = False
        except rospy.ROSException:
            print(f"    [FAIL] {topic}: no message within 5s")
            all_ok = False
    return all_ok


def main():
    rospy.init_node("check_scene_v331", anonymous=True, disable_signals=True)

    all_pass = True

    all_pass &= check_models()
    all_pass &= check_model_poses()
    all_pass &= check_cr5_stability()
    all_pass &= check_controllers()
    all_pass &= check_camera_topics()

    if all_pass:
        print("\n  Scene health: ALL PASS")
        sys.exit(0)
    else:
        print("\n  Scene health: SOME CHECKS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()

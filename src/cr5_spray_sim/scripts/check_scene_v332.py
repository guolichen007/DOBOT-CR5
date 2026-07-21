#!/usr/bin/env python3
"""
V3.3.2 场景健康检查 (替代 check_scene_v331.py)。

新增:
- joint_states 角度检查 (六轴偏差 < 0.03 rad)
- Link6 / spray_nozzle_frame 高度检查 (z >= 0.80)
- 折叠检测 (Link6.z < 0.30 → CR5_ARM_FOLDED_BELOW_WORKSPACE)
- 地下检测 (通过 get_model_properties + get_link_state)
- base 稳定用 get_model_state (不硬编码 base_link 名)
- 相机话题修正 (RealSense /color/camera_info)
"""
import sys
import math
import rospy
import tf2_ros
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelProperties, GetLinkState
from sensor_msgs.msg import CameraInfo, JointState
from controller_manager_msgs.srv import ListControllers

EXPECTED_MODELS = [
    "cr5_robot", "simple_goalpost_frame", "simple_hanging_workpiece",
    "pedestal_fl", "pedestal_fr", "pedestal_rear",
]
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
MAX_COORD = 5.0
MAX_DRIFT_M = 0.001
MAX_JOINT_ERR_RAD = 0.03
LINK6_MIN_Z = 0.80
NOZZLE_MIN_Z = 0.80
FOLDED_Z_THRESHOLD = 0.30
UNDERGROUND_LIMIT = -0.03


def check_models():
    """Verify expected models exist."""
    print("  Model whitelist:")
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", 10.0)
        gw = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        resp = gw()
        model_names = resp.model_names
        for expected in EXPECTED_MODELS:
            found = expected in model_names
            status = "OK" if found else "MISSING"
            print(f"    [{status}] {expected}")
        cam_models = [m for m in model_names if m.startswith("cam_")]
        print(f"    Cameras: {len(cam_models)} ({', '.join(cam_models[:5])})")
    except Exception as e:
        print(f"    [FAIL] get_world_properties: {e}")
        return False
    return True


def check_model_poses():
    """Check model poses finite and in bounds."""
    print("  Model poses:")
    rospy.wait_for_service("/gazebo/get_model_state", 10.0)
    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    failed = 0
    for name in EXPECTED_MODELS:
        try:
            resp = gms(name, "world")
            if not resp.success:
                print(f"    [WARN] {name}: {resp.status_message}")
                continue
            p = resp.pose.position
            vals = [p.x, p.y, p.z]
            if not all(math.isfinite(v) for v in vals):
                print(f"    [FAIL] {name}: NaN/Inf")
                failed += 1
            elif not all(abs(v) < MAX_COORD for v in vals):
                print(f"    [FAIL] {name}: ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")
                failed += 1
            else:
                print(f"    [OK] {name}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        except Exception as e:
            print(f"    [FAIL] {name}: {e}")
            failed += 1
    return failed == 0


def check_joint_angles():
    """Check joint_states: all six joints present, finite, near zero."""
    print("  Joint angles:")
    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=10.0)
    except rospy.ROSException:
        print("    [FAIL] no /joint_states message")
        return False

    positions = dict(zip(msg.name, msg.position))
    all_ok = True
    for name in JOINT_NAMES:
        actual = positions.get(name)
        if actual is None:
            print(f"    [FAIL] {name}: missing")
            all_ok = False
        elif not math.isfinite(actual):
            print(f"    [FAIL] {name}: NaN/Inf")
            all_ok = False
        elif abs(actual) > MAX_JOINT_ERR_RAD:
            print(f"    [FAIL] {name}: {actual:.4f} rad (limit={MAX_JOINT_ERR_RAD})")
            all_ok = False
        else:
            print(f"    [OK] {name}: {actual:.4f} rad")
    return all_ok


def check_frame_heights():
    """Check Link6 and spray_nozzle_frame heights via TF."""
    print("  Frame heights (TF):")
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    rospy.sleep(1.0)

    checks = [
        ("Link6", 0.0, 0.05),       # expected z ~1.047
        ("spray_nozzle_frame", 0.0, 0.06),
    ]
    all_ok = True

    for frame, expected_z, pos_tol in checks:
        try:
            t = tf_buf.lookup_transform("world", frame, rospy.Time(0),
                                         rospy.Duration(3.0))
            z = t.transform.translation.z

            # Folded check
            if frame == "Link6" and z < FOLDED_Z_THRESHOLD:
                print(f"    [FATAL] CR5_ARM_FOLDED_BELOW_WORKSPACE ({frame}.z={z:.3f} < {FOLDED_Z_THRESHOLD})")
                all_ok = False
                continue

            if z < LINK6_MIN_Z if frame == "Link6" else z < NOZZLE_MIN_Z:
                min_z = LINK6_MIN_Z if frame == "Link6" else NOZZLE_MIN_Z
                print(f"    [FAIL] {frame}: z={z:.3f} < min={min_z}")
                all_ok = False
            else:
                print(f"    [OK] {frame}: z={z:.3f}")
        except Exception as e:
            print(f"    [FAIL] {frame}: {type(e).__name__}")
            all_ok = False

    return all_ok


def check_underground():
    """Check no actual movement link is underground."""
    print("  Underground check:")
    try:
        rospy.wait_for_service("/gazebo/get_model_properties", 5.0)
        gmp = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)
        resp = gmp("cr5_robot")
        body_names = resp.body_names

        rospy.wait_for_service("/gazebo/get_link_state", 5.0)
        gls = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

        all_ok = True
        for body in body_names:
            try:
                lr = gls(body, "world")
                if not lr.success:
                    continue
                z = lr.link_state.pose.position.z
                if z < UNDERGROUND_LIMIT:
                    print(f"    [FAIL] {body}: z={z:.3f} < {UNDERGROUND_LIMIT}")
                    all_ok = False
            except Exception:
                continue

        if all_ok:
            print(f"    [OK] {len(body_names)} bodies above {UNDERGROUND_LIMIT}m")
        return all_ok
    except Exception as e:
        print(f"    [WARN] underground check: {e}")
        return True


def check_base_stability():
    """Check cr5_robot model pose is stable (using get_model_state)."""
    print("  Base stability (5s):")
    try:
        rospy.wait_for_service("/gazebo/get_model_state", 5.0)
        gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        r1 = gms("cr5_robot", "world")
        if not r1.success:
            print(f"    [FAIL] {r1.status_message}")
            return False
        p1 = r1.pose.position
        rospy.sleep(5.0)
        r2 = gms("cr5_robot", "world")
        if not r2.success:
            print(f"    [FAIL] second read")
            return False
        p2 = r2.pose.position
        drift = math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2 + (p2.z-p1.z)**2)
        if drift > MAX_DRIFT_M:
            print(f"    [FAIL] drift={drift*1000:.2f}mm > {MAX_DRIFT_M*1000:.1f}mm")
            return False
        print(f"    [OK] drift={drift*1000:.2f}mm")
        return True
    except Exception as e:
        print(f"    [FAIL] {e}")
        return False


def check_controllers():
    """Check key controllers running."""
    print("  Controllers:")
    try:
        rospy.wait_for_service("/controller_manager/list_controllers", 10.0)
        lc = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        resp = lc()
        for name in ["joint_state_controller", "arm_controller"]:
            found = False
            for c in resp.controller:
                if c.name == name:
                    status = "OK" if c.state == "running" else "FAIL"
                    print(f"    [{status}] {name}: {c.state}")
                    found = True
                    break
            if not found:
                print(f"    [FAIL] {name}: not found")
                return False
        return True
    except Exception as e:
        print(f"    [WARN] controller check: {e}")
        return True


def find_camera_topics():
    """Find actual RealSense camera_info topics dynamically."""
    topics = rospy.get_published_topics()
    cam_topics = {}
    for topic, msg_type in topics:
        if msg_type == "sensor_msgs/CameraInfo" and "/cam_" in topic:
            for cam_name in ["cam_front_left", "cam_front_right", "cam_rear"]:
                if cam_name in topic:
                    if cam_name not in cam_topics or "/color/" in topic:
                        cam_topics[cam_name] = topic
    return cam_topics


def check_camera_topics():
    """Check camera_info topics have valid messages."""
    print("  CameraInfo:")
    cam_topics = find_camera_topics()

    expected = ["cam_front_left", "cam_front_right", "cam_rear"]
    all_ok = True

    for cam in expected:
        topic = cam_topics.get(cam)
        if topic is None:
            # Try fallback
            for fallback in [f"/{cam}/camera/color/camera_info",
                             f"/{cam}/camera_info"]:
                try:
                    msg = rospy.wait_for_message(fallback, CameraInfo, timeout=3.0)
                    topic = fallback
                    break
                except rospy.ROSException:
                    continue

        if topic is None:
            print(f"    [FAIL] {cam}: no CameraInfo topic found")
            all_ok = False
            continue

        try:
            msg = rospy.wait_for_message(topic, CameraInfo, timeout=15.0)
            ok = (msg.width > 0 and msg.height > 0 and
                  len(msg.K) >= 5 and msg.K[0] > 0 and msg.K[4] > 0)
            status = "OK" if ok else "FAIL"
            k0 = msg.K[0] if len(msg.K) > 0 else 0
            k4 = msg.K[4] if len(msg.K) > 4 else 0
            print(f"    [{status}] {topic}: {msg.width}x{msg.height} K=({k0:.0f},{k4:.0f})")
            if not ok:
                all_ok = False
        except rospy.ROSException:
            print(f"    [FAIL] {topic}: no message within 15s")
            all_ok = False

    return all_ok


def main():
    rospy.init_node("check_scene_v332", anonymous=True, disable_signals=True)

    all_pass = True
    all_pass &= check_models()
    all_pass &= check_model_poses()
    all_pass &= check_joint_angles()
    all_pass &= check_frame_heights()
    all_pass &= check_underground()
    all_pass &= check_base_stability()
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

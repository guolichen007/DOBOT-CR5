#!/usr/bin/env python3
"""
V3.3.6 Scene Models Waiter.

等待所有必需模型出现在 Gazebo 场景中。
通过 /gazebo/get_world_properties 轮询，连续 5 次查询集合不变后返回。

用法:
  rosrun cr5_spray_sim wait_scene_models_v336.py [--timeout 45.0]

输出到 stderr:
  SCENE_MODELS_READY
  SCENE_MODELS_FAILED

退出码:
  0 = SCENE_MODELS_READY
  1 = 服务不可用或超时
  2 = 模型缺失
"""
import sys
import time
import rospy
from gazebo_msgs.srv import GetWorldProperties

REQUIRED_MODELS = {
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
}

STABLE_COUNT = 5
POLL_INTERVAL = 0.2


def _get_models():
    """获取当前所有模型名称集合。"""
    srv = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    resp = srv()
    return set(resp.model_names)


def main():
    rospy.init_node("wait_scene_models_v336", anonymous=True,
                    log_level=rospy.WARN)

    timeout = 45.0
    for i, arg in enumerate(sys.argv):
        if arg == "--timeout" and i + 1 < len(sys.argv):
            timeout = float(sys.argv[i + 1])

    # 等待服务
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("get_world_properties service not available")
        sys.stderr.write("SCENE_MODELS_FAILED\n")
        sys.stderr.flush()
        sys.exit(1)

    stable = 0
    last_set = None
    start = time.time()

    while time.time() - start < timeout:
        try:
            current = _get_models()
        except Exception as e:
            rospy.logwarn("get_world_properties failed: %s", e)
            time.sleep(POLL_INTERVAL)
            continue

        missing = REQUIRED_MODELS - current
        present = REQUIRED_MODELS & current

        if missing:
            stable = 0
            last_set = None
            rospy.loginfo("waiting for models: missing=%s, present=%s",
                          sorted(missing), sorted(present))
        elif current == last_set:
            stable += 1
            rospy.loginfo("models stable %d/%d", stable, STABLE_COUNT)
        else:
            stable = 1
            last_set = current
            rospy.loginfo("models changed, reset stable: %d models",
                          len(current))

        if stable >= STABLE_COUNT:
            elapsed = time.time() - start
            rospy.loginfo("all %d models ready after %.1fs: %s",
                          len(current), elapsed, sorted(current))
            sys.stderr.write("SCENE_MODELS_READY\n")
            sys.stderr.flush()
            sys.exit(0)

        time.sleep(POLL_INTERVAL)

    # 超时
    try:
        final = _get_models()
    except Exception:
        final = set()
    missing = sorted(REQUIRED_MODELS - final)
    present = sorted(REQUIRED_MODELS & final)
    rospy.logerr("timeout after %.1fs: missing=%s, present=%s",
                 timeout, missing, present)
    sys.stderr.write("SCENE_MODELS_FAILED\n")
    sys.stderr.flush()
    sys.exit(2)


if __name__ == "__main__":
    main()

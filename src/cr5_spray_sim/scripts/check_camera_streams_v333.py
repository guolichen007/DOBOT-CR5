#!/usr/bin/env python3
"""
V3.3.3 Camera Stream Check (wall-time based, requires real frames).

与 V3.3.2 的区别:
- 使用 wall clock 做所有超时
- 不满足于 topic 存在 / publisher 已注册
- 必须收到实际图像帧数据 (至少 3 帧)
- 验证 CameraInfo 参数有效 (width, height, K)
- 验证 color 编码和 data 非空
- 验证 depth 编码和 data 非空

退出码:
  0 = CAMERA_STREAMS_READY
  1 = TOPIC_ADVERTISED_BUT_NO_FRAMES
  2 = CAMERA_INFO_INVALID
"""
import sys
import time
import rospy
from sensor_msgs.msg import CameraInfo, Image

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
MIN_FRAMES = 3
WALL_TIMEOUT_S = 12.0
VALID_COLOR_ENCODINGS = {"rgb8", "bgr8", "bgra8", "rgba8"}
VALID_DEPTH_ENCODINGS = {"32FC1", "16UC1", "mono16"}


def find_topic(cam, suffix):
    """Find the actual topic name for a camera stream."""
    topics = rospy.get_published_topics()
    for topic, msg_type in topics:
        if cam in topic and topic.endswith(suffix):
            return topic
    return None


def check_camera_info(cam):
    """Check CameraInfo has valid parameters."""
    topic = find_topic(cam, "/camera_info")
    if topic is None:
        print(f"    [FAIL] {cam}: no CameraInfo topic found")
        return False, None

    msgs = []
    sub = rospy.Subscriber(topic, CameraInfo, lambda m: msgs.append(m), queue_size=5)
    wall_start = time.monotonic()
    while time.monotonic() - wall_start < 6.0:
        if len(msgs) >= 1:
            break
        time.sleep(0.1)
    sub.unregister()

    if len(msgs) == 0:
        print(f"    [FAIL] {cam} CameraInfo ({topic}): topic advertised but no frames")
        return False, topic

    msg = msgs[-1]
    ok = (msg.width > 0 and msg.height > 0 and
          len(msg.K) >= 5 and msg.K[0] > 0 and msg.K[4] > 0)
    if ok:
        print(f"    [OK] {cam} CameraInfo: {msg.width}x{msg.height} "
              f"K=({msg.K[0]:.0f},{msg.K[4]:.0f})")

        # Check distortion model
        dist = msg.distortion_model if msg.distortion_model else "none"
        print(f"         distortion={dist} D={[f'{d:.4f}' for d in msg.D[:5]] if len(msg.D)>=5 else msg.D}")
    else:
        print(f"    [FAIL] {cam} CameraInfo ({topic}): invalid params "
              f"(w={msg.width} h={msg.height} K_len={len(msg.K)})")
    return ok, topic


def check_image_stream(cam, stream_type):
    """Check image stream has real frames with valid data."""
    suffix = "/image_raw"
    topic = find_topic(cam, suffix)
    if topic is None:
        print(f"    [FAIL] {cam} {stream_type}: no image_raw topic")
        return False

    msgs = []
    sub = rospy.Subscriber(topic, Image, lambda m: msgs.append(m), queue_size=10)
    wall_start = time.monotonic()
    while time.monotonic() - wall_start < WALL_TIMEOUT_S:
        if len(msgs) >= MIN_FRAMES:
            break
        time.sleep(0.05)
    sub.unregister()

    hz = len(msgs) / max(time.monotonic() - wall_start, 0.1)

    if len(msgs) < MIN_FRAMES:
        print(f"    [FAIL] {cam} {stream_type} ({topic}): "
              f"only {len(msgs)} frames in {WALL_TIMEOUT_S}s (need >={MIN_FRAMES})")
        return False

    # Validate the last frame
    valid_encodings = VALID_COLOR_ENCODINGS if stream_type == "COLOR" else VALID_DEPTH_ENCODINGS
    msg = msgs[-1]
    enc_ok = msg.encoding in valid_encodings
    step_ok = msg.step > 0
    data_ok = len(msg.data) > 0

    status = "OK" if (enc_ok and step_ok and data_ok) else "FAIL"
    details = []
    if not enc_ok:
        details.append(f"encoding={msg.encoding}")
    if not step_ok:
        details.append(f"step={msg.step}")
    if not data_ok:
        details.append("data empty")

    print(f"    [{status}] {cam} {stream_type}: {len(msgs)} frames @ {hz:.1f}Hz "
          f"({msg.width}x{msg.height} {msg.encoding} step={msg.step} data={len(msg.data)}B)")
    if details:
        print(f"         issues: {', '.join(details)}")

    return enc_ok and step_ok and data_ok


def main():
    rospy.init_node("check_camera_streams_v333", anonymous=True, disable_signals=True)

    print("=== Camera Stream Check (wall-time) ===")
    wall_start = time.monotonic()

    all_pass = True
    for cam in CAMERAS:
        print(f"  {cam}:")

        # CameraInfo
        ci_ok, _ = check_camera_info(cam)
        if not ci_ok:
            all_pass = False
            continue

        # Color image
        color_ok = check_image_stream(cam, "COLOR")
        if not color_ok:
            all_pass = False

        # Depth image
        depth_ok = check_image_stream(cam, "DEPTH")
        if not depth_ok:
            all_pass = False

        if color_ok and depth_ok:
            print(f"    => {cam} COLOR PASS DEPTH PASS")

    elapsed = time.monotonic() - wall_start

    if all_pass:
        print(f"\nCAMERA_STREAMS_READY ({elapsed:.1f}s)")
        sys.exit(0)
    else:
        print(f"\nTOPIC_ADVERTISED_BUT_NO_FRAMES or CAMERA_INFO_INVALID ({elapsed:.1f}s)")
        sys.exit(1)


if __name__ == "__main__":
    main()

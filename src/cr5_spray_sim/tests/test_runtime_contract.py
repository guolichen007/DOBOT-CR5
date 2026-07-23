#!/usr/bin/env python3
"""test_runtime_contract.py — runtime_contract 模块单元测试 (离线)"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from runtime_contract import (
    EXPECTED_CAMERAS,
    EXPECTED_COLOR_TOPICS,
    EXPECTED_DEPTH_TOPICS,
    EXPECTED_CAMERA_INFO_TOPICS,
    EXPECTED_CALIBRATION_FRAMES,
    EXPECTED_CR5_FRAMES,
    validate_static,
)


def test_three_cameras():
    assert len(EXPECTED_CAMERAS) == 3, \
        f"Expected 3 cameras, got {len(EXPECTED_CAMERAS)}"
    assert "cam_front_left" in EXPECTED_CAMERAS
    assert "cam_front_right" in EXPECTED_CAMERAS
    assert "cam_rear" in EXPECTED_CAMERAS


def test_color_topics_match_cameras():
    for cam in EXPECTED_CAMERAS:
        color_topic = f"/{cam}/color/image_raw"
        assert color_topic in EXPECTED_COLOR_TOPICS, \
            f"Missing color topic: {color_topic}"


def test_depth_topics_match_cameras():
    for cam in EXPECTED_CAMERAS:
        depth_topic = f"/{cam}/depth/image_rect_raw"
        assert depth_topic in EXPECTED_DEPTH_TOPICS, \
            f"Missing depth topic: {depth_topic}"


def test_calibration_frames_contain_object():
    assert "object_frame" in EXPECTED_CALIBRATION_FRAMES
    assert "calibration_target_frame" in EXPECTED_CALIBRATION_FRAMES


def test_cr5_frames_count():
    assert len(EXPECTED_CR5_FRAMES) >= 7, \
        f"CR5 should have base_link + 6 links, got {len(EXPECTED_CR5_FRAMES)}"
    assert "base_link" in EXPECTED_CR5_FRAMES
    assert "Link6" in EXPECTED_CR5_FRAMES


def test_validate_static_offline():
    passed, report = validate_static(offline=True)
    assert passed, f"Static validation should pass, report: {report}"
    assert report["camera_count"] == 3
    assert report["color_topics"] == 3
    assert report["depth_topics"] == 3


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS: {t.__name__}")
        except Exception as e:
            print(f"FAIL: {t.__name__} — {e}")
            failed += 1
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)

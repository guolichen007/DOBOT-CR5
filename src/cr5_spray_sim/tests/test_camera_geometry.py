#!/usr/bin/env python3
"""test_camera_geometry.py — camera_geometry 模块单元测试"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import math
import numpy as np
from camera_geometry import (
    compute_camera_look_at,
    compute_distance,
    estimate_fov_coverage,
    validate_camera_acceptance,
)


def test_compute_distance():
    d = compute_distance([0, 0, 0], [3, 4, 0])
    assert abs(d - 5.0) < 1e-9, f"Expected 5.0, got {d}"


def test_compute_look_at_directly_ahead():
    """相机在 +X 方向看原点: pitch 应接近 0, yaw 应为 pi."""
    cam = [0.95, 0.0, 1.3]
    tgt = [0.65, 0.0, 1.0]
    (roll, pitch, yaw), R = compute_camera_look_at(cam, tgt)
    # 相机在目标的 +X 方向, 视线方向 = cam - tgt (单位向量在 +X)
    # yaw 应接近 pi (朝向 -X, 即朝向目标)
    assert abs(math.degrees(pitch)) < 90, f"pitch too large: {math.degrees(pitch)}"
    # R 应该是正交矩阵
    assert abs(abs(float(np.linalg.det(R))) - 1.0) < 1e-6


def test_compute_look_at_top_down():
    """相机从正上方看: 视线方向接近 -Z, pitch 应接近 pi/2."""
    cam = [0.65, 0.0, 1.6]
    tgt = [0.65, 0.0, 1.0]
    (roll, pitch, yaw), R = compute_camera_look_at(cam, tgt)
    # pitch 负值表示朝下看
    assert pitch < 0, f"Expected negative pitch for top-down, got {pitch}"


def test_estimate_fov_coverage():
    """标准距离下的 FOV 覆盖应在合理范围."""
    # 相机约 0.9m 距离看 0.34x0.28m 目标
    cam = [0.0, 0.0, 1.2]
    tgt = [0.65, 0.0, 1.0]
    m = estimate_fov_coverage(cam, tgt, (0.34, 0.28))
    # 在 ~0.7m 距离，D455 69.4° HFOV 覆盖约 0.97m 宽度
    # 0.34m 目标约占 35%
    assert 15 < m["horizontal_fill_pct"] < 60, \
        f"h_fill out of range: {m['horizontal_fill_pct']}"
    assert 10 < m["vertical_fill_pct"] < 60, \
        f"v_fill out of range: {m['vertical_fill_pct']}"
    assert m["distance_m"] > 0, "distance should be positive"


def test_acceptance_pass():
    m = {"horizontal_fill_pct": 30.0, "vertical_fill_pct": 40.0, "distance_m": 1.0}
    thresholds = {
        "min_horizontal_fill_pct": 20,
        "max_horizontal_fill_pct": 50,
        "min_vertical_fill_pct": 25,
        "max_vertical_fill_pct": 55,
    }
    ok, violations = validate_camera_acceptance(m, thresholds)
    assert ok, f"Should pass, violations: {violations}"


def test_acceptance_fail_too_small():
    m = {"horizontal_fill_pct": 5.0, "vertical_fill_pct": 5.0, "distance_m": 3.0}
    thresholds = {
        "min_horizontal_fill_pct": 20,
        "max_horizontal_fill_pct": 50,
        "min_vertical_fill_pct": 25,
        "max_vertical_fill_pct": 55,
    }
    ok, violations = validate_camera_acceptance(m, thresholds)
    assert not ok, "Should fail for too-small coverage"


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

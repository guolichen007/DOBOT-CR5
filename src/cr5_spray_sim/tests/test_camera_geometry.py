#!/usr/bin/env python3
"""test_camera_geometry.py — camera_geometry 模块单元测试"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import math
import numpy as np
from cr5_spray_sim.camera_geometry import (
    compute_camera_look_at,
    compute_distance,
    estimate_fov_coverage,
    validate_camera_acceptance,
)


def test_compute_distance():
    d = compute_distance([0, 0, 0], [3, 4, 0])
    assert abs(d - 5.0) < 1e-9, f"Expected 5.0, got {d}"


def test_compute_look_at_directly_ahead():
    """相机在 +X 方向看原点: yaw 应为 pi (朝向 -X = 朝向目标)."""
    cam = [0.95, 0.0, 1.3]
    tgt = [0.65, 0.0, 1.0]
    result = compute_camera_look_at(cam, tgt)
    # 相机在目标的 +X 方向, 观察方向应为 -X (yaw ≈ pi)
    assert abs(result["yaw"] - math.pi) < 0.1, \
        f"yaw should be ~pi, got {result['yaw']:.4f}"
    assert abs(math.degrees(result["pitch"])) < 90, \
        f"pitch too large: {math.degrees(result['pitch'])}"
    # R 应该是正交矩阵
    assert abs(abs(float(np.linalg.det(result["R"]))) - 1.0) < 1e-6


def test_compute_look_at_top_down():
    """相机从正上方看: pitch 应为正值 (Gazebo convention, +X 指向 -Z)."""
    cam = [0.65, 0.0, 1.6]
    tgt = [0.65, 0.0, 1.0]
    result = compute_camera_look_at(cam, tgt)
    assert result["pitch"] > 0, \
        f"Expected positive pitch for top-down (looking down), got {result['pitch']:.4f}"
    assert abs(result["pitch"] - math.pi/2) < 0.01, \
        f"Expected pitch ~π/2 for directly above, got {result['pitch']:.4f}"


def test_estimate_fov_coverage():
    """标准距离下的 FOV 覆盖应在合理范围."""
    cam = [0.0, 0.0, 1.2]
    tgt = [0.65, 0.0, 1.0]
    m = estimate_fov_coverage(cam, tgt, (0.34, 0.28))
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

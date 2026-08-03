#!/usr/bin/env python3
"""测试同步 manifest 验证."""
import os, sys, unittest

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    validate_sync_group, RGBDData,
)


def _make_rgbd(cam_name, valid=True, frame_id="", quality=None):
    data = RGBDData(camera_name=cam_name)
    data.color = None if not valid else None
    if valid:
        # 模拟基本有效数据
        import numpy as np
        data.color = np.zeros((480, 640, 3), dtype=np.uint8)
        data.depth_raw = np.ones((480, 640), dtype=np.uint16) * 1000
        data.color_width, data.color_height = 640, 480
        data.depth_width, data.depth_height = 640, 480
        data.depth_unit = "mm"
        data.color_K = np.eye(3)
        data.depth_K = np.eye(3)
        data.color_frame_id = frame_id or f"{cam_name}_color_optical_frame"
        data.depth_frame_id = frame_id or f"{cam_name}_depth_optical_frame"
    if quality:
        data.quality = quality
    return data


class TestSyncManifest(unittest.TestCase):

    def setUp(self):
        self.cameras = ["cam_front_left", "cam_front_right", "cam_rear"]

    def test_manifest_success_passes(self):
        """manifest success=true 且 skew 合格 → PASS."""
        rgbd_list = [_make_rgbd(c) for c in self.cameras]
        manifest = {
            "success": True,
            "captured": 3,
            "expected": 3,
            "camera_names": self.cameras,
            "cross_camera_sync": {
                "method": "exact_stamp_ns",
                "max_inter_camera_skew_s": 0.0,  # 所有 stamp 相同, 实际 skew=0
                "max_allowed_skew_s": 0.005,
                "per_camera_color_stamps": {
                    c: {"secs": 1000, "nsecs": 0, "stamp_ns": 1000000000000}
                    for c in self.cameras
                },
            },
        }
        passed, errors, warnings, report = validate_sync_group(
            rgbd_list, self.cameras, manifest=manifest)
        self.assertTrue(passed, msg="; ".join(errors))
        self.assertEqual(report["source"], "group_manifest.json")
        self.assertEqual(report["method"], "exact_stamp_ns")
        self.assertEqual(report["actual_skew_ms"], 0.0)

    def test_manifest_skew_exceeded_fails(self):
        """manifest skew 超限 → FAIL."""
        rgbd_list = [_make_rgbd(c) for c in self.cameras]
        manifest = {
            "success": True,
            "captured": 3,
            "expected": 3,
            "camera_names": self.cameras,
            "cross_camera_sync": {
                "method": "exact_stamp_ns",
                "max_inter_camera_skew_s": 0.010,  # 10ms > 5ms
                "max_allowed_skew_s": 0.005,
                "per_camera_color_stamps": {
                    c: {"secs": 1000, "nsecs": 0, "stamp_ns": 1000000000000}
                    for c in self.cameras
                },
            },
        }
        passed, errors, _, _ = validate_sync_group(
            rgbd_list, self.cameras, max_inter_camera_skew_ms=5.0,
            manifest=manifest)
        self.assertFalse(passed)
        self.assertTrue(any("skew" in e.lower() or "偏差" in e for e in errors),
                        f"错误应提到 skew: {errors}")

    def test_manifest_success_false_fails(self):
        """manifest success=false → FAIL."""
        rgbd_list = [_make_rgbd(c) for c in self.cameras]
        manifest = {
            "success": False,
            "captured": 2,
            "expected": 3,
            "camera_names": self.cameras,
            "cross_camera_sync": {
                "method": "exact_stamp_ns",
                "max_inter_camera_skew_s": 0.002,
                "max_allowed_skew_s": 0.005,
                "per_camera_color_stamps": {},
            },
        }
        passed, errors, _, _ = validate_sync_group(
            rgbd_list, self.cameras, manifest=manifest)
        self.assertFalse(passed)

    def test_manifest_missing_stamp_fails(self):
        """三台相机 stamp 缺失 → FAIL."""
        rgbd_list = [_make_rgbd(c) for c in self.cameras]
        manifest = {
            "success": True,
            "captured": 3,
            "expected": 3,
            "camera_names": self.cameras,
            "cross_camera_sync": {
                "method": "exact_stamp_ns",
                "max_inter_camera_skew_s": 0.002,
                "max_allowed_skew_s": 0.005,
                "per_camera_color_stamps": {
                    "cam_front_left": {"secs": 1000, "nsecs": 0, "stamp_ns": 1000000000000},
                    # cam_front_right 缺失!
                    "cam_rear": {"secs": 1000, "nsecs": 0, "stamp_ns": 1000000000000},
                },
            },
        }
        passed, errors, _, _ = validate_sync_group(
            rgbd_list, self.cameras, manifest=manifest)
        self.assertFalse(passed)

    def test_no_manifest_strict_fails(self):
        """无 manifest + 非 legacy → FAIL."""
        rgbd_list = [_make_rgbd(c) for c in self.cameras]
        passed, errors, _, _ = validate_sync_group(
            rgbd_list, self.cameras, allow_legacy_fallback=False)
        self.assertFalse(passed)
        self.assertTrue(any("manifest" in e.lower() for e in errors))

    def test_no_manifest_legacy_fallback(self):
        """无 manifest + legacy fallback → 使用 quality.yaml."""
        rgbd_list = [
            _make_rgbd(c, quality={"color_timestamp": {"secs": 1000, "nsecs": i * 1000000}})
            for i, c in enumerate(self.cameras)
        ]
        passed, errors, warnings, report = validate_sync_group(
            rgbd_list, self.cameras, allow_legacy_fallback=True)
        self.assertEqual(report["source"], "quality.yaml (legacy fallback)")
        self.assertIsNotNone(report["actual_skew_ms"])


if __name__ == "__main__":
    unittest.main()

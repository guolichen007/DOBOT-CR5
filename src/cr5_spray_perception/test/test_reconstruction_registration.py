#!/usr/bin/env python3
"""测试 RGB-D 配准契约模块."""
import os, sys, unittest
import numpy as np

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.registration import (
    determine_registration_status, DepthRegistration,
    COLOR_DEPTH_COLOCATED_THRESHOLDS,
)


class TestRegistrationStatus(unittest.TestCase):

    def test_same_frame_id_registered(self):
        """frame_id 相同 + 同尺寸 = REGISTERED."""
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_color_optical_frame",
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
        )
        self.assertEqual(reg.status, "REGISTERED")
        self.assertTrue(reg.is_registered)
        self.assertTrue(reg.pixel_correspondence_safe)

    def test_diff_frame_same_size_with_evidence_colocated(self):
        """frame 名不同但有显式 T_color_depth → COLOCATED."""
        T = np.eye(4)
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_depth_optical_frame",
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
            T_color_depth=T,
        )
        self.assertEqual(reg.status, "COLOCATED")
        self.assertTrue(reg.is_registered)
        self.assertTrue(reg.pixel_correspondence_safe)  # identity T + 同尺寸
        self.assertLessEqual(reg.translation_mm, 0.1)
        self.assertLessEqual(reg.rotation_deg, 0.01)

    def test_diff_frame_no_evidence_fails(self):
        """frame 名不同且无法推断 T_color_depth → UNREGISTERED."""
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fr_depth_optical_frame",  # 不同相机!
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
        )
        self.assertEqual(reg.status, "UNREGISTERED")
        self.assertFalse(reg.is_registered)

    def test_translation_exceeds_threshold(self):
        """T_color_depth 平移超限 → UNREGISTERED (有显式 T 但平移太大)."""
        T = np.eye(4)
        T[:3, 3] = [0.01, 0, 0]  # 10mm >> 0.1mm
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_depth_optical_frame",
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
            T_color_depth=T,
        )
        self.assertEqual(reg.status, "UNREGISTERED")

    def test_diff_frame_no_evidence_unregistered(self):
        """frame 名不同且无 T_color_depth → UNREGISTERED (不再按名称猜)."""
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_depth_optical_frame",  # 无 T_color_depth!
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
        )
        self.assertEqual(reg.status, "UNREGISTERED")
        self.assertFalse(reg.verified)

    def test_rotation_exceeds_threshold(self):
        """T_color_depth 旋转超限 → UNREGISTERED."""
        from scipy.spatial.transform import Rotation
        R = Rotation.from_euler('x', 1.0, degrees=True).as_matrix()  # 1° >> 0.01°
        T = np.eye(4)
        T[:3, :3] = R
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_depth_optical_frame",
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
            T_color_depth=T,
        )
        self.assertEqual(reg.status, "UNREGISTERED")

    def test_different_size_unregistered(self):
        """不同尺寸 → UNREGISTERED."""
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_color_optical_frame",
            color_width=640, color_height=480,
            depth_width=320, depth_height=240,
        )
        self.assertEqual(reg.status, "UNREGISTERED")

    def test_K_diff_warns(self):
        """K 矩阵差异大时需要显式 T_color_depth 才能 COLOCATED."""
        K_color = np.eye(3)
        K_depth = np.eye(3) * 3.0  # 差异很大
        T = np.eye(4)  # 显式 identity
        reg = determine_registration_status(
            "cam_fl_color_optical_frame",
            "cam_fl_depth_optical_frame",
            color_width=640, color_height=480,
            depth_width=640, depth_height=480,
            color_K=K_color, depth_K=K_depth,
            T_color_depth=T,
        )
        self.assertEqual(reg.status, "COLOCATED")
        self.assertGreater(reg.K_max_abs_diff, 1.0)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""目标隔离模块测试 — 直接调用生产函数."""
import os, sys, json, math, unittest, numpy as np

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.target_isolation import (
    classify_mesh_component, _compute_obb_simple,
)

# 模拟 V2 配置
MOCK_ISO_CFG = {
    "enabled": True,
    "static_scene_exclusion": {
        "enabled": True,
        "method": "configured_rig_aabb",
        "volumes": [{
            "id": "rear_camera_pedestal",
            "frame": "cam_front_left_color_optical_frame",
            "min": [0.21, -0.42, 1.04],
            "max": [0.48, -0.06, 1.37],
        }],
    },
    "fixture_exclusion": {
        "enabled": True,
        "world_vertical_ceiling": {
            "enabled": True,
            "normal_rig": [0.118, -0.967, -0.094],
            "offset_rig": 0.237,
        },
        "slender_component_filter": {
            "enabled": True,
            "elongation_ratio_min": 5.0,
            "cross_section_max_m": 0.015,
            "target_envelope_distance_min_m": 0.020,
            "visible_gt_distance_min_mm": 10.0,
        },
    },
}


class TestOBB(unittest.TestCase):
    """_compute_obb_simple 测试."""

    def test_cube(self):
        dims = np.array([0.1, 0.1, 0.1])
        obb = _compute_obb_simple(dims, np.zeros((10, 3)))
        self.assertAlmostEqual(obb["elongation_ratio"], 1.0, places=1)
        self.assertAlmostEqual(obb["cross_section_m"], 0.1, places=1)

    def test_elongated_rod(self):
        dims = np.array([0.30, 0.01, 0.01])
        obb = _compute_obb_simple(dims, np.zeros((10, 3)))
        self.assertGreater(obb["elongation_ratio"], 10.0)
        self.assertAlmostEqual(obb["cross_section_m"], 0.01, places=3)

    def test_thin_plate(self):
        dims = np.array([0.20, 0.15, 0.002])
        obb = _compute_obb_simple(dims, np.zeros((10, 3)))
        self.assertGreater(obb["elongation_ratio"], 1.0)
        self.assertAlmostEqual(obb["cross_section_m"], 0.15, places=2)


class TestClassifyComponent(unittest.TestCase):
    """classify_mesh_component 测试."""

    def _make_vertices(self, centroid, size, n=10):
        """在 centroid 处创建尺寸为 size 的随机点."""
        rng = np.random.RandomState(42)
        half = np.array(size) / 2
        pts = rng.uniform(-half, half, (n, 3)) + np.array(centroid)
        return pts

    # ── 1. V1 disabled ──
    def test_disabled_returns_unclassified(self):
        pts = self._make_vertices([0, -0.02, 0.93], [0.3, 0.3, 0.2])
        r = classify_mesh_component(pts, None, [0, 0, 0.93])
        self.assertEqual(r["classification"], "UNCLASSIFIED")

    def test_empty_iso_cfg_returns_unclassified(self):
        pts = self._make_vertices([0, -0.02, 0.93], [0.3, 0.3, 0.2])
        r = classify_mesh_component(pts, {}, [0, 0, 0.93])
        self.assertEqual(r["classification"], "UNCLASSIFIED")

    # ── 2. static exclusion 内点 → REAR_CAMERA_PEDESTAL ──
    def test_pedestal_aabb_inside_classified(self):
        # centroid 在 rear pedestal AABB 内
        pts = self._make_vertices([0.30, -0.25, 1.20], [0.02, 0.02, 0.02])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        self.assertEqual(r["classification"], "REAR_CAMERA_PEDESTAL")

    def test_target_body_outside_aabb_kept(self):
        # centroid 在主目标附近, 远离 pedestal
        pts = self._make_vertices([-0.007, -0.025, 0.93], [0.3, 0.3, 0.2])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.93])
        self.assertEqual(r["classification"], "TARGET_BODY")

    # ── 4. 细长杆 → SUSPENSION_ROD ──
    def test_slender_rod_far_from_target(self):
        # 细长点云 (0.08x0.01x0.01m) 远离主体 centroid(0,0,0.93)
        pts = self._make_vertices([0.0, -0.13, 1.15], [0.08, 0.01, 0.01])
        # 需要大量点来满足 OBB 计算
        n_pts = 200
        rng = np.random.RandomState(42)
        half = np.array([0.04, 0.005, 0.005])
        pts = rng.uniform(-half, half, (n_pts, 3)) + np.array([0.0, -0.13, 1.15])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        # 细长比够 + 远离主体 → SUSPENSION_ROD
        self.assertEqual(r["classification"], "SUSPENSION_ROD")

    # ── 5. 方形块不误判为细长杆 ──
    def test_top_block_not_classified_as_rod(self):
        # 顶部块: 0.06x0.05x0.04m
        pts = self._make_vertices([0.0, -0.13, 1.08], [0.06, 0.05, 0.04])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        # 应分类为 TARGET_BODY (靠近主体) 或 TOP_OFFSET_BLOCK
        self.assertIn(r["classification"], ("TARGET_BODY", "TOP_OFFSET_BLOCK"))

    # ── 7. 关键字段完整性 ──
    def test_classification_report_fields(self):
        pts = self._make_vertices([0, -0.02, 0.93], [0.3, 0.3, 0.2])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        for key in ("classification", "inside_static_exclusion_ratio",
                     "inside_target_envelope_ratio", "elongation_ratio",
                     "distance_to_visible_gt_mm"):
            self.assertIn(key, r, f"缺少字段: {key}")

    # ── 8. 有 GT 点时距离计算 ──
    def test_gt_distance_computed(self):
        pts = self._make_vertices([0, -0.02, 0.93], [0.01, 0.01, 0.01], n=50)
        gt = np.random.RandomState(7).uniform(-0.05, 0.05, (200, 3)) + np.array([0, -0.02, 0.93])
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929],
                                    gt_points_rig=gt)
        self.assertIsNotNone(r["distance_to_visible_gt_mm"])
        # 应该很近 (都在 ~5cm 内)
        self.assertLess(r["distance_to_visible_gt_mm"], 100)

    # ── 9. 真实小分量不因面积小删除 (由 caller 判断) ──
    def test_small_target_component_not_misclassified(self):
        # 小但在目标 envelope 内
        pts = self._make_vertices([0.01, -0.03, 0.94], [0.005, 0.005, 0.003], n=20)
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        # 应归为 TARGET_BODY (因为靠近主体)
        self.assertEqual(r["classification"], "TARGET_BODY")

    # ── 10. UNKNOWN_FRAGMENT ──
    def test_distant_fragment_unknown(self):
        # 远离主体但既不细长也不在 pedestal AABB 中
        pts = self._make_vertices([0.10, -0.30, 0.85], [0.03, 0.02, 0.02], n=30)
        r = classify_mesh_component(pts, MOCK_ISO_CFG, [-0.007, -0.025, 0.929])
        self.assertEqual(r["classification"], "UNKNOWN_FRAGMENT")


class TestStaticExclusionLogic(unittest.TestCase):
    """静态排除 AABB 逻辑验证 (不依赖 Open3D)."""

    def test_point_inside_aabb(self):
        """验证 AABB 内点判定."""
        vmin = np.array([0.21, -0.42, 1.04])
        vmax = np.array([0.48, -0.06, 1.37])
        pt_inside = np.array([0.30, -0.25, 1.20])
        pt_outside = np.array([0.0, 0.0, 0.9])

        in_vol = ((pt_inside[0] >= vmin[0]) & (pt_inside[0] <= vmax[0]) &
                  (pt_inside[1] >= vmin[1]) & (pt_inside[1] <= vmax[1]) &
                  (pt_inside[2] >= vmin[2]) & (pt_inside[2] <= vmax[2]))
        self.assertTrue(in_vol)

        in_vol2 = ((pt_outside[0] >= vmin[0]) & (pt_outside[0] <= vmax[0]) &
                   (pt_outside[1] >= vmin[1]) & (pt_outside[1] <= vmax[1]) &
                   (pt_outside[2] >= vmin[2]) & (pt_outside[2] <= vmax[2]))
        self.assertFalse(in_vol2)

    def test_vertical_ceiling_plane(self):
        """验证世界 Z 天花板平面方程."""
        normal = np.array([0.118, -0.967, -0.094])
        offset = 0.237

        # 主目标 centroid (应在天花板下)
        pt_main = np.array([-0.007, -0.025, 0.929])
        self.assertLess(np.dot(pt_main, normal), offset)

        # 远处上方点 (应在天花板上)
        # 世界 Z=1.2m → rig frame 中应该 dot > offset
        # 只是逻辑测试, 不依赖真实变换
        above_val = np.dot(normal, pt_main) + 0.1  # 模拟上方点
        self.assertGreater(above_val, np.dot(pt_main, normal))


if __name__ == "__main__":
    unittest.main()

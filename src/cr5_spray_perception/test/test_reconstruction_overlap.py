#!/usr/bin/env python3
"""测试 common overlap 和点云融合."""
import os, sys, unittest
import numpy as np

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.pointcloud_fusion import (
    compute_common_overlap, _voxel_downsample_with_colors,
)


class TestCommonOverlap(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)

    def test_perfect_overlap_zero_distance(self):
        """完全相同的点云 → common overlap = 0."""
        points = np.random.randn(500, 3).astype(np.float32) * 0.1
        result = compute_common_overlap(points, points, max_distance_m=0.5)
        cm = result["common_metrics"]
        self.assertLess(cm["A_to_B"]["median_mm"], 0.01)
        self.assertLess(cm["B_to_A"]["median_mm"], 0.01)
        # 支持度: 所有点都应有互为最近邻
        s = result["support"]
        self.assertGreater(s["common_ratio_a"], 0.9)
        self.assertGreater(s["common_ratio_b"], 0.9)

    def test_separated_clouds_low_common(self):
        """分开的点云 → 共同支持低."""
        points_a = np.random.randn(500, 3).astype(np.float32) * 0.1
        points_b = points_a + np.array([10.0, 0, 0])  # 10m 远
        result = compute_common_overlap(points_a, points_b, max_distance_m=0.5)
        s = result["support"]
        self.assertEqual(s["n_common_a"], 0)
        self.assertEqual(s["n_common_b"], 0)

    def test_partial_overlap(self):
        """部分重叠 → 共同区域限制."""
        # A 在原点附近 + 远方, B 在原点附近
        points_a = np.vstack([
            np.random.randn(300, 3).astype(np.float32) * 0.1,
            np.random.randn(200, 3).astype(np.float32) * 0.1 + np.array([5, 5, 5]),
        ])
        points_b = np.random.randn(400, 3).astype(np.float32) * 0.1
        result = compute_common_overlap(points_a, points_b, max_distance_m=0.5)
        s = result["support"]
        # common 应该排除 B 独有的区域
        self.assertLess(s["common_ratio_a"], 0.8,
                        f"A 中远方点不应与 B 互最近邻, ratio={s['common_ratio_a']:.3f}")

    def test_raw_vs_common_difference(self):
        """raw 指标包含独有区域, common 排除."""
        points_a = np.vstack([
            np.random.randn(100, 3).astype(np.float32) * 0.02,
            np.array([[10, 10, 10]] * 100, dtype=np.float32),
        ])
        points_b = np.random.randn(150, 3).astype(np.float32) * 0.02
        result = compute_common_overlap(points_a, points_b, max_distance_m=0.5)
        # raw P95 应该更大 (被独有区域污染)
        raw_p95 = result["raw_pairwise"]["A_to_B"]["p95_mm"]
        common_p95 = result["common_metrics"]["A_to_B"]["p95_mm"]
        # 注意: raw 可能包含非 finite 值
        if not np.isnan(raw_p95) and not np.isnan(common_p95):
            self.assertGreater(raw_p95, common_p95,
                               f"raw P95 ({raw_p95:.1f}) 应 > common P95 ({common_p95:.1f})")

    def test_empty_input(self):
        """空输入."""
        result = compute_common_overlap(
            np.zeros((0, 3)), np.zeros((0, 3)))
        self.assertEqual(result["support"]["n_a_total"], 0)
        self.assertEqual(result["support"]["n_common_a"], 0)

    def test_support_ratio_output(self):
        """support 输出包含 ratio."""
        points_a = np.random.randn(200, 3).astype(np.float32) * 0.05
        points_b = np.random.randn(300, 3).astype(np.float32) * 0.05
        result = compute_common_overlap(points_a, points_b, max_distance_m=0.5)
        s = result["support"]
        self.assertIn("common_ratio_a", s)
        self.assertIn("common_ratio_b", s)
        self.assertGreaterEqual(s["common_ratio_a"], 0.0)
        self.assertLessEqual(s["common_ratio_a"], 1.0)


class TestVoxelDownsample(unittest.TestCase):

    def test_downsample_preserves_colors(self):
        """下采样保留颜色."""
        points = np.random.randn(1000, 3).astype(np.float32) * 0.1
        colors = np.random.rand(1000, 3).astype(np.float32)
        p_down, c_down = _voxel_downsample_with_colors(points, colors, 0.05)
        self.assertEqual(p_down.shape[1], 3)
        self.assertEqual(c_down.shape[1], 3)
        self.assertEqual(p_down.shape[0], c_down.shape[0])
        self.assertLess(p_down.shape[0], points.shape[0])

    def test_no_downsample_zero_voxel(self):
        """voxel=0 → 不下采样."""
        points = np.random.randn(100, 3).astype(np.float32)
        colors = np.random.rand(100, 3).astype(np.float32)
        p_down, c_down = _voxel_downsample_with_colors(points, colors, 0.0)
        self.assertEqual(p_down.shape, points.shape)
        self.assertEqual(c_down.shape, colors.shape)


if __name__ == "__main__":
    unittest.main()

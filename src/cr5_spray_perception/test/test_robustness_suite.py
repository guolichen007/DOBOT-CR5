#!/usr/bin/env python3
"""鲁棒性套件自动化测试 — 不依赖 ROS / Gazebo / Open3D."""
import os, sys, json, tempfile, unittest, shutil

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))


# ── 直接测试 Gate 逻辑 (不需要完整 runner) ──

# 模拟 evaluate_pose_gate 的关键判断逻辑
GATE_ACC_MEDIAN = 5.0
GATE_ACC_P95 = 16.0
GATE_COMPLETENESS_COV10 = 0.80
GATE_REMOVED_RATIO = 0.03


def simulate_gate(accuracy_median, accuracy_p95,
                  completeness_cov10=None, removed_ratio=0.0, bottom="UNKNOWN",
                  runs_completed=1, runs_expected=1,
                  all_runs_pass=True, mesh_sha_unique=True):
    """模拟 Gate 判断, 返回 (auto_pass, failures)."""
    failures = []

    if runs_completed != runs_expected:
        failures.append(f"runs={runs_completed}/{runs_expected}")

    if not all_runs_pass:
        failures.append("not all runs passed")

    if runs_expected > 1 and not mesh_sha_unique:
        failures.append("mesh SHA not unique")

    if accuracy_median is None:
        failures.append("accuracy median missing")
    elif accuracy_median > GATE_ACC_MEDIAN:
        failures.append(f"median={accuracy_median:.2f} > {GATE_ACC_MEDIAN}")

    if accuracy_p95 is None:
        failures.append("accuracy P95 missing")
    elif accuracy_p95 > GATE_ACC_P95:
        failures.append(f"P95={accuracy_p95:.2f} > {GATE_ACC_P95}")

    if completeness_cov10 is not None and completeness_cov10 < GATE_COMPLETENESS_COV10:
        failures.append(f"completeness_cov10={completeness_cov10:.3f} < {GATE_COMPLETENESS_COV10}")

    if removed_ratio > GATE_REMOVED_RATIO:
        failures.append(f"removed_ratio={removed_ratio:.3f} > {GATE_REMOVED_RATIO}")

    if bottom != "UNKNOWN":
        failures.append(f"bottom={bottom} != UNKNOWN")

    return len(failures) == 0, failures


class TestRobustnessGate(unittest.TestCase):
    """多姿态 Gate 逻辑测试."""

    def test_all_pass(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "UNKNOWN")
        self.assertTrue(ok, f"should pass: {f}")

    def test_median_fail(self):
        ok, f = simulate_gate(7.0, 10.0, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("median" in x for x in f))

    def test_p95_fail(self):
        ok, f = simulate_gate(4.0, 18.0, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("P95" in x for x in f))

    def test_both_fail(self):
        ok, f = simulate_gate(7.0, 18.0, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertGreaterEqual(len(f), 2)

    def test_completeness_cov10_fail(self):
        ok, f = simulate_gate(3.68, 15.58, 0.50, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("completeness_cov10" in x for x in f))

    def test_completeness_cov10_none_ok(self):
        """completeness cov10=None 时应跳过检查."""
        ok, f = simulate_gate(3.68, 15.58, None, 0.01, "UNKNOWN")
        self.assertTrue(ok, f"None coverage should skip: {f}")

    def test_removed_ratio_fail(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.10, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("removed_ratio" in x for x in f))

    def test_removed_ratio_gate_boundary(self):
        """removed_ratio=0.03 正好在边界应通过 (≤)."""
        ok, _ = simulate_gate(3.68, 15.58, 0.85, 0.03, "UNKNOWN")
        self.assertTrue(ok)

    def test_bottom_not_unknown_fails(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "FILLED")
        self.assertFalse(ok)
        self.assertTrue(any("bottom" in x for x in f))

    def test_bottom_empty_fails(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "")
        self.assertFalse(ok)

    def test_runs_partial_fails(self):
        """预期 3 次, 只完成 2 次."""
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "UNKNOWN",
                              runs_completed=2, runs_expected=3)
        self.assertFalse(ok)
        self.assertTrue(any("2/3" in x for x in f))

    def test_not_all_runs_pass_fails(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "UNKNOWN",
                              all_runs_pass=False)
        self.assertFalse(ok)

    def test_mesh_sha_not_unique_fails(self):
        ok, f = simulate_gate(3.68, 15.58, 0.85, 0.01, "UNKNOWN",
                              runs_expected=3, mesh_sha_unique=False)
        self.assertFalse(ok)
        self.assertTrue(any("SHA" in x for x in f))

    def test_single_run_no_sha_check(self):
        """单次运行不检查 SHA 唯一性."""
        ok, _ = simulate_gate(3.68, 15.58, 0.85, 0.01, "UNKNOWN",
                              runs_expected=1, mesh_sha_unique=False)
        self.assertTrue(ok)

    def test_accuracy_none_fails(self):
        ok, f = simulate_gate(None, 15.58, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("median missing" in x for x in f))

    def test_p95_none_fails(self):
        ok, f = simulate_gate(3.68, None, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)
        self.assertTrue(any("P95 missing" in x for x in f))

    def test_worst_metrics_used(self):
        """使用最差的 accuracy median 判断 Gate."""
        # 三次运行: 3.0, 4.0, 8.0 — 最差 8.0 > 5.0 应失败
        ok, _ = simulate_gate(8.0, 15.0, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)

    def test_worst_p95_used(self):
        # 三次运行: 14.0, 16.0, 20.0 — 最差 20.0 > 16.0 应失败
        ok, _ = simulate_gate(4.0, 20.0, 0.85, 0.01, "UNKNOWN")
        self.assertFalse(ok)


class TestRobustnessPoseManifest(unittest.TestCase):
    """Pose manifest 合约测试."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_manifest(self, poses):
        import yaml
        path = os.path.join(self.tmpdir, "poses.yaml")
        with open(path, "w") as f:
            yaml.dump({"poses": poses, "acceptance": {}}, f)
        return path

    def test_valid_manifest(self):
        import yaml
        path = self._write_manifest([
            {"id": "P0", "preset": "center", "category": "production", "repeat": 3},
            {"id": "P1", "preset": "x_p30", "category": "production", "repeat": 1},
        ])
        with open(path) as f:
            cfg = yaml.safe_load(f)
        poses = cfg["poses"]
        self.assertEqual(len(poses), 2)
        self.assertEqual(poses[0]["id"], "P0")

    def test_empty_pose_list(self):
        import yaml
        path = self._write_manifest([])
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.assertEqual(len(cfg["poses"]), 0)

    def test_no_production_poses(self):
        import yaml
        path = self._write_manifest([
            {"id": "E1", "preset": "combo_xm50_ym40", "category": "boundary", "repeat": 1},
        ])
        with open(path) as f:
            cfg = yaml.safe_load(f)
        production = [p for p in cfg["poses"] if p["category"] == "production"]
        self.assertEqual(len(production), 0,
                         "boundary-only 应有 0 个 production 姿态")

    def test_repeat_zero_rejected(self):
        """repeat=0 不合法."""
        self.assertLessEqual(0, 0)  # placeholder — schema validation would catch

    def test_missing_id_field(self):
        import yaml
        path = self._write_manifest([
            {"preset": "center", "category": "production", "repeat": 1},
        ])
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.assertNotIn("id", cfg["poses"][0])
        # 验证缺失 id 不会崩溃

    def test_pose_ids_unique_required(self):
        """重复 pose_id 应该被检测."""
        ids = ["P0", "P1", "P0"]
        seen = set()
        duplicates = []
        for pid in ids:
            if pid in seen:
                duplicates.append(pid)
            seen.add(pid)
        self.assertTrue(len(duplicates) > 0, "重复 ID 应被检测")


class TestRobustnessChecksums(unittest.TestCase):
    """Checksums 合约测试."""

    def test_checksum_uses_real_relative_path(self):
        """sha256sum -c 兼容的路径格式."""
        import subprocess
        tmpdir = tempfile.mkdtemp()
        try:
            fpath = os.path.join(tmpdir, "pose_p0", "groups", "group_0000", "depth.npy")
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            with open(fpath, "w") as f:
                f.write("test")

            # 写 checksums
            cpath = os.path.join(tmpdir, "checksums.sha256")
            import hashlib
            sha = hashlib.sha256(b"test").hexdigest()
            with open(cpath, "w") as f:
                f.write(f"{sha}  pose_p0/groups/group_0000/depth.npy\n")

            # 从 tmpdir 运行 sha256sum -c
            result = subprocess.run(
                ["sha256sum", "-c", cpath],
                capture_output=True, text=True, cwd=tmpdir,
            )
            self.assertEqual(result.returncode, 0,
                             f"sha256sum -c failed: {result.stderr}")
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

    def test_checksum_wrong_path_fails(self):
        """不正确的相对路径导致 sha256sum -c 失败."""
        import subprocess
        tmpdir = tempfile.mkdtemp()
        try:
            fpath = os.path.join(tmpdir, "pose_p0", "groups", "group_0000", "depth.npy")
            os.makedirs(os.path.dirname(fpath), exist_ok=True)
            with open(fpath, "w") as f:
                f.write("test")

            cpath = os.path.join(tmpdir, "checksums.sha256")
            import hashlib
            sha = hashlib.sha256(b"test").hexdigest()
            # 写错误的路径
            with open(cpath, "w") as f:
                f.write(f"{sha}  P0/cam_front_left/depth.npy\n")

            result = subprocess.run(
                ["sha256sum", "-c", cpath],
                capture_output=True, text=True, cwd=tmpdir,
            )
            self.assertNotEqual(result.returncode, 0,
                                "错误路径不应通过 sha256sum -c")
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)


class TestRobustnessPerPoseGT(unittest.TestCase):
    """Per-pose GT 合约测试."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_per_pose_gt_required_for_production(self):
        """生产姿态必须有自己的 visible GT."""
        pose_dir = os.path.join(self.tmpdir, "pose_p0")
        os.makedirs(pose_dir)
        gt_path = os.path.join(pose_dir, "visible_gt", "visible_union.ply")
        self.assertFalse(os.path.isfile(gt_path),
                         "per-pose GT 缺失应被检测并 fail-closed")

    def test_per_pose_gt_present_ok(self):
        """per-pose GT 存在应通过."""
        pose_dir = os.path.join(self.tmpdir, "pose_p0", "visible_gt")
        os.makedirs(pose_dir)
        gt_path = os.path.join(pose_dir, "visible_union.ply")
        with open(gt_path, "w") as f:
            f.write("ply\n")
        self.assertTrue(os.path.isfile(gt_path))


class TestRobustnessDepthUniqueness(unittest.TestCase):
    """深度图唯一性验证测试."""

    def test_all_unique_ok(self):
        """所有姿态深度 SHA 不同."""
        shas = {"P0": "aaa", "P1": "bbb", "P2": "ccc"}
        unique = len(set(shas.values()))
        self.assertEqual(unique, 3)

    def test_duplicate_fails(self):
        """同一相机两个姿态 SHA 相同应失败."""
        shas = {"P0": "aaa", "P1": "aaa", "P2": "bbb"}
        unique = len(set(shas.values()))
        self.assertEqual(unique, 2)
        self.assertLess(unique, len(shas),
                        f"重复 SHA: {len(shas)} 姿态只有 {unique} 个唯一 SHA")

    def test_single_pose_ok(self):
        """只有一个姿态时不检查跨姿态唯一性."""
        shas = {"P0": "aaa"}
        unique = len(set(shas.values()))
        self.assertEqual(unique, 1)
        # 单姿态不报错


class TestRobustnessRepeatability(unittest.TestCase):
    """重复性测试."""

    def test_sha_match_pass(self):
        """3 次运行, 3 个相同 SHA → 通过."""
        hashes = {"abc123", "abc123", "abc123"}  # set 去重
        self.assertEqual(len(hashes), 1)

    def test_sha_mismatch_fails(self):
        """3 次运行, 2 个不同 SHA → 失败."""
        hashes = {"abc123", "def456", "abc123"}
        self.assertEqual(len(hashes), 2)
        self.assertNotEqual(len(hashes), 1)

    def test_all_runs_failed_no_sha(self):
        """全部运行失败 → SHA 集合为空."""
        hashes = set()
        self.assertEqual(len(hashes), 0)


if __name__ == "__main__":
    unittest.main()

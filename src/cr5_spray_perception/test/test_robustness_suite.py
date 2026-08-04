#!/usr/bin/env python3
"""鲁棒性套件自动化测试 — 直接调用生产函数."""
import os, sys, json, math, tempfile, unittest, shutil, importlib.util

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

# scripts 不是 Python 包, 用 importlib 加载
_SUITE_PATH = os.path.join(WS, "scripts", "run_reconstruction_robustness_suite.py")
_spec = importlib.util.spec_from_file_location("robustness_suite", _SUITE_PATH)
_suite = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_suite)

extract_run_metrics = _suite.extract_run_metrics
aggregate_worst_metrics = _suite.aggregate_worst_metrics
evaluate_pose_gate = _suite.evaluate_pose_gate
GATE_COMPLETENESS_COV10 = _suite.GATE_COMPLETENESS_COV10
GATE_REMOVED_RATIO = _suite.GATE_REMOVED_RATIO
EXIT_AUTO_FAIL = _suite.EXIT_AUTO_FAIL
EXIT_MANUAL_PENDING = _suite.EXIT_MANUAL_PENDING

from cr5_spray_perception.reconstruction.quality_contract import (
    sanitize_evaluation_label,
)


def _make_quality_report(acc_median=3.68, acc_p95=15.58, acc_rmse=11.46,
                          comp_median=3.75, comp_p95=15.41,
                          comp_cov10=0.85, chamfer=6.30,
                          removed=0.01, bottom="UNKNOWN", prod_pass=True,
                          acc_cov10=0.76):
    return {
        "accuracy": {
            "median_mm": acc_median, "p95_mm": acc_p95, "rmse_mm": acc_rmse,
            "coverage_10mm": acc_cov10,
        },
        "completeness": {
            "median_mm": comp_median, "p95_mm": comp_p95,
            "coverage_10mm": comp_cov10,
        },
        "quality": {"chamfer_mm": chamfer},
        "acceptance": {"production_pass": prod_pass},
        "mesh": {"removed_fragment_area_ratio": removed,
                 "raw_components": 100, "cleaned_components": 95},
        "unobserved": {"bottom_surface": bottom},
    }


class TestExtractRunMetrics(unittest.TestCase):

    def test_all_fields_present(self):
        report = _make_quality_report()
        m = extract_run_metrics(report)
        self.assertEqual(m["accuracy_median_mm"], 3.68)
        self.assertEqual(m["accuracy_p95_mm"], 15.58)
        self.assertEqual(m["completeness_coverage_10mm"], 0.85)
        self.assertEqual(m["removed_fragment_area_ratio"], 0.01)
        self.assertEqual(m["bottom_status"], "UNKNOWN")
        self.assertTrue(m["production_pass"])

    def test_missing_fields_are_none(self):
        report = {"accuracy": {}, "completeness": {}, "quality": {},
                   "acceptance": {}, "mesh": {}, "unobserved": {}}
        m = extract_run_metrics(report)
        self.assertIsNone(m["accuracy_median_mm"])
        self.assertIsNone(m["completeness_coverage_10mm"])
        self.assertIsNone(m["removed_fragment_area_ratio"])
        self.assertIsNone(m["bottom_status"])
        self.assertIsNone(m["production_pass"])


class TestAggregateWorstMetrics(unittest.TestCase):

    def test_single_run_passthrough(self):
        r1 = extract_run_metrics(_make_quality_report())
        worst, errors = aggregate_worst_metrics([r1])
        self.assertEqual(errors, [])
        self.assertEqual(worst["accuracy_median_mm"], 3.68)
        self.assertEqual(worst["completeness_coverage_10mm"], 0.85)

    def test_worst_median_taken(self):
        r1 = extract_run_metrics(_make_quality_report(acc_median=3.0))
        r2 = extract_run_metrics(_make_quality_report(acc_median=8.0))
        r3 = extract_run_metrics(_make_quality_report(acc_median=4.0))
        worst, errors = aggregate_worst_metrics([r1, r2, r3])
        self.assertEqual(errors, [])
        self.assertEqual(worst["accuracy_median_mm"], 8.0)

    def test_worst_p95_taken(self):
        r1 = extract_run_metrics(_make_quality_report(acc_p95=14.0))
        r2 = extract_run_metrics(_make_quality_report(acc_p95=20.0))
        worst, errors = aggregate_worst_metrics([r1, r2])
        self.assertEqual(worst["accuracy_p95_mm"], 20.0)

    def test_worst_cov10_is_min(self):
        r1 = extract_run_metrics(_make_quality_report(comp_cov10=0.90))
        r2 = extract_run_metrics(_make_quality_report(comp_cov10=0.60))
        r3 = extract_run_metrics(_make_quality_report(comp_cov10=0.88))
        worst, errors = aggregate_worst_metrics([r1, r2, r3])
        self.assertEqual(worst["completeness_coverage_10mm"], 0.60)

    def test_worst_removed_is_max(self):
        r1 = extract_run_metrics(_make_quality_report(removed=0.01))
        r2 = extract_run_metrics(_make_quality_report(removed=0.05))
        worst, errors = aggregate_worst_metrics([r1, r2])
        self.assertEqual(worst["removed_fragment_area_ratio"], 0.05)

    def test_any_bottom_violation(self):
        r1 = extract_run_metrics(_make_quality_report(bottom="UNKNOWN"))
        r2 = extract_run_metrics(_make_quality_report(bottom="FILLED"))
        worst, errors = aggregate_worst_metrics([r1, r2])
        self.assertNotEqual(worst["bottom_status"], "UNKNOWN")

    def test_all_production_pass_required(self):
        r1 = extract_run_metrics(_make_quality_report(prod_pass=True))
        r2 = extract_run_metrics(_make_quality_report(prod_pass=False))
        worst, errors = aggregate_worst_metrics([r1, r2])
        self.assertFalse(worst["production_pass"])

    def test_missing_required_field_fails(self):
        r1 = extract_run_metrics(_make_quality_report())
        r2 = extract_run_metrics(_make_quality_report())
        del r2["completeness_coverage_10mm"]  # simulate missing
        r2["completeness_coverage_10mm"] = None
        _, errors = aggregate_worst_metrics([r1, r2])
        self.assertTrue(len(errors) > 0)
        self.assertTrue(any("completeness_coverage_10mm" in e for e in errors))

    def test_missing_removed_ratio_fails(self):
        r1 = extract_run_metrics(_make_quality_report())
        r2 = extract_run_metrics(_make_quality_report())
        r2["removed_fragment_area_ratio"] = None
        _, errors = aggregate_worst_metrics([r1, r2])
        self.assertTrue(len(errors) > 0)

    def test_missing_bottom_fails(self):
        r1 = extract_run_metrics(_make_quality_report())
        r2 = extract_run_metrics(_make_quality_report())
        r2["bottom_status"] = None
        _, errors = aggregate_worst_metrics([r1, r2])
        self.assertTrue(len(errors) > 0)

    def test_no_runs_fails(self):
        _, errors = aggregate_worst_metrics([])
        self.assertTrue(len(errors) > 0)


class TestEvaluatePoseGate(unittest.TestCase):

    def _worst_from_report(self, **kwargs):
        r = _make_quality_report(**kwargs)
        m = extract_run_metrics(r)
        worst, _ = aggregate_worst_metrics([m])
        return worst

    def test_all_pass(self):
        worst = self._worst_from_report()
        result, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertTrue(auto, f"should pass: {failures}")

    def test_median_fail(self):
        worst = self._worst_from_report(acc_median=7.0)
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)
        self.assertTrue(any("median" in f for f in failures))

    def test_p95_fail(self):
        worst = self._worst_from_report(acc_p95=18.0)
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_cov10_fail(self):
        worst = self._worst_from_report(comp_cov10=0.50)
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_cov10_missing_fails(self):
        worst = self._worst_from_report()
        worst["completeness_coverage_10mm"] = None
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_removed_missing_fails(self):
        worst = self._worst_from_report()
        worst["removed_fragment_area_ratio"] = None
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_bottom_missing_fails(self):
        worst = self._worst_from_report()
        worst["bottom_status"] = None
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_bottom_filled_fails(self):
        worst = self._worst_from_report(bottom="FILLED")
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    def test_removed_boundary_pass(self):
        worst = self._worst_from_report(removed=0.03)
        _, auto, _ = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertTrue(auto)

    def test_removed_boundary_fail(self):
        worst = self._worst_from_report(removed=0.031)
        _, auto, _ = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)

    # ── 重复性 ──

    def test_three_identical_sha_pass(self):
        worst = self._worst_from_report()
        _, auto, _ = evaluate_pose_gate(
            worst, mesh_hashes=["abc", "abc", "abc"],
            n_runs_completed=3, n_runs_expected=3)
        self.assertTrue(auto)

    def test_three_different_sha_fail(self):
        worst = self._worst_from_report()
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc", "def", "ghi"],
            n_runs_completed=3, n_runs_expected=3)
        self.assertFalse(auto)
        self.assertTrue(any("mismatch" in f for f in failures))

    def test_single_run_missing_sha_fails(self):
        worst = self._worst_from_report()
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=[""], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)
        self.assertTrue(any("missing" in f for f in failures))

    def test_sha_count_mismatch_fails(self):
        worst = self._worst_from_report()
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc", "abc"],
            n_runs_completed=2, n_runs_expected=3)
        self.assertFalse(auto)
        self.assertTrue(any("count" in f for f in failures))

    # ── 运行完整性 ──

    def test_partial_runs_fails(self):
        worst = self._worst_from_report()
        _, auto, failures = evaluate_pose_gate(
            worst, mesh_hashes=["abc"],
            n_runs_completed=2, n_runs_expected=3)
        self.assertFalse(auto)

    def test_not_all_pass_fails(self):
        worst = self._worst_from_report(prod_pass=False)
        _, auto, _ = evaluate_pose_gate(
            worst, mesh_hashes=["abc"], n_runs_completed=1, n_runs_expected=1)
        self.assertFalse(auto)


class TestExitCodes(unittest.TestCase):
    def test_auto_fail_is_1(self):
        self.assertEqual(EXIT_AUTO_FAIL, 1)

    def test_manual_pending_is_2(self):
        self.assertEqual(EXIT_MANUAL_PENDING, 2)


def _parse_group_dir(message):
    """解析 GROUP_DIR:/path|SYNC_GROUP_... 格式 (与 capture_multi_pose_dataset 一致)."""
    prefix = "GROUP_DIR:"
    if prefix not in message:
        raise ValueError(f"GROUP_DIR prefix missing")
    payload = message[message.index(prefix) + len(prefix):]
    group_dir = payload.split("|", 1)[0].strip()
    if not group_dir:
        raise ValueError("empty GROUP_DIR")
    if not os.path.isdir(group_dir):
        raise FileNotFoundError(f"GROUP_DIR not a directory: {group_dir}")
    return group_dir


class TestParseGroupDir(unittest.TestCase):
    """测试 GROUP_DIR 解析 (真实格式)."""

    def test_real_format(self):
        msg = "GROUP_DIR:/tmp/run/groups/group_0000|SYNC_GROUP_3_OF_3_PASS: 3/3 cameras"
        tmpdir = tempfile.mkdtemp()
        try:
            gdir = os.path.join(tmpdir, "groups", "group_0000")
            os.makedirs(gdir)
            msg_real = f"GROUP_DIR:{gdir}|SYNC_GROUP_3_OF_3_PASS: 3/3 cameras"
            result = _parse_group_dir(msg_real)
            self.assertEqual(result, gdir)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

    def test_missing_prefix_raises(self):
        with self.assertRaises(ValueError):
            _parse_group_dir("no prefix here")

    def test_empty_dir_raises(self):
        with self.assertRaises(ValueError):
            _parse_group_dir("GROUP_DIR:|xxx")


class TestChecksums(unittest.TestCase):

    def test_sha256sum_c_format(self):
        import hashlib, subprocess
        tmpdir = tempfile.mkdtemp()
        try:
            fpath = os.path.join(tmpdir, "pose_p0", "groups", "group_0000",
                                 "cam_front_left", "depth.npy")
            os.makedirs(os.path.dirname(fpath))
            with open(fpath, "w") as f:
                f.write("test")
            sha = hashlib.sha256(b"test").hexdigest()
            cpath = os.path.join(tmpdir, "checksums.sha256")
            # 真实相对路径
            with open(cpath, "w") as f:
                f.write(f"{sha}  pose_p0/groups/group_0000/cam_front_left/depth.npy\n")
            result = subprocess.run(["sha256sum", "-c", cpath],
                                    capture_output=True, text=True, cwd=tmpdir)
            self.assertEqual(result.returncode, 0)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)


class TestSanitizeLabel(unittest.TestCase):

    def test_path_safety(self):
        label = sanitize_evaluation_label("release/v1.0")
        self.assertNotIn("/", label)
        self.assertNotIn("..", label)


if __name__ == "__main__":
    unittest.main()

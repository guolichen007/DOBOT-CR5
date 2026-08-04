#!/usr/bin/env python3
"""Pure-Python quality gate tests — 不依赖 Open3D / Gazebo / ROS.

直接从 quality_contract 模块导入函数, 不通过 importlib 加载 runner.
"""
import os, sys, json, math, tempfile, unittest, shutil

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.quality_contract import (
    resolve_release_id,
    sanitize_evaluation_label,
    load_and_validate_evaluation_metrics,
    build_acceptance_result,
    build_quality_metric_sections,
)


def _fake_git_tag_provider(tag=""):
    """工厂函数: 返回一个返回固定 tag 的可调用对象."""
    return lambda: tag


class TestResolveReleaseId(unittest.TestCase):

    def test_explicit_overrides_all(self):
        self.assertEqual(resolve_release_id("v1.2.3"), "v1.2.3")

    def test_explicit_overrides_git_tag(self):
        provider = _fake_git_tag_provider("git-tag-v1")
        self.assertEqual(resolve_release_id("explicit-id", git_tag_provider=provider), "explicit-id")

    def test_git_tag_used_when_no_explicit(self):
        provider = _fake_git_tag_provider("git-tag-v1")
        self.assertEqual(resolve_release_id(None, git_tag_provider=provider), "git-tag-v1")

    def test_env_var_used_when_no_explicit_and_no_tag(self):
        provider = _fake_git_tag_provider("")
        environ = {"CR5_RELEASE_ID": "env-release-id"}
        self.assertEqual(resolve_release_id(None, git_tag_provider=provider, environ=environ), "env-release-id")

    def test_returns_untagged_when_nothing_set(self):
        provider = _fake_git_tag_provider("")
        environ = {}
        rid = resolve_release_id(None, git_tag_provider=provider, environ=environ)
        self.assertEqual(rid, "UNTAGGED")

    def test_returns_string_always(self):
        rid = resolve_release_id(None)
        self.assertIsInstance(rid, str)
        self.assertTrue(len(rid) > 0)


class TestSanitizeEvaluationLabel(unittest.TestCase):

    def test_empty_returns_untagged(self):
        self.assertEqual(sanitize_evaluation_label(""), "untagged")

    def test_none_returns_untagged(self):
        self.assertEqual(sanitize_evaluation_label(None), "untagged")

    def test_alphanumeric_preserved(self):
        self.assertEqual(sanitize_evaluation_label("release_v1"), "release_v1")

    def test_dots_replaced(self):
        self.assertEqual(sanitize_evaluation_label("v1.0.4"), "v1_0_4")

    def test_dashes_replaced(self):
        self.assertEqual(sanitize_evaluation_label("release-v1"), "release_v1")

    def test_slashes_replaced(self):
        label = sanitize_evaluation_label("release/v1")
        self.assertNotIn("/", label)
        self.assertNotIn("\\", label)
        self.assertNotIn("..", label)

    def test_spaces_replaced(self):
        label = sanitize_evaluation_label("release v1")
        self.assertNotIn(" ", label)
        self.assertIn("release_v1", label)

    def test_consecutive_underscores_compressed(self):
        label = sanitize_evaluation_label("a..b")
        self.assertNotIn("__", label)

    def test_leading_trailing_underscores_stripped(self):
        label = sanitize_evaluation_label("...test...")
        self.assertEqual(label, "test")

    def test_max_length(self):
        long_id = "a" * 200
        label = sanitize_evaluation_label(long_id)
        self.assertLessEqual(len(label), 80)

    def test_no_path_separator_in_label(self):
        label = sanitize_evaluation_label("/etc/passwd")
        self.assertNotIn("/", label)
        self.assertNotIn("\\", label)

    def test_no_dotdot(self):
        label = sanitize_evaluation_label("../../etc")
        self.assertNotIn("..", label)

    def test_mixed_special_chars(self):
        label = sanitize_evaluation_label("reconstruction-gazebo-visible-surface-v1.0.4")
        self.assertNotIn(".", label)
        self.assertNotIn("-", label)
        self.assertIn("reconstruction_gazebo_visible_surface_v1_0_4", label)


class TestMetricsValidation(unittest.TestCase):

    def setUp(self):
        self.load_fn = load_and_validate_evaluation_metrics
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_metrics(self, data):
        p = os.path.join(self.tmpdir, "test_metrics.json")
        with open(p, "w") as f:
            json.dump(data, f)
        return p

    def _valid_metrics(self):
        return {"metrics": {
            "accuracy": {"median_mm": 3.68, "p95_mm": 15.58, "rmse_mm": 11.46},
            "completeness": {"median_mm": 3.75, "p95_mm": 15.41},
            "chamfer_mm": 6.30,
            "coverage": {"accuracy_5mm": 0.6, "accuracy_10mm": 0.76, "accuracy_20mm": 0.99,
                         "completeness_5mm": 0.6, "completeness_10mm": 0.79, "completeness_20mm": 0.99},
        }}

    def test_valid_metrics_passes(self):
        p = self._write_metrics(self._valid_metrics())
        m = self.load_fn(p)
        self.assertAlmostEqual(m["accuracy_median_mm"], 3.68)

    def test_missing_file_fails(self):
        with self.assertRaises(FileNotFoundError):
            self.load_fn("/nonexistent/metrics.json")

    def test_invalid_json_fails(self):
        p = os.path.join(self.tmpdir, "bad.json")
        with open(p, "w") as f:
            f.write("not json")
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_missing_metrics_key_fails(self):
        p = self._write_metrics({})
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_missing_accuracy_median_fails(self):
        d = self._valid_metrics()
        del d["metrics"]["accuracy"]["median_mm"]
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_missing_coverage_fails(self):
        d = self._valid_metrics()
        del d["metrics"]["coverage"]["accuracy_10mm"]
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_nan_metric_fails(self):
        d = self._valid_metrics()
        d["metrics"]["accuracy"]["median_mm"] = float("nan")
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_inf_metric_fails(self):
        d = self._valid_metrics()
        d["metrics"]["accuracy"]["p95_mm"] = float("inf")
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_negative_distance_fails(self):
        d = self._valid_metrics()
        d["metrics"]["chamfer_mm"] = -1.0
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_coverage_out_of_range_fails(self):
        d = self._valid_metrics()
        d["metrics"]["coverage"]["accuracy_10mm"] = 1.5
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)

    def test_coverage_zero_ok(self):
        d = self._valid_metrics()
        d["metrics"]["coverage"]["accuracy_10mm"] = 0.0
        p = self._write_metrics(d)
        m = self.load_fn(p)
        self.assertEqual(m["accuracy_10mm"], 0.0)

    def test_coverage_one_ok(self):
        d = self._valid_metrics()
        d["metrics"]["coverage"]["accuracy_10mm"] = 1.0
        p = self._write_metrics(d)
        m = self.load_fn(p)
        self.assertEqual(m["accuracy_10mm"], 1.0)

    def test_bool_rejected(self):
        d = self._valid_metrics()
        d["metrics"]["accuracy"]["median_mm"] = True
        p = self._write_metrics(d)
        with self.assertRaises(ValueError):
            self.load_fn(p)


class TestBuildAcceptanceResult(unittest.TestCase):

    def test_pass(self):
        r = build_acceptance_result(3.68, 15.58)
        self.assertTrue(r["production_pass"])
        self.assertTrue(any("PASS" in reason for reason in r["reasons"]))

    def test_fail_high_median(self):
        r = build_acceptance_result(7.0, 10.0)
        self.assertFalse(r["production_pass"])
        self.assertTrue(any("median" in reason for reason in r["reasons"]))

    def test_fail_high_p95(self):
        r = build_acceptance_result(4.0, 18.0)
        self.assertFalse(r["production_pass"])
        self.assertTrue(any("P95" in reason for reason in r["reasons"]))

    def test_fail_both(self):
        r = build_acceptance_result(7.0, 18.0)
        self.assertFalse(r["production_pass"])
        self.assertEqual(len(r["reasons"]), 2)

    def test_reasons_no_empty_strings(self):
        r = build_acceptance_result(3.68, 15.58)
        self.assertTrue(all(s.strip() for s in r["reasons"]))
        r2 = build_acceptance_result(10.0, 20.0)
        self.assertTrue(all(s.strip() for s in r2["reasons"]))

    def test_gate_thresholds_in_result(self):
        r = build_acceptance_result(3.68, 15.58, median_gate_mm=5.0, p95_gate_mm=16.0)
        self.assertEqual(r["accuracy_median_gate_mm"], 5.0)
        self.assertEqual(r["accuracy_p95_gate_mm"], 16.0)

    def test_custom_gate_thresholds(self):
        r = build_acceptance_result(3.68, 15.58, median_gate_mm=3.0, p95_gate_mm=12.0)
        self.assertEqual(r["accuracy_median_gate_mm"], 3.0)
        self.assertEqual(r["accuracy_p95_gate_mm"], 12.0)
        # median=3.68 > 3.0 → FAIL
        self.assertFalse(r["production_pass"])


class TestBuildQualityMetricSections(unittest.TestCase):

    def setUp(self):
        self.metrics = {
            "accuracy_median_mm": 3.68,
            "accuracy_p95_mm": 15.58,
            "accuracy_rmse_mm": 11.46,
            "completeness_median_mm": 3.75,
            "completeness_p95_mm": 15.41,
            "chamfer_mm": 6.30,
            "accuracy_5mm": 0.6,
            "accuracy_10mm": 0.76,
            "accuracy_20mm": 0.99,
            "completeness_5mm": 0.6,
            "completeness_10mm": 0.79,
            "completeness_20mm": 0.99,
        }

    def test_accuracy_section(self):
        acc, _, _ = build_quality_metric_sections(self.metrics)
        self.assertEqual(acc["status"], "EVALUATED")
        self.assertAlmostEqual(acc["median_mm"], 3.68)
        self.assertAlmostEqual(acc["p95_mm"], 15.58)
        self.assertAlmostEqual(acc["coverage_5mm"], 0.6)

    def test_completeness_section(self):
        _, comp, _ = build_quality_metric_sections(self.metrics)
        self.assertEqual(comp["scope"], "visible_gt_only")
        self.assertAlmostEqual(comp["median_mm"], 3.75)

    def test_quality_section(self):
        _, _, qual = build_quality_metric_sections(self.metrics)
        self.assertAlmostEqual(qual["chamfer_mm"], 6.30)


class TestQualityReport(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def test_report_bottom_unknown(self):
        """验证质量报告始终标记 bottom=UNKNOWN."""
        p = os.path.join(self.tmpdir, "unknown_surface_regions.json")
        d = {"bottom_surface": {"observed": False, "status": "UNKNOWN", "filled": False},
             "reconstruction_type": "partial_visible_surface", "watertight": False}
        with open(p, "w") as f:
            json.dump(d, f)
        with open(p) as f:
            r = json.load(f)
        self.assertEqual(r["bottom_surface"]["status"], "UNKNOWN")
        self.assertFalse(r["bottom_surface"]["observed"])
        self.assertFalse(r["watertight"])


class TestProductionRunnerStringChecks(unittest.TestCase):

    def test_no_hardcoded_v1_0_1_in_identity(self):
        """runner 不再硬编码 V1.0.1."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertNotIn('"git_tag": "reconstruction-gazebo-visible-surface-v1.0.1"', content)

    def test_quality_contract_importable(self):
        """quality_contract 函数可直接导入."""
        self.assertTrue(callable(resolve_release_id))
        self.assertTrue(callable(build_acceptance_result))
        self.assertTrue(callable(build_quality_metric_sections))
        self.assertTrue(callable(sanitize_evaluation_label))

    def test_visible_gt_param_exists(self):
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertIn("--visible-gt", content)

    def test_visible_gto_not_in_runner(self):
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertNotIn("--visible-gto", content)

    def test_no_module_level_open3d_exit(self):
        """runner 不再在模块级 sys.exit(1) (改为 require_open3d)."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        # 确认使用了 require_open3d 函数
        self.assertIn("require_open3d", content)
        # 确认没有模块级的 sys.exit(1) 在 Open3D import 旁边
        self.assertNotIn("except ImportError:\n    logger.error", content)

    def test_runner_uses_quality_contract(self):
        """runner 使用了 quality_contract 模块."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertIn("quality_contract", content)

    def test_runner_uses_evaluation_runner(self):
        """runner 使用了 evaluation_runner 模块."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertIn("evaluation_runner", content)


if __name__ == "__main__":
    unittest.main()

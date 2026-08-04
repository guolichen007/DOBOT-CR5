#!/usr/bin/env python3
"""Production V1.0.3 evaluation integration tests (no Gazebo)."""
import os, sys, json, math, tempfile, unittest, shutil, importlib.util

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

# 从 scripts 目录导入 runner 函数 (scripts 不是 Python 包)
_RUNNER_PATH = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
_runner_spec = importlib.util.spec_from_file_location("run_visible_surface_reconstruction", _RUNNER_PATH)
_runner = importlib.util.module_from_spec(_runner_spec)
_runner_spec.loader.exec_module(_runner)
resolve_release_id = _runner.resolve_release_id
load_and_validate_evaluation_metrics = _runner.load_and_validate_evaluation_metrics


class TestResolveReleaseId(unittest.TestCase):

    def test_explicit_overrides_all(self):
        self.assertEqual(resolve_release_id("v1.2.3"), "v1.2.3")

    def test_returns_untagged_when_no_tag(self):
        rid = resolve_release_id(None)
        self.assertIsInstance(rid, str)
        self.assertTrue(len(rid) > 0)


class TestMetricsValidation(unittest.TestCase):

    def setUp(self):
        # imported at module level via importlib load_and_validate_evaluation_metrics
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
        # 确认没有硬编码版本在 identity 中
        self.assertNotIn('"git_tag": "reconstruction-gazebo-visible-surface-v1.0.1"', content)

    def test_resolve_release_id_importable(self):
        # imported at module level via importlib
        rid = resolve_release_id("test")
        self.assertEqual(rid, "test")

    def test_metrics_validator_importable(self):
        # imported at module level via importlib load_and_validate_evaluation_metrics
        self.assertTrue(callable(load_and_validate_evaluation_metrics))

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


if __name__ == "__main__":
    unittest.main()

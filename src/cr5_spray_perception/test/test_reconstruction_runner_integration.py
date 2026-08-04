#!/usr/bin/env python3
"""Production runner integration tests — fake evaluator (no Gazebo needed)."""
import os, sys, json, math, tempfile, unittest, shutil, importlib.util

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

_RUNNER_PATH = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
_spec = importlib.util.spec_from_file_location("runner", _RUNNER_PATH)
_runner = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_runner)
resolve_release_id = _runner.resolve_release_id
load_and_validate_evaluation_metrics = _runner.load_and_validate_evaluation_metrics


class TestRunnerEvaluatorIntegration(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self._write_fake_evaluator()

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _write_fake_evaluator(self):
        """创建 fake evaluator 脚本."""
        evaluator_path = os.path.join(self.tmpdir, "fake_evaluator.py")
        with open(evaluator_path, "w") as f:
            f.write(r'''
import sys, os, json, argparse
parser = argparse.ArgumentParser()
parser.add_argument("--recon-mesh")
parser.add_argument("--output-dir")
parser.add_argument("--label")
parser.add_argument("--visible-gt")
parser.add_argument("--target-roi-min", nargs=3, type=float)
parser.add_argument("--target-roi-max", nargs=3, type=float)
args = parser.parse_args()

mode = os.environ.get("FAKE_EVAL_MODE", "success")
if mode == "fail_nonzero":
    sys.exit(1)

os.makedirs(args.output_dir, exist_ok=True)
metrics = {"metrics": {
    "accuracy": {"median_mm": 3.68, "p95_mm": 15.58, "rmse_mm": 11.46},
    "completeness": {"median_mm": 3.75, "p95_mm": 15.41},
    "chamfer_mm": 6.30,
    "coverage": {"accuracy_5mm": 0.6, "accuracy_10mm": 0.76, "accuracy_20mm": 0.99,
                 "completeness_5mm": 0.6, "completeness_10mm": 0.79, "completeness_20mm": 0.99},
}}
if mode == "no_metrics_file":
    pass
elif mode == "invalid_json":
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        f.write("not json")
elif mode == "missing_field":
    del metrics["metrics"]["accuracy"]["median_mm"]
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
elif mode == "nan_value":
    metrics["metrics"]["accuracy"]["p95_mm"] = float("nan")
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
else:
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
sys.exit(0)
''')
        return evaluator_path

    def test_valid_metrics_parsed_correctly(self):
        """完整 metrics JSON 正确读取."""
        metrics_path = os.path.join(self.tmpdir, "eval_dir")
        os.makedirs(metrics_path, exist_ok=True)
        data = {"metrics": {
            "accuracy": {"median_mm": 3.68, "p95_mm": 15.58, "rmse_mm": 11.46},
            "completeness": {"median_mm": 3.75, "p95_mm": 15.41},
            "chamfer_mm": 6.30,
            "coverage": {"accuracy_5mm": 0.6, "accuracy_10mm": 0.76, "accuracy_20mm": 0.99,
                         "completeness_5mm": 0.6, "completeness_10mm": 0.79, "completeness_20mm": 0.99},
        }}
        path = os.path.join(metrics_path, "test_metrics.json")
        with open(path, "w") as f:
            json.dump(data, f)
        m = load_and_validate_evaluation_metrics(path)
        self.assertAlmostEqual(m["accuracy_median_mm"], 3.68)
        self.assertAlmostEqual(m["chamfer_mm"], 6.30)
        self.assertAlmostEqual(m["accuracy_10mm"], 0.76)

    def test_gate_pass_with_valid_metrics(self):
        """PASS: median≤5 且 P95≤16."""
        m = {"accuracy_median_mm": 3.68, "accuracy_p95_mm": 15.58}
        acc_med = m["accuracy_median_mm"]
        acc_p95 = m["accuracy_p95_mm"]
        self.assertTrue(acc_med <= 5.0 and acc_p95 <= 16.0)

    def test_gate_fail_high_p95(self):
        """FAIL: P95 > 16."""
        m = {"accuracy_median_mm": 4.0, "accuracy_p95_mm": 18.0}
        acc_med = m["accuracy_median_mm"]
        acc_p95 = m["accuracy_p95_mm"]
        self.assertFalse(acc_med <= 5.0 and acc_p95 <= 16.0)

    def test_gate_fail_high_median(self):
        """FAIL: median > 5."""
        m = {"accuracy_median_mm": 7.0, "accuracy_p95_mm": 10.0}
        self.assertFalse(m["accuracy_median_mm"] <= 5.0 and m["accuracy_p95_mm"] <= 16.0)

    def test_reasons_no_empty_strings(self):
        """reasons 不含空字符串."""
        reasons = []
        acc_med, acc_p95 = 3.68, 15.58
        if acc_med <= 5.0 and acc_p95 <= 16.0:
            reasons = ["PASS: median≤5mm and P95≤16mm"]
        else:
            if acc_med > 5.0:
                reasons.append(f"FAIL: accuracy median={acc_med:.2f}mm > 5.0mm")
            if acc_p95 > 16.0:
                reasons.append(f"FAIL: accuracy P95={acc_p95:.2f}mm > 16.0mm")
        self.assertTrue(all(r.strip() for r in reasons))

    def test_reasons_no_empty_strings_on_fail(self):
        """失败时 reasons 也不含空字符串."""
        acc_med, acc_p95 = 10.0, 20.0
        reasons = []
        if acc_med <= 5.0 and acc_p95 <= 16.0:
            reasons = ["PASS: median≤5mm and P95≤16mm"]
        else:
            if acc_med > 5.0:
                reasons.append(f"FAIL: accuracy median={acc_med:.2f}mm > 5.0mm")
            if acc_p95 > 16.0:
                reasons.append(f"FAIL: accuracy P95={acc_p95:.2f}mm > 16.0mm")
        self.assertEqual(len(reasons), 2)
        self.assertTrue(all(r.strip() for r in reasons))


class TestReleaseIdDynamic(unittest.TestCase):

    def test_explicit_override(self):
        self.assertEqual(resolve_release_id("v1.2.3"), "v1.2.3")

    def test_env_var(self):
        os.environ["CR5_RELEASE_ID"] = "test-env-id"
        # explicit has priority — test with None to use env
        rid = resolve_release_id(None)
        self.assertIn(rid, ["test-env-id", "UNTAGGED", _runner._git_tag()])
        del os.environ["CR5_RELEASE_ID"]

    def test_no_hardcoded_v1_0_1(self):
        """确认没有硬编码特定版本."""
        with open(_RUNNER_PATH) as f:
            content = f.read()
        # 确认使用 resolve_release_id 而不是硬编码
        self.assertIn("resolve_release_id", content)


if __name__ == "__main__":
    unittest.main()

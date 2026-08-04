#!/usr/bin/env python3
"""Production runner integration tests — 真正执行 fake evaluator subprocess.

测试 run_evaluator() 的完整 subprocess 调用链.
所有测试不依赖 Open3D / Gazebo / ROS.
"""
import os, sys, json, math, tempfile, unittest, shutil, subprocess

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.quality_contract import (
    resolve_release_id,
    sanitize_evaluation_label,
    load_and_validate_evaluation_metrics,
    build_acceptance_result,
)
from cr5_spray_perception.reconstruction.evaluation_runner import run_evaluator


# ── fake evaluator 模板 ─────────────────────────────────────────────────────

_FAKE_EVALUATOR_TEMPLATE = r'''#!/usr/bin/env python3
"""Fake evaluator for integration testing."""
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
    sys.stderr.write("simulated evaluator failure\n")
    sys.exit(1)

if mode == "fail_timeout":
    import time
    time.sleep(999)
    sys.exit(0)

os.makedirs(args.output_dir, exist_ok=True)

metrics = {
    "metrics": {
        "accuracy": {"median_mm": 3.68, "p95_mm": 15.58, "rmse_mm": 11.46},
        "completeness": {"median_mm": 3.75, "p95_mm": 15.41},
        "chamfer_mm": 6.30,
        "coverage": {"accuracy_5mm": 0.6, "accuracy_10mm": 0.76, "accuracy_20mm": 0.99,
                     "completeness_5mm": 0.6, "completeness_10mm": 0.79, "completeness_20mm": 0.99},
    }
}

if mode == "no_metrics_file":
    pass  # 不生成 metrics 文件
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
elif mode == "inf_value":
    metrics["metrics"]["accuracy"]["median_mm"] = float("inf")
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
elif mode == "negative_distance":
    metrics["metrics"]["chamfer_mm"] = -1.0
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
elif mode == "coverage_out_of_range":
    metrics["metrics"]["coverage"]["accuracy_10mm"] = 1.5
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
elif mode == "bool_value":
    metrics["metrics"]["accuracy"]["median_mm"] = True
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
else:
    with open(os.path.join(args.output_dir, f"{args.label}_metrics.json"), "w") as f:
        json.dump(metrics, f)
sys.exit(0)
'''


# ── helpers ─────────────────────────────────────────────────────────────────

def _make_fake_files(tmpdir, evaluator_mode=None):
    """在临时目录创建 fake evaluator / recon mesh / visible GT.

    Returns: (evaluator_path, recon_mesh_path, visible_gt_path, output_dir)
    """
    # fake evaluator
    evaluator_path = os.path.join(tmpdir, "fake_evaluator.py")
    with open(evaluator_path, "w") as f:
        f.write(_FAKE_EVALUATOR_TEMPLATE)

    # fake recon mesh
    recon_mesh_path = os.path.join(tmpdir, "recon.ply")
    with open(recon_mesh_path, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex 0\nend_header\n")

    # fake visible GT
    visible_gt_path = os.path.join(tmpdir, "visible_gt.ply")
    with open(visible_gt_path, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex 0\nend_header\n")

    output_dir = os.path.join(tmpdir, "eval_output")

    # 设置 mode
    if evaluator_mode:
        os.environ["FAKE_EVAL_MODE"] = evaluator_mode

    return evaluator_path, recon_mesh_path, visible_gt_path, output_dir


# ── 测试类 1: Fake evaluator subprocess 集成 ─────────────────────────────

class TestRunEvaluatorSubprocess(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        # 清除可能残留的 mode
        os.environ.pop("FAKE_EVAL_MODE", None)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)
        os.environ.pop("FAKE_EVAL_MODE", None)

    def test_successful_evaluation(self):
        """fake evaluator 成功退出, run_evaluator 读取 metrics."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test-v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertIn("eval_label", result)
        self.assertIn("metrics_path", result)
        self.assertIn("metrics", result)
        self.assertTrue(os.path.isfile(result["metrics_path"]))

    def test_dynamic_label_passed_to_evaluator(self):
        """evaluator 收到动态 label (从 release_id 安全化而来)."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="reconstruction-gazebo-visible-surface-v1.0.4", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        # label 不应包含 . 或 -
        self.assertNotIn(".", result["eval_label"])
        self.assertNotIn("-", result["eval_label"])
        # metrics 文件名使用安全 label
        expected_name = f"{result['eval_label']}_metrics.json"
        self.assertEqual(os.path.basename(result["metrics_path"]), expected_name)

    def test_metrics_filename_uses_sanitized_label(self):
        """metrics 文件名不包含路径分隔符."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test/v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertNotIn("/", result["eval_label"])
        self.assertNotIn("\\", result["eval_label"])
        self.assertNotIn("..", result["eval_label"])

    def test_accuracy_returned(self):
        """Accuracy 指标正确返回."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test-v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        m = result["metrics"]
        self.assertAlmostEqual(m["accuracy_median_mm"], 3.68)
        self.assertAlmostEqual(m["accuracy_p95_mm"], 15.58)
        self.assertAlmostEqual(m["accuracy_rmse_mm"], 11.46)

    def test_completeness_returned(self):
        """Completeness 指标正确返回."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test-v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        m = result["metrics"]
        self.assertAlmostEqual(m["completeness_median_mm"], 3.75)
        self.assertAlmostEqual(m["completeness_p95_mm"], 15.41)

    def test_coverage_six_fields_returned(self):
        """Coverage 六项全部正确返回."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test-v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        m = result["metrics"]
        for key in ["accuracy_5mm", "accuracy_10mm", "accuracy_20mm",
                     "completeness_5mm", "completeness_10mm", "completeness_20mm"]:
            self.assertIn(key, m, f"缺少 coverage field: {key}")

    def test_chamfer_returned(self):
        """Chamfer 指标正确返回."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="test-v1", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertAlmostEqual(result["metrics"]["chamfer_mm"], 6.30)

    def test_evaluator_nonzero_exit_raises(self):
        """evaluator 非零退出时抛出 RuntimeError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "fail_nonzero")
        with self.assertRaises(RuntimeError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_no_metrics_file_raises(self):
        """evaluator 不生成 metrics 文件时抛出 FileNotFoundError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "no_metrics_file")
        with self.assertRaises(FileNotFoundError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_invalid_json_raises(self):
        """evaluator 生成非法 JSON 时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "invalid_json")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_missing_field_raises(self):
        """evaluator 生成缺少字段的 metrics 时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "missing_field")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_nan_value_raises(self):
        """evaluator 输出 NaN 时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "nan_value")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_inf_value_raises(self):
        """evaluator 输出 Inf 时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "inf_value")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_negative_distance_raises(self):
        """evaluator 输出负距离时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "negative_distance")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_coverage_out_of_range_raises(self):
        """evaluator 输出超出范围的 coverage 时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "coverage_out_of_range")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_bool_value_raises(self):
        """evaluator 输出 bool 值时抛出 ValueError."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "bool_value")
        with self.assertRaises(ValueError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_missing_recon_mesh_raises(self):
        """recon mesh 缺失时抛出 FileNotFoundError."""
        ep, _, vp, od = _make_fake_files(self.tmpdir, "success")
        with self.assertRaises(FileNotFoundError):
            run_evaluator(
                evaluator_script=ep, recon_mesh="/nonexistent/recon.ply", output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_missing_visible_gt_raises(self):
        """visible GT 缺失时抛出 FileNotFoundError."""
        ep, rp, _, od = _make_fake_files(self.tmpdir, "success")
        with self.assertRaises(FileNotFoundError):
            run_evaluator(
                evaluator_script=ep, recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt="/nonexistent/gt.ply",
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_missing_evaluator_script_raises(self):
        """evaluator 脚本缺失时抛出 FileNotFoundError."""
        _, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        with self.assertRaises(FileNotFoundError):
            run_evaluator(
                evaluator_script="/nonexistent/eval.py", recon_mesh=rp, output_dir=od,
                release_id="test-v1", visible_gt=vp,
                roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
            )

    def test_release_id_with_slash_is_safe(self):
        """release_id 包含 / 时生成安全文件名."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="release/v1.0", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertNotIn("/", result["eval_label"])
        self.assertNotIn("..", result["eval_label"])

    def test_release_id_with_spaces_is_safe(self):
        """release_id 包含空格时生成安全文件名."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="release v1 0", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertNotIn(" ", result["eval_label"])

    def test_release_id_with_dots_dashes_is_safe(self):
        """release_id 包含点和横线时替换为下划线."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="release-v1.0.4", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertNotIn(".", result["eval_label"])
        self.assertNotIn("-", result["eval_label"])

    def test_label_no_path_separator(self):
        """label 不允许包含路径分隔符."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        result = run_evaluator(
            evaluator_script=ep, recon_mesh=rp, output_dir=od,
            release_id="../../../etc/passwd", visible_gt=vp,
            roi_min=[-0.2, -0.2, 0.6], roi_max=[0.2, 0.2, 1.2],
        )
        self.assertNotIn("/", result["eval_label"])
        self.assertNotIn("..", result["eval_label"])


# ── 测试类 2: Gate 决策 (通过 build_acceptance_result) ──────────────────

class TestGateDecisions(unittest.TestCase):

    def test_gate_pass(self):
        r = build_acceptance_result(3.68, 15.58)
        self.assertTrue(r["production_pass"])

    def test_gate_fail_median(self):
        r = build_acceptance_result(7.0, 10.0)
        self.assertFalse(r["production_pass"])

    def test_gate_fail_p95(self):
        r = build_acceptance_result(4.0, 18.0)
        self.assertFalse(r["production_pass"])

    def test_gate_fail_both(self):
        r = build_acceptance_result(7.0, 18.0)
        self.assertFalse(r["production_pass"])
        self.assertEqual(len(r["reasons"]), 2)

    def test_reasons_no_empty_strings_pass(self):
        r = build_acceptance_result(3.68, 15.58)
        self.assertTrue(all(s.strip() for s in r["reasons"]))

    def test_reasons_no_empty_strings_fail(self):
        r = build_acceptance_result(10.0, 20.0)
        self.assertTrue(all(s.strip() for s in r["reasons"]))


# ── 测试类 3: CLI 级测试 (subprocess 启动轻量测试入口) ──────────────────

class TestCliLevelEvaluatorFailure(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        os.environ.pop("FAKE_EVAL_MODE", None)

    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)
        os.environ.pop("FAKE_EVAL_MODE", None)

    def _write_cli_harness(self):
        """创建一个 CLI 测试 harness, 在 subprocess 中调用 run_evaluator."""
        harness_path = os.path.join(self.tmpdir, "cli_harness.py")
        with open(harness_path, "w") as f:
            f.write('''#!/usr/bin/env python3
"""CLI harness for testing evaluator failure exit code."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))
from cr5_spray_perception.reconstruction.evaluation_runner import run_evaluator

evaluator = os.environ["TEST_EVALUATOR_SCRIPT"]
recon = os.environ["TEST_RECON_MESH"]
gt = os.environ["TEST_VISIBLE_GT"]
out_dir = os.environ["TEST_OUTPUT_DIR"]
release_id = os.environ.get("TEST_RELEASE_ID", "test-v1")

try:
    run_evaluator(
        evaluator_script=evaluator,
        recon_mesh=recon,
        output_dir=out_dir,
        release_id=release_id,
        visible_gt=gt,
        roi_min=[-0.2, -0.2, 0.6],
        roi_max=[0.2, 0.2, 1.2],
    )
    sys.exit(0)
except Exception as e:
    print(f"EVALUATOR_FAILED: {e}", file=sys.stderr)
    sys.exit(1)
''')
        return harness_path

    def test_evaluator_failure_nonzero_exit_code(self):
        """evaluator 失败时 CLI 进程退出码非零."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "fail_nonzero")
        harness = self._write_cli_harness()

        env = {
            **os.environ,
            "TEST_EVALUATOR_SCRIPT": ep,
            "TEST_RECON_MESH": rp,
            "TEST_VISIBLE_GT": vp,
            "TEST_OUTPUT_DIR": od,
            "TEST_RELEASE_ID": "test-v1",
            "FAKE_EVAL_MODE": "fail_nonzero",
        }
        # 需要 PYTHONPATH 确保 quality_contract 可导入
        env["PYTHONPATH"] = os.path.join(WS, "src") + ":" + env.get("PYTHONPATH", "")

        result = subprocess.run(
            [sys.executable, harness],
            capture_output=True, text=True, timeout=30,
            env=env,
        )
        self.assertNotEqual(result.returncode, 0,
                            f"expected non-zero exit, got {result.returncode}")

    def test_evaluator_success_zero_exit_code(self):
        """evaluator 成功时 CLI 进程退出码为零."""
        ep, rp, vp, od = _make_fake_files(self.tmpdir, "success")
        harness = self._write_cli_harness()

        env = {
            **os.environ,
            "TEST_EVALUATOR_SCRIPT": ep,
            "TEST_RECON_MESH": rp,
            "TEST_VISIBLE_GT": vp,
            "TEST_OUTPUT_DIR": od,
            "TEST_RELEASE_ID": "test-v1",
            "FAKE_EVAL_MODE": "success",
        }
        env["PYTHONPATH"] = os.path.join(WS, "src") + ":" + env.get("PYTHONPATH", "")

        result = subprocess.run(
            [sys.executable, harness],
            capture_output=True, text=True, timeout=30,
            env=env,
        )
        self.assertEqual(result.returncode, 0,
                         f"expected zero exit, got {result.returncode}: {result.stderr}")


# ── 测试类 4: Release ID 动态性 ──────────────────────────────────────────

class TestReleaseIdDynamic(unittest.TestCase):

    def setUp(self):
        os.environ.pop("CR5_RELEASE_ID", None)

    def tearDown(self):
        os.environ.pop("CR5_RELEASE_ID", None)

    def test_explicit_override(self):
        self.assertEqual(resolve_release_id("v1.2.3"), "v1.2.3")

    def test_no_hardcoded_v1_0_1(self):
        """确认运行器没有硬编码特定版本."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertNotIn('"git_tag": "reconstruction-gazebo-visible-surface-v1.0.1"', content)
        self.assertIn("resolve_release_id", content)


if __name__ == "__main__":
    unittest.main()

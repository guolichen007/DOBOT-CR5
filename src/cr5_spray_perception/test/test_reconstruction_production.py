#!/usr/bin/env python3
"""Production V1 封版回归测试."""
import os, sys, json, yaml, unittest, tempfile, shutil

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))


class TestProductionConfig(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cfg_path = os.path.join(WS, "config", "reconstruction", "visible_surface_production_v1.yaml")
        with open(cfg_path) as f:
            cls.config = yaml.safe_load(f)

    def test_schema_correct(self):
        self.assertEqual(self.config["schema_version"], "cr5_visible_surface_production_v1")

    def test_rig_source_stable_v1(self):
        self.assertEqual(self.config["rig"]["source"], "stable_v1")

    def test_allow_oracle_false(self):
        self.assertFalse(self.config["rig"]["allow_oracle"])

    def test_runtime_refinement_false(self):
        self.assertFalse(self.config["rig"]["allow_runtime_refinement"])

    def test_refinement_rejected(self):
        self.assertEqual(self.config["rig"]["refinement_status"], "REJECTED")

    def test_depth_edge_filter_disabled(self):
        self.assertFalse(self.config["filters"]["depth_edge_filter"]["enabled"])

    def test_multiview_default_disabled(self):
        self.assertFalse(self.config["filters"]["multiview_consistency"]["enabled"])

    def test_bottom_unknown(self):
        unobs = self.config.get("unobserved_surface", {})
        self.assertFalse(unobs["bottom"]["observed"])
        self.assertEqual(unobs["bottom"]["status"], "UNKNOWN")

    def test_reconstruction_partial_surface(self):
        self.assertEqual(self.config["reconstruction"]["type"], "partial_visible_surface")
        self.assertFalse(self.config["reconstruction"]["require_watertight"])

    def test_production_gate(self):
        acc = self.config["acceptance"]["production"]
        self.assertEqual(acc["accuracy_median_mm"], 5.0)
        self.assertEqual(acc["accuracy_p95_mm"], 16.0)

    def test_tsdf_params_frozen(self):
        tsdf = self.config["tsdf"]
        self.assertEqual(tsdf["voxel_length_m"], 0.005)
        self.assertEqual(tsdf["sdf_trunc_m"], 0.020)

    def test_integrated_frames_3(self):
        self.assertEqual(self.config["reconstruction"]["integrated_frames"], 3)


class TestProductionIsolation(unittest.TestCase):
    """验证生产模块不依赖 Gazebo/Oracle."""

    def test_pose_refinement_not_in_production_path(self):
        """refine_reconstruction_rig.py 不在一键入口导入链中."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertNotIn("pose_refinement", content)
        self.assertNotIn("refine_reconstruction_rig", content)

    def test_oracle_not_in_production_path(self):
        """生产入口代码不导入 Oracle (排除 docstring 和注释)."""
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            lines = f.readlines()
        code_lines = [l for l in lines if not l.strip().startswith('"""') and not l.strip().startswith('#')
                      and '正式外参' not in l and 'oracle diagnostic' not in l.lower()]
        code_only = "\n".join(code_lines)
        self.assertNotIn("oracle_rig", code_only.lower())
        self.assertNotIn("oracle_calibrated", code_only.lower())
        self.assertNotIn("gazebo_oracle_truth", code_only)

    def test_ablation_not_in_production_path(self):
        runner = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        with open(runner) as f:
            content = f.read()
        self.assertNotIn("ablation", content)

    def test_mesh_cleanup_preserves_offset_block(self):
        """mesh_cleanup 配置不满足只保留最大分量."""
        cfg = self._load_prod_config()
        mc = cfg.get("mesh_cleanup", {})
        self.assertFalse(mc.get("keep_largest_only", True))

    def test_mesh_cleanup_no_bottom_fill(self):
        """mesh_cleanup 模块的代码逻辑不包含 fill/bottom/watertight (排除 docstring)."""
        mc_path = os.path.join(WS, "src", "cr5_spray_perception", "reconstruction", "mesh_cleanup.py")
        with open(mc_path) as f:
            lines = f.readlines()
        code_only = "\n".join(l for l in lines if not l.strip().startswith('"""') and not l.strip().startswith('#') and '禁止' not in l and 'watertight' not in l)
        self.assertNotIn("fill_holes", code_only)
        self.assertNotIn("poisson", code_only.lower())

    def _load_prod_config(self):
        cfg_path = os.path.join(WS, "config", "reconstruction", "visible_surface_production_v1.yaml")
        with open(cfg_path) as f:
            return yaml.safe_load(f)


class TestProductionConfigContract(unittest.TestCase):

    def test_config_file_exists(self):
        path = os.path.join(WS, "config", "reconstruction", "visible_surface_production_v1.yaml")
        self.assertTrue(os.path.isfile(path), f"缺失: {path}")

    def test_production_runner_exists(self):
        path = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        self.assertTrue(os.path.isfile(path), f"缺失: {path}")

    def test_production_runner_executable(self):
        path = os.path.join(WS, "scripts", "run_visible_surface_reconstruction.py")
        self.assertTrue(os.access(path, os.R_OK), f"不可读: {path}")


if __name__ == "__main__":
    unittest.main()

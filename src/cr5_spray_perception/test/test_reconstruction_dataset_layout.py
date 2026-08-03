#!/usr/bin/env python3
"""测试 dataset_layout 路径解析器."""
import os, sys, json, tempfile, unittest

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest, list_available_groups,
    DatasetLayout,
)


class TestDatasetLayout(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def _make_run_structure(self, group_id=0, prefix="group", sub="groups"):
        """创建 run/groups/group_XXXX/cam_xxx/ 结构."""
        run_dir = os.path.join(self.tmpdir, "run_001")
        groups_dir = os.path.join(run_dir, sub)
        group_dir = os.path.join(groups_dir, f"{prefix}_{group_id:04d}")
        for cam in ["cam_front_left", "cam_front_right", "cam_rear"]:
            os.makedirs(os.path.join(group_dir, cam))
        return run_dir, groups_dir, group_dir

    def test_resolve_run_root(self):
        """run 根目录 → groups/group_0000."""
        run_dir, _, group_dir = self._make_run_structure()
        layout = resolve_capture_group(run_dir, 0)
        self.assertTrue(layout.is_valid)
        self.assertEqual(layout.group_dir, group_dir)
        self.assertEqual(layout.layout_type, "groups")
        self.assertEqual(layout.run_root, run_dir)

    def test_resolve_groups_root(self):
        """groups 根目录 → group_0000."""
        _, groups_dir, group_dir = self._make_run_structure()
        layout = resolve_capture_group(groups_dir, 0)
        self.assertTrue(layout.is_valid)
        self.assertEqual(layout.group_dir, group_dir)
        self.assertEqual(layout.layout_type, "groups")

    def test_resolve_direct_group(self):
        """直接 group_0000 目录."""
        run_dir, _, group_dir = self._make_run_structure()
        layout = resolve_capture_group(group_dir, 0)
        self.assertTrue(layout.is_valid)
        self.assertEqual(layout.group_dir, group_dir)
        self.assertEqual(layout.group_id, 0)

    def test_resolve_views_format(self):
        """views 格式 (view_0000)."""
        run_dir, _, view_dir = self._make_run_structure(prefix="view", sub="views")
        layout = resolve_capture_group(run_dir, 0)
        self.assertTrue(layout.is_valid)
        self.assertEqual(layout.group_dir, view_dir)
        self.assertEqual(layout.layout_type, "views")

    def test_resolve_views_root(self):
        """views 根目录 → view_0000."""
        _, views_dir, view_dir = self._make_run_structure(prefix="view", sub="views")
        layout = resolve_capture_group(views_dir, 0)
        self.assertTrue(layout.is_valid)
        self.assertEqual(layout.group_dir, view_dir)

    def test_resolve_direct_view(self):
        """直接 view_0000 目录."""
        _, _, view_dir = self._make_run_structure(prefix="view", sub="views")
        layout = resolve_capture_group(view_dir, 0)
        self.assertTrue(layout.is_valid)

    def test_nonexistent_path_fails(self):
        """不存在的路径抛出 FileNotFoundError."""
        with self.assertRaises(FileNotFoundError):
            resolve_capture_group("/nonexistent/path/12345", 0)

    def test_empty_dir_fails(self):
        """空目录无 group 时抛出 FileNotFoundError."""
        empty_dir = os.path.join(self.tmpdir, "empty")
        os.makedirs(empty_dir)
        with self.assertRaises(FileNotFoundError):
            resolve_capture_group(empty_dir, 0)

    def test_wrong_group_id_fails(self):
        """group_0000 存在但查 group_0005 失败."""
        run_dir, _, _ = self._make_run_structure(group_id=0)
        with self.assertRaises(FileNotFoundError):
            resolve_capture_group(run_dir, 5)

    def test_camera_dirs_scanned(self):
        """相机子目录被正确扫描."""
        run_dir, _, group_dir = self._make_run_structure()
        layout = resolve_capture_group(run_dir, 0)
        self.assertIn("cam_front_left", layout.camera_dirs)
        self.assertIn("cam_front_right", layout.camera_dirs)
        self.assertIn("cam_rear", layout.camera_dirs)

    def test_manifest_path(self):
        """manifest 路径正确."""
        run_dir, _, group_dir = self._make_run_structure()
        layout = resolve_capture_group(run_dir, 0)
        self.assertEqual(
            layout.manifest_path,
            os.path.join(group_dir, "group_manifest.json"))

    def test_load_group_manifest(self):
        """加载 group_manifest.json."""
        run_dir, _, group_dir = self._make_run_structure()
        manifest_path = os.path.join(group_dir, "group_manifest.json")
        data = {
            "success": True, "captured": 3, "expected": 3,
            "camera_names": ["cam_front_left", "cam_front_right", "cam_rear"],
            "cross_camera_sync": {
                "method": "cross_camera_bounded_skew",
                "max_inter_camera_skew_s": 0.002,
                "max_allowed_skew_s": 0.005,
            },
        }
        with open(manifest_path, "w") as f:
            json.dump(data, f)
        manifest = load_group_manifest(group_dir)
        self.assertIsNotNone(manifest)
        self.assertTrue(manifest["success"])

    def test_manifest_missing_returns_none(self):
        """无 manifest 返回 None."""
        run_dir, _, _ = self._make_run_structure()
        manifest = load_group_manifest(os.path.join(run_dir, "groups", "group_0000"))
        self.assertIsNone(manifest)

    def test_list_available_groups(self):
        """列出可用 group ID."""
        for gid in [0, 1, 3, 5]:
            self._make_run_structure(group_id=gid)
        run_dir = os.path.join(self.tmpdir, "run_001")
        ids = list_available_groups(run_dir)
        self.assertEqual(ids, [0, 1, 3, 5])


if __name__ == "__main__":
    unittest.main()

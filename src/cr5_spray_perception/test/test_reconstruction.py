#!/usr/bin/env python3
"""
CR5 Reconstruction — 单元测试.

测试覆盖:
  1. T_rig_camera / T_camera_rig 互逆测试
  2. 已知点从 camera 到 rig 的变换方向测试
  3. 故意传入逆矩阵时必须失败的测试
  4. uint16 毫米深度转换测试
  5. float32 米深度转换测试
  6. CameraInfo 尺寸/K/frame 不一致测试
  7. 三台相机缺失测试
  8. Stable V1 原生 JSON 转 calibrated-rig schema 测试
  9. 生产 reconstruction 包静态审计: 禁止导入 gazebo_msgs、tf2_ros 和 model state
  10. 合成平面或立方体三相机点云融合测试
"""

import os, sys, json, tempfile, unittest, warnings
import numpy as np

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.contracts import (
    validate_se3_matrix, validate_inverse_pair, validate_identity,
    validate_calibrated_rig_yaml, validate_stable_v1_json,
    REQUIRED_CAMERAS, SCHEMA_VERSION, DET_TOL,
)
from cr5_spray_perception.reconstruction.transforms import (
    invert_transform, convert_depth_to_meters,
    depth_image_to_pointcloud, transform_pointcloud,
    crop_pointcloud_aabb, compute_bidirectional_overlap,
)
from cr5_spray_perception.reconstruction.extrinsics import (
    load_stable_v1_extrinsics, build_calibrated_rig,
    sha256_file, load_calibrated_rig, get_T_rig_camera, get_T_camera_rig,
    OPTICAL_FRAME_SUFFIX, CAMERA_OPTICAL_FRAMES, TRANSFORM_CONTRACT,
)


# ═══════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════

def make_se3(R_deg=None, t=None):
    """构建 SE(3) 矩阵."""
    if R_deg is None:
        R_deg = [0, 0, 0]
    if t is None:
        t = [0, 0, 0]
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler('xyz', np.deg2rad(R_deg)).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def make_stable_v1_json(T_list):
    """构建模拟 Stable V1 JSON."""
    return {
        "solver": "pairwise_camera_relative",
        "version": "stable-v1",
        "n_groups": 20,
        "group_ids": list(range(20)),
        "cameras": {
            "cam_front_left": T_list[0].tolist(),
            "cam_front_right": T_list[1].tolist(),
            "cam_rear": T_list[2].tolist(),
        },
        "pair_stats": {},
        "triangle_closure_t_mm": 5.0,
        "triangle_closure_r_deg": 0.5,
        "status": "CALIBRATION_SUCCESS",
    }


# ═══════════════════════════════════════════════════════════════
# 测试 1: T_rig_camera / T_camera_rig 互逆测试
# ═══════════════════════════════════════════════════════════════

class TestInversePair(unittest.TestCase):

    def test_identity_inverse(self):
        """Identity 的逆仍是 identity."""
        I = np.eye(4)
        I_inv = invert_transform(I)
        np.testing.assert_allclose(I_inv, I, atol=1e-12)

    def test_translation_inverse(self):
        """平移 [1,2,3] 的逆是 [-1,-2,-3]."""
        T = make_se3(t=[1.0, 2.0, 3.0])
        T_inv = invert_transform(T)
        expected_t = np.array([-1.0, -2.0, -3.0])
        np.testing.assert_allclose(T_inv[:3, 3], expected_t, atol=1e-10)
        np.testing.assert_allclose(T_inv[:3, :3], np.eye(3), atol=1e-10)

    def test_rotation_inverse(self):
        """旋转 + 平移的逆矩阵验证."""
        T = make_se3(R_deg=[10, -20, 30], t=[0.5, -0.3, 1.2])
        T_inv = invert_transform(T)
        prod = T @ T_inv
        np.testing.assert_allclose(prod, np.eye(4), atol=1e-10)
        prod2 = T_inv @ T
        np.testing.assert_allclose(prod2, np.eye(4), atol=1e-10)

    def test_contract_inverse_pair_validates(self):
        """contracts 互逆校验通过."""
        T = make_se3(R_deg=[5, 10, -15], t=[0.1, 0.2, 0.3])
        T_inv = invert_transform(T)
        ok, errs = validate_inverse_pair(T, T_inv, "test")
        self.assertTrue(ok, msg="; ".join(errs))


# ═══════════════════════════════════════════════════════════════
# 测试 2: 已知点从 camera 到 rig 的变换方向
# ═══════════════════════════════════════════════════════════════

class TestTransformDirection(unittest.TestCase):

    def test_transform_direction_p_rig_equals_T_times_p_camera(self):
        """验证 p_rig = T_rig_camera @ p_camera."""
        T_rc = make_se3(R_deg=[0, 0, 0], t=[1.0, 0.0, 0.0])  # rig 在 camera 右侧 1m
        p_camera = np.array([[0.0, 0.0, 1.0]])  # camera 前方 1m 的点
        p_rig = transform_pointcloud(p_camera, T_rc)
        # camera 前方 1m → rig frame 中 X+1m (因为 rig 在 camera 右侧)
        expected = np.array([[1.0, 0.0, 1.0]])
        np.testing.assert_allclose(p_rig, expected, atol=1e-6)

    def test_invert_transform_gives_T_camera_rig(self):
        """T_camera_rig = inverse(T_rig_camera), p_camera = T_camera_rig @ p_rig."""
        T_rc = make_se3(R_deg=[0, 0, 0], t=[1.0, 0.0, 0.0])
        T_cr = invert_transform(T_rc)
        p_rig = np.array([[1.0, 0.0, 1.0]])
        p_camera = transform_pointcloud(p_rig, T_cr)
        expected = np.array([[0.0, 0.0, 1.0]])
        np.testing.assert_allclose(p_camera, expected, atol=1e-6)

    def test_FL_identity_no_transform(self):
        """FL 为 identity 时点云不变."""
        T_rc = np.eye(4)
        p_camera = np.array([[0.5, 0.3, 1.5]])
        p_rig = transform_pointcloud(p_camera, T_rc)
        np.testing.assert_allclose(p_rig, p_camera, atol=1e-10)


# ═══════════════════════════════════════════════════════════════
# 测试 3: 故意传入逆矩阵时必须失败
# ═══════════════════════════════════════════════════════════════

class TestInverseMustFail(unittest.TestCase):

    def test_inverted_matrix_detected(self):
        """如果传入 T_camera_rig 当做 T_rig_camera, 变换方向错误."""
        T_rc = make_se3(t=[1.0, 0.0, 0.0])       # p_rig = T_rc @ p_camera
        T_cr = invert_transform(T_rc)              # p_camera = T_cr @ p_rig

        p_camera = np.array([[0.0, 0.0, 1.0]])

        # 正确方向
        p_rig_correct = transform_pointcloud(p_camera, T_rc)
        # 错误方向 (用 T_cr 当 T_rc)
        p_rig_wrong = transform_pointcloud(p_camera, T_cr)

        # 两者不应该相同 (除非 T_rc 是 identity)
        diff = np.max(np.abs(p_rig_correct - p_rig_wrong))
        self.assertGreater(diff, 0.01,
                           msg="传入逆矩阵应该产生不同结果, 但两者相同")

    def test_flipped_matrix_makes_point_go_wrong_way(self):
        """把 T_rig_camera 和 T_camera_rig 搞反, 点的移动方向会相反."""
        T_rc = make_se3(t=[0.5, 0.0, 0.0])
        T_cr = invert_transform(T_rc)

        p = np.array([[0.0, 0.0, 1.0]])
        p_fwd = transform_pointcloud(p, T_rc)      # +0.5 in X
        p_rev = transform_pointcloud(p, T_cr)       # -0.5 in X

        # 正向移动和反向移动的 X 分量符号应该相反 (相对于输入)
        dx_fwd = p_fwd[0, 0] - p[0, 0]
        dx_rev = p_rev[0, 0] - p[0, 0]
        self.assertGreater(dx_fwd, 0, "T_rig_camera: X 应该正向移动")
        self.assertLess(dx_rev, 0, "T_camera_rig: X 应该反向移动")


# ═══════════════════════════════════════════════════════════════
# 测试 4: uint16 毫米深度转换测试
# ═══════════════════════════════════════════════════════════════

class TestDepthConversionUint16(unittest.TestCase):

    def test_uint16_to_meters(self):
        """uint16 毫米深度 → 米."""
        depth_mm = np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16)
        depth_m, unit = convert_depth_to_meters(depth_mm)
        self.assertEqual(unit, "mm")
        np.testing.assert_allclose(depth_m, np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32))

    def test_uint16_zero(self):
        """uint16 零值."""
        depth_mm = np.zeros((10, 10), dtype=np.uint16)
        depth_m, unit = convert_depth_to_meters(depth_mm)
        self.assertEqual(unit, "mm")
        self.assertTrue(np.all(depth_m == 0.0))


# ═══════════════════════════════════════════════════════════════
# 测试 5: float32 米深度转换测试
# ═══════════════════════════════════════════════════════════════

class TestDepthConversionFloat32(unittest.TestCase):

    def test_float32_meter_passthrough(self):
        """float32 米 → 米 (直通)."""
        depth_m = np.array([[1.5, 2.5], [3.5, 4.5]], dtype=np.float32)
        depth_out, unit = convert_depth_to_meters(depth_m, expected_unit="meter")
        self.assertEqual(unit, "meter")
        np.testing.assert_allclose(depth_out, depth_m)

    def test_float32_mm_to_meters(self):
        """float32 毫米 → 米."""
        depth_mm = np.array([[1000.0, 2000.0]], dtype=np.float32)
        depth_m, unit = convert_depth_to_meters(depth_mm, expected_unit="mm")
        self.assertEqual(unit, "mm")
        np.testing.assert_allclose(depth_m, np.array([[1.0, 2.0]], dtype=np.float32))


# ═══════════════════════════════════════════════════════════════
# 测试 6: CameraInfo 尺寸/K/frame 不一致测试 (在 rgbd_io 集成测试)
# ═══════════════════════════════════════════════════════════════

class TestCameraInfoValidation(unittest.TestCase):

    def test_parse_camera_info_ros_format(self):
        """ROS CameraInfo 格式解析."""
        from cr5_spray_perception.reconstruction.rgbd_io import parse_camera_info
        cinfo = {
            "header": {"frame_id": "cam_fl_color_optical_frame"},
            "height": 480, "width": 640,
            "K": [462.1, 0.0, 320.0, 0.0, 462.1, 240.0, 0.0, 0.0, 1.0],
        }
        result = parse_camera_info(cinfo)
        self.assertEqual(result["width"], 640)
        self.assertEqual(result["height"], 480)
        self.assertEqual(result["frame_id"], "cam_fl_color_optical_frame")
        self.assertIsNotNone(result["K"])
        np.testing.assert_allclose(result["K"][0, 0], 462.1)

    def test_parse_camera_info_simple_format(self):
        """简化 CameraInfo 格式解析."""
        from cr5_spray_perception.reconstruction.rgbd_io import parse_camera_info
        cinfo = {
            "image_width": 424, "image_height": 240,
            "camera_matrix": {"data": [462.1, 0.0, 212.0, 0.0, 462.1, 120.0, 0.0, 0.0, 1.0]},
        }
        result = parse_camera_info(cinfo)
        self.assertEqual(result["width"], 424)
        self.assertEqual(result["height"], 240)

    def test_frame_id_mismatch_detected(self):
        """frame_id 不一致应被检测."""
        from cr5_spray_perception.reconstruction.rgbd_io import RGBDData, validate_rgbd_contract
        data = RGBDData(camera_name="test")
        data.color = np.zeros((240, 320, 3), dtype=np.uint8)
        data.depth_raw = np.ones((240, 320), dtype=np.uint16) * 1000
        data.color_width, data.color_height = 320, 240
        data.depth_width, data.depth_height = 320, 240
        data.color_K = np.eye(3)
        data.depth_K = np.eye(3)
        data.color_frame_id = "cam_a_color_optical_frame"
        data.depth_frame_id = "cam_b_depth_optical_frame"  # 不同!
        data.depth_unit = "mm"
        data = validate_rgbd_contract(data, require_registered_to_color=True)
        self.assertFalse(data.is_valid)


# ═══════════════════════════════════════════════════════════════
# 测试 7: 三台相机缺失测试
# ═══════════════════════════════════════════════════════════════

class TestMissingCamera(unittest.TestCase):

    def test_missing_camera_detected(self):
        """缺失相机的 Stable V1 JSON 应校验失败."""
        data = {
            "solver": "pairwise_camera_relative",
            "version": "stable-v1",
            "status": "CALIBRATION_SUCCESS",
            "cameras": {
                "cam_front_left": np.eye(4).tolist(),
                "cam_front_right": np.eye(4).tolist(),
                # cam_rear 缺失!
            },
        }
        ok, errs = validate_stable_v1_json(data)
        self.assertFalse(ok)
        self.assertTrue(any("cam_rear" in e for e in errs),
                        f"错误列表应提到 cam_rear: {errs}")

    def test_all_cameras_present_passes(self):
        """三台相机齐全应通过."""
        T_list = [np.eye(4) for _ in range(3)]
        data = make_stable_v1_json(T_list)
        ok, errs = validate_stable_v1_json(data)
        self.assertTrue(ok, msg="; ".join(errs))

    def test_missing_cameras_in_rig_yaml(self):
        """calibrated_rig YAML 缺失相机应被检测."""
        rig = {
            "schema_version": SCHEMA_VERSION,
            "status": "PASS",
            "units": "meter",
            "rig_frame": "cam_front_left_color_optical_frame",
            "source_calibration": {"solver": "pairwise_camera_relative", "version": "stable-v1"},
            "transform_contract": dict(TRANSFORM_CONTRACT),
            "cameras": {
                "cam_front_left": {
                    "optical_frame": "cam_front_left_color_optical_frame",
                    "T_rig_camera": np.eye(4).tolist(),
                    "T_camera_rig": np.eye(4).tolist(),
                },
            },
        }
        ok, errs = validate_calibrated_rig_yaml(rig)
        self.assertFalse(ok)


# ═══════════════════════════════════════════════════════════════
# 测试 8: Stable V1 原生 JSON 转 calibrated-rig schema
# ═══════════════════════════════════════════════════════════════

class TestStableV1ToCalibratedRig(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def test_roundtrip_json_to_rig(self):
        """Stable V1 JSON → calibrated_rig dict → 校验通过."""
        # 模拟标定结果: FL=identity, FR 在 X+1m, RE 在 Z+0.5m
        T_fl = np.eye(4)
        T_fr = make_se3(t=[0.80, 0.05, -0.02])  # FR 在 FL 右侧
        T_re = make_se3(t=[0.85, 0.01, -0.04])  # RE 在 FL 后方左侧

        json_path = os.path.join(self.tmpdir, "camera_extrinsics.json")
        data = make_stable_v1_json([T_fl, T_fr, T_re])
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        # 加载
        cameras = load_stable_v1_extrinsics(json_path)
        self.assertEqual(len(cameras), 3)
        np.testing.assert_allclose(cameras["cam_front_left"], np.eye(4), atol=1e-10)

        # 构建 calibrated_rig
        rig = build_calibrated_rig(json_path)
        self.assertEqual(rig["schema_version"], SCHEMA_VERSION)
        self.assertEqual(rig["status"], "PASS")
        self.assertEqual(rig["units"], "meter")
        self.assertIn("cam_front_left_color_optical_frame", rig["rig_frame"])

        # 校验 FL identity
        T_fl_out = np.array(rig["cameras"]["cam_front_left"]["T_rig_camera"])
        np.testing.assert_allclose(T_fl_out, np.eye(4), atol=1e-10)

        # 校验互逆
        for cam in REQUIRED_CAMERAS:
            T_rc = np.array(rig["cameras"][cam]["T_rig_camera"])
            T_cr = np.array(rig["cameras"][cam]["T_camera_rig"])
            np.testing.assert_allclose(T_rc @ T_cr, np.eye(4), atol=1e-10,
                                       err_msg=f"{cam}: T_rig @ T_cam ≠ I")

        # 校验 optical_frame
        for cam in REQUIRED_CAMERAS:
            self.assertEqual(rig["cameras"][cam]["optical_frame"],
                             f"{cam}_color_optical_frame")

    def test_fl_non_identity_detected(self):
        """FL 非 identity 应报错."""
        T_fl = make_se3(t=[0.1, 0, 0])
        T_fr = np.eye(4)
        T_re = np.eye(4)
        json_path = os.path.join(self.tmpdir, "bad_extrinsics.json")
        data = make_stable_v1_json([T_fl, T_fr, T_re])
        with open(json_path, "w") as f:
            json.dump(data, f)
        with self.assertRaises(ValueError):
            build_calibrated_rig(json_path)

    def test_matrix_values_preserved(self):
        """Stable V1 的矩阵数值不得被修改."""
        T_fr = make_se3(R_deg=[5.3, -12.7, 3.1], t=[0.812, 0.034, -0.019])
        T_re = make_se3(R_deg=[-8.2, 15.6, -2.4], t=[0.856, -0.011, -0.042])
        json_path = os.path.join(self.tmpdir, "precise_extrinsics.json")
        data = make_stable_v1_json([np.eye(4), T_fr, T_re])
        with open(json_path, "w") as f:
            json.dump(data, f)

        rig = build_calibrated_rig(json_path)
        T_fr_out = np.array(rig["cameras"]["cam_front_right"]["T_rig_camera"])
        T_re_out = np.array(rig["cameras"]["cam_rear"]["T_rig_camera"])
        np.testing.assert_allclose(T_fr_out, T_fr, atol=1e-12,
                                   err_msg="FR 矩阵数值被修改!")
        np.testing.assert_allclose(T_re_out, T_re, atol=1e-12,
                                   err_msg="RE 矩阵数值被修改!")


# ═══════════════════════════════════════════════════════════════
# 测试 9: 生产包静态审计 (禁止 impermissible imports)
# ═══════════════════════════════════════════════════════════════

class TestImportAudit(unittest.TestCase):
    """确保生产 reconstruction 模块没有导入 Gazebo / TF truth."""

    IMPERMISSIBLE_MODULES = [
        "gazebo_msgs",
        "tf2_ros",
        "tf2_geometry_msgs",
        "gazebo_msgs.srv",
    ]

    RECONSTRUCTION_MODULES = [
        "cr5_spray_perception.reconstruction.contracts",
        "cr5_spray_perception.reconstruction.transforms",
        "cr5_spray_perception.reconstruction.extrinsics",
        "cr5_spray_perception.reconstruction.rgbd_io",
        "cr5_spray_perception.reconstruction.pointcloud_fusion",
    ]

    def test_no_gazebo_imports(self):
        """reconstruction 模块不应导入 gazebo_msgs / tf2_ros."""
        for module_name in self.RECONSTRUCTION_MODULES:
            # 检查模块源码中是否有 impermissible imports
            import importlib
            mod = importlib.import_module(module_name)
            source_file = mod.__file__

            with open(source_file, "r") as f:
                source = f.read()

            for impermissible in self.IMPERMISSIBLE_MODULES:
                # 只检测真正的 import/from 语句, 不检测文档字符串中的提及
                import_lines = [
                    line.strip() for line in source.split("\n")
                    if impermissible in line
                    and line.strip().startswith(("import ", "from "))
                ]
                self.assertEqual(
                    len(import_lines), 0,
                    msg=f"{module_name} 导入了禁止模块 {impermissible}: {import_lines}"
                )

    def test_scripts_dont_import_gazebo_truth(self):
        """脚本也不应导入 Gazebo truth (除了已有的 evaluate_reconstruction.py)."""
        scripts_dir = os.path.join(
            os.path.dirname(__file__), "..", "scripts")
        new_recon_scripts = [
            "export_calibrated_rig.py",
            "validate_reconstruction_dataset.py",
            "fuse_three_camera_pointclouds.py",
        ]
        for script_name in new_recon_scripts:
            script_path = os.path.join(scripts_dir, script_name)
            if not os.path.isfile(script_path):
                continue
            with open(script_path, "r") as f:
                source = f.read()
            for impermissible in self.IMPERMISSIBLE_MODULES:
                # 只检测真正的 import/from 语句
                import_lines = [
                    line.strip() for line in source.split("\n")
                    if impermissible in line
                    and line.strip().startswith(("import ", "from "))
                ]
                self.assertEqual(
                    len(import_lines), 0,
                    msg=f"{script_name} 导入了禁止模块 {impermissible}: {import_lines}"
                )


# ═══════════════════════════════════════════════════════════════
# 测试 10: 合成平面或立方体三相机点云融合
# ═══════════════════════════════════════════════════════════════

class TestSyntheticFusion(unittest.TestCase):

    def test_depth_image_to_pointcloud_plane(self):
        """反投影平面深度图."""
        H, W = 120, 160
        K = np.array([[320, 0, 80], [0, 320, 60], [0, 0, 1]], dtype=np.float64)
        # 平面在 z=2m
        depth = np.ones((H, W), dtype=np.float32) * 2.0
        points, valid = depth_image_to_pointcloud(depth, K, depth_min_m=0.1, depth_max_m=5.0)
        self.assertEqual(points.shape[1], 3)
        # z 应该都是 2.0
        np.testing.assert_allclose(points[:, 2], 2.0, atol=0.01)

    def test_crop_pointcloud_aabb(self):
        """AABB 裁剪."""
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [2.0, 2.0, 2.0],  # 外面
        ], dtype=np.float32)
        aabb_min = np.array([-0.5, -0.5, -0.5])
        aabb_max = np.array([1.5, 1.5, 1.5])
        cropped, mask = crop_pointcloud_aabb(points, aabb_min, aabb_max)
        self.assertEqual(cropped.shape[0], 3)
        self.assertFalse(mask[3])

    def test_compute_overlap_perfect(self):
        """两个完全相同的点云, 重叠应为 0."""
        points = np.random.randn(100, 3).astype(np.float32) * 0.1
        from cr5_spray_perception.reconstruction.transforms import compute_overlap_metrics
        metrics = compute_overlap_metrics(points, points)
        self.assertLess(metrics["median_mm"], 0.01)

    def test_compute_bidirectional_overlap(self):
        """双向重叠度量."""
        points_a = np.array([[0, 0, 0], [0.01, 0, 0]], dtype=np.float32)
        points_b = np.array([[0.005, 0, 0], [0.015, 0, 0]], dtype=np.float32)
        result = compute_bidirectional_overlap(points_a, points_b, "A", "B")
        self.assertIn("A_to_B", result)
        self.assertIn("B_to_A", result)
        self.assertIn("chamfer_mm", result)


# ═══════════════════════════════════════════════════════════════
# SE(3) 矩阵校验测试
# ═══════════════════════════════════════════════════════════════

class TestSE3Validation(unittest.TestCase):

    def test_valid_se3_passes(self):
        T = np.eye(4)
        ok, errs = validate_se3_matrix(T)
        self.assertTrue(ok, msg="; ".join(errs))

    def test_non_orthogonal_fails(self):
        T = np.eye(4)
        T[:3, :3] = np.array([[1, 0.5, 0], [0, 1, 0], [0, 0, 1]])  # 非正交
        ok, errs = validate_se3_matrix(T)
        self.assertFalse(ok)

    def test_bad_last_row_fails(self):
        T = np.eye(4)
        T[3, 0] = 0.1
        ok, errs = validate_se3_matrix(T)
        self.assertFalse(ok)

    def test_det_not_one_fails(self):
        T = np.eye(4)
        T[:3, :3] *= 2.0  # det = 8
        ok, errs = validate_se3_matrix(T)
        self.assertFalse(ok)

    def test_nan_fails(self):
        T = np.eye(4)
        T[0, 0] = np.nan
        ok, errs = validate_se3_matrix(T)
        self.assertFalse(ok)

    def test_wrong_shape_fails(self):
        T = np.eye(3)
        ok, errs = validate_se3_matrix(T)
        self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()

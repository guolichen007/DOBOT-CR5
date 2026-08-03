"""
CR5 Reconstruction — 外参与数据契约校验.

提供:
  - validate_se3_matrix:     4×4 矩阵的 finite / SO(3) / 底行校验
  - validate_inverse_pair:   T @ T_inv ≈ I 互逆校验
  - validate_identity:       矩阵是否为单位矩阵
  - validate_calibrated_rig_schema: 完整 calibrated_rig YAML schema 校验

所有校验函数返回 (passed: bool, errors: List[str]).
生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import numpy as np
from typing import Tuple, List, Dict, Optional

# ── 容忍度 ──
DET_TOL = 0.01          # det(R) 偏离 1.0 的容忍度
INVERSE_TOL_T_MM = 0.1  # 互逆校验平移容忍度 (mm)
INVERSE_TOL_T_M = 1e-7  # 互逆校验平移容忍度 (m)
INVERSE_TOL_R_DEG = 0.01  # 互逆校验旋转容忍度 (度)
IDENTITY_TOL_T_MM = 0.01  # identity 校验平移容忍度 (mm)
IDENTITY_TOL_R_DEG = 0.001  # identity 校验旋转容忍度 (度)

REQUIRED_CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
SCHEMA_VERSION = "cr5_calibrated_rig_v1"
REQUIRED_SOLVER = "pairwise_camera_relative"
REQUIRED_VERSION = "stable-v1"
REQUIRED_STATUS = "CALIBRATION_SUCCESS"


def validate_se3_matrix(T: np.ndarray, label: str = "matrix") -> Tuple[bool, List[str]]:
    """校验 4×4 矩阵是否合法的 SE(3).

    Args:
        T: 4×4 numpy array.
        label: 错误消息中的矩阵标识.

    Returns:
        (passed, errors)
    """
    errors = []

    if T is None:
        errors.append(f"{label}: 矩阵为 None")
        return False, errors

    if not isinstance(T, np.ndarray):
        errors.append(f"{label}: 不是 numpy 数组")
        return False, errors

    if T.shape != (4, 4):
        errors.append(f"{label}: shape={T.shape}, 期望 (4,4)")
        return False, errors

    # 所有数值 finite
    if not np.all(np.isfinite(T)):
        errors.append(f"{label}: 包含 NaN 或 Inf")

    # 底行必须为 [0,0,0,1]
    last_row = T[3, :]
    if not np.allclose(last_row, [0, 0, 0, 1], atol=1e-10):
        errors.append(f"{label}: 底行为 {last_row}, 期望 [0,0,0,1]")

    # 旋转矩阵正交: R @ R^T ≈ I
    R = T[:3, :3]
    RRT = R @ R.T
    if not np.allclose(RRT, np.eye(3), atol=1e-8):
        errors.append(f"{label}: 旋转矩阵非正交, max|R@R^T - I| = {np.max(np.abs(RRT - np.eye(3))):.2e}")

    # det(R) ≈ 1
    det = np.linalg.det(R)
    if abs(det - 1.0) > DET_TOL:
        errors.append(f"{label}: det(R)={det:.6f}, 偏离 1.0 超过 {DET_TOL}")

    return len(errors) == 0, errors


def validate_inverse_pair(T: np.ndarray, T_inv: np.ndarray,
                          label: str = "pair") -> Tuple[bool, List[str]]:
    """校验 T @ T_inv ≈ I 和 T_inv @ T ≈ I.

    Args:
        T: 正向变换 4×4.
        T_inv: 逆向变换 4×4.
        label: 标识.

    Returns:
        (passed, errors)
    """
    errors = []
    I4 = np.eye(4)

    prod1 = T @ T_inv
    err1 = np.max(np.abs(prod1 - I4))
    if err1 > INVERSE_TOL_T_M * 10:
        errors.append(f"{label}: T @ T_inv 偏离 I, max|diff|={err1:.2e}")

    prod2 = T_inv @ T
    err2 = np.max(np.abs(prod2 - I4))
    if err2 > INVERSE_TOL_T_M * 10:
        errors.append(f"{label}: T_inv @ T 偏离 I, max|diff|={err2:.2e}")

    return len(errors) == 0, errors


def validate_identity(T: np.ndarray, label: str = "matrix") -> Tuple[bool, List[str]]:
    """校验 4×4 矩阵是否为单位矩阵 (允许浮点误差).

    Args:
        T: 4×4 numpy array.
        label: 标识.

    Returns:
        (passed, errors)
    """
    errors = []
    I4 = np.eye(4)
    if not np.allclose(T, I4, atol=1e-10):
        max_diff = np.max(np.abs(T - I4))
        errors.append(f"{label}: 不是 identity, max|diff|={max_diff:.2e}")
    return len(errors) == 0, errors


def validate_calibrated_rig_yaml(data: dict) -> Tuple[bool, List[str]]:
    """校验完整的 calibrated_rig YAML schema.

    必须包含:
      - schema_version == cr5_calibrated_rig_v1
      - status == PASS
      - units == meter
      - rig_frame
      - source_calibration (solver, version, sha256)
      - transform_contract (primary, equation)
      - cameras: 三台相机各含 optical_frame, T_rig_camera, T_camera_rig

    Args:
        data: 已解析的 YAML dict.

    Returns:
        (passed, errors)
    """
    errors = []

    # 顶层字段
    valid_schemas = (SCHEMA_VERSION, "cr5_reconstruction_refined_rig_v1")
    if data.get("schema_version") not in valid_schemas:
        errors.append(f"schema_version: 期望 {'/'.join(valid_schemas)}, 实际 {data.get('schema_version')}")

    if data.get("status") != "PASS":
        errors.append(f"status: 期望 PASS, 实际 {data.get('status')}")

    if data.get("units") != "meter":
        errors.append(f"units: 期望 meter, 实际 {data.get('units')}")

    if not data.get("rig_frame"):
        errors.append("rig_frame 缺失")

    # source_calibration
    sc = data.get("source_calibration", {})
    valid_solvers = (REQUIRED_SOLVER, "gazebo_oracle_truth", "bounded_rgbd_pairwise_refinement")
    valid_versions = (REQUIRED_VERSION, "oracle-v1", "refined-v1")
    if sc.get("solver") not in valid_solvers:
        errors.append(f"source_calibration.solver: 期望 {'/'.join(valid_solvers)}, 实际 {sc.get('solver')}")
    if sc.get("version") not in valid_versions:
        errors.append(f"source_calibration.version: 期望 {'/'.join(valid_versions)}, 实际 {sc.get('version')}")

    # transform_contract
    tc = data.get("transform_contract", {})
    if tc.get("primary") != "T_rig_camera":
        errors.append(f"transform_contract.primary: 期望 T_rig_camera, 实际 {tc.get('primary')}")
    expected_eq = "p_rig = T_rig_camera @ p_camera"
    if tc.get("equation") != expected_eq:
        errors.append(f"transform_contract.equation: 期望 '{expected_eq}', 实际 '{tc.get('equation')}'")

    # cameras
    cameras = data.get("cameras", {})
    for cam_name in REQUIRED_CAMERAS:
        if cam_name not in cameras:
            errors.append(f"缺失相机: {cam_name}")
            continue

        cam_data = cameras[cam_name]
        if not cam_data.get("optical_frame"):
            errors.append(f"{cam_name}: optical_frame 缺失")

        T_rc = cam_data.get("T_rig_camera")
        if T_rc is None:
            errors.append(f"{cam_name}: T_rig_camera 缺失")
        else:
            T_rc_np = np.array(T_rc) if not isinstance(T_rc, np.ndarray) else T_rc
            ok, errs = validate_se3_matrix(T_rc_np, f"{cam_name}.T_rig_camera")
            errors.extend(errs)

        T_cr = cam_data.get("T_camera_rig")
        if T_cr is None:
            errors.append(f"{cam_name}: T_camera_rig 缺失")
        else:
            T_cr_np = np.array(T_cr) if not isinstance(T_cr, np.ndarray) else T_cr
            ok, errs = validate_se3_matrix(T_cr_np, f"{cam_name}.T_camera_rig")
            errors.extend(errs)

        # 互逆校验
        if T_rc is not None and T_cr is not None:
            T_rc_np = np.array(T_rc) if not isinstance(T_rc, np.ndarray) else T_rc
            T_cr_np = np.array(T_cr) if not isinstance(T_cr, np.ndarray) else T_cr
            ok, errs = validate_inverse_pair(T_rc_np, T_cr_np, f"{cam_name}")
            errors.extend(errs)

    # FL 必须为 identity
    fl_data = cameras.get("cam_front_left", {})
    T_fl = fl_data.get("T_rig_camera")
    if T_fl is not None:
        T_fl_np = np.array(T_fl) if not isinstance(T_fl, np.ndarray) else T_fl
        ok, errs = validate_identity(T_fl_np, "cam_front_left.T_rig_camera")
        errors.extend(errs)

    return len(errors) == 0, errors


def validate_stable_v1_json(data: dict) -> Tuple[bool, List[str]]:
    """校验 Stable V1 标定输出 JSON schema.

    Args:
        data: 已解析的 JSON dict.

    Returns:
        (passed, errors)
    """
    errors = []

    if data.get("solver") != REQUIRED_SOLVER:
        errors.append(f"solver: 期望 {REQUIRED_SOLVER}, 实际 {data.get('solver')}")
    if data.get("version") != REQUIRED_VERSION:
        errors.append(f"version: 期望 {REQUIRED_VERSION}, 实际 {data.get('version')}")
    if data.get("status") != REQUIRED_STATUS:
        errors.append(f"status: 期望 {REQUIRED_STATUS}, 实际 {data.get('status')}")

    cameras = data.get("cameras", {})
    for cam_name in REQUIRED_CAMERAS:
        if cam_name not in cameras:
            errors.append(f"缺失相机: {cam_name}")
            continue
        T_list = cameras[cam_name]
        if not isinstance(T_list, list) or len(T_list) != 4:
            errors.append(f"{cam_name}: 矩阵格式错误")
            continue
        T = np.array(T_list)
        ok, errs = validate_se3_matrix(T, f"{cam_name}")
        errors.extend(errs)

    # FL identity
    if "cam_front_left" in cameras:
        T_fl = np.array(cameras["cam_front_left"])
        ok, errs = validate_identity(T_fl, "cam_front_left")
        errors.extend(errs)

    return len(errors) == 0, errors

"""
CR5 Reconstruction — Stable V1 外参桥接.

Stable V1 camera_extrinsics.json → calibrated_rig.yaml

本模块只能做格式转换和元数据补充，禁止:
  - 优化、平均、ICP 或修改外参数值
  - 导入 gazebo_msgs / tf2_ros / model_states

用法:
  from cr5_spray_perception.reconstruction.extrinsics import load_calibrated_rig

  rig = load_calibrated_rig("path/to/camera_extrinsics.json")
  # rig["cameras"]["cam_front_right"]["T_rig_camera"]  → 4×4 np.ndarray
  # rig["cameras"]["cam_front_right"]["T_camera_rig"]   → 4×4 np.ndarray (逆矩阵)
"""

import os, sys, json, hashlib, logging
from typing import Dict, Optional, Tuple
from pathlib import Path

import numpy as np

from cr5_spray_perception.reconstruction.contracts import (
    validate_stable_v1_json, validate_calibrated_rig_yaml,
    REQUIRED_CAMERAS, SCHEMA_VERSION, validate_se3_matrix, validate_identity,
)
from cr5_spray_perception.reconstruction.transforms import invert_transform

logger = logging.getLogger(__name__)

# Stable V1 约定的 optical frame 命名规则
# Gazebo 第一阶段: color/depth 共光心, optical frame 使用 color_optical_frame
OPTICAL_FRAME_SUFFIX = "color_optical_frame"
RIG_FRAME = f"cam_front_left_{OPTICAL_FRAME_SUFFIX}"

# camera_name → optical_frame 映射
CAMERA_OPTICAL_FRAMES = {
    "cam_front_left":  f"cam_front_left_{OPTICAL_FRAME_SUFFIX}",
    "cam_front_right": f"cam_front_right_{OPTICAL_FRAME_SUFFIX}",
    "cam_rear":        f"cam_rear_{OPTICAL_FRAME_SUFFIX}",
}

TRANSFORM_CONTRACT = {
    "primary": "T_rig_camera",
    "equation": "p_rig = T_rig_camera @ p_camera",
    "inverse": "T_camera_rig = inverse(T_rig_camera)",
}


def sha256_file(filepath: str) -> str:
    """计算文件的 SHA256."""
    if not os.path.isfile(filepath):
        return ""
    sha = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            sha.update(chunk)
    return sha.hexdigest()


def load_stable_v1_extrinsics(json_path: str) -> Dict[str, np.ndarray]:
    """从 Stable V1 camera_extrinsics.json 加载原始外参矩阵.

    Args:
        json_path: JSON 文件路径.

    Returns:
        {cam_name: 4×4 np.ndarray} 字典, 矩阵语义为 T_rig_camera.

    Raises:
        ValueError: schema 校验失败.
        FileNotFoundError: 文件不存在.
    """
    if not os.path.isfile(json_path):
        raise FileNotFoundError(f"Stable V1 外参文件不存在: {json_path}")

    with open(json_path, "r") as f:
        data = json.load(f)

    passed, errors = validate_stable_v1_json(data)
    if not passed:
        raise ValueError(f"Stable V1 JSON schema 校验失败:\n  " + "\n  ".join(errors))

    cameras = {}
    for cam_name in REQUIRED_CAMERAS:
        T_list = data["cameras"][cam_name]
        cameras[cam_name] = np.array(T_list, dtype=np.float64)

    return cameras


def build_calibrated_rig(source_path: str) -> dict:
    """从 Stable V1 JSON 构建 calibrated_rig YAML 数据结构.

    只转换格式和补充元数据，不修改矩阵数值.

    Args:
        source_path: Stable V1 camera_extrinsics.json 路径.

    Returns:
        dict: 完整的 calibrated_rig 数据结构 (适合序列化为 YAML).

    Raises:
        ValueError: schema 或矩阵校验失败.
    """
    # 加载并校验原始外参
    T_rig_cameras = load_stable_v1_extrinsics(source_path)

    # 构建 source_calibration 元数据
    source_calibration = {
        "file": os.path.abspath(source_path),
        "sha256": sha256_file(source_path),
        "solver": "pairwise_camera_relative",
        "version": "stable-v1",
        "n_cameras": len(T_rig_cameras),
    }

    # 构建 cameras 数据
    cameras_output = {}
    for cam_name in REQUIRED_CAMERAS:
        T_rc = T_rig_cameras[cam_name]
        T_cr = invert_transform(T_rc)

        # 校验矩阵
        ok, errs = validate_se3_matrix(T_rc, f"{cam_name}.T_rig_camera")
        if not ok:
            raise ValueError(f"T_rig_camera 校验失败: {'; '.join(errs)}")
        ok, errs = validate_se3_matrix(T_cr, f"{cam_name}.T_camera_rig")
        if not ok:
            raise ValueError(f"T_camera_rig 校验失败: {'; '.join(errs)}")

        # 互逆校验
        I4 = np.eye(4)
        err = np.max(np.abs(T_rc @ T_cr - I4))
        if err > 1e-6:
            raise ValueError(f"{cam_name}: T_rig_camera @ T_camera_rig 偏离 I, max|diff|={err:.2e}")

        cameras_output[cam_name] = {
            "optical_frame": CAMERA_OPTICAL_FRAMES[cam_name],
            "T_rig_camera": T_rc.tolist(),
            "T_camera_rig": T_cr.tolist(),
        }

    # 确认 FL 为 identity
    fl_ok, fl_errs = validate_identity(T_rig_cameras["cam_front_left"], "cam_front_left")
    if not fl_ok:
        raise ValueError(f"FL 非 identity: {'; '.join(fl_errs)}")

    result = {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS",
        "units": "meter",
        "rig_frame": RIG_FRAME,
        "source_calibration": source_calibration,
        "transform_contract": TRANSFORM_CONTRACT,
        "cameras": cameras_output,
    }

    # 最终整体校验
    passed, errors = validate_calibrated_rig_yaml(result)
    if not passed:
        raise ValueError(f"calibrated_rig YAML 整体校验失败:\n  " + "\n  ".join(errors))

    return result


def load_calibrated_rig(yaml_or_json_path: str) -> dict:
    """加载 calibrated_rig 配置.

    如果输入是 Stable V1 JSON, 自动转换; 如果是已有 calibrated_rig YAML, 直接加载校验.

    Args:
        yaml_or_json_path: 文件路径 (.json 或 .yaml/.yml).

    Returns:
        dict: calibrated_rig 数据结构, 其中 T_rig_camera / T_camera_rig 为 np.ndarray.
    """
    import yaml

    filepath = os.path.abspath(yaml_or_json_path)
    ext = os.path.splitext(filepath)[1].lower()

    if ext == ".json":
        # Stable V1 → calibrated_rig
        data = build_calibrated_rig(filepath)
    elif ext in (".yaml", ".yml"):
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        passed, errors = validate_calibrated_rig_yaml(data)
        if not passed:
            raise ValueError(f"calibrated_rig YAML 校验失败:\n  " + "\n  ".join(errors))
    else:
        raise ValueError(f"不支持的文件格式: {ext}, 期望 .json / .yaml / .yml")

    # 将 list 转为 np.ndarray
    for cam_name in data.get("cameras", {}):
        cam = data["cameras"][cam_name]
        if isinstance(cam.get("T_rig_camera"), list):
            cam["T_rig_camera"] = np.array(cam["T_rig_camera"], dtype=np.float64)
        if isinstance(cam.get("T_camera_rig"), list):
            cam["T_camera_rig"] = np.array(cam["T_camera_rig"], dtype=np.float64)

    return data


def get_T_rig_camera(rig: dict, cam_name: str) -> np.ndarray:
    """从 calibrated_rig 获取 T_rig_camera 矩阵.

    Args:
        rig: calibrated_rig 数据.
        cam_name: 相机名.

    Returns:
        4×4 np.ndarray.
    """
    return rig["cameras"][cam_name]["T_rig_camera"]


def get_T_camera_rig(rig: dict, cam_name: str) -> np.ndarray:
    """从 calibrated_rig 获取 T_camera_rig 矩阵 (= inverse(T_rig_camera)).

    Args:
        rig: calibrated_rig 数据.
        cam_name: 相机名.

    Returns:
        4×4 np.ndarray.
    """
    return rig["cameras"][cam_name]["T_camera_rig"]

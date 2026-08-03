"""
CR5 Reconstruction — RGB-D 配准契约.

定义 depth-to-color 配准的三种状态并提供验证逻辑:

  REGISTERED:   深度已对齐到彩色 (同 frame_id, 同尺寸, 同 K)
  COLOCATED:    color/depth frame 名不同, 但固定变换接近 identity
                (Gazebo 当前情况: 两个 optical frame 重合)
  UNREGISTERED: 未配准, 不能直接索引 RGB 颜色

Gazebo 当前情况识别:
  - color/depth frame 名称不同 (xxx_color_optical_frame vs xxx_depth_optical_frame)
  - 固定变换接近 identity (translation ≤0.1mm, rotation ≤0.01°)
  - 图像尺寸相同
  - K 一致

生产链路: 不直接查询 Gazebo TF。
证据来源可以是:
  - 采集阶段写入的静态几何契约
  - dataset manifest
  - camera model 文档

用法:
  from cr5_spray_perception.reconstruction.registration import (
      DepthRegistration, determine_registration_status,
      COLOR_DEPTH_COLOCATED_THRESHOLDS,
  )
"""

import logging
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

logger = logging.getLogger(__name__)

# ── 门限 ──
COLOR_DEPTH_COLOCATED_THRESHOLDS = {
    "max_translation_mm": 0.1,          # color-depth 平移 ≤ 0.1 mm
    "max_rotation_deg": 0.01,           # color-depth 旋转 ≤ 0.01°
    "max_K_abs_diff": 1e-6,             # K 矩阵最大绝对差
    "require_same_size": True,          # 必须同尺寸
}


@dataclass
class DepthRegistration:
    """RGB-D 配准状态证据."""

    status: str = "UNKNOWN"          # REGISTERED | COLOCATED | UNREGISTERED
    color_frame: str = ""
    depth_frame: str = ""

    # 几何证据 (如可用)
    T_color_depth: Optional[np.ndarray] = None   # 4×4, depth→color 变换
    translation_mm: float = 0.0
    rotation_deg: float = 0.0

    # 内参证据
    color_K: Optional[np.ndarray] = None
    depth_K: Optional[np.ndarray] = None
    K_max_abs_diff: float = 0.0
    same_size: bool = False

    # 证据来源
    evidence_source: str = ""         # gazebo_static_tf | camera_model_contract | dataset_manifest
    verified: bool = False

    # 诊断
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def is_registered(self) -> bool:
        return self.status in ("REGISTERED", "COLOCATED")

    @property
    def rgb_indexing_safe(self) -> bool:
        """RGB 颜色可直接按像素索引? (仅 REGISTERED 状态)"""
        return self.status == "REGISTERED"


def _compute_T_from_frame_names(color_frame: str, depth_frame: str) -> Optional[np.ndarray]:
    """根据 frame 名称推断 T_color_depth.

    Gazebo 中 color_optical_frame 和 depth_optical_frame 通过固定 joint 连接,
    两者位姿相同, 变换为 identity.

    生产模块不查询 TF, 仅用已知的静态契约.
    """
    # 规则: 如果 color 和 depth frame 是同一台相机 (同前缀), 且后缀符合 convention
    # cam_xxx_color_optical_frame / cam_xxx_depth_optical_frame → 假设 identity
    color_base = color_frame.replace("_color_optical_frame", "")
    depth_base = depth_frame.replace("_depth_optical_frame", "")

    if color_base == depth_base and color_base != color_frame:
        # 同一台相机的 color/depth optical frame → identity
        return np.eye(4)

    # 其他情况: 无法推断
    return None


def determine_registration_status(
    color_frame: str,
    depth_frame: str,
    color_width: int,
    color_height: int,
    depth_width: int,
    depth_height: int,
    color_K: Optional[np.ndarray] = None,
    depth_K: Optional[np.ndarray] = None,
    T_color_depth: Optional[np.ndarray] = None,
    evidence_source: str = "heuristic",
    thresholds: Optional[dict] = None,
) -> DepthRegistration:
    """确定 RGB-D 配准状态.

    Args:
        color_frame: color CameraInfo frame_id.
        depth_frame: depth CameraInfo frame_id.
        color_width, color_height: 彩色图像尺寸.
        depth_width, depth_height: 深度图像尺寸.
        color_K: 彩色相机内参 3×3.
        depth_K: 深度相机内参 3×3.
        T_color_depth: 已知的 color←depth 变换 (如有).
        evidence_source: 证据来源标识.
        thresholds: 门限 dict (默认使用 COLOR_DEPTH_COLOCATED_THRESHOLDS).

    Returns:
        DepthRegistration 实例.
    """
    if thresholds is None:
        thresholds = COLOR_DEPTH_COLOCATED_THRESHOLDS

    reg = DepthRegistration(
        color_frame=color_frame,
        depth_frame=depth_frame,
        color_K=color_K,
        depth_K=depth_K,
        evidence_source=evidence_source,
    )

    # 检查尺寸
    reg.same_size = (depth_width == color_width and depth_height == color_height)

    # 情况 1: frame_id 相同
    if color_frame == depth_frame:
        if reg.same_size:
            reg.status = "REGISTERED"
            reg.verified = True
        else:
            reg.status = "UNREGISTERED"
            reg.errors.append(
                f"frame_id 相同但尺寸不同: color={color_width}x{color_height}, "
                f"depth={depth_width}x{depth_height}")
        return reg

    # 情况 2: frame_id 不同 → 需要几何证据
    # 尝试推断 T_color_depth
    if T_color_depth is None:
        T_color_depth = _compute_T_from_frame_names(color_frame, depth_frame)

    if T_color_depth is not None:
        # 从变换矩阵提取平移和旋转
        t = T_color_depth[:3, 3]
        translation_mm = float(np.linalg.norm(t) * 1000.0)

        # 旋转角度 (从旋转矩阵)
        R = T_color_depth[:3, :3]
        trace = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
        rotation_deg = float(np.arccos(trace) * 180.0 / np.pi)

        reg.T_color_depth = T_color_depth
        reg.translation_mm = translation_mm
        reg.rotation_deg = rotation_deg

        max_t_mm = thresholds["max_translation_mm"]
        max_r_deg = thresholds["max_rotation_deg"]

        t_ok = translation_mm <= max_t_mm
        r_ok = rotation_deg <= max_r_deg

        if t_ok and r_ok and reg.same_size:
            reg.status = "COLOCATED"
            reg.verified = True
            reg.warnings.append(
                f"color/depth frame 名不同但几何重合: "
                f"translation={translation_mm:.4f}mm (≤{max_t_mm}mm), "
                f"rotation={rotation_deg:.4f}° (≤{max_r_deg}°)")
        else:
            reasons = []
            if not t_ok:
                reasons.append(f"translation={translation_mm:.4f}mm > {max_t_mm}mm")
            if not r_ok:
                reasons.append(f"rotation={rotation_deg:.4f}° > {max_r_deg}°")
            if not reg.same_size:
                reasons.append(f"尺寸不同: color={color_width}x{color_height}, depth={depth_width}x{depth_height}")
            reg.status = "UNREGISTERED"
            reg.errors.append(f"color/depth 未配准: {'; '.join(reasons)}")
    else:
        # 无 T_color_depth 证据
        if reg.same_size:
            # frame 名不同、尺寸相同但无几何证据 → 不能默认为 COLOCATED
            reg.status = "UNREGISTERED"
            reg.errors.append(
                f"frame_id 不同且无 T_color_depth 证据: "
                f"color={color_frame}, depth={depth_frame}. "
                f"尺寸相同但不能推断共光心.")
        else:
            reg.status = "UNREGISTERED"
            reg.errors.append(
                f"frame_id 不同且尺寸不同: "
                f"color={color_width}x{color_height} ({color_frame}), "
                f"depth={depth_width}x{depth_height} ({depth_frame})")

    # K 矩阵比较
    if color_K is not None and depth_K is not None:
        reg.K_max_abs_diff = float(np.max(np.abs(color_K - depth_K)))
        max_k = thresholds["max_K_abs_diff"]
        if reg.K_max_abs_diff > max_k:
            if reg.status in ("REGISTERED", "COLOCATED"):
                reg.warnings.append(
                    f"K 矩阵差异较大: max|diff|={reg.K_max_abs_diff:.2e} > {max_k}")
        else:
            if reg.status == "COLOCATED":
                reg.warnings.append(
                    f"K 矩阵一致 (max|diff|={reg.K_max_abs_diff:.2e}), "
                    f"支持 COLOCATED 判定")

    return reg

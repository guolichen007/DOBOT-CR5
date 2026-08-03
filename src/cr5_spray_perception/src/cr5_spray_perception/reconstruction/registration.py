"""
CR5 Reconstruction — RGB-D 配准契约.

定义 depth-to-color 配准状态:
  REGISTERED:   深度已对齐到彩色 (同 frame_id, 同尺寸, 同 K)
  COLOCATED:    有显式 T_color_depth 证据, 几何重合但 frame 名不同
  UNREGISTERED: 未配准

pixel_correspondence_safe: 是否可以安全地按像素索引 RGB 颜色.
  需要: verified=true, T_color_depth 平移/旋转/K 差异全部在门限内

生产链路: 禁止查询 Gazebo TF. 证据来源必须是已保存的文件.
"""

import os, json, logging
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

logger = logging.getLogger(__name__)

# ── 门限 ──
COLOR_DEPTH_COLOCATED_THRESHOLDS = {
    "max_translation_mm": 0.1,
    "max_rotation_deg": 0.01,
    "max_K_abs_diff": 1e-6,
    "require_same_size": True,
}

# 像素对应安全门限 (比 COLOCATED 更严格)
PIXEL_CORRESPONDENCE_THRESHOLDS = {
    "max_translation_mm": 1.0,      # ≤1mm (放松: 1mm 位移影响 <2 pixels @ 640px FOV)
    "max_rotation_deg": 0.05,       # ≤0.05°
    "max_K_abs_diff": 1e-3,         # K 差异 ≤0.001
    "require_same_size": True,
}


@dataclass
class DepthRegistration:
    """RGB-D 配准状态证据."""

    status: str = "UNKNOWN"          # REGISTERED | COLOCATED | UNREGISTERED
    color_frame: str = ""
    depth_frame: str = ""

    # 几何证据
    T_color_depth: Optional[np.ndarray] = None   # 4×4, p_color = T_color_depth @ p_depth
    translation_mm: float = 0.0
    rotation_deg: float = 0.0

    # 内参证据
    color_K: Optional[np.ndarray] = None
    depth_K: Optional[np.ndarray] = None
    K_max_abs_diff: float = 0.0
    same_size: bool = False

    # 像素对应安全性
    pixel_correspondence_safe: bool = False

    # 证据来源
    evidence_source: str = ""
    evidence_file: str = ""
    verified: bool = False

    # 诊断
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def is_registered(self) -> bool:
        return self.status in ("REGISTERED", "COLOCATED")


def _compute_rotation_deg(R: np.ndarray) -> float:
    """从旋转矩阵提取旋转角度 (度)."""
    trace = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(trace) * 180.0 / np.pi)


def load_registration_evidence(evidence_file: str) -> Optional[dict]:
    """加载 depth_registration.json 证据文件.

    Returns:
        dict 或 None (文件不存在或格式错误).
    """
    if not os.path.isfile(evidence_file):
        return None
    try:
        with open(evidence_file, "r") as f:
            data = json.load(f)
        if data.get("schema_version") == "cr5_rgbd_registration_v1":
            return data
        logger.warning("registration evidence schema 不兼容: %s", data.get("schema_version"))
    except (json.JSONDecodeError, IOError) as e:
        logger.warning("无法解析 registration evidence: %s", e)
    return None


def determine_registration_from_evidence(
    evidence: dict,
    camera_name: str,
    color_width: int,
    color_height: int,
    depth_width: int,
    depth_height: int,
    color_K: Optional[np.ndarray] = None,
    depth_K: Optional[np.ndarray] = None,
    thresholds: Optional[dict] = None,
    pixel_thresholds: Optional[dict] = None,
) -> DepthRegistration:
    """从持久化 evidence 文件判定配准状态.

    Args:
        evidence: depth_registration.json 的 cameras 部分或完整文档.
        camera_name: 相机名.
        color_width/height: 彩色图像实际尺寸.
        depth_width/height: 深度图像实际尺寸.
        color_K/depth_K: 从 CameraInfo 读取的内参.
        thresholds: COLOCATED 门限.
        pixel_thresholds: pixel_correspondence_safe 门限.

    Returns:
        DepthRegistration 实例.
    """
    if thresholds is None:
        thresholds = COLOR_DEPTH_COLOCATED_THRESHOLDS
    if pixel_thresholds is None:
        pixel_thresholds = PIXEL_CORRESPONDENCE_THRESHOLDS

    # 提取相机 evidence
    cameras = evidence.get("cameras", evidence)
    cam_ev = cameras.get(camera_name, {})

    reg = DepthRegistration(
        color_frame=cam_ev.get("color_frame", ""),
        depth_frame=cam_ev.get("depth_frame", ""),
        color_K=color_K,
        depth_K=depth_K,
        evidence_source=evidence.get("source", "unknown"),
        evidence_file=evidence.get("_evidence_file", ""),
    )

    # 尺寸
    reg.same_size = (depth_width == color_width and depth_height == color_height)

    # 从 evidence 加载 T_color_depth
    T_list = cam_ev.get("T_color_depth")
    if T_list is not None and len(T_list) == 4:
        reg.T_color_depth = np.array(T_list, dtype=np.float64)
        t = reg.T_color_depth[:3, 3]
        reg.translation_mm = float(np.linalg.norm(t) * 1000.0)
        reg.rotation_deg = _compute_rotation_deg(reg.T_color_depth[:3, :3])
    else:
        reg.errors.append(f"{camera_name}: T_color_depth 缺失")
        reg.status = "UNREGISTERED"
        return reg

    # 检查 evidence 中的 declared status
    declared_status = cam_ev.get("status", "UNKNOWN")

    # K 比较
    if color_K is not None and depth_K is not None:
        reg.K_max_abs_diff = float(np.max(np.abs(color_K - depth_K)))

    # ── 判定 status ──
    max_t_mm = thresholds["max_translation_mm"]
    max_r_deg = thresholds["max_rotation_deg"]
    max_k = thresholds["max_K_abs_diff"]

    t_ok = reg.translation_mm <= max_t_mm
    r_ok = reg.rotation_deg <= max_r_deg
    k_ok = reg.K_max_abs_diff <= max_k

    if declared_status == "COLOCATED" and t_ok and r_ok and reg.same_size:
        reg.status = "COLOCATED"
        reg.verified = True
        if not k_ok:
            # K 超限 → 降低 verified
            reg.verified = False
            reg.errors.append(
                f"K 差异超限: max|diff|={reg.K_max_abs_diff:.2e} > {max_k}. "
                f"COLOCATED 声明不可验证.")
            reg.status = "UNREGISTERED"
    elif t_ok and r_ok and reg.same_size:
        # evidence 未声明 COLOCATED 但几何满足 → COLOCATED
        reg.status = "COLOCATED"
        reg.verified = True
    else:
        reg.status = "UNREGISTERED"
        reasons = []
        if not t_ok:
            reasons.append(f"translation={reg.translation_mm:.4f}mm > {max_t_mm}mm")
        if not r_ok:
            reasons.append(f"rotation={reg.rotation_deg:.4f}° > {max_r_deg}°")
        if not reg.same_size:
            reasons.append(f"尺寸不同")
        reg.errors.append(f"配准失败: {'; '.join(reasons)}")

    # ── 判定 pixel_correspondence_safe ──
    pt = pixel_thresholds
    pixel_t_ok = reg.translation_mm <= pt["max_translation_mm"]
    pixel_r_ok = reg.rotation_deg <= pt["max_rotation_deg"]
    pixel_k_ok = reg.K_max_abs_diff <= pt["max_K_abs_diff"]
    pixel_size_ok = reg.same_size

    reg.pixel_correspondence_safe = (
        reg.verified
        and reg.status in ("REGISTERED", "COLOCATED")
        and pixel_t_ok
        and pixel_r_ok
        and pixel_k_ok
        and pixel_size_ok
    )

    if reg.is_registered and not reg.pixel_correspondence_safe:
        reg.warnings.append(
            f"配准通过但像素对应不安全: "
            f"t={reg.translation_mm:.4f}mm (≤{pt['max_translation_mm']}mm? {pixel_t_ok}), "
            f"r={reg.rotation_deg:.4f}° (≤{pt['max_rotation_deg']}°? {pixel_r_ok}), "
            f"Kdiff={reg.K_max_abs_diff:.2e} (≤{pt['max_K_abs_diff']}? {pixel_k_ok})")

    return reg


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
    evidence_source: str = "camera_info_yaml",
    thresholds: Optional[dict] = None,
) -> DepthRegistration:
    """确定 RGB-D 配准状态 (无外部 evidence 文件时的基础判定).

    注意: 如果没有显式 T_color_depth 且 frame 名不同, 无法判定 COLOCATED.
    生产环境应使用 determine_registration_from_evidence().

    Args:
        color_frame, depth_frame: frame_id.
        color_width/height, depth_width/height: 图像尺寸.
        color_K, depth_K: 内参矩阵.
        T_color_depth: 已知变换 (如从 evidence 文件加载).
        evidence_source: 证据来源标识.
        thresholds: 门限.

    Returns:
        DepthRegistration 实例.
    """
    if thresholds is None:
        thresholds = COLOR_DEPTH_COLOCATED_THRESHOLDS

    reg = DepthRegistration(
        color_frame=color_frame, depth_frame=depth_frame,
        color_K=color_K, depth_K=depth_K,
        evidence_source=evidence_source,
    )

    reg.same_size = (depth_width == color_width and depth_height == color_height)

    # 情况 1: frame_id 相同
    if color_frame == depth_frame:
        if reg.same_size:
            reg.status = "REGISTERED"
            reg.verified = True
            reg.pixel_correspondence_safe = True
        else:
            reg.status = "UNREGISTERED"
            reg.errors.append(
                f"frame_id 相同但尺寸不同: color={color_width}x{color_height}, "
                f"depth={depth_width}x{depth_height}")
        return reg

    # 情况 2: frame_id 不同 → 必须有显式 T_color_depth
    if T_color_depth is not None:
        t = T_color_depth[:3, 3]
        reg.translation_mm = float(np.linalg.norm(t) * 1000.0)
        reg.rotation_deg = _compute_rotation_deg(T_color_depth[:3, :3])
        reg.T_color_depth = T_color_depth

        max_t_mm = thresholds["max_translation_mm"]
        max_r_deg = thresholds["max_rotation_deg"]

        t_ok = reg.translation_mm <= max_t_mm
        r_ok = reg.rotation_deg <= max_r_deg

        if t_ok and r_ok and reg.same_size:
            reg.status = "COLOCATED"
            reg.verified = True
        else:
            reasons = []
            if not t_ok:
                reasons.append(f"translation={reg.translation_mm:.4f}mm > {max_t_mm}mm")
            if not r_ok:
                reasons.append(f"rotation={reg.rotation_deg:.4f}° > {max_r_deg}°")
            if not reg.same_size:
                reasons.append(f"尺寸不同")
            reg.status = "UNREGISTERED"
            reg.errors.append(f"color/depth 未配准: {'; '.join(reasons)}")

        # pixel_correspondence_safe
        pt = PIXEL_CORRESPONDENCE_THRESHOLDS
        reg.pixel_correspondence_safe = (
            reg.verified
            and reg.status in ("REGISTERED", "COLOCATED")
            and reg.translation_mm <= pt["max_translation_mm"]
            and reg.rotation_deg <= pt["max_rotation_deg"]
            and reg.same_size
        )
    else:
        # 无 T_color_depth 且 frame 名不同
        reg.status = "UNREGISTERED"
        reg.errors.append(
            f"frame_id 不同且无 T_color_depth 证据: "
            f"color={color_frame}, depth={depth_frame}")

    # K 比较
    if color_K is not None and depth_K is not None:
        reg.K_max_abs_diff = float(np.max(np.abs(color_K - depth_K)))
        if reg.K_max_abs_diff > thresholds["max_K_abs_diff"]:
            if reg.verified:
                reg.warnings.append(
                    f"K 差异较大: max|diff|={reg.K_max_abs_diff:.2e} > "
                    f"{thresholds['max_K_abs_diff']}")

    return reg

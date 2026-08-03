"""
CR5 Reconstruction — RGB-D 数据加载与契约验证.

变更 (v2):
  - 优先从 group/depth_registration.json 加载 evidence
  - 无 evidence 时严格失败 (不再猜 frame 名)
  - 同步验证: captured==expected==3, 双重 skew 检查, 重算 skew
  - pixel_correspondence_safe 集成

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import os, yaml, json, logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

from cr5_spray_perception.reconstruction.registration import (
    DepthRegistration, determine_registration_status,
    determine_registration_from_evidence,
    load_registration_evidence, COLOR_DEPTH_COLOCATED_THRESHOLDS,
)

logger = logging.getLogger(__name__)


@dataclass
class RGBDData:
    """单帧 RGB-D 数据."""
    camera_name: str
    color_path: str = ""
    depth_path: str = ""
    color_cinfo_path: str = ""
    depth_cinfo_path: str = ""
    quality_path: str = ""

    color: Optional[np.ndarray] = None
    depth_raw: Optional[np.ndarray] = None
    depth_meters: Optional[np.ndarray] = None
    depth_unit: str = ""
    color_K: Optional[np.ndarray] = None
    depth_K: Optional[np.ndarray] = None

    color_width: int = 0
    color_height: int = 0
    depth_width: int = 0
    depth_height: int = 0
    color_cinfo_width: int = 0
    color_cinfo_height: int = 0
    depth_cinfo_width: int = 0
    depth_cinfo_height: int = 0

    color_frame_id: str = ""
    depth_frame_id: str = ""

    registration: Optional[DepthRegistration] = None
    quality: dict = field(default_factory=dict)

    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    @property
    def depth_registered_to_color(self) -> bool:
        if self.registration is not None:
            return self.registration.is_registered
        return (self.depth_width == self.color_width and
                self.depth_height == self.color_height and
                self.depth_frame_id == self.color_frame_id)

    @property
    def rgb_indexing_safe(self) -> bool:
        if self.registration is not None:
            return self.registration.pixel_correspondence_safe
        return self.depth_frame_id == self.color_frame_id


def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def parse_camera_info(cinfo: dict) -> dict:
    result = {"K": None, "width": 0, "height": 0, "frame_id": ""}
    if "K" in cinfo and isinstance(cinfo["K"], list) and len(cinfo["K"]) == 9:
        result["K"] = np.array(cinfo["K"]).reshape(3, 3).astype(np.float64)
    elif "camera_matrix" in cinfo:
        cm = cinfo["camera_matrix"]
        if "data" in cm:
            result["K"] = np.array(cm["data"]).reshape(3, 3).astype(np.float64)
    result["width"] = cinfo.get("width", cinfo.get("image_width", 0))
    result["height"] = cinfo.get("height", cinfo.get("image_height", 0))
    header = cinfo.get("header", {})
    result["frame_id"] = header.get("frame_id", cinfo.get("frame_id", ""))
    return result


def _validate_K(K: np.ndarray, label: str) -> List[str]:
    errors = []
    if K is None:
        errors.append(f"{label}: K 为 None"); return errors
    if K.shape != (3, 3):
        errors.append(f"{label}: K shape={K.shape}"); return errors
    if not np.all(np.isfinite(K)):
        errors.append(f"{label}: K 含 NaN/Inf")
    if K[0, 0] <= 0: errors.append(f"{label}: fx≤0")
    if K[1, 1] <= 0: errors.append(f"{label}: fy≤0")
    if abs(K[0, 1]) > 1e-6: errors.append(f"{label}: K[0,1]≠0")
    if abs(K[1, 0]) > 1e-6: errors.append(f"{label}: K[1,0]≠0")
    if abs(K[2, 2] - 1.0) > 1e-10: errors.append(f"{label}: K[2,2]≠1")
    return errors


def load_rgbd_data(camera_dir: str, camera_name: str) -> RGBDData:
    data = RGBDData(camera_name=camera_name)
    color_path = os.path.join(camera_dir, "color.png")
    depth_path = os.path.join(camera_dir, "depth.npy")
    color_cinfo_path = os.path.join(camera_dir, "color_camera_info.yaml")
    depth_cinfo_path = os.path.join(camera_dir, "depth_camera_info.yaml")
    quality_path = os.path.join(camera_dir, "quality.yaml")

    for label, path in [("color.png", color_path), ("depth.npy", depth_path),
                         ("color_camera_info.yaml", color_cinfo_path),
                         ("depth_camera_info.yaml", depth_cinfo_path)]:
        if not os.path.isfile(path):
            data.errors.append(f"{camera_name}: 缺失 {label}")

    data.color_path = color_path; data.depth_path = depth_path
    data.color_cinfo_path = color_cinfo_path; data.depth_cinfo_path = depth_cinfo_path
    data.quality_path = quality_path
    if not data.is_valid:
        return data

    data.color = cv2.imread(color_path)
    if data.color is None:
        data.errors.append(f"{camera_name}: 无法读取 color.png")
    else:
        data.color_height, data.color_width = data.color.shape[:2]

    try:
        data.depth_raw = np.load(depth_path)
    except Exception as e:
        data.errors.append(f"{camera_name}: depth.npy: {e}"); return data
    if data.depth_raw is None or data.depth_raw.size == 0:
        data.errors.append(f"{camera_name}: depth.npy 为空")
    else:
        data.depth_height, data.depth_width = data.depth_raw.shape[:2]

    try:
        parsed = parse_camera_info(load_yaml(color_cinfo_path))
        data.color_K = parsed["K"]; data.color_frame_id = parsed["frame_id"]
        data.color_cinfo_width = parsed["width"]; data.color_cinfo_height = parsed["height"]
        if data.color_K is None:
            data.errors.append(f"{camera_name}: color K 缺失")
    except Exception as e:
        data.errors.append(f"{camera_name}: color_camera_info: {e}")

    try:
        parsed = parse_camera_info(load_yaml(depth_cinfo_path))
        data.depth_K = parsed["K"]; data.depth_frame_id = parsed["frame_id"]
        data.depth_cinfo_width = parsed["width"]; data.depth_cinfo_height = parsed["height"]
        if data.depth_K is None:
            data.errors.append(f"{camera_name}: depth K 缺失")
    except Exception as e:
        data.errors.append(f"{camera_name}: depth_camera_info: {e}")

    if os.path.isfile(quality_path):
        try:
            data.quality = load_yaml(quality_path)
        except Exception:
            data.warnings.append(f"{camera_name}: quality.yaml 解析失败")

    if data.depth_raw is not None:
        if data.depth_raw.dtype == np.uint16:
            data.depth_unit = "mm"
        elif data.depth_raw.dtype == np.float32:
            data.depth_unit = "meter"
        else:
            data.errors.append(f"{camera_name}: depth dtype {data.depth_raw.dtype} 不支持")
    return data


def validate_rgbd_contract(rgbd: RGBDData,
                            require_registered_to_color: bool = True,
                            registration_evidence: Optional[dict] = None,
                            ) -> RGBDData:
    """验证 RGBDData 契约.

    Args:
        rgbd: RGBDData 实例.
        require_registered_to_color: 是否要求配准.
        registration_evidence: 从 depth_registration.json 加载的 evidence dict.
    """
    if rgbd.color is None: rgbd.errors.append(f"{rgbd.camera_name}: color 未加载")
    if rgbd.depth_raw is None: rgbd.errors.append(f"{rgbd.camera_name}: depth 未加载")
    if rgbd.color_K is None: rgbd.errors.append(f"{rgbd.camera_name}: color K 缺失")
    if rgbd.depth_K is None: rgbd.errors.append(f"{rgbd.camera_name}: depth K 缺失")
    if not rgbd.is_valid: return rgbd

    for err in _validate_K(rgbd.color_K, f"{rgbd.camera_name}.color_K"):
        rgbd.errors.append(err)
    for err in _validate_K(rgbd.depth_K, f"{rgbd.camera_name}.depth_K"):
        rgbd.errors.append(err)

    if rgbd.color_width < 1 or rgbd.color_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: color 尺寸无效")
    else:
        if rgbd.color_width != rgbd.color_cinfo_width:
            rgbd.errors.append(f"{rgbd.camera_name}: color 宽度 {rgbd.color_width} ≠ cinfo {rgbd.color_cinfo_width}")
        if rgbd.color_height != rgbd.color_cinfo_height:
            rgbd.errors.append(f"{rgbd.camera_name}: color 高度 {rgbd.color_height} ≠ cinfo {rgbd.color_cinfo_height}")

    if rgbd.depth_width < 1 or rgbd.depth_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: depth 尺寸无效")
    else:
        if rgbd.depth_width != rgbd.depth_cinfo_width:
            rgbd.errors.append(f"{rgbd.camera_name}: depth 宽度 {rgbd.depth_width} ≠ cinfo {rgbd.depth_cinfo_width}")
        if rgbd.depth_height != rgbd.depth_cinfo_height:
            rgbd.errors.append(f"{rgbd.camera_name}: depth 高度 {rgbd.depth_height} ≠ cinfo {rgbd.depth_cinfo_height}")

    if not rgbd.color_frame_id: rgbd.errors.append(f"{rgbd.camera_name}: color frame_id 空")
    if not rgbd.depth_frame_id: rgbd.errors.append(f"{rgbd.camera_name}: depth frame_id 空")

    # ── 配准判定 ──
    if rgbd.color_frame_id and rgbd.depth_frame_id:
        if registration_evidence is not None:
            reg = determine_registration_from_evidence(
                registration_evidence, rgbd.camera_name,
                rgbd.color_width, rgbd.color_height,
                rgbd.depth_width, rgbd.depth_height,
                rgbd.color_K, rgbd.depth_K)
        else:
            # 无 evidence → 回退到基础判定 (frame 名相同才能 REGISTERED)
            reg = determine_registration_status(
                rgbd.color_frame_id, rgbd.depth_frame_id,
                rgbd.color_width, rgbd.color_height,
                rgbd.depth_width, rgbd.depth_height,
                rgbd.color_K, rgbd.depth_K)
            if rgbd.color_frame_id != rgbd.depth_frame_id:
                reg.errors.append(
                    f"无 depth_registration.json evidence, "
                    f"不能验证 color≠depth frame 的共光心: "
                    f"color={rgbd.color_frame_id}, depth={rgbd.depth_frame_id}")

        rgbd.registration = reg
        for e in reg.errors: rgbd.errors.append(f"{rgbd.camera_name}: {e}")
        for w in reg.warnings: rgbd.warnings.append(f"{rgbd.camera_name}: {w}")

        if require_registered_to_color and not reg.is_registered:
            rgbd.errors.append(f"{rgbd.camera_name}: 深度未配准 (status={reg.status})")

    return rgbd


def validate_sync_group(rgbd_list, expected_cameras, max_inter_camera_skew_ms=5.0,
                         calibrated_optical_frames=None, manifest=None,
                         allow_legacy_fallback=False):
    """验证三相机同步组 (严格模式).

    必须满足:
      - captured == expected == 3
      - len(camera_names) == 3
      - method == exact_stamp_ns
      - actual skew ≤ manifest.max_allowed_skew_s
      - actual skew ≤ config max
      - recomputed skew 与 declared skew 一致
      - camera 列表与 calibrated_rig 一致
    """
    errors = []; warnings = []
    sync_report = {"source": "unknown", "method": "unknown",
                   "declared_skew_ms": None, "recomputed_skew_ms": None,
                   "manifest_allowed_ms": None, "config_allowed_ms": max_inter_camera_skew_ms,
                   "camera_count": 0, "cameras": [], "passed": False}

    present = {r.camera_name for r in rgbd_list}
    for cam in expected_cameras:
        if cam not in present:
            errors.append(f"缺失相机: {cam}")
    if errors: return False, errors, warnings, sync_report

    for rgbd in rgbd_list:
        if not rgbd.is_valid:
            for e in rgbd.errors: errors.append(f"{rgbd.camera_name}: {e}")
        for w in rgbd.warnings: warnings.append(f"{rgbd.camera_name}: {w}")

    if calibrated_optical_frames:
        for rgbd in rgbd_list:
            expected_frame = calibrated_optical_frames.get(rgbd.camera_name)
            if expected_frame and rgbd.color_frame_id and rgbd.color_frame_id != expected_frame:
                errors.append(f"{rgbd.camera_name}: frame_id '{rgbd.color_frame_id}' ≠ '{expected_frame}'")

    # ── manifest 同步验证 ──
    if manifest is not None:
        sync_report["source"] = "group_manifest.json"

        if not manifest.get("success", False):
            errors.append("manifest.success=false"); sync_report["passed"] = False
            return False, errors, warnings, sync_report

        captured = manifest.get("captured", 0)
        expected = manifest.get("expected", 0)
        if captured != 3 or expected != 3:
            errors.append(f"captured={captured}, expected={expected}, 都需要=3")
            sync_report["passed"] = False
            return False, errors, warnings, sync_report

        manifest_cams = manifest.get("camera_names", [])
        if set(manifest_cams) != set(expected_cameras):
            errors.append(f"manifest cameras {sorted(manifest_cams)} ≠ {sorted(expected_cameras)}")

        ccs = manifest.get("cross_camera_sync", {})
        method = ccs.get("method", "unknown")
        sync_report["method"] = method
        if method != "exact_stamp_ns":
            errors.append(f"method='{method}', 期望 'exact_stamp_ns'")

        declared_skew_s = ccs.get("max_inter_camera_skew_s", None)
        manifest_allowed_s = ccs.get("max_allowed_skew_s", None)

        if declared_skew_s is not None:
            sync_report["declared_skew_ms"] = declared_skew_s * 1000.0
        if manifest_allowed_s is not None:
            sync_report["manifest_allowed_ms"] = manifest_allowed_s * 1000.0

        # 从 per_camera_color_stamps 重算 skew
        per_cam_stamps = ccs.get("per_camera_color_stamps", {})
        stamps_ns = []
        for cam in expected_cameras:
            if cam not in per_cam_stamps:
                errors.append(f"manifest 缺失 {cam} stamp")
            else:
                sn = per_cam_stamps[cam].get("stamp_ns")
                if sn is None or not isinstance(sn, (int, float)):
                    errors.append(f"manifest {cam} stamp_ns 无效: {sn}")
                else:
                    stamps_ns.append(int(sn))

        sync_report["camera_count"] = len(stamps_ns)
        sync_report["cameras"] = list(per_cam_stamps.keys())

        if len(stamps_ns) == 3:
            recomputed_skew_ns = max(stamps_ns) - min(stamps_ns)
            recomputed_skew_ms = recomputed_skew_ns / 1e6
            sync_report["recomputed_skew_ms"] = recomputed_skew_ms

            # 检查 1: recomputed ≤ declared (容差 1μs)
            if declared_skew_s is not None:
                declared_skew_ms = declared_skew_s * 1000.0
                if abs(recomputed_skew_ms - declared_skew_ms) > 0.001:
                    errors.append(
                        f"重算 skew ({recomputed_skew_ms:.3f}ms) ≠ "
                        f"声明 skew ({declared_skew_ms:.3f}ms)")

            # 检查 2: recomputed ≤ manifest allowed
            if manifest_allowed_s is not None:
                if recomputed_skew_ms > manifest_allowed_s * 1000.0:
                    errors.append(
                        f"skew ({recomputed_skew_ms:.3f}ms) > "
                        f"manifest allowed ({manifest_allowed_s*1000:.3f}ms)")

            # 检查 3: recomputed ≤ config allowed
            if recomputed_skew_ms > max_inter_camera_skew_ms:
                errors.append(
                    f"skew ({recomputed_skew_ms:.3f}ms) > "
                    f"config ({max_inter_camera_skew_ms:.3f}ms)")
            else:
                warnings.append(
                    f"skew {recomputed_skew_ms:.3f}ms ≤ {max_inter_camera_skew_ms}ms")
        else:
            errors.append(f"三相机 stamp 不全: {len(stamps_ns)}/3")

        # 检查 4: declared ≤ manifest allowed
        if declared_skew_s is not None and manifest_allowed_s is not None:
            if declared_skew_s > manifest_allowed_s:
                errors.append(
                    f"声明 skew ({declared_skew_s*1000:.3f}ms) > "
                    f"manifest allowed ({manifest_allowed_s*1000:.3f}ms)")

    else:
        sync_report["source"] = "missing"
        if allow_legacy_fallback:
            sync_report["source"] = "quality.yaml (legacy fallback)"
            warnings.append("group_manifest.json 不存在, 降级到 quality.yaml")
            timestamps = {}
            for rgbd in rgbd_list:
                ts = rgbd.quality.get("color_timestamp", {})
                if ts:
                    timestamps[rgbd.camera_name] = ts.get("secs", 0) * 1e9 + ts.get("nsecs", 0)
            if len(timestamps) >= 2:
                skew_ms = (max(timestamps.values()) - min(timestamps.values())) / 1e6
                sync_report["recomputed_skew_ms"] = skew_ms
                if skew_ms > max_inter_camera_skew_ms:
                    errors.append(f"skew {skew_ms:.2f}ms > {max_inter_camera_skew_ms}ms")
        else:
            errors.append("group_manifest.json 不存在. 正式 Gate 要求 manifest.")

    sync_report["passed"] = len(errors) == 0
    return len(errors) == 0, errors, warnings, sync_report

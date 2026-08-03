"""
CR5 Reconstruction — RGB-D 数据加载与契约验证.

提供:
  - RGBDData:              单帧 RGB-D 数据容器 (含 CameraInfo width/height)
  - load_rgbd_data:        加载一组 RGB-D 文件
  - validate_rgbd_contract: 验证 RGB-D 数据契约 (含配准状态)
  - validate_sync_group:    跨相机同步组验证 (优先读取 group_manifest.json)

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import os, yaml, json, logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

from cr5_spray_perception.reconstruction.registration import (
    DepthRegistration, determine_registration_status,
    COLOR_DEPTH_COLOCATED_THRESHOLDS,
)

logger = logging.getLogger(__name__)

EXPECTED_COLOR_EXT = ".png"
EXPECTED_DEPTH_EXT = ".npy"
EXPECTED_CINFO_EXT = ".yaml"
EXPECTED_QUALITY_EXT = ".yaml"


@dataclass
class RGBDData:
    """单帧 RGB-D 数据."""
    camera_name: str
    color_path: str = ""
    depth_path: str = ""
    color_cinfo_path: str = ""
    depth_cinfo_path: str = ""
    quality_path: str = ""

    # 加载后的数据
    color: Optional[np.ndarray] = None          # (H, W, 3) uint8 BGR
    depth_raw: Optional[np.ndarray] = None      # 原始深度 (uint16 or float32)
    depth_meters: Optional[np.ndarray] = None   # 深度 (float32, 米)
    depth_unit: str = ""                        # "meter" or "mm"
    color_K: Optional[np.ndarray] = None        # 3×3
    depth_K: Optional[np.ndarray] = None        # 3×3

    # 图像尺寸 (从实际图像)
    color_width: int = 0
    color_height: int = 0
    depth_width: int = 0
    depth_height: int = 0

    # CameraInfo 声明的尺寸 (从 YAML)
    color_cinfo_width: int = 0
    color_cinfo_height: int = 0
    depth_cinfo_width: int = 0
    depth_cinfo_height: int = 0

    # frame_id
    color_frame_id: str = ""
    depth_frame_id: str = ""

    # RGB-D 配准契约
    registration: Optional[DepthRegistration] = None

    quality: dict = field(default_factory=dict)

    # 验证结果
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    @property
    def depth_registered_to_color(self) -> bool:
        """检查深度是否已配准到彩色 (含 COLOCATED)."""
        if self.registration is not None:
            return self.registration.is_registered
        # 回退: 旧逻辑 (frame_id 相同 + 尺寸相同)
        return (self.depth_width == self.color_width and
                self.depth_height == self.color_height and
                self.depth_frame_id == self.color_frame_id)

    @property
    def rgb_indexing_safe(self) -> bool:
        """RGB 颜色像素索引是否安全."""
        if self.registration is not None:
            return self.registration.rgb_indexing_safe
        return self.depth_frame_id == self.color_frame_id


def load_yaml(path: str) -> dict:
    """加载 YAML 文件."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


def parse_camera_info(cinfo: dict) -> dict:
    """解析 CameraInfo 消息格式的 YAML.

    支持两种格式:
      1. ROS CameraInfo YAML (含 K, D, width, height, header.frame_id)
      2. 简化格式 (含 image_width, image_height, camera_matrix.data)

    Returns:
        {"K": 3×3 or None, "width": int, "height": int, "frame_id": str}
    """
    result = {"K": None, "width": 0, "height": 0, "frame_id": ""}

    # 尝试 ROS CameraInfo 格式
    if "K" in cinfo and isinstance(cinfo["K"], list) and len(cinfo["K"]) == 9:
        result["K"] = np.array(cinfo["K"]).reshape(3, 3).astype(np.float64)
    elif "camera_matrix" in cinfo:
        cm = cinfo["camera_matrix"]
        if "data" in cm:
            result["K"] = np.array(cm["data"]).reshape(3, 3).astype(np.float64)

    # 尺寸
    result["width"] = cinfo.get("width", cinfo.get("image_width", 0))
    result["height"] = cinfo.get("height", cinfo.get("image_height", 0))

    # frame_id
    header = cinfo.get("header", {})
    result["frame_id"] = header.get("frame_id", cinfo.get("frame_id", ""))

    return result


def _validate_K(K: np.ndarray, label: str) -> List[str]:
    """校验相机内参矩阵 K."""
    errors = []
    if K is None:
        errors.append(f"{label}: K 为 None")
        return errors
    if K.shape != (3, 3):
        errors.append(f"{label}: K shape={K.shape}, 期望 (3,3)")
        return errors
    if not np.all(np.isfinite(K)):
        errors.append(f"{label}: K 含 NaN/Inf")
    fx, fy = K[0, 0], K[1, 1]
    if fx <= 0:
        errors.append(f"{label}: fx={fx} ≤ 0")
    if fy <= 0:
        errors.append(f"{label}: fy={fy} ≤ 0")
    # 检查 K[0,1] 应为 0 (非 skew)
    if abs(K[0, 1]) > 1e-6:
        errors.append(f"{label}: K[0,1]={K[0,1]:.2e}, 期望 0 (无 skew)")
    # K[1,0] 应为 0
    if abs(K[1, 0]) > 1e-6:
        errors.append(f"{label}: K[1,0]={K[1,0]:.2e}, 期望 0")
    # K[2,2] 应为 1
    if abs(K[2, 2] - 1.0) > 1e-10:
        errors.append(f"{label}: K[2,2]={K[2,2]:.2e}, 期望 1")
    return errors


def load_rgbd_data(camera_dir: str, camera_name: str) -> RGBDData:
    """从相机目录加载一组 RGB-D 文件.

    期望目录包含:
      color.png
      depth.npy
      color_camera_info.yaml
      depth_camera_info.yaml
      quality.yaml (可选)

    Args:
        camera_dir: 相机数据目录.
        camera_name: 相机名称 (用于日志标识).

    Returns:
        RGBDData 实例.
    """
    data = RGBDData(camera_name=camera_name)

    # 检查必需文件
    color_path = os.path.join(camera_dir, f"color{EXPECTED_COLOR_EXT}")
    depth_path = os.path.join(camera_dir, f"depth{EXPECTED_DEPTH_EXT}")
    color_cinfo_path = os.path.join(camera_dir, f"color_camera_info{EXPECTED_CINFO_EXT}")
    depth_cinfo_path = os.path.join(camera_dir, f"depth_camera_info{EXPECTED_CINFO_EXT}")
    quality_path = os.path.join(camera_dir, f"quality{EXPECTED_QUALITY_EXT}")

    for label, path in [
        ("color.png", color_path),
        ("depth.npy", depth_path),
        ("color_camera_info.yaml", color_cinfo_path),
        ("depth_camera_info.yaml", depth_cinfo_path),
    ]:
        if not os.path.isfile(path):
            data.errors.append(f"{camera_name}: 缺失 {label}: {path}")

    data.color_path = color_path
    data.depth_path = depth_path
    data.color_cinfo_path = color_cinfo_path
    data.depth_cinfo_path = depth_cinfo_path
    data.quality_path = quality_path

    if not data.is_valid:
        return data

    # 加载彩色图
    data.color = cv2.imread(color_path)
    if data.color is None:
        data.errors.append(f"{camera_name}: 无法读取 color.png")
    else:
        data.color_height, data.color_width = data.color.shape[:2]

    # 加载深度图
    try:
        data.depth_raw = np.load(depth_path)
    except Exception as e:
        data.errors.append(f"{camera_name}: 无法读取 depth.npy: {e}")
        return data

    if data.depth_raw is None or data.depth_raw.size == 0:
        data.errors.append(f"{camera_name}: depth.npy 为空")
    else:
        data.depth_height, data.depth_width = data.depth_raw.shape[:2]

    # 加载 color CameraInfo
    try:
        color_cinfo = load_yaml(color_cinfo_path)
        parsed = parse_camera_info(color_cinfo)
        data.color_K = parsed["K"]
        data.color_frame_id = parsed["frame_id"]
        data.color_cinfo_width = parsed["width"]
        data.color_cinfo_height = parsed["height"]
        if data.color_K is None:
            data.errors.append(f"{camera_name}: color_camera_info.yaml 缺少 K 矩阵")
    except Exception as e:
        data.errors.append(f"{camera_name}: 无法解析 color_camera_info.yaml: {e}")

    # 加载 depth CameraInfo
    try:
        depth_cinfo = load_yaml(depth_cinfo_path)
        parsed = parse_camera_info(depth_cinfo)
        data.depth_K = parsed["K"]
        data.depth_frame_id = parsed["frame_id"]
        data.depth_cinfo_width = parsed["width"]
        data.depth_cinfo_height = parsed["height"]
        if data.depth_K is None:
            data.errors.append(f"{camera_name}: depth_camera_info.yaml 缺少 K 矩阵")
    except Exception as e:
        data.errors.append(f"{camera_name}: 无法解析 depth_camera_info.yaml: {e}")

    # 加载 quality (可选)
    if os.path.isfile(quality_path):
        try:
            data.quality = load_yaml(quality_path)
        except Exception:
            data.warnings.append(f"{camera_name}: quality.yaml 解析失败, 跳过")

    # 确定深度单位
    if data.depth_raw is not None:
        if data.depth_raw.dtype == np.uint16:
            data.depth_unit = "mm"
        elif data.depth_raw.dtype == np.float32:
            data.depth_unit = "meter"
        else:
            data.errors.append(f"{camera_name}: 深度 dtype {data.depth_raw.dtype} 不支持, "
                             f"仅支持 uint16 / float32")

    return data


def validate_rgbd_contract(rgbd: RGBDData,
                            require_registered_to_color: bool = True) -> RGBDData:
    """验证 RGBDData 是否满足重建契约.

    检查项:
      - 图像尺寸与 CameraInfo width/height 精确一致
      - depth dtype 有效
      - CameraInfo K 合法 (3×3, finite, fx/fy>0)
      - frame_id 非空
      - depth-color 配准状态 (支持 COLOCATED)
      - color/depth K 比较

    Args:
        rgbd: RGBDData 实例.
        require_registered_to_color: 是否要求 depth 已配准到 color.

    Returns:
        更新了 errors/warnings 的 RGBDData 实例.
    """
    # 基本检查
    if rgbd.color is None:
        rgbd.errors.append(f"{rgbd.camera_name}: color 未加载")
    if rgbd.depth_raw is None:
        rgbd.errors.append(f"{rgbd.camera_name}: depth 未加载")
    if rgbd.color_K is None:
        rgbd.errors.append(f"{rgbd.camera_name}: color K 缺失")
    if rgbd.depth_K is None:
        rgbd.errors.append(f"{rgbd.camera_name}: depth K 缺失")

    if not rgbd.is_valid:
        return rgbd

    # ── CameraInfo K 校验 ──
    for err in _validate_K(rgbd.color_K, f"{rgbd.camera_name}.color_K"):
        rgbd.errors.append(err)
    for err in _validate_K(rgbd.depth_K, f"{rgbd.camera_name}.depth_K"):
        rgbd.errors.append(err)

    # ── 图像尺寸与 CameraInfo 精确校验 ──
    if rgbd.color_width < 1 or rgbd.color_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: color 图像尺寸无效 "
                          f"({rgbd.color_width}x{rgbd.color_height})")
    else:
        if rgbd.color_width != rgbd.color_cinfo_width:
            rgbd.errors.append(
                f"{rgbd.camera_name}: color 图像宽度 ({rgbd.color_width}) != "
                f"CameraInfo width ({rgbd.color_cinfo_width})")
        if rgbd.color_height != rgbd.color_cinfo_height:
            rgbd.errors.append(
                f"{rgbd.camera_name}: color 图像高度 ({rgbd.color_height}) != "
                f"CameraInfo height ({rgbd.color_cinfo_height})")

    if rgbd.depth_width < 1 or rgbd.depth_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: depth 图像尺寸无效 "
                          f"({rgbd.depth_width}x{rgbd.depth_height})")
    else:
        if rgbd.depth_width != rgbd.depth_cinfo_width:
            rgbd.errors.append(
                f"{rgbd.camera_name}: depth 图像宽度 ({rgbd.depth_width}) != "
                f"CameraInfo width ({rgbd.depth_cinfo_width})")
        if rgbd.depth_height != rgbd.depth_cinfo_height:
            rgbd.errors.append(
                f"{rgbd.camera_name}: depth 图像高度 ({rgbd.depth_height}) != "
                f"CameraInfo height ({rgbd.depth_cinfo_height})")

    # ── cx/cy 合理范围检查 ──
    if rgbd.color_K is not None:
        cx, cy = rgbd.color_K[0, 2], rgbd.color_K[1, 2]
        if cx < 0 or cx > rgbd.color_width * 2:
            rgbd.warnings.append(f"{rgbd.camera_name}: color cx={cx:.1f} 偏离图像范围")
        if cy < 0 or cy > rgbd.color_height * 2:
            rgbd.warnings.append(f"{rgbd.camera_name}: color cy={cy:.1f} 偏离图像范围")
    if rgbd.depth_K is not None:
        cx, cy = rgbd.depth_K[0, 2], rgbd.depth_K[1, 2]
        if cx < 0 or cx > rgbd.depth_width * 2:
            rgbd.warnings.append(f"{rgbd.camera_name}: depth cx={cx:.1f} 偏离图像范围")
        if cy < 0 or cy > rgbd.depth_height * 2:
            rgbd.warnings.append(f"{rgbd.camera_name}: depth cy={cy:.1f} 偏离图像范围")

    # ── frame_id 检查 ──
    if not rgbd.color_frame_id:
        rgbd.errors.append(f"{rgbd.camera_name}: color frame_id 为空")
    if not rgbd.depth_frame_id:
        rgbd.errors.append(f"{rgbd.camera_name}: depth frame_id 为空")

    # ── RGB-D 配准契约 ──
    if rgbd.color_frame_id and rgbd.depth_frame_id:
        reg = determine_registration_status(
            color_frame=rgbd.color_frame_id,
            depth_frame=rgbd.depth_frame_id,
            color_width=rgbd.color_width,
            color_height=rgbd.color_height,
            depth_width=rgbd.depth_width,
            depth_height=rgbd.depth_height,
            color_K=rgbd.color_K,
            depth_K=rgbd.depth_K,
            evidence_source="camera_info_yaml",
        )
        rgbd.registration = reg

        for e in reg.errors:
            rgbd.errors.append(f"{rgbd.camera_name}: {e}")
        for w in reg.warnings:
            rgbd.warnings.append(f"{rgbd.camera_name}: {w}")

        # 配准要求检查
        if require_registered_to_color and not reg.is_registered:
            rgbd.errors.append(
                f"{rgbd.camera_name}: 深度未配准到彩色 "
                f"(status={reg.status}, color={rgbd.color_frame_id}, "
                f"depth={rgbd.depth_frame_id})")

    return rgbd


def validate_sync_group(rgbd_list: List[RGBDData],
                         expected_cameras: List[str],
                         max_inter_camera_skew_ms: float = 5.0,
                         calibrated_optical_frames: Optional[Dict[str, str]] = None,
                         manifest: Optional[dict] = None,
                         allow_legacy_fallback: bool = False,
                         ) -> Tuple[bool, List[str], List[str], dict]:
    """验证三相机同步组.

    优先使用 group_manifest.json 中的精确同步证据.
    如果 manifest 不存在且 allow_legacy_fallback=False, 严格失败.

    Args:
        rgbd_list: 三台相机的 RGBDData 列表.
        expected_cameras: 期望的相机名列表.
        max_inter_camera_skew_ms: 最大跨相机时间偏差 (ms).
        calibrated_optical_frames: {cam_name: optical_frame} 校验 frame_id.
        manifest: 预加载的 group_manifest.json dict (可选).
        allow_legacy_fallback: 是否允许降级到 quality.yaml 时间戳.

    Returns:
        (passed, errors, warnings, sync_report)
        其中 sync_report 包含同步验证详情.
    """
    errors = []
    warnings = []
    sync_report = {
        "source": "unknown",
        "method": "unknown",
        "actual_skew_ms": None,
        "maximum_allowed_ms": max_inter_camera_skew_ms,
        "passed": False,
    }

    present = {r.camera_name for r in rgbd_list}

    # 三台相机齐全
    for cam in expected_cameras:
        if cam not in present:
            errors.append(f"缺失相机: {cam}")
    if errors:
        return False, errors, warnings, sync_report

    # 每台相机各自验证
    for rgbd in rgbd_list:
        if not rgbd.is_valid:
            for e in rgbd.errors:
                errors.append(f"{rgbd.camera_name}: {e}")
        for w in rgbd.warnings:
            warnings.append(f"{rgbd.camera_name}: {w}")

    # optical_frame 一致性检查
    if calibrated_optical_frames:
        for rgbd in rgbd_list:
            expected_frame = calibrated_optical_frames.get(rgbd.camera_name)
            if expected_frame and rgbd.color_frame_id:
                if rgbd.color_frame_id != expected_frame:
                    errors.append(
                        f"{rgbd.camera_name}: color frame_id '{rgbd.color_frame_id}' "
                        f"与 calibrated_rig optical_frame '{expected_frame}' 不一致")

    # ── 跨相机同步验证: 优先读取 manifest ──
    if manifest is not None:
        sync_report["source"] = "group_manifest.json"

        # 检查 manifest.success
        if not manifest.get("success", False):
            errors.append("group_manifest.json: success=false")
            sync_report["passed"] = False
            return len(errors) == 0, errors, warnings, sync_report

        # 检查 captured == expected
        captured = manifest.get("captured", 0)
        expected = manifest.get("expected", len(expected_cameras))
        if captured != expected:
            errors.append(
                f"group_manifest.json: captured={captured} != expected={expected}")
            sync_report["passed"] = False
            return len(errors) == 0, errors, warnings, sync_report

        # 检查 method
        ccs = manifest.get("cross_camera_sync", {})
        method = ccs.get("method", "unknown")
        sync_report["method"] = method
        if method != "exact_stamp_ns":
            errors.append(
                f"group_manifest.json: sync method='{method}', 期望 'exact_stamp_ns'")

        # 检查 skew
        actual_skew_s = ccs.get("max_inter_camera_skew_s", None)
        if actual_skew_s is not None:
            actual_skew_ms = actual_skew_s * 1000.0
            sync_report["actual_skew_ms"] = actual_skew_ms

            if actual_skew_ms > max_inter_camera_skew_ms:
                errors.append(
                    f"跨相机时间偏差 {actual_skew_ms:.2f} ms > "
                    f"{max_inter_camera_skew_ms} ms")
            else:
                warnings.append(
                    f"跨相机时间偏差 {actual_skew_ms:.2f} ms "
                    f"(≤ {max_inter_camera_skew_ms} ms)")
        else:
            errors.append("group_manifest.json: 缺少 max_inter_camera_skew_s")

        # 检查三台相机 stamp 齐全
        per_cam_stamps = ccs.get("per_camera_color_stamps", {})
        for cam in expected_cameras:
            if cam not in per_cam_stamps:
                errors.append(f"group_manifest.json: 缺失 {cam} 的 stamp")
            else:
                stamp = per_cam_stamps[cam]
                if stamp.get("stamp_ns") is None and stamp.get("secs") is None:
                    errors.append(f"group_manifest.json: {cam} stamp 为空")

        # manifest 相机列表与 calibrated_rig 一致
        manifest_cams = manifest.get("camera_names", [])
        if set(manifest_cams) != set(expected_cameras):
            errors.append(
                f"group_manifest.json 相机列表 ({sorted(manifest_cams)}) != "
                f"expected ({sorted(expected_cameras)})")

    else:
        # manifest 不存在
        sync_report["source"] = "missing"
        if allow_legacy_fallback:
            # 尝试从 quality.yaml 读取时间戳 (旧回退路径)
            warnings.append("group_manifest.json 不存在, 降级到 quality.yaml 时间戳")
            sync_report["source"] = "quality.yaml (legacy fallback)"
            timestamps = {}
            for rgbd in rgbd_list:
                ts = rgbd.quality.get("color_timestamp", {})
                if ts:
                    ts_ns = (ts.get("secs", 0) * int(1e9) + ts.get("nsecs", 0))
                    timestamps[rgbd.camera_name] = ts_ns

            if len(timestamps) >= 2:
                ts_values = list(timestamps.values())
                skew_ns = max(ts_values) - min(ts_values)
                skew_ms = skew_ns / 1e6
                sync_report["actual_skew_ms"] = skew_ms
                sync_report["method"] = "quality_yaml_approx"
                if skew_ms > max_inter_camera_skew_ms:
                    errors.append(
                        f"跨相机时间偏差 {skew_ms:.2f} ms > {max_inter_camera_skew_ms} ms")
                else:
                    warnings.append(
                        f"跨相机时间偏差 {skew_ms:.2f} ms (≤ {max_inter_camera_skew_ms} ms)")
            else:
                errors.append("无法从 quality.yaml 提取时间戳")
        else:
            # 严格模式: 无 manifest = 失败
            errors.append(
                "group_manifest.json 不存在. "
                "正式 Gate 要求 manifest 同步证据. "
                "使用 --allow-legacy-fallback 可降级到 quality.yaml 时间戳.")

    sync_report["passed"] = len(errors) == 0
    return len(errors) == 0, errors, warnings, sync_report

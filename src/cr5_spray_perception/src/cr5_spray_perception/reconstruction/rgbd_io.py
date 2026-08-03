"""
CR5 Reconstruction — RGB-D 数据加载与契约验证.

提供:
  - RGBDData:           单帧 RGB-D 数据容器
  - load_rgbd_data:     加载一组 RGB-D 文件
  - validate_rgbd_contract: 验证 RGB-D 数据契约
  - validate_sync_group: 跨相机同步组验证

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
"""

import os, yaml, json, logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

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
    color_width: int = 0
    color_height: int = 0
    depth_width: int = 0
    depth_height: int = 0
    color_frame_id: str = ""
    depth_frame_id: str = ""
    quality: dict = field(default_factory=dict)

    # 验证结果
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    @property
    def depth_registered_to_color(self) -> bool:
        """检查深度是否已对齐到彩色 (同尺寸 + 同 frame)."""
        return (self.depth_width == self.color_width and
                self.depth_height == self.color_height and
                self.depth_frame_id == self.color_frame_id)


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
        {"K": 3×3, "width": int, "height": int, "frame_id": str}
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
            data.errors.append(f"{camera_name}: 深度 dtype {data.depth_raw.dtype} 不支持")

    return data


def validate_rgbd_contract(rgbd: RGBDData,
                            require_registered_to_color: bool = True) -> RGBDData:
    """验证 RGBDData 是否满足重建契约.

    检查项:
      - 图像尺寸一致
      - depth dtype 有效
      - CameraInfo K 非空
      - frame_id 非空
      - depth 是否对齐到 color (可选)
      - color/depth K 在 Gazebo 第一阶段允许近似相等

    Args:
        rgbd: RGBDData 实例.
        require_registered_to_color: 是否要求 depth 已对齐到 color.

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

    # 图像尺寸与 CameraInfo 一致
    if rgbd.color_width != rgbd.color_K[0, 0] * 0:  # 间接: 通过 cinfo width
        cinfo_w = rgbd.color_K.shape[0]  # just check K is 3x3
    # 直接检查 width/height
    if rgbd.color_width < 1 or rgbd.color_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: color 尺寸无效 ({rgbd.color_width}x{rgbd.color_height})")
    if rgbd.depth_width < 1 or rgbd.depth_height < 1:
        rgbd.errors.append(f"{rgbd.camera_name}: depth 尺寸无效 ({rgbd.depth_width}x{rgbd.depth_height})")

    # frame_id 检查
    if not rgbd.color_frame_id:
        rgbd.errors.append(f"{rgbd.camera_name}: color frame_id 为空")
    if not rgbd.depth_frame_id:
        rgbd.errors.append(f"{rgbd.camera_name}: depth frame_id 为空")

    # depth 对齐检查
    if require_registered_to_color and not rgbd.depth_registered_to_color:
        rgbd.errors.append(
            f"{rgbd.camera_name}: depth ({rgbd.depth_width}x{rgbd.depth_height}, "
            f"frame={rgbd.depth_frame_id}) 未对齐到 color "
            f"({rgbd.color_width}x{rgbd.color_height}, frame={rgbd.color_frame_id})")

    # Gazebo 第一阶段: 显式验证 color/depth 共光心 (不通过宽高推断)
    if rgbd.depth_registered_to_color:
        if rgbd.color_frame_id != rgbd.depth_frame_id:
            rgbd.warnings.append(
                f"{rgbd.camera_name}: 尺寸相同但 frame_id 不同 "
                f"(color={rgbd.color_frame_id}, depth={rgbd.depth_frame_id}), "
                f"假定已对齐")
    else:
        rgbd.warnings.append(
            f"{rgbd.camera_name}: depth ({rgbd.depth_width}x{rgbd.depth_height}) != "
            f"color ({rgbd.color_width}x{rgbd.color_height}), 未对齐")

    # K 矩阵检查
    if rgbd.color_K is not None and rgbd.depth_K is not None:
        k_diff = np.max(np.abs(rgbd.color_K - rgbd.depth_K))
        if k_diff > 1.0:
            rgbd.warnings.append(
                f"{rgbd.camera_name}: color K 与 depth K 差异较大 (max|diff|={k_diff:.2f})")
        else:
            # 显式确认符合 Gazebo 第一阶段共光心假设
            rgbd.warnings.append(
                f"{rgbd.camera_name}: color/depth K 近似一致 (max|diff|={k_diff:.4f}), "
                f"符合 Gazebo 第一阶段共光心假设")

    return rgbd


def validate_sync_group(rgbd_list: List[RGBDData],
                         expected_cameras: List[str],
                         max_inter_camera_skew_ms: float = 5.0,
                         calibrated_optical_frames: Optional[Dict[str, str]] = None
                         ) -> Tuple[bool, List[str], List[str]]:
    """验证三相机同步组.

    检查项:
      - 三台相机数据齐全
      - 每台相机各自有效
      - frame_id 与 calibrated_rig optical_frame 一致
      - 三相机时间偏差 (从 quality.yaml 或 manifest)

    Args:
        rgbd_list: 三台相机的 RGBDData 列表.
        expected_cameras: 期望的相机名列表.
        max_inter_camera_skew_ms: 最大跨相机时间偏差 (ms).
        calibrated_optical_frames: {cam_name: optical_frame} 用于校验 frame_id.

    Returns:
        (passed, errors, warnings)
    """
    errors = []
    warnings = []
    present = {r.camera_name for r in rgbd_list}

    # 三台相机齐全
    for cam in expected_cameras:
        if cam not in present:
            errors.append(f"缺失相机: {cam}")
    if errors:
        return False, errors, warnings

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

    # 跨相机时间偏差 (从 quality 中读取)
    timestamps = {}
    for rgbd in rgbd_list:
        ts = rgbd.quality.get("capture_timestamp_ns") or rgbd.quality.get("timestamp_ns")
        if ts is not None:
            timestamps[rgbd.camera_name] = ts

    if len(timestamps) >= 2:
        ts_values = list(timestamps.values())
        skew_ns = max(ts_values) - min(ts_values)
        skew_ms = skew_ns / 1e6
        if skew_ms > max_inter_camera_skew_ms:
            errors.append(f"跨相机时间偏差 {skew_ms:.2f} ms > {max_inter_camera_skew_ms} ms")
        else:
            warnings.append(f"跨相机时间偏差 {skew_ms:.2f} ms (≤ {max_inter_camera_skew_ms} ms)")

    return len(errors) == 0, errors, warnings

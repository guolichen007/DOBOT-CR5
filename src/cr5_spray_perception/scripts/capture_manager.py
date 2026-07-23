#!/usr/bin/env python3
"""
CR5 Spray Demo: Multi-Camera RGB-D Capture Manager (V3 — 四路严格同步).

改进 (V2→V3):
- 四路同步: color Image + depth Image + color CameraInfo + depth CameraInfo
- 线程安全: threading.Lock + 不可变快照
- 参数校验: sync_slop_s / max_color_depth_skew_s / max_camera_info_skew_s
- 深度编码感知: 16UC1→mm/1000, 32FC1→m, 未知编码明确失败
- CameraInfo 检查: 彩色和深度分别检查尺寸/时间戳
- 静态 CameraInfo (Header 零时间戳) 允许但标记
- quality.yaml 写入顺序修复 (TF 查询后落盘)
- 文件原子写入: .tmp → flush → fsync → os.replace
"""
import os
import sys
import time
import json
import hashlib
import subprocess
import threading
import yaml
import numpy as np
import rospy
import rospkg
import tf2_ros
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def _sha256_file(path):
    """计算文件 SHA-256."""
    if not os.path.isfile(path):
        return "N/A"
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def _sha256_bytes(data):
    """计算字节串 SHA-256."""
    return hashlib.sha256(data).hexdigest()


def _atomic_write_yaml(filepath, data):
    """原子写入 YAML: .tmp → flush → fsync → os.replace."""
    tmp = filepath + ".tmp"
    with open(tmp, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, filepath)


def _atomic_write_json(filepath, data):
    """原子写入 JSON."""
    tmp = filepath + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2, default=str)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, filepath)


def _atomic_write_npy(filepath, arr):
    """原子写入 NPY."""
    tmp = filepath + ".tmp"
    np.save(tmp, arr)
    os.replace(tmp, filepath)


def _msg_timestamp_secs(msg):
    """从 ROS Header 提取时间戳 (秒). 零时间戳返回 None."""
    if not hasattr(msg, "header") or not hasattr(msg.header, "stamp"):
        return None
    t = msg.header.stamp
    if t.secs == 0 and t.nsecs == 0:
        return None
    return t.secs + t.nsecs * 1e-9


def _ts_dict(msg):
    """返回时间戳 dict."""
    t = msg.header.stamp
    return {"secs": t.secs, "nsecs": t.nsecs}


def depth_image_to_meters(depth_msg, depth_array, configured_scale=None):
    """
    将深度图转换为米.

    支持:
      - 16UC1 / mono16: 默认毫米, scale=0.001
      - 32FC1: 默认米, scale=1.0

    configured_scale: 可选 (未来扩展), 当前仅根据 encoding 推断.
    """
    encoding = getattr(depth_msg, "encoding", "unknown")
    dtype = depth_array.dtype

    if configured_scale is not None:
        return depth_array.astype(np.float64) * configured_scale

    if encoding in ("16UC1", "mono16") or dtype == np.uint16:
        return depth_array.astype(np.float64) / 1000.0

    if encoding in ("32FC1", "32FC2") or dtype in (np.float32, np.float64):
        return depth_array.astype(np.float64)

    # 未知编码 — 不能猜测
    raise ValueError(
        f"Unknown depth encoding: {encoding}, dtype: {dtype}. "
        f"Configure depth_scale_to_m explicitly.")


# ---------------------------------------------------------------------------
# CameraInfo 工具
# ---------------------------------------------------------------------------

def _cinfo_full_dict(msg):
    """提取完整 CameraInfo 字段."""
    return {
        "header": {
            "seq": msg.header.seq,
            "stamp": {"secs": msg.header.stamp.secs,
                      "nsecs": msg.header.stamp.nsecs},
            "frame_id": msg.header.frame_id,
        },
        "height": msg.height,
        "width": msg.width,
        "distortion_model": msg.distortion_model,
        "D": list(msg.D),
        "K": list(msg.K),
        "R": list(msg.R),
        "P": list(msg.P),
        "binning_x": msg.binning_x,
        "binning_y": msg.binning_y,
        "roi": {
            "x_offset": msg.roi.x_offset,
            "y_offset": msg.roi.y_offset,
            "height": msg.roi.height,
            "width": msg.roi.width,
            "do_rectify": msg.roi.do_rectify,
        },
    }


# ---------------------------------------------------------------------------
# CaptureManager V3
# ---------------------------------------------------------------------------

class CaptureManager:
    def __init__(self):
        rospy.init_node("capture_manager")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ── 参数 ──
        self.run_id = rospy.get_param("~run_id", time.strftime("%Y%m%d_%H%M%S"))
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/cr5_spray_data"))
        self.camera_names = rospy.get_param("~camera_names", [])
        self.timeout_s = rospy.get_param("~timeout_s", 10.0)
        self.min_depth_valid_ratio = rospy.get_param("~min_depth_valid_ratio", 0.3)
        self.depth_min_m = rospy.get_param("~depth_min_m", 0.1)
        self.depth_max_m = rospy.get_param("~depth_max_m", 10.0)

        # ── 新增精度参数 ──
        self.sync_slop_s = rospy.get_param("~sync_slop_s", 0.05)
        self.max_color_depth_skew_s = rospy.get_param(
            "~max_color_depth_skew_s", 0.030)
        self.max_camera_info_skew_s = rospy.get_param(
            "~max_camera_info_skew_s", 0.030)

        if not self.camera_names:
            rospy.logerr("capture_manager: camera_names is empty!")
            sys.exit(1)

        # ── 参数校验 ──
        self._validate_params()

        self.expected_camera_count = len(self.camera_names)

        # ── 线程安全 ──
        self._sync_lock = threading.Lock()
        # 采集 token/generation, 避免旧消息进入新 snapshot
        self._capture_active = False
        self._capture_gen = 0

        self.run_dir = os.path.join(self.output_dir, self.run_id)
        self.views_dir = os.path.join(self.run_dir, "views")
        self.logs_dir = os.path.join(self.run_dir, "logs")
        for d in [self.run_dir, self.views_dir, self.logs_dir]:
            os.makedirs(d, exist_ok=True)

        # ── 每相机 四路 message_filters 同步器 ──
        # 每组: color Image + depth Image + color CameraInfo + depth CameraInfo
        self._sync_data = {}   # cam → latest synchronized tuple

        for cam in self.camera_names:
            ns = "/" + cam
            color_sub = message_filters.Subscriber(
                ns + "/camera/color/image_raw", Image)
            depth_sub = message_filters.Subscriber(
                ns + "/camera/depth/image_raw", Image)
            color_info_sub = message_filters.Subscriber(
                ns + "/camera/color/camera_info", CameraInfo)
            depth_info_sub = message_filters.Subscriber(
                ns + "/camera/depth/camera_info", CameraInfo)

            ts = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub, color_info_sub, depth_info_sub],
                queue_size=10, slop=self.sync_slop_s,
                allow_headerless=False)
            ts.registerCallback(self._make_sync_cb(cam))

            self._sync_data[cam] = None

        # Services
        rospy.Service("~capture_all_fixed", Trigger, self._svc_capture_fixed)
        rospy.Service("~capture_scan_sequence", Trigger, self._svc_capture_scan)

        # Compute config SHA256 for manifest
        self._cfg_sha = self._compute_config_sha()
        self._git_sha = self._compute_git_sha()

        # Write initial manifest
        self._write_manifest(0)

        rospy.loginfo("CaptureManager V3 ready: %d cameras, slop=%.3fs, "
                      "max_color_depth_skew=%.3fs, max_cinfo_skew=%.3fs, "
                      "run_id=%s",
                      len(self.camera_names), self.sync_slop_s,
                      self.max_color_depth_skew_s, self.max_camera_info_skew_s,
                      self.run_id)

    # ------------------------------------------------------------------
    # 参数校验
    # ------------------------------------------------------------------

    def _validate_params(self):
        """启动时校验所有参数, 非法则立即失败."""
        errors = []

        if self.sync_slop_s <= 0:
            errors.append(f"sync_slop_s must be > 0, got {self.sync_slop_s}")
        if self.max_color_depth_skew_s < 0:
            errors.append(f"max_color_depth_skew_s must be >= 0, "
                          f"got {self.max_color_depth_skew_s}")
        if self.max_camera_info_skew_s < 0:
            errors.append(f"max_camera_info_skew_s must be >= 0, "
                          f"got {self.max_camera_info_skew_s}")
        if self.sync_slop_s < self.max_color_depth_skew_s:
            errors.append(
                f"sync_slop_s ({self.sync_slop_s}) must be >= "
                f"max_color_depth_skew_s ({self.max_color_depth_skew_s})")
        if self.sync_slop_s < self.max_camera_info_skew_s:
            errors.append(
                f"sync_slop_s ({self.sync_slop_s}) must be >= "
                f"max_camera_info_skew_s ({self.max_camera_info_skew_s})")

        if errors:
            for e in errors:
                rospy.logerr("PARAM ERROR: %s", e)
            sys.exit(1)

    # ------------------------------------------------------------------
    # 同步回调 (线程安全)
    # ------------------------------------------------------------------

    def _make_sync_cb(self, cam):
        """创建闭包捕获 cam 名称."""
        def cb(color_msg, depth_msg, color_info_msg, depth_info_msg):
            with self._sync_lock:
                if not self._capture_active:
                    return
                self._sync_data[cam] = (
                    color_msg, depth_msg, color_info_msg, depth_info_msg)
        return cb

    def _all_synced(self):
        """所有相机都有同步数据."""
        with self._sync_lock:
            return all(v is not None for v in self._sync_data.values())

    # ------------------------------------------------------------------
    # Git SHA
    # ------------------------------------------------------------------

    def _compute_git_sha(self):
        """通过 rospkg 定位仓库根目录获取 Git SHA."""
        try:
            rp = rospkg.RosPack()
            pkg_path = rp.get_path("cr5_spray_sim")
            # 向上寻找 .git 目录
            repo_root = pkg_path
            for _ in range(10):
                if os.path.isdir(os.path.join(repo_root, ".git")):
                    break
                repo_root = os.path.dirname(repo_root)
            else:
                return "unknown (no .git found)"

            sha = subprocess.check_output(
                ["git", "-C", repo_root, "rev-parse", "HEAD"],
                stderr=subprocess.DEVNULL).decode().strip()
            return sha
        except Exception as e:
            return f"unknown ({e})"

    # ------------------------------------------------------------------
    # 配置文件 SHA
    # ------------------------------------------------------------------

    def _compute_config_sha(self):
        """计算关键配置文件的 SHA-256."""
        try:
            rp = rospkg.RosPack()
            pkg = rp.get_path("cr5_spray_sim")
            scene_yaml = os.path.join(pkg, "config", "simulation_scene.yaml")
            calib_yaml = os.path.join(pkg, "config", "calibration",
                                      "calibration_target.yaml")
            return {
                "simulation_scene_yaml_sha256": _sha256_file(scene_yaml),
                "calibration_target_yaml_sha256": _sha256_file(calib_yaml),
            }
        except Exception:
            return {
                "simulation_scene_yaml_sha256": "N/A",
                "calibration_target_yaml_sha256": "N/A",
            }

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------

    def _svc_capture_fixed(self, req):
        ok, msg, path = self.capture_view("fixed")
        return TriggerResponse(success=ok, message=msg)

    def _svc_capture_scan(self, req):
        ok, msg, path = self.capture_view("scan")
        return TriggerResponse(success=ok, message=msg)

    # ------------------------------------------------------------------
    # 核心采集逻辑
    # ------------------------------------------------------------------

    def capture_view(self, mode):
        """严格四路同步采集所有相机.

        Returns: (success: bool, message: str, view_dir: str)
        """
        rospy.loginfo("Capturing view (mode=%s, strict 4-way sync)...", mode)

        # ── 清空旧数据 + 激活采集 ──
        with self._sync_lock:
            self._capture_active = True
            self._capture_gen += 1
            for cam in self.camera_names:
                self._sync_data[cam] = None

        t_start = time.time()
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                if self._all_synced():
                    break
                if time.time() - t_start > self.timeout_s:
                    ready = sum(1 for v in self._sync_data.values()
                                if v is not None)
                    msg = (f"Timeout waiting for synced data: {ready}/"
                           f"{self.expected_camera_count} cameras after "
                           f"{self.timeout_s:.0f}s")
                    rospy.logerr(msg)
                    return False, msg, ""
                rate.sleep()

            # ── 创建不可变快照 ──
            with self._sync_lock:
                snapshot = dict(self._sync_data)

        finally:
            with self._sync_lock:
                self._capture_active = False

        view_id = "view_{:04d}".format(len(os.listdir(self.views_dir)))
        view_dir = os.path.join(self.views_dir, view_id)
        os.makedirs(view_dir, exist_ok=True)

        captured = 0
        errors = []
        per_camera_detail = {}

        for cam in self.camera_names:
            try:
                data = snapshot.get(cam)
                if data is None:
                    errors.append(f"{cam}: no synchronized data")
                    continue

                color_msg, depth_msg, color_info_msg, depth_info_msg = data

                cam_dir = os.path.join(view_dir, cam)
                os.makedirs(cam_dir, exist_ok=True)

                # ── 时间戳检查 ──
                ts_checks = self._check_timestamps(
                    color_msg, depth_msg, color_info_msg, depth_info_msg, cam)
                if ts_checks["fatal"]:
                    errors.append(f"{cam}: {ts_checks['error']}")
                    continue

                # ── CameraInfo 尺寸检查 ──
                cinfo_checks = self._check_camera_info_dims(
                    color_msg, depth_msg, color_info_msg, depth_info_msg, cam)
                if cinfo_checks["fatal"]:
                    errors.append(f"{cam}: {cinfo_checks['error']}")
                    continue

                # ── 保存 color ──
                color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
                cv2.imwrite(os.path.join(cam_dir, "color.png"),
                            cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))

                # ── 保存 depth ──
                depth_img = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough")

                if not np.isfinite(depth_img).all():
                    errors.append(f"{cam}: depth image contains non-finite values")

                _atomic_write_npy(os.path.join(cam_dir, "depth.npy"), depth_img)

                # ── 深度单位转换 ──
                depth_encoding = getattr(depth_msg, "encoding", "unknown")
                depth_dtype = str(depth_img.dtype)
                try:
                    depth_m = depth_image_to_meters(depth_msg, depth_img)
                except ValueError as e:
                    errors.append(f"{cam}: depth conversion - {e}")
                    continue

                if depth_encoding in ("16UC1", "mono16"):
                    depth_scale_to_m = 0.001
                    depth_unit = "mm"
                elif depth_encoding in ("32FC1",):
                    depth_scale_to_m = 1.0
                    depth_unit = "m"
                else:
                    depth_scale_to_m = "unknown"
                    depth_unit = "unknown"

                # ── 深度质量 ──
                finite = np.isfinite(depth_m) & (depth_m > 0)
                in_range = ((depth_m >= self.depth_min_m) &
                            (depth_m <= self.depth_max_m))
                valid = finite & in_range
                valid_ratio = float(np.mean(valid)) if depth_m.size > 0 else 0.0

                depth_quality = {
                    "valid_depth_ratio": valid_ratio,
                    "depth_min_m": float(depth_m[valid].min()) if np.any(valid) else 0,
                    "depth_max_m": float(depth_m[valid].max()) if np.any(valid) else 0,
                    "depth_zeros": int(np.sum(~finite)),
                    "depth_encoding": depth_encoding,
                    "depth_dtype": depth_dtype,
                    "depth_shape": list(depth_img.shape),
                    "depth_unit": depth_unit,
                    "depth_scale_to_m": depth_scale_to_m,
                    "nonzero_pct": round(
                        float(np.sum(finite) / depth_img.size * 100), 2),
                    "finite_pct": round(
                        float(np.sum(np.isfinite(depth_m)) / depth_img.size * 100), 2),
                }

                # ── 保存 CameraInfo ──
                _atomic_write_yaml(
                    os.path.join(cam_dir, "color_camera_info.yaml"),
                    _cinfo_full_dict(color_info_msg))
                _atomic_write_yaml(
                    os.path.join(cam_dir, "depth_camera_info.yaml"),
                    _cinfo_full_dict(depth_info_msg))

                # ── 构建 quality (先汇总, 最后落盘) ──
                quality = {
                    "color_timestamp": _ts_dict(color_msg),
                    "depth_timestamp": _ts_dict(depth_msg),
                    "color_info_timestamp": _ts_dict(color_info_msg),
                    "depth_info_timestamp": _ts_dict(depth_info_msg),
                    "color_depth_skew_s": ts_checks["color_depth_skew_s"],
                    "color_info_skew_s": ts_checks["color_info_skew_s"],
                    "depth_info_skew_s": ts_checks["depth_info_skew_s"],
                    "color_info_timestamp_mode": ts_checks["color_info_timestamp_mode"],
                    "depth_info_timestamp_mode": ts_checks["depth_info_timestamp_mode"],
                    **depth_quality,
                }

                # ── 深度有效比例检查 ──
                if valid_ratio < self.min_depth_valid_ratio:
                    errors.append(
                        f"{cam}: low depth valid {valid_ratio:.1%} "
                        f"< {self.min_depth_valid_ratio:.0%}")

                # ── TF 查找 ──
                try:
                    trans = self.tf_buffer.lookup_transform(
                        "world", color_msg.header.frame_id,
                        color_msg.header.stamp, rospy.Duration(1.0))
                    tf_dict = {
                        "translation": {"x": trans.transform.translation.x,
                                        "y": trans.transform.translation.y,
                                        "z": trans.transform.translation.z},
                        "rotation": {"x": trans.transform.rotation.x,
                                     "y": trans.transform.rotation.y,
                                     "z": trans.transform.rotation.z,
                                     "w": trans.transform.rotation.w},
                        "child_frame_id": trans.child_frame_id,
                        "header": {
                            "frame_id": trans.header.frame_id,
                            "stamp": {
                                "secs": trans.header.stamp.secs,
                                "nsecs": trans.header.stamp.nsecs,
                            },
                        },
                    }
                    _atomic_write_yaml(
                        os.path.join(cam_dir, "T_world_camera.yaml"), tf_dict)
                    quality["tf_lookup_success"] = True
                    quality["tf_parent_frame"] = trans.header.frame_id
                    quality["tf_child_frame"] = trans.child_frame_id
                except Exception as e:
                    errors.append(f"{cam}: TF failed - {e}")
                    quality["tf_lookup_success"] = False
                    quality["tf_error"] = str(e)

                # ── 最后一次性落盘 quality.yaml ──
                _atomic_write_yaml(
                    os.path.join(cam_dir, "quality.yaml"), quality)

                per_camera_detail[cam] = quality
                captured += 1

            except Exception as e:
                errors.append(f"{cam}: {e}")
                rospy.logerr("%s: unexpected error: %s", cam, e)

        # ── 严格 N/N ──
        success = (captured == self.expected_camera_count and len(errors) == 0)

        # ── 写入 manifest ──
        self._write_manifest(captured, per_camera_detail, errors)

        elapsed = time.time() - t_start
        max_skew = 0.0
        if per_camera_detail:
            max_skew = max(
                d.get("color_depth_skew_s", 0)
                for d in per_camera_detail.values())

        status = "PASS" if success else "FAIL"
        msg = (f"SYNC_CAPTURE_{captured}_OF_{self.expected_camera_count}_"
               f"{status}: {captured}/{self.expected_camera_count} cameras "
               f"in {elapsed:.1f}s, max_skew={max_skew*1000:.1f}ms")
        if errors:
            msg += ". Errors: " + "; ".join(errors[:3])
        rospy.loginfo(msg)

        if success:
            rospy.loginfo("SYNC_CAPTURE_%d_OF_%d_PASS", captured,
                          self.expected_camera_count)
            print(f"SYNC_CAPTURE_{captured}_OF_{self.expected_camera_count}_PASS")
            print(f"max_color_depth_skew_s <= {self.max_color_depth_skew_s}")
        else:
            rospy.logerr("SYNC_CAPTURE_%d_OF_%d_FAIL", captured,
                         self.expected_camera_count)

        return success, msg, view_dir

    # ------------------------------------------------------------------
    # 时间戳检查
    # ------------------------------------------------------------------

    def _check_timestamps(self, color_msg, depth_msg,
                          color_info_msg, depth_info_msg, cam):
        """四路时间戳校验."""
        result = {
            "fatal": False,
            "error": "",
            "color_depth_skew_s": 0.0,
            "color_info_skew_s": 0.0,
            "depth_info_skew_s": 0.0,
            "color_info_timestamp_mode": "unknown",
            "depth_info_timestamp_mode": "unknown",
        }

        # color 和 depth 必须有有效时间戳
        color_ts = _msg_timestamp_secs(color_msg)
        depth_ts = _msg_timestamp_secs(depth_msg)

        if color_ts is None:
            result["fatal"] = True
            result["error"] = "color image has zero/invalid timestamp"
            return result
        if depth_ts is None:
            result["fatal"] = True
            result["error"] = "depth image has zero/invalid timestamp"
            return result

        # color-depth skew
        skew_cd = abs(depth_ts - color_ts)
        result["color_depth_skew_s"] = float(skew_cd)
        if skew_cd > self.max_color_depth_skew_s:
            result["fatal"] = True
            result["error"] = (
                f"color-depth skew {skew_cd*1000:.1f}ms > "
                f"{self.max_color_depth_skew_s*1000:.0f}ms")
            return result

        # CameraInfo 时间戳检查
        color_ci_ts = _msg_timestamp_secs(color_info_msg)
        depth_ci_ts = _msg_timestamp_secs(depth_info_msg)

        if color_ci_ts is not None:
            result["color_info_timestamp_mode"] = "stamped"
            skew_cci = abs(color_ci_ts - color_ts)
            result["color_info_skew_s"] = float(skew_cci)
            if skew_cci > self.max_camera_info_skew_s:
                result["fatal"] = True
                result["error"] = (
                    f"color-cinfo skew {skew_cci*1000:.1f}ms > "
                    f"{self.max_camera_info_skew_s*1000:.0f}ms")
                return result
        else:
            result["color_info_timestamp_mode"] = "static_or_unstamped"

        if depth_ci_ts is not None:
            result["depth_info_timestamp_mode"] = "stamped"
            skew_dci = abs(depth_ci_ts - depth_ts)
            result["depth_info_skew_s"] = float(skew_dci)
            if skew_dci > self.max_camera_info_skew_s:
                result["fatal"] = True
                result["error"] = (
                    f"depth-cinfo skew {skew_dci*1000:.1f}ms > "
                    f"{self.max_camera_info_skew_s*1000:.0f}ms")
                return result
        else:
            result["depth_info_timestamp_mode"] = "static_or_unstamped"

        return result

    # ------------------------------------------------------------------
    # CameraInfo 尺寸检查
    # ------------------------------------------------------------------

    @staticmethod
    def _check_camera_info_dims(color_msg, depth_msg,
                                color_info_msg, depth_info_msg, cam):
        """彩色和深度 CameraInfo 分别检查尺寸."""
        result = {"fatal": False, "error": ""}

        # 彩色
        if (color_info_msg.width != color_msg.width or
                color_info_msg.height != color_msg.height):
            result["fatal"] = True
            result["error"] = (
                f"color CameraInfo dim ({color_info_msg.width}x{color_info_msg.height}) "
                f"!= color Image dim ({color_msg.width}x{color_msg.height})")
            return result

        # 深度
        if (depth_info_msg.width != depth_msg.width or
                depth_info_msg.height != depth_msg.height):
            result["fatal"] = True
            result["error"] = (
                f"depth CameraInfo dim ({depth_info_msg.width}x{depth_info_msg.height}) "
                f"!= depth Image dim ({depth_msg.width}x{depth_msg.height})")
            return result

        return result

    # ------------------------------------------------------------------
    # Manifest
    # ------------------------------------------------------------------

    def _write_manifest(self, captured_count, per_camera_detail=None, errors=None):
        """Write/update manifest.yaml (增强版 V3)."""
        manifest = {
            "run_id": self.run_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "git_sha": self._git_sha,
            "ros_distro": os.environ.get("ROS_DISTRO", "noetic"),
            "expected_camera_count": self.expected_camera_count,
            "camera_names": self.camera_names,
            "views_captured": captured_count,
            "output_dir": self.run_dir,
            "sync_slop_s": self.sync_slop_s,
            "max_color_depth_skew_s": self.max_color_depth_skew_s,
            "max_camera_info_skew_s": self.max_camera_info_skew_s,
            "sync_method": "message_filters.ApproximateTimeSynchronizer(4-way)",
        }

        # 配置文件 SHA256
        if self._cfg_sha:
            manifest.update(self._cfg_sha)

        # 每相机详情
        if per_camera_detail:
            manifest["per_camera"] = {}
            for cam, detail in per_camera_detail.items():
                manifest["per_camera"][cam] = {
                    "color_timestamp": detail.get("color_timestamp"),
                    "depth_timestamp": detail.get("depth_timestamp"),
                    "color_info_timestamp": detail.get("color_info_timestamp"),
                    "depth_info_timestamp": detail.get("depth_info_timestamp"),
                    "color_depth_skew_s": detail.get("color_depth_skew_s"),
                    "color_info_skew_s": detail.get("color_info_skew_s"),
                    "depth_info_skew_s": detail.get("depth_info_skew_s"),
                    "color_info_timestamp_mode": detail.get(
                        "color_info_timestamp_mode"),
                    "depth_info_timestamp_mode": detail.get(
                        "depth_info_timestamp_mode"),
                    "valid_depth_ratio": detail.get("valid_depth_ratio"),
                    "depth_encoding": detail.get("depth_encoding"),
                    "depth_unit": detail.get("depth_unit"),
                    "tf_lookup_success": detail.get("tf_lookup_success"),
                    "tf_parent_frame": detail.get("tf_parent_frame"),
                    "tf_child_frame": detail.get("tf_child_frame"),
                }

        if errors:
            manifest["errors"] = errors

        _atomic_write_yaml(
            os.path.join(self.run_dir, "manifest.yaml"), manifest)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    CaptureManager().run()

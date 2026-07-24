#!/usr/bin/env python3
"""
JointCaptureManager — 跨相机同步采集.

在 CaptureManager V3 的 per-camera 4-way sync 基础上,
增加跨相机 3-way ApproximateTimeSynchronizer,
输出 SyncFrameGroup (三台相机同一时刻的 RGB-D 帧组).

架构:
  Stage 1 (已有): 每台相机 4-way ATS → per-camera synced tuple
  Stage 2 (新增): 3-way ATS 跨相机的 synced color → SyncFrameGroup

服务:
  /joint_capture_manager/capture_sync_group (Trigger)
    → 采集一个同步帧组
    → 保存到 <run_dir>/groups/<group_id>/<cam_name>/

用法:
  rosrun cr5_spray_perception joint_capture_manager.py \
    _run_id:=calib_session_001 \
    _camera_names:="[cam_front_left, cam_front_right, cam_rear]"
"""
import os
import sys
import time
import threading
import importlib.util
import cv2
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse

# 从 capture_manager.py 动态加载 (ROS Python scripts 不在标准 package 中)
_cm_path = os.path.join(os.path.dirname(__file__), "capture_manager.py")
_cm_spec = importlib.util.spec_from_file_location("capture_manager", _cm_path)
_cm = importlib.util.module_from_spec(_cm_spec)
_cm_spec.loader.exec_module(_cm)

CaptureManager = _cm.CaptureManager
_atomic_write_yaml = _cm._atomic_write_yaml
_atomic_write_json = _cm._atomic_write_json
_atomic_write_npy = _cm._atomic_write_npy
_cinfo_full_dict = _cm._cinfo_full_dict
_ts_dict = _cm._ts_dict
depth_image_to_meters = _cm.depth_image_to_meters


class JointCaptureManager(CaptureManager):
    """扩展 CaptureManager V3, 增加跨相机同步."""

    def __init__(self):
        super().__init__()

        # ── 跨相机同步 ──
        self.cross_sync_slop_s = rospy.get_param("~cross_sync_slop_s", 0.033)
        self._cross_sync_data = None
        self._cross_lock = threading.Lock()
        self._setup_cross_sync()

        # ── SyncFrameGroup 计数 ──
        self._group_counter = 0
        self.groups_dir = os.path.join(self.run_dir, "groups")
        os.makedirs(self.groups_dir, exist_ok=True)

        rospy.Service("~capture_sync_group", Trigger,
                      self._svc_capture_sync_group)

        rospy.loginfo("JointCaptureManager ready: %d cameras, "
                      "cross_slop=%.3fs",
                      len(self.camera_names), self.cross_sync_slop_s)

    def _setup_cross_sync(self):
        """创建跨相机的 3-way ApproximateTimeSynchronizer.

        对每台相机订阅 color/image_raw (已经 per-camera 4-way 同步过),
        用 ATS 确保三台相机的 color 帧时间戳接近.
        """
        color_subs = []
        for cam in self.camera_names:
            sub = message_filters.Subscriber(
                "/" + cam + "/camera/color/image_raw", Image)
            color_subs.append(sub)

        # 动态创建 3-way (或 N-way) 同步器
        self._cross_ats = message_filters.ApproximateTimeSynchronizer(
            color_subs, queue_size=5, slop=self.cross_sync_slop_s,
            allow_headerless=False)
        self._cross_ats.registerCallback(self._cross_sync_cb)

    def _cross_sync_cb(self, *color_msgs):
        """跨相机同步回调.

        当三台相机的 color 消息时间戳在 cross_sync_slop_s 内时触发.
        严格校验: per-camera sync 中的 color 消息必须与 ATS 触发的消息匹配.
        """
        with self._cross_lock:
            if not self._capture_active:
                return

            # 检查所有相机都有 per-camera sync 数据
            with self._sync_lock:
                all_ready = all(
                    self._sync_data.get(cam) is not None
                    for cam in self.camera_names)
                if not all_ready:
                    return

                # 严格校验: per-camera sync 的 color 时间戳必须匹配 ATS 触发消息
                max_skew = 0.0
                for i, cam in enumerate(self.camera_names):
                    data = self._sync_data.get(cam)
                    if data is None:
                        return
                    color_synced = data[0]  # per-camera 4-way sync 中的 color 消息
                    # 比较 ATS 触发的 color 消息和 per-camera sync 中的 color 消息
                    skew = abs((color_msgs[i].header.stamp -
                                color_synced.header.stamp).to_sec())
                    if skew > self.cross_sync_slop_s:
                        return  # per-camera sync 数据不是同一帧
                    max_skew = max(max_skew, skew)

                # 计算实际跨相机最大时间差
                inter_cam_skew = 0.0
                for i in range(len(self.camera_names)):
                    for j in range(i + 1, len(self.camera_names)):
                        d = abs((color_msgs[i].header.stamp -
                                 color_msgs[j].header.stamp).to_sec())
                        inter_cam_skew = max(inter_cam_skew, d)

                # 仿真硬门限 ≤5ms, 容忍 ≤10ms
                max_allowed_inter_cam_skew = rospy.get_param(
                    "~max_inter_camera_skew_s", 0.005)
                if inter_cam_skew > max_allowed_inter_cam_skew:
                    rospy.logwarn_throttle(
                        5.0, "Cross-camera skew %.1fms > %.1fms, rejecting",
                        inter_cam_skew * 1000, max_allowed_inter_cam_skew * 1000)
                    return

                snapshot = dict(self._sync_data)
                # 记录实际跨相机时间差
                snapshot["_cross_skew_s"] = inter_cam_skew
                snapshot["_max_color_skew_s"] = max_skew

            self._cross_sync_data = snapshot

    def capture_sync_group(self):
        """采集一个跨相机同步帧组.

        Returns: (success: bool, message: str, group_dir: str)
        """
        rospy.loginfo("Capturing sync group (cross-camera 3-way sync)...")

        # ── 激活采集 ──
        with self._sync_lock:
            self._capture_active = True
            self._capture_started_ros = rospy.Time.now()
            for cam in self.camera_names:
                self._sync_data[cam] = None

        with self._cross_lock:
            self._cross_sync_data = None

        t_start = time.time()
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                # 等待 per-camera sync + cross-camera sync
                with self._cross_lock:
                    cross_ready = self._cross_sync_data is not None
                if cross_ready and self._all_synced():
                    break
                if time.time() - t_start > self.timeout_s:
                    with self._sync_lock:
                        ready = sum(1 for v in self._sync_data.values()
                                    if v is not None)
                    msg = (f"Timeout waiting for sync group: {ready}/"
                           f"{self.expected_camera_count} cameras, "
                           f"cross={cross_ready}")
                    rospy.logerr(msg)
                    return False, msg, ""
                rate.sleep()

            with self._cross_lock:
                snapshot = dict(self._cross_sync_data)
        finally:
            with self._sync_lock:
                self._capture_active = False
                self._capture_started_ros = None
            with self._cross_lock:
                self._cross_sync_data = None

        # ── 保存帧组 ──
        group_id = self._group_counter
        self._group_counter += 1
        group_dir = os.path.join(self.groups_dir,
                                 "group_{:04d}".format(group_id))
        os.makedirs(group_dir, exist_ok=True)

        captured = 0
        errors = []
        per_camera_detail = {}

        for cam in self.camera_names:
            try:
                data = snapshot.get(cam)
                if data is None:
                    errors.append("{}: no synced data".format(cam))
                    continue

                color_msg, depth_msg, color_info_msg, depth_info_msg = data
                cam_dir = os.path.join(group_dir, cam)
                os.makedirs(cam_dir, exist_ok=True)

                # ── 时间戳检查 ──
                ts_checks = self._check_timestamps(
                    color_msg, depth_msg, color_info_msg, depth_info_msg, cam)
                if ts_checks["fatal"]:
                    errors.append("{}: {}".format(cam, ts_checks["error"]))
                    continue

                # ── 保存 color ──
                color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
                color_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
                color_path = os.path.join(cam_dir, "color.png")
                color_tmp = color_path + ".tmp.png"
                ok = cv2.imwrite(color_tmp, color_bgr)
                if not ok:
                    raise IOError("cv2.imwrite failed for {}".format(color_path))
                os.replace(color_tmp, color_path)

                # ── 保存 depth ──
                depth_img = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough")
                _atomic_write_npy(os.path.join(cam_dir, "depth.npy"), depth_img)

                # ── 深度质量 ──
                depth_encoding = getattr(depth_msg, "encoding", "unknown")
                depth_dtype = str(depth_img.dtype)
                try:
                    depth_m = depth_image_to_meters(depth_msg, depth_img)
                except ValueError as e:
                    errors.append("{}: depth conversion - {}".format(cam, e))
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

                finite = np.isfinite(depth_m) & (depth_m > 0)
                valid_ratio = float(np.mean(finite)) if depth_m.size > 0 else 0.0

                depth_quality = {
                    "valid_depth_ratio": valid_ratio,
                    "depth_encoding": depth_encoding,
                    "depth_dtype": depth_dtype,
                    "depth_shape": list(depth_img.shape),
                    "depth_unit": depth_unit,
                    "depth_scale_to_m": depth_scale_to_m,
                }

                # ── 保存 CameraInfo ──
                _atomic_write_yaml(
                    os.path.join(cam_dir, "color_camera_info.yaml"),
                    _cinfo_full_dict(color_info_msg))
                _atomic_write_yaml(
                    os.path.join(cam_dir, "depth_camera_info.yaml"),
                    _cinfo_full_dict(depth_info_msg))

                # ── quality.yaml ──
                quality = {
                    "color_timestamp": _ts_dict(color_msg),
                    "depth_timestamp": _ts_dict(depth_msg),
                    "color_depth_skew_s": ts_checks["color_depth_skew_s"],
                    "cross_camera_sync": "3-way ApproximateTimeSynchronizer",
                    **depth_quality,
                }
                _atomic_write_yaml(
                    os.path.join(cam_dir, "quality.yaml"), quality)

                per_camera_detail[cam] = quality
                captured += 1

            except Exception as e:
                errors.append("{}: {}".format(cam, e))
                rospy.logerr("%s: %s", cam, e)

        # ── group manifest ──
        success = (captured == self.expected_camera_count and len(errors) == 0)
        group_manifest = {
            "group_id": group_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "camera_names": self.camera_names,
            "captured": captured,
            "expected": self.expected_camera_count,
            "success": success,
            "errors": errors if errors else [],
            "per_camera": per_camera_detail,
        }
        _atomic_write_json(
            os.path.join(group_dir, "group_manifest.json"), group_manifest)

        msg = ("SYNC_GROUP_{}_OF_{}_{}: {}/{} cameras".format(
            captured, self.expected_camera_count,
            "PASS" if success else "FAIL",
            captured, self.expected_camera_count))
        rospy.loginfo(msg)
        return success, msg, group_dir

    def _svc_capture_sync_group(self, req):
        ok, msg, path = self.capture_sync_group()
        # 将 group_dir 嵌入 message，格式: "GROUP_DIR:<path>|<original_msg>"
        if ok and path:
            full_msg = "GROUP_DIR:{}|{}".format(path, msg)
        else:
            full_msg = msg
        return TriggerResponse(success=ok, message=full_msg)


if __name__ == "__main__":
    JointCaptureManager().run()

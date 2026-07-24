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
    """扩展 CaptureManager V3, 增加跨相机同步.

    P0-A 修复: 按 color stamp 精确匹配 RGB-D tuple.
    - per-camera 缓存最近 N 个四路 tuple, 按 color timestamp (ns) 索引
    - ATS 回调用精确 color stamp 查找对应 tuple (不允许 33ms 模糊匹配)
    - 最终 inter-camera skew 从实际保存的 color stamp 重新计算
    """

    # 精确匹配容差: 同一帧不同 topic 的时间戳抖动 < 100us
    EXACT_MATCH_TOLERANCE_S = 0.0001

    def __init__(self):
        super().__init__()

        # ── 跨相机同步 ──
        self.cross_sync_slop_s = rospy.get_param("~cross_sync_slop_s", 0.033)
        self.max_inter_camera_skew_s = rospy.get_param("~max_inter_camera_skew_s", 0.005)
        self._cross_sync_data = None
        self._cross_lock = threading.Lock()

        # ── Per-camera tuple 缓存 (color stamp → tuple) ──
        self._tuple_cache = {}  # cam → OrderedDict[stamp_ns, tuple]
        self._max_cache_entries = 15
        self._patch_sync_callbacks()
        self._setup_cross_sync()

        # ── SyncFrameGroup 计数 ──
        self._group_counter = 0
        self.groups_dir = os.path.join(self.run_dir, "groups")
        os.makedirs(self.groups_dir, exist_ok=True)

        rospy.Service("~capture_sync_group", Trigger,
                      self._svc_capture_sync_group)

        rospy.loginfo("JointCaptureManager ready: %d cameras, "
                      "cross_slop=%.3fs, max_inter_cam_skew=%.1fms",
                      len(self.camera_names), self.cross_sync_slop_s,
                      self.max_inter_camera_skew_s * 1000)

    def _patch_sync_callbacks(self):
        """在基类 per-camera sync CB 之后追加缓存写入.

        基类 _make_sync_cb 已经在 __init__ 中注册.
        这里拦截 _sync_data 写入并在每次更新时同步写入 _tuple_cache.
        """
        # 存储原始方法引用
        self._base_make_sync_cb = self._make_sync_cb

        def patched_make_sync_cb(cam):
            base_cb = self._base_make_sync_cb(cam)
            def cb(color_msg, depth_msg, color_info_msg, depth_info_msg):
                base_cb(color_msg, depth_msg, color_info_msg, depth_info_msg)
                # 写入缓存: 按 color timestamp (ns) 索引
                stamp_ns = color_msg.header.stamp.to_nsec()
                with self._sync_lock:
                    if cam not in self._tuple_cache:
                        self._tuple_cache[cam] = {}
                    cache = self._tuple_cache[cam]
                    cache[stamp_ns] = (color_msg, depth_msg,
                                       color_info_msg, depth_info_msg)
                    # 限制缓存大小
                    while len(cache) > self._max_cache_entries:
                        oldest = min(cache.keys())
                        del cache[oldest]
            return cb

        self._make_sync_cb = patched_make_sync_cb

    def _setup_cross_sync(self):
        """创建跨相机的 3-way ApproximateTimeSynchronizer."""
        color_subs = []
        for cam in self.camera_names:
            sub = message_filters.Subscriber(
                "/" + cam + "/camera/color/image_raw", Image)
            color_subs.append(sub)

        self._cross_ats = message_filters.ApproximateTimeSynchronizer(
            color_subs, queue_size=5, slop=self.cross_sync_slop_s,
            allow_headerless=False)
        self._cross_ats.registerCallback(self._cross_sync_cb)

    def _cross_sync_cb(self, *color_msgs):
        """跨相机同步回调 (P0-A 修复版).

        使用 ATS 触发的 color 消息精确时间戳 (ns)
        从 tuple 缓存中查找对应的 RGB-D 四路 tuple.
        不允许模糊匹配.
        """
        with self._cross_lock:
            if not self._capture_active:
                return

            with self._sync_lock:
                # 按精确 color stamp 查找匹配的四路 tuple
                matched_snapshot = {}
                match_stamps = {}
                for i, cam in enumerate(self.camera_names):
                    stamp_ns = color_msgs[i].header.stamp.to_nsec()
                    cache = self._tuple_cache.get(cam, {})
                    matched = cache.get(stamp_ns)
                    if matched is None:
                        return  # 缓存中没有精确匹配, 拒绝
                    # 二次验证: 缓存中的 color 消息必须与 ATS 消息完全一致
                    cached_color = matched[0]
                    cached_stamp_ns = cached_color.header.stamp.to_nsec()
                    if cached_stamp_ns != stamp_ns:
                        return  # 缓存数据不一致, 拒绝
                    matched_snapshot[cam] = matched
                    match_stamps[cam] = color_msgs[i].header.stamp

                # 从实际保存的 color stamp 重新计算 inter-camera skew
                inter_cam_skew = 0.0
                stamps = list(match_stamps.values())
                for i in range(len(stamps)):
                    for j in range(i + 1, len(stamps)):
                        d = abs((stamps[i] - stamps[j]).to_sec())
                        inter_cam_skew = max(inter_cam_skew, d)

                # 硬门限检查 (基于实际保存数据)
                if inter_cam_skew > self.max_inter_camera_skew_s:
                    rospy.logwarn_throttle(
                        5.0,
                        "Cross-camera skew %.1fms > %.1fms (measured from saved data), rejecting",
                        inter_cam_skew * 1000,
                        self.max_inter_camera_skew_s * 1000)
                    return

                snapshot = matched_snapshot
                snapshot["_cross_skew_s"] = inter_cam_skew
                snapshot["_cross_match_method"] = "exact_stamp_ns"
                snapshot["_cross_color_stamps"] = {
                    cam: {"secs": s.secs, "nsecs": s.nsecs}
                    for cam, s in match_stamps.items()
                }

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
            # 清空 tuple 缓存 (避免上一次采集的旧数据)
            self._tuple_cache.clear()

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
        cross_skew = snapshot.get("_cross_skew_s", None)
        cross_stamps = snapshot.get("_cross_color_stamps", {})
        group_manifest = {
            "group_id": group_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "camera_names": self.camera_names,
            "captured": captured,
            "expected": self.expected_camera_count,
            "success": success,
            "errors": errors if errors else [],
            "per_camera": per_camera_detail,
            "cross_camera_sync": {
                "method": "3-way ApproximateTimeSynchronizer + exact stamp lookup",
                "max_inter_camera_skew_s": cross_skew,
                "max_allowed_skew_s": self.max_inter_camera_skew_s,
                "ats_slop_s": self.cross_sync_slop_s,
                "per_camera_color_stamps": cross_stamps,
            },
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

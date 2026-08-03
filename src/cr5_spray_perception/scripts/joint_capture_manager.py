#!/usr/bin/env python3
"""
JointCaptureManager — 跨相机同步采集 (V2: override _make_sync_cb + pending resolve).

在 CaptureManager V3 的 per-camera 4-way sync 基础上,
增加跨相机 3-way ApproximateTimeSynchronizer,
输出 SyncFrameGroup (三台相机同一时刻的 RGB-D 帧组).

架构:
  Stage 1 (已有): 每台相机 4-way ATS → per-camera synced tuple
  Stage 2 (新增): 3-way ATS 跨相机的 synced color → SyncFrameGroup

修复 (P0-1):
  - override _make_sync_cb (而非 monkey-patch): Python MRO 在 super().__init__()
    中注册 callback 时自动分派到子类版本
  - pending cross group 机制: 解决 cross ATS / per-camera tuple ATS 回调时序竞态
  - 精确 stamp 匹配: 禁止最近邻时间戳代替精确绑定
  - manifest 含 stamp_ns: 可审计证据

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

    P0-1 修复: override _make_sync_cb (Python 动态分派).

    - _tuple_cache 在 super().__init__() 之前初始化
    - super().__init__() 中 ts.registerCallback(self._make_sync_cb(cam))
      通过 Python MRO 自动找到 JointCaptureManager._make_sync_cb
    - 不再使用 monkey-patch (_patch_sync_callbacks 已删除)

    精确匹配 + pending resolve:
    - per-camera tuple 写入 _tuple_cache[color_stamp_ns]
    - cross ATS 回调先尝试立即 resolve，失败则记入 _pending_cross_groups
    - per-camera tuple 写入后调用 _try_resolve_cross_group
    - 任何路径 resolve 成功 → 生成 _cross_sync_data
    - 禁止最近邻时间戳代替精确绑定
    """

    def __init__(self):
        # ── 必须在 super().__init__() 之前初始化 ──
        # super().__init__() 会调用 self._make_sync_cb(cam) 注册回调,
        # Python MRO 将分派到本类的 override, 因此 _tuple_cache 等必须已存在.
        self._tuple_cache = {}       # cam → {stamp_ns: (color, depth, cinfo, dinfo)}
        self._max_cache_entries = 15
        self._pending_cross_groups = []  # [{color_stamps, ros_stamps}, ...]
        self._cross_sync_data = None
        self._cross_lock = threading.Lock()

        # ── 调用基类 __init__ ──
        # 基类内部: rospy.init_node → 创建 per-camera 4-way ATS
        # → ts.registerCallback(self._make_sync_cb(cam))
        # → Python 自动使用 JointCaptureManager._make_sync_cb ✓
        super().__init__()

        # ── 跨相机同步参数 (rospy 此时已初始化) ──
        self.cross_sync_slop_s = rospy.get_param("~cross_sync_slop_s", 0.033)
        self.max_inter_camera_skew_s = rospy.get_param(
            "~max_inter_camera_skew_s", 0.005)

        # ── 跨相机 3-way ATS ──
        self._setup_cross_sync()

        # ── SyncFrameGroup 计数 ──
        self._group_counter = 0
        self.groups_dir = os.path.join(self.run_dir, "groups")
        os.makedirs(self.groups_dir, exist_ok=True)

        # ── 服务 ──
        rospy.Service("~capture_sync_group", Trigger,
                      self._svc_capture_sync_group)

        rospy.loginfo("JointCaptureManager ready: %d cameras, "
                      "cross_slop=%.3fs, max_inter_cam_skew=%.1fms",
                      len(self.camera_names), self.cross_sync_slop_s,
                      self.max_inter_camera_skew_s * 1000)

    # ═══════════════════════════════════════════════════════════════
    # _make_sync_cb override (替代 monkey-patch)
    # ═══════════════════════════════════════════════════════════════

    def _make_sync_cb(self, cam):
        """Override 基类方法.

        基类 CaptureManager.__init__ 注册 callback 时,
        Python MRO 自动找到此版本 (动态分派).

        流程:
          1. 调用基类 callback (capture_active 检查 + _sync_data 更新)
          2. 确认消息被采集窗口接受
          3. 写入 _tuple_cache[cam][stamp_ns]
          4. 调用 _try_resolve_cross_group() 尝试匹配等待中的跨相机组
        """
        base_cb = super()._make_sync_cb(cam)

        def cb(color_msg, depth_msg, color_info_msg, depth_info_msg):
            # 1. 基类 callback (含 capture_active / 时间窗口检查)
            base_cb(color_msg, depth_msg, color_info_msg, depth_info_msg)

            # 2. 确认消息被基类接受 (与 _sync_data 中的 tuple 完全一致)
            with self._sync_lock:
                synced = self._sync_data.get(cam)
                if synced is None:
                    return
                # 必须是同一个对象 (不是旧 tuple)
                if synced[0] is not color_msg:
                    return

            # 3. 写入 tuple cache (按精确 color stamp_ns 索引)
            stamp_ns = color_msg.header.stamp.to_nsec()
            with self._sync_lock:
                if cam not in self._tuple_cache:
                    self._tuple_cache[cam] = {}
                self._tuple_cache[cam][stamp_ns] = (
                    color_msg, depth_msg, color_info_msg, depth_info_msg)
                # 限制缓存大小
                cache = self._tuple_cache[cam]
                while len(cache) > self._max_cache_entries:
                    oldest = min(cache.keys())
                    del cache[oldest]

            # 4. 尝试 resolve 等待中的跨相机组
            self._try_resolve_cross_group()

        return cb

    # ═══════════════════════════════════════════════════════════════
    # Cross-camera sync
    # ═══════════════════════════════════════════════════════════════

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
        """跨相机同步回调 (pending resolve 版).

        先尝试立即从 _tuple_cache 精确匹配 3 个 tuple:
          - 全部匹配 + skew ≤ max → 直接生成 _cross_sync_data
          - 任一缺失 → 记录为 pending group, 等待 per-camera callback 触发 resolve
        不允许最近邻时间戳代替精确绑定.
        """
        with self._cross_lock:
            if not self._capture_active:
                return

            # 构建 pending group 数据
            color_stamps = {}
            ros_stamps = {}
            for i, cam in enumerate(self.camera_names):
                stamp_ns = color_msgs[i].header.stamp.to_nsec()
                color_stamps[cam] = stamp_ns
                ros_stamps[cam] = color_msgs[i].header.stamp

            # 尝试立即 resolve
            with self._sync_lock:
                matched_snapshot = {}
                match_stamps = {}

                for cam in self.camera_names:
                    stamp_ns = color_stamps[cam]
                    cache = self._tuple_cache.get(cam, {})
                    tup = cache.get(stamp_ns)
                    if tup is None:
                        break
                    # 二次验证: 缓存中的 color 消息 stamp 必须与 ATS 消息一致
                    if tup[0].header.stamp.to_nsec() != stamp_ns:
                        break
                    matched_snapshot[cam] = tup
                    match_stamps[cam] = ros_stamps[cam]

                if len(matched_snapshot) == len(self.camera_names):
                    # 从实际保存的 color stamp 重新计算 inter-camera skew
                    stamps = list(match_stamps.values())
                    inter_cam_skew = 0.0
                    for i in range(len(stamps)):
                        for j in range(i + 1, len(stamps)):
                            d = abs((stamps[i] - stamps[j]).to_sec())
                            inter_cam_skew = max(inter_cam_skew, d)

                    # 硬门限检查
                    if inter_cam_skew <= self.max_inter_camera_skew_s:
                        snapshot = dict(matched_snapshot)
                        snapshot["_cross_skew_s"] = inter_cam_skew
                        snapshot["_cross_match_method"] = "cross_camera_bounded_skew"
                        snapshot["_cross_color_stamps"] = {
                            c: {
                                "secs": s.secs,
                                "nsecs": s.nsecs,
                                "stamp_ns": s.to_nsec(),
                            }
                            for c, s in match_stamps.items()
                        }
                        self._cross_sync_data = snapshot
                        return  # 立即成功, 不需要 pending

                    else:
                        rospy.logwarn_throttle(
                            5.0,
                            "Cross-camera skew %.1fms > %.1fms, rejecting",
                            inter_cam_skew * 1000,
                            self.max_inter_camera_skew_s * 1000)
                        return  # skew 超限, 不入 pending

            # 不能立即 resolve → 保存为 pending group
            self._pending_cross_groups.append({
                "color_stamps": color_stamps,
                "ros_stamps": ros_stamps,
            })
            # 限制 pending 数量
            while len(self._pending_cross_groups) > 20:
                self._pending_cross_groups.pop(0)

    def _try_resolve_cross_group(self):
        """per-camera tuple 写入后调用, 尝试匹配等待中的跨相机组.

        遍历 _pending_cross_groups, 检查是否所有 3 台相机的 tuple 已到齐.
        对到齐的组进行 skew 验证, 通过则生成 _cross_sync_data.
        """
        with self._cross_lock:
            if not self._capture_active:
                return
            if not self._pending_cross_groups:
                return
            # 已有已 resolve 的 group, 不再重新 resolve
            if self._cross_sync_data is not None:
                return

            resolved_idx = None
            for idx, pending in enumerate(self._pending_cross_groups):
                color_stamps = pending["color_stamps"]
                ros_stamps = pending["ros_stamps"]

                with self._sync_lock:
                    matched = {}
                    match_stamps = {}

                    for cam in self.camera_names:
                        stamp_ns = color_stamps.get(cam)
                        if stamp_ns is None:
                            break
                        cache = self._tuple_cache.get(cam, {})
                        tup = cache.get(stamp_ns)
                        if tup is None:
                            break
                        if tup[0].header.stamp.to_nsec() != stamp_ns:
                            break
                        matched[cam] = tup
                        match_stamps[cam] = ros_stamps[cam]

                    if len(matched) == len(self.camera_names):
                        stamps = list(match_stamps.values())
                        inter_cam_skew = 0.0
                        for i in range(len(stamps)):
                            for j in range(i + 1, len(stamps)):
                                d = abs((stamps[i] - stamps[j]).to_sec())
                                inter_cam_skew = max(inter_cam_skew, d)

                        if inter_cam_skew <= self.max_inter_camera_skew_s:
                            snapshot = dict(matched)
                            snapshot["_cross_skew_s"] = inter_cam_skew
                            snapshot["_cross_match_method"] = "cross_camera_bounded_skew"
                            snapshot["_cross_color_stamps"] = {
                                c: {
                                    "secs": s.secs,
                                    "nsecs": s.nsecs,
                                    "stamp_ns": s.to_nsec(),
                                }
                                for c, s in match_stamps.items()
                            }
                            self._cross_sync_data = snapshot
                            resolved_idx = idx
                            break

            # 清理已 resolve 的 pending group
            if resolved_idx is not None:
                self._pending_cross_groups.pop(resolved_idx)

    # ═══════════════════════════════════════════════════════════════
    # Sync group capture
    # ═══════════════════════════════════════════════════════════════

    def capture_sync_group(self):
        """采集一个跨相机同步帧组.

        Returns: (success: bool, message: str, group_dir: str)
        """
        rospy.loginfo("Capturing sync group (cross-camera 3-way sync)...")

        # ── 激活采集 + 清空旧数据 ──
        with self._sync_lock:
            self._capture_active = True
            self._capture_started_ros = rospy.Time.now()
            for cam in self.camera_names:
                self._sync_data[cam] = None
            # 清空 tuple 缓存 + pending groups (避免上一次采集的旧数据)
            self._tuple_cache.clear()
            self._pending_cross_groups.clear()

        with self._cross_lock:
            self._cross_sync_data = None
            self._pending_cross_groups.clear()

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
                           f"cross={'ready' if cross_ready else 'pending'}")
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
                self._pending_cross_groups.clear()

        # ── 保存帧组 ──
        group_id = self._group_counter
        self._group_counter += 1
        group_dir = os.path.join(self.groups_dir,
                                 "group_{:04d}".format(group_id))
        os.makedirs(group_dir, exist_ok=True)

        captured = 0
        errors = []
        per_camera_detail = {}

        # 从 snapshot 中提取跨相机时间戳 (保存前再次验证)
        cross_stamps_from_snapshot = snapshot.get("_cross_color_stamps", {})
        cross_skew_from_snapshot = snapshot.get("_cross_skew_s", None)

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

        # ── 最终时间戳验证: 从实际保存的三台 quality.yaml 重算 max skew ──
        final_skew = 0.0
        final_stamps = {}
        for cam in self.camera_names:
            if cam in per_camera_detail:
                ts = per_camera_detail[cam].get("color_timestamp", {})
                final_stamps[cam] = ts
        cam_list = list(final_stamps.keys())
        for i in range(len(cam_list)):
            for j in range(i + 1, len(cam_list)):
                ti = final_stamps[cam_list[i]]
                tj = final_stamps[cam_list[j]]
                si = ti.get("secs", 0) + ti.get("nsecs", 0) * 1e-9
                sj = tj.get("secs", 0) + tj.get("nsecs", 0) * 1e-9
                final_skew = max(final_skew, abs(si - sj))

        # ── group manifest ──
        success = (captured == self.expected_camera_count and len(errors) == 0)

        # 如果 final_skew 超限, 强制 FAIL
        if success and final_skew > self.max_inter_camera_skew_s:
            rospy.logerr("Final skew %.1fms > %.1fms — forcing group FAIL",
                         final_skew * 1000, self.max_inter_camera_skew_s * 1000)
            success = False
            if not errors:
                errors.append(
                    "final_inter_camera_skew {:.1f}ms > {:.1f}ms".format(
                        final_skew * 1000, self.max_inter_camera_skew_s * 1000))

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
                "method": "cross_camera_bounded_skew",
                "max_inter_camera_skew_s": final_skew,
                "max_allowed_skew_s": self.max_inter_camera_skew_s,
                "ats_slop_s": self.cross_sync_slop_s,
                "per_camera_color_stamps": {
                    cam: {
                        "secs": ts.get("secs"),
                        "nsecs": ts.get("nsecs"),
                        "stamp_ns": (ts.get("secs", 0) * int(1e9) +
                                     ts.get("nsecs", 0)),
                    }
                    for cam, ts in final_stamps.items()
                },
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

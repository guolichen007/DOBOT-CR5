#!/usr/bin/env python3
"""
CR5 Spray Demo: Multi-Camera RGB-D Capture Manager (V2 — 严格同步).

改进:
- 使用 message_filters.ApproximateTimeSynchronizer 每相机 color+depth+CameraInfo 成组
- 单相机 color-depth skew ≤ 0.030 s
- 三台相机必须 3/3 成功才返回成功
- 增强 manifest (git_sha, YAML SHA256, per-camera timestamp/skew/valid_depth_ratio, TF)
"""
import os
import sys
import time
import json
import hashlib
import subprocess
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


class CaptureManager:
    def __init__(self):
        rospy.init_node("capture_manager")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.run_id = rospy.get_param("~run_id", time.strftime("%Y%m%d_%H%M%S"))
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/cr5_spray_data"))
        self.camera_names = rospy.get_param("~camera_names", [])
        self.timeout_s = rospy.get_param("~timeout_s", 10.0)
        self.min_depth_valid_ratio = rospy.get_param("~min_depth_valid_ratio", 0.3)
        self.depth_min_m = rospy.get_param("~depth_min_m", 0.1)
        self.depth_max_m = rospy.get_param("~depth_max_m", 10.0)
        self.max_skew_s = rospy.get_param("~max_color_depth_skew_s", 0.030)

        if not self.camera_names:
            rospy.logerr("capture_manager: camera_names is empty!")
            sys.exit(1)

        self.expected_camera_count = len(self.camera_names)

        self.run_dir = os.path.join(self.output_dir, self.run_id)
        self.views_dir = os.path.join(self.run_dir, "views")
        self.logs_dir = os.path.join(self.run_dir, "logs")
        for d in [self.run_dir, self.views_dir, self.logs_dir]:
            os.makedirs(d, exist_ok=True)

        # ── 每相机 message_filters 同步器 ──
        # 每组: color/image_raw + depth/image_raw + color/camera_info
        self._sync_data = {}   # cam → latest synchronized tuple
        self._sync_lock = {}   # cam → threading.Lock (not used, single-threaded ok)

        for cam in self.camera_names:
            ns = "/" + cam
            color_sub = message_filters.Subscriber(
                ns + "/camera/color/image_raw", Image)
            depth_sub = message_filters.Subscriber(
                ns + "/camera/depth/image_raw", Image)
            info_sub = message_filters.Subscriber(
                ns + "/camera/color/camera_info", CameraInfo)

            ts = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub, info_sub],
                queue_size=10, slop=0.05,  # 50ms slop for initial grouping
                allow_headerless=False)
            ts.registerCallback(self._make_sync_cb(cam))

            self._sync_data[cam] = None

        # Services
        rospy.Service("~capture_all_fixed", Trigger, self._svc_capture_fixed)
        rospy.Service("~capture_scan_sequence", Trigger, self._svc_capture_scan)

        # Compute config SHA256 for manifest
        self._cfg_sha = self._compute_config_sha()

        # Write initial manifest
        self._write_manifest(0)

        rospy.loginfo("CaptureManager V2 ready: %d cameras, max_skew=%.3fs, "
                      "run_id=%s", len(self.camera_names), self.max_skew_s,
                      self.run_id)

    def _make_sync_cb(self, cam):
        """创建闭包捕获 cam 名称."""
        def cb(color_msg, depth_msg, info_msg):
            self._sync_data[cam] = (color_msg, depth_msg, info_msg)
        return cb

    def _all_synced(self):
        """所有相机都有同步数据."""
        return all(v is not None for v in self._sync_data.values())

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

    def _svc_capture_fixed(self, req):
        ok, msg, path = self.capture_view("fixed")
        return TriggerResponse(success=ok, message=msg)

    def _svc_capture_scan(self, req):
        ok, msg, path = self.capture_view("scan")
        return TriggerResponse(success=ok, message=msg)

    def capture_view(self, mode):
        """严格同步采集所有相机.

        Returns: (success: bool, message: str, view_dir: str)
        success=True 仅当 captured==expected_camera_count 且 errors 为空.
        """
        rospy.loginfo("Capturing view (mode=%s, strict sync)...", mode)

        # 清空旧数据
        for cam in self.camera_names:
            self._sync_data[cam] = None

        t_start = time.time()
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self._all_synced():
                break
            if time.time() - t_start > self.timeout_s:
                ready = sum(1 for v in self._sync_data.values() if v is not None)
                msg = (f"Timeout waiting for synced data: {ready}/"
                       f"{self.expected_camera_count} cameras after "
                       f"{self.timeout_s:.0f}s")
                rospy.logerr(msg)
                return False, msg, ""
            rate.sleep()

        view_id = "view_{:04d}".format(len(os.listdir(self.views_dir)))
        view_dir = os.path.join(self.views_dir, view_id)
        os.makedirs(view_dir, exist_ok=True)

        captured = 0
        errors = []
        per_camera_detail = {}

        for cam in self.camera_names:
            try:
                color_msg, depth_msg, info_msg = self._sync_data[cam]

                cam_dir = os.path.join(view_dir, cam)
                os.makedirs(cam_dir, exist_ok=True)

                img_ts = color_msg.header.stamp
                depth_ts = depth_msg.header.stamp

                # CameraInfo 维度检查
                if info_msg.width != color_msg.width or info_msg.height != color_msg.height:
                    errors.append(f"{cam}: CameraInfo/Image dim mismatch")
                    continue

                # 严格 skew 检查
                skew = abs((depth_ts - img_ts).to_sec())
                if skew > self.max_skew_s:
                    errors.append(
                        f"{cam}: depth skew {skew*1000:.1f}ms > "
                        f"{self.max_skew_s*1000:.0f}ms")
                    continue

                # 保存 color
                color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
                cv2.imwrite(os.path.join(cam_dir, "color.png"),
                            cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))

                # 保存 depth
                depth_img = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough")
                np.save(os.path.join(cam_dir, "depth.npy"), depth_img)

                # Depth 质量
                if depth_img.dtype == np.uint16:
                    depth_m = depth_img.astype(np.float32) / 1000.0
                else:
                    depth_m = depth_img.astype(np.float32)
                finite = np.isfinite(depth_m) & (depth_m > 0)
                in_range = ((depth_m >= self.depth_min_m) &
                            (depth_m <= self.depth_max_m))
                valid = finite & in_range
                valid_ratio = float(np.mean(valid)) if depth_img.size > 0 else 0.0

                quality = {
                    "valid_depth_ratio": valid_ratio,
                    "depth_min_m": float(depth_m[valid].min()) if np.any(valid) else 0,
                    "depth_max_m": float(depth_m[valid].max()) if np.any(valid) else 0,
                    "depth_zeros": int(np.sum(~finite)),
                    "color_timestamp": {"secs": img_ts.secs, "nsecs": img_ts.nsecs},
                    "depth_timestamp": {"secs": depth_ts.secs, "nsecs": depth_ts.nsecs},
                    "timestamp_skew_s": float(skew),
                }
                with open(os.path.join(cam_dir, "quality.yaml"), "w") as f:
                    yaml.dump(quality, f)

                if valid_ratio < self.min_depth_valid_ratio:
                    errors.append(
                        f"{cam}: low depth valid {valid_ratio:.1%} "
                        f"< {self.min_depth_valid_ratio:.0%}")

                # 保存 CameraInfo
                with open(os.path.join(cam_dir, "color_camera_info.yaml"), "w") as f:
                    yaml.dump(self._cinfo_dict(info_msg), f)

                # TF 查找
                try:
                    trans = self.tf_buffer.lookup_transform(
                        "world", color_msg.header.frame_id, img_ts,
                        rospy.Duration(1.0))
                    tf_dict = {
                        "translation": {"x": trans.transform.translation.x,
                                        "y": trans.transform.translation.y,
                                        "z": trans.transform.translation.z},
                        "rotation": {"x": trans.transform.rotation.x,
                                     "y": trans.transform.rotation.y,
                                     "z": trans.transform.rotation.z,
                                     "w": trans.transform.rotation.w},
                    }
                    with open(os.path.join(cam_dir, "T_world_camera.yaml"), "w") as f:
                        yaml.dump(tf_dict, f)
                    quality["tf_frame"] = "world"
                except Exception as e:
                    errors.append(f"{cam}: TF failed - {e}")
                    quality["tf_frame"] = "unavailable"

                per_camera_detail[cam] = quality
                captured += 1

            except Exception as e:
                errors.append(f"{cam}: {e}")

        # ── 严格 3/3 ──
        success = (captured == self.expected_camera_count and len(errors) == 0)

        # 写入 manifest
        self._write_manifest(captured, per_camera_detail, errors)

        elapsed = time.time() - t_start
        max_skew = 0.0
        if per_camera_detail:
            max_skew = max(
                d.get("timestamp_skew_s", 0) for d in per_camera_detail.values())

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
            print(f"max_color_depth_skew_s <= {self.max_skew_s}")
        else:
            rospy.logerr("SYNC_CAPTURE_%d_OF_%d_FAIL", captured,
                         self.expected_camera_count)

        return success, msg, view_dir

    def _write_manifest(self, captured_count, per_camera_detail=None, errors=None):
        """Write/update manifest.yaml (增强版)."""
        git_sha = "unknown"
        try:
            git_sha = subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            pass

        manifest = {
            "run_id": self.run_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "git_sha": git_sha,
            "ros_distro": os.environ.get("ROS_DISTRO", "noetic"),
            "expected_camera_count": self.expected_camera_count,
            "camera_names": self.camera_names,
            "views_captured": captured_count,
            "output_dir": self.run_dir,
            "max_color_depth_skew_s": self.max_skew_s,
            "sync_method": "message_filters.ApproximateTimeSynchronizer",
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
                    "timestamp_skew_s": detail.get("timestamp_skew_s"),
                    "valid_depth_ratio": detail.get("valid_depth_ratio"),
                    "tf_frame": detail.get("tf_frame", "N/A"),
                }

        if errors:
            manifest["errors"] = errors

        with open(os.path.join(self.run_dir, "manifest.yaml"), "w") as f:
            yaml.dump(manifest, f, default_flow_style=False)

    @staticmethod
    def _cinfo_dict(msg):
        return {
            "height": msg.height, "width": msg.width,
            "K": list(msg.K), "P": list(msg.P),
            "distortion_model": msg.distortion_model,
            "D": list(msg.D),
        }

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    CaptureManager().run()

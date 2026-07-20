#!/usr/bin/env python3
"""
CR5 Spray Demo: Multi-Camera RGB-D Capture Manager
支持任意数量相机。同步采集 color+depth+CameraInfo，保存 manifest。
"""
import os
import sys
import time
import json
import subprocess
import yaml
import numpy as np
import rospy
import tf2_ros
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge


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
        self.timeout_s = rospy.get_param("~timeout_s", 5.0)
        self.min_depth_valid_ratio = rospy.get_param("~min_depth_valid_ratio", 0.3)
        self.depth_min_m = rospy.get_param("~depth_min_m", 0.1)
        self.depth_max_m = rospy.get_param("~depth_max_m", 10.0)

        if not self.camera_names:
            rospy.logerr("capture_manager: camera_names is empty!")
            sys.exit(1)

        self.run_dir = os.path.join(self.output_dir, self.run_id)
        self.views_dir = os.path.join(self.run_dir, "views")
        self.logs_dir = os.path.join(self.run_dir, "logs")
        for d in [self.run_dir, self.views_dir, self.logs_dir]:
            os.makedirs(d, exist_ok=True)

        # Per-camera latest data (paired by timestamp)
        self.latest = {}
        for cam in self.camera_names:
            ns = "/" + cam
            self.latest[cam] = {"color": None, "depth": None,
                                "color_info": None, "depth_info": None}
            rospy.Subscriber(ns + "/camera/color/image_raw", Image,
                             lambda m, c=cam: self._cb(c, "color", m))
            rospy.Subscriber(ns + "/camera/color/camera_info", CameraInfo,
                             lambda m, c=cam: self._cb(c, "color_info", m))
            rospy.Subscriber(ns + "/camera/depth/image_raw", Image,
                             lambda m, c=cam: self._cb(c, "depth", m))
            rospy.Subscriber(ns + "/camera/depth/camera_info", CameraInfo,
                             lambda m, c=cam: self._cb(c, "depth_info", m))

        # Services (use std_srvs/Trigger)
        rospy.Service("~capture_all_fixed", Trigger, self._svc_capture_fixed)
        rospy.Service("~capture_scan_sequence", Trigger, self._svc_capture_scan)

        # Write initial manifest
        self._write_manifest(0)

        rospy.loginfo("CaptureManager ready: %d cameras, run_id=%s",
                      len(self.camera_names), self.run_id)

    def _cb(self, cam, key, msg):
        self.latest[cam][key] = msg

    def _all_ready(self):
        """Check all cameras have all 4 message types."""
        return all(
            v is not None
            for cam in self.camera_names
            for v in self.latest[cam].values())

    def _svc_capture_fixed(self, req):
        ok, msg, path = self.capture_view("fixed")
        return TriggerResponse(success=ok, message=msg)

    def _svc_capture_scan(self, req):
        ok, msg, path = self.capture_view("scan")
        return TriggerResponse(success=ok, message=msg)

    def capture_view(self, mode):
        """Capture all cameras synchronously."""
        rospy.loginfo("Capturing view (mode=%s)...", mode)
        t_start = time.time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self._all_ready():
                break
            if time.time() - t_start > self.timeout_s:
                msg = "Timeout waiting for camera data after {}s".format(
                    self.timeout_s)
                rospy.logerr(msg)
                return False, msg, ""
            rate.sleep()

        view_id = "view_{:04d}".format(len(os.listdir(self.views_dir)))
        view_dir = os.path.join(self.views_dir, view_id)
        os.makedirs(view_dir, exist_ok=True)

        captured = 0
        errors = []

        for cam in self.camera_names:
            try:
                cam_dir = os.path.join(view_dir, cam)
                os.makedirs(cam_dir, exist_ok=True)

                color_msg = self.latest[cam]["color"]
                depth_msg = self.latest[cam]["depth"]
                cinfo = self.latest[cam]["color_info"]
                dinfo = self.latest[cam]["depth_info"]

                # Use image timestamps for TF lookup
                img_ts = color_msg.header.stamp

                # Validate CameraInfo matches image dimensions
                if cinfo.width != color_msg.width or cinfo.height != color_msg.height:
                    errors.append("{}: CameraInfo/Image dim mismatch".format(cam))
                    continue

                # Quality: timestamp skew
                skew = abs((depth_msg.header.stamp - img_ts).to_sec())
                if skew > 0.5:
                    errors.append("{}: depth skew {:.2f}s".format(cam, skew))
                    continue

                # Save color
                color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
                cv2.imwrite(os.path.join(cam_dir, "color.png"),
                            cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))

                # Save depth
                depth_img = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding="passthrough")
                np.save(os.path.join(cam_dir, "depth.npy"), depth_img)

                # Depth quality check
                if depth_img.dtype == np.uint16:
                    depth_m = depth_img.astype(np.float32) / 1000.0
                else:
                    depth_m = depth_img.astype(np.float32)
                finite = np.isfinite(depth_m) & (depth_m > 0)
                in_range = (depth_m >= self.depth_min_m) & (depth_m <= self.depth_max_m)
                valid = finite & in_range
                valid_ratio = np.mean(valid) if depth_img.size > 0 else 0.0

                quality = {
                    "valid_depth_ratio": float(valid_ratio),
                    "depth_min_m": float(depth_m[valid].min()) if np.any(valid) else 0,
                    "depth_max_m": float(depth_m[valid].max()) if np.any(valid) else 0,
                    "depth_zeros": int(np.sum(~finite)),
                    "timestamp_skew_s": float(skew),
                }
                with open(os.path.join(cam_dir, "quality.yaml"), "w") as f:
                    yaml.dump(quality, f)

                if valid_ratio < self.min_depth_valid_ratio:
                    errors.append("{}: low depth valid {:.1%}".format(
                        cam, valid_ratio))
                    continue

                # Save CameraInfo
                with open(os.path.join(cam_dir, "color_camera_info.yaml"), "w") as f:
                    yaml.dump(self._cinfo_dict(cinfo), f)
                with open(os.path.join(cam_dir, "depth_camera_info.yaml"), "w") as f:
                    yaml.dump(self._cinfo_dict(dinfo), f)

                # TF lookup using image timestamp
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
                except Exception as e:
                    errors.append("{}: TF failed - {}".format(cam, e))

                captured += 1

            except Exception as e:
                errors.append("{}: {}".format(cam, e))

        # Update manifest
        self._write_manifest(captured)

        elapsed = time.time() - t_start
        msg = "Captured {}/{} cameras in {:.1f}s".format(
            captured, len(self.camera_names), elapsed)
        if errors:
            msg += ". Errors: " + "; ".join(errors[:3])
        rospy.loginfo(msg)

        return (captured > 0), msg, view_dir

    def _write_manifest(self, captured_count):
        """Write/update manifest.yaml."""
        git_sha = "unknown"
        try:
            git_sha = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            pass

        manifest = {
            "run_id": self.run_id,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "git_sha": git_sha,
            "ros_distro": os.environ.get("ROS_DISTRO", "noetic"),
            "camera_count": len(self.camera_names),
            "camera_names": self.camera_names,
            "views_captured": captured_count,
            "output_dir": self.run_dir,
        }
        with open(os.path.join(self.run_dir, "manifest.yaml"), "w") as f:
            yaml.dump(manifest, f)

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

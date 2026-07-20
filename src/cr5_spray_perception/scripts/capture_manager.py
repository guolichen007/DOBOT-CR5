#!/usr/bin/env python3
"""
CR5 Spray Demo: Multi-Camera RGB-D Capture Manager
支持任意数量相机；不硬编码 cam0/cam1。
每个 view 保存 color、depth (npy)、camera_info (yaml)、TF。
"""
import os
import sys
import time
import yaml
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CaptureManager:
    def __init__(self):
        rospy.init_node("capture_manager")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Config
        self.run_id = rospy.get_param("~run_id", time.strftime("%Y%m%d_%H%M%S"))
        self.output_dir = rospy.get_param("~output_dir",
            os.path.expanduser("~/cr5_spray_data"))
        self.camera_names = rospy.get_param("~camera_names", [])
        self.timeout_s = rospy.get_param("~timeout_s", 5.0)
        self.min_depth_valid_ratio = rospy.get_param("~min_depth_valid_ratio", 0.3)

        if not self.camera_names:
            rospy.logerr("capture_manager: camera_names is empty!")
            sys.exit(1)

        # Create output structure
        self.run_dir = os.path.join(self.output_dir, self.run_id)
        self.views_dir = os.path.join(self.run_dir, "views")
        self.logs_dir = os.path.join(self.run_dir, "logs")
        for d in [self.run_dir, self.views_dir, self.logs_dir]:
            os.makedirs(d, exist_ok=True)

        # Subscribers (one per camera)
        self.subs = {}
        self.latest_color = {}
        self.latest_depth = {}
        self.latest_color_info = {}
        self.latest_depth_info = {}

        for cam in self.camera_names:
            ns = "/" + cam
            self.subs[cam] = {
                "color": rospy.Subscriber(
                    ns + "/camera/color/image_raw", Image,
                    lambda msg, c=cam: self._color_cb(c, msg)),
                "color_info": rospy.Subscriber(
                    ns + "/camera/color/camera_info", CameraInfo,
                    lambda msg, c=cam: self._color_info_cb(c, msg)),
                "depth": rospy.Subscriber(
                    ns + "/camera/depth/image_raw", Image,
                    lambda msg, c=cam: self._depth_cb(c, msg)),
                "depth_info": rospy.Subscriber(
                    ns + "/camera/depth/camera_info", CameraInfo,
                    lambda msg, c=cam: self._depth_info_cb(c, msg)),
            }

        # Services
        rospy.Service("~capture_all_fixed", rospy.srv.Empty,
                      lambda req: self._capture_service("fixed", req))
        rospy.Service("~capture_wrist_view", rospy.srv.Empty,
                      lambda req: self._capture_service("wrist", req))
        rospy.Service("~capture_scan_sequence", rospy.srv.Empty,
                      lambda req: self._capture_service("scan", req))

        rospy.loginfo("CaptureManager ready: %d cameras, run_id=%s",
                      len(self.camera_names), self.run_id)

    def _color_cb(self, cam, msg): self.latest_color[cam] = msg
    def _color_info_cb(self, cam, msg): self.latest_color_info[cam] = msg
    def _depth_cb(self, cam, msg): self.latest_depth[cam] = msg
    def _depth_info_cb(self, cam, msg): self.latest_depth_info[cam] = msg

    def _capture_service(self, mode, req):
        view_id = "view_{}".format(len(os.listdir(self.views_dir)))
        success = self.capture_view(view_id, mode)
        return rospy.srv.EmptyResponse()

    def capture_view(self, view_id, mode):
        """Capture one view: all cameras at current moment."""
        rospy.loginfo("Capturing view '%s' (mode=%s)...", view_id, mode)
        t_start = time.time()

        # Wait for data from all cameras
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ready = all(
                cam in self.latest_color and cam in self.latest_depth
                for cam in self.camera_names
            )
            if ready:
                break
            if time.time() - t_start > self.timeout_s:
                rospy.logerr("Timeout waiting for camera data")
                return False
            rate.sleep()

        ts = rospy.Time.now()
        view_dir = os.path.join(self.views_dir, view_id)
        os.makedirs(view_dir, exist_ok=True)

        # Save per-camera data
        for cam in self.camera_names:
            cam_dir = os.path.join(view_dir, cam)
            os.makedirs(cam_dir, exist_ok=True)

            color_msg = self.latest_color[cam]
            depth_msg = self.latest_depth[cam]
            cinfo_msg = self.latest_color_info[cam]
            dinfo_msg = self.latest_depth_info[cam]

            # Save color as PNG
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
            import cv2
            cv2.imwrite(os.path.join(cam_dir, "color.png"),
                        cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR))

            # Save depth as NPY (16UC1 or 32FC1)
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg,
                desired_encoding="passthrough")
            np.save(os.path.join(cam_dir, "depth.npy"), depth_img)

            # Save CameraInfo as YAML
            ci_dict = self._camera_info_to_dict(cinfo_msg)
            with open(os.path.join(cam_dir, "color_camera_info.yaml"), "w") as f:
                yaml.dump(ci_dict, f)

            di_dict = self._camera_info_to_dict(dinfo_msg)
            with open(os.path.join(cam_dir, "depth_camera_info.yaml"), "w") as f:
                yaml.dump(di_dict, f)

            # Try to get TF T_world_camera
            try:
                trans = self.tf_buffer.lookup_transform(
                    "world", color_msg.header.frame_id, ts,
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
                rospy.logwarn("TF lookup failed for %s: %s", cam, e)

            # Quality check
            n_valid = np.count_nonzero(depth_img) if depth_img.dtype == np.uint16 \
                else np.count_nonzero(np.isfinite(depth_img))
            n_total = depth_img.size
            valid_ratio = n_valid / max(n_total, 1)
            quality = {"valid_depth_ratio": float(valid_ratio)}
            with open(os.path.join(cam_dir, "quality.yaml"), "w") as f:
                yaml.dump(quality, f)

        rospy.loginfo("View '%s' captured: %d cameras, %.1fs",
                      view_id, len(self.camera_names), time.time() - t_start)
        return True

    @staticmethod
    def _camera_info_to_dict(msg):
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

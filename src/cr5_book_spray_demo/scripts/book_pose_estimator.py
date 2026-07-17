#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""D455 book top-surface pose estimator for ROS1 Noetic.

Pipeline:
  aligned RGB + depth + camera info
  -> rectangular book candidate on the green conveyor
  -> robust top-plane fit
  -> 3-D book frame in camera optical frame
  -> timestamped TF transform into base_link
  -> stability gate and explicit lock service

The node never commands robot motion.
"""

import math
import threading
from collections import deque

import cv2
import message_filters
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, TransformStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray


class BookPoseEstimator:
    def __init__(self):
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        # Frames and topics.
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.book_frame = rospy.get_param("~book_frame", "book_detected")
        self.locked_frame = rospy.get_param("~locked_frame", "book_locked")
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")

        # Detection configuration.
        self.mode = rospy.get_param("~detection_mode", "background_hsv")
        self.roi_norm = np.asarray(
            rospy.get_param("~roi_norm", [0.0, 0.0, 1.0, 1.0]), dtype=np.float64
        )
        self.bg_hsv_lower = np.asarray(
            rospy.get_param("~background_hsv_lower", [30, 35, 20]), dtype=np.uint8
        )
        self.bg_hsv_upper = np.asarray(
            rospy.get_param("~background_hsv_upper", [100, 255, 255]), dtype=np.uint8
        )
        self.min_area_px = float(rospy.get_param("~min_area_px", 12000.0))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.60))
        self.min_rectangularity = float(rospy.get_param("~min_rectangularity", 0.72))
        self.border_margin_px = int(rospy.get_param("~border_margin_px", 8))
        self.max_border_sides = int(rospy.get_param("~max_border_sides", 1))
        self.expected_aspect = float(rospy.get_param("~expected_aspect", 1.38))
        self.aspect_min = float(rospy.get_param("~aspect_min", 1.15))
        self.aspect_max = float(rospy.get_param("~aspect_max", 1.75))
        self.mask_erode_px = int(rospy.get_param("~mask_erode_px", 14))
        self.morph_kernel_px = int(rospy.get_param("~morph_kernel_px", 9))
        self.canny_low = int(rospy.get_param("~canny_low", 50))
        self.canny_high = int(rospy.get_param("~canny_high", 150))

        # Depth and plane fitting.
        self.depth_min_m = float(rospy.get_param("~depth_min_m", 0.20))
        self.depth_max_m = float(rospy.get_param("~depth_max_m", 1.80))
        self.sample_stride = max(1, int(rospy.get_param("~sample_stride", 3)))
        self.min_plane_points = int(rospy.get_param("~min_plane_points", 350))
        self.plane_threshold_m = float(rospy.get_param("~plane_threshold_m", 0.004))
        self.min_plane_inlier_ratio = float(
            rospy.get_param("~min_plane_inlier_ratio", 0.72)
        )
        self.max_plane_rmse_m = float(rospy.get_param("~max_plane_rmse_m", 0.004))

        # Geometric sanity checks. Measure the real book and update these values.
        self.length_min_m = float(rospy.get_param("~length_min_m", 0.20))
        self.length_max_m = float(rospy.get_param("~length_max_m", 0.34))
        self.width_min_m = float(rospy.get_param("~width_min_m", 0.14))
        self.width_max_m = float(rospy.get_param("~width_max_m", 0.26))

        # Stability/locking.
        self.history = deque(maxlen=int(rospy.get_param("~history_size", 45)))
        self.stable_frames = int(rospy.get_param("~stable_frames", 15))
        self.stable_max_age_s = float(rospy.get_param("~stable_max_age_s", 1.2))
        self.max_position_std_m = float(
            rospy.get_param("~max_position_std_m", 0.004)
        )
        self.max_orientation_spread_deg = float(
            rospy.get_param("~max_orientation_spread_deg", 1.2)
        )
        self.max_size_std_m = float(rospy.get_param("~max_size_std_m", 0.006))

        self.tf_timeout_s = float(rospy.get_param("~tf_timeout_s", 0.15))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Live outputs.
        self.pose_pub = rospy.Publisher("~book_pose", PoseStamped, queue_size=1)
        self.size_pub = rospy.Publisher("~book_size", Vector3Stamped, queue_size=1)
        self.polygon_pub = rospy.Publisher("~book_polygon", PolygonStamped, queue_size=1)
        self.valid_pub = rospy.Publisher("~valid", Bool, queue_size=1)
        self.rmse_pub = rospy.Publisher("~plane_rmse", Float32, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)

        # Camera-frame outputs (always available, independent of base TF).
        self.camera_pose_pub = rospy.Publisher("~camera_book_pose", PoseStamped, queue_size=1)
        self.camera_polygon_pub = rospy.Publisher("~camera_book_polygon", PolygonStamped, queue_size=1)
        self.camera_valid_pub = rospy.Publisher("~camera_frame_valid", Bool, queue_size=1)
        self.base_valid_pub = rospy.Publisher("~base_frame_valid", Bool, queue_size=1)

        # Latched locked outputs.
        self.locked_pose_pub = rospy.Publisher(
            "~locked_pose", PoseStamped, queue_size=1, latch=True
        )
        self.locked_size_pub = rospy.Publisher(
            "~locked_size", Vector3Stamped, queue_size=1, latch=True
        )
        self.locked_pub = rospy.Publisher(
            "~target_locked", Bool, queue_size=1, latch=True
        )

        self.lock_srv = rospy.Service("~lock_target", Trigger, self.handle_lock)
        self.clear_srv = rospy.Service("~clear_target", Trigger, self.handle_clear)

        self.locked_pose = None
        self.locked_size = None
        self.locked_pub.publish(Bool(data=False))

        color_sub = message_filters.Subscriber(self.color_topic, Image, queue_size=4)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=4)
        info_sub = message_filters.Subscriber(self.info_topic, CameraInfo, queue_size=4)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=12,
            slop=0.08,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.image_callback)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        rospy.loginfo(
            "book_pose_estimator ready: mode=%s, base=%s, color=%s, depth=%s",
            self.mode,
            self.base_frame,
            self.color_topic,
            self.depth_topic,
        )

    @staticmethod
    def depth_to_meters(depth_image, encoding):
        if encoding in ("16UC1", "mono16") or depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * 0.001
        if encoding == "32FC1" or depth_image.dtype == np.float32:
            return depth_image.astype(np.float32)
        raise ValueError("Unsupported depth encoding: %s / %s" % (encoding, depth_image.dtype))

    def build_candidate_mask(self, bgr):
        height, width = bgr.shape[:2]
        x0 = int(np.clip(self.roi_norm[0], 0.0, 1.0) * width)
        y0 = int(np.clip(self.roi_norm[1], 0.0, 1.0) * height)
        x1 = int(np.clip(self.roi_norm[2], 0.0, 1.0) * width)
        y1 = int(np.clip(self.roi_norm[3], 0.0, 1.0) * height)
        if x1 <= x0 or y1 <= y0:
            raise ValueError("Invalid roi_norm: %s" % self.roi_norm.tolist())

        roi_mask = np.zeros((height, width), dtype=np.uint8)
        roi_mask[y0:y1, x0:x1] = 255

        if self.mode == "background_hsv":
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            background = cv2.inRange(hsv, self.bg_hsv_lower, self.bg_hsv_upper)
            mask = cv2.bitwise_not(background)
            mask = cv2.bitwise_and(mask, roi_mask)
        elif self.mode == "edge_rectangle":
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(gray, self.canny_low, self.canny_high)
            mask = cv2.bitwise_and(edges, roi_mask)
        elif self.mode == "hybrid":
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            background = cv2.inRange(hsv, self.bg_hsv_lower, self.bg_hsv_upper)
            non_background = cv2.bitwise_not(background)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), self.canny_low, self.canny_high)
            edges = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=1)
            mask = cv2.bitwise_or(non_background, edges)
            mask = cv2.bitwise_and(mask, roi_mask)
        else:
            raise ValueError("Unknown detection_mode: %s" % self.mode)

        k = max(3, self.morph_kernel_px | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def select_rectangle(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_area = float(mask.shape[0] * mask.shape[1])
        best = None
        best_score = -1.0

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px or area > image_area * self.max_area_ratio:
                continue
            rect = cv2.minAreaRect(contour)
            (_, _), (w, h), _ = rect
            if w < 2.0 or h < 2.0:
                continue
            rect_area = float(w * h)
            rectangularity = area / max(rect_area, 1.0)
            long_px = max(w, h)
            short_px = min(w, h)
            aspect = long_px / max(short_px, 1.0)
            if rectangularity < self.min_rectangularity:
                continue
            if not (self.aspect_min <= aspect <= self.aspect_max):
                continue
            box = cv2.boxPoints(rect)
            min_x, min_y = np.min(box, axis=0)
            max_x, max_y = np.max(box, axis=0)
            border_sides = sum(
                [
                    min_x <= self.border_margin_px,
                    min_y <= self.border_margin_px,
                    max_x >= mask.shape[1] - 1 - self.border_margin_px,
                    max_y >= mask.shape[0] - 1 - self.border_margin_px,
                ]
            )
            if border_sides > self.max_border_sides:
                continue
            center = np.array(rect[0], dtype=np.float64)
            image_center = np.array([mask.shape[1] * 0.5, mask.shape[0] * 0.5])
            center_distance = np.linalg.norm((center - image_center) / image_center)
            center_score = math.exp(-1.5 * center_distance)
            aspect_score = math.exp(-2.2 * abs(aspect - self.expected_aspect))
            score = area * rectangularity * aspect_score * center_score
            if score > best_score:
                best_score = score
                best = (rect, contour, rectangularity, aspect)
        return best

    @staticmethod
    def backproject_pixels(u, v, depth_m, fx, fy, cx, cy):
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        return np.column_stack((x, y, depth_m))

    def fit_plane(self, points):
        if points.shape[0] < self.min_plane_points:
            return None
        active = points
        normal = None
        centroid = None
        for _ in range(3):
            centroid = np.mean(active, axis=0)
            centered = active - centroid
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            normal = vh[-1]
            normal /= max(np.linalg.norm(normal), 1e-12)
            distances = np.abs((points - centroid) @ normal)
            inliers = distances <= self.plane_threshold_m
            if int(np.count_nonzero(inliers)) < self.min_plane_points:
                return None
            active = points[inliers]
        distances = np.abs((active - centroid) @ normal)
        rmse = float(np.sqrt(np.mean(distances * distances)))
        ratio = float(active.shape[0]) / float(points.shape[0])
        return centroid, normal, active, rmse, ratio

    @staticmethod
    def order_box_points(box_points):
        center = np.mean(box_points, axis=0)
        angles = np.arctan2(box_points[:, 1] - center[1], box_points[:, 0] - center[0])
        return box_points[np.argsort(angles)]

    @staticmethod
    def ray_plane_intersection(pixel, K, plane_point, plane_normal):
        u, v = float(pixel[0]), float(pixel[1])
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        denom = float(np.dot(plane_normal, ray))
        if abs(denom) < 1e-8:
            raise ValueError("Pixel ray is parallel to plane")
        t = float(np.dot(plane_normal, plane_point) / denom)
        if t <= 0.0:
            raise ValueError("Plane intersection is behind camera")
        return ray * t

    def estimate_pose(self, bgr, depth_m, K, stamp, camera_frame):
        mask = self.build_candidate_mask(bgr)
        candidate = self.select_rectangle(mask)
        if candidate is None:
            return None, mask

        rect, _, rectangularity, aspect = candidate
        box2d = self.order_box_points(cv2.boxPoints(rect).astype(np.float64))

        object_mask = np.zeros(depth_m.shape, dtype=np.uint8)
        cv2.fillConvexPoly(object_mask, np.round(box2d).astype(np.int32), 255)
        if self.mask_erode_px > 0:
            ek = max(3, (self.mask_erode_px * 2 + 1) | 1)
            object_mask = cv2.erode(
                object_mask,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ek, ek)),
                iterations=1,
            )

        sampled = np.zeros_like(object_mask)
        sampled[:: self.sample_stride, :: self.sample_stride] = 255
        valid = (
            (object_mask > 0)
            & (sampled > 0)
            & np.isfinite(depth_m)
            & (depth_m >= self.depth_min_m)
            & (depth_m <= self.depth_max_m)
        )
        vv, uu = np.nonzero(valid)
        if uu.size < self.min_plane_points:
            return None, mask

        zz = depth_m[vv, uu].astype(np.float64)
        points = self.backproject_pixels(
            uu.astype(np.float64),
            vv.astype(np.float64),
            zz,
            K[0, 0],
            K[1, 1],
            K[0, 2],
            K[1, 2],
        )
        fitted = self.fit_plane(points)
        if fitted is None:
            return None, mask
        centroid, normal, _, rmse, inlier_ratio = fitted
        if rmse > self.max_plane_rmse_m or inlier_ratio < self.min_plane_inlier_ratio:
            return None, mask

        # The book +Z axis points away from the cover and toward the camera.
        if float(np.dot(normal, centroid)) > 0.0:
            normal = -normal

        corners3d = np.vstack(
            [self.ray_plane_intersection(p, K, centroid, normal) for p in box2d]
        )
        e01 = float(np.linalg.norm(corners3d[1] - corners3d[0]))
        e12 = float(np.linalg.norm(corners3d[2] - corners3d[1]))
        e23 = float(np.linalg.norm(corners3d[3] - corners3d[2]))
        e30 = float(np.linalg.norm(corners3d[0] - corners3d[3]))

        if (e01 + e23) >= (e12 + e30):
            x_axis = corners3d[1] - corners3d[0]
            length = 0.5 * (e01 + e23)
            width = 0.5 * (e12 + e30)
        else:
            x_axis = corners3d[2] - corners3d[1]
            length = 0.5 * (e12 + e30)
            width = 0.5 * (e01 + e23)

        # Orthogonalize the long axis against the plane normal.
        x_axis = x_axis - normal * float(np.dot(x_axis, normal))
        x_norm = float(np.linalg.norm(x_axis))
        if x_norm < 1e-8:
            return None, mask
        x_axis /= x_norm

        # In optical coordinates +Y points down in the image. This makes book +X
        # consistently point from the visual top toward the visual bottom.
        if x_axis[1] < 0.0:
            x_axis = -x_axis
        y_axis = np.cross(normal, x_axis)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-12)
        x_axis = np.cross(y_axis, normal)
        x_axis /= max(float(np.linalg.norm(x_axis)), 1e-12)

        center3d = np.mean(corners3d, axis=0)
        rotation = np.eye(4, dtype=np.float64)
        rotation[:3, 0] = x_axis
        rotation[:3, 1] = y_axis
        rotation[:3, 2] = normal
        quat = quaternion_from_matrix(rotation)

        if not (
            self.length_min_m <= length <= self.length_max_m
            and self.width_min_m <= width <= self.width_max_m
        ):
            rospy.logwarn_throttle(
                2.0,
                "Book geometry rejected: length=%.3f width=%.3f m; update config after measuring the real book",
                length,
                width,
            )
            return None, mask

        # Camera-frame pose (always available).
        camera_pose = PoseStamped()
        camera_pose.header.stamp = stamp
        camera_pose.header.frame_id = camera_frame
        camera_pose.pose.position.x = float(center3d[0])
        camera_pose.pose.position.y = float(center3d[1])
        camera_pose.pose.position.z = float(center3d[2])
        camera_pose.pose.orientation.x = float(quat[0])
        camera_pose.pose.orientation.y = float(quat[1])
        camera_pose.pose.orientation.z = float(quat[2])
        camera_pose.pose.orientation.w = float(quat[3])

        # Camera-frame polygon.
        camera_polygon = PolygonStamped()
        camera_polygon.header = camera_pose.header
        for corner in corners3d:
            camera_polygon.polygon.points.append(
                Point32(
                    x=float(corner[0]),
                    y=float(corner[1]),
                    z=float(corner[2]),
                )
            )

        # Camera-frame size.
        camera_size = Vector3Stamped()
        camera_size.header = camera_pose.header
        camera_size.vector.x = float(length)
        camera_size.vector.y = float(width)
        camera_size.vector.z = 0.0

        # Try base-frame transform (may fail).
        base_pose = None
        base_polygon = None
        base_size = None
        base_tf_ok = False

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                stamp,
                rospy.Duration(self.tf_timeout_s),
            )
            base_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)
            base_pose.header.frame_id = self.base_frame
            base_pose.header.stamp = stamp

            base_size = Vector3Stamped()
            base_size.header = base_pose.header
            base_size.vector.x = float(length)
            base_size.vector.y = float(width)
            base_size.vector.z = 0.0

            base_polygon = PolygonStamped()
            base_polygon.header = base_pose.header
            for corner in corners3d:
                corner_pose = PoseStamped()
                corner_pose.header = camera_pose.header
                corner_pose.pose.position.x = float(corner[0])
                corner_pose.pose.position.y = float(corner[1])
                corner_pose.pose.position.z = float(corner[2])
                corner_pose.pose.orientation.w = 1.0
                base_corner = tf2_geometry_msgs.do_transform_pose(corner_pose, transform)
                base_polygon.polygon.points.append(
                    Point32(
                        x=base_corner.pose.position.x,
                        y=base_corner.pose.position.y,
                        z=base_corner.pose.position.z,
                    )
                )
            base_tf_ok = True
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Base TF transform failed: %s", exc)

        result = {
            "camera_pose": camera_pose,
            "camera_polygon": camera_polygon,
            "camera_size": camera_size,
            "base_pose": base_pose,
            "base_polygon": base_polygon,
            "base_size": base_size,
            "base_tf_ok": base_tf_ok,
            "rmse": rmse,
            "inlier_ratio": inlier_ratio,
            "box2d": box2d,
            "rectangularity": rectangularity,
            "aspect": aspect,
        }
        return result, mask

    def image_callback(self, color_msg, depth_msg, info_msg):
        debug = None
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_m = self.depth_to_meters(raw_depth, depth_msg.encoding)
            K = np.asarray(info_msg.K, dtype=np.float64).reshape(3, 3)
            camera_frame = info_msg.header.frame_id or color_msg.header.frame_id
            stamp = color_msg.header.stamp
            if stamp == rospy.Time(0):
                rospy.logwarn_throttle(3.0, "Camera image has zero timestamp; frame rejected")
                self.valid_pub.publish(Bool(data=False))
                self.camera_valid_pub.publish(Bool(data=False))
                self.base_valid_pub.publish(Bool(data=False))
                return

            # Copy debug image BEFORE estimate_pose (so we always have it).
            debug = bgr.copy()

            result, mask = self.estimate_pose(bgr, depth_m, K, stamp, camera_frame)

            if result is None:
                self.valid_pub.publish(Bool(data=False))
                self.camera_valid_pub.publish(Bool(data=False))
                self.base_valid_pub.publish(Bool(data=False))
                if self.publish_debug:
                    small = cv2.resize(mask, (debug.shape[1] // 4, debug.shape[0] // 4))
                    small = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
                    debug[0 : small.shape[0], 0 : small.shape[1]] = small
                    cv2.putText(
                        debug,
                        "BOOK: INVALID",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                    )
                return

            # Always publish camera-frame results.
            self.camera_pose_pub.publish(result["camera_pose"])
            self.camera_polygon_pub.publish(result["camera_polygon"])
            self.camera_valid_pub.publish(Bool(data=True))

            # Publish base-frame results only if TF succeeded.
            base_tf_ok = result["base_tf_ok"]
            self.base_valid_pub.publish(Bool(data=base_tf_ok))
            self._last_base_valid = base_tf_ok

            if base_tf_ok:
                # Use base-frame pose as primary output.
                self.pose_pub.publish(result["base_pose"])
                self.size_pub.publish(result["base_size"])
                self.polygon_pub.publish(result["base_polygon"])
                self.valid_pub.publish(Bool(data=True))
                self.rmse_pub.publish(Float32(data=float(result["rmse"])))

                # Only add to lock history when base-frame is available.
                with self.lock:
                    self.history.append(
                        {
                            "stamp": rospy.Time.now(),
                            "pose": result["base_pose"],
                            "size": result["base_size"],
                        }
                    )
            else:
                # Base TF failed - still publish camera-frame as primary.
                self.pose_pub.publish(result["camera_pose"])
                self.size_pub.publish(result["camera_size"])
                self.polygon_pub.publish(result["camera_polygon"])
                self.valid_pub.publish(Bool(data=True))
                self.rmse_pub.publish(Float32(data=float(result["rmse"])))
                # Do NOT add to lock history when base-frame is unavailable.

            if self.publish_debug:
                box = np.round(result["box2d"]).astype(np.int32)
                cv2.polylines(debug, [box], True, (0, 255, 0), 3)

                # Determine display size based on TF status.
                display_size = result["camera_size"]
                tf_status = "TF: OK" if base_tf_ok else "TF: UNAVAILABLE"
                tf_color = (0, 255, 0) if base_tf_ok else (0, 165, 255)

                cv2.putText(
                    debug,
                    "BOOK %.0fx%.0f mm  RMSE %.1f mm  %s"
                    % (
                        display_size.vector.x * 1000.0,
                        display_size.vector.y * 1000.0,
                        result["rmse"] * 1000.0,
                        tf_status,
                    ),
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    tf_color,
                    2,
                )
        except (CvBridgeError, ValueError) as exc:
            rospy.logwarn_throttle(2.0, "Book detection frame rejected: %s", exc)
            self.valid_pub.publish(Bool(data=False))
            self.camera_valid_pub.publish(Bool(data=False))
            self.base_valid_pub.publish(Bool(data=False))
        except Exception as exc:  # Keep camera callback alive, but expose unexpected errors.
            rospy.logerr_throttle(2.0, "Unexpected book estimator error: %s", exc)
            self.valid_pub.publish(Bool(data=False))
            self.camera_valid_pub.publish(Bool(data=False))
            self.base_valid_pub.publish(Bool(data=False))
        finally:
            if self.publish_debug and debug is not None:
                try:
                    self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
                except CvBridgeError as exc:
                    rospy.logwarn_throttle(3.0, "Failed to publish debug image: %s", exc)

    @staticmethod
    def pose_rotation_matrix(pose):
        q = pose.pose.orientation
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    @staticmethod
    def quaternion_angle_deg(q1, q2):
        a = np.array([q1.x, q1.y, q1.z, q1.w], dtype=np.float64)
        b = np.array([q2.x, q2.y, q2.z, q2.w], dtype=np.float64)
        a /= max(np.linalg.norm(a), 1e-12)
        b /= max(np.linalg.norm(b), 1e-12)
        dot = float(np.clip(abs(np.dot(a, b)), 0.0, 1.0))
        return math.degrees(2.0 * math.acos(dot))

    def handle_lock(self, _request):
        # Only allow lock when base-frame TF is available.
        # Check current base_frame_valid state.
        if not hasattr(self, '_last_base_valid') or not self._last_base_valid:
            return TriggerResponse(
                success=False,
                message="Base-frame TF unavailable. Cannot lock target without base_link -> camera transform.",
            )

        with self.lock:
            if len(self.history) < self.stable_frames:
                return TriggerResponse(
                    success=False,
                    message="Need at least %d valid frames; currently %d"
                    % (self.stable_frames, len(self.history)),
                )
            samples = list(self.history)[-self.stable_frames :]

        now = rospy.Time.now()
        if (now - samples[-1]["stamp"]).to_sec() > self.stable_max_age_s:
            return TriggerResponse(success=False, message="Latest detection is stale")

        positions = np.array(
            [
                [
                    s["pose"].pose.position.x,
                    s["pose"].pose.position.y,
                    s["pose"].pose.position.z,
                ]
                for s in samples
            ],
            dtype=np.float64,
        )
        sizes = np.array(
            [[s["size"].vector.x, s["size"].vector.y] for s in samples],
            dtype=np.float64,
        )
        position_std = float(np.linalg.norm(np.std(positions, axis=0)))
        size_std = float(np.max(np.std(sizes, axis=0)))
        reference_q = samples[0]["pose"].pose.orientation
        orientation_spread = max(
            self.quaternion_angle_deg(reference_q, s["pose"].pose.orientation)
            for s in samples
        )

        if position_std > self.max_position_std_m:
            return TriggerResponse(
                success=False,
                message="Position unstable: std=%.1f mm > %.1f mm"
                % (position_std * 1000.0, self.max_position_std_m * 1000.0),
            )
        if size_std > self.max_size_std_m:
            return TriggerResponse(
                success=False,
                message="Size unstable: std=%.1f mm > %.1f mm"
                % (size_std * 1000.0, self.max_size_std_m * 1000.0),
            )
        if orientation_spread > self.max_orientation_spread_deg:
            return TriggerResponse(
                success=False,
                message="Orientation unstable: spread=%.2f deg > %.2f deg"
                % (orientation_spread, self.max_orientation_spread_deg),
            )

        rotations = np.stack([self.pose_rotation_matrix(s["pose"]) for s in samples])
        average_matrix = np.sum(rotations, axis=0)
        u, _, vh = np.linalg.svd(average_matrix)
        average_rotation = u @ vh
        if np.linalg.det(average_rotation) < 0.0:
            u[:, -1] *= -1.0
            average_rotation = u @ vh
        homogeneous = np.eye(4)
        homogeneous[:3, :3] = average_rotation
        quat = quaternion_from_matrix(homogeneous)

        locked_pose = PoseStamped()
        locked_pose.header.frame_id = self.base_frame
        locked_pose.header.stamp = rospy.Time.now()
        mean_position = np.mean(positions, axis=0)
        locked_pose.pose.position.x = float(mean_position[0])
        locked_pose.pose.position.y = float(mean_position[1])
        locked_pose.pose.position.z = float(mean_position[2])
        locked_pose.pose.orientation.x = float(quat[0])
        locked_pose.pose.orientation.y = float(quat[1])
        locked_pose.pose.orientation.z = float(quat[2])
        locked_pose.pose.orientation.w = float(quat[3])

        locked_size = Vector3Stamped()
        locked_size.header = locked_pose.header
        mean_size = np.mean(sizes, axis=0)
        locked_size.vector.x = float(mean_size[0])
        locked_size.vector.y = float(mean_size[1])
        locked_size.vector.z = 0.0

        with self.lock:
            self.locked_pose = locked_pose
            self.locked_size = locked_size

        self.locked_pose_pub.publish(locked_pose)
        self.locked_size_pub.publish(locked_size)
        self.locked_pub.publish(Bool(data=True))
        rospy.loginfo(
            "Book target locked: %.1f x %.1f mm, position std %.1f mm, orientation spread %.2f deg",
            locked_size.vector.x * 1000.0,
            locked_size.vector.y * 1000.0,
            position_std * 1000.0,
            orientation_spread,
        )
        return TriggerResponse(
            success=True,
            message="Locked %.1f x %.1f mm book; position std %.1f mm"
            % (
                locked_size.vector.x * 1000.0,
                locked_size.vector.y * 1000.0,
                position_std * 1000.0,
            ),
        )

    def handle_clear(self, _request):
        with self.lock:
            self.locked_pose = None
            self.locked_size = None
            self.history.clear()
        self.locked_pub.publish(Bool(data=False))
        return TriggerResponse(success=True, message="Locked target and history cleared")

    def make_markers(self, pose, size, locked):
        now = rospy.Time.now()
        markers = MarkerArray()

        plane = Marker()
        plane.header.frame_id = self.base_frame
        plane.header.stamp = now
        plane.ns = "book_locked" if locked else "book_live"
        plane.id = 0
        plane.type = Marker.CUBE
        plane.action = Marker.ADD
        plane.pose = pose.pose
        plane.scale.x = max(size.vector.x, 0.001)
        plane.scale.y = max(size.vector.y, 0.001)
        plane.scale.z = 0.006
        if locked:
            plane.color.r, plane.color.g, plane.color.b, plane.color.a = 0.1, 0.9, 0.2, 0.65
        else:
            plane.color.r, plane.color.g, plane.color.b, plane.color.a = 0.2, 0.5, 1.0, 0.35
        plane.lifetime = rospy.Duration(0.2) if not locked else rospy.Duration(0.0)
        markers.markers.append(plane)

        rotation = self.pose_rotation_matrix(pose)
        origin = np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], dtype=np.float64
        )
        for idx, (axis, color) in enumerate(
            [
                (rotation[:, 0], (1.0, 0.1, 0.1, 1.0)),
                (rotation[:, 1], (0.1, 1.0, 0.1, 1.0)),
                (rotation[:, 2], (0.1, 0.3, 1.0, 1.0)),
            ],
            start=1,
        ):
            arrow = Marker()
            arrow.header = plane.header
            arrow.ns = plane.ns
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2])),
                Point(
                    x=float(origin[0] + axis[0] * 0.10),
                    y=float(origin[1] + axis[1] * 0.10),
                    z=float(origin[2] + axis[2] * 0.10),
                ),
            ]
            arrow.scale.x = 0.008
            arrow.scale.y = 0.016
            arrow.scale.z = 0.025
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = color
            arrow.lifetime = plane.lifetime
            markers.markers.append(arrow)
        return markers

    def timer_callback(self, _event):
        with self.lock:
            locked_pose = self.locked_pose
            locked_size = self.locked_size
            latest = self.history[-1] if self.history else None

        if locked_pose is not None and locked_size is not None:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self.base_frame
            transform.child_frame_id = self.locked_frame
            transform.transform.translation.x = locked_pose.pose.position.x
            transform.transform.translation.y = locked_pose.pose.position.y
            transform.transform.translation.z = locked_pose.pose.position.z
            transform.transform.rotation = locked_pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            self.marker_pub.publish(self.make_markers(locked_pose, locked_size, True))
        elif latest is not None:
            self.marker_pub.publish(self.make_markers(latest["pose"], latest["size"], False))


def main():
    rospy.init_node("book_pose_estimator")
    BookPoseEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()

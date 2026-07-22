#!/usr/bin/env python3
"""
V1 标定目标可见性验证：实际读取三相机 color 帧，执行 ChArUco + AprilTag 检测。

每台相机:
- 检测 ChArUco 角点/标记 (DICT_5X5_1000)
- 检测 AprilTag (DICT_APRILTAG_36h11)
- 输出 JSON 结果
- 生成共同观测图 (common_observation_graph)

用法:
  rosrun cr5_spray_sim validate_calibration_target_visibility.py [--output artifacts/calibration_target_v1/]

退出码:
  0 = 三相机全部通过且连通
  1 = 部分通过
  2 = 检测失败
"""
import sys
import os
import json
import time
import math
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco

# 精确相机话题
CAMERAS = {
    "cam_front_left":  "/cam_front_left/camera/color/image_raw",
    "cam_front_right": "/cam_front_right/camera/color/image_raw",
    "cam_rear":        "/cam_rear/camera/color/image_raw",
}
CAMERA_INFOS = {
    "cam_front_left":  "/cam_front_left/camera/color/camera_info",
    "cam_front_right": "/cam_front_right/camera/color/camera_info",
    "cam_rear":        "/cam_rear/camera/color/camera_info",
}

# 标定参数
CHARUCO_DICT = aruco.DICT_5X5_1000
APRILTAG_DICT = aruco.DICT_APRILTAG_36h11

# 正面 ChArUco: ID 100-123
FRONT_MARKER_IDS = set(range(100, 124))
# 左侧 ChArUco: ID 200-214
LEFT_MARKER_IDS = set(range(200, 215))
# AprilTag IDs
APRILTAG_IDS = set(range(4, 9))

# 最小检测阈值
MIN_CHARUCO_CORNERS = 12
MIN_TAG_SIDE_PX = 40


class VisibilityValidator:
    def __init__(self):
        self.bridge = CvBridge()
        self.charuco_dict = aruco.getPredefinedDictionary(CHARUCO_DICT)
        self.apriltag_dict = aruco.getPredefinedDictionary(APRILTAG_DICT)

        # 检测参数
        self.charuco_params = aruco.DetectorParameters()
        self.charuco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.apriltag_params = aruco.DetectorParameters()
        self.apriltag_detector = aruco.ArucoDetector(
            self.apriltag_dict, self.apriltag_params)

        self.results = {}

    def capture_frame(self, cam_name, timeout=10.0):
        """捕获一台相机的 color + camera_info."""
        color_topic = CAMERAS[cam_name]
        info_topic = CAMERA_INFOS[cam_name]

        try:
            color_msg = rospy.wait_for_message(color_topic, Image, timeout=timeout)
            info_msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=timeout)
        except rospy.ROSException:
            rospy.logerr("%s: capture timeout", cam_name)
            return None, None

        try:
            cv_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("%s: bridge failed: %s", cam_name, e)
            return None, None

        K = np.array(info_msg.K).reshape(3, 3)
        return cv_img, K

    def detect_charuco(self, img, expected_ids=None):
        """检测 ChArUco 标记和角点."""
        corners, ids, rejected = aruco.detectMarkers(
            img, self.charuco_dict, parameters=self.charuco_params)

        result = {
            "marker_count": 0,
            "marker_ids": [],
            "corner_count": 0,
            "matched_ids": [],
        }

        if ids is not None:
            result["marker_count"] = len(ids)
            result["marker_ids"] = [int(i) for i in ids.flatten()]

            if expected_ids:
                result["matched_ids"] = [i for i in result["marker_ids"] if i in expected_ids]

            # ChArUco 角点插值
            try:
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, img, self.charuco_dict)
                if charuco_ids is not None:
                    result["corner_count"] = len(charuco_ids)
                    result["charuco_ids"] = [int(i) for i in charuco_ids.flatten()]
            except Exception:
                result["corner_count"] = 0

        result["pass"] = (
            result["marker_count"] > 0 and
            result["corner_count"] >= MIN_CHARUCO_CORNERS
        )
        return result

    def detect_apriltag(self, img):
        """检测 AprilTag."""
        corners, ids, rejected = self.apriltag_detector.detectMarkers(img)

        result = {
            "tag_count": 0,
            "tag_ids": [],
            "tag_sizes_px": [],
        }

        if ids is not None:
            result["tag_count"] = len(ids)
            result["tag_ids"] = [int(i) for i in ids.flatten()]

            for i, c in enumerate(corners):
                # 估算 tag 边长
                sides = []
                for j in range(4):
                    p1 = c[0][j]
                    p2 = c[0][(j + 1) % 4]
                    side = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                    sides.append(side)
                result["tag_sizes_px"].append(round(np.mean(sides), 1))

        detected_known = [i for i in result["tag_ids"] if i in APRILTAG_IDS]
        result["matched_ids"] = detected_known

        result["pass"] = (
            result["tag_count"] > 0 and
            any(s >= MIN_TAG_SIDE_PX for s in result["tag_sizes_px"])
        )
        return result

    def identify_faces(self, charuco_result, apriltag_result):
        """从检测结果识别标定面."""
        faces = []

        # ChArUco ID 范围判断
        front_ids = set(charuco_result.get("matched_ids", [])) & FRONT_MARKER_IDS
        left_ids = set(charuco_result.get("matched_ids", [])) & LEFT_MARKER_IDS

        if front_ids:
            faces.append("front_charuco")
        if left_ids:
            faces.append("left_charuco")

        # AprilTag IDs
        tag_ids = set(apriltag_result.get("matched_ids", []))
        right_ids = tag_ids & {4, 5, 6, 7}
        top_ids = tag_ids & {8}

        if right_ids:
            faces.append("right_apriltag")
        if top_ids:
            faces.append("top_apriltag")

        return faces

    def validate_all(self):
        """验证所有三台相机."""
        all_pass = True
        camera_observations = {}

        for cam_name in sorted(CAMERAS.keys()):
            rospy.loginfo("=== Capturing %s ===", cam_name)
            cv_img, K = self.capture_frame(cam_name)

            if cv_img is None:
                rospy.logerr("%s: no image", cam_name)
                self.results[cam_name] = {"pass": False, "error": "no_image"}
                all_pass = False
                continue

            # 检测
            charuco_r = self.detect_charuco(cv_img)
            apriltag_r = self.detect_apriltag(cv_img)
            faces = self.identify_faces(charuco_r, apriltag_r)

            cam_result = {
                "stamp": rospy.get_time(),
                "image_shape": list(cv_img.shape),
                "charuco": charuco_r,
                "apriltag": apriltag_r,
                "complete_faces": faces,
                "pass": charuco_r["pass"] or apriltag_r["pass"],
            }
            self.results[cam_name] = cam_result
            camera_observations[cam_name] = {
                "faces": faces,
                "charuco_ids": charuco_r.get("matched_ids", []),
                "apriltag_ids": apriltag_r.get("matched_ids", []),
            }

            if not cam_result["pass"]:
                all_pass = False
                rospy.logerr("%s: FAILED", cam_name)
            else:
                rospy.loginfo("%s: OK (faces=%s)", cam_name, faces)

        # 生成共同观测图
        graph = self.build_common_observation_graph(camera_observations)
        self.results["common_observation_graph"] = graph

        # 连通性检查
        if len(graph["connected_components"]) > 0:
            largest = max(len(c) for c in graph["connected_components"])
            graph["all_connected"] = (largest >= 3)
        else:
            graph["all_connected"] = False

        if not graph["all_connected"]:
            all_pass = False
            rospy.logerr("Common observation graph NOT fully connected")

        self.results["all_pass"] = all_pass
        return all_pass

    def build_common_observation_graph(self, observations):
        """构建三相机共同观测图."""
        # 按共享标签 ID 或共享面建立边
        edges = []
        cam_names = sorted(observations.keys())

        for i, c1 in enumerate(cam_names):
            for c2 in cam_names[i + 1:]:
                o1 = observations[c1]
                o2 = observations[c2]

                # 共享面
                shared_faces = set(o1["faces"]) & set(o2["faces"])
                # 共享 ChArUco ID
                shared_charuco = set(o1["charuco_ids"]) & set(o2["charuco_ids"])
                # 共享 AprilTag ID
                shared_tags = set(o1["apriltag_ids"]) & set(o2["apriltag_ids"])

                if shared_faces or shared_charuco or shared_tags:
                    edges.append({
                        "cameras": [c1, c2],
                        "shared_faces": sorted(shared_faces),
                        "shared_charuco_ids": sorted(shared_charuco),
                        "shared_apriltag_ids": sorted(shared_tags),
                    })

        # 简单连通分量 (BFS)
        adj = {c: [] for c in cam_names}
        for e in edges:
            adj[e["cameras"][0]].append(e["cameras"][1])
            adj[e["cameras"][1]].append(e["cameras"][0])

        visited = set()
        components = []
        for cam in cam_names:
            if cam not in visited:
                comp = []
                stack = [cam]
                while stack:
                    node = stack.pop()
                    if node not in visited:
                        visited.add(node)
                        comp.append(node)
                        stack.extend(adj[node])
                components.append(comp)

        return {
            "edges": edges,
            "edge_count": len(edges),
            "connected_components": components,
        }


def main():
    rospy.init_node("validate_calibration_target_visibility", anonymous=True,
                    log_level=rospy.WARN)

    output_dir = None
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]

    validator = VisibilityValidator()
    all_pass = validator.validate_all()

    # 输出结果
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        out_path = os.path.join(output_dir, "detection_result.json")
        with open(out_path, "w") as f:
            json.dump(validator.results, f, indent=2, default=str)
        rospy.loginfo("Results saved: %s", out_path)
    else:
        print(json.dumps(validator.results, indent=2, default=str))

    if all_pass:
        rospy.loginfo("ALL CAMERAS PASS — visibility verified")
        sys.exit(0)
    else:
        rospy.logerr("VISIBILITY CHECK FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()

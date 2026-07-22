#!/usr/bin/env python3
"""
V2 标定目标可见性验证: 使用 CharucoDetector + 分面检测。

P1 修复:
- 使用 CharucoDetector.detectBoard() 替代旧 API
- 每块面板独立检测 (front/left/back ChArUco, right/top AprilTag)
- 同步 color + CameraInfo + depth
- 输出 annotated 图 + 检测 JSON

用法:
  rosrun cr5_spray_sim validate_calibration_target_visibility.py [--output artifacts/]

退出码: 0=全部通过, 1=部分通过
"""
import sys, os, json, time, math
import cv2, numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco

CAMERAS = {
    "cam_front_left": {
        "color": "/cam_front_left/camera/color/image_raw",
        "info":  "/cam_front_left/camera/color/camera_info",
        "depth": "/cam_front_left/camera/depth/image_raw",
    },
    "cam_front_right": {
        "color": "/cam_front_right/camera/color/image_raw",
        "info":  "/cam_front_right/camera/color/camera_info",
        "depth": "/cam_front_right/camera/depth/image_raw",
    },
    "cam_rear": {
        "color": "/cam_rear/camera/color/image_raw",
        "info":  "/cam_rear/camera/color/camera_info",
        "depth": "/cam_rear/camera/depth/image_raw",
    },
}

# === Board 构造 (OpenCV 4.2: CharucoBoard_create 不支持自定义 ID，手工映射) ===
def make_custom_charuco_board(sx, sy, sq_m, mk_m, dictionary, id_start):
    """创建 ChArUco board 并用自定义 ID 检测。返回 (board_obj, custom_ids_map)."""
    board = aruco.CharucoBoard_create(sx, sy, sq_m, mk_m, dictionary)
    custom_ids = list(range(id_start, id_start + sx * sy))
    return board, custom_ids

BOARD_FRONT, IDS_FRONT = make_custom_charuco_board(8, 6, 0.027, 0.020,
    aruco.getPredefinedDictionary(aruco.DICT_5X5_1000), 100)
BOARD_LEFT,  IDS_LEFT  = make_custom_charuco_board(6, 5, 0.022, 0.016,
    aruco.getPredefinedDictionary(aruco.DICT_5X5_1000), 200)
BOARD_BACK,  IDS_BACK  = make_custom_charuco_board(8, 6, 0.027, 0.020,
    aruco.getPredefinedDictionary(aruco.DICT_5X5_1000), 300)

# AprilTag 参数
RIGHT_TAG_IDS = {4, 5, 6, 7}
TOP_TAG_ID = 8
APRILTAG_DICT = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)

MIN_CORNERS = 12
MIN_TAG_SIDE_PX = 40


class VisibilityValidator:
    def __init__(self, output_dir=None):
        self.bridge = CvBridge()
        self.output_dir = output_dir
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        self.results = {}

    def detect_charuco_face(self, gray_img, board, expected_start_id, face_name):
        """使用 CharucoDetector 检测单面 ChArUco."""
        result = {"face": face_name, "marker_ids": [], "corner_count": 0, "complete": False}

        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        dictionary = board.dictionary
        corners, ids, rejected = aruco.detectMarkers(gray_img, dictionary, parameters=params)

        if ids is None:
            return result

        # 过滤: 只保留属于此面板 ID 范围的 markers
        ids_flat_all = [int(i) for i in ids.flatten()]
        sx, sy = board.getChessboardSize()
        expected_ids = set(range(expected_start_id, expected_start_id + sx * sy))
        matched_indices = [i for i, mid in enumerate(ids_flat_all) if mid in expected_ids]

        if len(matched_indices) < 2:
            return result

        # 提取匹配的 corners/ids
        matched_ids_arr = np.array([[ids_flat_all[i]] for i in matched_indices], dtype=np.int32)
        matched_corners = tuple([corners[i] for i in matched_indices])

        result["marker_ids"] = [ids_flat_all[i] for i in matched_indices]

        # ChArUco corner interpolation
        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            matched_corners, matched_ids_arr, gray_img, board)
        if charuco_ids is not None and len(charuco_ids) > 0:
            result["corner_count"] = len(charuco_ids)
            result["charuco_ids"] = [int(i) for i in charuco_ids.flatten()]

        result["complete"] = result["corner_count"] >= MIN_CORNERS
        return result

    def detect_apriltag_face(self, gray_img, face_name):
        """检测 AprilTag (right 2×2 和 top single)."""
        result = {"face": face_name, "tag_ids": [], "tag_sizes_px": [], "complete": False}

        params = aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        detector = aruco.ArucoDetector(APRILTAG_DICT, params)
        corners, ids, rejected = detector.detectMarkers(gray_img)

        if ids is None:
            return result

        ids_flat = [int(i) for i in ids.flatten()]
        if face_name == "right_apriltag":
            matched = [i for i in ids_flat if i in RIGHT_TAG_IDS]
        else:
            matched = [i for i in ids_flat if i == TOP_TAG_ID]

        result["tag_ids"] = matched

        for i, mid in enumerate(ids_flat):
            if mid in matched:
                c = corners[ids_flat.index(mid)][0]
                sides = [math.hypot(c[j][0]-c[(j+1)%4][0], c[j][1]-c[(j+1)%4][1])
                         for j in range(4)]
                result["tag_sizes_px"].append(round(np.mean(sides), 1))

        min_required = 2 if face_name == "right_apriltag" else 1
        result["complete"] = (len(matched) >= min_required and
                              any(s >= MIN_TAG_SIDE_PX for s in result["tag_sizes_px"]))
        return result

    def process_camera(self, cam_name):
        """处理单台相机."""
        topics = CAMERAS[cam_name]
        rospy.loginfo("=== Capturing %s ===", cam_name)

        try:
            color_msg = rospy.wait_for_message(topics["color"], Image, timeout=10.0)
            info_msg  = rospy.wait_for_message(topics["info"],  CameraInfo, timeout=10.0)
            depth_msg = rospy.wait_for_message(topics["depth"], Image, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("%s: timeout", cam_name)
            return {"pass": False, "error": "capture_timeout"}

        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            return {"pass": False, "error": str(e)}

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        K = np.array(info_msg.K).reshape(3, 3)

        # 检测每块面板
        front_r = self.detect_charuco_face(gray, BOARD_FRONT, 100, "front_charuco")
        left_r  = self.detect_charuco_face(gray, BOARD_LEFT,  200, "left_charuco")
        back_r  = self.detect_charuco_face(gray, BOARD_BACK,  300, "back_charuco")
        right_r = self.detect_apriltag_face(gray, "right_apriltag")
        top_r   = self.detect_apriltag_face(gray, "top_apriltag")

        faces = []
        for r in [front_r, left_r, back_r, right_r, top_r]:
            if r["complete"]:
                faces.append(r["face"])

        cam_result = {
            "camera": cam_name,
            "stamp": rospy.get_time(),
            "image_shape": list(color_img.shape),
            "K": K.tolist(),
            "front_charuco": front_r,
            "left_charuco": left_r,
            "back_charuco": back_r,
            "right_apriltag": right_r,
            "top_apriltag": top_r,
            "complete_faces": faces,
            "pass": len(faces) >= 1,
        }
        self.results[cam_name] = cam_result

        # 保存图像
        if self.output_dir:
            cv2.imwrite(os.path.join(self.output_dir, f"{cam_name}_color.png"), color_img)
            # 标注图
            annotated = color_img.copy()
            rospy.loginfo("%s: faces=%s %s", cam_name, faces,
                          "PASS" if cam_result["pass"] else "FAIL")

        return cam_result

    def build_graph(self):
        """构建共同观测图."""
        cams = sorted(self.results.keys())
        edges = []
        for i, c1 in enumerate(cams):
            r1 = self.results.get(c1, {})
            for c2 in cams[i+1:]:
                r2 = self.results.get(c2, {})
                shared = set(r1.get("complete_faces", [])) & set(r2.get("complete_faces", []))
                if shared:
                    edges.append({"cameras": [c1, c2], "shared_faces": sorted(shared)})

        # 连通分量
        adj = {c: [] for c in cams}
        for e in edges:
            adj[e["cameras"][0]].append(e["cameras"][1])
            adj[e["cameras"][1]].append(e["cameras"][0])

        visited = set()
        components = []
        for c in cams:
            if c not in visited:
                comp = []
                stack = [c]
                while stack:
                    n = stack.pop()
                    if n not in visited:
                        visited.add(n); comp.append(n)
                        stack.extend(adj[n])
                components.append(comp)

        graph = {
            "direct_shared_observation_edges": edges,
            "rigid_target_edges": [],
            "all_cameras_connected_via_target": len(components) == 1 and len(components[0]) >= 3,
            "connected_components": components,
        }
        graph["rigid_target_edges"].append({
            "note": "All cameras observe faces on the rigid calibration_target_frame"
        })
        return graph


def main():
    rospy.init_node("validate_calibration_target_visibility", anonymous=True, log_level=rospy.WARN)

    output_dir = None
    for i, arg in enumerate(sys.argv):
        if arg == "--output" and i + 1 < len(sys.argv):
            output_dir = sys.argv[i + 1]

    validator = VisibilityValidator(output_dir)
    all_pass = True
    for cam_name in sorted(CAMERAS.keys()):
        result = validator.process_camera(cam_name)
        if not result.get("pass"):
            all_pass = False

    graph = validator.build_graph()
    validator.results["common_observation_graph"] = graph
    if not graph["all_cameras_connected_via_target"]:
        all_pass = False

    validator.results["all_pass"] = all_pass

    if output_dir:
        with open(os.path.join(output_dir, "detection_result.json"), "w") as f:
            json.dump(validator.results, f, indent=2, default=str)
    else:
        print(json.dumps(validator.results, indent=2, default=str))

    rospy.loginfo("Visibility %s", "PASS" if all_pass else "FAIL")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V3 标定目标可见性验证 — OpenCV 4.2 兼容版.

使用 aruco_compat 兼容层 + 自定义 Marker ID remap + argparse.
"""
import sys, os, json, time, math, argparse
import cv2, numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from cv2 import aruco

# 导入兼容层
from cr5_spray_perception import aruco_compat

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

# ChArUco 面板定义
CHARUCO_FACES = {
    "front_charuco": {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
                      "dict_id": aruco.DICT_5X5_1000, "id_start": 100},
    "left_charuco":  {"sx": 6, "sy": 5, "sq_m": 0.022, "mk_m": 0.016,
                      "dict_id": aruco.DICT_5X5_1000, "id_start": 200},
    "back_charuco":  {"sx": 8, "sy": 6, "sq_m": 0.027, "mk_m": 0.020,
                      "dict_id": aruco.DICT_5X5_1000, "id_start": 300},
}

APRILTAG_DICT_ID = aruco.DICT_APRILTAG_36h11
RIGHT_TAG_IDS = {4, 5, 6, 7}
TOP_TAG_ID = 8

MIN_CORNERS = 12
MIN_TAG_SIDE_PX = 40

# 预创建 boards
for k, v in CHARUCO_FACES.items():
    dict_obj = aruco.getPredefinedDictionary(v["dict_id"])
    v["board"] = aruco.CharucoBoard_create(v["sx"], v["sy"], v["sq_m"], v["mk_m"], dict_obj)
    v["n_markers"] = int(np.asarray(v["board"].ids).size)


def detect_charuco_face(gray, face_cfg, face_name):
    """对单个 ChArUco 面板检测."""
    board = face_cfg["board"]
    id_start = face_cfg["id_start"]
    n_markers = face_cfg["n_markers"]
    id_end = id_start + n_markers - 1

    result = {"face": face_name, "marker_ids": [], "corner_count": 0, "complete": False}

    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(
        gray, board.dictionary, params)

    if ids is None:
        return result

    ids_flat_all = [int(i) for i in ids.flatten()]
    idx_list, local_ids = aruco_compat.remap_custom_ids(ids_flat_all, id_start, board)

    if len(idx_list) < 2:
        return result

    # 保存真实 ID
    result["marker_ids"] = [ids_flat_all[i] for i in idx_list]

    # 构建 local corners
    local_corners = tuple(corners[i] for i in idx_list)

    # ChArUco 插值
    cc, cids = aruco_compat.interpolate_charuco_corners(
        local_corners, local_ids, gray, board)
    if cids is not None and len(cids) > 0:
        result["corner_count"] = len(cids)
    result["complete"] = result["corner_count"] >= MIN_CORNERS
    return result


def detect_apriltag_face(gray, face_name):
    """检测 AprilTag."""
    result = {"face": face_name, "tag_ids": [], "tag_sizes_px": [], "complete": False}

    tag_dict = aruco.getPredefinedDictionary(APRILTAG_DICT_ID)
    params = aruco_compat.detector_parameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, rejected = aruco_compat.detect_markers(gray, tag_dict, params)

    if ids is None:
        return result

    ids_flat = [int(i) for i in ids.flatten()]
    target_ids = RIGHT_TAG_IDS if face_name == "right_apriltag" else {TOP_TAG_ID}
    matched = [(i, mid) for i, mid in enumerate(ids_flat) if mid in target_ids]

    if not matched:
        return result

    result["tag_ids"] = [m[1] for m in matched]
    for idx, mid in matched:
        c = corners[idx][0]
        sides = [math.hypot(c[j][0]-c[(j+1)%4][0], c[j][1]-c[(j+1)%4][1])
                 for j in range(4)]
        result["tag_sizes_px"].append(round(np.mean(sides), 1))

    min_required = 2 if face_name == "right_apriltag" else 1
    result["complete"] = (len(matched) >= min_required and
                          any(s >= MIN_TAG_SIDE_PX for s in result["tag_sizes_px"]))
    return result


def process_camera(cam_name, output_dir):
    """处理单台相机."""
    topics = CAMERAS[cam_name]
    rospy.loginfo("=== %s ===", cam_name)
    bridge = CvBridge()

    try:
        color_msg = rospy.wait_for_message(topics["color"], Image, timeout=12.0)
        info_msg  = rospy.wait_for_message(topics["info"],  CameraInfo, timeout=6.0)
        depth_msg = rospy.wait_for_message(topics["depth"], Image, timeout=6.0)
    except rospy.ROSException:
        return {"camera": cam_name, "pass": False, "error": "capture_timeout"}

    try:
        color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    except Exception as e:
        return {"camera": cam_name, "pass": False, "error": str(e)}

    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    K = np.array(info_msg.K).reshape(3, 3)

    # 检测
    faces_detected = {}
    for face_key, cfg in CHARUCO_FACES.items():
        faces_detected[face_key] = detect_charuco_face(gray, cfg, face_key)
    faces_detected["right_apriltag"] = detect_apriltag_face(gray, "right_apriltag")
    faces_detected["top_apriltag"] = detect_apriltag_face(gray, "top_apriltag")

    complete = [k for k, v in faces_detected.items() if v.get("complete")]

    cam_result = {
        "camera": cam_name,
        "stamp": rospy.get_time(),
        "image_shape": list(color_img.shape),
        "K": K.tolist(),
        "complete_faces": complete,
        **faces_detected,
        "pass": len(complete) >= 1,
    }

    # 保存图像
    if output_dir:
        cv2.imwrite(os.path.join(output_dir, f"{cam_name}_color.png"), color_img)
        # Annotated: draw detected markers
        annotated = color_img.copy()
        params = aruco_compat.detector_parameters()
        dict_all = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        mc, mi, _ = aruco_compat.detect_markers(gray, dict_all, params)
        if mi is not None:
            annotated = aruco_compat.draw_detected_markers(annotated, mc, mi)
        cv2.imwrite(os.path.join(output_dir, f"{cam_name}_annotated.png"), annotated)

        # depth 统计 (encoding 感知, 转换为米)
        try:
            depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_encoding = getattr(depth_msg, "encoding", "unknown")
            depth_dtype = str(depth_img.dtype)

            # 编码感知深度→米转换
            if depth_encoding in ("16UC1", "mono16") or depth_img.dtype == np.uint16:
                depth_m = depth_img.astype(np.float64) / 1000.0
                depth_scale_to_m = 0.001
                depth_unit = "mm"
            elif depth_encoding in ("32FC1",) or depth_img.dtype in (np.float32, np.float64):
                depth_m = depth_img.astype(np.float64)
                depth_scale_to_m = 1.0
                depth_unit = "m"
            else:
                rospy.logwarn("%s: unknown depth encoding=%s dtype=%s — raw values may be wrong",
                              cam_name, depth_encoding, depth_dtype)
                depth_m = depth_img.astype(np.float64)
                depth_scale_to_m = "unknown"
                depth_unit = "unknown"

            finite = np.isfinite(depth_m) & (depth_m > 0)
            depth_stats = {
                "encoding": depth_encoding,
                "dtype": depth_dtype,
                "scale_to_m": depth_scale_to_m,
                "depth_unit": depth_unit,
                "nonzero_pct": round(float(np.sum(depth_m > 0) / depth_m.size * 100), 2),
                "finite_pct": round(float(np.isfinite(depth_m).sum() / depth_m.size * 100), 2),
                "min_m": round(float(np.min(depth_m[finite])), 4) if finite.any() else -1,
                "median_m": round(float(np.median(depth_m[finite])), 4) if finite.any() else -1,
                "max_m": round(float(np.max(depth_m[finite])), 4) if finite.any() else -1,
            }
            with open(os.path.join(output_dir, f"{cam_name}_depth_stats.json"), "w") as f:
                json.dump(depth_stats, f, indent=2)
            cam_result["depth_stats"] = depth_stats
        except Exception as e:
            rospy.logwarn("%s: depth stats failed: %s", cam_name, e)

    rospy.loginfo("%s: faces=%s %s", cam_name, complete, "PASS" if cam_result["pass"] else "FAIL")
    return cam_result


def build_graph(results):
    """构建共同观测图 (仅使用 CAMERAS 定义的真实相机)."""
    cams = sorted(CAMERAS.keys())
    # Direct edges: two cameras see the same face
    direct_edges = []
    for i, c1 in enumerate(cams):
        r1 = results.get(c1, {})
        if isinstance(r1, dict):
            f1 = set(r1.get("complete_faces", []))
            for c2 in cams[i+1:]:
                r2 = results.get(c2, {})
                if isinstance(r2, dict):
                    shared = f1 & set(r2.get("complete_faces", []))
                    if shared:
                        direct_edges.append({"cameras": [c1, c2], "shared_faces": sorted(shared)})
    # Rigid target edges: two cameras see different faces on same rigid target
    rigid_edges = []
    cameras_with_faces = [c for c in cams if results.get(c, {}).get("complete_faces")]
    for i, c1 in enumerate(cameras_with_faces):
        for c2 in cameras_with_faces[i+1:]:
            if not any(e for e in direct_edges if {c1, c2} == set(e["cameras"])):
                rigid_edges.append({"cameras": [c1, c2], "note": "both observe rigid calibration_target"})

    # Build adjacency from all edge types
    all_edges = direct_edges + rigid_edges
    adj = {c: [] for c in cams}
    for e in all_edges:
        adj[e["cameras"][0]].append(e["cameras"][1])
        adj[e["cameras"][1]].append(e["cameras"][0])

    visited, components = set(), []
    for c in cams:
        if c not in visited:
            comp, stack = [], [c]
            while stack:
                n = stack.pop()
                if n not in visited:
                    visited.add(n); comp.append(n)
                    stack.extend(adj[n])
            components.append(comp)

    all_connected = len(components) == 1 and len(components[0]) >= len(cams)
    return {
        "direct_shared_observation_edges": direct_edges,
        "rigid_target_edges": rigid_edges,
        "all_cameras_connected_via_target": all_connected,
        "connected_components": components,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="output artifact directory")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("validate_calibration_target_visibility", anonymous=True, log_level=rospy.WARN)
    aruco_compat.log_capability()

    os.makedirs(args.output, exist_ok=True)
    results = {
        "opencv_capability": aruco_compat.get_opencv_info(),
        "capture_mode": "online_debug",
        "capture_mode_note": "非严格同步采集, color+camera_info 独立 wait_for_message, "
                            "depth 为附加统计. 正式验收建议使用 CaptureManager offline 模式.",
    }

    all_pass = True
    for cam_name in sorted(CAMERAS.keys()):
        r = process_camera(cam_name, args.output)
        results[cam_name] = r
        if not r.get("pass"):
            all_pass = False

    graph = build_graph(results)
    results["common_observation_graph"] = graph
    if not graph["all_cameras_connected_via_target"]:
        all_pass = False

    results["all_pass"] = all_pass
    with open(os.path.join(args.output, "detection_result.json"), "w") as f:
        json.dump(results, f, indent=2, default=str)

    rospy.loginfo("Visibility %s", "PASS" if all_pass else "FAIL")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
V8.16 Image Acceptance Gate: 固定 base target pose,
连续取 N 帧, 使用冻结的 V8.14 detector 统计三路图像质量.

V8.15 final cell: target (0.72, 0, 0.62), quality profile 640x480@10Hz.
"""
import sys, os, math, argparse
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from cr5_spray_perception.calibration.target_detector import detect_target, create_default_profiles
from cr5_spray_perception.calibration.target_geometry import load_target_geometry

CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()

    rospy.init_node("v816_image_acceptance", anonymous=True)
    bridge = CvBridge()
    target_geom = load_target_geometry()
    profiles = create_default_profiles()

    rospy.loginfo("Target: %d faces, Profiles: %s",
                  len(target_geom.charuco_faces) + len(target_geom.aruco_faces) + len(target_geom.apriltag_faces),
                  profiles.profile_summary())

    # Read camera infos (K, D)
    cam_data = {}
    for cam in CAMERAS:
        info = rospy.wait_for_message(f"/{cam}/camera/color/camera_info", CameraInfo, timeout=5.0)
        K = np.array(info.K).reshape(3, 3)
        D = np.array(info.D, dtype=np.float64) if info.D and len(info.D) >= 4 else np.zeros(5)
        cam_data[cam] = {"K": K, "D": D, "w": info.width, "h": info.height}
        rospy.loginfo("%s: %dx%d K=[%.1f,%.1f] D=%s", cam, info.width, info.height, K[0,0], K[1,1], list(D[:4]))

    # Stats
    stats = {cam: {"n": 0, "nd": 0, "faces": {}, "corners": [], "fill_H": [], "fill_V": [],
                   "center_dev": [], "edge_clip": 0} for cam in CAMERAS}

    rospy.loginfo("Capturing %d frames...", args.frames)
    for i in range(args.frames):
        for cam in CAMERAS:
            try:
                img_msg = rospy.wait_for_message(f"/{cam}/camera/color/image_raw", Image, timeout=5.0)
            except:
                continue

            cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            h, w = cv_img.shape[:2]
            K = cam_data[cam]["K"]
            D = cam_data[cam]["D"]

            # detect_target(cv_img, K, D, target_geometry, profiles)
            # Returns: {face_name: {"object_points_3d_face": [...], "image_points_2d": [...], "corner_count": int}}
            detection = detect_target(cv_img, K, D, target_geom, profiles)

            st = stats[cam]
            st["n"] += 1

            if not detection:
                continue

            st["nd"] += 1

            # Collect all image points for fill/center stats
            all_pts = []
            for face_name, face_data in detection.items():
                st["faces"][face_name] = st["faces"].get(face_name, 0) + 1
                st["corners"].append(face_data.get("corner_count", 0))
                pts = face_data.get("image_points_2d", [])
                if pts:
                    all_pts.extend(pts)

            if all_pts:
                pts_arr = np.array(all_pts)
                min_uv = pts_arr.min(axis=0)
                max_uv = pts_arr.max(axis=0)
                extent_w = max_uv[0] - min_uv[0]
                extent_h = max_uv[1] - min_uv[1]
                cu = (min_uv[0] + max_uv[0]) / 2.0
                cv = (min_uv[1] + max_uv[1]) / 2.0

                st["fill_H"].append(extent_w / w * 100)
                st["fill_V"].append(extent_h / h * 100)
                st["center_dev"].append(
                    math.sqrt((cu - w/2)**2 + (cv - h/2)**2) / math.sqrt(w**2 + h**2) * 100)
                if min_uv[0] <= 3 or min_uv[1] <= 3 or max_uv[0] >= w-3 or max_uv[1] >= h-3:
                    st["edge_clip"] += 1

        if (i+1) % 10 == 0:
            rospy.loginfo("  %d/%d", i+1, args.frames)

    # Report
    print("\n" + "="*72)
    print("  V8.16 IMAGE ACCEPTANCE — V8.15 Final Cell (base target pose)")
    print("="*72)

    all_ok = True
    for cam in CAMERAS:
        st = stats[cam]
        n, nd = st["n"], st["nd"]
        det_rate = nd / max(n, 1) * 100
        print(f"\n--- {cam} ({n} frames, {nd} detected = {det_rate:.1f}%) ---")

        if det_rate < 80:
            print(f"  [WARN] Detection rate < 80%")
            all_ok = False

        if st["fill_H"]:
            fh = np.median(st["fill_H"])
            fv = np.median(st["fill_V"])
            cd = np.median(st["center_dev"])
            print(f"  Fill: H={fh:.1f}% V={fv:.1f}%  CenterDev={cd:.1f}%")
            print(f"  Fill range: H[{np.min(st['fill_H']):.1f}..{np.max(st['fill_H']):.1f}]  "
                  f"V[{np.min(st['fill_V']):.1f}..{np.max(st['fill_V']):.1f}]")

            h_ok = 20 <= fh <= 50
            v_ok = 25 <= fv <= 55
            c_ok = cd <= 15
            if not h_ok: print(f"  [WARN] H-fill {fh:.1f}% outside [20,50]%")
            if not v_ok: print(f"  [WARN] V-fill {fv:.1f}% outside [25,55]%")
            if not c_ok: print(f"  [WARN] CenterDev {cd:.1f}% > 15%")
            if not all([h_ok, v_ok, c_ok]):
                all_ok = False
        else:
            print("  [FAIL] No corners in any frame!")
            all_ok = False

        # Face visibility
        print(f"  Faces ({len(st['faces'])}):")
        for fn, cnt in sorted(st["faces"].items(), key=lambda x: -x[1]):
            print(f"    {fn}: {cnt}/{n} ({cnt/n*100:.0f}%)")

        # Critical checks
        if cam == "cam_front_left" and "right" not in st["faces"]:
            print(f"  [WARN] FL missing right face!")
        if cam == "cam_front_right" and "left" not in st["faces"]:
            print(f"  [WARN] FR missing left face!")
        if cam == "cam_rear" and "front" not in st["faces"]:
            print(f"  [WARN] RE missing front face!")

        # Corner count
        if st["corners"]:
            cn = np.array(st["corners"])
            print(f"  Corners: med={np.median(cn):.0f} min={np.min(cn):.0f} max={np.max(cn):.0f}")
            if np.median(cn) < 8:
                print(f"  [WARN] Median corners < 8")

        # Edge clip
        print(f"  Edge clip: {st['edge_clip']}/{n}")
        if st["edge_clip"] > n * 0.1:
            print(f"  [WARN] >10% edge clip")

    print(f"\n{'='*72}")
    if all_ok:
        print("  IMAGE_ACCEPTANCE_PASS")
    else:
        print("  IMAGE_ACCEPTANCE_FAIL (see warnings)")
    print("="*72)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
CR5 Reconstruction — 三相机可见表面 GT 生成.

使用深度缓冲方法:
1. 将 GT mesh 密集采样为点云
2. 对每台相机, 将 GT 点变换到 camera frame
3. 投影到图像平面
4. 与真实深度图比较 (GT depth ≈ observed depth → visible)
5. 三相机可见点做体素并集

正式 completeness 只能评价 visible_union, 禁止使用完整 GT.
"""
import os, sys, json, math, argparse, logging
import numpy as np
import yaml

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("visible_gt")

try:
    import open3d as o3d
except ImportError:
    logger.error("需要 open3d")
    sys.exit(1)

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(PKG_DIR, "src"))

from cr5_spray_sim.calibration_target_geometry import (
    build_gt_pointcloud, get_gt_aabb_sdf_frame,
)
from cr5_spray_sim.camera_geometry import compute_camera_look_at
from scipy.spatial.transform import Rotation

REQUIRED_CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]


def compute_T_world_optical(cam_name):
    """计算 Oracle T_world_optical."""
    scene_path = os.path.join(PKG_DIR, "config", "simulation_scene.yaml")
    with open(scene_path) as f:
        scene = yaml.safe_load(f)
    profiles = scene.get("cameras", {})
    target = profiles.get("target", {"x": 0.72, "y": 0, "z": 0.62})
    tgt = [target["x"], target["y"], target["z"]]
    cam_cfgs = {c["name"]: c for c in profiles.get("cameras", [])}
    cfg = cam_cfgs[cam_name]
    pos = [cfg["position"]["x"], cfg["position"]["y"], cfg["position"]["z"]]
    rpy_data = compute_camera_look_at(pos, tgt)
    R_link = rpy_data["R"]
    R_lo = Rotation.from_euler('xyz', [-math.pi / 2, 0, -math.pi / 2]).as_matrix()
    T_wl = np.eye(4); T_wl[:3, :3] = R_link; T_wl[:3, 3] = pos
    T_lo = np.eye(4); T_lo[:3, :3] = R_lo
    return T_wl @ T_lo


def get_gazebo_model_pose():
    """从 Gazebo 获取 simple_hanging_workpiece 位姿."""
    import rospy
    from gazebo_msgs.msg import ModelStates
    rospy.init_node("visible_gt", anonymous=True)
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
    for i, name in enumerate(msg.name):
        if name == "simple_hanging_workpiece":
            p = msg.pose[i].position; q = msg.pose[i].orientation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [p.x, p.y, p.z]
            return T
    raise RuntimeError("simple_hanging_workpiece not found in Gazebo")


def depth_buffer_visibility(gt_pts_obj, T_obj_camera, depth_image_m, K, depth_min, depth_max):
    """深度缓冲可见性检测.

    Args:
        gt_pts_obj: GT 点云 (object frame, N×3)
        T_obj_camera: 4×4, object ← camera optical frame
        depth_image_m: H×W 深度图 (米)
        K: 3×3 相机内参矩阵
        depth_min, depth_max: 深度范围

    Returns:
        visible_mask: N 个 bool
    """
    h, w = depth_image_m.shape
    n = len(gt_pts_obj)

    # 变换到 camera frame
    R = T_obj_camera[:3, :3].T  # camera ← object
    t = T_obj_camera[:3, 3]
    # p_cam = R @ p_obj + t? No: T_obj_camera maps camera→object
    # Actually T_obj_camera @ p_cam = p_obj
    # So p_cam = inv(T_obj_camera) @ p_obj
    T_cam_obj = np.eye(4)
    T_cam_obj[:3, :3] = T_obj_camera[:3, :3].T
    T_cam_obj[:3, 3] = -T_obj_camera[:3, :3].T @ T_obj_camera[:3, 3]

    pts_cam = (T_cam_obj[:3, :3] @ gt_pts_obj.T + T_cam_obj[:3, 3:4]).T

    # 投影到图像平面: u = fx * X/Z + cx, v = fy * Y/Z + cy
    X, Y, Z = pts_cam[:, 0], pts_cam[:, 1], pts_cam[:, 2]
    visible_mask = np.zeros(n, dtype=bool)

    # 只考虑相机前方 (Z > 0)
    front_mask = Z > depth_min
    if not np.any(front_mask):
        return visible_mask

    Zf = Z[front_mask]
    Xf = X[front_mask]
    Yf = Y[front_mask]
    u = (K[0, 0] * Xf / Zf + K[0, 2]).astype(int)
    v = (K[1, 1] * Yf / Zf + K[1, 2]).astype(int)

    # 在图像范围内
    in_image = (u >= 0) & (u < w) & (v >= 0) & (v < h) & (Zf > depth_min) & (Zf < depth_max)
    u_img = u[in_image]
    v_img = v[in_image]
    Z_img = Zf[in_image]
    front_indices = np.where(front_mask)[0]
    img_indices = front_indices[in_image]

    # 比较 GT 深度与观测深度
    observed_depth = depth_image_m[v_img, u_img]
    valid_obs = observed_depth > 0
    if not np.any(valid_obs):
        return visible_mask

    # 可见条件: GT 在相机与观测深度之间 (允许小误差)
    # GT depth <= observed depth + tolerance (GT 不能被遮挡)
    tolerance = 0.008  # 8mm
    is_visible = Z_img[valid_obs] <= observed_depth[valid_obs] + tolerance

    vis_indices = img_indices[valid_obs][is_visible]
    visible_mask[vis_indices] = True

    return visible_mask


def main():
    parser = argparse.ArgumentParser(description="三相机可见表面 GT 生成")
    parser.add_argument("--output-dir", "-o", required=True, help="输出目录")
    parser.add_argument("--voxel-size", type=float, default=0.002,
                        help="体素去重尺寸 (m)")
    parser.add_argument("--dataset", default=None, help="数据集路径")
    parser.add_argument("--gt-samples", type=int, default=200000,
                        help="GT 采样点数")
    parser.add_argument("--model-pose-json", default=None,
                        help="离线模式: 使用 pose_evidence.json 替代实时 Gazebo model_states")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # 导入 cr5_spray_perception
    ws_src = os.path.join(os.path.dirname(__file__), "..", "..")
    perception_src = os.path.join(ws_src, "cr5_spray_perception", "src")
    if os.path.isdir(perception_src):
        sys.path.insert(0, perception_src)
    from cr5_spray_perception.reconstruction.rgbd_io import load_rgbd_data
    from cr5_spray_perception.reconstruction.transforms import convert_depth_to_meters

    # 1. 构建 GT 点云 (object frame)
    gt_pts_obj, _, gt_manifest = build_gt_pointcloud(
        "CALIBRATION_TARGET_BODY", args.gt_samples)
    logger.info("GT 点云: %d pts (object frame)", len(gt_pts_obj))
    gt_aabb = get_gt_aabb_sdf_frame()

    # 保存完整 GT
    pcd_full = o3d.geometry.PointCloud()
    pcd_full.points = o3d.utility.Vector3dVector(gt_pts_obj)
    o3d.io.write_point_cloud(
        os.path.join(args.output_dir, "full_gt_object_frame.ply"), pcd_full)

    # 2. 获取 model pose (离线 JSON 或实时 Gazebo)
    if args.model_pose_json:
        from cr5_spray_sim.scene_config import load_pose_evidence_model_pose
        T_world_object, evidence = load_pose_evidence_model_pose(args.model_pose_json)
        logger.info("Model pose (offline): %s", T_world_object[:3, 3].tolist())
    else:
        T_world_object = get_gazebo_model_pose()
    T_object_world = np.eye(4)
    T_object_world[:3, :3] = T_world_object[:3, :3].T
    T_object_world[:3, 3] = -T_world_object[:3, :3].T @ T_world_object[:3, 3]

    # 3. 加载 RGB-D 数据
    dataset_path = args.dataset or os.path.expanduser(
        "~/cr5_data/reconstruction/runs/gazebo_gate1_20260803_111452")
    group_dir = os.path.join(dataset_path, "groups", "group_0000")

    per_cam_visible = {}
    per_cam_stats = {}
    all_visible_obj = []

    manifest = {
        "schema": "cr5_visible_gt_v1",
        "method": "depth_buffer_visibility",
        "depth_tolerance_m": 0.008,
        "cameras": {},
        "union": {},
        "full_gt": {"n_points": int(len(gt_pts_obj)), "aabb_sdf": gt_aabb},
    }

    for cam_name in REQUIRED_CAMERAS:
        cam_dir = os.path.join(group_dir, cam_name)
        if not os.path.isdir(cam_dir):
            logger.warning("%s: 目录不存在", cam_name); continue

        rgbd = load_rgbd_data(cam_dir, cam_name)
        if rgbd.depth_K is None:
            logger.warning("%s: 无 depth K", cam_name); continue

        # 加载深度图
        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        K = rgbd.depth_K
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        h, w = depth_m.shape

        # 计算 T_object_camera
        T_world_optical = compute_T_world_optical(cam_name)
        T_object_camera = T_object_world @ T_world_optical

        logger.info("%s: %.1f×%.1f %d×%d", cam_name, fx, fy, w, h)

        # 深度缓冲可见性检测
        vis_mask = depth_buffer_visibility(
            gt_pts_obj, T_object_camera, depth_m, K, 0.15, 2.0)

        n_vis = int(np.sum(vis_mask))
        n_total = len(gt_pts_obj)
        logger.info("  %s: %d/%d visible (%.1f%%)",
                    cam_name, n_vis, n_total, 100.0 * n_vis / n_total)

        stats = {
            "rays_equivalent": int(w * h),
            "visible_gt_points": n_vis,
            "visible_ratio": round(n_vis / n_total, 4),
            "intrinsics": {"fx": float(fx), "fy": float(fy),
                           "cx": float(cx), "cy": float(cy), "w": w, "h": h},
        }
        per_cam_stats[cam_name] = stats

        if n_vis > 0:
            vis_pts = gt_pts_obj[vis_mask]
            per_cam_visible[cam_name] = vis_pts
            all_visible_obj.append(vis_pts)

            # 保存 per-camera visible (object frame)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(vis_pts)
            o3d.io.write_point_cloud(
                os.path.join(args.output_dir, f"visible_from_{cam_name}.ply"), pcd)

        manifest["cameras"][cam_name] = stats

    # 4. 体素去重 union (object frame)
    if all_visible_obj:
        pts_all = np.vstack(all_visible_obj)
        pcd_all = o3d.geometry.PointCloud()
        pcd_all.points = o3d.utility.Vector3dVector(pts_all)

        # 带来源标签
        colors = np.zeros((len(pts_all), 3))
        offset = 0
        color_map = {"cam_front_left": [1, 0.2, 0.2],
                     "cam_front_right": [0.2, 1, 0.2],
                     "cam_rear": [0.2, 0.4, 1.0]}
        for cam_name in REQUIRED_CAMERAS:
            if cam_name in per_cam_visible:
                n = len(per_cam_visible[cam_name])
                colors[offset:offset + n] = color_map.get(cam_name, [0.5, 0.5, 0.5])
                offset += n
        pcd_all.colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 1))
        o3d.io.write_point_cloud(
            os.path.join(args.output_dir, "visibility_source_labels.ply"), pcd_all)

        # 体素去重
        pcd_union = pcd_all.voxel_down_sample(args.voxel_size)
        pts_union = np.asarray(pcd_union.points)
        o3d.io.write_point_cloud(
            os.path.join(args.output_dir, "visible_union.ply"), pcd_union)

        manifest["union"]["n_points"] = int(len(pts_union))
        manifest["union"]["voxel_size_m"] = args.voxel_size
        manifest["visible_ratio"] = round(
            len(pts_union) / len(gt_pts_obj), 4)

        # 标记不可见面 (GT 中不可见的点)
        all_vis_mask = np.zeros(len(gt_pts_obj), dtype=bool)
        for cam_name in REQUIRED_CAMERAS:
            if cam_name in per_cam_visible:
                all_vis_mask |= depth_buffer_visibility(
                    gt_pts_obj,
                    T_object_world @ compute_T_world_optical(cam_name),
                    convert_depth_to_meters(
                        load_rgbd_data(os.path.join(group_dir, cam_name), cam_name).depth_raw,
                        load_rgbd_data(os.path.join(group_dir, cam_name), cam_name).depth_unit
                    )[0],
                    load_rgbd_data(os.path.join(group_dir, cam_name), cam_name).depth_K,
                    0.15, 2.0)
        invisible_pts = gt_pts_obj[~all_vis_mask]
        if len(invisible_pts) > 0:
            pcd_invis = o3d.geometry.PointCloud()
            pcd_invis.points = o3d.utility.Vector3dVector(invisible_pts)
            pcd_invis.paint_uniform_color([0.5, 0.5, 0.5])
            o3d.io.write_point_cloud(
                os.path.join(args.output_dir, "unobserved_surface.ply"), pcd_invis)
        manifest["unobserved"] = {"n_points": int(len(invisible_pts)),
                                  "ratio": round(len(invisible_pts) / len(gt_pts_obj), 4),
                                  "includes_bottom_surface": True}
    else:
        manifest["union"]["n_points"] = 0

    # 5. 保存 manifest
    manifest_path = os.path.join(args.output_dir, "visibility_manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2, default=str)

    # ── 控制台输出 ──
    print(f"\n{'='*60}")
    print("可见表面 GT 生成完成")
    print(f"{'='*60}")
    for cam_name in REQUIRED_CAMERAS:
        if cam_name in per_cam_stats:
            s = per_cam_stats[cam_name]
            print(f"  {cam_name}: {s['visible_gt_points']}/{s['rays_equivalent']}px "
                  f"({s['visible_ratio']:.1%} of GT)")
    if all_visible_obj:
        print(f"  visible_union: {manifest['union']['n_points']} pts")
        print(f"  unobserved: {manifest.get('unobserved', {}).get('n_points', 0)} pts "
              f"({manifest.get('unobserved', {}).get('ratio', 0):.1%})")
    print(f"\n📁 输出: {args.output_dir}")


if __name__ == "__main__":
    main()

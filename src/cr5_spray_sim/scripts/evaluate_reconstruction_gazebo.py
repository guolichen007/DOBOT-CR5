#!/usr/bin/env python3
"""
CR5 Simulation — Gazebo GT 三维重建评价.

从 Gazebo scene truth 构建 calibration_target GT mesh,
与 TSDF 重建结果对比, 计算 accuracy/completeness/Chamfer/coverage.

仅用于离线评价, 不参与生产重建链路.
"""
import os, sys, json, yaml, math, argparse, logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("evaluate_gazebo")

try:
    import open3d as o3d
except ImportError:
    logger.error("需要 open3d")
    sys.exit(1)

PKG_DIR = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(PKG_DIR, "src"))

# Rig frame
RIG_FRAME = "cam_front_left_color_optical_frame"


def load_scene_config():
    scene_path = os.path.join(PKG_DIR, "config", "simulation_scene.yaml")
    with open(scene_path) as f:
        return yaml.safe_load(f)


def build_cylinder_gt_mesh(target_type_cfg, n_samples=50000):
    """从 motor_housing_cylinder 配置构建 GT 点云 (在 object frame).

    object frame: Y 轴为圆柱主轴, XZ 为径向.
    """
    body_len = target_type_cfg.get("body", {}).get("length_y", 0.36)
    body_r = target_type_cfg.get("body", {}).get("radius", 0.105)
    left_cap_len = target_type_cfg.get("left_end_cap", {}).get("length_y", 0.035)
    left_cap_r = target_type_cfg.get("left_end_cap", {}).get("radius", 0.115)
    right_cap_len = target_type_cfg.get("right_end_cap", {}).get("length_y", 0.045)
    right_cap_r = target_type_cfg.get("right_end_cap", {}).get("radius", 0.115)
    shaft_len = target_type_cfg.get("shaft", {}).get("length_y", 0.055)
    shaft_r = target_type_cfg.get("shaft", {}).get("radius", 0.040)

    all_points = []
    np.random.seed(42)

    def sample_cylinder(length, radius, y_center, n):
        """在 Y 轴上采样圆柱表面点."""
        # 侧面
        n_side = int(n * 0.85)
        theta = np.random.uniform(0, 2 * np.pi, n_side)
        y = np.random.uniform(-length / 2, length / 2, n_side) + y_center
        x = radius * np.cos(theta)
        z = radius * np.sin(theta)
        side_pts = np.column_stack([x, y, z])

        # 两端
        n_cap = int(n * 0.15)
        r_cap = np.sqrt(np.random.uniform(0, radius**2, n_cap))
        theta_cap = np.random.uniform(0, 2 * np.pi, n_cap)
        x_cap = r_cap * np.cos(theta_cap)
        z_cap = r_cap * np.sin(theta_cap)
        cap1 = np.column_stack([x_cap, np.full(n_cap, -length / 2 + y_center), z_cap])
        cap2 = np.column_stack([x_cap, np.full(n_cap, length / 2 + y_center), z_cap])
        return np.vstack([side_pts, cap1, cap2])

    # Body center at y=0
    all_points.append(sample_cylinder(body_len, body_r, 0.0, int(n_samples * 0.5)))

    # Left end cap
    left_y = -body_len / 2 - left_cap_len / 2
    all_points.append(sample_cylinder(left_cap_len, left_cap_r, left_y, int(n_samples * 0.15)))

    # Right end cap
    right_y = body_len / 2 + right_cap_len / 2
    all_points.append(sample_cylinder(right_cap_len, right_cap_r, right_y, int(n_samples * 0.15)))

    # Shaft (at right side)
    shaft_y = body_len / 2 + right_cap_len + shaft_len / 2
    all_points.append(sample_cylinder(shaft_len, shaft_r, shaft_y, int(n_samples * 0.1)))

    # Left shaft (symmetry)
    shaft_y2 = -body_len / 2 - left_cap_len - shaft_len / 2
    all_points.append(sample_cylinder(shaft_len, shaft_r, shaft_y2, int(n_samples * 0.1)))

    pts = np.vstack(all_points)
    return pts


def get_gazebo_model_pose(model_name="simple_hanging_workpiece"):
    """从 Gazebo 获取模型位姿."""
    import rospy
    from gazebo_msgs.msg import ModelStates
    rospy.init_node("gt_eval", anonymous=True)
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
    for i, name in enumerate(msg.name):
        if name == model_name:
            p = msg.pose[i].position
            q = msg.pose[i].orientation
            return {"position": [p.x, p.y, p.z],
                    "orientation": [q.x, q.y, q.z, q.w]}
    raise RuntimeError(f"模型 {model_name} 未找到. 可用: {msg.name}")


def compute_T_world_rig():
    """计算 T_world_rig (world → FL optical frame) 从 scene config."""
    from cr5_spray_sim.camera_geometry import compute_camera_look_at

    scene = load_scene_config()
    profiles = scene.get("cameras", {})
    target = profiles.get("target", {"x": 0.72, "y": 0, "z": 0.62})
    tgt = [target["x"], target["y"], target["z"]]

    cam_cfgs = {c["name"]: c for c in profiles.get("cameras", [])}
    fl_cfg = cam_cfgs["cam_front_left"]
    pos_fl = [fl_cfg["position"]["x"], fl_cfg["position"]["y"], fl_cfg["position"]["z"]]
    rpy_fl = compute_camera_look_at(pos_fl, tgt)

    from scipy.spatial.transform import Rotation
    R_link = rpy_fl["R"]
    R_lo = Rotation.from_euler('xyz', [-math.pi/2, 0, -math.pi/2]).as_matrix()

    T_wl = np.eye(4)
    T_wl[:3, :3] = R_link
    T_wl[:3, 3] = pos_fl

    T_lo = np.eye(4)
    T_lo[:3, :3] = R_lo

    return T_wl @ T_lo


def quat_to_matrix(q):
    """四元数 [x,y,z,w] → 旋转矩阵."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


def crop_mesh_to_roi(mesh, roi_min, roi_max):
    """裁剪 mesh 到 ROI 范围."""
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    mask = np.all(vertices >= roi_min, axis=1) & np.all(vertices <= roi_max, axis=1)
    if not np.any(mask):
        logger.warning("ROI 裁剪后无顶点!")
        return mesh

    # 保留使用了 ROI 内顶点的所有三角形
    kept_vert_idx = np.where(mask)[0]
    kept_set = set(kept_vert_idx)
    tri_mask = np.array([all(v in kept_set for v in tri) for tri in triangles])

    if not np.any(tri_mask):
        logger.warning("ROI 裁剪后无三角形!")
        return mesh

    # Remap
    new_verts = vertices[kept_vert_idx]
    old_to_new = {old: new for new, old in enumerate(kept_vert_idx)}
    new_tris = np.array([[old_to_new[t[0]], old_to_new[t[1]], old_to_new[t[2]]]
                         for t in triangles[tri_mask]])

    cropped = o3d.geometry.TriangleMesh()
    cropped.vertices = o3d.utility.Vector3dVector(new_verts)
    cropped.triangles = o3d.utility.Vector3iVector(new_tris)
    if mesh.has_vertex_colors():
        cropped.vertex_colors = o3d.utility.Vector3dVector(
            np.asarray(mesh.vertex_colors)[kept_vert_idx])
    return cropped


def evaluate_reconstruction(recon_mesh_path, gt_points_rig, output_dir, target_roi=None,
                            sample_n=50000, thresholds_mm=(5, 10, 20, 30)):
    """评价重建 mesh vs GT 点云.

    Args:
        recon_mesh_path: TSDF 重建的 mesh 路径
        gt_points_rig: GT 点云 (rig frame)
        output_dir: 输出目录
        sample_n: 评价采样点数
        thresholds_mm: coverage 阈值

    Returns:
        dict: 评价指标
    """
    os.makedirs(output_dir, exist_ok=True)

    # 加载重建 mesh
    mesh = o3d.io.read_triangle_mesh(recon_mesh_path)
    logger.info("原始 mesh: %d verts, %d tris",
                len(mesh.vertices), len(mesh.triangles))

    # ROI 裁剪
    if target_roi is not None:
        roi_min = np.array(target_roi.get("min", [-0.5, -1, 0]))
        roi_max = np.array(target_roi.get("max", [2, 1, 2.5]))
        mesh = crop_mesh_to_roi(mesh, roi_min, roi_max)
        logger.info("ROI 裁剪后: %d verts, %d tris",
                    len(mesh.vertices), len(mesh.triangles))
        # 保存裁剪后的 mesh
        o3d.io.write_triangle_mesh(
            os.path.join(output_dir, "recon_mesh_roi_cropped.ply"), mesh)

    recon_pcd = mesh.sample_points_uniformly(number_of_points=sample_n)
    recon_pts = np.asarray(recon_pcd.points)

    # 对 GT 降采样
    if len(gt_points_rig) > sample_n:
        idx = np.random.RandomState(42).choice(len(gt_points_rig), sample_n, replace=False)
        gt_pts = gt_points_rig[idx]
    else:
        gt_pts = gt_points_rig

    logger.info("Recon: %d pts, GT: %d pts", len(recon_pts), len(gt_pts))

    # ── Accuracy: recon → GT ──
    gt_pcd = o3d.geometry.PointCloud()
    gt_pcd.points = o3d.utility.Vector3dVector(gt_pts)
    gt_tree = o3d.geometry.KDTreeFlann(gt_pcd)

    acc_dists = []
    for pt in recon_pts:
        _, idx, dist2 = gt_tree.search_knn_vector_3d(pt, 1)
        acc_dists.append(np.sqrt(dist2[0]))
    acc_dists = np.array(acc_dists) * 1000.0  # 转 mm

    # ── Completeness: GT (visible) → recon ──
    recon_pcd_o3d = o3d.geometry.PointCloud()
    recon_pcd_o3d.points = o3d.utility.Vector3dVector(recon_pts)
    recon_tree = o3d.geometry.KDTreeFlann(recon_pcd_o3d)

    comp_dists = []
    for pt in gt_pts:
        _, idx, dist2 = recon_tree.search_knn_vector_3d(pt, 1)
        comp_dists.append(np.sqrt(dist2[0]))
    comp_dists = np.array(comp_dists) * 1000.0

    # ── 统计 ──
    metrics = {
        "accuracy": {
            "mean_mm": float(np.mean(acc_dists)),
            "median_mm": float(np.median(acc_dists)),
            "p95_mm": float(np.percentile(acc_dists, 95)),
            "rmse_mm": float(np.sqrt(np.mean(acc_dists**2))),
            "max_mm": float(np.max(acc_dists)),
        },
        "completeness": {
            "mean_mm": float(np.mean(comp_dists)),
            "median_mm": float(np.median(comp_dists)),
            "p95_mm": float(np.percentile(comp_dists, 95)),
            "rmse_mm": float(np.sqrt(np.mean(comp_dists**2))),
            "max_mm": float(np.max(comp_dists)),
        },
        "chamfer_mm": float((np.mean(acc_dists) + np.mean(comp_dists)) / 2.0),
        "coverage": {},
        "n_recon_points": int(len(recon_pts)),
        "n_gt_points": int(len(gt_pts)),
    }

    for t_mm in thresholds_mm:
        metrics["coverage"][f"accuracy_{t_mm}mm"] = float(np.mean(acc_dists < t_mm))
        metrics["coverage"][f"completeness_{t_mm}mm"] = float(np.mean(comp_dists < t_mm))

    # ── 保存距离彩色 PLY ──
    # accuracy (recon → GT distances)
    acc_colors = np.zeros((len(recon_pts), 3))
    acc_clipped = np.clip(acc_dists / 50.0, 0, 1)  # 0-50mm → 0-1
    acc_colors[:, 0] = acc_clipped  # red = far
    acc_colors[:, 1] = 1 - acc_clipped  # green = close
    acc_pcd = o3d.geometry.PointCloud()
    acc_pcd.points = o3d.utility.Vector3dVector(recon_pts)
    acc_pcd.colors = o3d.utility.Vector3dVector(acc_colors)
    o3d.io.write_point_cloud(
        os.path.join(output_dir, "reconstruction_to_gt_distances.ply"), acc_pcd)

    # completeness (GT → recon distances)
    comp_colors = np.zeros((len(gt_pts), 3))
    comp_clipped = np.clip(comp_dists / 50.0, 0, 1)
    comp_colors[:, 0] = comp_clipped
    comp_colors[:, 1] = 1 - comp_clipped
    comp_pcd = o3d.geometry.PointCloud()
    comp_pcd.points = o3d.utility.Vector3dVector(gt_pts)
    comp_pcd.colors = o3d.utility.Vector3dVector(comp_colors)
    o3d.io.write_point_cloud(
        os.path.join(output_dir, "gt_to_reconstruction_distances.ply"), comp_pcd)

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Gazebo GT 重建评价")
    parser.add_argument("--recon-mesh", "-r", required=True,
                        help="TSDF 重建 mesh 路径")
    parser.add_argument("--output-dir", "-o", required=True,
                        help="输出目录")
    parser.add_argument("--gt-samples", type=int, default=50000,
                        help="GT 采样点数")
    parser.add_argument("--eval-samples", type=int, default=50000,
                        help="评价采样点数")
    parser.add_argument("--label", default="stable_v1",
                        help="标签 (stable_v1 / oracle)")
    parser.add_argument("--target-roi-min", nargs=3, type=float,
                        default=[-0.262, -0.280, 0.661],
                        help="目标 ROI min x y z (rig frame)")
    parser.add_argument("--target-roi-max", nargs=3, type=float,
                        default=[0.251, 0.180, 1.205],
                        help="目标 ROI max x y z (rig frame)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # 1. 构建 GT 点云 (object frame)
    scene = load_scene_config()
    target_cfg = scene.get("simple_hanging_workpiece", {})
    target_type = target_cfg.get("default_type", "motor_housing_cylinder")
    type_cfg = target_cfg.get("types", {}).get(target_type, {})
    gt_pts_obj = build_cylinder_gt_mesh(type_cfg, n_samples=args.gt_samples)
    logger.info("GT 点云 (object frame): %d pts", len(gt_pts_obj))

    # 保存 canonical GT
    gt_obj_pcd = o3d.geometry.PointCloud()
    gt_obj_pcd.points = o3d.utility.Vector3dVector(gt_pts_obj)
    gt_obj_path = os.path.join(args.output_dir, "canonical_calibration_target_gt.ply")
    o3d.io.write_point_cloud(gt_obj_path, gt_obj_pcd)

    # 2. 获取 Gazebo model pose
    model_pose = get_gazebo_model_pose("simple_hanging_workpiece")
    T_world_object = np.eye(4)
    T_world_object[:3, :3] = quat_to_matrix(model_pose["orientation"])
    T_world_object[:3, 3] = model_pose["position"]

    # 3. 获取 T_world_rig
    T_world_rig = compute_T_world_rig()
    T_rig_world = np.eye(4)
    T_rig_world[:3, :3] = T_world_rig[:3, :3].T
    T_rig_world[:3, 3] = -T_world_rig[:3, :3].T @ T_world_rig[:3, 3]

    # 4. transform GT to rig frame
    T_rig_object = T_rig_world @ T_world_object
    gt_pts_rig = (T_rig_object[:3, :3] @ gt_pts_obj.T + T_rig_object[:3, 3:4]).T
    logger.info("GT 点云 (rig frame): %d pts", len(gt_pts_rig))

    # 保存 rig-frame GT
    gt_rig_pcd = o3d.geometry.PointCloud()
    gt_rig_pcd.points = o3d.utility.Vector3dVector(gt_pts_rig)
    o3d.io.write_point_cloud(
        os.path.join(args.output_dir, "gt_rig_frame.ply"), gt_rig_pcd)

    # 5. 评价 (使用 target ROI 裁剪重建 mesh)
    target_roi = {"min": list(args.target_roi_min), "max": list(args.target_roi_max)}
    metrics = evaluate_reconstruction(
        args.recon_mesh, gt_pts_rig, args.output_dir,
        target_roi=target_roi, sample_n=args.eval_samples)

    # 6. 保存
    eval_result = {
        "schema": "cr5_gazebo_gt_evaluation_v1",
        "label": args.label,
        "recon_mesh": os.path.abspath(args.recon_mesh),
        "target_type": target_type,
        "gt_samples": args.gt_samples,
        "eval_samples": args.eval_samples,
        "model_pose_gazebo": {
            "position": model_pose["position"],
            "orientation_xyzw": model_pose["orientation"],
        },
        "metrics": metrics,
    }

    metrics_path = os.path.join(args.output_dir, f"{args.label}_metrics.json")
    with open(metrics_path, "w") as f:
        json.dump(eval_result, f, indent=2, default=str)

    # ── 输出 ──
    print(f"\n{'='*60}")
    print(f"GT 评价: {args.label}")
    print(f"{'='*60}")
    print(f"Accuracy:   median={metrics['accuracy']['median_mm']:.2f}mm, "
          f"P95={metrics['accuracy']['p95_mm']:.2f}mm, "
          f"RMSE={metrics['accuracy']['rmse_mm']:.2f}mm")
    print(f"Completeness: median={metrics['completeness']['median_mm']:.2f}mm, "
          f"P95={metrics['completeness']['p95_mm']:.2f}mm")
    print(f"Chamfer: {metrics['chamfer_mm']:.2f}mm")
    for k, v in metrics["coverage"].items():
        print(f"  {k}: {v:.3f}")
    print(f"\n📁 指标: {metrics_path}")


if __name__ == "__main__":
    main()

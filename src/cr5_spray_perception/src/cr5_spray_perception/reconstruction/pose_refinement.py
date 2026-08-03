"""
CR5 Reconstruction — 受约束位姿细化.

以 Stable V1 为强先验, 仅使用 RGB-D 目标表面点云估计微小外参修正.
FL anchor = identity, 只优化 FR/RE.

禁止: Gazebo truth, Oracle, GT mesh, 回写 pairwise_solver.
"""
import logging
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

logger = logging.getLogger(__name__)

# ── 约束 ──
MAX_TRANSLATION_NORM_M = 0.025    # 25 mm
MAX_ROTATION_ANGLE_DEG = 1.5      # 1.5°
MAX_TRANSLATION_NORM_M = 0.025
MAX_ROTATION_RAD = np.deg2rad(1.5)
MAX_STEP_TRANS_M = 0.002          # 2 mm per iteration
MAX_STEP_ROT_RAD = np.deg2rad(0.15)  # 0.15° per iteration
MAX_ITERATIONS = 30
HUBER_DELTA_M = 0.010             # 10 mm
MAX_CORRESPONDENCE_DIST_M = 0.030 # 30 mm initial
MIN_CORRESPONDENCE_DIST_M = 0.015 # 15 mm final
MIN_CORRESPONDENCES = 100
PRIOR_TRANS_WEIGHT = 100.0        # 1/m — prior strength
PRIOR_ROT_WEIGHT = 500.0          # 1/rad
CLOSURE_WEIGHT = 200.0
LOW_OVERLAP_RATIO = 0.02          # FL-RE low overlap threshold


def se3_to_params(T):
    """SE(3) → [tx, ty, tz, rx, ry, rz] (旋转向量)."""
    t = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3])
    rvec = r.as_rotvec()
    return np.concatenate([t, rvec])


def params_to_se3(params):
    """[tx, ty, tz, rx, ry, rz] → SE(3)."""
    t = params[:3]
    rvec = params[3:6]
    R = Rotation.from_rotvec(rvec).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def compute_normals(points, k=20):
    """估计点云法向 (使用 Open3D)."""
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(k))
        return np.asarray(pcd.normals)
    except ImportError:
        return np.zeros_like(points)


def find_correspondences(points_source, points_target, normals_target,
                         max_dist_m=0.030):
    """最近邻匹配 + 距离/法向筛选."""
    from scipy.spatial import KDTree
    if len(points_source) == 0 or len(points_target) == 0:
        return np.array([]), np.array([]), np.array([]), np.array([])

    tree = KDTree(points_target)
    dists, idx = tree.query(points_source)

    valid = dists < max_dist_m
    if not np.any(valid):
        return np.array([]), np.array([]), np.array([]), np.array([])

    src = points_source[valid]
    tgt = points_target[idx[valid]]
    nrm = normals_target[idx[valid]]
    return src, tgt, nrm, dists[valid]


def build_refinement_residual(x, source_points, target_points, target_normals,
                               T_stable, prior_trans_weight, prior_rot_weight):
    """构建优化残差.

    x = [tx, ty, tz, rx, ry, rz] — Delta params (FR or RE)
    残差 = [surface_residuals, prior_trans, prior_rot]
    """
    T_delta = params_to_se3(x)
    T_refined = T_delta @ T_stable

    n = len(source_points)
    residuals = []

    # Surface residuals: n_i^T · (T_refined · p_src - p_tgt)
    for i in range(n):
        p_src_h = np.append(source_points[i], 1.0)
        p_transformed = T_refined @ p_src_h
        r = np.dot(target_normals[i], p_transformed[:3] - target_points[i])
        residuals.append(r)

    # Prior: delta should be small
    t_norm = np.linalg.norm(x[:3])
    r_norm = np.linalg.norm(x[3:6])
    residuals.append(prior_trans_weight * t_norm)
    residuals.append(prior_rot_weight * r_norm)

    return np.array(residuals)


def check_limits(x, max_trans, max_rot):
    """检查修正量是否在限制内."""
    t_norm = np.linalg.norm(x[:3])
    r_norm = np.linalg.norm(x[3:6])
    t_ok = t_norm <= max_trans * 1.01  # 1% tolerance
    r_ok = r_norm <= max_rot * 1.01
    return t_ok and r_ok, {"trans_norm_mm": t_norm * 1000,
                           "rot_norm_deg": np.degrees(r_norm)}


def compute_triangle_closure(T_fl_fr, T_fl_re, T_fr_re):
    """计算三角闭环误差.

    T_fr_re 应该 = inv(T_fl_fr) @ T_fl_re
    """
    expected = np.linalg.inv(T_fl_fr) @ T_fl_re
    diff = expected @ np.linalg.inv(T_fr_re)
    t_err = np.linalg.norm(diff[:3, 3]) * 1000  # mm
    r_err = np.arccos(np.clip((np.trace(diff[:3, :3]) - 1) / 2, -1, 1))
    return t_err, np.degrees(r_err)


def refine_camera_pair(source_points, target_points, target_normals,
                       T_rig_source_stable, T_rig_target_stable,
                       max_dist_m=0.030, low_overlap=False):
    """细化一个相机对的外参.

    Args:
        source_points: 源相机点云 (rig frame)
        target_points: 目标相机点云 (rig frame)
        target_normals: 目标相机法向
        T_rig_source_stable: Stable V1 T_rig_source
        T_rig_target_stable: Stable V1 T_rig_target
        max_dist_m: 最大 correspondence 距离
        low_overlap: 是否低重叠 (降低权重)

    Returns:
        dict with delta, refined_T, residuals, stats
    """
    src, tgt, nrm, dists = find_correspondences(
        source_points, target_points, target_normals, max_dist_m)

    n_corr = len(src)
    if n_corr < MIN_CORRESPONDENCES:
        return {"status": "FAIL", "reason": f"insufficient_correspondences ({n_corr})",
                "n_correspondences": n_corr}

    # 初始残差
    res_before = np.array([np.dot(nrm[i], source_points[i] if i < len(source_points) else np.zeros(3) - tgt[i])
                          for i in range(min(10, n_corr))])
    res_before_stats = {"median_mm": np.median(np.abs(res_before)) * 1000,
                        "mean_mm": np.mean(np.abs(res_before)) * 1000}

    # 优化权重
    pt_weight = PRIOR_TRANS_WEIGHT * (0.3 if low_overlap else 1.0)
    pr_weight = PRIOR_ROT_WEIGHT * (0.3 if low_overlap else 1.0)

    # 初始 Delta = 0
    x0 = np.zeros(6)

    try:
        result = least_squares(
            lambda x: build_refinement_residual(
                x, src, tgt, nrm, T_rig_source_stable, pt_weight, pr_weight),
            x0, method='trf', loss='huber', f_scale=HUBER_DELTA_M,
            max_nfev=MAX_ITERATIONS, verbose=0)

        x_opt = result.x
        within_limits, limit_info = check_limits(x_opt, MAX_TRANSLATION_NORM_M, MAX_ROTATION_RAD)

        if not within_limits:
            return {"status": "FAIL", "reason": "limits_exceeded",
                    "limit_info": limit_info}

        T_delta = params_to_se3(x_opt)
        T_refined = T_delta @ T_rig_source_stable

        # 最终残差
        res_after = np.array([
            np.dot(nrm[i], (T_refined @ np.append(src[i], 1.0))[:3] - tgt[i])
            for i in range(n_corr)])
        res_after_stats = {"median_mm": np.median(np.abs(res_after)) * 1000,
                          "mean_mm": np.mean(np.abs(res_after)) * 1000,
                          "rmse_mm": np.sqrt(np.mean(res_after**2)) * 1000}

        return {
            "status": "PASS",
            "n_correspondences": n_corr,
            "x_opt": x_opt.tolist(),
            "T_delta": T_delta.tolist(),
            "T_refined": T_refined.tolist(),
            "translation_delta_mm": float(np.linalg.norm(x_opt[:3]) * 1000),
            "rotation_delta_deg": float(np.degrees(np.linalg.norm(x_opt[3:6]))),
            "residual_before": res_before_stats,
            "residual_after": res_after_stats,
            "cost": float(result.cost),
        }
    except Exception as e:
        return {"status": "FAIL", "reason": f"optimization_error: {e}"}


def refine_reconstruction_rig(rgbd_list, stable_rig, config):
    """为主重建运行位姿细化.

    Returns:
        dict with refined_rig, refinement_report
    """
    from cr5_spray_perception.reconstruction.extrinsics import get_T_rig_camera
    from cr5_spray_perception.reconstruction.transforms import (
        convert_depth_to_meters, depth_image_to_pointcloud, transform_pointcloud)

    depth_min = config.get("depth_min_m", 0.15)
    depth_max = config.get("depth_max_m", 2.0)
    target_roi = config.get("target_roi_rig", {})
    roi_min = np.array(target_roi.get("min", [-0.281, -0.216, 0.668]))
    roi_max = np.array(target_roi.get("max", [0.277, 0.179, 1.195]))
    voxel_size = 0.005

    rgbd_map = {r.camera_name: r for r in rgbd_list}
    cameras = ["cam_front_left", "cam_front_right", "cam_rear"]

    # 生成每台相机的目标表面点云 (rig frame)
    per_cam_clouds = {}
    per_cam_stats = {}
    for cam_name in cameras:
        if cam_name not in rgbd_map:
            continue
        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(stable_rig, cam_name)

        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        points_cam, valid_mask = depth_image_to_pointcloud(
            depth_m, rgbd.depth_K, depth_min, depth_max)

        if np.sum(valid_mask) == 0:
            continue

        points_rig = transform_pointcloud(points_cam, T_rc)
        # 裁剪到 target ROI
        in_roi = ((points_rig[:, 0] >= roi_min[0]) & (points_rig[:, 0] <= roi_max[0]) &
                  (points_rig[:, 1] >= roi_min[1]) & (points_rig[:, 1] <= roi_max[1]) &
                  (points_rig[:, 2] >= roi_min[2]) & (points_rig[:, 2] <= roi_max[2]))
        points_rig_roi = points_rig[in_roi]

        if len(points_rig_roi) == 0:
            continue

        # 体素下采样
        try:
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_rig_roi.astype(np.float64))
            pcd_down = pcd.voxel_down_sample(voxel_size)
            pts = np.asarray(pcd_down.points)
        except ImportError:
            pts = points_rig_roi

        n_pts = len(pts)
        if n_pts < 100:
            continue

        # 估计法向
        normals = compute_normals(pts)

        per_cam_clouds[cam_name] = {"points": pts, "normals": normals}
        per_cam_stats[cam_name] = {"n_points_roi": int(np.sum(in_roi)),
                                   "n_after_voxel": n_pts}
        logger.info("[%s] ROI pts=%d → voxel=%d",
                    cam_name, int(np.sum(in_roi)), n_pts)

    # ── 逐对细化 ──
    pairs = [
        ("cam_front_left", "cam_front_right", False),   # FL-FR
        ("cam_front_left", "cam_rear", True),            # FL-RE (low overlap)
        ("cam_front_right", "cam_rear", False),          # FR-RE
    ]

    pair_results = {}
    all_passed = True

    for src, tgt, low_overlap in pairs:
        if src not in per_cam_clouds or tgt not in per_cam_clouds:
            pair_results[f"{src}_{tgt}"] = {"status": "SKIP", "reason": "missing_cloud"}
            continue

        T_src = get_T_rig_camera(stable_rig, src)
        T_tgt = get_T_rig_camera(stable_rig, tgt)
        src_pts = per_cam_clouds[src]["points"]
        tgt_pts = per_cam_clouds[tgt]["points"]
        tgt_nrm = per_cam_clouds[tgt]["normals"]

        # 细化 src camera (T_rig_src)
        result = refine_camera_pair(
            src_pts, tgt_pts, tgt_nrm, T_src, T_tgt,
            low_overlap=low_overlap)

        result["source_camera"] = src
        result["target_camera"] = tgt
        result["low_overlap"] = low_overlap
        pair_results[f"{src}_{tgt}"] = result

        if result["status"] != "PASS":
            all_passed = False

    # ── 构建 refined rig ──
    # FL is identity (anchor)
    T_fl = np.eye(4)

    # FR: aggregate from FL-FR (preferred) or FR-RE×RE-FL
    fr_refined = None
    re_refined = None

    fl_fr = pair_results.get("cam_front_left_cam_front_right", {})
    fl_re = pair_results.get("cam_front_left_cam_rear", {})
    fr_re = pair_results.get("cam_front_right_cam_rear", {})

    if fl_fr.get("status") == "PASS":
        fr_refined = np.array(fl_fr["T_refined"])
    else:
        fr_refined = get_T_rig_camera(stable_rig, "cam_front_right")

    if fl_re.get("status") == "PASS":
        re_refined = np.array(fl_re["T_refined"])
    else:
        re_refined = get_T_rig_camera(stable_rig, "cam_rear")

    # 三角闭环
    T_fr = fr_refined
    T_re = re_refined
    closure_t, closure_r = compute_triangle_closure(T_fl, T_fr, T_re)

    # 构建 refined rig (calibrated_rig 兼容格式)
    refined_rig = {
        "schema_version": "cr5_reconstruction_refined_rig_v1",
        "status": "PASS" if all_passed else "DEGRADED",
        "units": "meter",
        "rig_frame": stable_rig.get("rig_frame", "cam_front_left_color_optical_frame"),
        "source_calibration": {
            "file": "derived_from_stable_v1_rgbd_overlap",
            "solver": "bounded_rgbd_pairwise_refinement",
            "version": "refined-v1",
            "n_cameras": 3,
            "base_stable_v1_sha256": stable_rig.get("source_calibration", {}).get("sha256", "N/A"),
            "anchor_camera": "cam_front_left",
            "uses_gazebo_truth": False,
            "uses_oracle": False,
            "uses_gt_mesh": False,
            "production_scope": "current_reconstruction_run_only",
            "modifies_stable_v1": False,
        },
        "transform_contract": stable_rig.get("transform_contract", {}),
        "cameras": {},
    }

    for cam_name in cameras:
        T_stable = get_T_rig_camera(stable_rig, cam_name)
        if cam_name == "cam_front_left":
            T_refined = np.eye(4)
            delta = np.zeros(6)
        elif cam_name == "cam_front_right":
            T_refined = fr_refined
            delta = se3_to_params(np.array(fl_fr.get("T_delta", np.eye(4))) if fl_fr.get("status") == "PASS" else np.eye(4))
        else:  # cam_rear
            T_refined = re_refined
            delta = se3_to_params(np.array(fl_re.get("T_delta", np.eye(4))) if fl_re.get("status") == "PASS" else np.eye(4))

        T_cr = np.linalg.inv(T_refined)
        refined_rig["cameras"][cam_name] = {
            "optical_frame": f"{cam_name}_color_optical_frame",
            "T_rig_camera": T_refined.tolist(),
            "T_camera_rig": T_cr.tolist(),
            "T_rig_camera_stable": T_stable.tolist(),
            "delta_T_camera": params_to_se3(delta).tolist() if not np.allclose(delta, 0) else np.eye(4).tolist(),
            "translation_delta_mm": float(np.linalg.norm(delta[:3]) * 1000),
            "rotation_delta_deg": float(np.degrees(np.linalg.norm(delta[3:6]))),
        }

    report = {
        "schema": "cr5_refinement_report_v1",
        "status": "PASS" if all_passed else "FAIL",
        "anchor_camera": "cam_front_left",
        "per_camera_stats": per_cam_stats,
        "pair_results": {k: {sk: sv for sk, sv in v.items() if sk != "T_delta" and sk != "T_refined" and sk != "x_opt"}
                        for k, v in pair_results.items()},
        "triangle_closure": {
            "before_mm": float("nan"), "after_mm": closure_t,
            "after_deg": closure_r,
        },
        "limits": {
            "max_translation_mm": MAX_TRANSLATION_NORM_M * 1000,
            "max_rotation_deg": MAX_ROTATION_ANGLE_DEG,
            "within_limits": all_passed,
        },
    }

    return refined_rig, report, pair_results

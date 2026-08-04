#!/usr/bin/env python3
"""
Rig 初始化工具 — Phase 1 (PnP Audit) + Phase 2 (Pose Graph Init).

用法:
  # Phase 1: 审计所有 group 的 PnP 质量和成对相机变换一致性
  rosrun cr5_spray_perception rig_initialization.py audit \
    --observations <path> --output <dir>

  # Phase 2: 构建 3-camera robust pose graph 初始化
  rosrun cr5_spray_perception rig_initialization.py init \
    --observations <path> --output <dir>
"""
import argparse, json, math, os, sys, yaml
import numpy as np


CAMERAS = ["cam_front_left", "cam_front_right", "cam_rear"]
PAIRS = [
    ("cam_front_left", "cam_front_right"),
    ("cam_front_left", "cam_rear"),
    ("cam_front_right", "cam_rear"),
]

# ── 质量门限 ──
PNP_RMSE_GOOD = 3.0       # px
PNP_RMSE_REJECT = 6.0     # px (hard reject)
MIN_POINTS_GOOD = 12
TRANSLATION_MAD_LIMIT_MM = 15.0
ROTATION_MAD_LIMIT_DEG = 1.5
MIN_SUPPORTING_GROUPS = 6

# ── SE3 工具函数 ──


def _euler_matrix(ai, aj, ak):
    """tf.transformations.euler_matrix 等价实现."""
    from math import cos, sin
    Rx = np.array([[1, 0, 0], [0, cos(ai), -sin(ai)], [0, sin(ai), cos(ai)]])
    Ry = np.array([[cos(aj), 0, sin(aj)], [0, 1, 0], [-sin(aj), 0, cos(aj)]])
    Rz = np.array([[cos(ak), -sin(ak), 0], [sin(ak), cos(ak), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quaternion_from_matrix(T):
    """从 4x4 旋转矩阵提取四元数 [x,y,z,w]."""
    R = np.asarray(T[:3, :3], dtype=np.float64)
    q = np.empty(4)
    t = R.trace()
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return [float(v) for v in q]


def qt_to_T(qt):
    """[qw,qx,qy,qz,tx,ty,tz] → 4×4 变换矩阵."""
    qw, qx, qy, qz = qt[0:4]
    tx, ty, tz = qt[4:7]
    T = np.eye(4)
    T[:3, :3] = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2],
    ])
    T[:3, 3] = [tx, ty, tz]
    return T


def T_to_qt(T):
    """4×4 变换矩阵 → [qw,qx,qy,qz,tx,ty,tz]."""
    q = _quaternion_from_matrix(T)
    return [q[3], q[0], q[1], q[2],
            float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]


def se3_log(T):
    """SE(3) 对数映射: T → 6D 切空间向量 [rx,ry,rz,tx,ty,tz]."""
    R = T[:3, :3]
    t = T[:3, 3]
    tr = (np.trace(R) - 1.0) / 2.0
    tr = np.clip(tr, -1.0, 1.0)
    theta = math.acos(tr)
    if theta < 1e-12:
        return np.array([0.0, 0.0, 0.0, t[0], t[1], t[2]])
    s = theta / (2.0 * math.sin(theta))
    rx = s * (R[2, 1] - R[1, 2])
    ry = s * (R[0, 2] - R[2, 0])
    rz = s * (R[1, 0] - R[0, 1])
    return np.array([rx, ry, rz, t[0], t[1], t[2]])


def se3_distance_mm_deg(T1, T2, sigma_t_mm=30.0, sigma_r_deg=3.0):
    """归一化 SE(3) 距离 (无量纲)."""
    dT = np.linalg.inv(T1) @ T2
    log = se3_log(dT)
    d_t = np.linalg.norm(log[3:6]) * 1000.0  # mm
    d_r = np.linalg.norm(log[0:3])  # rad
    d_r_deg = math.degrees(d_r)
    return d_t, d_r_deg, math.sqrt((d_t / sigma_t_mm)**2 + (d_r_deg / sigma_r_deg)**2)


def rotation_distance_deg(R1, R2):
    """SO(3) 测地距离 (度)."""
    c = (np.trace(R2.T @ R1) - 1.0) / 2.0
    c = np.clip(c, -1.0, 1.0)
    return math.degrees(math.acos(c))


# ── PnP 相关 ──


def _load_calib_module():
    """通过 importlib 加载 run_multi_frame_calibration 模块."""
    import importlib.util
    script_path = os.path.join(os.path.dirname(__file__),
                               "run_multi_frame_calibration.py")
    if not os.path.isfile(script_path):
        raise FileNotFoundError(
            "Cannot find run_multi_frame_calibration.py at {}".format(script_path))
    spec = importlib.util.spec_from_file_location(
        "multi_frame_calibration", script_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_observations(path):
    """加载观测数据 (支持 ceres JSON 和 accumulated_observations YAML)."""
    ext = os.path.splitext(path)[1].lower()
    with open(path, "r") as f:
        if ext in (".yaml", ".yml"):
            return yaml.safe_load(f)
        else:
            return json.load(f)


def _extract_groups(data):
    """从观测数据中提取 per-group per-camera 的 obj_pts/img_pts.

    支持两种格式:
    1. accumulated_observations.yaml: cameras + observations (dict keyed by group id)
    2. ceres JSON: cameras + targets + observations (flattened arrays)

    Returns:
        groups: {group_id: {cam_name: {obj_pts: [[x,y,z],...], img_pts: [[u,v],...]}}}
        cameras_info: {cam_name: {fx, fy, cx, cy, width, height}}
    """
    groups = {}
    cameras_info = {}

    # 解析相机内参
    if "cameras" in data:
        cam_list = data["cameras"]
        if isinstance(cam_list, list):
            # ceres JSON format
            for c in cam_list:
                cameras_info[c["name"]] = {
                    "fx": c.get("fx", 0), "fy": c.get("fy", 0),
                    "cx": c.get("cx", 0), "cy": c.get("cy", 0),
                }
        elif isinstance(cam_list, dict):
            # accumulated_observations.yaml format
            for name, info in cam_list.items():
                K = info.get("K", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                K_arr = np.array(K).reshape(3, 3)
                cameras_info[name] = {
                    "fx": float(K_arr[0, 0]), "fy": float(K_arr[1, 1]),
                    "cx": float(K_arr[0, 2]), "cy": float(K_arr[1, 2]),
                }

    # 解析观测
    obs_data = data.get("observations", data.get("groups", {}))
    targets_data = data.get("targets", [])

    if isinstance(obs_data, dict):
        # accumulated_observations.yaml: {group_id: {cam: {obj_pts, img_pts}}}
        for gid_str, gdata in obs_data.items():
            gid = int(gid_str) if not isinstance(gid_str, int) else gid_str
            groups[gid] = {}
            for cam_name in CAMERAS:
                cd = gdata.get(cam_name, {})
                if not cd:
                    continue
                obj_pts = cd.get("object_points_3d", [])
                img_pts = cd.get("image_points_2d", [])
                if obj_pts and img_pts:
                    groups[gid][cam_name] = {
                        "obj_pts": obj_pts,
                        "img_pts": img_pts,
                        "n_pts": len(obj_pts),
                    }
    elif isinstance(obs_data, list):
        # ceres JSON: observations 是扁平数组, 需要按 (camera_idx, target_idx) 分组
        # 建立 camera_idx → cam_name 映射
        idx_to_cam = {}
        for c in data.get("cameras", []):
            idx_to_cam[c.get("camera_idx", len(idx_to_cam))] = c.get("name", "")
        # 实际上 ceres JSON 的 cameras 是数组, 顺序就是 idx
        cam_names_list = [c["name"] for c in data.get("cameras", [])]

        # 建立 target_idx → group_id 映射
        tgt_idx_to_gid = {}
        for j, t in enumerate(targets_data):
            tgt_idx_to_gid[j] = t.get("group_id", j)

        for obs in obs_data:
            cam_idx = obs.get("camera_idx", 0)
            tgt_idx = obs.get("target_idx", 0)
            cam_name = cam_names_list[cam_idx] if cam_idx < len(cam_names_list) else "cam_{}".format(cam_idx)
            gid = tgt_idx_to_gid.get(tgt_idx, tgt_idx)

            obj_flat = obs.get("obj_pts", [])
            img_flat = obs.get("img_pts", [])
            n_pts = len(obj_flat) // 3
            if n_pts == 0:
                continue

            obj_pts = [[obj_flat[3*k], obj_flat[3*k+1], obj_flat[3*k+2]]
                       for k in range(n_pts)]
            img_pts = [[img_flat[2*k], img_flat[2*k+1]]
                       for k in range(n_pts)]

            if gid not in groups:
                groups[gid] = {}
            groups[gid][cam_name] = {
                "obj_pts": obj_pts,
                "img_pts": img_pts,
                "n_pts": n_pts,
            }

    return groups, cameras_info


def compute_planarity_score(obj_pts):
    """SVD 平面拟合: 返回 mean(|residual|), 0 = 完全平面."""
    if len(obj_pts) < 3:
        return 0.0
    pts = np.array(obj_pts, dtype=np.float64)
    centroid = np.mean(pts, axis=0)
    pts_c = pts - centroid
    U, S, Vt = np.linalg.svd(pts_c, full_matrices=False)
    # 最小奇异值对应的法向量
    if S.shape[0] >= 3:
        normal = Vt[2, :]
        residuals = np.abs(pts_c @ normal)
        return float(np.mean(residuals))
    return 0.0


def compute_singular_value_ratio(obj_pts):
    """返回 s3/s2 比值: 0 = 完全平面, >0.2 = 明显非平面."""
    if len(obj_pts) < 3:
        return 0.0, 0.0, 0.0
    pts = np.array(obj_pts, dtype=np.float64)
    centroid = np.mean(pts, axis=0)
    pts_c = pts - centroid
    U, S, Vt = np.linalg.svd(pts_c, full_matrices=False)
    s3 = S[2] if S.shape[0] >= 3 else 0.0
    s2 = S[1] if S.shape[0] >= 2 else 1.0
    s1 = S[0] if S.shape[0] >= 1 else 1.0
    return float(s3), float(s2), float(s3 / s2) if s2 > 1e-12 else 1.0


# ── Phase 1: Audit ──


def audit_rig_candidates(observations_path, output_dir):
    """Phase 1: 审计所有 group 的 PnP 和 pairwise rig candidate 质量.

    Returns:
        dict: 完整审计报告
    """
    calib = _load_calib_module()
    # 确保 YAML 面定义已加载
    yaml_poses = calib._load_face_poses_from_yaml()
    if yaml_poses and len(yaml_poses) >= 5:
        calib.FACE_POSES_TARGET = yaml_poses
        calib.T_TARGET_FACE = {n: calib.build_T_target_face(n)
                               for n in calib.FACE_POSES_TARGET}

    data = load_observations(observations_path)
    groups, cameras_info = _extract_groups(data)

    print("Auditing {} groups, {} cameras...".format(len(groups), len(cameras_info)))
    print("  Cameras: {}".format(sorted(cameras_info.keys())))

    per_group = []
    all_pairwise = {pair: [] for pair in PAIRS}

    for gid in sorted(groups.keys()):
        gdata = groups[gid]
        group_entry = {"group_id": gid, "cameras": {}, "pairwise": {}}

        # 对每个相机运行 PnP
        pnp_results = {}
        for cam_name in CAMERAS:
            cd = gdata.get(cam_name)
            if cd is None or cd["n_pts"] < 4:
                group_entry["cameras"][cam_name] = {
                    "status": "SKIP", "reason": "too few points" if cd else "missing"}
                continue

            obj_pts = cd["obj_pts"]
            img_pts = cd["img_pts"]
            K_info = cameras_info.get(cam_name, {})
            K = np.array([[K_info.get("fx", 381), 0, K_info.get("cx", 212)],
                          [0, K_info.get("fy", 381), K_info.get("cy", 120)],
                          [0, 0, 1]], dtype=np.float64)

            # 使用 calib.solve_pnp
            T, rvec, tvec, stats = calib.solve_pnp(
                obj_pts, img_pts, K.tolist(), None)  # D=None (已去畸变)

            if T is None:
                group_entry["cameras"][cam_name] = {
                    "status": "FAIL",
                    "reason": stats.get("error", "PnP failed"),
                    "n_pts": cd["n_pts"],
                }
                continue

            # 计算 planarity
            s3, s2, sv_ratio = compute_singular_value_ratio(obj_pts)
            planarity = compute_planarity_score(obj_pts)

            # 获取检测到的面 (从 face_counts 如果有的话)
            face_counts = cd.get("face_counts", {})

            pnp_results[cam_name] = T
            group_entry["cameras"][cam_name] = {
                "status": "OK",
                "n_pts": cd["n_pts"],
                "rmse_px": float(stats.get("rmse_px", 0)),
                "inliers": int(stats.get("inliers", 0)),
                "max_error_px": float(stats.get("max_error_px", 0)),
                "s3": float(s3),
                "s2": float(s2),
                "s3_s2_ratio": float(sv_ratio),
                "planarity_mm": float(planarity * 1000),
                "faces": face_counts,
            }

        # 计算 pairwise transforms
        for cam_a, cam_b in PAIRS:
            Ta = pnp_results.get(cam_a)
            Tb = pnp_results.get(cam_b)
            if Ta is None or Tb is None:
                continue
            # T_ab = Ta @ inv(Tb) = T_a_target @ T_target_b
            Tab = Ta @ np.linalg.inv(Tb)
            t_ab = Tab[:3, 3]
            r_ab_deg = rotation_distance_deg(Tab[:3, :3], np.eye(3))
            pair_data = {
                "T": Tab,
                "translation_m": [float(t_ab[0]), float(t_ab[1]), float(t_ab[2])],
                "translation_mm": float(np.linalg.norm(t_ab) * 1000),
                "rotation_deg": float(r_ab_deg),
            }
            group_entry["pairwise"]["{}_{}".format(cam_a, cam_b)] = pair_data
            all_pairwise[(cam_a, cam_b)].append({
                "group": gid, "T": Tab, **pair_data,
            })

        per_group.append(group_entry)

    # 计算 pairwise 的鲁棒统计
    pair_stats = {}
    for pair in PAIRS:
        pair_name = "{}_{}".format(pair[0], pair[1])
        pw_list = all_pairwise[pair]
        n = len(pw_list)
        if n < 2:
            pair_stats[pair_name] = {"n_candidates": n, "verdict": "INSUFFICIENT_DATA"}
            continue

        # 计算 translation 和 rotation 的鲁棒中位数 + MAD
        trans_list = np.array([p["translation_mm"] for p in pw_list])
        rot_list = np.array([p["rotation_deg"] for p in pw_list])

        # 用归一化 SE3 距离做聚类
        # 先取中位数作为初始参考
        translations_m = np.array([p["translation_m"] for p in pw_list])
        median_t = np.median(translations_m, axis=0)

        # 计算每个 candidate 到 median 的平移偏差
        t_deviations = np.array([
            np.linalg.norm(p["translation_m"] - median_t) * 1000
            for p in pw_list
        ])

        mad_t = float(np.median(t_deviations))
        # 使用 3*MAD 作为内点门限
        inlier_mask_t = t_deviations < max(3.0 * mad_t, TRANSLATION_MAD_LIMIT_MM)
        n_inliers = int(np.sum(inlier_mask_t))

        # 用内点重新算中位数
        if n_inliers >= 2:
            median_t_inlier = np.median(translations_m[inlier_mask_t], axis=0)
            median_r_inlier = float(np.median(rot_list[inlier_mask_t]))
        else:
            median_t_inlier = median_t
            median_r_inlier = float(np.median(rot_list))

        # 更新偏差 (基于内点中位数)
        t_deviations_final = np.array([
            np.linalg.norm(np.array(p["translation_m"]) - median_t_inlier) * 1000
            for p in pw_list
        ])
        mad_t_final = float(np.median(np.abs(t_deviations_final - np.median(t_deviations_final))))

        # 旋转 MAD
        rot_deviations = np.abs(rot_list - median_r_inlier)
        mad_r_final = float(np.median(rot_deviations))

        # 分类误差分布
        inlier_fraction = n_inliers / n
        if inlier_fraction >= 0.7:
            pattern = "clustered_outliers" if n_inliers < n else "unimodal"
        elif n_inliers >= n / 3 and (n - n_inliers) >= 3:
            pattern = "bimodal"
        else:
            pattern = "continuous_scatter"

        # 硬门判断
        hard_gate = (
            mad_t_final < TRANSLATION_MAD_LIMIT_MM and
            mad_r_final < ROTATION_MAD_LIMIT_DEG and
            n_inliers >= MIN_SUPPORTING_GROUPS
        )

        pair_stats[pair_name] = {
            "n_candidates": n,
            "n_inliers": n_inliers,
            "consensus_ratio": float(n_inliers / n) if n > 0 else 0.0,
            "median_translation_mm": float(np.linalg.norm(median_t_inlier) * 1000),
            "mad_translation_mm": mad_t_final,
            "median_rotation_deg": median_r_inlier,
            "mad_rotation_deg": mad_r_final,
            "error_pattern": pattern,
            "verdict": "PASS" if hard_gate else "FAIL",
            "hard_gate_details": {
                "mad_t_lt_{}mm".format(TRANSLATION_MAD_LIMIT_MM): mad_t_final < TRANSLATION_MAD_LIMIT_MM,
                "mad_r_lt_{}deg".format(ROTATION_MAD_LIMIT_DEG): mad_r_final < ROTATION_MAD_LIMIT_DEG,
                "n_inliers_ge_{}".format(MIN_SUPPORTING_GROUPS): n_inliers >= MIN_SUPPORTING_GROUPS,
            },
        }

        # 更新 per_group 中的 pairwise deviation
        for pg_entry in per_group:
            pair_key = "{}_{}".format(pair[0], pair[1])
            pw = pg_entry.get("pairwise", {}).get(pair_key)
            if pw:
                t_dev = np.linalg.norm(
                    np.array(pw["translation_m"]) - median_t_inlier) * 1000
                pg_entry["pairwise"][pair_key]["deviation_from_median_mm"] = float(t_dev)
                pg_entry["pairwise"][pair_key]["inlier"] = bool(
                    t_dev < max(3.0 * mad_t_final, TRANSLATION_MAD_LIMIT_MM))
                pg_entry["pairwise"][pair_key]["translation_consensus_mm"] = \
                    [float(v) for v in median_t_inlier]
                pg_entry["pairwise"][pair_key]["rotation_consensus_deg"] = median_r_inlier

    # 构建报告
    report = {
        "audit_summary": {
            "n_groups_total": len(groups),
            "n_groups_with_full_triplet": sum(
                1 for g in per_group
                if all(g["cameras"].get(c, {}).get("status") == "OK" for c in CAMERAS)
            ),
            "n_groups_with_at_least_two": sum(
                1 for g in per_group
                if sum(1 for c in CAMERAS
                       if g["cameras"].get(c, {}).get("status") == "OK") >= 2
            ),
            "pairs": pair_stats,
        },
        "per_group_details": per_group,
        "quality_thresholds": {
            "pnp_rmse_good_px": PNP_RMSE_GOOD,
            "pnp_rmse_reject_px": PNP_RMSE_REJECT,
            "min_points_good": MIN_POINTS_GOOD,
            "translation_mad_limit_mm": TRANSLATION_MAD_LIMIT_MM,
            "rotation_mad_limit_deg": ROTATION_MAD_LIMIT_DEG,
            "min_supporting_groups": MIN_SUPPORTING_GROUPS,
        },
    }

    # 写报告 (先做 numpy → native 转换)
    os.makedirs(output_dir, exist_ok=True)

    def _to_native(obj):
        """递归转换 numpy 类型为 Python 原生类型."""
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return [_to_native(v) for v in obj.tolist()]
        if isinstance(obj, dict):
            return {k: _to_native(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_to_native(v) for v in obj]
        return obj

    report = _to_native(report)

    report_path = os.path.join(output_dir, "rig_audit_report.yaml")
    with open(report_path, "w") as f:
        yaml.dump(report, f, default_flow_style=False, sort_keys=False)
    print("Audit report: {}".format(report_path))

    # 详细 JSON (含原始值)
    detail_path = os.path.join(output_dir, "rig_audit_detail.json")
    # 转换为可序列化的格式
    detail = {
        "audit_summary": report["audit_summary"],
        "per_group_details": [],
    }
    for g in per_group:
        gd = {"group_id": g["group_id"], "cameras": g["cameras"]}
        gd["pairwise"] = {}
        for pk, pw in g.get("pairwise", {}).items():
            gd["pairwise"][pk] = {
                k: v for k, v in pw.items() if k != "T"
            }
        detail["per_group_details"].append(gd)
    with open(detail_path, "w") as f:
        json.dump(detail, f, indent=2)
    print("Detail JSON: {}".format(detail_path))

    return report


# ── CLI ──


def main():
    parser = argparse.ArgumentParser(
        description="Rig Initialization — PnP Audit + Pose Graph Init")
    subparsers = parser.add_subparsers(dest="command")

    # audit
    audit_p = subparsers.add_parser(
        "audit", help="Phase 1: Per-group PnP + pairwise diagnostics")
    audit_p.add_argument("--observations", required=True,
                         help="accumulated_observations.yaml or ceres JSON")
    audit_p.add_argument("--output", required=True,
                         help="output directory for audit reports")

    # init (Phase 2 — 占位)
    init_p = subparsers.add_parser(
        "init", help="Phase 2: 3-camera robust pose graph initialization")
    init_p.add_argument("--observations", required=True)
    init_p.add_argument("--output", required=True)
    init_p.add_argument("--audit-report", default=None,
                        help="pre-computed audit report (skip re-running PnP)")

    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == "audit":
        audit_rig_candidates(args.observations, args.output)
    elif args.command == "init":
        print("Phase 2 (pose graph init) — 待实现")
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Bundle Adjustment CLI — 调用 C++ Ceres 优化器.

读取累积观测 (accumulated_observations.yaml), 转换为 Ceres 输入 JSON,
调用 ceres_ba_optimizer 子进程, 解析结果并输出 BA-refined 外参.

用法:
  rosrun cr5_spray_perception bundle_adjustment.py \
    --observations <path> --output <dir>
"""
import sys
import os
import json
import subprocess
import argparse
import math
import numpy as np
import yaml
from datetime import datetime


def load_observations(path):
    """加载 accumulated_observations.yaml."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


def rvec_tvec_to_quat_trans(rvec, tvec):
    """将 OpenCV rvec/tvec 转换为 [qw,qx,qy,qz,tx,ty,tz]."""
    import cv2
    R, _ = cv2.Rodrigues(np.array(rvec, dtype=float))
    # 旋转矩阵 → 四元数
    from tf.transformations import quaternion_from_matrix
    T = np.eye(4)
    T[:3, :3] = R
    q = quaternion_from_matrix(T)
    t = np.array(tvec, dtype=float).flatten()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2]),
            float(t[0]), float(t[1]), float(t[2])]


def build_ceres_input(observations_data):
    """将观测数据转换为 Ceres BA 输入 JSON."""
    data = {}
    if isinstance(observations_data, dict):
        data = observations_data
    else:
        return None

    cameras_json = []
    cam_names = sorted(data.get("cameras", {}).keys())
    cam_to_idx = {name: i for i, name in enumerate(cam_names)}

    for cam_name in cam_names:
        cam_info = data["cameras"][cam_name]
        K = cam_info.get("K", [[1,0,0],[0,1,0],[0,0,1]])
        if isinstance(K, list):
            K = np.array(K).reshape(3, 3)

        init_rvec = cam_info.get("rvec_init", [0, 0, 0])
        init_tvec = cam_info.get("tvec_init", [0, 0, 0])
        qt = rvec_tvec_to_quat_trans(init_rvec, init_tvec)

        cameras_json.append({
            "name": cam_name,
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "initial_pose": qt,
        })

    targets_json = []
    groups = data.get("groups", data.get("observations", []))
    if isinstance(groups, dict):
        # groups dict keyed by group_id
        group_ids = sorted(groups.keys())
    elif isinstance(groups, list):
        group_ids = list(range(len(groups)))
    else:
        group_ids = []

    tgt_to_idx = {}
    for j, gid in enumerate(group_ids):
        tgt_to_idx[int(gid)] = j
        targets_json.append({
            "group_id": int(gid),
            "initial_pose": [0, 0, 0, 1, 0, 0, 0],  # 默认恒等
        })

    obs_json = []
    for gid_str in (group_ids if isinstance(group_ids[0], str)
                    else [str(g) for g in group_ids]):
        gid = int(gid_str)
        group_data = groups.get(gid_str, groups[gid] if isinstance(groups, list)
                                else groups.get(gid, {}))
        if not isinstance(group_data, dict):
            continue

        for cam_name in cam_names:
            cam_data = group_data.get(cam_name, {})
            if not isinstance(cam_data, dict):
                continue
            obj_pts = cam_data.get("object_points_3d", [])
            img_pts = cam_data.get("image_points_2d", [])
            if not obj_pts or not img_pts:
                continue

            cam_idx = cam_to_idx.get(cam_name)
            tgt_idx = tgt_to_idx.get(gid)
            if cam_idx is None or tgt_idx is None:
                continue

            cam_info = data["cameras"][cam_name]
            K = np.array(cam_info.get("K", [[1,0,0],[0,1,0],[0,0,1]])).reshape(3, 3)

            # Flatten obj_pts/img_pts
            obj_flat = []
            for pt in obj_pts:
                if isinstance(pt, (list, tuple)):
                    obj_flat.extend([float(v) for v in pt[:3]])
            img_flat = []
            for pt in img_pts:
                if isinstance(pt, (list, tuple)):
                    img_flat.extend([float(v) for v in pt[:2]])

            obs_json.append({
                "camera_idx": cam_idx,
                "target_idx": tgt_idx,
                "fx": float(K[0, 0]), "fy": float(K[1, 1]),
                "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                "obj_pts": obj_flat,
                "img_pts": img_flat,
            })

    return {
        "cameras": cameras_json,
        "targets": targets_json,
        "observations": obs_json,
        "options": {
            "max_iterations": 500,
            "fix_first_camera": True,
        },
    }, cam_names


def run_ceres_ba(input_json, output_dir):
    """运行 Ceres BA 子进程."""
    input_path = os.path.join(output_dir, "ceres_ba_input.json")
    output_path = os.path.join(output_dir, "ceres_ba_output.json")

    with open(input_path, "w") as f:
        json.dump(input_json, f, indent=2)

    # 查找 ceres_ba_optimizer 可执行文件
    exe_path = None
    search_paths = [
        os.path.join(os.path.dirname(__file__), "..", "..", "devel",
                     "lib", "cr5_spray_perception", "ceres_ba_optimizer"),
    ]
    # 通过 rospack 查找
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("cr5_spray_perception")
        search_paths.append(os.path.join(
            os.path.dirname(pkg_path), "devel", "lib",
            "cr5_spray_perception", "ceres_ba_optimizer"))
    except Exception:
        pass

    for p in search_paths:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            exe_path = p
            break

    if exe_path is None:
        raise FileNotFoundError(
            "ceres_ba_optimizer not found. Build with: catkin_make")

    result = subprocess.run(
        [exe_path, input_path, output_path],
        capture_output=True, text=True, timeout=60)
    print(result.stdout)

    if result.returncode != 0:
        print("STDERR:", result.stderr, file=sys.stderr)

    with open(output_path) as f:
        output = json.load(f)

    return output


def build_extrinsics_yaml(ba_output, cam_names):
    """从 BA 输出构建 standard initial_extrinsics.yaml."""
    import cv2

    cameras_dict = {}
    for cam in ba_output.get("cameras", []):
        name = cam["name"]
        qt = cam["optimized_pose"]
        qw, qx, qy, qz = qt[0], qt[1], qt[2], qt[3]
        tx, ty, tz = qt[4], qt[5], qt[6]

        # 四元数 → 旋转矩阵 → rvec
        from tf.transformations import quaternion_matrix
        T = quaternion_matrix([qx, qy, qz, qw])
        R = T[:3, :3]
        rvec = cv2.Rodrigues(R)[0].flatten().tolist()
        tvec = [tx, ty, tz]

        # T_camera_target = [R | t] 4x4
        T_ct = np.eye(4)
        T_ct[:3, :3] = R
        T_ct[:3, 3] = [tx, ty, tz]
        T_tc = np.linalg.inv(T_ct)

        cameras_dict[name] = {
            "optical_frame": "{}_color_optical_frame".format(name),
            "T_camera_target": T_ct.tolist(),
            "T_target_camera": T_tc.tolist(),
            "T_camera_target_rvec": rvec,
            "T_camera_target_tvec": tvec,
        }

    return {
        "schema_version": 1,
        "calibration_id": "ba_{}".format(datetime.now().strftime("%Y%m%d_%H%M%S")),
        "generated_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "method": "multi_frame_bundle_adjustment",
        "optimization_framework": "Ceres Solver (SE(3) LM, Huber 2px)",
        "status": "PASS" if ba_output.get("success") else "FAIL",
        "target_frame": "calibration_target_frame",
        "transform_contract": {
            "primary_transform": "T_target_camera",
            "inverse_transform": "T_camera_target",
            "primary_equation": "p_target = T_target_camera @ p_camera",
            "inverse_equation": "p_camera = T_camera_target @ p_target",
        },
        "ba_stats": {
            "initial_cost": ba_output.get("initial_cost"),
            "final_cost": ba_output.get("final_cost"),
            "iterations": ba_output.get("iterations"),
            "time_ms": ba_output.get("time_ms"),
            "num_targets": len(ba_output.get("targets", [])),
        },
        "cameras": cameras_dict,
    }


def main():
    parser = argparse.ArgumentParser(description="Bundle Adjustment CLI")
    parser.add_argument("--observations", required=True,
                        help="accumulated_observations.yaml path")
    parser.add_argument("--output", default="artifacts/calibration/ba_extrinsics",
                        help="output directory")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # 1. 加载观测
    print("Loading observations from {}".format(args.observations))
    obs_data = load_observations(args.observations)

    # 2. 构建 Ceres 输入
    print("Building Ceres input...")
    ceres_input, cam_names = build_ceres_input(obs_data)
    print("  {} cameras, {} targets, {} observations".format(
        len(ceres_input["cameras"]),
        len(ceres_input["targets"]),
        len(ceres_input["observations"])))

    # 3. 运行 Ceres BA
    print("Running Ceres Bundle Adjustment...")
    ba_output = run_ceres_ba(ceres_input, args.output)

    # 4. 构建外参 YAML
    print("Building extrinsics YAML...")
    extrinsics = build_extrinsics_yaml(ba_output, cam_names)

    extrinsics_path = os.path.join(args.output, "initial_extrinsics.yaml")
    with open(extrinsics_path, "w") as f:
        yaml.dump(extrinsics, f, default_flow_style=False)

    print("BA Report:")
    print("  initial_cost: {:.2f}".format(ba_output.get("initial_cost", 0)))
    print("  final_cost:   {:.2f}".format(ba_output.get("final_cost", 0)))
    reduction = (1 - ba_output.get("final_cost", 0) /
                 max(ba_output.get("initial_cost", 1), 1)) * 100
    print("  reduction:    {:.1f}%".format(reduction))
    print("  iterations:   {}".format(ba_output.get("iterations", 0)))
    print("  time_ms:      {:.1f}".format(ba_output.get("time_ms", 0)))
    print("  status:       {}".format(extrinsics["status"]))
    print("\nWrote: {}".format(extrinsics_path))


if __name__ == "__main__":
    main()

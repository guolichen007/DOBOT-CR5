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

        # 优先使用 direct initial_pose [qw,qx,qy,qz,tx,ty,tz]
        qt = cam_info.get("initial_pose")
        if qt is None:
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
        # gid 可能是 int 或 "group_0000" 字符串
        if isinstance(gid, str):
            try:
                numeric_gid = int(gid)
            except ValueError:
                # "group_0000" → 0
                numeric_gid = int(gid.split("_")[-1]) if "_" in gid else j
        else:
            numeric_gid = int(gid)

        tgt_to_idx[numeric_gid] = j

        # 读取目标初始位姿 (如果有)
        group_data = groups.get(gid, groups.get(numeric_gid, {}))
        tgt_init_pose = group_data.get("target_initial_pose", None) if isinstance(group_data, dict) else None
        if tgt_init_pose is None:
            tgt_init_pose = [1, 0, 0, 0, 0, 0, 0]  # 恒等四元数 [qw,qx,qy,qz,tx,ty,tz]

        targets_json.append({
            "group_id": numeric_gid,
            "initial_pose": tgt_init_pose,
        })

    obs_json = []
    for gid_raw in group_ids:
        # 统一处理 int / "group_0000" 两种 group ID
        if isinstance(gid_raw, str):
            try:
                gid = int(gid_raw)
            except ValueError:
                gid = int(gid_raw.split("_")[-1]) if "_" in gid_raw else 0
        else:
            gid = int(gid_raw)

        group_data = groups.get(gid_raw, groups.get(gid, {}))
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
        # 从 scripts/ 向上 3 级到 workspace root, 再进入 devel/lib/
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "devel",
                     "lib", "cr5_spray_perception", "ceres_ba_optimizer"),
    ]
    # 通过 rospack 查找
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("cr5_spray_perception")
        # pkg_path = .../src/cr5_spray_perception, 向上 2 级到 workspace root
        search_paths.append(os.path.join(
            os.path.dirname(os.path.dirname(pkg_path)), "devel", "lib",
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
    """从 BA 输出构建 standard initial_extrinsics.yaml.

    BA 优化变量:
      cam_pose  = T_rig_camera (rig = 第一台相机 optical frame)
      tgt_pose  = T_rig_target (标定目标在 rig 坐标系中的位姿)

    输出契约:
      T_rig_camera: 将 camera 坐标系中的点转换到 rig 坐标系
      T_camera_rig: 逆变换, 将 rig 坐标系中的点转换到 camera 坐标系
      rig_frame: 第一台相机的 color_optical_frame
    """
    import cv2

    first_cam = cam_names[0] if cam_names else "cam_front_left"
    rig_frame = "{}_color_optical_frame".format(first_cam)

    cameras_dict = {}
    for cam in ba_output.get("cameras", []):
        name = cam["name"]
        qt = cam["optimized_pose"]
        qw, qx, qy, qz = qt[0], qt[1], qt[2], qt[3]
        tx, ty, tz = qt[4], qt[5], qt[6]

        # 四元数 → 旋转矩阵 → rvec
        from tf.transformations import quaternion_matrix
        T_rig_cam = quaternion_matrix([qx, qy, qz, qw])
        T_rig_cam[:3, 3] = [tx, ty, tz]

        rvec = cv2.Rodrigues(T_rig_cam[:3, :3])[0].flatten().tolist()
        tvec = [tx, ty, tz]

        # T_camera_rig = inv(T_rig_camera)
        T_cam_rig = np.linalg.inv(T_rig_cam)

        cameras_dict[name] = {
            "optical_frame": "{}_color_optical_frame".format(name),
            "T_rig_camera": T_rig_cam.tolist(),
            "T_camera_rig": T_cam_rig.tolist(),
            "T_rig_camera_rvec": rvec,
            "T_rig_camera_tvec": tvec,
        }

    return {
        "schema_version": 1,
        "calibration_id": "ba_{}".format(datetime.now().strftime("%Y%m%d_%H%M%S")),
        "generated_at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "method": "multi_frame_bundle_adjustment",
        "optimization_framework": "Ceres Solver (SE(3) LM, Huber 2px)",
        "status": "PASS" if ba_output.get("success") else "FAIL",
        "rig_frame": rig_frame,
        "rig_definition": "first camera ({}) color optical frame, gauge-fixed at identity".format(first_cam),
        "transform_contract": {
            "primary_transform": "T_rig_camera",
            "inverse_transform": "T_camera_rig",
            "primary_equation": "p_rig = T_rig_camera @ p_camera",
            "inverse_equation": "p_camera = T_camera_rig @ p_rig",
            "note": "For TSDF fusion, transform each camera's points to rig_frame using T_rig_camera",
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

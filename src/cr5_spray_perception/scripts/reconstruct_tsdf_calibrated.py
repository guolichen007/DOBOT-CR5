#!/usr/bin/env python3
"""
CR5 Reconstruction — 校准三相机 TSDF 三维重建.

生产链路: 仅使用 Stable V1 calibrated_rig + RGB-D + target ROI.
禁止读取 Gazebo TF / Oracle / model_states.

TSDF frame: cam_front_left_color_optical_frame (rig frame)
Integration: extrinsic = T_camera_rig = inverse(T_rig_camera)

参数矩阵: 3 voxel × 3 sdf_trunc = 9 组
"""
import os, sys, argparse, json, yaml, logging, hashlib, shutil, datetime, subprocess
import numpy as np
import cv2

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("tsdf_reconstruct")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract, RGBDData,
)
from cr5_spray_perception.reconstruction.extrinsics import (
    load_calibrated_rig, get_T_rig_camera, get_T_camera_rig,
)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest,
)
from cr5_spray_perception.reconstruction.registration import load_registration_evidence
from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb,
)


def _sha256_file(path):
    if not path or not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _get_git_sha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=WS, stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return "unknown"


def _get_env_info():
    ts = datetime.datetime.utcnow().isoformat() + "Z"
    info = {"git_sha": _get_git_sha(), "timestamp": ts, "timestamp_utc": ts}
    for pkg in ["numpy", "open3d"]:
        try:
            mod = __import__(pkg)
            info[f"{pkg}_version"] = getattr(mod, "__version__", "unknown")
        except Exception:
            info[f"{pkg}_version"] = "unknown"
    return info


def backproject_rgbd(rgbd, depth_min_m, depth_max_m):
    """反投影 RGB-D → camera-frame 点云."""
    depth_m, detected_unit = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
    points_cam, valid_mask = depth_image_to_pointcloud(
        depth_m, rgbd.depth_K, depth_min_m, depth_max_m)
    if rgbd.color is not None and points_cam.shape[0] > 0:
        color_rgb = cv2.cvtColor(rgbd.color, cv2.COLOR_BGR2RGB)
        h, w = valid_mask.shape
        if color_rgb.shape[:2] == (h, w):
            colors = color_rgb[valid_mask].astype(np.float64) / 255.0
        else:
            colors = np.zeros((points_cam.shape[0], 3), dtype=np.float64)
    else:
        colors = np.zeros((points_cam.shape[0], 3), dtype=np.float64)
    return points_cam, colors, valid_mask


def integrate_tsdf(tsdf_volume, rgbd_image, intrinsic, T_camera_rig):
    """将 RGBDImage 集成到 TSDF volume.

    Args:
        tsdf_volume: open3d.pipelines.integration.ScalableTSDFVolume
        rgbd_image: open3d.geometry.RGBDImage
        intrinsic: open3d.camera.PinholeCameraIntrinsic
        T_camera_rig: 4×4 extrinsic (camera → rig/volume)
    """
    import open3d as o3d

    if rgbd_image is None:
        return False

    tsdf_volume.integrate(rgbd_image, intrinsic, T_camera_rig.astype(np.float64))
    return True


def validate_mesh(mesh, output_dir, label):
    """验证重建 mesh."""
    import open3d as o3d
    result = {
        "file_exists": False, "vertices": 0, "triangles": 0,
        "connected_components": 0, "aabb": None, "surface_area": 0.0,
        "degenerate_triangles": 0, "errors": [],
    }

    if mesh is None:
        result["errors"].append("mesh 为 None")
        return result

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    result["vertices"] = int(len(vertices))
    result["triangles"] = int(len(triangles))

    if len(vertices) == 0 or len(triangles) == 0:
        result["errors"].append("空 mesh")
        return result

    result["aabb"] = {
        "min": vertices.min(axis=0).tolist(),
        "max": vertices.max(axis=0).tolist(),
    }

    # 连通分量分析
    try:
        tri_ids_raw, counts_raw, _ = mesh.cluster_connected_triangles()
        tri_ids = np.asarray(tri_ids_raw)
        counts = np.asarray(counts_raw)
        result["connected_components"] = int(len(counts))
        result["component_triangle_counts"] = sorted([int(c) for c in counts], reverse=True)

        # 输出采样各个分量的 mesh (所有分量合并)
        mesh_all = o3d.geometry.TriangleMesh()
        np.random.seed(42)
        for i, count in enumerate(counts):
            if count < 2:  # 至少需要 1 个三角形
                continue
            comp_mask = tri_ids == i
            comp_tris = triangles[comp_mask]
            if len(comp_tris) < 1:
                continue
            comp_verts_idx = np.unique(comp_tris.flatten())
            remap = {vi: idx for idx, vi in enumerate(comp_verts_idx)}
            comp_v = vertices[comp_verts_idx]
            comp_t = np.array([[remap[t[0]], remap[t[1]], remap[t[2]]] for t in comp_tris])
            comp_mesh = o3d.geometry.TriangleMesh()
            comp_mesh.vertices = o3d.utility.Vector3dVector(comp_v)
            comp_mesh.triangles = o3d.utility.Vector3iVector(comp_t)
            color = np.random.rand(3) * 0.5 + 0.3
            comp_mesh.vertex_colors = o3d.utility.Vector3dVector(
                np.tile(color, (len(comp_v), 1)))
            mesh_all += comp_mesh
        o3d.io.write_triangle_mesh(
            os.path.join(output_dir, "mesh_all_components.ply"), mesh_all)

        # 主分量
        largest_idx = int(np.argmax(counts))
        main_mask = tri_ids == largest_idx
        main_triangles = triangles[main_mask]
        used_verts = np.unique(main_triangles.flatten())
        vert_map = {v: i for i, v in enumerate(used_verts)}
        main_vertices = vertices[used_verts]
        main_triangles_remapped = np.array([
            [vert_map[t[0]], vert_map[t[1]], vert_map[t[2]]]
            for t in main_triangles
        ])

        mesh_main = o3d.geometry.TriangleMesh()
        mesh_main.vertices = o3d.utility.Vector3dVector(main_vertices)
        mesh_main.triangles = o3d.utility.Vector3iVector(main_triangles_remapped)
        if mesh.has_vertex_colors():
            mesh_main.vertex_colors = o3d.utility.Vector3dVector(
                np.asarray(mesh.vertex_colors)[used_verts])
        o3d.io.write_triangle_mesh(
            os.path.join(output_dir, "mesh_primary_component.ply"), mesh_main)

        result["primary_component_triangles"] = int(len(main_triangles_remapped))
        result["removed_components"] = int(result["connected_components"] - 1) if result["connected_components"] > 1 else 0

    except Exception as e:
        result["errors"].append(f"连通分量分析失败: {type(e).__name__}: {e}")

    # 退化三角形
    try:
        result["degenerate_triangles"] = int(np.sum(
            mesh.get_non_manifold_edges()))
    except Exception:
        pass

    # 表面积
    try:
        result["surface_area"] = float(mesh.get_surface_area())
    except Exception:
        pass

    return result


def run_tsdf_reconstruction(rgbd_list, calibrated_rig, config, output_dir, voxel_length, sdf_trunc):
    """运行 TSDF 重建."""
    import open3d as o3d
    os.makedirs(output_dir, exist_ok=True)

    depth_min_m = config.get("depth_min_m", 0.15)
    depth_max_m = config.get("depth_max_m", 2.0)
    target_roi = config.get("target_roi_rig", {})
    roi_min = np.array(target_roi.get("min", [-0.5, -1.0, 0.0]))
    roi_max = np.array(target_roi.get("max", [2.0, 1.0, 2.5]))

    # 扩展 volume 范围确保包含整个 target ROI
    volume_extent = roi_max - roi_min
    volume_origin = roi_min - np.array([0.05, 0.05, 0.05])  # 额外 margin
    volume_size = volume_extent + np.array([0.1, 0.1, 0.1])

    # voxel_grid_shape: 确保覆盖 volume_size
    grid_shape = np.ceil(volume_size / voxel_length).astype(int) + 1

    tsdf_volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    per_frame_info = []
    rgbd_map = {r.camera_name: r for r in rgbd_list}

    # 仅集成一次每台相机 (3 次 integration)
    integrated_count = 0
    for cam_name in REQUIRED_CAMERAS:
        if cam_name not in rgbd_map:
            logger.warning("跳过缺失相机: %s", cam_name)
            continue

        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(calibrated_rig, cam_name)
        T_cr = get_T_camera_rig(calibrated_rig, cam_name)

        # 转换深度为米
        depth_m, detected_unit = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        h, w = depth_m.shape

        # 构建相机内参
        intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h,
            rgbd.depth_K[0, 0], rgbd.depth_K[1, 1],
            rgbd.depth_K[0, 2], rgbd.depth_K[1, 2])

        # 构建 RGBDImage (深度以米为单位, 彩色为 uint8)
        color_uint8 = cv2.cvtColor(rgbd.color, cv2.COLOR_BGR2RGB)
        depth_o3d = o3d.geometry.Image(depth_m.astype(np.float32))
        color_o3d = o3d.geometry.Image(color_uint8.astype(np.uint8))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, depth_scale=1.0, depth_trunc=depth_max_m,
            convert_rgb_to_intensity=False)

        # 统计有效深度像素
        valid_depth = (depth_m > depth_min_m) & (depth_m < depth_max_m) & np.isfinite(depth_m)
        n_raw = int(np.sum(valid_depth))

        # 计算 ROI 内的点数 (用于报告)
        points_cam, _, _ = backproject_rgbd(rgbd, depth_min_m, depth_max_m)
        points_rig = transform_pointcloud(points_cam, T_rc)
        _, crop_mask = crop_pointcloud_aabb(points_rig, roi_min, roi_max)
        n_roi = int(np.sum(crop_mask))

        success = integrate_tsdf(tsdf_volume, rgbd_image, intrinsic, T_cr)

        frame_info = {
            "camera_name": cam_name,
            "optical_frame": f"{cam_name}_color_optical_frame",
            "n_raw_depth_pixels": n_raw,
            "n_roi_points": n_roi,
            "integrated": success,
            "T_rig_camera": T_rc.tolist(),
            "T_camera_rig": T_cr.tolist(),
        }
        per_frame_info.append(frame_info)

        if success:
            integrated_count += 1
            logger.info("[%s] 集成: depth_pixels=%d → roi_points=%d", cam_name, n_raw, n_roi)
        else:
            logger.warning("[%s] 集成失败", cam_name)

    if integrated_count != 3:
        logger.error("预期 3 次集成, 实际 %d", integrated_count)

    # 提取 mesh
    logger.info("提取 mesh (voxel=%.4f, trunc=%.3f)...", voxel_length, sdf_trunc)
    mesh = tsdf_volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # 保存
    mesh_path = os.path.join(output_dir, "reconstructed_mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh)

    # 提取点云
    pcd = tsdf_volume.extract_point_cloud()
    pcd_path = os.path.join(output_dir, "reconstructed_pointcloud.ply")
    o3d.io.write_point_cloud(pcd_path, pcd)

    # 验证
    mesh_validation = validate_mesh(mesh, output_dir, "reconstructed")

    return {
        "mesh_path": mesh_path,
        "pointcloud_path": pcd_path,
        "mesh_validation": mesh_validation,
        "per_frame_integration": per_frame_info,
        "integrated_count": integrated_count,
        "voxel_length": voxel_length,
        "sdf_trunc": sdf_trunc,
    }


def main():
    parser = argparse.ArgumentParser(description="校准三相机 TSDF 重建")
    parser.add_argument("--dataset", "-d", required=True, help="数据集路径")
    parser.add_argument("--rig", "-r", required=True, help="calibrated_rig YAML")
    parser.add_argument("--config", "-c", default=None, help="融合配置 YAML")
    parser.add_argument("--output", "-o", required=True, help="输出根目录")
    parser.add_argument("--group-id", "-g", type=int, default=0, help="group ID")
    parser.add_argument("--registration-evidence", default=None,
                        help="depth_registration.json 路径")
    parser.add_argument("--source", default="stable_v1",
                        choices=["stable_v1", "oracle"],
                        help="外参来源")
    # 参数矩阵 (逗号分隔)
    parser.add_argument("--voxel-lengths", default="0.005,0.0075,0.010",
                        help="逗号分隔的 voxel_length (m)")
    parser.add_argument("--sdf-truncs", default="0.020,0.030,0.040",
                        help="逗号分隔的 sdf_trunc (m)")
    args = parser.parse_args()

    voxel_lengths = [float(x) for x in args.voxel_lengths.split(",")]
    sdf_truncs = [float(x) for x in args.sdf_truncs.split(",")]

    # 加载配置
    if args.config and os.path.isfile(args.config):
        with open(args.config, "r") as f:
            config = yaml.safe_load(f) or {}
    else:
        config = {}

    config.setdefault("depth_min_m", 0.15)
    config.setdefault("depth_max_m", 2.0)

    # 数据集
    try:
        layout = resolve_capture_group(args.dataset, args.group_id)
    except FileNotFoundError as e:
        logger.error(str(e)); sys.exit(1)

    # 外参
    rig = load_calibrated_rig(args.rig)
    if rig.get("status") != "PASS":
        logger.error("rig status=%s", rig.get("status")); sys.exit(1)

    # Registration evidence
    evidence = None
    ev_path = args.registration_evidence or os.path.join(layout.group_dir, "depth_registration.json")
    if os.path.isfile(ev_path):
        evidence = load_registration_evidence(ev_path)

    manifest = load_group_manifest(layout.group_dir)

    # 加载 RGB-D
    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        if not os.path.isdir(cam_dir):
            logger.error("相机目录不存在: %s", cam_dir); sys.exit(1)
        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True,
                                       registration_evidence=evidence)
        if not rgbd.is_valid:
            logger.error("%s 数据验证失败:", cam_name)
            for e in rgbd.errors: logger.error("  %s", e)
            sys.exit(1)
        rgbd_list.append(rgbd)

    # 环境信息
    env_info = _get_env_info()

    # ── 参数矩阵运行 ──
    all_results = []
    for vl in voxel_lengths:
        for st in sdf_truncs:
            run_name = f"voxel_{int(vl*10000):04d}_trunc_{int(st*1000):03d}"
            run_dir = os.path.join(args.output, args.source, f"group_{args.group_id:04d}", run_name)
            os.makedirs(run_dir, exist_ok=True)

            logger.info("=" * 50)
            logger.info("TSDF: voxel=%.4fm, trunc=%.3fm → %s", vl, st, run_dir)
            logger.info("=" * 50)

            try:
                result = run_tsdf_reconstruction(rgbd_list, rig, config, run_dir, vl, st)
            except Exception as e:
                logger.error("TSDF 失败: %s", e)
                result = {"error": str(e), "voxel_length": vl, "sdf_trunc": st}

            # 保存 metadata
            meta = {
                "schema": "cr5_tsdf_reconstruction_v1",
                "source": args.source,
                "group_id": args.group_id,
                "voxel_length_m": vl,
                "sdf_trunc_m": st,
                "config": {
                    "depth_min_m": config.get("depth_min_m"),
                    "depth_max_m": config.get("depth_max_m"),
                    "target_roi": config.get("target_roi_rig"),
                },
                "result": result,
                "provenance": {
                    "git_sha": env_info["git_sha"],
                    "timestamp": env_info["timestamp"],
                    "rig_path": os.path.abspath(args.rig),
                    "rig_sha256": _sha256_file(args.rig),
                    "registration_evidence_sha256": _sha256_file(ev_path) if evidence else None,
                    "numpy_version": env_info.get("numpy_version"),
                    "open3d_version": env_info.get("open3d_version"),
                },
            }

            meta_path = os.path.join(run_dir, "reconstruction_metadata.json")
            with open(meta_path, "w") as f:
                json.dump(meta, f, indent=2, default=str)
            all_results.append(meta)

            # 网格验证
            mv = result.get("mesh_validation", {})
            if mv.get("errors"):
                logger.error("Mesh 验证问题: %s", mv["errors"])
            elif mv.get("vertices", 0) > 0:
                logger.info("Mesh: %d vertices, %d triangles, %d components",
                            mv["vertices"], mv["triangles"],
                            mv.get("connected_components", 0))

    # ── 汇总表格 ──
    print("\n" + "=" * 80)
    print("TSDF 参数矩阵汇总")
    print("=" * 80)
    print(f"{'voxel':>8s} {'trunc':>8s} {'verts':>8s} {'tris':>8s} {'comps':>6s} {'status':>10s}")
    print("-" * 60)
    for r in all_results:
        mv = r["result"].get("mesh_validation", {})
        vl = r["voxel_length_m"]
        st = r["sdf_trunc_m"]
        verts = mv.get("vertices", 0)
        tris = mv.get("triangles", 0)
        comps = mv.get("connected_components", 0)
        errors = mv.get("errors", [])
        status = "OK" if not errors and verts > 0 else "FAIL"
        print(f"{vl:8.4f} {st:8.3f} {verts:8d} {tris:8d} {comps:6d} {status:>10s}")

    summary_path = os.path.join(args.output, args.source, "tsdf_summary.json")
    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w") as f:
        json.dump(all_results, f, indent=2, default=str)

    print(f"\n📁 汇总: {summary_path}")


if __name__ == "__main__":
    main()

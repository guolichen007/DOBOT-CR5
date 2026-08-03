#!/usr/bin/env python3
"""
CR5 Reconstruction — 可见表面 TSDF A/B 消融运行.

固定参数: voxel=0.005, trunc=0.020
四组:
  A_BASELINE: 仅 target ROI mask
  B_EDGE_ONLY: target ROI + depth edge filter
  C_MULTIVIEW_ONLY: target ROI + multiview consistency
  D_EDGE_AND_MULTIVIEW: target ROI + edge + multiview
"""
import os, sys, json, yaml, logging, argparse, shutil
import numpy as np
import cv2

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("ablation")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract)
from cr5_spray_perception.reconstruction.extrinsics import (
    load_calibrated_rig, get_T_rig_camera, get_T_camera_rig)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group, load_group_manifest)
from cr5_spray_perception.reconstruction.registration import load_registration_evidence
from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb)
from cr5_spray_perception.reconstruction.depth_filtering import (
    build_final_integration_mask, DEFAULT_EDGE_CONFIG)
from cr5_spray_perception.reconstruction.multiview_consistency import (
    build_confidence_mask, CONFLICT, EDGE_UNCERTAIN)

try:
    import open3d as o3d
except ImportError:
    logger.error("需要 open3d"); sys.exit(1)


def apply_mask_to_depth(depth_m, integration_mask):
    """将 mask 应用于深度图 (mask 外置零)."""
    return np.where(integration_mask, depth_m, 0.0)


def integrate_frame(tsdf_volume, rgbd, T_cr, depth_m, integration_mask,
                    depth_min_m, depth_max_m, output_dir, cam_name):
    """对单帧执行 masked TSDF 积分."""
    h, w = depth_m.shape
    masked_depth = apply_mask_to_depth(depth_m, integration_mask)
    n_masked = int(np.sum(masked_depth > 0))
    if n_masked == 0:
        raise RuntimeError(f"{cam_name}: integration mask 后无有效深度")

    # 保存 masked depth
    np.save(os.path.join(output_dir, f"{cam_name}_masked_depth.npy"), masked_depth)
    cv2.imwrite(os.path.join(output_dir, f"{cam_name}_integration_mask.png"),
                (integration_mask.astype(np.uint8) * 255))
    depth_preview = np.clip(masked_depth / 2.0 * 255, 0, 255).astype(np.uint8)
    cv2.imwrite(os.path.join(output_dir, f"{cam_name}_masked_depth_preview.png"), depth_preview)

    # 构建 RGBDImage
    color_uint8 = cv2.cvtColor(rgbd.color, cv2.COLOR_BGR2RGB)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h,
        rgbd.depth_K[0, 0], rgbd.depth_K[1, 1], rgbd.depth_K[0, 2], rgbd.depth_K[1, 2])
    depth_o3d = o3d.geometry.Image(masked_depth.astype(np.float32))
    color_o3d = o3d.geometry.Image(color_uint8.astype(np.uint8))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, depth_scale=1.0, depth_trunc=depth_max_m,
        convert_rgb_to_intensity=False)
    tsdf_volume.integrate(rgbd_image, intrinsic, T_cr.astype(np.float64))
    return n_masked


def run_ablation_group(group_name, rgbd_list, calib_rig, config, output_base,
                       use_edge_filter, use_multiview):
    """运行一组消融实验."""
    output_dir = os.path.join(output_base, group_name)
    os.makedirs(output_dir, exist_ok=True)

    depth_min_m = config.get("depth_min_m", 0.15)
    depth_max_m = config.get("depth_max_m", 2.0)
    target_roi = config.get("target_roi_rig", {})
    roi_min = np.array(target_roi.get("min", [-0.281, -0.216, 0.668]))
    roi_max = np.array(target_roi.get("max", [0.277, 0.179, 1.195]))
    voxel_length = 0.005
    sdf_trunc = 0.020
    edge_cfg = DEFAULT_EDGE_CONFIG.copy()

    tsdf_volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length, sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    rgbd_map = {r.camera_name: r for r in rgbd_list}
    all_stats = {}
    integrated_count = 0

    for cam_name in REQUIRED_CAMERAS:
        if cam_name not in rgbd_map:
            continue
        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(calib_rig, cam_name)
        T_cr = get_T_camera_rig(calib_rig, cam_name)

        # 深度处理
        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        h, w = depth_m.shape

        # valid depth mask
        valid_mask = (depth_m > depth_min_m) & (depth_m < depth_max_m) & np.isfinite(depth_m)

        # target ROI mask (3D→2D)
        points_cam, valid_mask_3d = depth_image_to_pointcloud(depth_m, rgbd.depth_K, depth_min_m, depth_max_m)
        n_valid = int(np.sum(valid_mask_3d))
        if n_valid > 0:
            points_rig = transform_pointcloud(points_cam, T_rc)
            inside_roi = (
                (points_rig[:, 0] >= roi_min[0]) & (points_rig[:, 0] <= roi_max[0]) &
                (points_rig[:, 1] >= roi_min[1]) & (points_rig[:, 1] <= roi_max[1]) &
                (points_rig[:, 2] >= roi_min[2]) & (points_rig[:, 2] <= roi_max[2]))
            roi_image = np.zeros((h, w), dtype=bool)
            v_idx = np.where(valid_mask_3d.flatten())[0]
            for k, orig_idx in enumerate(v_idx):
                if inside_roi[k]:
                    roi_image.flat[orig_idx] = True
        else:
            roi_image = np.zeros((h, w), dtype=bool)

        # 边缘 mask
        if use_edge_filter:
            edge_result = build_final_integration_mask(
                depth_m, valid_mask, roi_image, edge_cfg,
                min_isolated_px=edge_cfg.get("remove_isolated_components_px", 20))
            base_integration = edge_result["final_integration_mask"]
            edge_stats = edge_result["stats"]
            # 保存边缘诊断
            cv2.imwrite(os.path.join(output_dir, f"{cam_name}_depth_edge_reject.png"),
                        (edge_result["depth_edge_rejection_mask"].astype(np.uint8) * 255))
            cv2.imwrite(os.path.join(output_dir, f"{cam_name}_isolated_noise.png"),
                        (edge_result["isolated_noise_mask"].astype(np.uint8) * 255))
        else:
            base_integration = valid_mask & roi_image
            edge_stats = {"edge_rejected_pixels": 0, "isolated_rejected_pixels": 0}

        # 多视角一致性 mask
        mv_stats = {"conflict_rejected": 0, "edge_uncertain_rejected": 0,
                    "unique_valid_kept": 0}
        if use_multiview and np.sum(base_integration) > 0:
            # 对 base_integration 中的有效点做多视角检查
            # 构建点列表和 UV 映射
            target_rgbd_list = [rgbd_map[c] for c in REQUIRED_CAMERAS if c != cam_name and c in rgbd_map]

            # 生成 base 内的点
            base_depth = np.where(base_integration, depth_m, 0.0)
            pts_cam_b, vm_b = depth_image_to_pointcloud(base_depth, rgbd.depth_K, depth_min_m, depth_max_m)
            if len(pts_cam_b) > 0:
                # UV map
                v_ys, v_xs = np.where(vm_b)
                uv_map = np.column_stack([v_xs, v_ys])

                # 简化版: 不做逐点多视角检查 (太慢), 直接用源边缘 mask
                source_edge = np.zeros((h, w), dtype=bool)
                if use_edge_filter:
                    source_edge = edge_result.get("depth_edge_rejection_mask",
                                                   np.zeros((h, w), dtype=bool))

                mv_result = build_confidence_mask(
                    cam_name, pts_cam_b, base_depth, vm_b, source_edge,
                    target_rgbd_list, calib_rig, uv_map)

                # 构建 confidence image mask
                conf_img = mv_result["confidence_mask_image"]
                # 最终: base_integration AND confidence (排除 CONFLICT + EDGE_UNCERTAIN)
                final_integration = base_integration & conf_img
                mv_stats = {
                    "conflict_rejected": mv_result["stats"]["conflict"],
                    "edge_uncertain_rejected": mv_result["stats"]["edge_uncertain"],
                    "unique_valid_kept": mv_result["stats"]["unique_valid"],
                    "total_checked": mv_result["stats"]["total_points"],
                }
            else:
                final_integration = base_integration
        else:
            final_integration = base_integration

        # 统计
        n_final = int(np.sum(final_integration))
        stats = {
            "valid_depth_pixels": int(np.sum(valid_mask)),
            "roi_pixels": int(np.sum(roi_image)),
            "base_integration_pixels": int(np.sum(base_integration)),
            "edge_rejected": edge_stats.get("edge_rejected_pixels", 0),
            "isolated_rejected": edge_stats.get("isolated_rejected_pixels", 0),
            **mv_stats,
            "final_integration_pixels": n_final,
            "retained_ratio": round(n_final / max(int(np.sum(base_integration)), 1), 4),
        }
        all_stats[cam_name] = stats

        # 积分
        n_m = integrate_frame(tsdf_volume, rgbd, T_cr, depth_m, final_integration,
                              depth_min_m, depth_max_m, output_dir, cam_name)
        integrated_count += 1
        logger.info("[%s/%s] %s: valid=%d roi=%d base=%d final=%d (%.1f%%)",
                    group_name, cam_name[:6], cam_name,
                    stats["valid_depth_pixels"], stats["roi_pixels"],
                    stats["base_integration_pixels"], stats["final_integration_pixels"],
                    100 * stats["retained_ratio"])

    if integrated_count != 3:
        logger.error("%s: 预期 3 次集成, 实际 %d", group_name, integrated_count)

    # 提取 mesh
    mesh = tsdf_volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    mesh_path = os.path.join(output_dir, "reconstructed_mesh.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh)

    # 保存 stats
    stats_path = os.path.join(output_dir, "mask_statistics.json")
    with open(stats_path, "w") as f:
        json.dump({"group": group_name, "use_edge_filter": use_edge_filter,
                   "use_multiview": use_multiview, "voxel_length": voxel_length,
                   "sdf_trunc": sdf_trunc, "per_camera": all_stats,
                   "n_vertices": len(mesh.vertices),
                   "n_triangles": len(mesh.triangles)}, f, indent=2, default=str)

    return mesh_path, all_stats


def main():
    parser = argparse.ArgumentParser(description="可见表面 TSDF A/B 消融")
    parser.add_argument("--dataset", "-d", required=True)
    parser.add_argument("--rig", "-r", required=True, help="calibrated_rig YAML")
    parser.add_argument("--config", "-c", default=None, help="融合配置 YAML")
    parser.add_argument("--output", "-o", required=True, help="输出根目录")
    parser.add_argument("--group-id", "-g", type=int, default=0)
    parser.add_argument("--registration-evidence", default=None)
    parser.add_argument("--source", default="stable_v1", choices=["stable_v1", "oracle"])
    args = parser.parse_args()

    # 加载配置
    if args.config and os.path.isfile(args.config):
        with open(args.config) as f:
            config = yaml.safe_load(f) or {}
    else:
        config = {}
    config.setdefault("depth_min_m", 0.15)
    config.setdefault("depth_max_m", 2.0)

    # 数据集
    layout = resolve_capture_group(args.dataset, args.group_id)
    rig = load_calibrated_rig(args.rig)
    if rig.get("status") != "PASS":
        logger.error("rig status=%s", rig.get("status")); sys.exit(1)

    ev_path = args.registration_evidence or os.path.join(layout.group_dir, "depth_registration.json")
    evidence = load_registration_evidence(ev_path) if os.path.isfile(ev_path) else None

    # 加载 RGB-D
    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True, registration_evidence=evidence)
        if not rgbd.is_valid:
            for e in rgbd.errors: logger.error("  %s", e)
            sys.exit(1)
        rgbd_list.append(rgbd)

    output_base = os.path.join(args.output, args.source, f"group_{args.group_id:04d}")

    # 四组
    groups = [
        ("A_BASELINE", False, False),
        ("B_EDGE_ONLY", True, False),
        ("C_MULTIVIEW_ONLY", False, True),
        ("D_EDGE_AND_MULTIVIEW", True, True),
    ]

    results = {}
    for gname, use_edge, use_mv in groups:
        logger.info("=" * 50)
        logger.info("运行: %s (edge=%s, multiview=%s)", gname, use_edge, use_mv)
        logger.info("=" * 50)
        mesh_path, stats = run_ablation_group(
            gname, rgbd_list, rig, config, output_base, use_edge, use_mv)
        results[gname] = {"mesh_path": mesh_path, "stats": stats}

    # 汇总
    summary = {"source": args.source, "voxel_length": 0.005, "sdf_trunc": 0.020,
               "groups": {}}
    for gname, r in results.items():
        summary["groups"][gname] = {"mesh": r["mesh_path"], "stats": r["stats"]}
    summary_path = os.path.join(output_base, "ablation_summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2, default=str)

    print(f"\n{'='*60}")
    print(f"Ablation 完成: {args.source}")
    print(f"{'='*60}")
    for gname in groups:
        g = gname[0]
        if g in results:
            s = results[g]["stats"]
            tot_final = sum(s[c].get("final_integration_pixels", 0) for c in REQUIRED_CAMERAS if c in s)
            print(f"  {g}: final_pixels={tot_final}")
    print(f"\n📁 {output_base}")


if __name__ == "__main__":
    main()

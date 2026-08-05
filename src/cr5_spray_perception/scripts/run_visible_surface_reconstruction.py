#!/usr/bin/env python3
"""
CR5 Reconstruction — 可见表面三维重建正式入口.

一条命令完成:
  dataset → target mask → TSDF → mesh cleanup → normal → evaluation → provenance

正式配置: visible_surface_production.yaml
正式外参: 冻结三相机标定 (refinement REJECTED, Oracle DIAGNOSTIC ONLY)
"""
import os, sys, json, yaml, logging, argparse, datetime, hashlib, shutil, subprocess

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("visible_surface")

WS = os.path.join(os.path.dirname(__file__), "..")
sys.path.insert(0, os.path.join(WS, "src"))

from cr5_spray_perception.reconstruction.rgbd_io import (
    load_rgbd_data, validate_rgbd_contract)
from cr5_spray_perception.reconstruction.extrinsics import (
    load_calibrated_rig, get_T_rig_camera, get_T_camera_rig)
from cr5_spray_perception.reconstruction.contracts import REQUIRED_CAMERAS
from cr5_spray_perception.reconstruction.dataset_layout import (
    resolve_capture_group)
from cr5_spray_perception.reconstruction.registration import load_registration_evidence
from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb)
from cr5_spray_perception.reconstruction.mesh_cleanup import cleanup_mesh
from cr5_spray_perception.reconstruction.quality_contract import (
    resolve_release_id, build_acceptance_result, build_quality_metric_sections,
    sanitize_evaluation_label, load_and_validate_evaluation_metrics)
from cr5_spray_perception.reconstruction.evaluation_runner import run_evaluator
import numpy as np
import cv2
import math


def require_open3d():
    """延迟加载 Open3D, 仅在实际执行 TSDF 重建时需要."""
    try:
        import open3d as o3d
        return o3d
    except ImportError as exc:
        raise RuntimeError("open3d is required for reconstruction") from exc


def _sha256_file(p):
    if not p or not os.path.isfile(p): return None
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def _git_sha():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=WS, stderr=subprocess.DEVNULL).decode().strip()
    except:
        return "unknown"


def _git_tag():
    """获取当前 HEAD 的 exact tag, 无则返回空字符串."""
    try:
        return subprocess.check_output(
            ["git", "describe", "--tags", "--exact-match", "HEAD"],
            cwd=WS, stderr=subprocess.DEVNULL).decode().strip()
    except:
        return ""


def _git_branch():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=WS, stderr=subprocess.DEVNULL).decode().strip()
    except:
        return "unknown"


def generate_target_masks(rgbd_list, calib_rig, config, output_dir):
    """对每台相机生成 target ROI 深度 mask."""
    o3d = require_open3d()

    os.makedirs(output_dir, exist_ok=True)
    roi_cfg = config.get("target_roi_rig", {})
    roi_min = np.array(roi_cfg.get("min", [-0.281, -0.216, 0.668]))
    roi_max = np.array(roi_cfg.get("max", [0.277, 0.179, 1.195]))
    dmin = config.get("tsdf", {}).get("depth_min_m", 0.15)
    dmax = config.get("tsdf", {}).get("depth_max_m", 2.0)

    masks = {}
    per_cam_pts_rig = {}
    for cam_name in REQUIRED_CAMERAS:
        rgbd = next((r for r in rgbd_list if r.camera_name == cam_name), None)
        if rgbd is None:
            continue
        T_rc = get_T_rig_camera(calib_rig, cam_name)
        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        h, w = depth_m.shape

        pts_cam, valid_3d = depth_image_to_pointcloud(depth_m, rgbd.depth_K, dmin, dmax)
        n_valid = int(np.sum(valid_3d))
        if n_valid == 0:
            masks[cam_name] = np.zeros((h, w), dtype=bool)
            per_cam_pts_rig[cam_name] = np.zeros((0, 3))
            continue

        pts_rig = transform_pointcloud(pts_cam, T_rc)
        in_roi = ((pts_rig[:, 0] >= roi_min[0]) & (pts_rig[:, 0] <= roi_max[0]) &
                  (pts_rig[:, 1] >= roi_min[1]) & (pts_rig[:, 1] <= roi_max[1]) &
                  (pts_rig[:, 2] >= roi_min[2]) & (pts_rig[:, 2] <= roi_max[2]))

        img_mask = np.zeros((h, w), dtype=bool)
        v_idx = np.where(valid_3d.flatten())[0]
        for k, orig_idx in enumerate(v_idx):
            if in_roi[k]:
                img_mask.flat[orig_idx] = True

        masks[cam_name] = img_mask
        per_cam_pts_rig[cam_name] = pts_rig[in_roi]
        n_roi = int(np.sum(in_roi))
        logger.info("[%s] mask: valid=%d roi=%d (%.1f%%)", cam_name, n_valid, n_roi, 100.0*n_roi/max(n_valid,1))

        # 保存
        masked_d = np.where(img_mask, depth_m, 0.0)
        np.save(os.path.join(output_dir, f"{cam_name}_masked_depth.npy"), masked_d)
        cv2.imwrite(os.path.join(output_dir, f"{cam_name}_target_mask.png"), (img_mask.astype(np.uint8)*255))
        if len(pts_rig[in_roi]) > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts_rig[in_roi].astype(np.float64))
            o3d.io.write_point_cloud(os.path.join(output_dir, f"{cam_name}_target_points_rig.ply"), pcd)

    return masks, per_cam_pts_rig


def run_tsdf(rgbd_list, calib_rig, config, output_dir, depth_masks):
    """运行 TSDF 重建."""
    o3d = require_open3d()

    os.makedirs(output_dir, exist_ok=True)
    tsdf_cfg = config.get("tsdf", {})
    vl = tsdf_cfg.get("voxel_length_m", 0.005)
    st = tsdf_cfg.get("sdf_trunc_m", 0.020)
    dmin = tsdf_cfg.get("depth_min_m", 0.15)
    dmax = tsdf_cfg.get("depth_max_m", 2.0)

    tsdf_vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=vl, sdf_trunc=st, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    integrated = 0
    for cam_name in REQUIRED_CAMERAS:
        rgbd = next((r for r in rgbd_list if r.camera_name == cam_name), None)
        if rgbd is None or cam_name not in depth_masks:
            continue
        T_cr = get_T_camera_rig(calib_rig, cam_name)
        depth_m, _ = convert_depth_to_meters(rgbd.depth_raw, rgbd.depth_unit)
        h, w = depth_m.shape
        mask = depth_masks[cam_name]
        masked_d = np.where(mask, depth_m, 0.0)
        if np.sum(masked_d > 0) == 0:
            raise RuntimeError(f"{cam_name}: mask 后无有效深度")

        intr = o3d.camera.PinholeCameraIntrinsic(w, h, rgbd.depth_K[0,0], rgbd.depth_K[1,1], rgbd.depth_K[0,2], rgbd.depth_K[1,2])
        c_uint8 = cv2.cvtColor(rgbd.color, cv2.COLOR_BGR2RGB)
        rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(c_uint8.astype(np.uint8)),
            o3d.geometry.Image(masked_d.astype(np.float32)),
            depth_scale=1.0, depth_trunc=dmax, convert_rgb_to_intensity=False)
        tsdf_vol.integrate(rgbd_img, intr, T_cr.astype(np.float64))
        integrated += 1

    if integrated != 3:
        logger.error("集成 %d/3 帧!", integrated)

    mesh = tsdf_vol.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    mesh_path = os.path.join(output_dir, "visible_surface_mesh_raw.ply")
    o3d.io.write_triangle_mesh(mesh_path, mesh)
    pcd = tsdf_vol.extract_point_cloud()
    o3d.io.write_point_cloud(os.path.join(output_dir, "visible_surface_pointcloud.ply"), pcd)
    logger.info("TSDF: %d verts, %d tris", len(mesh.vertices), len(mesh.triangles))
    return mesh_path


def main():
    parser = argparse.ArgumentParser(description="可见表面三维重建正式入口")
    parser.add_argument("--dataset", "-d", required=True)
    parser.add_argument("--group-id", "-g", type=int, default=0)
    parser.add_argument("--rig", "-r", required=True)
    parser.add_argument("--config", "-c", default=None)
    parser.add_argument("--output", "-o", required=True)
    parser.add_argument("--registration-evidence", default=None)
    parser.add_argument("--evaluate", action="store_true",
                        help="运行可见表面 GT 评价 (需要 Gazebo + visible GT)")
    parser.add_argument("--visible-gt", default=None,
                        help="visible_union.ply 路径")
    parser.add_argument("--model-pose-json", default=None,
                        help="pose_evidence.json 路径 (离线评价, 禁止读取实时 Gazebo)")
    parser.add_argument("--release-id", default=None,
                        help="发布标识 (默认: git exact tag > CR5_RELEASE_ID env > UNTAGGED)")
    args = parser.parse_args()

    # 加载生产配置
    if args.config and os.path.isfile(args.config):
        with open(args.config) as f:
            config = yaml.safe_load(f) or {}
    else:
        config_path = os.path.join(WS, "config", "reconstruction", "visible_surface_production.yaml")
        with open(config_path) as f:
            config = yaml.safe_load(f)
    logger.info("配置: %s", config.get("schema_version", "?"))

    # 校验: refinement 必须禁用
    rig_cfg = config.get("rig", {})
    if rig_cfg.get("allow_runtime_refinement") or rig_cfg.get("allow_oracle"):
        logger.error("生产配置不允许 refinement/Oracle!"); sys.exit(1)

    # 解析 release identity (使用 quality_contract)
    release_id = resolve_release_id(args.release_id, git_tag_provider=_git_tag)

    # 数据集
    layout = resolve_capture_group(args.dataset, args.group_id)
    rig = load_calibrated_rig(args.rig)
    if rig.get("status") != "PASS":
        logger.error("rig status=%s", rig.get("status")); sys.exit(1)

    ev_path = args.registration_evidence or os.path.join(layout.group_dir, "depth_registration.json")
    evidence = load_registration_evidence(ev_path) if os.path.isfile(ev_path) else None

    rgbd_list = []
    for cam_name in REQUIRED_CAMERAS:
        cam_dir = layout.camera_dirs.get(cam_name) or os.path.join(layout.group_dir, cam_name)
        rgbd = load_rgbd_data(cam_dir, cam_name)
        rgbd = validate_rgbd_contract(rgbd, require_registered_to_color=True, registration_evidence=evidence)
        if not rgbd.is_valid:
            for e in rgbd.errors: logger.error("  %s", e)
            sys.exit(1)
        rgbd_list.append(rgbd)

    os.makedirs(args.output, exist_ok=True)
    logger.info("数据集: %s group=%d", args.dataset, args.group_id)

    # ── 1. Target masks ──
    mask_dir = os.path.join(args.output, "masked_depth")
    masks, per_cam_pts_rig = generate_target_masks(rgbd_list, rig, config, mask_dir)

    # ── 1.5. Target isolation (V2 only) ──
    iso_cfg = config.get("target_isolation", {})
    iso_report = {"enabled": False}
    if iso_cfg.get("enabled", False):
        from cr5_spray_perception.reconstruction.target_isolation import apply_target_isolation
        iso_dir = os.path.join(args.output, "target_isolation")
        masks, iso_report = apply_target_isolation(
            rgbd_list, per_cam_pts_rig, masks, rig, config, iso_dir)

    # ── 2. TSDF ──
    tsdf_dir = os.path.join(args.output, "tsdf")
    mesh_path = run_tsdf(rgbd_list, rig, config, tsdf_dir, masks)

    # ── 3. Mesh cleanup ──
    o3d = require_open3d()
    cleanup_dir = os.path.join(args.output, "mesh_cleanup")
    cleanup_cfg = config.get("mesh_cleanup", {})
    cleaned, removed, comp_report = cleanup_mesh(
        mesh_path, per_cam_pts_rig, cleanup_dir, cleanup_cfg,
        iso_cfg=iso_cfg,
        gt_points_rig=per_cam_pts_rig.get("visible_gt_pts_rig"),
    )

    # 保存最终 mesh
    final_path = os.path.join(args.output, "visible_surface_mesh_final.ply")
    o3d.io.write_triangle_mesh(final_path, cleaned)

    # ── 4. GT 评价 (显式传入 --evaluate 时 fail-closed) ──
    eval_metrics = None
    eval_label = None
    if args.evaluate:
        if not args.visible_gt:
            logger.error("--evaluate 需要 --visible-gt"); sys.exit(1)
        if args.model_pose_json and not os.path.isfile(args.model_pose_json):
            logger.error("--model-pose-json 文件不存在: %s", args.model_pose_json); sys.exit(1)

        eval_dir = os.path.join(args.output, "visible_evaluation")
        eval_script = os.path.join(WS, "..", "cr5_spray_sim", "scripts", "evaluate_reconstruction_gazebo.py")

        target_roi = config.get("target_roi_rig", {})
        rmin = target_roi.get("min", [-0.281, -0.216, 0.668])
        rmax = target_roi.get("max", [0.277, 0.179, 1.195])

        # 使用 evaluation_runner.run_evaluator (fail-closed)
        eval_result = run_evaluator(
            evaluator_script=eval_script,
            recon_mesh=final_path,
            output_dir=eval_dir,
            release_id=release_id,
            visible_gt=args.visible_gt,
            roi_min=rmin,
            roi_max=rmax,
            timeout_s=180,
            env={**os.environ, "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI", "http://localhost:11311")},
            model_pose_json=args.model_pose_json,
        )
        eval_label = eval_result["eval_label"]
        eval_metrics = eval_result["metrics"]
        logger.info("评价完成: acc med=%.2fmm P95=%.2fmm",
                    eval_metrics["accuracy_median_mm"], eval_metrics["accuracy_p95_mm"])

    # ── 5. 重建质量报告 ──
    ts = datetime.datetime.utcnow().isoformat() + "Z"
    identity = {
        "git_sha": _git_sha(),
        "git_tag": _git_tag() or "",
        "git_branch": _git_branch(),
        "release_id": release_id,
        "timestamp": ts,
        "dataset": os.path.abspath(args.dataset),
        "group_id": args.group_id,
        "rig_path": os.path.abspath(args.rig),
        "rig_sha256": _sha256_file(args.rig) or "N/A",
        "config_path": os.path.abspath(args.config) if args.config else "N/A",
        "config_sha256": _sha256_file(args.config) if args.config else "N/A",
        "registration_evidence_sha256": _sha256_file(ev_path) if evidence else "N/A",
    }

    if eval_metrics:
        # 使用 quality_contract 构建报告段落
        accuracy_section, completeness_section, quality_section = \
            build_quality_metric_sections(eval_metrics)

        # 使用 quality_contract 构建 acceptance
        acceptance_section = build_acceptance_result(
            accuracy_median_mm=eval_metrics["accuracy_median_mm"],
            accuracy_p95_mm=eval_metrics["accuracy_p95_mm"],
            median_gate_mm=5.0,
            p95_gate_mm=16.0,
        )
    else:
        accuracy_section = {"status": "NOT_AVAILABLE"}
        completeness_section = {"scope": "visible_gt_only", "status": "NOT_AVAILABLE"}
        quality_section = {"chamfer_mm": "NOT_AVAILABLE"}
        acceptance_section = {"production_pass": "NOT_EVALUATED",
                              "reasons": ["NOT_EVALUATED: --evaluate not requested"]}

    quality = {
        "schema": "cr5_reconstruction_quality_report_v2",
        "identity": identity,
        "reconstruction": {
            "type": "partial_visible_surface",
            "frame": "cam_front_left_color_optical_frame",
            "integrated_frames": 3,
            "voxel_length_m": config.get("tsdf", {}).get("voxel_length_m", 0.005),
            "sdf_trunc_m": config.get("tsdf", {}).get("sdf_trunc_m", 0.020),
        },
        "accuracy": accuracy_section,
        "completeness": completeness_section,
        "quality": quality_section,
        "mesh": {
            "raw_vertices": comp_report["total_vertices_raw"],
            "raw_triangles": comp_report["total_triangles_raw"],
            "cleaned_vertices": comp_report["total_vertices_cleaned"],
            "cleaned_triangles": comp_report["total_triangles_cleaned"],
            "raw_components": comp_report["raw_components"],
            "cleaned_components": comp_report["cleaned_components"],
            "removed_components": comp_report["removed_components"],
            "removed_fragment_area_ratio": comp_report["removed_fragment_area_ratio"],
            "largest_component_area_ratio": comp_report["largest_component_tri_ratio"],
        },
        "target_isolation": iso_report,
        "unobserved": {"bottom_surface": "UNKNOWN", "filled": False},
        "acceptance": acceptance_section,
        "filters": {"refinement": "REJECTED", "depth_edge": "disabled", "multiview_consistency": "disabled"},
        "provenance_dir": args.output,
    }

    qpath = os.path.join(args.output, "reconstruction_quality_report.json")
    with open(qpath, "w") as f:
        json.dump(quality, f, indent=2, default=str)

    # ── 保存 unknown_surface_regions.json ──
    with open(os.path.join(args.output, "unknown_surface_regions.json"), "w") as f:
        json.dump({"bottom_surface": {"observed": False, "status": "UNKNOWN", "filled": False},
                   "reconstruction_type": "partial_visible_surface", "watertight": False}, f, indent=2)

    # ── 保存 effective config ──
    eff_cfg_path = os.path.join(args.output, "effective_config.yaml")
    shutil.copy2(args.config or os.path.join(WS, "config", "reconstruction", "visible_surface_production.yaml"), eff_cfg_path)

    # ── 控制台 ──
    print(f"\n{'='*60}")
    print(f"可见表面重建完成")
    print(f"{'='*60}")
    print(f"  TSDF: {comp_report['total_vertices_raw']}→{comp_report['total_vertices_cleaned']} verts")
    print(f"  分量: {comp_report['raw_components']}→{comp_report['cleaned_components']}")
    print(f"  删除面积比: {comp_report['removed_fragment_area_ratio']:.3f}")
    print(f"  底面: UNKNOWN (未补全)")
    print(f"\n📁 {args.output}")


if __name__ == "__main__":
    main()

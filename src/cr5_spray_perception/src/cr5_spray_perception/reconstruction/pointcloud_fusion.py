"""
CR5 Reconstruction — 三相机点云融合与重叠度量.

提供:
  - PointCloudFusion: 三相机点云融合器
  - fuse_three_camera_pointclouds: 顶层融合函数
  - CAMERA_COLORS: 按相机着色的默认颜色

生产链路: 禁止导入 gazebo_msgs / tf2_ros / model_states.
禁止默认开启 ICP 对齐。

用法:
  from cr5_spray_perception.reconstruction.pointcloud_fusion import (
      fuse_three_camera_pointclouds, CAMERA_COLORS,
  )
"""

import os, json, logging
from typing import Dict, List, Optional, Tuple

import numpy as np

from cr5_spray_perception.reconstruction.transforms import (
    convert_depth_to_meters, depth_image_to_pointcloud,
    transform_pointcloud, crop_pointcloud_aabb,
    compute_bidirectional_overlap, downsample_pointcloud,
)
from cr5_spray_perception.reconstruction.extrinsics import (
    get_T_rig_camera, get_T_camera_rig,
)

logger = logging.getLogger(__name__)

# 按相机着色颜色 (BGR for Open3D, 0-1 range)
CAMERA_COLORS = {
    "cam_front_left":  [1.0, 0.2, 0.2],  # 红色
    "cam_front_right": [0.2, 1.0, 0.2],  # 绿色
    "cam_rear":        [0.2, 0.4, 1.0],  # 蓝色
}

# 重叠对
OVERLAP_PAIRS = [
    ("cam_front_left", "cam_front_right"),
    ("cam_front_left", "cam_rear"),
    ("cam_front_right", "cam_rear"),
]


def _has_open3d() -> bool:
    try:
        import open3d
        return True
    except ImportError:
        return False


def _write_ply_colored(filepath: str, points: np.ndarray, colors: np.ndarray):
    """写入带颜色的 PLY 点云.

    Args:
        filepath: 输出路径.
        points: (N, 3) float32.
        colors: (N, 3) float32 [0,1].
    """
    if _has_open3d():
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
        o3d.io.write_point_cloud(filepath, pcd)
    else:
        # 手动写入 PLY
        N = points.shape[0]
        with open(filepath, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {N}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            for i in range(N):
                r = int(np.clip(colors[i, 0] * 255, 0, 255))
                g = int(np.clip(colors[i, 1] * 255, 0, 255))
                b = int(np.clip(colors[i, 2] * 255, 0, 255))
                f.write(f"{points[i,0]:.6f} {points[i,1]:.6f} {points[i,2]:.6f} {r} {g} {b}\n")


def _write_ply_rgb(filepath: str, points: np.ndarray, rgb_colors: np.ndarray):
    """写入带原始 RGB 颜色的 PLY 点云.

    Args:
        filepath: 输出路径.
        points: (N, 3) float32.
        rgb_colors: (N, 3) float32 [0,1].
    """
    _write_ply_colored(filepath, points, rgb_colors)


def backproject_camera_pointcloud(
    rgbd_data,                 # RGBDData
    depth_min_m: float,
    depth_max_m: float,
    expected_depth_unit: str = "meter",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """从 RGBDData 反投影生成 camera optical frame 点云.

    Args:
        rgbd_data: RGBDData 实例 (已加载).
        depth_min_m: 最小深度 (米).
        depth_max_m: 最大深度 (米).
        expected_depth_unit: 期望的深度单位.

    Returns:
        (points_cam, valid_mask, colors_rgb) 元组.
          - points_cam: (N,3) float32, camera optical frame.
          - valid_mask: (H,W) bool.
          - colors_rgb: (N,3) float32 [0,1] RGB.
    """
    # 深度转换
    depth_m, detected_unit = convert_depth_to_meters(
        rgbd_data.depth_raw, expected_depth_unit)

    # 反投影 (使用 depth CameraInfo K)
    points_cam, valid_mask = depth_image_to_pointcloud(
        depth_m, rgbd_data.depth_K, depth_min_m, depth_max_m)

    # 提取有效像素的颜色 (从 BGR 转 RGB)
    color_rgb = cv2.cvtColor(rgbd_data.color, cv2.COLOR_BGR2RGB) if rgbd_data.color is not None else None
    if color_rgb is not None:
        colors_rgb = color_rgb[valid_mask].astype(np.float32) / 255.0
    else:
        colors_rgb = np.zeros((points_cam.shape[0], 3), dtype=np.float32)

    return points_cam, valid_mask, colors_rgb


# 延迟导入 cv2
import cv2


def fuse_three_camera_pointclouds(
    rgbd_list: list,             # List[RGBDData]
    calibrated_rig: dict,
    config: dict,
    output_dir: str,
    allow_icp: bool = False,
) -> dict:
    """三相机点云融合主函数.

    流程:
      1. 每台相机反投影 → camera optical frame 点云
      2. T_rig_camera 变换 → rig frame
      3. AABB 裁剪
      4. 体素下采样
      5. 输出独立 PLY + 按相机着色融合 PLY + RGB 融合 PLY
      6. 配对重叠度量

    Args:
        rgbd_list: 三台相机的 RGBDData 列表.
        calibrated_rig: calibrated_rig 数据.
        config: 融合配置 dict.
        output_dir: 输出目录.
        allow_icp: 是否允许 diagnostic ICP (默认 False, ICP 后结果不作为正式外参).

    Returns:
        dict: 包含 paths, metrics, stats 的结果.
    """
    os.makedirs(output_dir, exist_ok=True)

    depth_min_m = config.get("depth_min_m", 0.15)
    depth_max_m = config.get("depth_max_m", 2.0)
    voxel_size_m = config.get("voxel_downsample_m", 0.005)
    roi_min = np.array(config.get("roi_rig", {}).get("min", [-1.0, -1.0, -1.0]))
    roi_max = np.array(config.get("roi_rig", {}).get("max", [2.0, 1.0, 2.0]))
    overlap_thresholds_mm = config.get("overlap_thresholds_mm", [10, 20, 30])
    camera_names = config.get("camera_names", ["cam_front_left", "cam_front_right", "cam_rear"])

    # 构建 {cam_name: RGBDData} 映射
    rgbd_map = {r.camera_name: r for r in rgbd_list}

    per_camera_pcd = {}        # camera frame 点云
    per_camera_pcd_rig = {}    # rig frame 点云
    per_camera_colors = {}     # RGB 颜色
    per_camera_stats = {}      # 统计

    all_rig_points = []
    all_camera_colors = []     # 按相机着色 (FL=红, FR=绿, RE=蓝)
    all_rgb_colors = []

    for cam_name in camera_names:
        if cam_name not in rgbd_map:
            logger.warning(f"跳过缺失相机: {cam_name}")
            continue

        rgbd = rgbd_map[cam_name]
        T_rc = get_T_rig_camera(calibrated_rig, cam_name)

        logger.info(f"[{cam_name}] 反投影...")
        points_cam, valid_mask, colors_rgb = backproject_camera_pointcloud(
            rgbd, depth_min_m, depth_max_m)
        n_raw = points_cam.shape[0]
        logger.info(f"[{cam_name}] 原始点数: {n_raw}")

        # 变换到 rig frame
        logger.info(f"[{cam_name}] 变换到 rig frame...")
        points_rig = transform_pointcloud(points_cam, T_rc)

        # AABB 裁剪
        points_rig_cropped, crop_mask = crop_pointcloud_aabb(points_rig, roi_min, roi_max)
        colors_rgb_cropped = colors_rgb[crop_mask]
        n_cropped = points_rig_cropped.shape[0]
        logger.info(f"[{cam_name}] AABB 裁剪: {n_raw} → {n_cropped}")

        # 体素下采样
        if voxel_size_m > 0:
            # 先合并点云和颜色进行下采样
            try:
                import open3d as o3d
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(
                    points_rig_cropped.astype(np.float64))
                pcd.colors = o3d.utility.Vector3dVector(
                    colors_rgb_cropped.astype(np.float64))
                pcd_down = pcd.voxel_down_sample(voxel_size_m)
                points_rig_cropped = np.asarray(pcd_down.points, dtype=np.float32)
                colors_rgb_cropped = np.asarray(pcd_down.colors, dtype=np.float32)
                n_down = points_rig_cropped.shape[0]
                logger.info(f"[{cam_name}] 下采样: {n_cropped} → {n_down}")
            except ImportError:
                logger.warning(f"[{cam_name}] Open3D 不可用, 跳过下采样")
                n_down = n_cropped

        per_camera_pcd[cam_name] = points_cam
        per_camera_pcd_rig[cam_name] = points_rig_cropped
        per_camera_colors[cam_name] = colors_rgb_cropped
        per_camera_stats[cam_name] = {
            "n_raw": int(n_raw),
            "n_after_crop": int(n_cropped),
            "n_final": int(points_rig_cropped.shape[0]),
        }

        # 累积到融合列表
        all_rig_points.append(points_rig_cropped)
        cam_color = np.tile(np.array(CAMERA_COLORS.get(cam_name, [0.5, 0.5, 0.5]),
                                     dtype=np.float32),
                            (points_rig_cropped.shape[0], 1))
        all_camera_colors.append(cam_color)
        all_rgb_colors.append(colors_rgb_cropped)

    # ── 输出独立 PLY ──
    per_camera_dir = os.path.join(output_dir, "per_camera")
    os.makedirs(per_camera_dir, exist_ok=True)
    for cam_name in camera_names:
        if cam_name in per_camera_pcd_rig:
            ply_path = os.path.join(per_camera_dir, f"{cam_name}.ply")
            _write_ply_rgb(ply_path, per_camera_pcd_rig[cam_name],
                          per_camera_colors[cam_name])
            logger.info(f"独立 PLY 已保存: {ply_path}")

    # ── 融合 PLY ──
    fused_dir = os.path.join(output_dir, "fused")
    os.makedirs(fused_dir, exist_ok=True)

    fused_points = np.vstack(all_rig_points) if all_rig_points else np.zeros((0, 3))
    fused_cam_colors = np.vstack(all_camera_colors) if all_camera_colors else np.zeros((0, 3))
    fused_rgb_colors = np.vstack(all_rgb_colors) if all_rgb_colors else np.zeros((0, 3))

    # 融合点云整体去重下采样
    logger.info("融合点云去重下采样...")
    if voxel_size_m > 0 and fused_points.shape[0] > 0:
        try:
            import open3d as o3d
            pcd_fused = o3d.geometry.PointCloud()
            pcd_fused.points = o3d.utility.Vector3dVector(
                fused_points.astype(np.float64))
            pcd_fused.colors = o3d.utility.Vector3dVector(
                fused_cam_colors.astype(np.float64))
            pcd_fused_down = pcd_fused.voxel_down_sample(voxel_size_m * 0.5)
            fused_points = np.asarray(pcd_fused_down.points, dtype=np.float32)
            fused_cam_colors = np.asarray(pcd_fused_down.colors, dtype=np.float32)
            # RGB 颜色也进行同样的下采样
            pcd_fused_rgb = o3d.geometry.PointCloud()
            pcd_fused_rgb.points = o3d.utility.Vector3dVector(
                np.vstack(all_rig_points).astype(np.float64))
            pcd_fused_rgb.colors = o3d.utility.Vector3dVector(
                fused_rgb_colors.astype(np.float64))
            pcd_fused_rgb_down = pcd_fused_rgb.voxel_down_sample(voxel_size_m * 0.5)
            fused_rgb_colors = np.asarray(pcd_fused_rgb_down.colors, dtype=np.float32)
            logger.info(f"融合去重: {np.vstack(all_rig_points).shape[0]} → {fused_points.shape[0]}")
        except ImportError:
            pass

    colored_ply_path = os.path.join(fused_dir, "fused_colored_by_camera.ply")
    _write_ply_colored(colored_ply_path, fused_points, fused_cam_colors)
    logger.info(f"按相机着色融合 PLY: {colored_ply_path}")

    rgb_ply_path = os.path.join(fused_dir, "fused_rgb.ply")
    _write_ply_rgb(rgb_ply_path, fused_points, fused_rgb_colors)
    logger.info(f"RGB 融合 PLY: {rgb_ply_path}")

    # ── 重叠度量 ──
    overlap_metrics = {}
    for cam_a, cam_b in OVERLAP_PAIRS:
        if cam_a not in per_camera_pcd_rig or cam_b not in per_camera_pcd_rig:
            continue
        pair_key = f"{cam_a}_{cam_b}"
        logger.info(f"计算重叠度量: {pair_key}...")
        points_a = per_camera_pcd_rig[cam_a]
        points_b = per_camera_pcd_rig[cam_b]

        # 双向最近邻距离
        metrics = compute_bidirectional_overlap(
            points_a, points_b,
            label_a=cam_a, label_b=cam_b,
            thresholds_mm=overlap_thresholds_mm,
        )
        overlap_metrics[pair_key] = metrics

    # ── 构建结果 ──
    result = {
        "paths": {
            "per_camera_dir": per_camera_dir,
            "fused_dir": fused_dir,
            "fused_colored_by_camera": colored_ply_path,
            "fused_rgb": rgb_ply_path,
        },
        "per_camera_stats": per_camera_stats,
        "fused_stats": {
            "n_points": int(fused_points.shape[0]),
        },
        "overlap_metrics": overlap_metrics,
        "config_summary": {
            "depth_min_m": depth_min_m,
            "depth_max_m": depth_max_m,
            "voxel_downsample_m": voxel_size_m,
            "roi_min": roi_min.tolist(),
            "roi_max": roi_max.tolist(),
            "allow_icp": allow_icp,
        },
    }

    # 保存指标
    metrics_path = os.path.join(fused_dir, "overlap_metrics.json")
    with open(metrics_path, "w") as f:
        json.dump(overlap_metrics, f, indent=2, default=str)
    logger.info(f"重叠指标已保存: {metrics_path}")

    return result

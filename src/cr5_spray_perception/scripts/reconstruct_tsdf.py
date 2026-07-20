#!/usr/bin/env python3
"""
CR5 Spray Demo: TSDF 三维重建
支持位姿模式: ground_truth / noisy_pose
使用 object_frame 坐标系融合，每台相机使用各自独立内参。
"""
import os
import sys
import yaml
import json
import argparse
import logging
import numpy as np

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("reconstruct_tsdf")

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    logger.error("Open3D not available")
    sys.exit(1)


def quat_to_rot(x, y, z, w):
    """Quaternion (x,y,z,w) to 3x3 rotation matrix."""
    return o3d.geometry.get_rotation_matrix_from_quaternion(
        np.array([w, x, y, z]))


def load_intrinsic(cinfo_file):
    """Load per-camera intrinsic from yaml CameraInfo."""
    with open(cinfo_file) as f:
        ci = yaml.safe_load(f)
    return o3d.camera.PinholeCameraIntrinsic(
        ci["width"], ci["height"],
        ci["K"][0], ci["K"][4],
        ci["K"][2], ci["K"][5])


def load_object_pose(tf_file):
    """Load T_world_camera from yaml, return as 4x4 matrix."""
    with open(tf_file) as f:
        d = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(
        d["rotation"]["x"], d["rotation"]["y"],
        d["rotation"]["z"], d["rotation"]["w"])
    T[:3, 3] = [d["translation"]["x"],
                d["translation"]["y"],
                d["translation"]["z"]]
    return T


def load_object_tf(tf_file):
    """Load T_world_object from file, return 4x4 or None."""
    if not os.path.exists(tf_file):
        return None
    with open(tf_file) as f:
        d = yaml.safe_load(f)
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(
        d["rotation"]["x"], d["rotation"]["y"],
        d["rotation"]["z"], d["rotation"]["w"])
    T[:3, 3] = [d["translation"]["x"],
                d["translation"]["y"],
                d["translation"]["z"]]
    return T


def run(dataset_dir, output_dir, config, pose_mode="ground_truth"):
    """Main TSDF reconstruction."""

    # All parameters in meters
    voxel_length = config.get("voxel_length", 0.005)
    sdf_trunc = config.get("sdf_trunc_m", 0.04)  # meters
    depth_trunc = config.get("depth_trunc_m", 2.0)  # meters
    clean_small = config.get("clean_small_components", True)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    views_dir = os.path.join(dataset_dir, "views")
    if not os.path.isdir(views_dir):
        logger.error("No views directory: %s", views_dir)
        return None

    view_dirs = sorted(os.listdir(views_dir))
    fused_views = 0

    # Try to load T_world_object for object_frame reconstruction
    manifest_file = os.path.join(dataset_dir, "manifest.yaml")
    T_world_object = None
    if os.path.exists(manifest_file):
        with open(manifest_file) as f:
            manifest = yaml.safe_load(f)
        obj_tf_file = manifest.get("object_tf_file", "")
        if obj_tf_file:
            T_world_object = load_object_tf(os.path.join(dataset_dir, obj_tf_file))

    for view_id in view_dirs:
        view_path = os.path.join(views_dir, view_id)
        if not os.path.isdir(view_path):
            continue

        for cam_name in sorted(os.listdir(view_path)):
            cam_path = os.path.join(view_path, cam_name)
            if not os.path.isdir(cam_path):
                continue

            color_file = os.path.join(cam_path, "color.png")
            depth_file = os.path.join(cam_path, "depth.npy")
            cinfo_file = os.path.join(cam_path, "color_camera_info.yaml")
            tf_file = os.path.join(cam_path, "T_world_camera.yaml")

            if not all(os.path.exists(f) for f in
                       [color_file, depth_file, cinfo_file, tf_file]):
                logger.warning("Missing files for %s/%s, skipping", view_id, cam_name)
                continue

            # Load data
            color = o3d.io.read_image(color_file)
            depth_npy = np.load(depth_file)

            # Determine depth scale from encoding
            if depth_npy.dtype == np.uint16:
                depth_o3d = o3d.geometry.Image(depth_npy)
                depth_scale = 1000.0  # mm → m
            elif depth_npy.dtype == np.float32:
                # Already in meters, convert to mm for O3D RGBDImage
                depth_o3d = o3d.geometry.Image(
                    (depth_npy * 1000.0).astype(np.uint16))
                depth_scale = 1000.0
            else:
                logger.warning("Unknown depth dtype %s, skipping", depth_npy.dtype)
                continue

            # Per-camera intrinsic (NOT shared)
            intrinsic = load_intrinsic(cinfo_file)

            # Load T_world_camera
            T_world_camera = load_object_pose(tf_file)

            # Transform to object_frame if available
            if T_world_object is not None:
                T_object_camera = np.linalg.inv(T_world_object) @ T_world_camera
            else:
                T_object_camera = T_world_camera

            # Pose noise if requested
            if pose_mode == "noisy_pose":
                noise_trans_m = config.get("noise_translation_m", 0.005)
                noise_rot_deg = config.get("noise_rotation_deg", 1.0)
                T_object_camera[:3, 3] += np.random.randn(3) * noise_trans_m
                angle = np.deg2rad(noise_rot_deg)
                axis = np.random.randn(3)
                axis /= np.linalg.norm(axis)
                R_noise = o3d.geometry.get_rotation_matrix_from_axis_angle(
                    axis * angle)
                T_object_camera[:3, :3] = R_noise @ T_object_camera[:3, :3]

            # Create RGBD and integrate
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth_o3d,
                depth_scale=depth_scale,
                depth_trunc=depth_trunc,
                convert_rgb_to_intensity=False)
            volume.integrate(rgbd, intrinsic,
                             np.linalg.inv(T_object_camera))
            fused_views += 1

    logger.info("Fused %d views into TSDF volume (voxel=%.4fm, sdf_trunc=%.3fm)",
                fused_views, voxel_length, sdf_trunc)

    if fused_views == 0:
        logger.error("No views fused! Check dataset and file paths.")
        return None

    # Extract mesh
    mesh = volume.extract_triangle_mesh()

    if len(mesh.triangles) == 0:
        logger.error("Extracted mesh has 0 triangles!")
        return None

    mesh.compute_vertex_normals()
    mesh.remove_unreferenced_vertices()

    if clean_small and len(mesh.triangles) > 0:
        try:
            triangle_clusters, cluster_n_triangles, _ = \
                mesh.cluster_connected_triangles()
            if len(cluster_n_triangles) > 1:
                largest_idx = np.argmax(cluster_n_triangles)
                mesh.remove_triangles_by_mask(
                    np.asarray(triangle_clusters) != largest_idx)
                mesh.remove_unreferenced_vertices()
        except Exception as e:
            logger.warning("Small component cleanup failed: %s", e)

    mesh.compute_vertex_normals()

    # Save
    os.makedirs(output_dir, exist_ok=True)
    out_ply = os.path.join(output_dir, "reconstructed_mesh.ply")
    o3d.io.write_triangle_mesh(out_ply, mesh)

    # Save point cloud
    pcd = mesh.sample_points_uniformly(50000)
    out_pcd = os.path.join(output_dir, "reconstructed_pointcloud.ply")
    o3d.io.write_point_cloud(out_pcd, pcd)

    # Metadata
    metadata = {
        "fused_views": fused_views,
        "vertices": len(mesh.vertices),
        "triangles": len(mesh.triangles),
        "voxel_length_m": voxel_length,
        "sdf_trunc_m": sdf_trunc,
        "depth_trunc_m": depth_trunc,
        "pose_mode": pose_mode,
        "object_frame": T_world_object is not None,
    }
    with open(os.path.join(output_dir, "reconstruction_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    logger.info("Mesh saved: %s (%d vertices, %d triangles)",
                out_ply, len(mesh.vertices), len(mesh.triangles))

    return mesh


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_dir", help="Dataset root directory")
    parser.add_argument("--output", default=".", help="Output directory")
    parser.add_argument("--pose-mode", default="ground_truth",
                        choices=["ground_truth", "noisy_pose"],
                        help="Pose source mode (refined_pose not yet implemented)")
    parser.add_argument("--voxel-length", type=float, default=0.005,
                        help="TSDF voxel length (meters)")
    parser.add_argument("--sdf-trunc", type=float, default=0.04,
                        help="SDF truncation distance (meters)")
    parser.add_argument("--depth-trunc", type=float, default=2.0,
                        help="Depth truncation (meters)")
    args = parser.parse_args()

    config = {
        "voxel_length": args.voxel_length,
        "sdf_trunc_m": args.sdf_trunc,
        "depth_trunc_m": args.depth_trunc,
        "clean_small_components": True,
    }

    mesh = run(args.dataset_dir, args.output, config, args.pose_mode)
    if mesh is None:
        logger.error("Reconstruction failed")
        sys.exit(1)

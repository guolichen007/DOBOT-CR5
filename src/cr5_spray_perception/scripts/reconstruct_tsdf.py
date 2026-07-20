#!/usr/bin/env python3
"""
CR5 Spray Demo: TSDF 三维重建
支持三种位姿模式: ground_truth / noisy_pose / refined_pose
"""
import os
import sys
import yaml
import numpy as np
import rospy
import argparse

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    rospy.logwarn("Open3D not available; reconstruction disabled")


class TSDFReconstructor:
    def __init__(self, dataset_dir, output_dir, config):
        self.dataset_dir = dataset_dir
        self.output_dir = output_dir
        self.config = config
        os.makedirs(output_dir, exist_ok=True)

    def run(self, pose_mode="ground_truth"):
        if not HAS_OPEN3D:
            rospy.logerr("Cannot reconstruct: Open3D not installed")
            return None

        voxel_length = self.config.get("voxel_length", 0.005)
        sdf_trunc = self.config.get("sdf_trunc", 0.04)
        depth_trunc = self.config.get("depth_trunc", 2.0)

        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=voxel_length,
            sdf_trunc=sdf_trunc * voxel_length,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

        views_dir = os.path.join(self.dataset_dir, "views")
        if not os.path.isdir(views_dir):
            rospy.logerr("No views directory found: %s", views_dir)
            return None

        view_dirs = sorted(os.listdir(views_dir))
        intrinsic = None
        fused_views = 0

        for view_id in view_dirs:
            view_path = os.path.join(views_dir, view_id)
            for cam_name in os.listdir(view_path):
                cam_path = os.path.join(view_path, cam_name)
                color_file = os.path.join(cam_path, "color.png")
                depth_file = os.path.join(cam_path, "depth.npy")
                cinfo_file = os.path.join(cam_path, "color_camera_info.yaml")
                tf_file = os.path.join(cam_path, "T_world_camera.yaml")

                if not all(os.path.exists(f) for f in
                           [color_file, depth_file, cinfo_file, tf_file]):
                    continue

                # Load data
                color = o3d.io.read_image(color_file)
                depth_npy = np.load(depth_file)

                # Convert depth to O3D image (16UC1 in mm)
                if depth_npy.dtype == np.uint16:
                    depth_o3d = o3d.geometry.Image(depth_npy)
                elif depth_npy.dtype == np.float32:
                    depth_o3d = o3d.geometry.Image(
                        (depth_npy * 1000).astype(np.uint16))
                else:
                    continue

                # Load CameraInfo
                with open(cinfo_file) as f:
                    ci = yaml.safe_load(f)
                if intrinsic is None:
                    intrinsic = o3d.camera.PinholeCameraIntrinsic(
                        ci["width"], ci["height"],
                        ci["K"][0], ci["K"][4],
                        ci["K"][2], ci["K"][5"])

                # Load pose
                with open(tf_file) as f:
                    tf_dict = yaml.safe_load(f)
                T = np.eye(4)
                T[:3, :3] = self._quat_to_rot(
                    tf_dict["rotation"]["x"], tf_dict["rotation"]["y"],
                    tf_dict["rotation"]["z"], tf_dict["rotation"]["w"])
                T[:3, 3] = [tf_dict["translation"]["x"],
                            tf_dict["translation"]["y"],
                            tf_dict["translation"]["z"]]

                if pose_mode == "noisy_pose":
                    # Add configurable noise
                    noise_trans = self.config.get("noise_translation", 0.01)
                    noise_rot = self.config.get("noise_rotation_deg", 1.0)
                    T[:3, 3] += np.random.randn(3) * noise_trans
                    # Small random rotation perturbation
                    angle = np.deg2rad(noise_rot)
                    axis = np.random.randn(3)
                    axis /= np.linalg.norm(axis)
                    R_noise = o3d.geometry.get_rotation_matrix_from_axis_angle(
                        axis * angle)
                    T[:3, :3] = R_noise @ T[:3, :3]

                # Integrate
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color, depth_o3d,
                    depth_scale=1000.0, depth_trunc=depth_trunc,
                    convert_rgb_to_intensity=False)
                volume.integrate(rgbd, intrinsic, np.linalg.inv(T))
                fused_views += 1

        rospy.loginfo("Fused %d views into TSDF volume", fused_views)

        # Extract mesh
        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()

        # Remove small components
        mesh.remove_unreferenced_vertices()
        if self.config.get("clean_small_components", True):
            triangle_clusters, cluster_n_triangles, _ = \
                mesh.cluster_connected_triangles()
            largest_idx = np.argmax(cluster_n_triangles)
            mesh.remove_triangles_by_mask(
                triangle_clusters != largest_idx)

        mesh.compute_vertex_normals()

        # Save
        out_ply = os.path.join(self.output_dir, "reconstructed_mesh.ply")
        o3d.io.write_triangle_mesh(out_ply, mesh)
        rospy.loginfo("Mesh saved: %s (%d vertices, %d triangles)",
                      out_ply, len(mesh.vertices), len(mesh.triangles))

        return mesh

    @staticmethod
    def _quat_to_rot(x, y, z, w):
        """Quaternion to rotation matrix."""
        return o3d.geometry.get_rotation_matrix_from_quaternion(
            np.array([w, x, y, z]))


if __name__ == "__main__":
    rospy.init_node("reconstruct_tsdf")
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_dir")
    parser.add_argument("--output", default=".")
    parser.add_argument("--pose-mode", default="ground_truth",
                        choices=["ground_truth", "noisy_pose", "refined_pose"])
    parser.add_argument("--voxel-length", type=float, default=0.005)
    args = parser.parse_args()

    config = {
        "voxel_length": args.voxel_length,
        "sdf_trunc": 8,
        "depth_trunc": 2.0,
        "clean_small_components": True,
    }

    recon = TSDFReconstructor(args.dataset_dir, args.output, config)
    recon.run(pose_mode=args.pose_mode)

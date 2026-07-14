import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

def adjust_orientation_to_y_axis(quaternion, target_direction=[0, 1, 0]):
    """
    调整四元数，使得朝向与目标方向（默认为Y轴正方向）成锐角
    
    参数:
        quaternion: 四元数 [x, y, z, w]
        target_direction: 目标方向向量
    
    返回:
        调整后的四元数 [x, y, z, w]
    """
    # 将四元数转换为旋转矩阵
    rotation = R.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()
    
    # 提取当前朝向（假设为第一主成分方向）
    current_direction = rotation_matrix[:, 0]  # 第一列通常是第一主成分
    
    # 计算当前朝向与目标方向的点积
    dot_product = np.dot(current_direction, target_direction)
    
    # 如果点积为负，说明夹角大于90度，需要翻转180度
    if dot_product < 0:
        # 创建绕任意轴旋转180度的四元数
        # 这里我们绕Z轴旋转180度，保持水平方向
        flip_quaternion = R.from_euler('z', 180, degrees=True).as_quat()
        
        # 组合旋转：先应用原始旋转，再应用180度翻转
        adjusted_rotation = rotation * R.from_quat(flip_quaternion)
        adjusted_quaternion = adjusted_rotation.as_quat()
        
        print(f"  朝向调整: 与Y轴夹角为{np.arccos(abs(dot_product))*180/np.pi:.1f}度，已翻转180度")
        return adjusted_quaternion
    else:
        print(f"  朝向保持: 与Y轴夹角为{np.arccos(abs(dot_product))*180/np.pi:.1f}度")
        return quaternion
    
def cluster_point_cloud_and_compute_poses(pcd_path, eps=0.02, min_points=20):
    """
    加载点云，进行聚类，并计算每个聚类的质心位姿
    
    参数:
        pcd_path: PCD文件路径
        eps: DBSCAN聚类距离阈值
        min_points: 形成聚类的最小点数
    
    返回:
        poses: 每个聚类的位姿列表
        clustered_pcd: 带颜色的聚类点云
    """
    # 1. 加载点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f"原始点云点数: {len(pcd.points)}")
    
    # 2. 预处理：降采样和去噪
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"预处理后点数: {len(pcd.points)}")
    
    # 3. 使用DBSCAN进行聚类
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    max_label = labels.max()
    print(f"聚类数量: {max_label + 1}")
    
    # 4. 为每个聚类分配随机颜色
    colors = np.random.random((max_label + 1, 3))
    colors[0] = [0, 0, 0]  # 噪声点设为黑色
    colored_pcd = o3d.geometry.PointCloud()
    colored_pcd.points = pcd.points
    colored_pcd.colors = o3d.utility.Vector3dVector(colors[labels])
    
    # 5. 计算每个聚类的质心位姿
    poses = []
    for cluster_id in range(0, max_label + 1):
        # 提取当前聚类的点
        cluster_indices = np.where(labels == cluster_id)[0]
        cluster_points = np.asarray(pcd.points)[cluster_indices]
        
        if len(cluster_points) < min_points:
            continue
            
        # 计算质心
        centroid = np.mean(cluster_points, axis=0)
        
        # 计算协方差矩阵和主方向
        cov_matrix = np.cov(cluster_points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # 按特征值降序排序
        sorted_indices = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, sorted_indices]
        
        # 确保右手坐标系
        if np.linalg.det(eigenvectors) < 0:
            eigenvectors[:, 2] = -eigenvectors[:, 2]
        
        # 将旋转矩阵转换为四元数
        rotation_matrix = eigenvectors
        
        quaternion = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]

        # 调整朝向，使其与Y轴正方向成锐角
        adjusted_quaternion = adjust_orientation_to_y_axis(quaternion)


        # 创建位姿对象
        pose = {
            'position': {
                'x': float(centroid[0]),
                'y': float(centroid[1]),
                'z': float(centroid[2])
            },

            'orientation': {
                'x': float(0.677),
                'y': float(0.022),
                'z': float(-0.033),
                'w': float(-0.735)
            }
            # 'orientation': {
            #     'x': float(adjusted_quaternion[0]),
            #     'y': float(adjusted_quaternion[1]),
            #     'z': float(adjusted_quaternion[2]),
            #     'w': float(adjusted_quaternion[3])
            # }
        }
        
        poses.append(pose)
        
        print(f"聚类 {cluster_id}:")
        print(f"  点数: {len(cluster_points)}")
        print(f"  质心位置: ({pose['position']['x']:.3f}, {pose['position']['y']:.3f}, {pose['position']['z']:.3f})")
        print(f"  朝向四元数: ({pose['orientation']['x']:.3f}, {pose['orientation']['y']:.3f}, {pose['orientation']['z']:.3f}, {pose['orientation']['w']:.3f})")
    
    return poses, colored_pcd

def visualize_results(pcd, poses):
    """
    可视化聚类结果和质心
    """
    # 创建坐标系
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    # 创建质心标记
    centroid_markers = []
    for i, pose in enumerate(poses):
        # 质心球体
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
        sphere.paint_uniform_color([1, 0, 0])  # 红色
        sphere.translate([pose['position']['x'], pose['position']['y'], pose['position']['z']])
        centroid_markers.append(sphere)
        
        # 方向箭头
        rotation_matrix = R.from_quat([
            pose['orientation']['x'],
            pose['orientation']['y'], 
            pose['orientation']['z'],
            pose['orientation']['w']
        ]).as_matrix()
        
        # 创建箭头表示主方向
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=0.005, 
            cone_radius=0.008,
            cylinder_height=0.03, 
            cone_height=0.02
        )
        arrow.rotate(rotation_matrix)
        arrow.translate([pose['position']['x'], pose['position']['y'], pose['position']['z']])
        arrow.paint_uniform_color([0, 1, 0])  # 绿色
        centroid_markers.append(arrow)
    
    # 可视化
    o3d.visualization.draw_geometries([pcd, coordinate_frame] + centroid_markers)

# 使用示例
if __name__ == "__main__":
    # 替换为你的PCD文件路径
    pcd_file_path = "/home/youdao/cr5_ws/src/slamit/script/data/1759138709norm_modify.pcd"
    
    try:
        # 执行聚类和位姿计算
        poses, clustered_pcd = cluster_point_cloud_and_compute_poses(pcd_file_path)
        
        print(f"\n总共检测到 {len(poses)} 个有效聚类")
        
        # 可视化结果
        visualize_results(clustered_pcd, poses)
        
        # 保存带颜色的聚类点云
        o3d.io.write_point_cloud("clustered_point_cloud.pcd", clustered_pcd)
        
    except Exception as e:
        print(f"处理过程中出现错误: {e}")
import open3d as o3d
import numpy as np
import math

def visualize_comparison(original_pcd, filtered_pcd):
    """
    可视化原始点云和过滤后的点云对比
    """
    # 为点云设置不同颜色以便区分
    original_pcd.paint_uniform_color([1, 0, 0])  # 红色 - 原始点云
    filtered_pcd.paint_uniform_color([0, 1, 0])  # 绿色 - 过滤后的点云
    
    # 创建可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='点云法向量过滤对比')
    
    # 添加点云到可视化
    vis.add_geometry(original_pcd)
    vis.add_geometry(filtered_pcd)
    
    # 设置渲染选项
    opt = vis.get_render_option()
    opt.point_size = 2.0
    
    # 运行可视化
    vis.run()
    vis.destroy_window()


# 高级版本：使用向量化操作提高性能
def filter_points_by_normal_angle_fast(pcd, std_normal, max_angle_degrees=20):
    """
    使用向量化操作快速过滤点云（性能优化版本）
    """
    # 确保点云有法向量
    if not pcd.has_normals():
        print("点云没有法向量，正在计算法向量...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
    
    # 获取数据
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    # 归一化标准法向量
    std_normal = np.array(std_normal, dtype=np.float64)
    std_normal = std_normal / np.linalg.norm(std_normal)
    
    # 归一化所有法向量
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    valid_normals_mask = norms.flatten() > 0
    normals_normalized = normals.copy()
    normals_normalized[valid_normals_mask] = normals[valid_normals_mask] / norms[valid_normals_mask]
    
    # 计算点积（向量化操作）
    dot_products = np.sum(normals_normalized * std_normal, axis=1)
    dot_products = np.clip(dot_products, -1.0, 1.0)
    
    # 计算角度
    angles_rad = np.arccos(dot_products)
    angles_deg = np.degrees(angles_rad)
    
    # 创建过滤掩码
    valid_mask = angles_deg <= max_angle_degrees
    
    # 过滤点云
    filtered_pcd = pcd.select_by_index(np.where(valid_mask)[0])
    
    # 打印统计信息
    print(f"快速过滤结果:")
    print(f"原始点云点数: {len(points)}")
    print(f"过滤后点云点数: {np.sum(valid_mask)}")
    print(f"保留比例: {np.sum(valid_mask)/len(points)*100:.2f}%")
    print(f"法向量夹角范围: [{angles_deg.min():.2f}°, {angles_deg.max():.2f}°]")
    
    return filtered_pcd, np.where(valid_mask)[0]

if __name__ == "__main__":
    # 如果要使用快速版本，取消下面的注释
    pcd_file_path = "/home/youdao/cr5_ws/src/slamit/script/data/1759138709clip.pcd"
    std_normal = [-0.08, -0.96, -0.2]
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    filtered_pcd_fast, _ = filter_points_by_normal_angle_fast(pcd, std_normal, 30)
    output_file = pcd_file_path.replace('.pcd', '_filtered.pcd')
    o3d.io.write_point_cloud(output_file, filtered_pcd_fast)
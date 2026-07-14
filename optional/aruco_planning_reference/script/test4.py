import open3d as o3d
import numpy as np

def compute_average_normal_open3d(file_path):
    """
    使用Open3D计算点云的平均法向量
    """
    # 加载点云文件
    pcd = o3d.io.read_point_cloud(file_path)
    
    # 估计法向量（需要设置搜索半径或最近邻数量）
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    
    # 获取法向量
    normals = np.asarray(pcd.normals)
    
    # 计算平均法向量
    average_normal = np.mean(normals, axis=0)
    
    # 归一化平均法向量
    average_normal_normalized = average_normal / np.linalg.norm(average_normal)
    
    return average_normal_normalized, normals

# std_normal = [-0.08793624 -0.96318257 -0.2]


# 使用示例
if __name__ == "__main__":
    # 替换为您的点云文件路径
    file_path = "/home/youdao/cr5_ws/src/slamit/script/data/1759138709norm.pcd"  # 支持ply, pcd, xyz等格式
    
    try:
        avg_normal, all_normals = compute_average_normal_open3d(file_path)
        
        print("平均法向量:", avg_normal)
        print("法向量模长:", np.linalg.norm(avg_normal))
        print("点云中点的数量:", len(all_normals))
        
    except Exception as e:
        print(f"错误: {e}")
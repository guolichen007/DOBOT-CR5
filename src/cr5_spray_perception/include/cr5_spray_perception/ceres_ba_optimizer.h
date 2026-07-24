#ifndef CR5_SPRAY_PERCEPTION_CERES_BA_OPTIMIZER_H
#define CR5_SPRAY_PERCEPTION_CERES_BA_OPTIMIZER_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <Eigen/Core>

namespace cr5_spray {

// ────────────────────────────────────────────────────────────
// SE(3) 局部参数化: Quaternion [qw,qx,qy,qz] + Translation [tx,ty,tz]
// GlobalSize = 7, LocalSize = 6
//
// delta = [d_angle_x, d_angle_y, d_angle_z, d_tx, d_ty, d_tz]
// x     = [qw, qx, qy, qz, tx, ty, tz]
// ────────────────────────────────────────────────────────────
class SE3Parameterization : public ceres::LocalParameterization {
public:
  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override;
  bool ComputeJacobian(const double* x, double* jacobian) const override;
  int GlobalSize() const override { return 7; }
  int LocalSize() const override { return 6; }
};

// ────────────────────────────────────────────────────────────
// 重投影误差代价函数 (自动微分)
//
// camera_pose: [qw,qx,qy,qz,tx,ty,tz]  (T_world_camera)
// target_pose: [qw,qx,qy,qz,tx,ty,tz]  (T_world_target)
//   p_cam = T_world_camera.inverse() * T_world_target * P_3d
//   然后投影: u = fx * x/z + cx,  v = fy * y/z + cy
//
// 观测: (observed_x, observed_y) 像素坐标
// 3D 点: (point_x, point_y, point_z) 在 target 坐标系中
// ────────────────────────────────────────────────────────────
struct ReprojectionError {
  ReprojectionError(double fx, double fy, double cx, double cy,
                    double observed_x, double observed_y,
                    double point_x, double point_y, double point_z)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        ox_(observed_x), oy_(observed_y),
        px_(point_x), py_(point_y), pz_(point_z) {}

  template <typename T>
  bool operator()(const T* const camera_pose,
                  const T* const target_pose,
                  T* residuals) const;

private:
  double fx_, fy_, cx_, cy_;
  double ox_, oy_;
  double px_, py_, pz_;
};

// ────────────────────────────────────────────────────────────
// Bundle Adjuster: 管理观测 + 求解
// ────────────────────────────────────────────────────────────
struct Observation {
  int camera_idx;
  int target_idx;
  std::vector<double> obj_pts;   // [x0,y0,z0, x1,y1,z1, ...]
  std::vector<double> img_pts;   // [u0,v0, u1,v1, ...]
  double fx, fy, cx, cy;         // camera intrinsics
};

struct BAResult {
  bool success;
  double initial_cost;
  double final_cost;
  int iterations;
  double time_ms;
  std::vector<double> optimized_poses;  // flattened: 7 doubles per pose
  std::string message;
};

class CeresBundleAdjuster {
public:
  CeresBundleAdjuster();

  void add_observation(const Observation& obs);

  void set_initial_pose(int pose_idx,
                        const std::vector<double>& quat_trans);

  void set_pose_constant(int pose_idx);

  BAResult solve(int max_iterations = 500);

private:
  ceres::Problem problem_;
  std::vector<double> parameters_;   // all pose parameters
  std::vector<int> param_sizes_;     // 7 per pose
  double initial_cost_;
};

}  // namespace cr5_spray

#endif  // CR5_SPRAY_PERCEPTION_CERES_BA_OPTIMIZER_H

/**
 * Ceres Bundle Adjustment 优化器.
 *
 * SE(3) 局部参数化 + 自动微分重投影误差.
 * 多相机 + 多目标位姿联合优化.
 *
 * 用法: ceres_ba_optimizer input.json output.json
 * Python bundle_adjustment.py 负责 JSON 生成和解析.
 */
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using json = nlohmann::json;

// ════════════════════════════════════════════════════════════
// SE(3) 局部参数化: Quaternion [qw,qx,qy,qz] + Translation [tx,ty,tz]
// GlobalSize=7, LocalSize=6
// ════════════════════════════════════════════════════════════
class SE3Parameterization : public ceres::LocalParameterization {
public:
  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    double half_norm = 0.5 * std::sqrt(
        delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    double dq_w, dq_x, dq_y, dq_z;
    if (half_norm < 1e-12) {
      dq_w = 1.0;
      dq_x = 0.5 * delta[0]; dq_y = 0.5 * delta[1]; dq_z = 0.5 * delta[2];
    } else {
      double s = std::sin(half_norm) / (2.0 * half_norm);
      dq_w = std::cos(half_norm);
      dq_x = s * delta[0]; dq_y = s * delta[1]; dq_z = s * delta[2];
    }
    double qw = x[0], qx = x[1], qy = x[2], qz = x[3];
    x_plus_delta[0] = dq_w * qw - dq_x * qx - dq_y * qy - dq_z * qz;
    x_plus_delta[1] = dq_w * qx + dq_x * qw + dq_y * qz - dq_z * qy;
    x_plus_delta[2] = dq_w * qy - dq_x * qz + dq_y * qw + dq_z * qx;
    x_plus_delta[3] = dq_w * qz + dq_x * qy - dq_y * qx + dq_z * qw;
    x_plus_delta[4] = x[4] + delta[3];
    x_plus_delta[5] = x[5] + delta[4];
    x_plus_delta[6] = x[6] + delta[5];
    return true;
  }

  bool ComputeJacobian(const double* x, double* jacobian) const override {
    // Jacobian of left-multiply Plus (q' = dq ⊗ q) at delta=0.
    // Verified against numerical differentiation (2026-07-29).
    // 7 rows (qw,qx,qy,qz,tx,ty,tz) × 6 cols (drx,dry,drz,dtx,dty,dtz)
    std::fill(jacobian, jacobian + 7 * 6, 0.0);
    // ∂qw'/∂(drx,dry,drz)
    jacobian[0 * 6 + 0] = -0.5 * x[1];
    jacobian[0 * 6 + 1] = -0.5 * x[2];
    jacobian[0 * 6 + 2] = -0.5 * x[3];
    // ∂qx'/∂(drx,dry,drz)
    jacobian[1 * 6 + 0] =  0.5 * x[0];
    jacobian[1 * 6 + 1] =  0.5 * x[3];  // FIXED: was -0.5*x[3]
    jacobian[1 * 6 + 2] = -0.5 * x[2];  // FIXED: was +0.5*x[2]
    // ∂qy'/∂(drx,dry,drz)
    jacobian[2 * 6 + 0] = -0.5 * x[3];  // FIXED: was +0.5*x[3]
    jacobian[2 * 6 + 1] =  0.5 * x[0];
    jacobian[2 * 6 + 2] =  0.5 * x[1];  // FIXED: was -0.5*x[1]
    // ∂qz'/∂(drx,dry,drz)
    jacobian[3 * 6 + 0] =  0.5 * x[2];  // FIXED: was -0.5*x[2]
    jacobian[3 * 6 + 1] = -0.5 * x[1];  // FIXED: was +0.5*x[1]
    jacobian[3 * 6 + 2] =  0.5 * x[0];
    // ∂t/∂dt = Identity
    jacobian[4 * 6 + 3] = 1.0; jacobian[5 * 6 + 4] = 1.0;
    jacobian[6 * 6 + 5] = 1.0;
    return true;
  }

  int GlobalSize() const override { return 7; }
  int LocalSize() const override { return 6; }
};

// ════════════════════════════════════════════════════════════
// 重投影误差 (自动微分) — V8: Brown-Conrady 畸变支持
// ════════════════════════════════════════════════════════════
struct ReprojectionError {
  ReprojectionError(double fx, double fy, double cx, double cy,
                    double ox, double oy, double px, double py, double pz,
                    double k1, double k2, double p1, double p2, double k3,
                    double weight = 1.0)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        ox_(ox), oy_(oy), px_(px), py_(py), pz_(pz),
        k1_(k1), k2_(k2), p1_(p1), p2_(p2), k3_(k3),
        sqrt_weight_(std::sqrt(std::max(weight, 1e-12))) {}

  template <typename T>
  bool operator()(const T* const cam_pose, const T* const tgt_pose,
                  T* residual) const {
    // p_target → p_world
    T pw[3];
    T p_target[3] = { T(px_), T(py_), T(pz_) };
    ceres::QuaternionRotatePoint(tgt_pose, p_target, pw);
    pw[0] += tgt_pose[4]; pw[1] += tgt_pose[5]; pw[2] += tgt_pose[6];

    // p_world → p_camera
    T pc_minus[3] = { pw[0] - cam_pose[4], pw[1] - cam_pose[5], pw[2] - cam_pose[6] };
    T cam_q_conj[4] = { cam_pose[0], -cam_pose[1], -cam_pose[2], -cam_pose[3] };
    T pc[3];
    ceres::QuaternionRotatePoint(cam_q_conj, pc_minus, pc);

    // 深度保护: z ≤ 0 时返回大残差, 避免除零/负深度
    if (pc[2] <= T(1e-6)) {
      residual[0] = T(1e6) * T(sqrt_weight_);
      residual[1] = T(1e6) * T(sqrt_weight_);
      return true;
    }

    // Normalized coordinates
    T xp = pc[0] / pc[2];
    T yp = pc[1] / pc[2];

    // Brown-Conrady distortion (k1,k2,k3 radial + p1,p2 tangential)
    T r2 = xp * xp + yp * yp;
    T r4 = r2 * r2;
    T r6 = r2 * r4;
    T radial = T(1.0) + T(k1_) * r2 + T(k2_) * r4 + T(k3_) * r6;
    T x_dist = xp * radial + T(2.0) * T(p1_) * xp * yp + T(p2_) * (r2 + T(2.0) * xp * xp);
    T y_dist = yp * radial + T(p1_) * (r2 + T(2.0) * yp * yp) + T(2.0) * T(p2_) * xp * yp;

    // Pixel coordinates
    T u_pred = T(fx_) * x_dist + T(cx_);
    T v_pred = T(fy_) * y_dist + T(cy_);

    residual[0] = T(sqrt_weight_) * (T(ox_) - u_pred);
    residual[1] = T(sqrt_weight_) * (T(oy_) - v_pred);
    return true;
  }

private:
  double fx_, fy_, cx_, cy_, ox_, oy_, px_, py_, pz_;
  double k1_, k2_, p1_, p2_, k3_;
  double sqrt_weight_;
};

// ════════════════════════════════════════════════════════════
// Camera Prior: penalize deviation of a scalar parameter from target
// ════════════════════════════════════════════════════════════
struct ScalarPrior {
  ScalarPrior(double target, int param_idx, double weight)
      : target_(target), param_idx_(param_idx), weight_(weight) {}
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = weight_ * (pose[param_idx_] - T(target_));
    return true;
  }
  double target_, weight_;
  int param_idx_;
};

struct TranslationPrior {
  TranslationPrior(double target, double weight) : target_(target), weight_(weight) {}
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = weight_ * (pose[4] - T(target_));  // hardcoded for tx
    return true;
  }
  double target_, weight_;
};

// V6: Quaternion rotation prior — 3D tangent-space residual
//   residual[i] = w * Im(q_current * conj(q_target))[i]
//   For small rotations: residual ≈ w * (angle_axis / 2)
//   Use w = 2.0 / sigma_R to make residual dimensionless (~N(0,1) under prior)
struct QuaternionPrior : public ceres::SizedCostFunction<3, 7> {
  QuaternionPrior(const double* q_target, double weight)
      : weight_(weight) {
    for (int i = 0; i < 4; ++i) target_[i] = q_target[i];
  }

  bool Evaluate(const double* const* parameters,
                double* residuals, double** jacobians) const override {
    const double* q = parameters[0];
    // dq = q * conj(q_target)
    // Im(dq) = [x, y, z] where:
    //   x = -qw*tx + qx*tw - qy*tz + qz*ty
    //   y = -qw*ty + qx*tz + qy*tw - qz*tx
    //   z = -qw*tz - qx*ty + qy*tx + qz*tw
    // with (tw,tx,ty,tz) = target_ (original, NOT pre-conjugated)
    double tw = target_[0], tx = target_[1], ty = target_[2], tz = target_[3];
    double dq_x = -q[0]*tx + q[1]*tw - q[2]*tz + q[3]*ty;
    double dq_y = -q[0]*ty + q[1]*tz + q[2]*tw - q[3]*tx;
    double dq_z = -q[0]*tz - q[1]*ty + q[2]*tx + q[3]*tw;

    residuals[0] = weight_ * dq_x;
    residuals[1] = weight_ * dq_y;
    residuals[2] = weight_ * dq_z;

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      std::fill(jacobians[0], jacobians[0] + 3 * 7, 0.0);
      // ∂x/∂qw = -tx     ∂x/∂qx = tw     ∂x/∂qy = -tz    ∂x/∂qz = ty
      jacobians[0][0 * 7 + 0] = weight_ * (-tx);
      jacobians[0][0 * 7 + 1] = weight_ * tw;
      jacobians[0][0 * 7 + 2] = weight_ * (-tz);
      jacobians[0][0 * 7 + 3] = weight_ * ty;
      // ∂y/∂qw = -ty     ∂y/∂qx = tz     ∂y/∂qy = tw     ∂y/∂qz = -tx
      jacobians[0][1 * 7 + 0] = weight_ * (-ty);
      jacobians[0][1 * 7 + 1] = weight_ * tz;
      jacobians[0][1 * 7 + 2] = weight_ * tw;
      jacobians[0][1 * 7 + 3] = weight_ * (-tx);
      // ∂z/∂qw = -tz     ∂z/∂qx = -ty    ∂z/∂qy = tx     ∂z/∂qz = tw
      jacobians[0][2 * 7 + 0] = weight_ * (-tz);
      jacobians[0][2 * 7 + 1] = weight_ * (-ty);
      jacobians[0][2 * 7 + 2] = weight_ * tx;
      jacobians[0][2 * 7 + 3] = weight_ * tw;
    }
    return true;
  }

  double target_[4];
  double weight_;
};

// ════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: ceres_ba_optimizer <input.json> <output.json>" << std::endl;
    return 1;
  }

  // — 读取输入 JSON —
  std::ifstream ifs(argv[1]);
  if (!ifs) { std::cerr << "Cannot open: " << argv[1] << std::endl; return 1; }
  json input = json::parse(ifs);
  ifs.close();

  // — 解析相机 —
  auto jcameras = input.at("cameras");
  int n_cameras = static_cast<int>(jcameras.size());

  // — 解析目标位姿 —
  auto jtargets = input.at("targets");
  int n_targets = static_cast<int>(jtargets.size());

  int n_poses = n_cameras + n_targets;  // cameras 在前, targets 在后
  std::vector<double> params(n_poses * 7, 0.0);

  // 填充相机初值
  for (int i = 0; i < n_cameras; ++i) {
    auto& p = jcameras[i].at("initial_pose");
    for (int k = 0; k < 7; ++k) params[i * 7 + k] = p[k].get<double>();
  }
  // 填充目标初值
  for (int j = 0; j < n_targets; ++j) {
    auto& p = jtargets[j].at("initial_pose");
    int idx = n_cameras + j;
    for (int k = 0; k < 7; ++k) params[idx * 7 + k] = p[k].get<double>();
  }

  // — 构建 Ceres Problem —
  ceres::Problem problem;

  // 先添加所有参数块
  for (int i = 0; i < n_poses; ++i) {
    problem.AddParameterBlock(params.data() + i * 7, 7);
    problem.SetParameterization(params.data() + i * 7,
                                new SE3Parameterization());
  }

  // 固定第一台相机 (规范固定, 消除 gauge 自由度)
  bool fix_first = input.value("fix_first_camera",
                               input["options"].value("fix_first_camera", true));
  if (fix_first && n_cameras > 0) {
    problem.SetParameterBlockConstant(params.data());
  }

  // V4: 可选固定所有相机 (Stage 1: camera-fixed, targets-only)
  bool fix_all_cameras = input["options"].value("fix_all_cameras", false);
  if (fix_all_cameras) {
    for (int i = 0; i < n_cameras; ++i) {
      problem.SetParameterBlockConstant(params.data() + i * 7);
    }
  }

  // V6: 可选固定所有目标 (诊断: camera-only BA)
  bool fix_all_targets = input["options"].value("fix_all_targets", false);
  if (fix_all_targets) {
    for (int j = 0; j < n_targets; ++j) {
      problem.SetParameterBlockConstant(params.data() + (n_cameras + j) * 7);
    }
  }

  // V4/V6: Camera soft prior
  // 旧 API: camera_prior_weight (translation only, σt = 1/√weight)
  // 新 API: camera_prior_sigma_translation_m + camera_prior_sigma_rotation_rad
  double sigma_t_m = input["options"].value("camera_prior_sigma_translation_m", -1.0);
  double sigma_r_rad = input["options"].value("camera_prior_sigma_rotation_rad", -1.0);
  double cam_prior_w = input["options"].value("camera_prior_weight", 0.0);

  // 兼容旧 API: camera_prior_weight → σt = 1/√w
  if (sigma_t_m < 0 && cam_prior_w > 0) {
    sigma_t_m = 1.0 / std::sqrt(cam_prior_w);
  }

  // 应用 camera prior (translation + rotation)
  if (sigma_t_m > 0 || sigma_r_rad > 0) {
    for (int i = 0; i < n_cameras; ++i) {
      if (fix_first && i == 0) continue;
      auto& init = jcameras[i].at("initial_pose");

      // Translation prior: σt → weight = 1/σt
      if (sigma_t_m > 0) {
        double w_t = 1.0 / sigma_t_m;
        for (int d = 0; d < 3; ++d) {
          double t0 = init[4 + d].get<double>();
          auto* cost = new ceres::AutoDiffCostFunction<ScalarPrior, 1, 7>(
              new ScalarPrior(t0, 4 + d, w_t));
          problem.AddResidualBlock(cost, nullptr, params.data() + i * 7);
        }
      }

      // Rotation prior: σR (rad) → weight = 2/σR (so residual ≈ angle/σR)
      if (sigma_r_rad > 0) {
        double w_r = 2.0 / sigma_r_rad;
        double q0[4] = {init[0].get<double>(), init[1].get<double>(),
                        init[2].get<double>(), init[3].get<double>()};
        auto* cost = new QuaternionPrior(q0, w_r);
        problem.AddResidualBlock(cost, nullptr, params.data() + i * 7);
      }
    }
  }

  // Huber loss 门限: 默认 2.0px (实机), 仿真/噪声数据可用 ~20px
  double huber_threshold = input["options"].value("huber_threshold_px", 2.0);

  // 添加观测 (V7: per-observation weight)
  auto jobs = input.at("observations");
  int n_residuals = 0;
  for (auto& obs : jobs) {
    int cam_idx = obs.at("camera_idx").get<int>();
    int tgt_idx = obs.at("target_idx").get<int>() + n_cameras;  // offset
    auto& obj = obs.at("obj_pts");
    auto& img = obs.at("img_pts");
    double fx = obs.at("fx").get<double>();
    double fy = obs.at("fy").get<double>();
    double cx = obs.at("cx").get<double>();
    double cy = obs.at("cy").get<double>();
    double obs_weight = obs.value("weight", 1.0);

    // V8: per-observation distortion params (Brown-Conrady)
    auto jdist = obs.value("distortion", json::array({0.0, 0.0, 0.0, 0.0, 0.0}));
    double dk1 = jdist.size() > 0 ? jdist[0].get<double>() : 0.0;
    double dk2 = jdist.size() > 1 ? jdist[1].get<double>() : 0.0;
    double dp1 = jdist.size() > 2 ? jdist[2].get<double>() : 0.0;
    double dp2 = jdist.size() > 3 ? jdist[3].get<double>() : 0.0;
    double dk3 = jdist.size() > 4 ? jdist[4].get<double>() : 0.0;

    int n_pts = static_cast<int>(obj.size()) / 3;
    for (int k = 0; k < n_pts; ++k) {
      auto* cost = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 7>(
          new ReprojectionError(fx, fy, cx, cy,
                                img[2*k].get<double>(),
                                img[2*k+1].get<double>(),
                                obj[3*k].get<double>(),
                                obj[3*k+1].get<double>(),
                                obj[3*k+2].get<double>(),
                                dk1, dk2, dp1, dp2, dk3,
                                obs_weight));
      problem.AddResidualBlock(cost, new ceres::HuberLoss(huber_threshold),
                               params.data() + cam_idx * 7,
                               params.data() + tgt_idx * 7);
      ++n_residuals;
    }
  }

  std::cerr << "BA: " << n_cameras << " cameras, " << n_targets << " targets, "
            << n_residuals << " residuals" << std::endl;

  // — 求解 —
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  options.max_num_iterations = input["options"].value("max_iterations", 500);
  options.function_tolerance = 1e-6;
  options.num_threads = 4;

  auto t0 = std::chrono::steady_clock::now();
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  auto t1 = std::chrono::steady_clock::now();

  // — 逐残差评估 (per-camera RMSE, max residual, n_groups) —
  std::map<int, std::vector<double>> cam_errors;  // camera_idx → per-residual errors
  std::map<int, std::set<int>> cam_targets;       // camera_idx → set of target indices
  double total_sq_error = 0.0, max_error = 0.0;
  int total_residual_pairs = 0;

  ceres::Problem::EvaluateOptions eval_opts;
  eval_opts.apply_loss_function = false;  // raw residual
  std::vector<double> residuals;
  problem.Evaluate(eval_opts, nullptr, &residuals, nullptr, nullptr);

  // 按观测顺序评估 (与添加顺序一致)
  int residual_idx = 0;
  for (auto& obs : jobs) {
    int cam_idx = obs.at("camera_idx").get<int>();
    int tgt_raw = obs.at("target_idx").get<int>();
    cam_targets[cam_idx].insert(tgt_raw);  // track which target groups this camera sees
    auto& img = obs.at("img_pts");
    int n_pts = static_cast<int>(img.size()) / 2;
    for (int k = 0; k < n_pts; ++k) {
      if (residual_idx * 2 + 1 < static_cast<int>(residuals.size())) {
        double e_u = residuals[residual_idx * 2];
        double e_v = residuals[residual_idx * 2 + 1];
        double e_px = std::sqrt(e_u * e_u + e_v * e_v);
        cam_errors[cam_idx].push_back(e_px);
        total_sq_error += e_px * e_px;
        max_error = std::max(max_error, e_px);
        ++total_residual_pairs;
      }
      ++residual_idx;
    }
  }

  double overall_rmse = total_residual_pairs > 0
      ? std::sqrt(total_sq_error / total_residual_pairs) : 0.0;

  // — 质量分层评估 —
  double max_per_cam_rmse = 0.0;
  int min_cam_obs = 999999;
  int min_cam_groups = 999999;
  for (auto& kv : cam_errors) {
    auto& errors = kv.second;
    if (errors.empty()) continue;
    double sum_sq = 0.0;
    for (double e : errors) sum_sq += e * e;
    max_per_cam_rmse = std::max(max_per_cam_rmse,
                                std::sqrt(sum_sq / errors.size()));
    min_cam_obs = std::min(min_cam_obs, static_cast<int>(errors.size()));
    int n_groups = static_cast<int>(cam_targets[kv.first].size());
    min_cam_groups = std::min(min_cam_groups, n_groups);
  }

  bool optimizer_usable = summary.IsSolutionUsable();
  // 实机验收门限: overall_rmse <= 1.0px, per-camera <= 1.5px, max_residual <= 5px
  // 每台相机: >= 40 角点观测, >= 5 有效 frame groups
  // 总观测: >= 20
  bool quality_pass = optimizer_usable
      && overall_rmse <= 1.0
      && max_per_cam_rmse <= 1.5
      && max_error <= 5.0
      && total_residual_pairs >= 20
      && min_cam_obs >= 40
      && min_cam_groups >= 5;

  // — 构建输出 JSON —
  json output;
  // 优化器收敛即视为成功; 质量门限由 quality_status 独立报告
  // 仿真数据/噪声数据可能不满足实机验收门限, 但不应因此拒绝已收敛的结果
  bool accept_degraded = input["options"].value("accept_degraded_quality", false);
  output["success"] = optimizer_usable && (quality_pass || accept_degraded);
  output["optimizer_usable"] = optimizer_usable;
  output["quality_status"] = quality_pass ? "PASS" : (optimizer_usable ? "DEGRADED" : "FAIL");
  output["initial_cost"] = summary.initial_cost;
  output["final_cost"] = summary.final_cost;
  output["iterations"] = static_cast<int>(
      summary.iterations.size() > 0
          ? summary.iterations.back().iteration : 0);
  output["time_ms"] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  output["message"] = summary.message;
  output["overall_rmse_px"] = overall_rmse;
  output["max_residual_px"] = max_error;
  output["max_per_camera_rmse_px"] = max_per_cam_rmse;
  output["n_observations"] = total_residual_pairs;
  output["quality_thresholds"] = {
    {"overall_rmse_px_max", 1.0},
    {"per_camera_rmse_px_max", 1.5},
    {"max_residual_px_max", 5.0},
    {"min_total_observations", 20},
    {"min_observations_per_camera", 40},
    {"min_groups_per_camera", 5},
  };

  // per-camera RMSE
  json per_cam_rmse = json::object();
  for (auto& kv : cam_errors) {
    auto& errors = kv.second;
    if (errors.empty()) continue;
    double sum_sq = 0.0;
    for (double e : errors) sum_sq += e * e;
    double rmse = std::sqrt(sum_sq / errors.size());
    int n_groups = static_cast<int>(cam_targets[kv.first].size());
    // 找到相机名
    std::string cam_name = "cam_" + std::to_string(kv.first);
    if (kv.first < n_cameras) {
      cam_name = jcameras[kv.first]["name"];
    }
    per_cam_rmse[cam_name] = {
      {"rmse_px", rmse},
      {"n_residuals", errors.size()},
      {"n_groups", n_groups},
      {"max_error_px", *std::max_element(errors.begin(), errors.end())}
    };
  }
  output["per_camera_rmse"] = per_cam_rmse;

  // 相机结果
  json out_cameras = json::array();
  for (int i = 0; i < n_cameras; ++i) {
    json cam;
    cam["name"] = jcameras[i]["name"];
    json pose = json::array();
    for (int k = 0; k < 7; ++k) pose.push_back(params[i * 7 + k]);
    cam["optimized_pose"] = pose;
    out_cameras.push_back(cam);
  }
  output["cameras"] = out_cameras;

  // 目标结果
  json out_targets = json::array();
  for (int j = 0; j < n_targets; ++j) {
    json tgt;
    tgt["group_id"] = jtargets[j]["group_id"];
    json pose = json::array();
    int idx = n_cameras + j;
    for (int k = 0; k < 7; ++k) pose.push_back(params[idx * 7 + k]);
    tgt["optimized_pose"] = pose;
    out_targets.push_back(tgt);
  }
  output["targets"] = out_targets;

  // 写入输出
  std::ofstream ofs(argv[2]);
  if (!ofs) { std::cerr << "Cannot write: " << argv[2] << std::endl; return 1; }
  // 紧凑输出 (Python 可读)
  ofs << output.dump(2) << std::endl;
  ofs.close();

  std::cout << "BA done: cost " << summary.initial_cost
            << " → " << summary.final_cost
            << " (" << output["iterations"].get<int>() << " iters, "
            << output["time_ms"].get<double>() << "ms)" << std::endl;

  return output["success"].get<bool>() ? 0 : 1;
}

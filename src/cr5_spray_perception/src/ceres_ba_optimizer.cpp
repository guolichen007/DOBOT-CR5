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
    std::fill(jacobian, jacobian + 7 * 6, 0.0);
    jacobian[0 * 6 + 0] = -0.5 * x[1]; jacobian[0 * 6 + 1] = -0.5 * x[2];
    jacobian[0 * 6 + 2] = -0.5 * x[3];
    jacobian[1 * 6 + 0] =  0.5 * x[0]; jacobian[1 * 6 + 1] = -0.5 * x[3];
    jacobian[1 * 6 + 2] =  0.5 * x[2];
    jacobian[2 * 6 + 0] =  0.5 * x[3]; jacobian[2 * 6 + 1] =  0.5 * x[0];
    jacobian[2 * 6 + 2] = -0.5 * x[1];
    jacobian[3 * 6 + 0] = -0.5 * x[2]; jacobian[3 * 6 + 1] =  0.5 * x[1];
    jacobian[3 * 6 + 2] =  0.5 * x[0];
    jacobian[4 * 6 + 3] = 1.0; jacobian[5 * 6 + 4] = 1.0;
    jacobian[6 * 6 + 5] = 1.0;
    return true;
  }

  int GlobalSize() const override { return 7; }
  int LocalSize() const override { return 6; }
};

// ════════════════════════════════════════════════════════════
// 重投影误差 (自动微分)
// ════════════════════════════════════════════════════════════
struct ReprojectionError {
  ReprojectionError(double fx, double fy, double cx, double cy,
                    double ox, double oy, double px, double py, double pz)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        ox_(ox), oy_(oy), px_(px), py_(py), pz_(pz) {}

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
      residual[0] = T(1e6);
      residual[1] = T(1e6);
      return true;
    }

    T u_pred = T(fx_) * pc[0] / pc[2] + T(cx_);
    T v_pred = T(fy_) * pc[1] / pc[2] + T(cy_);

    residual[0] = T(ox_) - u_pred;
    residual[1] = T(oy_) - v_pred;
    return true;
  }

private:
  double fx_, fy_, cx_, cy_, ox_, oy_, px_, py_, pz_;
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

  // 添加观测
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

    int n_pts = static_cast<int>(obj.size()) / 3;
    for (int k = 0; k < n_pts; ++k) {
      auto* cost = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 7>(
          new ReprojectionError(fx, fy, cx, cy,
                                img[2*k].get<double>(),
                                img[2*k+1].get<double>(),
                                obj[3*k].get<double>(),
                                obj[3*k+1].get<double>(),
                                obj[3*k+2].get<double>()));
      problem.AddResidualBlock(cost, new ceres::HuberLoss(2.0),
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

  // — 逐残差评估 (per-camera RMSE, max residual) —
  std::map<int, std::vector<double>> cam_errors;  // camera_idx → per-residual errors
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
  for (auto& kv : cam_errors) {
    auto& errors = kv.second;
    if (errors.empty()) continue;
    double sum_sq = 0.0;
    for (double e : errors) sum_sq += e * e;
    max_per_cam_rmse = std::max(max_per_cam_rmse,
                                std::sqrt(sum_sq / errors.size()));
  }

  bool optimizer_usable = summary.IsSolutionUsable();
  // 仿真实门限: overall_rmse <= 1.0px, per-camera <= 1.5px, max_residual <= 5px
  // 实机容忍: overall_rmse <= 2.0px, per-camera <= 2.5px
  // 当前使用仿真门限
  bool quality_pass = optimizer_usable
      && overall_rmse <= 1.0
      && max_per_cam_rmse <= 1.5
      && max_error <= 5.0
      && total_residual_pairs >= 20;

  // — 构建输出 JSON —
  json output;
  output["success"] = optimizer_usable && quality_pass;
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
    {"min_observations", 20},
  };

  // per-camera RMSE
  json per_cam_rmse = json::object();
  for (auto& kv : cam_errors) {
    auto& errors = kv.second;
    if (errors.empty()) continue;
    double sum_sq = 0.0;
    for (double e : errors) sum_sq += e * e;
    double rmse = std::sqrt(sum_sq / errors.size());
    // 找到相机名
    std::string cam_name = "cam_" + std::to_string(kv.first);
    if (kv.first < n_cameras) {
      cam_name = jcameras[kv.first]["name"];
    }
    per_cam_rmse[cam_name] = {
      {"rmse_px", rmse},
      {"n_residuals", errors.size()},
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

#include "registration_common.hpp"
// Native, offline mixed pose / matching-cost graph. No ROS or GT input.
#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/config.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <tbb/global_control.h>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <sys/resource.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

template <int N> Eigen::Matrix<double, N, N> matrix(const Json& value) {
  if (value.size() != N) throw std::invalid_argument("matrix row count");
  Eigen::Matrix<double, N, N> result;
  for (int r = 0; r < N; ++r) {
    if (value.at(r).size() != N) throw std::invalid_argument("matrix column count");
    for (int c = 0; c < N; ++c) result(r, c) = value.at(r).at(c).get<double>();
  }
  if (!result.allFinite()) throw std::invalid_argument("nonfinite matrix");
  return result;
}

gtsam::Pose3 pose(const Json& value) {
  const auto m = matrix<4>(value);
  const Eigen::Matrix3d r = m.topLeftCorner<3, 3>();
  if (!m.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8) ||
      !(r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
      std::abs(r.determinant() - 1) > 1e-5) throw std::invalid_argument("invalid SE3");
  return gtsam::Pose3(m);
}

Matrix6 information(const Json& value) {
  Matrix6 m = matrix<6>(value);
  if (!m.isApprox(m.transpose(), 1e-8) || m.llt().info() != Eigen::Success)
    throw std::invalid_argument("information must be SPD");
  return m;
}

Json poses(const gtsam::Values& values) {
  Json output = Json::array();
  for (auto key : values.keys()) {
    Json m = Json::array();
    const auto t = values.at<gtsam::Pose3>(key).matrix();
    for (int r = 0; r < 4; ++r) m.push_back({t(r, 0), t(r, 1), t(r, 2), t(r, 3)});
    output.push_back({{"key", key}, {"T_world_body", m}});
  }
  return output;
}

using namespace s3e_registration;

Json run(const Json& input) {
  const auto start = Clock::now();
  if (input.at("schema_version") != 1) throw std::invalid_argument("unsupported input schema");
  const auto cfg = input.at("settings");
  const int threads = cfg.at("num_threads"), iterations = cfg.at("max_iterations"), neighbors = cfg.at("covariance_neighbors");
  const double distance = cfg.at("correspondence_m"), ratio = cfg.at("max_information_ratio");
  if (threads < 1 || threads > 64 || iterations < 1 || neighbors < 3 ||
      !std::isfinite(distance) || distance <= 0 || !std::isfinite(ratio) || ratio <= 0)
    throw std::invalid_argument("invalid solver settings");
  tbb::global_control limit(tbb::global_control::max_allowed_parallelism, threads);
  gtsam::Values initial;
  for (const auto& node : input.at("nodes")) initial.insert(node.at("key").get<gtsam::Key>(), pose(node.at("T_world_body")));
  gtsam::NonlinearFactorGraph graph;
  for (const auto& f : input.at("pose_factors")) {
    const auto i = f.at("i").get<gtsam::Key>();
    if (!initial.exists(i)) throw std::invalid_argument("unknown pose endpoint");
    const auto noise = gtsam::noiseModel::Gaussian::Information(information(f.at("information")));
    if (f.at("kind") == "anchor") graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(i, pose(f.at("measurement")), noise);
    else {
      const auto j = f.at("j").get<gtsam::Key>();
      if (!initial.exists(j) || i == j) throw std::invalid_argument("invalid pose endpoints");
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(i, j, pose(f.at("measurement")), noise);
    }
  }
  const auto num_pose_factors = graph.size();
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = iterations; params.relativeErrorTol = 1e-6; params.absoluteErrorTol = 1e-6;
  gtsam::LevenbergMarquardtOptimizer pose_solver(graph, initial, params);
  const auto baseline = pose_solver.optimize();
  const double baseline_error = graph.error(baseline);
  const auto prepared = Clock::now();
  std::map<gtsam::Key, Cloud> clouds;
  for (const auto& spec : input.at("clouds")) {
    auto key = spec.at("key").get<gtsam::Key>();
    if (!initial.exists(key) || clouds.count(key)) throw std::invalid_argument("invalid cloud key");
    clouds.emplace(key, load_cloud(spec, neighbors, threads));
  }
  Json diagnostics = Json::array();
  std::vector<std::shared_ptr<ScaledGICP>> registrations;
  std::vector<size_t> accepted_indices;
  std::set<std::pair<gtsam::Key, gtsam::Key>> seen;
  for (const auto& pair : input.at("registrations")) {
    auto i = pair.at("i").get<gtsam::Key>(), j = pair.at("j").get<gtsam::Key>();
    if (i == j || !seen.emplace(std::min(i, j), std::max(i, j)).second)
      throw std::invalid_argument("duplicate or self registration pair");
    const auto& target = clouds.at(i); const auto& source = clouds.at(j);
    const auto relative = baseline.at<gtsam::Pose3>(i).between(baseline.at<gtsam::Pose3>(j));
    Json diag = pair; diag["initial_quality"] = quality(target, source, relative, distance);
    diag["reason"] = rejection(diag["initial_quality"], cfg); diag["accepted"] = diag["reason"] == "accepted";
    if (diag["accepted"].get<bool>()) {
      auto factor = std::make_shared<ScaledGICP>(i, j, target.points, source.points, target.tree);
      // TBB parallelizes different factors; avoid nested OpenMP teams here.
      factor->set_num_threads(1); factor->set_max_correspondence_distance(distance);
      factor->set_correspondence_update_tolerance(0, 0);
      auto raw = factor->linearize(baseline);
      const auto a = dynamic_cast<gtsam::HessianFactor*>(raw.get())->augmentedInformation();
      const Matrix6 h = a.block<6, 6>(6, 6);
      Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6> eigen(h, information(pair.at("information")));
      if (eigen.info() != Eigen::Success || eigen.eigenvalues().maxCoeff() <= 0)
        throw std::runtime_error("invalid registration information");
      // Conservative initial curvature cap relative to the verified pose factor.
      // Freeze this scalar; changing it per iteration would change the objective.
      factor->scale = ratio / eigen.eigenvalues().maxCoeff();
      diag["information_scale"] = factor->scale;
      diag["initial_max_information_ratio"] = ratio;
      diag["initial_error"] = factor->error(baseline);
      graph.add(factor); registrations.push_back(factor); accepted_indices.push_back(diagnostics.size());
    }
    diagnostics.push_back(diag);
  }
  const double preparation_s = std::chrono::duration<double>(Clock::now() - prepared).count();
  const auto solve_start = Clock::now();
  const double initial_error = graph.error(baseline);
  gtsam::LevenbergMarquardtOptimizer solver(graph, baseline, params);
  const auto result = solver.optimize();
  const double optimization_s = std::chrono::duration<double>(Clock::now() - solve_start).count();
  double pose_error = 0;
  for (size_t k = 0; k < num_pose_factors; ++k) pose_error += graph[k]->error(result);
  bool safe = true;
  for (size_t k = 0; k < registrations.size(); ++k) {
    auto& diag = diagnostics[accepted_indices[k]]; auto& factor = registrations[k];
    const auto i = factor->keys()[0], j = factor->keys()[1];
    diag["final_error_fixed_last_correspondences"] = factor->error(result);
    diag["linearizations"] = factor->linearizations;
    diag["final_quality"] = quality(clouds.at(i), clouds.at(j), result.at<gtsam::Pose3>(i).between(result.at<gtsam::Pose3>(j)), distance);
    diag["final_quality_reason"] = rejection(diag["final_quality"], cfg);
    safe &= diag["final_quality_reason"] == "accepted";
  }
  // A loss of overlap can make the trimmed cost deceptively small. Do not
  // publish such a result as successful; preserve the diagnostics for inspection.
  struct rusage usage{}; getrusage(RUSAGE_SELF, &usage);
  return {{"schema_version", 1}, {"success", safe}, {"poses", poses(result)}, {"baseline_poses", poses(baseline)},
          {"registrations", diagnostics}, {"pose_factor_count", num_pose_factors}, {"registration_factor_count", registrations.size()},
          {"baseline_pose_error", baseline_error}, {"initial_mixed_error", initial_error},
          {"final_mixed_error", graph.error(result)}, {"final_pose_error", pose_error},
          {"iterations", solver.iterations()}, {"preparation_s", preparation_s}, {"optimization_s", optimization_s},
          {"wall_s", std::chrono::duration<double>(Clock::now() - start).count()}, {"peak_rss_kib", usage.ru_maxrss},
          {"gtsam_version", GTSAM_VERSION_STRING}, {"gtsam_points_version", GTSAM_POINTS_VERSION_STRING},
          {"gtsam_points_commit", GTSAM_POINTS_GIT_HASH}, {"device", "CPU"}};
}

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--version") {
      std::cout << Json({{"gtsam", GTSAM_VERSION_STRING}, {"gtsam_points", GTSAM_POINTS_VERSION_STRING},
                         {"gtsam_points_commit", GTSAM_POINTS_GIT_HASH}, {"device", "CPU"}}).dump() << '\n'; return 0;
    }
    if (argc != 3) throw std::invalid_argument("usage: s3e_mixed_pgo INPUT.json OUTPUT.json");
    std::ifstream f(argv[1]); Json input; f >> input;
    Json result = run(input); std::ofstream out(argv[2]); out << result.dump() << '\n';
    if (!out) throw std::runtime_error("cannot write result");
    return result.at("success").get<bool>() ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << "mixed PGO: " << e.what() << '\n'; return 1;
  }
}

#pragma once
// Shared live registration factor and geometry checks for batch PGO and CBS.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <fstream>
namespace s3e_registration {
using Json = nlohmann::json;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
// Upstream aggregates all point residuals in one binary nonlinear factor.
// Scale BOTH its objective and complete Gaussian linearization; do not turn
// it into a fixed BetweenFactor or wrap only its reported error.
class ScaledGICP : public gtsam_points::IntegratedGICPFactor {
 public:
  using gtsam_points::IntegratedGICPFactor::IntegratedGICPFactor;
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ScaledGICP(*this));
  }
  double scale = 1;
  mutable size_t linearizations = 0;
  double error(const gtsam::Values& v) const override {
    return scale * gtsam_points::IntegratedGICPFactor::error(v);
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& v) const override {
    ++linearizations;
    auto raw = gtsam_points::IntegratedGICPFactor::linearize(v);
    auto* h = dynamic_cast<gtsam::HessianFactor*>(raw.get());
    if (!h) throw std::runtime_error("expected GICP HessianFactor");
    const Eigen::MatrixXd a = scale * h->augmentedInformation();
    return gtsam::GaussianFactor::shared_ptr(new gtsam::HessianFactor(
        keys()[0], keys()[1], a.block<6, 6>(0, 0), a.block<6, 6>(0, 6),
        a.block<6, 1>(0, 12), a.block<6, 6>(6, 6), a.block<6, 1>(6, 12), a(12, 12)));
  }
};

struct Cloud {
  gtsam_points::PointCloudCPU::Ptr points;
  std::shared_ptr<gtsam_points::KdTree> tree;
  int extent_rank;
};

inline Cloud load_cloud(const Json& spec, int neighbors, int threads) {
  const auto count = spec.at("points").get<size_t>();
  if (count < static_cast<size_t>(neighbors) || count > 1000000)
    throw std::invalid_argument("invalid cloud point count");
  std::ifstream f(spec.at("path").get<std::string>(), std::ios::binary | std::ios::ate);
  if (!f || f.tellg() != static_cast<std::streamoff>(count * 3 * sizeof(double)))
    throw std::invalid_argument("cloud binary size mismatch");
  f.seekg(0);
  std::vector<Eigen::Vector4d> xyz(count);
  for (auto& p : xyz) {
    f.read(reinterpret_cast<char*>(p.data()), 3 * sizeof(double)); p[3] = 1;
    if (!f || !p.allFinite()) throw std::invalid_argument("invalid cloud data");
  }
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(xyz);
  cloud->add_covs(gtsam_points::estimate_covariances(*cloud, neighbors, threads));
  cloud->add_normals(gtsam_points::estimate_normals(cloud->points, cloud->covs, count, threads));
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& p : xyz) mean += p.head<3>();
  mean /= count;
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const auto& p : xyz) {
    const Eigen::Vector3d d = p.head<3>() - mean; scatter.noalias() += d * d.transpose();
  }
  const Eigen::Vector3d eig = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(scatter / count).eigenvalues();
  const int rank = (eig.array() > std::max(1e-12, eig.maxCoeff()*1e-8)).count();
  return {cloud, std::make_shared<gtsam_points::KdTree>(cloud->points, cloud->size()), rank};
}

inline Json quality(const Cloud& target, const Cloud& source, const gtsam::Pose3& t,
             double distance) {
  const auto rotation = t.rotation().matrix();
  const Eigen::Isometry3d delta(t.matrix());
  Matrix6 h = Matrix6::Zero();
  size_t inliers = 0, reverse = 0; double squared = 0;
  for (size_t k = 0; k < source.points->size(); ++k) {
    const auto& p = source.points->points[k]; Eigen::Vector4d q = delta * p;
    size_t idx = 0; double d = 0;
    if (target.tree->knn_search(q.data(), 1, &idx, &d) && d < distance * distance) {
      Eigen::Vector3d n = rotation.transpose() * target.points->normals[idx].head<3>();
      Eigen::Matrix<double, 6, 1> j;
      j << p.head<3>().cross(n), n;
      h.noalias() += j * j.transpose(); ++inliers; squared += d;
    }
  }
  for (size_t k = 0; k < target.points->size(); ++k) {
    Eigen::Vector4d q = delta.inverse() * target.points->points[k];
    size_t idx = 0; double d = 0;
    if (source.tree->knn_search(q.data(), 1, &idx, &d) && d < distance * distance) ++reverse;
  }
  const Eigen::Matrix<double, 6, 1> eig = Eigen::SelfAdjointEigenSolver<Matrix6>(h / std::max<size_t>(inliers, 1)).eigenvalues();
  return {{"inliers", inliers}, {"overlap", std::min(double(inliers) / source.points->size(), double(reverse) / target.points->size())},
          {"rmse_m", inliers ? Json(std::sqrt(squared / inliers)) : Json(nullptr)},
          {"observability", eig[0]}, {"condition", eig[5] / std::max(eig[0], 1e-15)},
          {"target_extent_rank", target.extent_rank}, {"source_extent_rank", source.extent_rank}};
}

inline std::string rejection(const Json& q, const Json& cfg) {
  if (q.at("inliers").get<int>() < cfg.at("min_inliers").get<int>()) return "insufficient_inliers";
  if (q.at("overlap").get<double>() < cfg.at("min_overlap").get<double>()) return "low_overlap";
  if (q.at("target_extent_rank").get<int>() < 3 || q.at("source_extent_rank").get<int>() < 3 ||
      q.at("observability").get<double>() < cfg.at("min_observability").get<double>() ||
      q.at("condition").get<double>() > cfg.at("max_condition").get<double>()) return "unobservable";
  return "accepted";
}


}  // namespace s3e_registration

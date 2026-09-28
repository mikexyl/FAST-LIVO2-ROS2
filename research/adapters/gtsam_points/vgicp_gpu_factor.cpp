#include "vgicp_gpu_factor.hpp"
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <cuda_runtime_api.h>
#include <algorithm>
#include <mutex>

namespace s3e_registration {
struct GpuGeometry {
  gtsam_points::PointCloudGPU::Ptr points;
  std::vector<gtsam_points::GaussianVoxelMapGPU::Ptr> levels;
};

std::shared_ptr<GpuGeometry> upload_vgicp_geometry(const Cloud& cloud, const Json& cfg) {
  int devices=0;
  if (cudaGetDeviceCount(&devices)!=cudaSuccess || devices<1)
    throw std::runtime_error("vgicp_gpu requires a working CUDA device; CPU fallback is disabled");
  auto result=std::make_shared<GpuGeometry>();
  result->points=gtsam_points::PointCloudGPU::clone(*cloud.points);
  // GLIM uses a bounded median-distance sample to choose each submap's voxel size.
  std::vector<double> distances;
  const size_t n=cloud.points->size(), stride=std::max<size_t>(1,n/256);
  for(size_t k=0;k<n;k+=stride) distances.push_back(cloud.points->points[k].head<3>().norm());
  const size_t mid=distances.size()/2;
  std::nth_element(distances.begin(),distances.begin()+mid,distances.end());
  const double lo=cfg.at("gpu_voxel_m"),hi=cfg.at("gpu_voxel_max_m");
  const double dmin=cfg.at("gpu_distance_min_m"),dmax=cfg.at("gpu_distance_max_m");
  const double p=std::clamp((distances[mid]-dmin)/(dmax-dmin),0.,1.);
  const double base=lo+p*(hi-lo), scaling=cfg.at("gpu_voxel_scaling");
  for(int level=0;level<cfg.at("gpu_voxel_levels").get<int>();++level) {
    auto map=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(base*std::pow(scaling,level));
    map->insert(*result->points);result->levels.push_back(map);
  }
  if(cudaDeviceSynchronize()!=cudaSuccess) throw std::runtime_error("CUDA geometry upload failed");
  return result;
}

Json gpu_geometry_diagnostics(const GpuGeometry& geometry) {
  Json resolutions=Json::array();size_t bytes=geometry.points->memory_usage_gpu();
  for(const auto& level:geometry.levels) {resolutions.push_back(level->voxel_resolution());bytes+=level->memory_usage_gpu();}
  return {{"points",geometry.points->size()},{"voxel_resolutions_m",resolutions},{"geometry_gpu_bytes",bytes}};
}

struct ScaledVGICPGPU::Impl {
  std::shared_ptr<GpuGeometry> target,source;
  gtsam_points::StreamTempBufferRoundRobin streams;
  std::vector<std::shared_ptr<gtsam_points::IntegratedVGICPFactorGPU>> native;
  gtsam_points::NonlinearFactorSetGPU batch;
  mutable std::mutex mutex;
  bool valid=false;
  Eigen::Matrix4d at=Eigen::Matrix4d::Identity();
  Eigen::Matrix<double,13,13> h;
  double at_error=0.;
  size_t requests=0,passes=0,hits=0,evaluations=0;
  Impl(std::shared_ptr<GpuGeometry> t,std::shared_ptr<GpuGeometry> s)
      :target(std::move(t)),source(std::move(s)),streams(target->levels.size()) {
    for(const auto& level:target->levels) {
      const auto sb=streams.get_stream_buffer();
      auto f=std::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(0,1,level,source->points,sb.first,sb.second);
      native.push_back(f);batch.add(f);
    }
  }
  // All native factors use internal keys 0/1. Wrapper rekeying therefore cannot
  // leave stale native keys; immutable endpoint geometry remains shared.
  static gtsam::Values local(const gtsam::Pose3& delta) {
    gtsam::Values v;v.insert(0,gtsam::Pose3());v.insert(1,delta);return v;
  }
  void linearize(const gtsam::Pose3& delta) {
    ++requests;
    if(valid && (at.array()==delta.matrix().array()).all()) {++hits;return;}
    const auto v=local(delta);
    batch.linearize(v);h.setZero();at_error=0.;
    for(const auto& f:native) {
      auto gaussian=f->linearize(v);
      h+=dynamic_cast<gtsam::HessianFactor&>(*gaussian).augmentedInformation();
      // Upstream GPU reports the sum of squared Mahalanobis residuals;
      // GTSAM HessianFactor uses one half of that quadratic objective.
      at_error+=.5*f->error(v);
    }
    if(!h.allFinite() || !std::isfinite(at_error)) throw std::runtime_error("Non-finite CUDA VGICP result");
    at=delta.matrix();valid=true;++passes;
  }
};

ScaledVGICPGPU::ScaledVGICPGPU(gtsam::Key a,gtsam::Key b,std::shared_ptr<GpuGeometry> target,
    std::shared_ptr<GpuGeometry> source):gtsam::NonlinearFactor(gtsam::KeyVector{a,b}),
    impl_(std::make_shared<Impl>(std::move(target),std::move(source))) {}
ScaledVGICPGPU::~ScaledVGICPGPU()=default;
gtsam::NonlinearFactor::shared_ptr ScaledVGICPGPU::clone() const {
  auto copy=std::make_shared<ScaledVGICPGPU>(keys()[0],keys()[1],impl_->target,impl_->source);
  copy->scale=scale;return copy;
}
gtsam::GaussianFactor::shared_ptr ScaledVGICPGPU::linearize(const gtsam::Values& v) const {
  const auto delta=v.at<gtsam::Pose3>(keys()[0]).between(v.at<gtsam::Pose3>(keys()[1]));
  std::lock_guard<std::mutex> lock(impl_->mutex);impl_->linearize(delta);
  const Eigen::Matrix<double,13,13> h=scale*impl_->h;
  return std::make_shared<gtsam::HessianFactor>(keys()[0],keys()[1],h.block<6,6>(0,0),h.block<6,6>(0,6),
      h.block<6,1>(0,12),h.block<6,6>(6,6),h.block<6,1>(6,12),h(12,12));
}
double ScaledVGICPGPU::error(const gtsam::Values& v) const {
  const auto delta=v.at<gtsam::Pose3>(keys()[0]).between(v.at<gtsam::Pose3>(keys()[1]));
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if(!impl_->valid) impl_->linearize(delta);
  if((impl_->at.array()==delta.matrix().array()).all()) return scale*impl_->at_error;
  const auto local=Impl::local(delta);impl_->batch.error(local);double total=0.;
  for(const auto& f:impl_->native) total+=f->error(local);
  ++impl_->evaluations;
  if(!std::isfinite(total)) throw std::runtime_error("Non-finite CUDA VGICP error");
  return .5*scale*total;
}
Json ScaledVGICPGPU::diagnostics() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Json inliers=Json::array();for(const auto& f:impl_->native) inliers.push_back(f->num_inliers());
  return {{"linearizations",impl_->requests},{"gpu_linearization_batches",impl_->passes},
    {"exact_cache_hits",impl_->hits},{"gpu_error_batches",impl_->evaluations},
    {"gpu_inliers_per_level",inliers},{"gpu_levels",impl_->native.size()},
    {"backend","gtsam_points::IntegratedVGICPFactorGPU"},{"synchronous_factor_fallback",false}};
}
}  // namespace s3e_registration

#pragma once
#include "registration_common.hpp"

namespace s3e_registration {
// Immutable per-endpoint GPU geometry. Native kernels and voxel construction
// are unmodified upstream gtsam_points, also used by GLIM.
struct GpuGeometry;
std::shared_ptr<GpuGeometry> upload_vgicp_geometry(const Cloud&, const Json&);
Json gpu_geometry_diagnostics(const GpuGeometry&);

// Ordinary GTSAM interface, so CBS local solves AND nonlinear cavity graphs
// execute the same GPU objective. Two GLIM voxel levels form one scaled pair.
class ScaledVGICPGPU : public gtsam::NonlinearFactor {
 public:
  ScaledVGICPGPU(gtsam::Key, gtsam::Key, std::shared_ptr<GpuGeometry>,
                std::shared_ptr<GpuGeometry>);
  ~ScaledVGICPGPU() override;
  size_t dim() const override { return 6; }
  double scale = 1.;
  double error(const gtsam::Values&) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values&) const override;
  gtsam::NonlinearFactor::shared_ptr clone() const override;
  Json diagnostics() const;
 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace s3e_registration

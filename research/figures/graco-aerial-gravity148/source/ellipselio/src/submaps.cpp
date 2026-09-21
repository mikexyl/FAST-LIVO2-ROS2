#include "map_processing.h"

namespace ellipselio {
void MappingNode::InitMapBuffer(MapBuffer& buffer) {
  if (&buffer != map_.get()) {
    buffer.ioctree_.SetBucketSize(1);
    buffer.ioctree_.SetMaxOctants(kMaxMapPoints);
    buffer.ioctree_.SetMaxNewPoints(kMaxProcPoints);
    buffer.ioctree_.SetMinExtent(map_resolution_);
  }
  buffer.n_means_ = Eigen::ArrayXi::Zero(lid_process_->num_bins_);
  buffer.min_neighbours_.assign(lid_process_->num_bins_, kMinNeighbours);
  buffer.max_neighbours_.assign(lid_process_->num_bins_, kMaxNeighbours);
  buffer.cnt_neighbours_.assign(lid_process_->num_bins_, 1);
}

void MappingNode::ResetMapBuffer(MapBuffer& b, int64_t begin) {
  b.ioctree_.clear();
  b.map_cloud_->clear();
  b.tensors_p1_.clear(); b.tensors_p2_.clear();
  b.eigenvectors_.clear(); b.eigenvalues_.clear(); b.salivalues_.clear();
  b.neighbours_.clear(); b.filters_.clear();
  b.update_idx_.clear(); b.saliency_idxs_.clear();
  b.old_map_size_ = b.new_map_size_ = b.last_map_size_ = 0;
  b.n_means_.setZero();
  std::fill(b.min_neighbours_.begin(), b.min_neighbours_.end(), kMinNeighbours);
  std::fill(b.max_neighbours_.begin(), b.max_neighbours_.end(), kMaxNeighbours);
  std::fill(b.cnt_neighbours_.begin(), b.cnt_neighbours_.end(), 1);
  b.geometry.clear(); b.members.clear();
  b.begin_ns = begin; b.last_ns = 0; b.id = next_submap_id_++;
}

void MappingNode::AdvanceSubmaps(int64_t stamp) {
  if (!submaps_enabled_) return;
  if (map_->id < 0) ResetMapBuffer(*map_, stamp);
  // Half-open intervals: retire before matching/inserting the boundary scan.
  // Missing scans advance the schedule too; empty maps never fall back to history.
  while (stamp >= map_->begin_ns + submap_duration_ns_) {
    if (!successor_started_)
      ResetMapBuffer(*successor_, map_->begin_ns + submap_stride_ns_);
#ifdef ELLIPSELIO_RESEARCH_EXPORT
    ExportSubmap(true, stamp);
#endif
    std::swap(map_, successor_);
    successor_started_ = false;
    ++analytics_msg_.handovers;
    if (publish_markers_) {
      visualization_msgs::msg::MarkerArray clear;
      visualization_msgs::msg::Marker marker;
      marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear.markers.push_back(marker);
      pub_mark_->publish(clear);
      map_->last_map_size_ = 0;
    }
  }
  if (!successor_started_ && stamp >= map_->begin_ns + submap_stride_ns_) {
    ResetMapBuffer(*successor_, map_->begin_ns + submap_stride_ns_);
    successor_started_ = true;
  }
}
}  // namespace ellipselio

#ifdef ELLIPSELIO_RESEARCH_EXPORT
namespace ellipselio {
void MappingNode::CloseMappingOutputs() {
  if (outputs_closed_) return;
  if (submap_export_) {
    ExportSubmap(false, scan_end_time_.nanoseconds());
    if (successor_started_) {
      std::swap(map_, successor_);
      ExportSubmap(false, scan_end_time_.nanoseconds());
      std::swap(map_, successor_);
    }
    submap_export_->close();
  }
  if (diagnostics_.is_open()) {
    diagnostics_.flush();
    if (!diagnostics_) throw std::runtime_error("Per-scan diagnostics flush failed");
    diagnostics_.close();
  }
  outputs_closed_ = true;
}

void MappingNode::ExportSubmap(bool complete, int64_t available_ns) {
  if (!submap_export_ || map_->members.empty()) return;
  ResearchExport::Packet packet;
  const auto& state = map_->anchor.state;
  const Eigen::Matrix3f R = state.rot.toRotationMatrix().transpose().cast<float>();
  const Eigen::Vector3f t = state.pos.cast<float>();
  std::vector<float> geometry, ellipsoids;
  geometry.reserve(map_->geometry.size() * 3);
  for (const auto& p : map_->geometry) {
    const Eigen::Vector3f q = R * (p.getVector3fMap() - t);
    if (!q.allFinite()) throw std::runtime_error("Non-finite submap member geometry");
    geometry.insert(geometry.end(), q.data(), q.data() + 3);
  }
  for (size_t i = 0; i < map_->map_cloud_->size(); ++i) {
    if (!map_->filters_[i][1]) continue;
    const Eigen::Vector3f q = R * (map_->map_cloud_->points[i].getVector3fMap() - t);
    const auto& axes = map_->eigenvalues_[i];
    const Eigen::Matrix3f basis = R * map_->eigenvectors_[i];
    if (!q.allFinite() || !axes.allFinite() || !basis.allFinite() || axes.minCoeff() <= 0)
      throw std::runtime_error("Invalid submap ellipsoid");
    ellipsoids.insert(ellipsoids.end(), q.data(), q.data() + 3);
    ellipsoids.insert(ellipsoids.end(), axes.data(), axes.data() + 3);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ellipsoids.push_back(basis(r,c));
  }
  std::vector<double> pose(16, 0);
  const auto rotation = state.rot.toRotationMatrix();
  const Eigen::Vector3d gravity_world = state.grav.get_vect();
  const Eigen::Vector3d gravity_imu = rotation.transpose() * gravity_world;
  if (!gravity_imu.allFinite() || gravity_imu.norm() < 1e-6)
    throw std::runtime_error("Invalid submap anchor gravity");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) pose[4*r+c] = rotation(r,c);
    pose[4*r+3] = state.pos[r];
  }
  pose[15] = 1;
  packet.metadata = {{"schema_version", 1}, {"robot_id", submap_robot_},
    {"submap_id", map_->id}, {"keyframe_id", map_->id}, {"complete", complete},
    {"retrievable", complete}, {"member_scan_ids", map_->members},
    {"begin_ns", map_->begin_ns}, {"end_ns", map_->begin_ns + submap_duration_ns_},
    {"last_member_ns", map_->last_ns}, {"stamp_ns", map_->anchor.time.nanoseconds()},
    {"available_ns", available_ns}, {"T_world_imu", pose},
    {"gravity_world_m_s2", {gravity_world.x(), gravity_world.y(), gravity_world.z()}},
    {"gravity_imu_m_s2", {gravity_imu.x(), gravity_imu.y(), gravity_imu.z()}},
    {"gravity_source", "ellipselio_filter_at_anchor"},
    {"frame", "last_member_imu"},
    {"geometry_source", "native processed scans inserted into this submap; not full-resolution raw geometry"},
    {"geometry_count", geometry.size()/3}, {"ellipsoid_count", ellipsoids.size()/15}};
  for (const auto* values : {&geometry, &ellipsoids}) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(values->data());
    packet.messages.emplace_back(bytes, bytes + values->size()*sizeof(float));
  }
  submap_export_->enqueue(std::move(packet));
}
}  // namespace ellipselio
#endif

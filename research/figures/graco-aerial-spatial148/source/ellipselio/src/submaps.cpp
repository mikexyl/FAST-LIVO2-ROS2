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
  b.retire_ns = 0; b.extent_m = 0; b.finish_reason.clear();
  b.origin.setZero(); b.up = V3D::UnitZ();
}

void MappingNode::AdvanceSubmaps(int64_t stamp) {
  if (!submaps_enabled_) return;
  submap_event_.clear();
  analytics_msg_.successor_support = 0;
  analytics_msg_.successor_support_ratio = 0;
  if (map_->id < 0) ResetMapBuffer(*map_, stamp);
  if (submap_strategy_ == "spatial") {
    AdvanceSpatialSubmaps(stamp);
    return;
  }
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
    submap_event_ = "handover_temporal";
    ClearPublishedMap();
  }
  if (!successor_started_ && stamp >= map_->begin_ns + submap_stride_ns_) {
    ResetMapBuffer(*successor_, map_->begin_ns + submap_stride_ns_);
    successor_started_ = true;
  }
}

void MappingNode::ClearPublishedMap() {
  if (!publish_markers_) return;
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(marker);
  pub_mark_->publish(clear);
  map_->last_map_size_ = 0;
}

bool MappingNode::SuccessorSupportsScan() {
  if (!successor_started_ || successor_->map_cloud_->empty() || scan_cloud_->empty()) return false;
  size_t samples = 0, matches = 0;
  const size_t stride = std::max<size_t>(1, (scan_cloud_->size()+1999)/2000);
  const auto& state = kf_state_.state;
  for (size_t i=0; i<scan_cloud_->size(); i+=stride) {
    const V3F point = scan_cloud_->points[i].getVector3fMap();
    if (!point.allFinite() || point.squaredNorm() > 80*80) continue;
    const V3F world = (state.rot*(state.offset_R_L_I*point.cast<double>()+state.offset_T_L_I)+state.pos).cast<float>();
    if (!world.allFinite()) continue;
    ++samples;
    std::vector<int> indices;
    std::vector<float> distances;
    successor_->ioctree_.KnnNeighbors(world, 1, indices, distances, kMaxSearchRes);
    if (indices.empty()) continue;
    const int j = indices.front();
    if (j >= 0 && j < static_cast<int>(successor_->filters_.size()) &&
        successor_->filters_[j][1] && successor_->eigenvalues_[j].allFinite() &&
        successor_->eigenvalues_[j].minCoeff() > 0 && successor_->eigenvectors_[j].allFinite()) ++matches;
  }
  analytics_msg_.successor_support = matches;
  analytics_msg_.successor_support_ratio = samples ? static_cast<double>(matches)/samples : 0;
  return matches >= static_cast<size_t>(submap_min_support_) &&
         analytics_msg_.successor_support_ratio >= submap_support_ratio_;
}

void MappingNode::AdvanceSpatialSubmaps(int64_t stamp) {
  // Spatial decisions use corrected member poses, never an uncorrected jump.
  // Apply before the next LiDAR update: its iterated matching map stays fixed.
  const bool radius = map_->extent_m >= submap_radius_m_;
  const bool expired = stamp-map_->begin_ns >= submap_max_age_ns_;
  // Preserve the existing 10M-point snapshot transport limit without dropping
  // member scans. This is a format/capacity guard, not a normal spatial trigger.
  const bool capacity = map_->geometry.size()+scan_cloud_->size() > kMaxMapPoints;
  const bool successor_fresh = successor_started_ &&
      stamp-successor_->begin_ns < submap_max_age_ns_ &&
      successor_->geometry.size()+scan_cloud_->size() <= kMaxMapPoints;
  const std::string reason = expired ? "max_age" : capacity ? "payload_capacity" : "radius";
  if ((radius || expired || capacity) && successor_fresh && SuccessorSupportsScan()) {
    map_->retire_ns = stamp; map_->finish_reason = reason;
#ifdef ELLIPSELIO_RESEARCH_EXPORT
    ExportSubmap(true, stamp);
#endif
    std::swap(map_, successor_);
    successor_started_ = false;
    ++analytics_msg_.handovers;
    submap_event_ = "handover_"+reason;
    ClearPublishedMap();
  } else if (expired || capacity) {
    map_->retire_ns = stamp; map_->finish_reason = reason+"_recovery";
#ifdef ELLIPSELIO_RESEARCH_EXPORT
    ExportSubmap(true, stamp);
    if (successor_started_) {
      std::swap(map_, successor_);
      ExportSubmap(false, stamp);
      std::swap(map_, successor_);
    }
#endif
    ResetMapBuffer(*map_, stamp);
    if (successor_started_) ResetMapBuffer(*successor_, stamp);
    successor_started_ = false;
    submap_event_ = reason+"_recovery";
    ClearPublishedMap();
  } else if (radius) {
    submap_event_ = "handover_waiting_for_support";
  }
  if (!successor_started_ &&
      (map_->extent_m >= submap_radius_m_-submap_overlap_m_ ||
       stamp-map_->begin_ns >= submap_max_age_ns_/2 ||
       map_->geometry.size() >= kMaxMapPoints/2)) {
    ResetMapBuffer(*successor_, stamp);
    successor_started_ = true;
    if (submap_event_.empty()) submap_event_ = "successor_started";
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
  const bool spatial = submap_strategy_ == "spatial";
  packet.metadata = {{"schema_version", spatial ? 3 : 1}, {"robot_id", submap_robot_},
    {"submap_id", map_->id}, {"keyframe_id", map_->id}, {"complete", complete},
    {"retrievable", complete}, {"member_scan_ids", map_->members},
    {"begin_ns", map_->begin_ns}, {"end_ns", spatial ?
      (map_->retire_ns ? map_->retire_ns : map_->last_ns+1) : map_->begin_ns + submap_duration_ns_},
    {"last_member_ns", map_->last_ns}, {"stamp_ns", map_->anchor.time.nanoseconds()},
    {"available_ns", available_ns}, {"T_world_imu", pose},
    {"gravity_world_m_s2", {gravity_world.x(), gravity_world.y(), gravity_world.z()}},
    {"gravity_imu_m_s2", {gravity_imu.x(), gravity_imu.y(), gravity_imu.z()}},
    {"gravity_source", "ellipselio_filter_at_anchor"},
    {"frame", "last_member_imu"},
    {"geometry_source", "native processed scans inserted into this submap; not full-resolution raw geometry"},
    {"geometry_count", geometry.size()/3}, {"ellipsoid_count", ellipsoids.size()/15}};
  if (spatial) {
    packet.metadata["strategy"] = "spatial";
    packet.metadata["distance_metric"] = submap_distance_metric_;
    packet.metadata["origin_world"] = {map_->origin.x(), map_->origin.y(), map_->origin.z()};
    packet.metadata["up_world"] = {map_->up.x(), map_->up.y(), map_->up.z()};
    packet.metadata["extent_m"] = map_->extent_m;
    packet.metadata["radius_m"] = submap_radius_m_;
    packet.metadata["overlap_m"] = submap_overlap_m_;
    packet.metadata["max_age_s"] = submap_max_age_ns_*1e-9;
    packet.metadata["finish_reason"] = complete ? map_->finish_reason : "shutdown_or_recovery_tail";
  }
  for (const auto* values : {&geometry, &ellipsoids}) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(values->data());
    packet.messages.emplace_back(bytes, bytes + values->size()*sizeof(float));
  }
  submap_export_->enqueue(std::move(packet));
}
}  // namespace ellipselio
#endif

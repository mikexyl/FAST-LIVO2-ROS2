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
  b.retire_ns = 0; b.keyframes.clear(); b.key_voxels.reset();
  b.origin.setZero(); b.key_position.setZero(); b.key_rotation.setIdentity();
  b.extent_m = 0; b.key_overlap = 1; b.finish_reason.clear();
}

void MappingNode::AdvanceSubmaps(int64_t stamp) {
  if (!submaps_enabled_) return;
  submap_event_.clear();
  analytics_msg_.successor_support = 0;
  analytics_msg_.successor_overlap = 0;
  if (map_->id < 0) ResetMapBuffer(*map_, stamp);
  if (submap_strategy_ == "motion_overlap") {
    AdvanceMotionSubmaps(stamp);
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

void MappingNode::ClearPublishedMap() {
  if (!publish_markers_) return;
  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(marker);
  pub_mark_->publish(clear);
  map_->last_map_size_ = 0;
}

MappingNode::VoxelSet MappingNode::ScanVoxels() const {
  VoxelSet cells;
  const auto origin = kf_state_.state.pos.cast<float>();
  for (const auto& point : *scan_cloud_) {
    const auto p = point.getVector3fMap();
    if (!p.allFinite() || (p-origin).squaredNorm() > 80*80) continue;
    cells.insert({static_cast<int64_t>(std::floor(p.x()/submap_voxel_)),
                  static_cast<int64_t>(std::floor(p.y()/submap_voxel_)),
                  static_cast<int64_t>(std::floor(p.z()/submap_voxel_))});
  }
  return cells;
}

double MappingNode::VoxelOverlap(const VoxelSet& current, const VoxelSet& reference) {
  if (current.empty() || reference.empty()) return 0;
  size_t matches = 0;
  for (const auto& cell : current) {
    bool found = false;
    // A one-voxel neighbourhood reduces arbitrary grid-boundary sensitivity.
    for (int x=-1; x<=1 && !found; ++x)
      for (int y=-1; y<=1 && !found; ++y)
        for (int z=-1; z<=1 && !found; ++z)
          found = reference.count({cell[0]+x, cell[1]+y, cell[2]+z});
    matches += found;
  }
  return static_cast<double>(matches)/current.size();
}

void MappingNode::ObserveSubmapScan() {
  auto& b = *map_;
  const auto& state = kf_state_.state;
  const Eigen::Quaterniond rotation(state.rot.toRotationMatrix());
  if (b.members.size() == 1) b.origin = state.pos;
  b.extent_m = std::max(b.extent_m, (state.pos-b.origin).norm());
  b.key_overlap = b.key_voxels ? VoxelOverlap(*current_voxels_, *b.key_voxels) : 0;
  const bool novel = b.keyframes.empty() ||
    (state.pos-b.key_position).norm() >= submap_key_translation_ ||
    rotation.angularDistance(b.key_rotation) >= submap_key_rotation_ ||
    b.key_overlap < submap_key_overlap_;
  // Empty/weak scans must not exhaust the keyframe budget. The first scan can
  // seed a keyframe once usable geometry exists, even after a weak startup.
  if (novel && current_voxels_ && current_voxels_->size() >= static_cast<size_t>(submap_min_support_) &&
      (b.keyframes.empty() || analytics_msg_.lidar_updated)) {
    b.keyframes.push_back(map_counter_);
    b.key_position = state.pos; b.key_rotation = rotation; b.key_voxels = current_voxels_;
  }
}

bool MappingNode::SuccessorSupportsScan() {
  if (!successor_started_ || successor_->map_cloud_->empty() || scan_cloud_->empty()) return false;
  size_t samples = 0, matches = 0;
  const size_t stride = std::max<size_t>(1, scan_cloud_->size()/2000);
  const auto& state = kf_state_.state;
  for (size_t i=0; i<scan_cloud_->size(); i+=stride) {
    const auto p = scan_cloud_->points[i].getVector3fMap();
    if (!p.allFinite() || p.squaredNorm() > 80*80) continue;
    const V3F world = (state.rot*(state.offset_R_L_I*p.cast<double>()+state.offset_T_L_I)+state.pos).cast<float>();
    if (!world.allFinite()) continue;
    ++samples;
    std::vector<int> indices; std::vector<float> distances;
    successor_->ioctree_.KnnNeighbors(world, 1, indices, distances, kMaxSearchRes);
    if (indices.empty()) continue;
    const int j=indices.front();
    if (successor_->filters_[j][1] && successor_->eigenvalues_[j].allFinite() &&
        successor_->eigenvalues_[j].minCoeff() > 0 && successor_->eigenvectors_[j].allFinite()) ++matches;
  }
  analytics_msg_.successor_support = matches;
  analytics_msg_.successor_overlap = samples ? static_cast<double>(matches)/samples : 0;
  return matches >= static_cast<size_t>(submap_min_support_) && analytics_msg_.successor_overlap >= submap_support_ratio_;
}

void MappingNode::AdvanceMotionSubmaps(int64_t stamp) {
  const auto age = stamp-map_->begin_ns;
  const bool stale = age >= submap_max_age_ns_;
  const bool extent = map_->extent_m >= submap_extent_;
  const bool keys = map_->keyframes.size() >= static_cast<size_t>(submap_keyframes_);
  if ((stale || extent || keys) && successor_started_ &&
      stamp-successor_->begin_ns < submap_max_age_ns_ && SuccessorSupportsScan()) {
    map_->retire_ns = stamp;
    map_->finish_reason = stale ? "max_age" : extent ? "extent" : "keyframes";
    submap_event_ = "handover_"+map_->finish_reason;
#ifdef ELLIPSELIO_RESEARCH_EXPORT
    ExportSubmap(true, stamp);
#endif
    std::swap(map_, successor_);
    successor_started_ = false;
    ++analytics_msg_.handovers;
    ClearPublishedMap();
  } else if (stale) {
    // A timestamp gap or unsupported successor must never expose expired
    // tensors to odometry. This is an explicit recovery reset, not a supported
    // handover. IMU propagation survives and the current scan seeds fresh maps.
    map_->retire_ns = stamp; map_->finish_reason = "stale_recovery";
#ifdef ELLIPSELIO_RESEARCH_EXPORT
    ExportSubmap(true, stamp);
    if (successor_started_) {
      std::swap(map_, successor_);
      ExportSubmap(false, stamp);
      std::swap(map_, successor_);
    }
#endif
    ResetMapBuffer(*map_, stamp);
    ResetMapBuffer(*successor_, stamp);
    successor_started_ = false;
    submap_event_ = "stale_recovery";
    ClearPublishedMap();
  } else if (extent || keys) {
    submap_event_ = "handover_waiting_for_support";
  }
  // Begin growing the successor at half of the nominal geometric/keyframe
  // budget, or half of the stale-age guard. No accumulated tensors are copied.
  if (!successor_started_ &&
      (map_->extent_m >= submap_extent_/2 ||
       map_->keyframes.size() >= static_cast<size_t>((submap_keyframes_+1)/2) ||
       stamp-map_->begin_ns >= submap_max_age_ns_/2)) {
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
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) pose[4*r+c] = rotation(r,c);
    pose[4*r+3] = state.pos[r];
  }
  pose[15] = 1;
  const bool motion = submap_strategy_ == "motion_overlap";
  const bool retrievable = complete && (!motion ||
    (map_->keyframes.size() >= static_cast<size_t>(submap_retrieval_keys_) && ellipsoids.size()/15 >= static_cast<size_t>(submap_min_support_)));
  const int64_t end_ns = motion ? (map_->retire_ns ? map_->retire_ns : map_->last_ns+1) : map_->begin_ns+submap_duration_ns_;
  packet.metadata = {{"schema_version", motion ? 2 : 1}, {"robot_id", submap_robot_},
    {"submap_id", map_->id}, {"keyframe_id", map_->id}, {"complete", complete},
    {"retrievable", retrievable}, {"member_scan_ids", map_->members},
    {"begin_ns", map_->begin_ns}, {"end_ns", end_ns},
    {"last_member_ns", map_->last_ns}, {"stamp_ns", map_->anchor.time.nanoseconds()},
    {"available_ns", available_ns}, {"T_world_imu", pose},
    {"frame", "last_member_imu"},
    {"geometry_source", "native processed scans inserted into this submap; not full-resolution raw geometry"},
    {"geometry_count", geometry.size()/3}, {"ellipsoid_count", ellipsoids.size()/15}};
  if (motion) packet.metadata.update({{"strategy", submap_strategy_}, {"selected_keyframe_scan_ids", map_->keyframes},
    {"extent_m", map_->extent_m}, {"max_age_s", submap_max_age_ns_*1e-9},
    {"finish_reason", complete ? map_->finish_reason : "shutdown_tail"},
    {"min_retrieval_keyframes", submap_retrieval_keys_}, {"min_retrieval_ellipsoids", submap_min_support_}});
  for (const auto* values : {&geometry, &ellipsoids}) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(values->data());
    packet.messages.emplace_back(bytes, bytes + values->size()*sizeof(float));
  }
  submap_export_->enqueue(std::move(packet));
}
}  // namespace ellipselio
#endif

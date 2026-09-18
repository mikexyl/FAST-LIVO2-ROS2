#include "map_processing.h"
#include <pcl/point_types.h>

namespace ellipselio {
void MappingNode::ExportResearchFrame() {
  if (research_cloud_->empty()) throw std::runtime_error("empty synchronized full cloud");
  const auto stamp = kf_state_.time;
  if (stamp != research_stamp_) throw std::runtime_error("LiDAR update changed deskew reference timestamp");
  const auto& state = kf_state_.state;
  const std::string body = research_robot_ + "/imu";
  const std::string lidar = research_robot_ + "/lidar";
  const std::string world = research_robot_ + "/odom_ellipselio";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(research_cloud_->size());
  for (const auto& p : *research_cloud_) {
    pcl::PointXYZI q; q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = p.intensity;
    cloud.push_back(q);
  }
  sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp; msg.header.frame_id = lidar;
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp; odom.header.frame_id = world; odom.child_frame_id = body;
  odom.pose.pose.position.x = state.pos.x(); odom.pose.pose.position.y = state.pos.y(); odom.pose.pose.position.z = state.pos.z();
  odom.pose.pose.orientation.x = state.rot.coeffs()[0]; odom.pose.pose.orientation.y = state.rot.coeffs()[1];
  odom.pose.pose.orientation.z = state.rot.coeffs()[2]; odom.pose.pose.orientation.w = state.rot.coeffs()[3];
  std::vector<double> marginal, matrix, rotation, translation;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = state.rot.toRotationMatrix(); T.block<3,1>(0,3) = state.pos;
  const int tangent[6] = {3,4,5,0,1,2};
  for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
    marginal.push_back(kf_state_.cov(tangent[r],tangent[c]));
    odom.pose.covariance[r*6+c] = kf_state_.cov(r,c);
  }
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) matrix.push_back(T(r,c));
  for (int r = 0; r < 3; ++r) {
    translation.push_back(state.offset_T_L_I[r]);
    for (int c = 0; c < 3; ++c) rotation.push_back(state.offset_R_L_I.toRotationMatrix()(r,c));
  }
  ResearchExport::Packet packet;
  packet.metadata = {{"schema_version", 1}, {"frontend", "ellipselio"}, {"robot_id", research_robot_},
    {"frame_id", research_frame_++}, {"stamp_ns", stamp.nanoseconds()},
    {"scan_start_ns", scan_start_time_.nanoseconds()}, {"scan_end_ns", scan_end_time_.nanoseconds()},
    {"world_frame", world}, {"body_frame", body}, {"cloud_frame", lidar}, {"cloud_points", cloud.size()},
    {"T_world_body", matrix}, {"calibration", {{"R_body_lidar",rotation},{"t_body_lidar",translation}}},
    {"filter_marginal_rotation_translation", marginal},
    {"covariance_provenance", "EllipseLIO filter marginal; diagnostic only; not a relative edge covariance"},
    {"lidar_updated", research_lidar_updated_}, {"lidar_update_invoked", map_counter_ > 0}, {"visual_update_invoked", false},
    {"visual_correction_enabled", false}, {"image_available", false},
    {"cloud_source", "all valid range-filtered input points in processed interval, deskewed before adaptive filtering"},
    {"pose_source", "EllipseLIO post-LiDAR-update IMU state at exact deskew reference time"}};
  packet.add("/"+research_robot_+"/research/cloud", "sensor_msgs/msg/PointCloud2", msg);
  packet.add("/"+research_robot_+"/research/odometry", "nav_msgs/msg/Odometry", odom);
  research_export_->enqueue(std::move(packet));
}
}  // namespace ellipselio

#include "map_processing.h"
#include <cassert>
#include <iostream>

namespace ellipselio {
struct AreaMapTest {
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled",true),
      rclcpp::Parameter("mapping.area_maps.enabled",true),
      rclcpp::Parameter("mapping.area_maps.radius_m",10.),
      rclcpp::Parameter("mapping.area_maps.robot","A"),
      rclcpp::Parameter("publish.markers",false),
      rclcpp::Parameter("imu.topic","/test/imu"),
      rclcpp::Parameter("lidar.topic","/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar",std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar",std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto n=std::make_shared<MappingNode>(options);n->TimerCallback();
    assert(n->area_history_ && n->area_history_.get()!=n->map_.get());
    auto* history=n->area_history_.get();
    n->kf_state_.cov.setIdentity();
    for (int scan=0;scan<80;++scan) {
      n->kf_state_.state.pos=V3D(scan,0,40);n->kf_state_.time=rclcpp::Time(100000000000LL+scan*1000000000LL,RCL_ROS_TIME);
      n->scan_end_time_=n->kf_state_.time;n->scan_cloud_->clear();n->scan_cloud_bins_.setZero();n->start_bin_=0;
      for (int i=0;i<27;++i) {
        EllipseLioPoint p{};p.bin_idx=0;p.x=5+(i%3)*.2-scan;p.y=2+((i/3)%3)*.2;p.z=3+(i/9)*.2-40;
        n->scan_cloud_->push_back(p);
      }
      n->scan_cloud_bins_[0]=27;n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      const auto covariance=n->kf_state_.cov;const auto position=n->kf_state_.state.pos;
      for (const auto& p:*n->map_->map_cloud_)assert(p.scan_idx<scan);
      n->MapIncremental();++n->map_counter_;
      assert(n->area_history_.get()==history && n->map_.get()!=history);
      assert(n->kf_state_.cov==covariance && n->kf_state_.state.pos==position);
      assert(history->members.empty() && history->geometry.empty());
      for (const auto& neighbors:history->neighbours_)
        for (int j:neighbors)assert(j>=0 && size_t(j)<history->map_cloud_->size());
    }
    assert(n->analytics_msg_.handovers>10);
    assert(history->map_cloud_->points.front().scan_idx==0);
    for (auto& p:*n->map_->map_cloud_)assert(p.scan_idx>=70); // Odometry is still recent.
    const size_t count=history->map_cloud_->size();assert(count>0);
    const V3D up=V3D::UnitZ(),center(5,2,140);
    auto near=n->SelectAreaPoints(*history,center,up);assert(near.size()==count);
    assert(n->SelectAreaPoints(*history,V3D(100,0,140),up).empty());
    assert(n->SelectAreaPoints(*history,center,up)==near); // Leaving/returning never ages out points.
    assert(n->SelectAreaPoints(*history,V3D(5,100,3),V3D::UnitY()).size()==count);
    // Exact inclusion of boundary, exclusion just outside, independent of age/height.
    MappingNode::MapBuffer boundary;
    for (float x:{10.f,10.001f}) {EllipseLioPoint p{};p.x=x;p.z=-500;boundary.map_cloud_->push_back(p);}
    assert((n->SelectAreaPoints(boundary,V3D::Zero(),up)==std::vector<int>{0}));
    n->kf_state_.state.pos=center;n->kf_state_.state.grav=S2(V3D(0,0,-9.81));
    for (size_t i=0;i<count;++i) {
      history->filters_[i][1]=1;history->eigenvalues_[i]=V3F::Constant(.2);history->eigenvectors_[i]=M3F::Identity();
    }
    n->scan_end_time_=rclcpp::Time(n->scan_times_.back()+100000000LL,RCL_ROS_TIME);
    auto packet=n->AreaSnapshot("test");
    assert(packet.metadata["anchor_sensor_ns"]==n->scan_times_.back());
    assert(packet.metadata["geometry_count"]==count && packet.metadata["ellipsoid_count"]==count);
    assert(packet.metadata["last_member_ns"]<packet.metadata["anchor_sensor_ns"]);
    assert(packet.metadata["age_limit_s"].is_null() && packet.messages.size()==5);
    assert(packet.messages[2].size()==count*sizeof(int32_t));
    const auto saved=packet.messages;
    history->map_cloud_->points.front().x+=1000;
    assert(packet.messages==saved); // Writer packet owns immutable geometry and IDs.
    std::cout<<"Accumulated map survives odometry handovers; all in-area points retained regardless of age/height; return visits, frame transforms, identities, immutable packets and filter state passed\n";
  }
};
}
int main(int argc,char** argv) {rclcpp::init(argc,argv);ellipselio::AreaMapTest::run();rclcpp::shutdown();}

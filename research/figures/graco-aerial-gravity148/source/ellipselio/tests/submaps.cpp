#include "map_processing.h"
#include <cassert>
#include <iostream>
#include <set>
namespace ellipselio {
struct SubmapTest {
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled", true),
      rclcpp::Parameter("publish.markers", false),
      rclcpp::Parameter("imu.topic", "/test/imu"),
      rclcpp::Parameter("lidar.topic", "/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar", std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar", std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto node = std::make_shared<MappingNode>(options);
    node->TimerCallback();
    node->kf_state_.cov.setIdentity();
    constexpr int64_t epoch = 100000000000LL;
    const auto position = node->kf_state_.state.pos;
    const auto covariance = node->kf_state_.cov;
    std::set<void*> buffers;
    for (int scan = 0; scan < 400; ++scan) {
      const int64_t stamp = epoch + scan*100000000LL;
      node->AdvanceSubmaps(stamp);
      buffers.insert(node->map_.get()); buffers.insert(node->successor_.get());
      const int active = scan < 100 ? 0 : (scan-100)/50+1;
      assert(node->map_->id == active);
      assert(node->successor_started_ == (scan >= 50));
      // The new scan cannot be in the matching map yet.
      for (const auto& p : *node->map_->map_cloud_) assert(p.scan_idx < scan);
      node->scan_cloud_->clear();
      node->scan_cloud_bins_.setZero();
      for (int i=0; i<27; ++i) {
        EllipseLioPoint p{};
        p.x=1+(i%3)*.2f; p.y=2+((i/3)%3)*.2f; p.z=3+(i/9)*.2f;
        p.bin_idx=0; node->scan_cloud_->push_back(p);
      }
      node->scan_cloud_bins_[0]=27;
      node->start_bin_=0;
      node->scan_end_time_=rclcpp::Time(stamp,RCL_ROS_TIME);
      node->kf_state_.time=node->scan_end_time_;
      node->MapIncremental();
      node->map_counter_++;
      for (const auto& p : *node->map_->map_cloud_) {
        assert(p.scan_idx >= active*50);
        assert(p.scan_idx <= scan);
      }
      for (const auto& list : node->map_->neighbours_)
        for (int index : list) assert(index>=0 && index<int(node->map_->map_cloud_->size()));
      assert(node->map_->members.front()==active*50);
      assert(node->map_->members.back()==scan);
      assert(node->map_->geometry.size()==node->map_->members.size()*27);
      assert(node->kf_state_.state.pos == position);
      assert(node->kf_state_.cov == covariance);
    }
    assert(buffers.size()==2);
    node->AdvanceSubmaps(epoch+100000000000LL);
    assert(node->map_->begin_ns==epoch+95000000000LL);
    assert(node->map_->map_cloud_->empty());
    assert(node->map_->tensors_p1_.empty());
    assert(node->map_->members.empty());
    // Empty/one-point scans fail without a statistical reduction.
    esekfom::dyn_share_datastruct<double> data;
    node->scan_cloud_->clear(); node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid);
    node->scan_cloud_->push_back(EllipseLioPoint{});
    node->TensorRegistration(node->kf_state_.state,data); assert(!data.valid);
    // A genuinely one-feature match (two scan points, only one within the
    // unchanged correspondence radius) must fail before sample variance.
    node->scan_cloud_->clear();
    EllipseLioPoint point{}; point.x=1; point.y=0; point.z=0; point.bin_idx=0;
    node->scan_cloud_->push_back(point);
    node->scan_cloud_bins_.setZero(); node->scan_cloud_bins_[0]=1;
    node->scan_end_time_=rclcpp::Time(epoch+100000000000LL,RCL_ROS_TIME);
    node->MapIncremental();
    node->map_counter_++;
    node->scan_end_time_=rclcpp::Time(epoch+100100000000LL,RCL_ROS_TIME);
    node->map_->filters_[0][1]=1;
    node->map_->salivalues_[0]=V3F(1,0,0);
    node->map_->eigenvectors_[0]=M3F::Identity();
    point.z=.02f; node->scan_cloud_->points[0]=point;
    point.x=100; point.y=100; point.z=100; node->scan_cloud_->push_back(point);
    node->mean_bin_=1; node->ekfom_iter_cnt_=0;
    node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid); assert(node->analytics_msg_.num_feats==1);
    // A nonfinite tensor cannot enter correspondence statistical reductions.
    node->map_->salivalues_[0][0]=std::numeric_limits<float>::quiet_NaN();
    node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid); assert(node->analytics_msg_.num_feats==0);
    // Disabled scheduling leaves persistent geometry, tensors and ownership intact.
    auto* persistent=node->map_.get();
    const size_t persistent_size=node->map_->map_cloud_->size();
    node->submaps_enabled_=false;
    node->AdvanceSubmaps(epoch+1000000000000LL);
    assert(node->map_.get()==persistent);
    assert(node->map_->map_cloud_->size()==persistent_size);
    std::cout << "Window boundaries, overlap, map-local indices, member geometry, two-buffer reuse, gaps, state preservation and empty scans passed\n";
  }
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); ellipselio::SubmapTest::run(); rclcpp::shutdown();
}

#include "map_processing.h"
#include <cassert>
#include <iostream>
#include <set>
namespace ellipselio {
struct SubmapTest {
  static std::shared_ptr<MappingNode> motion_node() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled", true),
      rclcpp::Parameter("mapping.submaps.strategy", "motion_overlap"),
      rclcpp::Parameter("mapping.submaps.max_keyframes", 4),
      rclcpp::Parameter("mapping.submaps.extent_m", 100.0),
      rclcpp::Parameter("mapping.submaps.overlap_voxel_m", .1),
      rclcpp::Parameter("mapping.submaps.min_handover_matches", 2),
      rclcpp::Parameter("mapping.submaps.min_handover_overlap", .1),
      rclcpp::Parameter("publish.markers", false),
      rclcpp::Parameter("imu.topic", "/test/imu"),
      rclcpp::Parameter("lidar.topic", "/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar", std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar", std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto node=std::make_shared<MappingNode>(options); node->TimerCallback();
    node->kf_state_.cov.setIdentity();
    return node;
  }

  static void motion_scan(MappingNode& n, int64_t stamp, double position, double yaw=0) {
    n.kf_state_.state.pos=V3D(position,0,0);
    n.kf_state_.state.rot=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,V3D::UnitZ()));
    n.scan_cloud_->clear(); n.scan_cloud_bins_.setZero();
    // Observe the same finite world geometry from different sensor poses.
    for (int x=-4;x<=4;++x) for (int y=-4;y<=4;++y) {
      EllipseLioPoint p{};
      const V3D local=n.kf_state_.state.rot.inverse()*(V3D(x*.2,y*.2,1)-n.kf_state_.state.pos);
      p.getVector3fMap()=local.cast<float>(); p.bin_idx=0; n.scan_cloud_->push_back(p);
    }
    n.scan_cloud_bins_[0]=n.scan_cloud_->size(); n.start_bin_=0;
    n.scan_end_time_=rclcpp::Time(stamp,RCL_ROS_TIME); n.kf_state_.time=n.scan_end_time_;
    const auto covariance=n.kf_state_.cov;
    n.AdvanceSubmaps(stamp);
    assert(n.kf_state_.cov==covariance);
    assert(n.kf_state_.state.pos==V3D(position,0,0));
    for (auto id:n.map_->members) assert(id<n.map_counter_);
    n.analytics_msg_.lidar_updated=true;
    n.MapIncremental(); ++n.map_counter_;
    // Isolate policy/support testing from the tensor fit's saliency threshold.
    for (auto* b:{n.map_.get(),n.successor_.get()}) {
      for (size_t i=0;i<b->map_cloud_->size();++i) {
        b->filters_[i][1]=1; b->eigenvalues_[i]=V3F::Ones(); b->eigenvectors_[i]=M3F::Identity();
      }
      for (const auto& list:b->neighbours_) for (auto id:list)
        assert(id>=0 && id<static_cast<int>(b->map_cloud_->size()));
    }
  }

  static void motion() {
    constexpr int64_t epoch=100000000000LL;
    std::vector<int64_t> ids;
    for (int period:{100000000,200000000}) {
      auto n=motion_node(); std::set<void*> buffers;
      for (int i=0;i<40;++i) {
        motion_scan(*n,epoch+int64_t(i)*period,i*.5);
        buffers.insert(n->map_.get()); buffers.insert(n->successor_.get());
        if (period==100000000) ids.push_back(n->map_->id);
        else assert(ids[i]==n->map_->id); // same geometry, different speed
        if (n->submap_event_=="handover_keyframes") {
          assert(n->analytics_msg_.successor_support>=2);
          assert(n->analytics_msg_.successor_overlap>=.1);
        }
      }
      assert(n->analytics_msg_.handovers>4); assert(buffers.size()==2);
    }
    auto n=motion_node();
    for (int i=0;i<140;++i) motion_scan(*n,epoch+i*100000000LL,0);
    assert(n->map_->id==0 && n->map_->keyframes.size()==1 && !n->successor_started_);
    motion_scan(*n,epoch+14000000000LL,0,.3);
    assert(n->map_->keyframes.size()==2); // rotation alone selects a keyframe
    MappingNode::VoxelSet a{{{0,0,0}},{{10,0,0}}},b=a,c{{{100,0,0}}};
    assert(MappingNode::VoxelOverlap(a,b)==1);
    assert(MappingNode::VoxelOverlap(a,c)==0);
    assert(MappingNode::VoxelOverlap({},b)==0);
    // A genuinely changed view selects a keyframe without pose displacement.
    n->current_voxels_=std::make_shared<const MappingNode::VoxelSet>(MappingNode::VoxelSet{{{100,0,0}},{{110,0,0}}});
    const size_t before=n->map_->keyframes.size();
    n->ObserveSubmapScan(); assert(n->map_->keyframes.size()==before+1);
    // Unsupported nominal handovers wait; expired maps are cleared without
    // claiming a supported handover and never queried across a timestamp gap.
    n->map_->extent_m=101; n->successor_started_=true;
    n->ResetMapBuffer(*n->successor_,epoch+14000000000LL);
    n->AdvanceSubmaps(epoch+15000000000LL);
    assert(n->submap_event_=="handover_waiting_for_support");
    assert(!n->map_->map_cloud_->empty());
    const auto count=n->analytics_msg_.handovers;
    n->AdvanceSubmaps(epoch+31000000000LL);
    assert(n->submap_event_=="stale_recovery");
    assert(n->map_->map_cloud_->empty() && n->map_->tensors_p1_.empty());
    assert(n->analytics_msg_.handovers==count);
    std::cout<<"Motion/rotation/overlap selection, speed invariance, support-gated handovers, stationary retention and stale recovery passed\n";
  }
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
  rclcpp::init(argc,argv); ellipselio::SubmapTest::run(); ellipselio::SubmapTest::motion(); rclcpp::shutdown();
}

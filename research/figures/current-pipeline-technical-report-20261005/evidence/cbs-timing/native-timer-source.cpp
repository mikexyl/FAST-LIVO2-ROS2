#include <Eigen/Cholesky>
#include <algorithm>
#include <aria_viz/visualizer_rerun.h>
#include <cbs/bpsam/bpsam.h>
#include <cbs/factors/sim3_scale_factors.h>
#include <cbs/gbp/gaussian.h>
#include <cbs/key.h>
#include <cbs/utils/graph_partition.h>
#include <cbs/utils/gtsam_ptr.h>
#include <cbs_ros/msg/belief.hpp>
#include <cbs_ros/msg/factor.hpp>
#include <cbs_ros/msg/node_stats.hpp>
#include <cbs_ros/msg/estimate.hpp>
#include <cbs_ros/msg/persistent_input.hpp>
#include <cbs_ros/serialized_size.h>
#include <cbs_ros/belief_response.h>
#include <cbs_ros/msg/pose_graph.hpp>
#include <cbs_ros/msg/value.hpp>
#include <cbs_ros/pose_graph_factor.h>
#include <cbs_ros/distributed_pcm.h>
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
#include <cbs_ros/registration_factors.h>
#include <tbb/global_control.h>
#endif
#include <cbs_ros/ros_utils.h>
#include <cbs_ros/srv/keys.hpp>
#include <cbs_ros/srv/optimization_step.hpp>
#include <ctime>
#include <deque>
#include <filesystem>
#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>
#include <memory>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_edge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <spdlog/fmt/fmt.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

using KeysSrv = cbs_ros::srv::Keys;
using OptimizationStepSrv = cbs_ros::srv::OptimizationStep;
using PoseGraphMsg = pose_graph_tools_msgs::msg::PoseGraph;
using PoseGraphEdgeMsg = pose_graph_tools_msgs::msg::PoseGraphEdge;

namespace {

enum class BeliefSharingStage { Pose, Anchor };
enum class BeliefStageSwitchStrategy { FixedIteration, RosMessage, Random };
enum class CommunicationTopologyMode { DynamicFactors, PublishedGraph };
enum class OptimizationPhase : uint8_t {
  Initialize = 0,
  Exchange = 1,
  Update = 2,
  Finalize = 3,
};

struct LoopScaleMetadata {
  bool has_scale{false};
  double scale{1.0};
  double scale_sigma{-1.0};
};

std::string normalizeToken(std::string token) {
  std::transform(token.begin(), token.end(), token.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  std::replace(token.begin(), token.end(), '-', '_');
  return token;
}

bool parseBeliefSharingStage(const std::string &input,
                             BeliefSharingStage &stage) {
  const std::string token = normalizeToken(input);
  if (token == "pose" || token == "poses" || token == "local" ||
      token == "key" || token == "keys" || token == "pose_beliefs") {
    stage = BeliefSharingStage::Pose;
    return true;
  }
  if (token == "anchor" || token == "anchors" || token == "robot" ||
      token == "robots" || token == "global" || token == "anchor_beliefs") {
    stage = BeliefSharingStage::Anchor;
    return true;
  }
  return false;
}

const char *beliefSharingStageName(BeliefSharingStage stage) {
  return stage == BeliefSharingStage::Pose ? "pose" : "anchor";
}

double beliefSharingStageValue(BeliefSharingStage stage) {
  return stage == BeliefSharingStage::Pose ? 0.0 : 1.0;
}

BeliefStageSwitchStrategy parseBeliefStageSwitchStrategy(std::string strategy) {
  strategy = normalizeToken(std::move(strategy));
  if (strategy == "ros_message" || strategy == "message" ||
      strategy == "topic") {
    return BeliefStageSwitchStrategy::RosMessage;
  }
  if (strategy == "random" || strategy == "random_individual" ||
      strategy == "independent_random") {
    return BeliefStageSwitchStrategy::Random;
  }
  if (strategy != "fixed_iteration" && strategy != "fixed" &&
      strategy != "iteration") {
    throw std::invalid_argument(
        "belief_stage_switch_strategy must be fixed_iteration, ros_message, "
        "or random");
  }
  return BeliefStageSwitchStrategy::FixedIteration;
}

CommunicationTopologyMode parseCommunicationTopologyMode(std::string mode) {
  mode = normalizeToken(std::move(mode));
  if (mode == "dynamic_factors") {
    return CommunicationTopologyMode::DynamicFactors;
  }
  if (mode == "published_graph") {
    return CommunicationTopologyMode::PublishedGraph;
  }
  throw std::invalid_argument(
      "communication_topology_mode must be dynamic_factors or "
      "published_graph");
}

template <class Value> constexpr int valueDim() {
  return gtsam::traits<Value>::dimension;
}

template <class Value> Value identityValue() {
  return gtsam::traits<Value>::Identity();
}

template <class Value> Value poseToValue(const gtsam::Pose3 &pose) {
  if constexpr (std::is_same_v<Value, gtsam::Pose3>) {
    return pose;
  } else {
    static_assert(std::is_same_v<Value, gtsam::Similarity3>);
    return gtsam::Similarity3(pose.rotation(), pose.translation(), 1.0);
  }
}

template <class Value> gtsam::Pose3 valueToPose3(const Value &value) {
  if constexpr (std::is_same_v<Value, gtsam::Pose3>) {
    return value;
  } else {
    static_assert(std::is_same_v<Value, gtsam::Similarity3>);
    return gtsam::Pose3(value.rotation(), value.scale() * value.translation());
  }
}

double resolveScaleSigma(double value, double fallback) {
  return value >= 0.0 ? value : fallback;
}

} // namespace

template <class Value> class CBSRosNode {
  using BPSAM = cbs::BPSAMT<Value>;

public:
  explicit CBSRosNode(const rclcpp::Node::SharedPtr &node) : node_(node) {
    // Parameters
    int robot_id_int = declareParameter("robot_id", -1);
    if (robot_id_int < 0 || robot_id_int >= 26) {
      throw std::invalid_argument(
          "Parameter 'robot_id' is required and must be between 0 and 25");
    }
    robot_id_ = robot_id_int + 'a';
    stats_topic_ = declareParameter(
        "stats_topic", fmt::format("/cbs_ros_{}/stats", robot_id_int));

    std::string log_dir_str = declareParameter("log_dir", std::string(""));

    if (not getParameter("robot_name", robot_name_)) {
      std::string param_name = "robot" + std::to_string(robot_id_int) + "_name";
      CHECK(getParameter(param_name, robot_name_));
    }

    persistent_ = declareParameter("persistent", false);
    pose_stage_probability_ = declareParameter("pose_stage_probability", 0.5);
    if (!std::isfinite(pose_stage_probability_) || pose_stage_probability_ <= 0 || pose_stage_probability_ >= 1)
      throw std::invalid_argument("pose_stage_probability must be in (0,1)");
    max_iterations_ = declareParameter("max_iterations", max_iterations_);
    synchronize_optimization_rounds_ = declareParameter(
        "synchronize_optimization_rounds", synchronize_optimization_rounds_);
    communication_topology_mode_ =
        parseCommunicationTopologyMode(declareParameter(
            "communication_topology_mode", std::string("dynamic_factors")));
    topology_wait_timeout_sec_ = declareParameter("topology_wait_timeout_sec",
                                                  topology_wait_timeout_sec_);
    if (topology_wait_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("topology_wait_timeout_sec must be positive");
    }

    use_robust_noise_models_ =
        declareParameter("use_robust_noise_models", use_robust_noise_models_);
    sim3_scale_sigma_ = declareParameter("sim3_scale_sigma", sim3_scale_sigma_);
    sim3_odom_scale_sigma_ =
        declareParameter("sim3_odom_scale_sigma", sim3_odom_scale_sigma_);
    sim3_loop_scale_sigma_ =
        declareParameter("sim3_loop_scale_sigma", sim3_loop_scale_sigma_);
    sim3_inter_loop_scale_sigma_ = declareParameter(
        "sim3_inter_loop_scale_sigma", sim3_inter_loop_scale_sigma_);
    sim3_pose_scale_prior_sigma_ = declareParameter(
        "sim3_pose_scale_prior_sigma", sim3_pose_scale_prior_sigma_);
    if (sim3_pose_scale_prior_sigma_ == 0.0) {
      throw std::invalid_argument(
          "Sim3 absolute pose scale prior sigma must be negative to disable or "
          "strictly positive to enable");
    }

    belief_stage_switch_strategy_ =
        parseBeliefStageSwitchStrategy(declareParameter(
            "belief_stage_switch_strategy", std::string(persistent_ ? "random" : "fixed_iteration")));
    belief_stage_fixed_iterations_ = declareParameter(
        "belief_stage_fixed_iterations", belief_stage_fixed_iterations_);
    if (belief_stage_fixed_iterations_ <= 0) {
      throw std::invalid_argument(
          "belief_stage_fixed_iterations must be positive");
    }
    belief_stage_topic_ =
        declareParameter("belief_stage_topic", belief_stage_topic_);
    belief_republish_hellinger_threshold_ =
        declareParameter("belief_republish_hellinger_threshold",
                         belief_republish_hellinger_threshold_);
    if (synchronize_optimization_rounds_ &&
        belief_stage_switch_strategy_ !=
            BeliefStageSwitchStrategy::RosMessage) {
      throw std::invalid_argument("synchronize_optimization_rounds requires "
                                  "belief_stage_switch_strategy=ros_message");
    }
    if (synchronize_optimization_rounds_ &&
        communication_topology_mode_ !=
            CommunicationTopologyMode::PublishedGraph) {
      throw std::invalid_argument(
          "synchronize_optimization_rounds requires "
          "communication_topology_mode=published_graph");
    }

    if (persistent_ && (synchronize_optimization_rounds_ ||
        belief_stage_switch_strategy_ != BeliefStageSwitchStrategy::Random ||
        communication_topology_mode_ != CommunicationTopologyMode::DynamicFactors ||
        max_iterations_ != -1))
      throw std::invalid_argument("Persistent CBS requires independent random stages, dynamic topology and no iteration limit");
    bool enable_soft_reset = declareParameter("enable_soft_reset", !persistent_);
    bool enable_dcs = declareParameter("enable_dcs", false);
    bool online = declareParameter("online", true);
    online_ = online;

    std::string method;
    if (online) {
      method = "cbs_online";
    } else if (enable_soft_reset && enable_dcs) {
      method = "cbs_plus";
    } else if (enable_soft_reset && !enable_dcs) {
      method = "cbs_with_soft";
    } else if (!enable_soft_reset && enable_dcs) {
      method = "cbs_with_dcs";
    } else {
      method = "cbs";
    }
    if constexpr (std::is_same_v<Value, gtsam::Similarity3>) {
      method += "_sim3";
    }

    if (!log_dir_str.empty()) {
      if (online) {
        log_dir_ = std::filesystem::path(log_dir_str) / robot_name_ / method;
      } else {
        log_dir_ = std::filesystem::path(log_dir_str) / method / robot_name_;
      }
      if (not std::filesystem::exists(log_dir_)) {
        // create the directory if not exists
        std::filesystem::create_directories(log_dir_);
      } else {
        // remove existing *.g2o and *.tum files
        for (const auto &entry :
             std::filesystem::directory_iterator(log_dir_)) {
          if (entry.is_regular_file()) {
            const std::string filename = entry.path().filename().string();
            if (filename.size() > 4 &&
                (filename.substr(filename.size() - 4) == ".g2o" ||
                 filename.substr(filename.size() - 4) == ".tum" ||
                 filename.substr(filename.size() - 4) == ".csv" ||
                 filename.substr(filename.size() - 4) == ".txt")) {
              std::filesystem::remove(entry.path());
            }
          }
        }
      }
      auto csv_path =
          log_dir_ / ("stats_robot_" + std::to_string(robot_id_int) + ".csv");
      csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
      if (csv_file_.is_open()) {
        csv_file_ << "iteration,residual,bytes_sent,bytes_recv\n";
        csv_file_.flush();
      } else {
        RCLCPP_WARN(node_->get_logger(), "Unable to open CSV file %s",
                    csv_path.string().c_str());
      }
      auto timing_csv_path =
          log_dir_ / ("timing_robot_" + std::to_string(robot_id_int) + ".csv");
      timing_csv_file_.open(timing_csv_path, std::ios::out | std::ios::trunc);
      if (timing_csv_file_.is_open()) {
        timing_csv_file_
            << "iteration,num_new_factors,num_removed_factors,"
               "num_new_values,num_accepted_beliefs,update_ms,stats_ms,"
               "belief_request_ms,visualization_ms,active_callback_ms\n";
        timing_csv_file_.flush();
      } else {
        RCLCPP_WARN(node_->get_logger(), "Unable to open timing CSV file %s",
                    timing_csv_path.string().c_str());
      }
      auto belief_service_timing_csv_path =
          log_dir_ / ("belief_service_timing_robot_" +
                      std::to_string(robot_id_int) + ".csv");
      belief_service_timing_csv_file_.open(belief_service_timing_csv_path,
                                           std::ios::out | std::ios::trunc);
      if (belief_service_timing_csv_file_.is_open()) {
        belief_service_timing_csv_file_
            << "request,iteration,requester_agent_id,include_agent_keys,"
               "explicit_key_count,compact_key_bytes,resolved_key_count,"
               "returned_belief_count,marginalization_graph,lock_wait_ms,"
               "prepare_request_ms,get_beliefs_ms,pack_response_ms,"
               "callback_ms\n";
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Unable to open belief service timing CSV file %s",
                    belief_service_timing_csv_path.string().c_str());
      }
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "CBS filesystem logging disabled because log_dir is empty");
    }

    // BPSAM Init
    typename cbs::BPSAMT<Value>::Params params;
    params.robot_id = robot_id_;
    params.persistent = persistent_;
    params.persistent_belief.minimum = declareParameter("persistent_d_min", 0.01);
    params.persistent_belief.target = declareParameter("persistent_d_target", 0.1);
    params.persistent_belief.maximum = declareParameter("persistent_d_max", 0.3);
    if (persistent_) {
      params.persistent_belief.validate();
      if (enable_soft_reset || enable_dcs)
        throw std::invalid_argument("Persistent CBS uses sender-side fixed damping without soft reset or DCS");
    }
    params.gbp_update_params.type = gbp::GaussianMergeType::Contract;
    params.gbp_update_params.metric_type = gbp::MetricType::Hellinger;
    params.gbp_update_params.d_reset = declareParameter("d_reset", 0.5);
    params.gbp_update_params.contract_alpha =
        declareParameter("contract_alpha", 0.5);
    const double configured_target_hellinger =
        declareParameter("target_hellinger", -1.0);
    if (!persistent_ && belief_stage_switch_strategy_ == BeliefStageSwitchStrategy::Random) {
      if (configured_target_hellinger > 0.0 &&
          std::abs(configured_target_hellinger -
                   params.gbp_update_params.d_reset) > 1e-12) {
        throw std::invalid_argument(
            "random belief-stage switching requires target_hellinger to "
            "equal d_reset");
      }
      params.gbp_update_params.target_hellinger =
          params.gbp_update_params.d_reset;
      RCLCPP_INFO(node_->get_logger(),
                  "Random belief-stage switching uses "
                  "target_hellinger=d_reset=%.6g",
                  params.gbp_update_params.d_reset);
    } else {
      params.gbp_update_params.target_hellinger =
          configured_target_hellinger;
    }
    params.gbp_update_params.gamma = -1.0; // disabled
    params.sam_params_.findUnusedFactorSlots = true;
    registration_enabled_ = declareParameter("registration_enabled", false);
    if (registration_enabled_) {
      if constexpr (!std::is_same_v<Value, gtsam::Pose3>) {
        throw std::invalid_argument("Registration factors require Pose3");
      }
#ifndef CBS_ROS_WITH_REGISTRATION_FACTORS
      throw std::invalid_argument("Rebuild cbs_ros with CBS_ROS_WITH_REGISTRATION_FACTORS=ON");
#else
      registration_gpu_ = nlohmann::json::parse(declareParameter("registration_settings", std::string("{}"))).value("factor", "gicp") == "vgicp_gpu";
      // Matching costs change their correspondences. Refresh at each local
      // update rather than reusing a stale iSAM2 Gaussian factor cache.
      params.sam_params_.relinearizeThreshold = registration_gpu_ ? 0.1 : 0.0;
      params.sam_params_.relinearizeSkip = 1;
      params.sam_params_.cacheLinearizedFactors = registration_gpu_;
      params.sam_params_.enablePartialRelinearizationCheck = false;
#endif
    }
    params.enable_gkcm = declareParameter("enable_gkcm", false);
    if (params.enable_gkcm) {
      throw std::invalid_argument(
          "enable_gkcm is unsupported in CBS ROS; disable it explicitly");
    }
    params.gbp_update_params.enable_soft_reset = enable_soft_reset;
    params.enable_belief_dcs = enable_dcs;
    params.belief_similarity_threshold = belief_republish_hellinger_threshold_;
    params.gbp_update_params.lambda_p = 0.5;
    params.sim3_pose_scale_prior_sigma = sim3_pose_scale_prior_sigma_;

    bpsam_ = std::make_shared<cbs::BPSAMT<Value>>(params);

    // Visualization
    const auto rerun_application_id =
        declareParameter("rerun_application_id", std::string("code-slam"));
    const auto rerun_recording_id_param =
        declareParameter("rerun_recording_id", std::string(""));
    std::optional<std::string> rerun_recording_id;
    if (!rerun_recording_id_param.empty()) {
      rerun_recording_id = rerun_recording_id_param;
    }
    const auto rerun_host = declareParameter(
        "rerun_host", std::string("rerun+http://127.0.0.1:9876/proxy"));
    aria::viz::VisualizerRerun::Params viz_params(
        rerun_application_id, rerun_recording_id, rerun_host);
    if (declareParameter("enable_visualization", true)) {
      viz_ = std::make_shared<aria::viz::VisualizerRerun>(viz_params);
      viz_->addSpdlogToRerun(spdlog::level::info, "cbs/" + robot_name_);
    }

    // The optimization timer waits synchronously for peer belief responses.
    // Keep ROS communication in independent callback groups so the timer cannot
    // prevent pose-graph delivery, service handling, or client responses.
    pose_graph_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribers
    const auto default_pose_graph_topic =
        "/" + robot_name_ + "/kimera_distributed/pose_graph/updates";
    const auto pose_graph_topic =
        declareParameter("pose_graph_topic", default_pose_graph_topic);
    const auto additional_pose_graph_topics = declareParameter(
        "additional_pose_graph_topics", std::vector<std::string>{});
    const bool pose_graph_transient_local =
        declareParameter("pose_graph_transient_local", true);
    rclcpp::SubscriptionOptions pose_graph_options;
    pose_graph_options.callback_group = pose_graph_callback_group_;
    auto pose_graph_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
    if (pose_graph_transient_local) {
      pose_graph_qos.transient_local();
    } else {
      pose_graph_qos.durability_volatile();
    }
    std::vector<std::string> pose_graph_topics{pose_graph_topic};
    pose_graph_topics.insert(pose_graph_topics.end(),
                             additional_pose_graph_topics.begin(),
                             additional_pose_graph_topics.end());
    std::sort(pose_graph_topics.begin(), pose_graph_topics.end());
    const auto duplicate =
        std::adjacent_find(pose_graph_topics.begin(), pose_graph_topics.end());
    if (duplicate != pose_graph_topics.end()) {
      throw std::invalid_argument("Duplicate CBS pose graph topic: " +
                                  *duplicate);
    }
    for (const auto &topic : pose_graph_topics) {
      if (topic.empty()) {
        throw std::invalid_argument("CBS pose graph topics must be nonempty");
      }
      pose_graph_subscriptions_.push_back(
          node_->create_subscription<PoseGraphMsg>(
              topic, pose_graph_qos,
              std::bind(&CBSRosNode::poseGraphCallback, this,
                        std::placeholders::_1),
              pose_graph_options));
      RCLCPP_INFO(node_->get_logger(), "Subscribed to pose graph input %s",
                  topic.c_str());
    }

    if (belief_stage_switch_strategy_ ==
        BeliefStageSwitchStrategy::RosMessage) {
      rclcpp::SubscriptionOptions belief_stage_options;
      belief_stage_options.callback_group = service_callback_group_;
      belief_stage_subscription_ =
          node_->create_subscription<std_msgs::msg::String>(
              belief_stage_topic_, rclcpp::QoS(1).reliable().transient_local(),
              std::bind(&CBSRosNode::beliefStageCallback, this,
                        std::placeholders::_1),
              belief_stage_options);
      RCLCPP_INFO(node_->get_logger(),
                  "Using ROS-message belief stage switching on %s; "
                  "initial stage %s",
                  belief_stage_topic_.c_str(),
                  beliefSharingStageName(belief_sharing_stage_));
    }

    if (communication_topology_mode_ ==
        CommunicationTopologyMode::PublishedGraph) {
      rclcpp::SubscriptionOptions topology_options;
      topology_options.callback_group = service_callback_group_;
      const auto topic = fmt::format("/cbs_ros_{}/neighbors", robot_id_int);
      topology_subscription_ =
          node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
              topic, rclcpp::QoS(1).reliable().transient_local(),
              std::bind(&CBSRosNode::topologyCallback, this,
                        std::placeholders::_1),
              topology_options);
      topology_wait_started_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_->get_logger(),
                  "Using strict published communication topology on %s",
                  topic.c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Using dynamic inter-robot-factor communication topology");
    }

    // Services. Jazzy added typed QoS overloads; Humble only exposes the raw
    // RMW profile overload. Both branches use the standard service QoS.
#if RCLCPP_VERSION_GTE(28, 0, 0)
    const auto service_qos = rclcpp::ServicesQoS();
#else
    const auto service_qos = rmw_qos_profile_services_default;
#endif
    request_beliefs_srv_ = node_->create_service<KeysSrv>(
        fmt::format("/cbs_ros_{}/request_beliefs", int(robot_id_ - 'a')),
        std::bind(&CBSRosNode::requestBeliefsCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        service_qos, service_callback_group_);
    optimization_step_srv_ = node_->create_service<OptimizationStepSrv>(
        fmt::format("/cbs_ros_{}/optimization_step", robot_id_int),
        std::bind(&CBSRosNode::optimizationStepCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        service_qos, service_callback_group_);
    stats_publisher_ = node_->create_publisher<cbs_ros::msg::NodeStats>(
        stats_topic_, rclcpp::QoS(10).reliable());

    estimate_publisher_ = node_->create_publisher<cbs_ros::msg::Estimate>(
        "/" + robot_name_ + "/cbs/estimate",
        rclcpp::QoS(1).reliable().transient_local());

    settle_iterations_ = declareParameter("settle_iterations", persistent_ ? 10 : 0);
    belief_request_timeout_sec_ = declareParameter(
        "belief_request_timeout_sec", belief_request_timeout_sec_);
    if (!std::isfinite(belief_request_timeout_sec_) ||
        belief_request_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("belief_request_timeout_sec must be positive");
    }
    if (settle_iterations_ < 0) {
      throw std::invalid_argument("settle_iterations must be nonnegative");
    }
    input_complete_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/" + robot_name_ + "/cbs/input_complete",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          // PCM seals have an explicit graph-message barrier. A Bool on a
          // separate topic cannot establish that all graph input has arrived.
          if (pcm_) return;
          if (message->data && !input_complete_iteration_) {
            input_complete_iteration_ = iteration_;
          }
        });

    // Clients
    int num_robots_int = declareParameter("num_robots", 2);
    if (num_robots_int <= 0 || num_robots_int > 26) {
      throw std::invalid_argument("num_robots must be in [1, 26]");
    }
    num_robots_ = (size_t)num_robots_int;

    // if self id is smaller than num_robots, exit
    const size_t robot_index = static_cast<size_t>(robot_id_ - 'a');
    if (robot_index >= num_robots_) {
      throw std::invalid_argument(
          fmt::format("Robot ID {} is out of bounds for num_robots {}",
                      robot_index, num_robots_));
    }

    if (declareParameter("pcm_enabled", false)) {
      if constexpr (!std::is_same_v<Value, gtsam::Pose3>) {
        throw std::invalid_argument("Distributed PCM currently supports Pose3 only");
      }
      if (synchronize_optimization_rounds_ ||
          communication_topology_mode_ != CommunicationTopologyMode::DynamicFactors) {
        throw std::invalid_argument(
            "Distributed PCM requires independent timers and dynamic_factors topology");
      }
      cbs::PcmOptions options;
      options.probability = declareParameter("pcm_probability", 0.99);
      const auto minimum = declareParameter("pcm_minimum_clique_size", 2);
      if (minimum < 1) throw std::invalid_argument("PCM minimum clique must be >= 1");
      options.minimum_clique_size = static_cast<size_t>(minimum);
      pcm_ = std::make_unique<cbs_ros::DistributedPcm>(
          node_, robot_id_int, num_robots_int,
          declareParameter("pcm_session_id", std::string("")), options,
          declareParameter("pcm_timeout_sec", 60.0), log_dir_, persistent_);
    }
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
    if (registration_enabled_) {
      if (!pcm_) throw std::invalid_argument("Registration input requires the immutable PCM gate");
      auto settings = nlohmann::json::parse(node_->get_parameter("registration_settings").as_string());
      const int threads = settings.at("num_threads");
      if (threads < 1 || threads > 64) throw std::invalid_argument("Invalid registration thread count");
      registration_threads_ = std::make_unique<tbb::global_control>(
          tbb::global_control::max_allowed_parallelism, threads);
      registration_ = std::make_unique<cbs_ros::RegistrationFactors>(robot_id_int,
          node_->get_parameter("pcm_session_id").as_string(),
          declareParameter("registration_directory", std::string("")), settings, log_dir_);
    }
#endif

    for (size_t i = 0; i < num_robots_; ++i) {
      if (i == robot_index)
        continue;
      std::string srv_name = fmt::format("/cbs_ros_{}/request_beliefs", i);
      neighbor_clients_.push_back(node_->create_client<KeysSrv>(
          srv_name, service_qos, client_callback_group_));
      neighbor_client_ids_.push_back(static_cast<cbs::AgentId>('a' + i));
    }

    if (persistent_) {
      if (!pcm_) throw std::invalid_argument("Persistent ROS input requires distributed PCM");
      persistent_session_ = node_->get_parameter("pcm_session_id").as_string();
      persistent_subscription_ = node_->create_subscription<cbs_ros::msg::PersistentInput>(
          "/" + robot_name_ + "/cbs/persistent_input",
          rclcpp::QoS(1).reliable().transient_local(),
          [this](cbs_ros::msg::PersistentInput::ConstSharedPtr input) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (input->session_id != persistent_session_)
              throw std::runtime_error("Persistent input session mismatch");
            if (persistent_input_ && input->revision == persistent_input_->revision) {
              if (*input != *persistent_input_)
                throw std::runtime_error("Changed duplicate persistent input");
              return;
            }
            if (input->revision != applied_revision_ + 1 ||
                (persistent_input_ && persistent_input_->revision != applied_revision_) ||
                input_complete_iteration_)
              throw std::runtime_error("Nonsequential persistent input or input after final seal");
            persistent_input_ = input;
            persistent_started_ = false;
            persistent_staged_ = false;
          }, pose_graph_options);
    }

    // Timer for optimization loop
    double loop_rate = declareParameter("loop_rate", persistent_ ? 1.0 : 5.0);
    if (loop_rate <= 0.0) {
      throw std::invalid_argument("loop_rate must be positive");
    }
    if (!synchronize_optimization_rounds_) {
      timer_ = node_->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(1.0 / loop_rate)),
          std::bind(&CBSRosNode::timerCallback, this), timer_callback_group_);
    }

    RCLCPP_INFO(node_->get_logger(), "CBSRosNode initialized for robot %c",
                robot_id_);
  }

  void topologyCallback(
      const std_msgs::msg::UInt8MultiArray::ConstSharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::set<cbs::AgentId> published;
    for (const uint8_t neighbor : message->data) {
      if (neighbor >= num_robots_) {
        throw std::runtime_error(
            fmt::format("Published neighbor {} is outside num_robots={}",
                        neighbor, num_robots_));
      }
      const auto neighbor_id = static_cast<cbs::AgentId>('a' + neighbor);
      if (neighbor_id == robot_id_) {
        throw std::runtime_error(
            "Published communication topology contains the local robot");
      }
      if (!published.insert(neighbor_id).second) {
        throw std::runtime_error(fmt::format(
            "Published communication topology contains duplicate neighbor {}",
            neighbor));
      }
    }
    for (const auto observed : observed_neighbor_ids_) {
      if (published.count(observed) == 0) {
        throw std::runtime_error(fmt::format(
            "Published topology for robot {} omits observed inter-robot "
            "factor neighbor {}",
            robot_id_ - 'a', observed - 'a'));
      }
    }
    neighbor_ids_ = std::move(published);
    topology_received_ = true;

    std::ostringstream neighbors;
    for (const auto neighbor : neighbor_ids_) {
      if (neighbors.tellp() > 0) {
        neighbors << ',';
      }
      neighbors << static_cast<int>(neighbor - 'a');
    }
    RCLCPP_INFO(node_->get_logger(),
                "Robot %c accepted published neighbors [%s]", robot_id_,
                neighbors.str().c_str());
  }

  void poseGraphCallback(const PoseGraphMsg::ConstSharedPtr input) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (persistent_) throw std::runtime_error("Use the persistent input topic in persistent mode");
    PoseGraphMsg::ConstSharedPtr msg = input;
    if (pcm_) msg = std::make_shared<PoseGraphMsg>(pcm_->observe(*input));
    ingestGraph(*msg);
  }

  // Caller owns mutex_; only admitted immutable prefixes reach this method.
  void ingestGraph(const PoseGraphMsg& graph) {
    const auto* msg = &graph;
    // gtsam::NonlinearFactorGraph new_factors;
    // gtsam::Values new_values;

    size_t n_odom{0}, n_loop{0};
    for (const auto &edge : msg->edges) {
      if (edge.type == PoseGraphEdgeMsg::ODOM) {
        n_odom++;
      } else if (edge.type == PoseGraphEdgeMsg::LOOPCLOSE) {
        n_loop++;
      }
    }
    if (viz_) {
      viz_->setTime();
      viz_->drawScalar(robot_name_ + "/odom_edges", n_odom);
      viz_->drawScalar(robot_name_ + "/loop_edges", n_loop);
    }

    for (const auto &edge : msg->edges) {
      if (edge.robot_from < 0 || edge.robot_to < 0 ||
          static_cast<size_t>(edge.robot_from) >= num_robots_ ||
          static_cast<size_t>(edge.robot_to) >= num_robots_) {
        throw std::runtime_error("Pose-graph edge contains invalid robot id");
      }
      // Conversion from PoseGraphEdge to BetweenFactor
      gtsam::Key key1 = cbs::toPoseKey(edge.robot_from + 'a', edge.key_from);
      gtsam::Key key2 = cbs::toPoseKey(edge.robot_to + 'a', edge.key_to);

      gtsam::Pose3 measurement(
          gtsam::Rot3::Quaternion(
              edge.pose.orientation.w, edge.pose.orientation.x,
              edge.pose.orientation.y, edge.pose.orientation.z),
          gtsam::Point3(edge.pose.position.x, edge.pose.position.y,
                        edge.pose.position.z));
      cbs_ros::validateScaleMetadata(edge);

      gtsam::Matrix66 cov_mat;
      for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
          cov_mat(r, c) = edge.covariance[r * 6 + c];

      const bool cov_invalid =
          !cov_mat.allFinite() || cov_mat.isZero() ||
          (cov_mat.diagonal().array() <= 0.0).any() ||
          !cov_mat.isApprox(cov_mat.transpose(), 1e-9) ||
          Eigen::LLT<gtsam::Matrix66>(cov_mat).info() != Eigen::Success;
      if (cov_invalid) {
        throw std::runtime_error(
            fmt::format("Edge covariance is invalid for factor ({}, {}).",
                        edge.key_from, edge.key_to));
      }

      const auto route = cbs_ros::routePoseGraphEdge(
          edge, static_cast<int>(robot_id_ - 'a'));
      if (route.neighbor_robot) {
        const auto neighbor =
            static_cast<cbs::AgentId>('a' + *route.neighbor_robot);
        observed_neighbor_ids_.insert(neighbor);
        if (communication_topology_mode_ ==
            CommunicationTopologyMode::DynamicFactors) {
          neighbor_ids_.insert(neighbor);
        } else if (topology_received_ && neighbor_ids_.count(neighbor) == 0) {
          throw std::runtime_error(fmt::format(
              "Observed inter-robot factor to robot {}, which is absent "
              "from the published topology",
              neighbor - 'a'));
        }
      }
      if (!route.owns_factor) {
        continue;
      }

      const auto factor = cbs_ros::makePoseGraphBetweenFactor<Value>(
          edge, key1, key2, measurement, cov_mat,
          resolveScaleSigma(sim3_odom_scale_sigma_, sim3_scale_sigma_),
          use_robust_noise_models_);

      if (edge.type == PoseGraphEdgeMsg::LOOPCLOSE) {
        loop_scale_metadata_[std::make_pair(key1, key2)] =
            LoopScaleMetadata{edge.has_scale,
                              edge.has_scale ? edge.scale : 1.0,
                              edge.has_scale ? edge.scale_sigma : -1.0};
      }

      if (existing_factors_.find(std::make_pair(key1, key2)) !=
          existing_factors_.end()) {
        auto existing_factor = existing_factors_[std::make_pair(key1, key2)];
        if (existing_factor->equals(*factor)) {
          // factor already exists, do nothing
          continue;
        } else {
          existing_factors_[std::make_pair(key1, key2)] = factor;
          const auto factor_key = std::make_pair(key1, key2);
          if (factors_to_add_.count(factor_key) == 0) {
            factor_add_order_.push_back(factor_key);
          }
          factors_to_add_[factor_key] = factor;
          factors_to_remove_.insert(existing_factor);
        }
      } else {
        existing_factors_[std::make_pair(key1, key2)] = factor;
        const auto factor_key = std::make_pair(key1, key2);
        if (factors_to_add_.count(factor_key) == 0) {
          factor_add_order_.push_back(factor_key);
        }
        factors_to_add_[factor_key] = factor;
      }
    }

    for (const auto &node : msg->nodes) {
      if (node.robot_id < 0 ||
          static_cast<size_t>(node.robot_id) >= num_robots_) {
        throw std::runtime_error("Pose-graph node contains invalid robot id");
      }
      const auto cbs_key = cbs::toPoseKey(node.robot_id + 'a', node.key);
      // Filter existing values
      if (!bpsam_->valueExists(cbs_key) &&
          !accumulated_values_.exists(cbs_key)) {
        gtsam::Pose3 pose(gtsam::Rot3::Quaternion(
                              node.pose.orientation.w, node.pose.orientation.x,
                              node.pose.orientation.y, node.pose.orientation.z),
                          gtsam::Point3(node.pose.position.x,
                                        node.pose.position.y,
                                        node.pose.position.z));

        accumulated_values_.insert_or_assign(cbs_key, poseToValue<Value>(pose));
        timestamps_[cbs_key] = rclcpp::Time(node.header.stamp).nanoseconds();
        pending_new_values_since_update_++;
      }
    }
    if (!msg->nodes.empty()) {
      initial_pose_graph_received_ = true;
    }
  }

  void requestBeliefsCallback(const std::shared_ptr<KeysSrv::Request> req,
                              std::shared_ptr<KeysSrv::Response> res) {
    using SteadyClock = std::chrono::steady_clock;
    const auto callback_start = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto lock_acquired = SteadyClock::now();
    if (req->agent_id < 'a' ||
        req->agent_id >= static_cast<cbs::AgentId>('a' + num_robots_) ||
        req->agent_id == robot_id_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Rejecting belief request with invalid agent id %d",
                   static_cast<int>(req->agent_id));
      return;
    }
    if (communication_topology_mode_ ==
            CommunicationTopologyMode::PublishedGraph &&
        topology_received_ && neighbor_ids_.count(req->agent_id) == 0) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Rejecting belief request from non-neighbor robot %d",
                   static_cast<int>(req->agent_id - 'a'));
      return;
    }
    std::vector<gtsam::Key> compact_keys;
    try {
      compact_keys = cbs_ros::decodeCompactKeys(req->compact_keys);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Rejecting malformed compact key request from robot %c: %s",
                   req->agent_id, error.what());
      return;
    }

    KeySet keys;
    const char *marginalization_graph = "local";
    if (!req->include_agent_keys && req->keys.empty() && compact_keys.empty()) {
      // if empty, then share all anchor beliefs
      for (size_t i = 0; i < num_robots_; ++i) {
        keys.insert(cbs::toRobotKey(static_cast<cbs::AgentId>(i + 'a')));
      }
      if (!persistent_) bpsam_->setMarginalizationGraph(BPSAM::MarginalizationType::GLOBAL);
      marginalization_graph = "global";
    } else {
      if (req->include_agent_keys) {
        for (auto key : bpsam_->allKeys()) {
          if (keyBelongsToAgent(key, req->agent_id)) {
            keys.insert(key);
          }
        }
      }
      for (auto k : req->keys) {
        keys.insert(k);
      }
      for (auto k : compact_keys) {
        keys.insert(k);
      }
      if (!persistent_) bpsam_->setMarginalizationGraph(BPSAM::MarginalizationType::LOCAL);
    }

    const auto get_beliefs_start = SteadyClock::now();
    if (!persistent_ && !req->include_agent_keys &&
        !req->initialized_anchor_ids.empty()) {
      KeySet initialized;
      for (const auto id : req->initialized_anchor_ids) {
        if (id >= num_robots_) throw std::invalid_argument("Invalid initialized anchor ID");
        initialized.insert(cbs::toRobotKey(static_cast<cbs::AgentId>('a' + id)));
      }
      bpsam_->refreshMissingAnchorBootstraps(req->agent_id, initialized);
    }
    std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> beliefs;
    if (persistent_) {
      KeySet poses, anchors;
      for (const auto key : keys) {
        if (cbs::isPoseKey(key)) poses.insert(key);
        else if (cbs::isRobotKey(key)) anchors.insert(key);
      }
      if (!poses.empty()) {
        bpsam_->setMarginalizationGraph(BPSAM::MarginalizationType::LOCAL);
        beliefs = bpsam_->getBeliefs(poses, req->agent_id, false);
      }
      if (!anchors.empty()) {
        bpsam_->setMarginalizationGraph(BPSAM::MarginalizationType::GLOBAL);
        const auto anchor_beliefs = bpsam_->getBeliefs(anchors, req->agent_id, true);
        beliefs.insert(anchor_beliefs.begin(), anchor_beliefs.end());
      }
    } else beliefs = bpsam_->getBeliefs(keys, req->agent_id, true);
    const auto get_beliefs_end = SteadyClock::now();

    for (const auto &[key, agent_map] : beliefs) {
      for (const auto &[agent_id, gaussian] : agent_map) {
        cbs_ros::appendGaussianToBatch(gaussian, agent_id, res->beliefs);
      }
    }
    const auto response_packed = SteadyClock::now();

    belief_service_request_count_++;
    if (belief_service_timing_csv_file_.is_open()) {
      belief_service_timing_csv_file_
          << belief_service_request_count_ << "," << iteration_ << ","
          << static_cast<int>(req->agent_id) << ","
          << static_cast<int>(req->include_agent_keys) << ","
          << req->keys.size() + compact_keys.size() << ","
          << req->compact_keys.size() << "," << keys.size() << ","
          << res->beliefs.keys.size() << "," << marginalization_graph << ","
          << std::fixed << std::setprecision(6)
          << elapsedMilliseconds(callback_start, lock_acquired) << ","
          << elapsedMilliseconds(lock_acquired, get_beliefs_start) << ","
          << elapsedMilliseconds(get_beliefs_start, get_beliefs_end) << ","
          << elapsedMilliseconds(get_beliefs_end, response_packed) << ","
          << elapsedMilliseconds(callback_start, response_packed) << "\n";
      if (belief_service_request_count_ % 10 == 0) {
        belief_service_timing_csv_file_.flush();
      }
    }
  }

  void processBeliefs(const std::vector<cbs::AgentId> &sender_ids,
                      const std::vector<gbp::Gaussian> &beliefs) {
    if (sender_ids.size() != beliefs.size()) {
      throw std::invalid_argument(
          "Belief sender-id count does not match belief count");
    }
    std::map<gtsam::Key, std::vector<std::pair<cbs::AgentId, gbp::Gaussian>>>
        input_beliefs;
    size_t received = 0;
    for (size_t index = 0; index < beliefs.size(); ++index) {
      if (sender_ids[index] < 'a' ||
          sender_ids[index] >= static_cast<cbs::AgentId>('a' + num_robots_)) {
        throw std::invalid_argument("Belief batch contains invalid sender id");
      }
      if (sender_ids[index] == bpsam_->id()) {
        continue;
      }
      input_beliefs[beliefs[index].key()].emplace_back(sender_ids[index],
                                                       beliefs[index]);
      ++received;
    }
    beliefs_received_since_update_ += received;
    const int num_rejected = bpsam_->addBeliefs(std::move(input_beliefs));
    if (num_rejected < 0 || static_cast<size_t>(num_rejected) > received) {
      throw std::runtime_error("BPSAM returned an invalid rejection count");
    }
    beliefs_accepted_since_update_ +=
        received - static_cast<size_t>(num_rejected);
  }

  void
  beliefStageCallback(const std_msgs::msg::String::ConstSharedPtr message) {
    BeliefSharingStage stage;
    if (!parseBeliefSharingStage(message->data, stage)) {
      RCLCPP_WARN(node_->get_logger(),
                  "Ignoring unknown belief stage '%s'; expected pose or "
                  "anchor",
                  message->data.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (belief_sharing_stage_ == stage) {
      return;
    }
    belief_sharing_stage_ = stage;
    RCLCPP_INFO(node_->get_logger(), "Switched belief stage to %s",
                beliefSharingStageName(stage));
  }

  bool preparePersistentInput() {
    if (!persistent_input_) return false;
    if (persistent_input_->revision == applied_revision_) return true;
    if (!persistent_started_) {
      const auto revision = std::to_string(persistent_input_->revision);
      const auto session = persistent_session_ + "/" + revision;
      const auto logs = log_dir_ / "revisions" / revision;
      std::filesystem::create_directories(logs);
      // PCM starts a new immutable transaction on the same subscriptions.
      // Optimizer, belief history and geometry caches remain alive.
      pcm_->beginRevision(session, logs);
      persistent_graph_ = pcm_->observe(persistent_input_->graph);
      cbs_ros::msg::PcmInputSeal seal;
      seal.session_id = session;
      seal.graph_messages = 1;
      seal.own_nodes = persistent_graph_.nodes.size();
      pcm_->seal(seal);
      persistent_started_ = true;
    }
    if (!pcm_->advance()) {
      // Continue optimizing the last admitted graph while the next PCM
      // transaction exchanges its evidence. There is no stage barrier.
      return initial_pose_graph_received_;
    }
    if (persistent_staged_) return true;
    PoseGraphMsg admitted = persistent_graph_;
    admitted.edges.clear();
    std::set<std::pair<gtsam::Key, gtsam::Key>> retained;
    for (const auto& edge : persistent_graph_.edges) {
      const auto i = cbs::toPoseKey('a' + edge.robot_from, edge.key_from);
      const auto j = cbs::toPoseKey('a' + edge.robot_to, edge.key_to);
      if (pcm_->admits(i, j)) {
        admitted.edges.push_back(edge);
        retained.emplace(i, j);
      }
    }
    // The draft's persistent algorithm grows F_local; it does not specify a
    // retraction/reset protocol for beliefs derived from revoked measurements.
    // Fail closed, rather than leave stale evidence in a supposedly valid run.
    for (const auto& [pair, factor] : existing_factors_) {
      if (!retained.count(pair))
        throw std::runtime_error("Persistent CBS cannot retract an admitted factor; PCM selection changed. Preserve diagnostics and use sealed mode for this input.");
    }
    for (const auto& node : admitted.nodes) {
      const auto key = cbs::toPoseKey('a' + node.robot_id, node.key);
      const auto old = immutable_nodes_.find(key);
      if (old != immutable_nodes_.end() && old->second != node)
        throw std::runtime_error("Persistent keyframe metadata changed");
      immutable_nodes_.emplace(key, node);
    }
    if (admitted.nodes.size() != immutable_nodes_.size())
      throw std::runtime_error("Persistent input removed local keyframes");
    ingestGraph(admitted);
    neighbor_ids_ = pcm_->neighbors();
    observed_neighbor_ids_ = neighbor_ids_;
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
    if (registration_) {
      const auto revision = std::to_string(persistent_input_->revision);
      registration_->beginRevision(persistent_session_ + "/" + revision,
          std::filesystem::path(node_->get_parameter("registration_directory").as_string()) / revision);
      registration_added_ = false;
    }
#endif
    persistent_staged_ = true;
    return true;
  }

  void requestBeliefsAsync() {
    using Clock = std::chrono::steady_clock;
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < neighbor_clients_.size(); ++i) {
      const auto peer = neighbor_client_ids_[i];
      auto& client = neighbor_clients_[i];
      if (!neighbor_ids_.count(peer)) continue;
      auto previous = pending_requests_.find(peer);
      if (previous != pending_requests_.end()) {
        if (std::chrono::duration<double>(Clock::now() - previous->second.started).count() < belief_request_timeout_sec_)
          continue;
        client->remove_pending_request(previous->second.request_id);
        pending_requests_.erase(previous);
        RCLCPP_WARN(node_->get_logger(), "Expired belief request to %c; retaining its previous beliefs", peer);
      }
      if (!client->service_is_ready()) continue;
      auto request = std::make_shared<KeysSrv::Request>();
      request->agent_id = robot_id_;
      request->include_agent_keys = belief_sharing_stage_ == BeliefSharingStage::Pose;
      if (request->include_agent_keys) {
        std::vector<gtsam::Key> keys;
        for (auto key : bpsam_->allKeys())
          if (cbs::isPoseKey(key) && !keyBelongsToAgent(key, robot_id_)) keys.push_back(key);
        request->compact_keys = cbs_ros::encodeCompactKeys(keys);
      }
      const auto token = ++request_token_;
      pending_requests_[peer] = {token, 0, Clock::now()};
      bandwidth_sent_bytes_ += cbs_ros::serializedSize(*request);
      const auto future = client->async_send_request(request,
          [this, peer, token](typename rclcpp::Client<KeysSrv>::SharedFuture result) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto pending = pending_requests_.find(peer);
            if (pending == pending_requests_.end() || pending->second.token != token) return;
            const auto response = result.get();
            pending_requests_.erase(pending);
            bandwidth_recv_bytes_ += cbs_ros::serializedSize(*response);
            std::vector<cbs::AgentId> senders;
            const auto beliefs = cbs_ros::batchToGaussians(response->beliefs, &senders);
            processBeliefs(senders, beliefs);
          });
      pending_requests_.at(peer).request_id = future.request_id;
    }
  }

  struct UpdateMetrics {
    size_t num_new_factors{0};
    size_t num_removed_factors{0};
    size_t num_new_values{0};
    size_t num_received_beliefs{0};
    size_t num_accepted_beliefs{0};
    double update_ms{0.0};
    double stats_ms{0.0};
  };

  bool communicationTopologyReady(bool enforce_timeout) const {
    if (communication_topology_mode_ ==
        CommunicationTopologyMode::DynamicFactors) {
      return true;
    }
    if (topology_received_) {
      return true;
    }
    if (enforce_timeout &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      topology_wait_started_)
                .count() >= topology_wait_timeout_sec_) {
      throw std::runtime_error(fmt::format(
          "Timed out after {:.3f}s waiting for the published communication "
          "topology for robot {}",
          topology_wait_timeout_sec_, robot_id_ - 'a'));
    }
    return false;
  }

  bool performUpdate(std::optional<size_t> synchronized_round,
                     bool publish_stats, UpdateMetrics *metrics_out) {
    using SteadyClock = std::chrono::steady_clock;
    std::lock_guard<std::mutex> lock(mutex_);
    if (persistent_) {
      if (!preparePersistentInput()) return false;
    } else if (pcm_ && !pcm_->advance()) return false;
    if (!initial_pose_graph_received_ || !communicationTopologyReady(false)) {
      return false;
    }

    if (!persistent_ && pcm_ && !pcm_applied_) {
      // No call to BPSAM::update (including separator/anchor initialization)
      // has happened yet. Rejected measurements cannot influence any belief.
      for (auto it = factors_to_add_.begin(); it != factors_to_add_.end();) {
        if (!pcm_->admits(it->first.first, it->first.second)) {
          existing_factors_.erase(it->first);
          loop_scale_metadata_.erase(it->first);
          it = factors_to_add_.erase(it);
        } else {
          ++it;
        }
      }
      neighbor_ids_ = pcm_->neighbors();
      observed_neighbor_ids_ = neighbor_ids_;
      input_complete_iteration_ = iteration_;
      pcm_applied_ = true;
    }
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
    if (registration_) {
      std::set<std::pair<gtsam::Key, gtsam::Key>> selected;
      for (const auto& [pair, metadata] : loop_scale_metadata_) selected.insert(pair);
      if (!registration_->prepare(persistent_ ? existing_factors_ : factors_to_add_, selected)) return false;
    }
#endif

    UpdateMetrics metrics;

    FactorIndexSet factors_to_remove;
    auto &vi = bpsam_->getVariableIndex();
    for (const auto &factor : factors_to_remove_) {
      if (auto it = vi.find(factor->keys()[0]); it != vi.end()) {
        for (auto existing_factor_id : it->second) {
          auto existing_factor = bpsam_->getFactorsUnsafe()[existing_factor_id];
          if (!existing_factor || existing_factor->keys().size() < 2) {
            continue;
          }

          if ((existing_factor->keys()[0] == factor->keys()[0] &&
               existing_factor->keys()[1] == factor->keys()[1]) ||
              (existing_factor->keys()[0] == factor->keys()[1] &&
               existing_factor->keys()[1] == factor->keys()[0])) {
            factors_to_remove.insert(existing_factor_id);
          }
        }
      }
    }

    std::vector<NonlinearFactor::shared_ptr> ordered_new_factors;
    ordered_new_factors.reserve(factor_add_order_.size() + 2);
    for (const auto &factor_key : factor_add_order_) {
      const auto factor_it = factors_to_add_.find(factor_key);
      if (factor_it != factors_to_add_.end()) {
        ordered_new_factors.push_back(factor_it->second);
      }
    }
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
    if (registration_ && !registration_added_) {
      for (const auto& factor : (persistent_ ? registration_->newFactors() : registration_->factors())) ordered_new_factors.push_back(factor);
    }
#endif

    if (bpsam_->getFactorsUnsafe().empty()) {
      ordered_new_factors.push_back(cbs::make_shared<gtsam::PriorFactor<Value>>(
          cbs::toPoseKey(robot_id_, 0), identityValue<Value>(),
          gtsam::noiseModel::Isotropic::Sigma(valueDim<Value>(), 0.1)));
      ordered_new_factors.push_back(cbs::make_shared<BetweenFactor<Value>>(
          cbs::toRobotKey(robot_id_), cbs::toPoseKey(robot_id_, 0),
          identityValue<Value>(),
          gtsam::noiseModel::Isotropic::Sigma(valueDim<Value>(), 1e-4)));
      accumulated_values_.insert_or_assign(cbs::toPoseKey(robot_id_, 0),
                                           identityValue<Value>());
      accumulated_values_.insert_or_assign(cbs::toRobotKey(robot_id_),
                                           identityValue<Value>());
    }

    auto factor_order = [this](const NonlinearFactor::shared_ptr &factor) {
      size_t local_pose_index = std::numeric_limits<size_t>::max();
      bool has_foreign_pose = false;
      for (const gtsam::Key key : factor->keys()) {
        const gtsam::LabeledSymbol symbol(key);
        if (!cbs::isPoseKey(symbol)) {
          continue;
        }
        if (cbs::getAgentId(symbol) == robot_id_) {
          local_pose_index = std::min(local_pose_index, symbol.index());
        } else {
          has_foreign_pose = true;
        }
      }
      return std::make_pair(local_pose_index, has_foreign_pose ? 1 : 0);
    };
    std::stable_sort(ordered_new_factors.begin(), ordered_new_factors.end(),
                     [&](const auto &lhs, const auto &rhs) {
                       return factor_order(lhs) < factor_order(rhs);
                     });

    NonlinearFactorGraph new_factors;
    for (const auto &factor : ordered_new_factors) {
      new_factors.add(factor);
    }
    metrics.num_new_factors = new_factors.size();

    typename BPSAM::UpdateParams update_params;
    if (registration_enabled_ && !registration_gpu_) {
      update_params.force_relinearize = true;
      update_params.forceFullSolve = true;
    }
    update_params.removeFactorIndices =
        FactorIndices(factors_to_remove.begin(), factors_to_remove.end());
    metrics.num_removed_factors = factors_to_remove.size();
    metrics.num_new_values = pending_new_values_since_update_;
    metrics.num_received_beliefs = beliefs_received_since_update_;
    metrics.num_accepted_beliefs = beliefs_accepted_since_update_;

    const auto update_start = SteadyClock::now();
    bpsam_->update(new_factors, accumulated_values_, update_params);
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
    if (registration_) registration_added_ = true;
#endif
    if (persistent_ && persistent_staged_) {
      applied_revision_ = persistent_input_->revision;
      if (persistent_input_->final) input_complete_iteration_ = iteration_;
      persistent_staged_ = false;
    }
    metrics.update_ms = elapsedMilliseconds(update_start, SteadyClock::now());
    factors_to_add_.clear();
    factor_add_order_.clear();
    factors_to_remove_.clear();
    pending_new_values_since_update_ = 0;
    beliefs_received_since_update_ = 0;
    beliefs_accepted_since_update_ = 0;

    if (synchronized_round) {
      iteration_ = *synchronized_round;
    } else if (publish_stats) {
      iteration_++;
      updateBeliefSharingStage();
    }
    if (publish_stats) {
      const auto stats_start = SteadyClock::now();
      publishEstimate();
      publishStats(metrics.num_new_factors, metrics.num_removed_factors,
                   metrics.num_new_values, metrics.num_received_beliefs,
                   metrics.num_accepted_beliefs);
      metrics.stats_ms = elapsedMilliseconds(stats_start, SteadyClock::now());
    }
    if (metrics_out) {
      *metrics_out = metrics;
    }
    return true;
  }

  void timerCallback() {
    using SteadyClock = std::chrono::steady_clock;
    const auto callback_start = SteadyClock::now();
    UpdateMetrics metrics;
    if (!performUpdate(std::nullopt, true, &metrics)) {
      std::lock_guard<std::mutex> lock(mutex_);
      communicationTopologyReady(true);
      return;
    }

    double belief_request_ms = 0.0;
    double visualization_ms = 0.0;

    bool settled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      settled = settle_iterations_ > 0 && input_complete_iteration_ &&
                iteration_ >= *input_complete_iteration_ + settle_iterations_;
    }
    if (settled || (max_iterations_ > 0 &&
        iteration_ > static_cast<size_t>(max_iterations_))) {
      RCLCPP_INFO(node_->get_logger(),
                  "Finished local optimization at iteration %zu.", iteration_);

      {
        std::lock_guard<std::mutex> lock(mutex_);
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
        if (registration_) registration_->finish(bpsam_->calculateEstimate());
#endif
        optimization_finished_ = true;
        publishEstimate();
        visualize(true);
      }

      // Keep serving final beliefs until the launcher stops the process. A
      // faster robot must not disappear while a slower peer is still settling.
      timer_->cancel();
      return;
    }

    // Communicate
    const auto belief_request_start = SteadyClock::now();
    if (persistent_) requestBeliefsAsync();
    else requestBeliefs();
    belief_request_ms =
        elapsedMilliseconds(belief_request_start, SteadyClock::now());

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Visualize
      const auto visualization_start = SteadyClock::now();
      visualize();
      visualization_ms =
          elapsedMilliseconds(visualization_start, SteadyClock::now());
    }

    const double active_callback_ms =
        elapsedMilliseconds(callback_start, SteadyClock::now());
    logTiming(metrics.num_new_factors, metrics.num_removed_factors,
              metrics.num_new_values, metrics.num_accepted_beliefs,
              metrics.update_ms, metrics.stats_ms, belief_request_ms,
              visualization_ms, active_callback_ms);


  }

  void optimizationStepCallback(
      const std::shared_ptr<OptimizationStepSrv::Request> request,
      std::shared_ptr<OptimizationStepSrv::Response> response) {
    try {
      if (!synchronize_optimization_rounds_) {
        response->success = false;
        response->message =
            "synchronize_optimization_rounds is disabled for this node";
        return;
      }

      const auto phase = static_cast<OptimizationPhase>(request->phase);
      if (phase == OptimizationPhase::Initialize) {
        if (optimization_initialized_) {
          response->success = true;
          response->message = "already initialized";
          return;
        }
        if (!performUpdate(std::nullopt, false, nullptr)) {
          bool waiting_for_topology = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            communicationTopologyReady(true);
            waiting_for_topology = !topology_received_;
          }
          response->success = false;
          response->message = waiting_for_topology
                                  ? "waiting for published topology"
                                  : "waiting for initial pose graph";
          return;
        }
        optimization_initialized_ = true;
        response->success = true;
        response->message = "initialized";
        return;
      }

      if (!optimization_initialized_) {
        response->success = false;
        response->message = "optimizer is not initialized";
        return;
      }

      if (phase == OptimizationPhase::Exchange) {
        BeliefSharingStage requested_stage;
        if (!parseBeliefSharingStage(request->belief_stage, requested_stage)) {
          response->success = false;
          response->message = "invalid belief stage";
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (request->round != next_synchronized_round_) {
            response->success = false;
            response->message =
                fmt::format("expected round {}", next_synchronized_round_);
            return;
          }
          belief_sharing_stage_ = requested_stage;
        }
        requestBeliefs();
        exchanged_synchronized_round_ = request->round;
        response->success = true;
        response->message = "exchange complete";
        return;
      }

      if (phase == OptimizationPhase::Update) {
        if (!exchanged_synchronized_round_ ||
            *exchanged_synchronized_round_ != request->round ||
            request->round != next_synchronized_round_) {
          response->success = false;
          response->message = "exchange phase has not completed for this round";
          return;
        }
        if (!performUpdate(static_cast<size_t>(request->round), true,
                           nullptr)) {
          response->success = false;
          response->message = "pose graph or topology is unavailable";
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          visualize();
          ++next_synchronized_round_;
          exchanged_synchronized_round_.reset();
        }
        response->success = true;
        response->message = "update complete";
        return;
      }

      if (phase == OptimizationPhase::Finalize) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!optimization_finished_) {
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
          if (registration_) registration_->finish(bpsam_->calculateEstimate());
#endif
          visualize(true);
          optimization_finished_ = true;
        }
        response->success = true;
        response->message = "finalized";
        return;
      }

      response->success = false;
      response->message = "unknown optimization phase";
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(), "Optimization step failed: %s",
                   error.what());
      response->success = false;
      response->message = error.what();
    }
  }

  template <typename TimePoint>
  static double elapsedMilliseconds(const TimePoint &start,
                                    const TimePoint &end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  void logTiming(size_t num_new_factors, size_t num_removed_factors,
                 size_t num_new_values, size_t num_accepted_beliefs,
                 double update_ms, double stats_ms, double belief_request_ms,
                 double visualization_ms, double active_callback_ms) {
    if (timing_csv_file_.is_open()) {
      timing_csv_file_ << iteration_ << "," << num_new_factors << ","
                       << num_removed_factors << "," << num_new_values << ","
                       << num_accepted_beliefs << "," << std::fixed
                       << std::setprecision(6) << update_ms << "," << stats_ms
                       << "," << belief_request_ms << "," << visualization_ms
                       << "," << active_callback_ms << "\n";
      timing_csv_file_.flush();
    }

    const std::string timing_entity =
        "/cbs/metrics/" + robot_name_ + "/timing_ms";
    if (!viz_) return;
    viz_->setTime();
    viz_->drawScalar(timing_entity + "/update", update_ms);
    viz_->drawScalar(timing_entity + "/stats", stats_ms);
    viz_->drawScalar(timing_entity + "/belief_request", belief_request_ms);
    viz_->drawScalar(timing_entity + "/visualization", visualization_ms);
    viz_->drawScalar(timing_entity + "/active_callback", active_callback_ms);
  }

  void requestBeliefs() {
    size_t bytes_sent = 0;
    size_t bytes_recv = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!communicationTopologyReady(true)) {
        return;
      }
    }

    for (size_t i = 0; i < neighbor_clients_.size(); ++i) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (neighbor_ids_.count(neighbor_client_ids_.at(i)) == 0) {
          continue;
        }
      }
      auto &client = neighbor_clients_[i];
      auto request = std::make_shared<KeysSrv::Request>();
      request->agent_id = robot_id_;
      {
        std::lock_guard<std::mutex> lock(mutex_);

        request->include_agent_keys =
            belief_sharing_stage_ == BeliefSharingStage::Pose;
        // A Gaussian can arrive before its publisher's connecting separator.
        // Acknowledge factor-connected anchors, not unconnected cached guesses,
        // so a missing anchor can receive fresh bootstrap replies afterward.
        for (const auto& [key, factors] : bpsam_->getVariableIndex()) {
          if (cbs::isRobotKey(key) && !factors.empty()) {
            request->initialized_anchor_ids.push_back(cbs::getAgentId(key) - 'a');
          }
        }
        if (request->include_agent_keys) {
          std::vector<gtsam::Key> explicit_keys;
          for (auto key : bpsam_->allKeys()) {
            if (!keyBelongsToAgent(key, robot_id_)) {
              explicit_keys.push_back(key);
            }
          }
          request->compact_keys = cbs_ros::encodeCompactKeys(explicit_keys);
        }
      }

      if (!client->service_is_ready()) {
        if (synchronize_optimization_rounds_) {
          throw std::runtime_error(fmt::format(
              "Belief service for configured neighbor {} is unavailable",
              neighbor_client_ids_.at(i) - 'a'));
        }
        continue;
      }

      bytes_sent += cbs_ros::serializedSize(*request);
      auto future = client->async_send_request(request);
      try {
        auto response = cbs_ros::awaitBeliefResponse(
            future, belief_request_timeout_sec_,
            fmt::format("configured neighbor {}", neighbor_client_ids_.at(i) - 'a'));
        bytes_recv += cbs_ros::serializedSize(*response);
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<cbs::AgentId> sender_ids;
        const auto beliefs =
            cbs_ros::batchToGaussians(response->beliefs, &sender_ids);
        processBeliefs(sender_ids, beliefs);
      } catch (...) {
        client->remove_pending_request(future);
        throw;
      }
    }

    bandwidth_sent_bytes_ += bytes_sent;
    bandwidth_recv_bytes_ += bytes_recv;
  }

  bool keyBelongsToAgent(gtsam::Key key, cbs::AgentId agent_id) const {
    const gtsam::LabeledSymbol labeled_key(key);
    if (!cbs::isPoseKey(labeled_key) && !cbs::isRobotKey(labeled_key)) {
      return false;
    }
    return cbs::getAgentId(labeled_key) == agent_id;
  }

  void updateBeliefSharingStage() {
    if (belief_stage_switch_strategy_ ==
        BeliefStageSwitchStrategy::RosMessage) {
      return;
    }

    if (belief_stage_switch_strategy_ == BeliefStageSwitchStrategy::Random) {
      std::bernoulli_distribution choose_pose(pose_stage_probability_);
      belief_sharing_stage_ = choose_pose(belief_stage_rng_)
                                  ? BeliefSharingStage::Pose
                                  : BeliefSharingStage::Anchor;
      return;
    }

    const auto block =
        iteration_ / static_cast<size_t>(belief_stage_fixed_iterations_);
    belief_sharing_stage_ =
        block % 2 == 0 ? BeliefSharingStage::Pose : BeliefSharingStage::Anchor;
  }

  double resultLogmapChange(const gtsam::Values &previous,
                            const gtsam::Values &current,
                            uint64_t &num_changed_values) const {
    double sum_squared = 0.0;
    num_changed_values = 0;

    for (const auto &[key, value] : current) {
      if (!previous.exists(key)) {
        continue;
      }
      const auto &current_value = value.template cast<Value>();
      const auto &previous_value = previous.template at<Value>(key);
      const auto delta = gtsam::traits<Value>::Logmap(previous_value.inverse() *
                                                      current_value);
      sum_squared += delta.squaredNorm();
      num_changed_values++;
    }

    if (num_changed_values == 0) {
      return 0.0;
    }
    return std::sqrt(sum_squared / static_cast<double>(num_changed_values));
  }

  void publishEstimate() {
    const auto raw = bpsam_->calculateEstimate();
    cbs::AgentId reference = robot_id_;
    for (size_t i = 0; i < num_robots_; ++i) {
      const auto candidate = static_cast<cbs::AgentId>('a' + i);
      if (raw.exists(cbs::toRobotKey(candidate))) {
        reference = candidate;
        break;
      }
    }
    bool initialized = false;
    const auto values = estimateInWorldPose3(raw, initialized, reference);
    if (!initialized) throw std::runtime_error("Missing local CBS anchor");
    cbs_ros::msg::Estimate message;
    message.header.stamp = node_->now();
    message.header.frame_id = "cbs/robot" + std::to_string(reference - 'a');
    message.robot_id = static_cast<uint8_t>(robot_id_ - 'a');
    message.reference_robot_id = static_cast<uint8_t>(reference - 'a');
    message.reference_available = raw.exists(cbs::toRobotKey('a'));
    message.finished = optimization_finished_;
    message.iteration = iteration_;
    message.graph_revision = applied_revision_;
    for (const auto& [key, value] : values) {
      if (!cbs::isPoseKey(key) || cbs::getAgentId(key) != robot_id_) continue;
      const auto stamp = timestamps_.find(key);
      if (stamp == timestamps_.end()) {
        throw std::runtime_error("Own keyframe is missing its timestamp");
      }
      const auto pose = value.template cast<gtsam::Pose3>();
      const auto q = pose.rotation().toQuaternion();
      geometry_msgs::msg::Pose out;
      out.position.x = pose.x();
      out.position.y = pose.y();
      out.position.z = pose.z();
      out.orientation.x = q.x();
      out.orientation.y = q.y();
      out.orientation.z = q.z();
      out.orientation.w = q.w();
      message.keyframe_ids.push_back(cbs::getPoseIndex(key));
      message.stamp_ns.push_back(stamp->second);
      message.poses.push_back(out);
    }
    estimate_publisher_->publish(message);
  }

  void publishStats(size_t num_new_factors, size_t num_removed_factors,
                    size_t num_new_values, size_t num_received_beliefs,
                    size_t num_accepted_beliefs) {
    const auto estimate = bpsam_->calculateEstimate();
    const double residual = bpsam_->getFactorsUnsafe().error(estimate);

    double residual_signed_change = 0.0;
    double residual_abs_change = 0.0;
    double residual_rel_change = 0.0;
    double result_logmap_change = 0.0;
    uint64_t num_changed_values = 0;
    if (has_previous_stats_) {
      residual_signed_change = residual - previous_residual_;
      residual_abs_change = std::abs(residual_signed_change);
      residual_rel_change = residual_abs_change /
                            std::max(std::abs(previous_residual_),
                                     std::numeric_limits<double>::epsilon());
      result_logmap_change =
          resultLogmapChange(previous_estimate_, estimate, num_changed_values);
    }

    cbs_ros::msg::NodeStats message;
    message.header.stamp = node_->now();
    message.header.frame_id = robot_name_;
    message.robot_id = static_cast<uint8_t>(robot_id_ - 'a');
    message.robot_name = robot_name_;
    message.iteration = iteration_;
    message.belief_stage = beliefSharingStageName(belief_sharing_stage_);
    message.residual = residual;
    message.residual_signed_change = residual_signed_change;
    message.residual_abs_change = residual_abs_change;
    message.residual_rel_change = residual_rel_change;
    message.result_logmap_change = result_logmap_change;
    message.meaningful_update = num_new_factors > 0 ||
                                num_removed_factors > 0 || num_new_values > 0 ||
                                num_accepted_beliefs > 0;
    message.num_new_factors = num_new_factors;
    message.num_removed_factors = num_removed_factors;
    message.num_new_values = num_new_values;
    message.num_received_beliefs = num_received_beliefs;
    message.num_accepted_beliefs = num_accepted_beliefs;
    message.num_values = estimate.size();
    message.num_changed_values = num_changed_values;
    message.bandwidth_sent_bytes = bandwidth_sent_bytes_;
    message.bandwidth_recv_bytes = bandwidth_recv_bytes_;
    stats_publisher_->publish(message);

    const std::string metrics_entity = "/cbs/metrics/" + robot_name_;
    if (viz_) {
      viz_->drawScalar(metrics_entity + "/residual", residual);
      viz_->drawScalar(metrics_entity + "/residual_abs_change",
                       residual_abs_change);
      viz_->drawScalar(metrics_entity + "/residual_rel_change",
                       residual_rel_change);
      viz_->drawScalar(metrics_entity + "/result_logmap_change",
                       result_logmap_change);
      viz_->drawScalar(metrics_entity + "/meaningful_update",
                       message.meaningful_update ? 1.0 : 0.0);
    }

    previous_estimate_ = estimate;
    previous_residual_ = residual;
    has_previous_stats_ = true;
  }

  void visualize(bool force_save = false) {
    if (!viz_) return;
    viz_->setTime();

    auto agent_color = aria::viz::AgentColorMap::get(bpsam_->id());
    const std::string map_entity = "/cbs/map/" + robot_name_;
    const std::string metrics_entity = "/cbs/metrics/" + robot_name_;
    const auto raw_estimate = bpsam_->calculateEstimate();
    bool initialized = false;
    const auto pose_estimate = estimateInWorldPose3(raw_estimate, initialized);
    const auto pose_graph =
        convertGraphToPose3<Value>(bpsam_->getFactorsUnsafe());

    std::vector<Point3> traj_points;
    std::vector<std::string> labels;
    for (auto const &[key, value] : pose_estimate) {
      if (cbs::isPoseKey(key) and cbs::getAgentId(key) == robot_id_) {
        traj_points.push_back(value.template cast<Pose3>().translation());
        labels.push_back(MultiRobotKeyFormatter(key));
      }
    }

    viz_->drawPoints(map_entity + "/trajectory", traj_points, {agent_color},
                     {1.0f}, labels);
    viz_->drawScalar(metrics_entity + "/num_poses", traj_points.size());
    viz_->drawScalar(metrics_entity + "/belief_stage",
                     beliefSharingStageValue(belief_sharing_stage_));

    viz_->drawFactors(map_entity + "/graph", pose_graph, pose_estimate,
                      agent_color, 1.0f, true, false,
                      gtsam::MultiRobotKeyFormatter);

    size_t num_odom_factors{0}, num_loop_factors{0};
    for (const auto &factor : bpsam_->getFactorsUnsafe()) {
      if (!factor)
        continue;
      if (factor->keys().size() != 2)
        continue;

      gtsam::Key key0 = factor->keys()[0];
      gtsam::Key key1 = factor->keys()[1];
      if (cbs::getAgentId(key0) == robot_id_ and
          cbs::getAgentId(key1) == robot_id_) {
        if (cbs::isPoseKey(key0) and cbs::isPoseKey(key1)) {
          num_odom_factors++;
        }
      } else {
        num_loop_factors++;
      }
    }

    viz_->drawScalar(metrics_entity + "/num_factors",
                     bpsam_->getFactorsUnsafe().size());
    viz_->drawScalar(metrics_entity + "/num_odom_factors", num_odom_factors);
    viz_->drawScalar(metrics_entity + "/num_loop_factors", num_loop_factors);
    viz_->drawScalar(metrics_entity + "/bandwidth_sent_bytes",
                     static_cast<double>(bandwidth_sent_bytes_));
    viz_->drawScalar(metrics_entity + "/bandwidth_recv_bytes",
                     static_cast<double>(bandwidth_recv_bytes_));

    // Values are composed in their native state space before Sim3 is
    // projected to Pose3. This is intentionally identity in the viewer.
    const Pose3 T_map_agent;
    viz_->drawTf(map_entity, T_map_agent);

    auto divergence = bpsam_->getDivergences();
    std::map<gtsam::Key, double> cur_max_div;
    std::map<gtsam::Key, double> tv_per_key;

    {
      const auto &estimate = pose_estimate;
      constexpr float kMaxRadius = 5.0f;
      constexpr float kMinRadius = 1.0f;
      constexpr double kLogScale = 100.0;
      auto logscale = [](double v) {
        return std::log1p(v * kLogScale) / std::log1p(kLogScale);
      };

      // --- current divergence: max across agents per key ---
      for (auto const &[agent_id, key_div_map] : divergence) {
        for (auto const &[key, dxy] : key_div_map) {
          cur_max_div[key] = std::max(cur_max_div[key], dxy);
        }
      }

      // maintain rolling history (max 10 iterations)
      divergence_history_.push_back(cur_max_div);
      if (divergence_history_.size() > 20)
        divergence_history_.pop_front();

      // compute total variation per key over history: sum_k |D^(k) - D^(k-1)|
      for (auto const &[key, _] : cur_max_div) {
        double tv = 0.0;
        gtsam::Key k = key;
        for (size_t i = 1; i < divergence_history_.size(); ++i) {
          auto get = [&](size_t idx) {
            auto it = divergence_history_[idx].find(k);
            return (it != divergence_history_[idx].end()) ? it->second : 0.0;
          };
          tv += std::abs(get(i) - get(i - 1));
        }
        tv_per_key[key] = tv;
      }

      // helper: normalize values, size and color by normalized divergence
      auto drawNormalizedBalls =
          [&](const std::string &entity_path,
              const std::map<gtsam::Key, double> &key_vals, auto label_fn) {
            std::vector<std::pair<Point3, double>> pts_vals;
            for (auto const &[key, val] : key_vals) {
              if (val <= 0.0)
                continue;
              if (!estimate.exists(key))
                continue;
              auto pt = aria::viz::Visualizer::getPoint3(key, estimate);
              if (!pt)
                continue;
              pts_vals.emplace_back(T_map_agent * (*pt), logscale(val));
            }
            if (pts_vals.empty())
              return;

            double vmax = std::max_element(pts_vals.begin(), pts_vals.end(),
                                           [](auto &a, auto &b) {
                                             return a.second < b.second;
                                           })
                              ->second;
            double vmin = std::min_element(pts_vals.begin(), pts_vals.end(),
                                           [](auto &a, auto &b) {
                                             return a.second < b.second;
                                           })
                              ->second;
            double vrange = vmax - vmin;

            std::vector<Point3> points;
            std::vector<float> radii;
            std::vector<Eigen::Vector4f> colors;
            std::vector<std::string> labels;
            for (auto const &[pt, val] : pts_vals) {
              float n = (vrange > 0.0)
                            ? static_cast<float>((val - vmin) / vrange)
                            : 1.f;
              points.push_back(pt);
              radii.push_back(kMinRadius + n * (kMaxRadius - kMinRadius));
              colors.push_back(
                  Eigen::Vector4f(n * 255.f, (1.f - n) * 255.f, 0.f, 180.f));
              labels.push_back(label_fn(val));
            }
            viz_->drawPoints(entity_path, points, colors, radii, labels);
          };

      drawNormalizedBalls(map_entity + "/divergence", cur_max_div,
                          [](double v) { return fmt::format("{:.3f}", v); });
      drawNormalizedBalls(map_entity + "/divergence_tv", tv_per_key,
                          [](double v) { return fmt::format("tv:{:.4f}", v); });

      // scalars: per-agent current divergence + overall TV
      for (auto const &[agent_id, key_div_map] : divergence) {
        if (key_div_map.empty())
          continue;
        double sum = 0.0, max_d = 0.0;
        for (auto const &[key, dxy] : key_div_map) {
          sum += dxy;
          max_d = std::max(max_d, dxy);
        }
        std::string prefix =
            metrics_entity + "/divergence/" + std::string(1, agent_id);
        viz_->drawScalar(prefix + "/mean", logscale(sum / key_div_map.size()));
        viz_->drawScalar(prefix + "/max", logscale(max_d));
      }

      if (!tv_per_key.empty()) {
        double tv_sum = 0.0, tv_max = 0.0;
        for (auto const &[key, tv] : tv_per_key) {
          tv_sum += tv;
          tv_max = std::max(tv_max, tv);
        }
        viz_->drawScalar(metrics_entity + "/divergence_tv/mean",
                         logscale(tv_sum / tv_per_key.size()));
        viz_->drawScalar(metrics_entity + "/divergence_tv/max",
                         logscale(tv_max));
      }
    }

    if (log_dir_.empty()) {
      return;
    }

    if (csv_file_.is_open()) {
      double residual =
          bpsam_->getFactorsUnsafe().error(bpsam_->calculateEstimate());
      csv_file_ << iteration_ << "," << residual << "," << bandwidth_sent_bytes_
                << "," << bandwidth_recv_bytes_ << "\n";
      csv_file_.flush();
    }

    // Save trajectory and g2o files every 10 seconds online, matching the
    // ROS1 behavior of saving more frequently in offline sweeps.
    std::time_t current_time = std::time(nullptr);
    const int save_interval = online_ ? 10 : 1;
    if (current_time - last_save_time_ >= save_interval || force_save) {
      last_save_time_ = current_time;
      std::string timestamp = std::to_string(current_time);

      if (initialized) {
        saveTrajectoryInWorldTUM(pose_estimate, timestamp);
        const auto graph_path =
            log_dir_ / ("cbs_robot_" +
                        std::to_string(int(robot_id_ - 'a')) + "_" +
                        timestamp + ".g2o");
        bpsam_->saveLocalGraphG2o(graph_path, true);
        saveLoopScaleSidecar(graph_path);
        spdlog::info(
            "Saved local graph with {} factors and {} values to {}",
            bpsam_->getFactorsUnsafe().size(),
            bpsam_->calculateEstimate().size(),
            (log_dir_ / ("cbs_robot_" + std::to_string(int(robot_id_ - 'a')) +
                         "_" + timestamp + ".g2o"))
                .string());
      } else {
        spdlog::warn("Not saving trajectory and graph because the "
                     "transformation to world frame is not initialized yet.");
      }
      saveDivergenceTxt(divergence, timestamp);
      saveDivergenceTVTxt(tv_per_key, timestamp);
    }
  }

  void saveTrajectoryInWorldTUM(const gtsam::Values &values,
                                const std::string &timestamp) {
    std::map<std::time_t, gtsam::Pose3> trajectory_in_world;
    for (auto const &[key, value] : values) {
      if (cbs::getAgentId(key) == robot_id_ and cbs::isPoseKey(key)) {
        const gtsam::Pose3 T_world_pose = value.template cast<gtsam::Pose3>();
        const auto timestamp_it = timestamps_.find(key);
        if (timestamp_it != timestamps_.end()) {
          trajectory_in_world[timestamp_it->second] = T_world_pose;
        }
      }
    }

    // save the trajectory in world frame to a file in TUM format
    std::string filename = "Robot " + std::to_string(int(robot_id_ - 'a')) +
                           "_" + timestamp + ".tum";

    spdlog::info("Saved trajectory size in world frame: {}, to {}",
                 trajectory_in_world.size(), (log_dir_ / filename).string());

    std::ofstream file(log_dir_ / filename);
    if (file.is_open()) {
      file << std::fixed << std::setprecision(6);
      for (const auto &[timestamp, pose] : trajectory_in_world) {
        // format: timestamp x y z q_x q_y q_z q_w
        file << std::setprecision(6) << (double)timestamp / 1e9 << " "
             << std::setprecision(9) << pose.x() << " " << pose.y() << " "
             << pose.z() << " " << pose.rotation().toQuaternion().x() << " "
             << pose.rotation().toQuaternion().y() << " "
             << pose.rotation().toQuaternion().z() << " "
             << pose.rotation().toQuaternion().w() << "\n";
      }
      file.close();
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unable to open file %s",
                  filename.c_str());
    }
  }

  void saveLoopScaleSidecar(const std::filesystem::path &graph_path) const {
    std::filesystem::path sidecar = graph_path;
    sidecar.replace_extension(".scale.csv");
    std::ofstream output(sidecar, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("Unable to write Sim3 loop sidecar: " +
                               sidecar.string());
    }
    output << std::setprecision(17);
    output << "robot_from,key_from,robot_to,key_to,has_scale,scale,"
              "scale_sigma\n";
    for (const auto &[keys, metadata] : loop_scale_metadata_) {
      output << static_cast<int>(cbs::getAgentId(keys.first) - 'a') << ','
             << cbs::getPoseIndex(keys.first) << ','
             << static_cast<int>(cbs::getAgentId(keys.second) - 'a') << ','
             << cbs::getPoseIndex(keys.second) << ',' << metadata.has_scale
             << ',' << metadata.scale << ',' << metadata.scale_sigma << '\n';
    }
    if (!output.good()) {
      throw std::runtime_error("Failed while writing Sim3 loop sidecar: " +
                               sidecar.string());
    }
  }

  void saveDivergenceTxt(
      const std::map<cbs::AgentId, std::map<gtsam::Key, double>> &divergence,
      const std::string &timestamp) {
    std::string filename = "divergence_robot_" +
                           std::to_string(int(robot_id_ - 'a')) + "_" +
                           timestamp + ".txt";
    std::ofstream file(log_dir_ / filename);
    if (!file.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Unable to open file %s",
                  filename.c_str());
      return;
    }

    file << std::fixed << std::setprecision(9);
    file << "# agent_id key divergence\n";
    for (const auto &[agent_id, key_div_map] : divergence) {
      for (const auto &[key, value] : key_div_map) {
        file << agent_id << " " << MultiRobotKeyFormatter(key) << " " << value
             << "\n";
      }
    }
  }

  void saveDivergenceTVTxt(const std::map<gtsam::Key, double> &tv_per_key,
                           const std::string &timestamp) {
    std::string filename = "divergence_tv_robot_" +
                           std::to_string(int(robot_id_ - 'a')) + "_" +
                           timestamp + ".txt";
    std::ofstream file(log_dir_ / filename);
    if (!file.is_open()) {
      RCLCPP_WARN(node_->get_logger(), "Unable to open file %s",
                  filename.c_str());
      return;
    }

    file << std::fixed << std::setprecision(9);
    file << "# key divergence_tv\n";
    for (const auto &[key, value] : tv_per_key) {
      file << MultiRobotKeyFormatter(key) << " " << value << "\n";
    }
  }

private:
  template <typename T>
  T declareParameter(const std::string &name, const T &default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    T value{};
    node_->get_parameter(name, value);
    return value;
  }

  template <typename T> bool getParameter(const std::string &name, T &value) {
    if (!node_->has_parameter(name)) {
      return false;
    }
    return node_->get_parameter(name, value);
  }

  gtsam::Values estimateAsPose3(const gtsam::Values &values) const {
    gtsam::Values converted;
    for (const auto &[key, value] : values) {
      if (cbs::isPoseKey(key) || cbs::isRobotKey(key)) {
        converted.insert_or_assign(
            key, valueToPose3<Value>(value.template cast<Value>()));
      }
    }
    return converted;
  }

  gtsam::Values estimateInWorldPose3(const gtsam::Values &values,
                                     bool &initialized,
                                     cbs::AgentId reference = 'a') const {
    const auto projected = estimateAsPose3(values);
    const gtsam::Key reference_anchor = cbs::toRobotKey(reference);
    initialized = projected.exists(reference_anchor);
    const Pose3 world_from_agent =
        initialized ? projected.template at<Pose3>(reference_anchor).inverse()
                    : Pose3::Identity();

    gtsam::Values aligned;
    for (const auto &[key, value] : projected) {
      if (cbs::isPoseKey(key) || cbs::isRobotKey(key)) {
        aligned.insert_or_assign(key, world_from_agent *
                                          value.template cast<Pose3>());
      } else {
        aligned.insert_or_assign(key, value);
      }
    }
    return aligned;
  }

  bool persistent_{false};
  double pose_stage_probability_{0.5};
  uint64_t applied_revision_{0};
  bool persistent_started_{false}, persistent_staged_{false};
  std::string persistent_session_;
  cbs_ros::msg::PersistentInput::ConstSharedPtr persistent_input_;
  rclcpp::Subscription<cbs_ros::msg::PersistentInput>::SharedPtr persistent_subscription_;
  PoseGraphMsg persistent_graph_;
  std::map<gtsam::Key, pose_graph_tools_msgs::msg::PoseGraphNode> immutable_nodes_;
  struct PendingRequest {
    uint64_t token;
    int64_t request_id;
    std::chrono::steady_clock::time_point started;
  };
  uint64_t request_token_{0};
  std::map<cbs::AgentId, PendingRequest> pending_requests_;

  rclcpp::Node::SharedPtr node_;
  cbs::AgentId robot_id_;
  std::string robot_name_;
  std::shared_ptr<BPSAM> bpsam_;
  std::unique_ptr<cbs_ros::DistributedPcm> pcm_;
  bool pcm_applied_{false};
  bool registration_enabled_{false};
  bool registration_gpu_{false};
#ifdef CBS_ROS_WITH_REGISTRATION_FACTORS
  std::unique_ptr<cbs_ros::RegistrationFactors> registration_;
  std::unique_ptr<tbb::global_control> registration_threads_;
  bool registration_added_{false};
#endif
  std::shared_ptr<aria::viz::VisualizerRerun> viz_;

  std::vector<rclcpp::Subscription<PoseGraphMsg>::SharedPtr>
      pose_graph_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      belief_stage_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      topology_subscription_;
  rclcpp::Service<KeysSrv>::SharedPtr request_beliefs_srv_;
  rclcpp::Service<OptimizationStepSrv>::SharedPtr optimization_step_srv_;
  rclcpp::Publisher<cbs_ros::msg::NodeStats>::SharedPtr stats_publisher_;
  rclcpp::Publisher<cbs_ros::msg::Estimate>::SharedPtr estimate_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr input_complete_subscription_;
  int settle_iterations_{0};
  double belief_request_timeout_sec_{60.0};
  std::optional<size_t> input_complete_iteration_;
  std::vector<rclcpp::Client<KeysSrv>::SharedPtr> neighbor_clients_;
  std::vector<cbs::AgentId> neighbor_client_ids_;
  std::set<cbs::AgentId> neighbor_ids_;
  std::set<cbs::AgentId> observed_neighbor_ids_;
  rclcpp::CallbackGroup::SharedPtr pose_graph_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  size_t num_robots_;
  rclcpp::TimerBase::SharedPtr timer_;

  int max_iterations_{-1};
  std::map<std::pair<gtsam::Key, gtsam::Key>, NonlinearFactor::shared_ptr>
      existing_factors_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, LoopScaleMetadata>
      loop_scale_metadata_;
  std::set<NonlinearFactor::shared_ptr> factors_to_remove_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, NonlinearFactor::shared_ptr>
      factors_to_add_;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> factor_add_order_;
  bool initial_pose_graph_received_{false};

  std::map<Key, int64_t> timestamps_;

  std::mutex mutex_;

  gtsam::Values accumulated_values_;
  gtsam::Values previous_estimate_;
  double previous_residual_{0.0};
  bool has_previous_stats_{false};

  // per-key max divergence history (across agents), capped at 10 iterations
  std::deque<std::map<gtsam::Key, double>> divergence_history_;

  size_t iteration_{0};
  bool synchronize_optimization_rounds_{false};
  bool optimization_initialized_{false};
  bool optimization_finished_{false};
  size_t next_synchronized_round_{0};
  std::optional<size_t> exchanged_synchronized_round_;
  size_t bandwidth_sent_bytes_{0};
  size_t bandwidth_recv_bytes_{0};
  size_t pending_new_values_since_update_{0};
  size_t beliefs_received_since_update_{0};
  size_t beliefs_accepted_since_update_{0};
  std::filesystem::path log_dir_;
  std::ofstream csv_file_;
  std::ofstream timing_csv_file_;
  std::ofstream belief_service_timing_csv_file_;
  size_t belief_service_request_count_{0};
  std::time_t last_save_time_{0};
  bool online_{true};
  bool use_robust_noise_models_{true};
  double sim3_scale_sigma_{0.1};
  double sim3_odom_scale_sigma_{-1.0};
  double sim3_loop_scale_sigma_{-1.0};
  double sim3_inter_loop_scale_sigma_{-1.0};
  double sim3_pose_scale_prior_sigma_{-1.0};
  CommunicationTopologyMode communication_topology_mode_{
      CommunicationTopologyMode::DynamicFactors};
  bool topology_received_{false};
  double topology_wait_timeout_sec_{10.0};
  std::chrono::steady_clock::time_point topology_wait_started_{
      std::chrono::steady_clock::now()};
  BeliefStageSwitchStrategy belief_stage_switch_strategy_{
      BeliefStageSwitchStrategy::FixedIteration};
  int belief_stage_fixed_iterations_{10};
  BeliefSharingStage belief_sharing_stage_{BeliefSharingStage::Pose};
  std::mt19937 belief_stage_rng_{std::random_device{}()};
  std::string belief_stage_topic_{"/cbs_ros/belief_stage"};
  std::string stats_topic_;
  double belief_republish_hellinger_threshold_{0.01};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("cbs_ros_node", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  if (!node->has_parameter("pgo_formulation")) {
    node->declare_parameter<std::string>("pgo_formulation", "sim3");
  }
  const auto pgo_formulation =
      node->get_parameter("pgo_formulation").as_string();
  if (pgo_formulation == "pose3") {
    CBSRosNode<gtsam::Pose3> cbs_node(node);
    executor.spin();
  } else if (pgo_formulation == "sim3") {
    CBSRosNode<gtsam::Similarity3> cbs_node(node);
    executor.spin();
  } else {
    throw std::invalid_argument("pgo_formulation must be pose3 or sim3");
  }
  rclcpp::shutdown();
  return 0;
}

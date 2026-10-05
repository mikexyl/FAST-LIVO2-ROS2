#include "cbs/bpsam/bpsam.h"

#include <aria_common/benchmark.h>
#include <cbs/gbp/contraction/augmented_hellinger.h>
#include <cbs/gbp/contraction/hellinger.h>
#include <cbs/gbp/contraction/kl_divergence.h>
#include <cbs/gbp/factor.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/dataset.h>
#include <tbb/parallel_for_each.h>

#include <optional>
#include <type_traits>

#include "cbs/key.h"
#include "cbs/utils/gtsam_ptr.h"
#include "cbs/utils/between_initialization.h"

namespace cbs {

template <class VALUE>
BPSAMT<VALUE>::BPSAMT(const Params& params)
    : BaseSAM(params.sam_params_), params_(params) {
  if (params_.enable_gkcm) {
    throw std::invalid_argument(
        "GkCM/PCM is not supported by this CBS branch.");
  }

  switch (params_.gbp_update_params.metric_type) {
    case gbp::MetricType::AugmentedHellinger:
      belief_metric_ =
          std::make_unique<gbp::AugmentedHellinger>(params_.gbp_update_params);
      break;
    case gbp::MetricType::KLDivergence:
      belief_metric_ =
          std::make_unique<gbp::KLDivergence>(params_.gbp_update_params);
      break;
    case gbp::MetricType::Hellinger:
      belief_metric_ =
          std::make_unique<gbp::Hellinger>(params_.gbp_update_params);
      break;
    default:
      throw std::invalid_argument(
          "The requested CBS belief metric is not implemented; no fallback "
          "metric will be selected.");
  }
}

template <class VALUE>
int BPSAMT<VALUE>::addBeliefs(
    std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs) {
  int num_rejected = 0;
  for (auto& [key, agent_id_gaussians] : beliefs) {
    for (auto it = agent_id_gaussians.begin(); it != agent_id_gaussians.end();) {
      auto const& [agent_id, gaussian] = *it;
      bool is_first_message = addGBPNode(agent_id, key, gaussian);
      auto params = params_.gbp_update_params;
      // Persistent replies were damped by their server. Applying contraction
      // here again would implement a different algorithm.
      if (params_.persistent) params.type = gbp::GaussianMergeType::Replace;
      auto result = gbp_graph_[agent_id]
          .template getFactor<gbp::PriorFactor<VALUE>>(key)->update(gaussian, params);
      if (params_.gbp_update_params.enable_soft_reset)
        CHECK(result.status.at(0) == gbp::UpdateResult::Success);
      if (result.status.at(0) != gbp::UpdateResult::Success ||
          (is_first_message && !params_.persistent)) {
        it = agent_id_gaussians.erase(it);
        ++num_rejected;
      } else {
        ++it;
      }
    }
  }

  spdlog::debug("BPSAM {}: Added {} beliefs, rejected {}",
                params_.robot_id,
                beliefs.size() - num_rejected,
                num_rejected);

  for (const auto& [key, agent_id_gaussians] : beliefs) {
    for (const auto& [agent_id, gaussian] : agent_id_gaussians) {
      auto robot_key = toRobotKey(agent_id);
      new_gbp_keys_.emplace_back(robot_key, key);
    }
  }

  std::stringstream gbp_keys_ss;
  for (const auto& [robot_key, consensus_key] : new_gbp_keys_) {
    gbp_keys_ss << MultiRobotKeyFormatter(robot_key) << " "
                << MultiRobotKeyFormatter(consensus_key) << " ";
  }
  spdlog::debug(
      "BPSAM {}: Added new gbp keys: {}", params_.robot_id, gbp_keys_ss.str());

  return num_rejected;
}

template <class VALUE>
NonlinearFactor::shared_ptr BPSAMT<VALUE>::beliefToFactor(Key key0,
                                                          Key key1,
                                                          gbp::Gaussian belief,
                                                          bool robust) const {
  auto value0 = traits<VALUE>::Expmap(belief.mu());
  auto sigma = belief.Sigma();

  if (params_.enable_belief_dcs) {
    double omega = belief.relax_factor();  // in [0,1]

    // Tunable parameters
    static constexpr double kScaleAlpha =
        3.0;  // multiplicative inflation strength

    // Multiplicative scaling (preserve covariance structure)
    double scale = 1.0 + kScaleAlpha * omega;
    sigma *= scale;
    double beta_t = 1 * 1;                       // 5 cm std
    double beta_r = pow(5.0 * M_PI / 180.0, 2);  // 5 deg std
    double beta_s = pow(0.10, 2);                // scale log std floor
    // Additive floor to prevent overconfident factors
    Eigen::MatrixXd floor = Eigen::MatrixXd::Zero(sigma.rows(), sigma.cols());
    if (sigma.rows() >= 3 && sigma.cols() >= 3) {
      floor.block<3, 3>(0, 0) = beta_t * omega * Eigen::Matrix3d::Identity();
    }
    if (sigma.rows() >= 6 && sigma.cols() >= 6) {
      floor.block<3, 3>(3, 3) = beta_r * omega * Eigen::Matrix3d::Identity();
    }
    if (sigma.rows() > 6 && sigma.cols() > 6) {
      floor(6, 6) = beta_s * omega;
    }
    sigma += floor;
  }

  auto noise = noiseModel::Gaussian::Covariance(sigma);
  if (robust) {
    auto robust_noise = noiseModel::Robust::Create(  //
        noiseModel::mEstimator::Huber::Create(1.345),
        noise);
    return cbs::make_shared<BetweenFactor<VALUE>>(
        key0, key1, value0, robust_noise);
  } else {
    return cbs::make_shared<BetweenFactor<VALUE>>(key0, key1, value0, noise);
  }
}

template <class VALUE>
std::map<Key, std::map<AgentId, gbp::Gaussian>> BPSAMT<VALUE>::getBeliefs(
    KeySet keys,
    bool use_other_robots) const {
  std::map<Key, std::map<AgentId, gbp::Gaussian>> beliefs;

  keys.erase(toRobotKey(params_.robot_id));

  // print all robot key beliefs in gbp_graph
  std::stringstream robot_key_ss;
  for (auto const& [agent_id, gbp_graph] : gbp_graph_) {
    for (auto const& [key, belief] : gbp_graph.vars()) {
      if (isRobotKey(LabeledSymbol(key))) {
        robot_key_ss << agent_id << ":" << MultiRobotKeyFormatter(key) << " ";
      }
    }
  }
  spdlog::debug(
      "BPSAM {}: Robot keys in gbp_graph: {}", id(), robot_key_ss.str());

  std::stringstream keys_ss;
  keys_ss << "Requested keys: ";
  for (const auto& key : keys) {
    keys_ss << MultiRobotKeyFormatter(key) << " ";
  }
  spdlog::debug("BPSAM {}: {}", id(), keys_ss.str());

  if (use_other_robots) {
    std::stringstream ss;
    // find robot keys from the keys
    for (const auto& key : keys) {
      if (!isRobotKey(LabeledSymbol(key))) continue;
      // try to find it in the gbp graph
      for (const auto& [agent_id, gbp_graph] : gbp_graph_) {
        if (gbp_graph.contains(key) == 0) continue;
        auto const gbp_var = gbp_graph.getVar(key);
        beliefs[key].emplace(agent_id, *gbp_var);
        ss << agent_id << ":" << MultiRobotKeyFormatter(key) << " ";
      }
    }
    spdlog::debug("BPSAM {}: relayed beliefs: {}", id(), ss.str());
  }

  // erase the keys that doesn't exist in the graph
  for (auto it = keys.begin(); it != keys.end();) {
    if (variableIndex_.find(*it) == variableIndex_.end()) {
      it = keys.erase(it);
    } else {
      ++it;
    }
  }

  std::map<Key, Matrix> marginals;
  for (auto key : keys) {
    try {
      marginals[key] = marginalCovariance(key);
    } catch (...) {
    }
  }

  for (const auto& [key, marginal] : marginals) {
    auto v = calculateEstimate<VALUE>(key);
    auto mu = traits<VALUE>::Logmap(v);
    auto belief = gbp::Gaussian(key, mu, marginal, 1);
    beliefs[key].emplace(params_.robot_id, belief);
  }

  std::stringstream belief_ss;
  for (const auto& [key, aid_belief] : beliefs) {
    for (auto [agent, gauss] : aid_belief) {
      belief_ss << agent << ":" << MultiRobotKeyFormatter(key) << " ";
    }
  }
  spdlog::debug("BPSAM {}: returning beliefs: {}", id(), belief_ss.str());

  return beliefs;
}

template <class VALUE>
std::map<Key, std::map<AgentId, gbp::Gaussian>> BPSAMT<VALUE>::getBeliefs(
    KeySet keys,
    AgentId requesting_robot,
    bool use_other_robots) const {
  // Collect beliefs the same way as the original overload.
  auto beliefs = getBeliefs(keys, use_other_robots);

  const double threshold = params_.belief_similarity_threshold;
  auto& robot_last = last_sent_beliefs_[requesting_robot];
  auto& robot_sent_counts = sent_belief_counts_[requesting_robot];

  if (params_.persistent) {
    for (auto it = beliefs.begin(); it != beliefs.end();) {
      for (auto jt = it->second.begin(); jt != it->second.end();) {
        const auto previous = robot_last.find({it->first, jt->first});
        if (jt->first != params_.robot_id) {
          // Anchor covariances learned only through incoming anchor factors
          // are absent from the GLOBAL cavity. Forward the original producer's
          // Gaussian, as in coordinated CBS, instead of dropping that route or
          // inventing a locally independent measurement. It has already been
          // damped; do not contract or apply the divergence gate a second time.
          // The receive-side canonical/moment conversion can introduce roundoff.
          const bool unchanged = previous != robot_last.end() &&
              previous->second.mu().isApprox(jt->second.mu(), 1e-12) &&
              previous->second.Sigma().isApprox(jt->second.Sigma(), 1e-12);
          if (jt->first == requesting_robot || unchanged) {
            jt = it->second.erase(jt);
          } else {
            ++jt;
          }
          continue;
        }
        auto filtered = params_.persistent_belief(
            jt->second, previous == robot_last.end() ? nullptr : &previous->second);
        if (!filtered) jt = it->second.erase(jt);
        else { jt->second = *filtered; ++jt; }
      }
      if (it->second.empty()) it = beliefs.erase(it);
      else ++it;
    }
  } else if (threshold > 0.0) {
    size_t n_filtered = 0;
    for (auto it = beliefs.begin(); it != beliefs.end();) {
      const Key& belief_key = it->first;
      auto& aid_belief = it->second;

      for (auto jt = aid_belief.begin(); jt != aid_belief.end();) {
        const AgentId producing_agent = jt->first;
        const gbp::Gaussian& curr = jt->second;

        auto last_it = robot_last.find({belief_key, producing_agent});
        const auto sent_count_it =
            robot_sent_counts.find({belief_key, producing_agent});
        const size_t sent_count = sent_count_it == robot_sent_counts.end()
                                      ? 0u
                                      : sent_count_it->second;
        if (last_it != robot_last.end() && sent_count >= 2u) {
          const gbp::Gaussian& last = last_it->second;
          double h = belief_metric_->distance(last, curr);

          if (h < threshold) {
            spdlog::debug(
                "BPSAM {}: Filtering belief for key {} (producer {}) "
                "sent to robot {} — metric distance {:.4f} < threshold {:.4f}",
                id(),
                MultiRobotKeyFormatter(belief_key),
                producing_agent,
                requesting_robot,
                h,
                threshold);
            jt = aid_belief.erase(jt);
            n_filtered++;
            continue;
          }
        }
        ++jt;
      }

      if (aid_belief.empty()) {
        it = beliefs.erase(it);
      } else {
        ++it;
      }
    }

    spdlog::debug("BPSAM {}: getBeliefs(robot={}) filtered {} beliefs",
                  id(),
                  requesting_robot,
                  n_filtered);
  }

  // Update bookkeeping with whatever is actually being returned.
  for (const auto& [belief_key, aid_belief] : beliefs) {
    for (const auto& [producing_agent, gauss] : aid_belief) {
      auto [it, inserted] = robot_last.emplace(
          std::make_pair(belief_key, producing_agent), gauss);
      if (!inserted) {
        it->second.replace(gauss);
      }
      auto& sent_count =
          robot_sent_counts[std::make_pair(belief_key, producing_agent)];
      sent_count = std::min<size_t>(sent_count + 1u, 2u);
    }
  }

  return beliefs;
}

template <class VALUE>
bool BPSAMT<VALUE>::existsInGbpGraph(Symbol symbol) const {
  for (const auto& [agent_id, gbp_graph] : gbp_graph_) {
    if (gbp_graph.contains(symbol)) {
      return true;
    }
  }

  return false;
}

template <class VALUE>
KeySet BPSAMT<VALUE>::allKeys() const {
  KeySet keys;
  for (const auto& [key, _] : variableIndex_) {
    keys.insert(key);
  }
  for (const auto& [agent_id, gbp_graph] : gbp_graph_) {
    keys.insert(toRobotKey(agent_id));
    for (const auto& [key, _] : gbp_graph.vars()) {
      keys.insert(key);
    }
  }

  return keys;
}

template <class VALUE>
Values BPSAMT<VALUE>::getInvolvedValues(const Values& values,
                                        const KeySet& keys) {
  Values involved_values;
  for (const auto& key : keys) {
    if (values.exists(key)) {
      involved_values.insert(key, values.at(key));
    }
  }
  return involved_values;
}

template <class VALUE>
bool BPSAMT<VALUE>::addGBPNode(AgentId agent_id, Key key, gbp::Gaussian prior) {
  auto var_key = key;
  Values new_gbp_values;
  bool is_robot_key = isRobotKey(LabeledSymbol(key));
  Key publishing_agent_key = toRobotKey(agent_id);

  bool is_first_message = false;
  bool add_to_sam_graph = true;
  if (is_robot_key and not valueExists(publishing_agent_key)) {
    // if it's a robot key, check if the publish key exists
    spdlog::warn(
        "BPSAM {}: Rejecting inter-robot message, key {} does not exist, {} -> "
        "{}",
        params_.robot_id,
        MultiRobotKeyFormatter(publishing_agent_key),
        MultiRobotKeyFormatter(publishing_agent_key),
        MultiRobotKeyFormatter(var_key));
    add_to_sam_graph = false;
    is_first_message = true;
    // return false;
  }

  auto T_l_p = valueExists(key) ? calculateEstimate<VALUE>(key)
                                : traits<VALUE>::Identity();
  auto T_r_p = traits<VALUE>::Expmap(prior.mu());
  auto T_l_r = T_l_p * T_r_p.inverse();

  if (is_robot_key and not valueExists(key) and
      valueExists(publishing_agent_key)) {
    auto T_l_r = calculateEstimate<VALUE>(publishing_agent_key);
    auto T_l_robot = T_l_r * T_r_p;
    T_l_p = T_l_robot;
    new_gbp_values.insert_or_assign(key, T_l_robot);
  }

  if (gbp_graph_.find(agent_id) == gbp_graph_.end()) {
    gbp_graph_[agent_id] = gbp::Graph();

    if (not valueExists(publishing_agent_key)) {
      new_gbp_values.insert_or_assign(publishing_agent_key, T_l_r);
      gbp_values_.insert_or_assign(new_gbp_values);

      spdlog::debug("BPSAM {}: Inserting robot key: {}",
                    params_.robot_id,
                    MultiRobotKeyFormatter(publishing_agent_key));
    }
    if (not add_to_sam_graph) {
      spdlog::debug("BPSAM {}: Cached robot key: {}",
                    params_.robot_id,
                    MultiRobotKeyFormatter(publishing_agent_key));
      T_r_r_values_.insert_or_assign(publishing_agent_key, T_l_r);
    }
  }

  if (gbp_graph_.at(agent_id).contains(key) == 0) {
    typename gbp::Variable<VALUE>::shared_ptr prior_value(
        new gbp::Variable<VALUE>(prior));
    typename gbp::PriorFactor<VALUE>::shared_ptr prior_factor(
        new gbp::PriorFactor<VALUE>(prior));
    prior_factor->addAdjVar(prior_value);
    gbp_graph_[agent_id].add(prior_factor);

    gbp_values_.insert_or_assign(new_gbp_values);
    is_first_message = true;
  }

  return is_first_message;
}

template <class VALUE>
ISAM2Result BPSAMT<VALUE>::update(const NonlinearFactorGraph& newFactors,
                                  const Values& newTheta,
                                  const UpdateParams& update_params) {
  bool robust_belief_factor = true;

  NonlinearFactorGraph newFactorsCopy = newFactors;
  const KeySet new_factor_keys = newFactorsCopy.keys();

  Values newThetaCopy(newTheta);

  newThetaCopy.insert_or_assign(gbp_values_);

  NonlinearFactorGraph newBeliefFactors;
  std::vector<std::pair<Key, Key>> deferred_beliefs;
  FactorIndices factors_to_remove;
  if (params_.persistent) {
    std::sort(new_gbp_keys_.begin(), new_gbp_keys_.end());
    new_gbp_keys_.erase(std::unique(new_gbp_keys_.begin(), new_gbp_keys_.end()), new_gbp_keys_.end());
  }

  NonlinearFactorGraph gbp_factors;
  spdlog::debug(
      "BPSAM {}: new belief keys {}", params_.robot_id, new_gbp_keys_.size());
  for (auto [robot_key, var_key] : new_gbp_keys_) {
    auto belief = gbp_graph_[robot_key].getVar(var_key);
    auto factor =
        beliefToFactor(robot_key, var_key, *belief, robust_belief_factor);
    gbp_factors.push_back(factor);
  }

  // get factors connected to the new message of gbp
  {
    aria::Timer timer("BPSAM::update::updateBeliefFactors", params_.robot_id);
    for (auto const& factor : gbp_factors) {
      // only add the factors where the second key(which is the consensus key
      // or the local key) exists, which means the merging factor will be
      // added first and then next step the belief factor
      auto second_key = factor->keys().back();
      auto first_key = factor->keys().front();
      bool connected =
          variableIndex_.find(first_key) != variableIndex_.end() or
          variableIndex_.find(second_key) != variableIndex_.end() or
          new_factor_keys.count(first_key) or new_factor_keys.count(second_key);
      if (connected) {
        newBeliefFactors.push_back(factor);
        if (variableIndex_.find(first_key) == variableIndex_.end() and
            not valueExists(first_key) and isAnchorBeliefFactor(factor) and
            not gbp_values_.exists(first_key) &&
            (!params_.persistent || valueExists(second_key))) {
          VALUE second_pose = calculateEstimate<VALUE>(second_key);
          auto as_belief_factor =
              cbs::dynamic_pointer_cast<BetweenFactor<VALUE>>(factor);
          CHECK(as_belief_factor);
          VALUE first_pose =
              second_pose * as_belief_factor->measured().inverse();
          newThetaCopy.insert_or_assign(first_key, first_pose);
        }
      } else {
        if (params_.persistent) deferred_beliefs.emplace_back(first_key, second_key);
        spdlog::debug("BPSAM {}: Skipping factor {} -> {}, not connected",
                      params_.robot_id,
                      MultiRobotKeyFormatter(first_key),
                      MultiRobotKeyFormatter(second_key));
      }
    }
    spdlog::debug("BPSAM {}: new belief factors {}",
                  params_.robot_id,
                  newBeliefFactors.size());

    {
      aria::Timer timer("BPSAM::update::removeOldBeliefs", params_.robot_id);
      for (auto const& [agent_key, old_key] : new_gbp_keys_) {
        auto factor_indices_it = variableIndex_.find(old_key);
        if (factor_indices_it == variableIndex_.end()) {
          continue;
        }

        auto factor_indices = factor_indices_it->second;
        // remove old priors
        for (auto const& factor_index : factor_indices) {
          auto factor = this->getFactorsUnsafe().at(factor_index);
          if (factor->keys().size() == 2) {
            size_t diff_key_i = factor->keys()[0] == old_key ? 1 : 0;
            Key diff_key = factor->keys()[diff_key_i];
            if (diff_key == agent_key) {
              factors_to_remove.push_back(factor_index);
            }
          }
        }
      }
    }
  }

  // remove repeated factors to remove
  std::sort(factors_to_remove.begin(), factors_to_remove.end());
  factors_to_remove.erase(
      std::unique(factors_to_remove.begin(), factors_to_remove.end()),
      factors_to_remove.end());

  gbp_factors = NonlinearFactorGraph();
  gbp_values_.clear();
  new_gbp_keys_ = std::move(deferred_beliefs);

  NonlinearFactorGraph odom_valid_belief_factors = newBeliefFactors;

  // update
  ISAM2UpdateParams isam2_update_params =
      static_cast<ISAM2UpdateParams>(update_params);
  isam2_update_params.removeFactorIndices.insert(
      isam2_update_params.removeFactorIndices.end(),
      factors_to_remove.begin(),
      factors_to_remove.end());

  odom_valid_belief_factors += newFactorsCopy;

  if constexpr (std::is_same_v<VALUE, Similarity3>) {
    // Projected SE3 separators have no scale residual. Keep a weak gauge on
    // each remote copy so the LOCAL cavity remains nonsingular after the
    // absolute pose-scale priors are removed from the communication graph.
    constexpr double kRemoteCopyScaleGaugeSigma = 1.0;
    KeySet projected_remote_pose_keys;
    for (const auto& factor : newFactorsCopy) {
      if (!cbs::dynamic_pointer_cast<ProjectedSim3BetweenFactor>(factor)) {
        continue;
      }
      for (Key key : factor->keys()) {
        const LabeledSymbol symbol(key);
        if (isPoseKey(symbol) && getAgentId(symbol) != params_.robot_id) {
          projected_remote_pose_keys.insert(key);
        }
      }
    }
    for (Key key : projected_remote_pose_keys) {
      if (sim3_remote_copy_scale_gauge_keys_.count(key) != 0) {
        continue;
      }
      Similarity3 initial_value = Similarity3::Identity();
      if (newThetaCopy.exists(key)) {
        initial_value = newThetaCopy.at<Similarity3>(key);
      } else if (valueExists(key)) {
        initial_value = calculateEstimate<Similarity3>(key);
      }
      odom_valid_belief_factors.push_back(
          cbs::make_shared<Sim3ScalePriorFactor>(
              key,
              std::log(initial_value.scale()),
              noiseModel::Isotropic::Sigma(1, kRemoteCopyScaleGaugeSigma),
              Sim3ScalePriorRole::RemoteCopyGauge));
      sim3_remote_copy_scale_gauge_keys_.insert(key);
    }

    if (params_.sim3_pose_scale_prior_sigma > 0.0) {
      size_t num_scale_priors_added = 0;
      const auto noise =
          noiseModel::Isotropic::Sigma(1, params_.sim3_pose_scale_prior_sigma);
      auto add_pose_scale_prior = [&](Key key) {
        if (!isPoseKey(LabeledSymbol(key)) ||
            sim3_pose_scale_prior_keys_.count(key) != 0) {
          return;
        }
        odom_valid_belief_factors.push_back(
            cbs::make_shared<Sim3ScalePriorFactor>(
                key, 0.0, noise, Sim3ScalePriorRole::AbsolutePose));
        sim3_pose_scale_prior_keys_.insert(key);
        num_scale_priors_added++;
      };

      KeySet scale_prior_candidate_keys = odom_valid_belief_factors.keys();
      for (const auto& [key, value] : newThetaCopy) {
        (void)value;
        scale_prior_candidate_keys.insert(key);
      }
      for (const auto& [key, factors] : this->getVariableIndex()) {
        (void)factors;
        scale_prior_candidate_keys.insert(key);
      }
      for (Key key : scale_prior_candidate_keys) {
        add_pose_scale_prior(key);
      }
      spdlog::debug("BPSAM {}: Added {} absolute Sim3 pose scale priors",
                    params_.robot_id,
                    num_scale_priors_added);
    }
  }

  // remove keys already exist
  // FactorGraph::exists tests a factor SLOT, not a variable key. In persistent
  // mode that check discarded every supplied pose (LabeledSymbol keys are
  // much larger than the factor count), initializing the whole map at identity.
  const KeySet connected_keys = odom_valid_belief_factors.keys();
  Values filtered_new_theta;
  for (const auto& [key, value] : newThetaCopy) {
    if (not valueExists(key) &&
        (!params_.persistent || connected_keys.count(key))) {
      filtered_new_theta.insert(key, value);
    }
  }

  // Infer new remote separator guesses from already available local poses.
  // Identity is only a fallback for keys with no connected between measurement.
  if (!odom_valid_belief_factors.empty()) {
    initializeBetweenValues<VALUE>(odom_valid_belief_factors,
                                   calculateEstimate(), filtered_new_theta);
  }

  // Add any remaining missing variables.
  for (const auto& key : odom_valid_belief_factors.keys()) {
    if ((not valueExists(key)) and (not filtered_new_theta.exists(key))) {
      filtered_new_theta.insert_or_assign(key, traits<VALUE>::Identity());
    }
  }

  ISAM2Result result;
  spdlog::debug(
      "BPSAM {}: Updating: {} new factors, {} new values, {} "
      "factors to remove",
      params_.robot_id,
      odom_valid_belief_factors.size(),
      filtered_new_theta.size(),
      isam2_update_params.removeFactorIndices.size());
  int max_retries = 20;
  int retry_count = 0;
  NonlinearFactorGraph current_new_factors = odom_valid_belief_factors;
  Values current_new_theta = filtered_new_theta;
  ISAM2UpdateParams current_update_params = isam2_update_params;

  // optimize and retry after removing dangling robot keys
  // TODO(mike): check graph connectivity rather than this workaround
  while (true) {
    try {
      result = BaseSAM::update(
          current_new_factors, current_new_theta, current_update_params);
      break;
    } catch (const gtsam::ValuesKeyDoesNotExist& e) {
      spdlog::error("BPSAM {}: Caught ValuesKeyDoesNotExist exception: {}",
                    params_.robot_id,
                    MultiRobotKeyFormatter(e.key()));
      getFactorsUnsafe().saveGraph(
          update_params.output_dir /
              fmt::format("error_graph_{}.dot", params_.robot_id),
          MultiRobotKeyFormatter);
      throw;
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      spdlog::error(
          "BPSAM {}: Caught IndeterminantLinearSystemException at key {}",
          params_.robot_id,
          MultiRobotKeyFormatter(e.nearbyVariable()));

      if (params_.persistent) throw;  // Never silently remove retained beliefs.
      if (retry_count >= max_retries) {
        spdlog::error("BPSAM {}: Max retries reached ({})",
                      params_.robot_id,
                      max_retries);
        throw;
      }

      if (isRobotKey(e.nearbyVariable())) {
        spdlog::warn(
            "BPSAM {}: nearbyVariable is a robot key, removing its factors "
            "and trying to optimize again (attempt {}/{})",
            params_.robot_id,
            retry_count + 1,
            max_retries);

        ISAM2UpdateParams retry_params;
        Key bad_key = e.nearbyVariable();
        for (size_t i = 0; i < getFactorsUnsafe().size(); ++i) {
          const auto& factor = getFactorsUnsafe()[i];
          if (factor && std::find(factor->begin(), factor->end(), bad_key) !=
                            factor->end()) {
            retry_params.removeFactorIndices.push_back(i);
          }
        }

        spdlog::warn("BPSAM {}: Retrying update with {} factors removed",
                     params_.robot_id,
                     retry_params.removeFactorIndices.size());

        // Prepare for next iteration:
        // 1. No new factors/values to add (already added in previous failed
        // attempt)
        current_new_factors = NonlinearFactorGraph();
        current_new_theta = Values();
        // 2. Set params to remove the bad factors
        current_update_params = retry_params;

        retry_count++;
      } else {
        spdlog::error("BPSAM {}: nearbyVariable is not a robot key",
                      params_.robot_id);
        getFactorsUnsafe().saveGraph(
            update_params.output_dir /
                fmt::format("error_graph_{}.dot", params_.robot_id),
            MultiRobotKeyFormatter);
        writeG2o(getFactorsUnsafe(),
                 {},
                 update_params.output_dir /
                     fmt::format("error_graph_{}.g2o", params_.robot_id));
        throw;
      }
    } catch (...) {
      spdlog::error("BPSAM {}: Caught unknown exception during update",
                    params_.robot_id);
      throw;
    }
  }

  if constexpr (std::is_same_v<VALUE, Similarity3>) {
    // An iSAM2 update that throws can already have appended factors before the
    // retry removes them. Rebuild these caches from the graph that actually
    // survived so a later connected update can restore a removed prior.
    sim3_pose_scale_prior_keys_.clear();
    for (const auto& factor : getFactorsUnsafe()) {
      if (!factor) {
        continue;
      }
      const auto scale_prior =
          cbs::dynamic_pointer_cast<Sim3ScalePriorFactor>(factor);
      if (!scale_prior) {
        continue;
      }
      if (scale_prior->role() == Sim3ScalePriorRole::AbsolutePose) {
        sim3_pose_scale_prior_keys_.insert(scale_prior->key());
      }
    }
  }

  tmp_marginals_.reset();
  marginal_cache_.clear();

  return result;
}

template class BPSAMT<Pose3>;

template class BPSAMT<Similarity3>;

}  // namespace cbs

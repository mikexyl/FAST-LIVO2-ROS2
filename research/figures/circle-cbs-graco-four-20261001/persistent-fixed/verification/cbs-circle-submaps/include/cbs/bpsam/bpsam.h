#include <memory>
#pragma once

#include <aria_common/benchmark.h>
#include <aria_common/logging.h>
#include <aria_common/macros.h>
#include <aria_common/types.h>
#include <cbs/gbp/belief_metric.h>
#include <cbs/gbp/persistent_belief.h>
#include <cbs/gbp/gbp.h>
#include <gtsam/config.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/dataset.h>

#include <memory>
#include <queue>
#include <set>
#include <type_traits>

#include "cbs/bpsam/batch_sam_wrapper.h"
#include "cbs/factors/sim3_scale_factors.h"
#include "cbs/key.h"
#include "cbs/utils/graph_io.h"

using namespace gtsam;

namespace cbs {
using GncParamsGN = GncParams<GaussNewtonParams>;
using GncOptimizerGN = GncOptimizer<GncParamsGN>;
// using BaseSAM = cbs::SAMWrapper<GncParamsGN, GncOptimizerGN>;
// using BaseSAMParams = ISAM2Params;
// using BaseSAM = cbs::SAMWrapper<GaussNewtonParams, GaussNewtonOptimizer>;
// using BaseSAMParams = ISAM2Params;
using BaseSAM = gtsam::ISAM2;
using BaseSAMParams = gtsam::ISAM2Params;

template <class VALUE>
class BPSAMT : public BaseSAM {
 public:
  using Ptr = std::shared_ptr<BPSAMT<VALUE>>;
  using ConstPtr = std::shared_ptr<const BPSAMT<VALUE>>;
  using UniquePtr = std::unique_ptr<BPSAMT<VALUE>>;

  ARIA_DELETE_COPY_CONSTRUCTORS(BPSAMT);

  struct Params {
    AgentId robot_id;
    BaseSAMParams sam_params_;
    gbp::UpdateParams gbp_update_params;
    NonlinearOptimizerParams tol;
    bool enable_gkcm{false};
    bool enable_belief_dcs{false};
    double sim3_pose_scale_prior_sigma{-1.0};
    // Distance threshold under the selected belief metric below which a belief
    // is considered too similar to the last one sent to the requesting robot
    // and is omitted.
    // 0.0 disables the filter.
    double belief_similarity_threshold{0.01};
    bool persistent{false};
    gbp::PersistentBeliefPolicy persistent_belief;
  };

  struct UpdateParams : ISAM2UpdateParams {
    std::set<NonlinearFactor::shared_ptr> unreliable_factors_;
    std::filesystem::path output_dir;
  };

  BPSAMT(const Params& params);

  const NonlinearFactor::shared_ptr getFactor(FactorIndex factor_index) const {
    if (factor_index >= BaseSAM::getFactorsUnsafe().size()) {
      return nullptr;
    }
    return BaseSAM::getFactorsUnsafe().at(factor_index);
  }

  /**
   * @brief all keys, excluding consensus keys, including gbp keys
   *
   * @return KeyVector
   */
  KeySet allKeys() const;
  KeyList allKeysInSAM() const {
    KeyList keys;
    for (const auto& [key, _] : this->getVariableIndex()) {
      keys.push_back(key);
    }
    return keys;
  }

  auto beliefGraph() const { return gbp_graph_; }

  bool existsInGbpGraph(Symbol symbol) const;

  static Values getInvolvedValues(const Values& values, const KeySet& keys);

  NonlinearFactor::shared_ptr beliefToFactor(Key key0,
                                             Key key1,
                                             gbp::Gaussian belief,
                                             bool robust = false) const;

  int addBeliefs(
      std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs);

  int addBeliefs(std::map<Key, std::map<AgentId, gbp::Gaussian>> beliefs) {
    std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs_vec;
    for (auto const& [key, belief] : beliefs) {
      for (auto const& [agent_id, gauss] : belief) {
        beliefs_vec[key].emplace_back(agent_id, gauss);
      }
    }
    return addBeliefs(beliefs_vec);
  }

  auto getCovariance(Key key) { return this->marginalCovariance(key); }

  auto getSigmas(Key key) {
    auto cov = getCovariance(key);
    return cov.diagonal().array().sqrt();
  }

  bool addGBPNode(AgentId agent_id, Key symbol, gbp::Gaussian prior);
  bool addGBPNodePose(AgentId agent_id, Key symbol, gbp::Gaussian prior) {
    return addGBPNode(agent_id, symbol, prior);
  }

  ISAM2Result update(const NonlinearFactorGraph& newFactors = {},
                     const Values& newTheta = {},
                     const UpdateParams& update_params = UpdateParams());

  KeySet keysInGBP() {
    KeySet keys;
    for (auto const& [agent_id, graph] : gbp_graph_) {
      for (auto const& [key, _] : graph.vars()) {
        keys.insert(key);
      }
    }
    return keys;
  }

  bool factorExists(const NonlinearFactor::shared_ptr& factor) const {
    CHECK(factor);
    CHECK(factor->keys().size() > 0);
    auto key0 = factor->keys().at(0);
    auto factors_it = this->getVariableIndex().find(key0);
    if (factors_it == this->getVariableIndex().end()) return false;
    for (auto factor_index : factors_it->second) {
      if (getFactor(factor_index) == factor) return true;
    }
    return false;
  }

  std::map<Key, std::map<AgentId, gbp::Gaussian>> getBeliefs(
      KeySet keys,
      bool use_other_robots = false) const;

  /**
   * @brief Get beliefs for a specific requesting robot, filtering out beliefs
   * that are too similar under the selected metric
   * (distance < belief_similarity_threshold) to the last beliefs sent to that
   * robot. The bookkeeping is updated with whatever is actually returned.
   */
  std::map<Key, std::map<AgentId, gbp::Gaussian>> getBeliefs(
      KeySet keys,
      AgentId requesting_robot,
      bool use_other_robots = false) const;

  KeySet poseKeys() const {
    KeySet keys;
    for (auto const& [key, _] : this->getVariableIndex()) {
      if (isPoseKey(LabeledSymbol(key))) {
        keys.insert(key);
      }
    }
    return keys;
  }

  Matrix marginalCovariance(Key key) const {
    if (not tmp_marginals_) {
      return this->BaseSAM::marginalCovariance(key);
    }

    return tmp_marginals_->marginalCovariance(key);
  }

  KeyList allRobotKeys() const {
    KeyList keys;
    for (auto const& [key, _] : this->getVariableIndex()) {
      if (isRobotKey(key)) {
        keys.emplace_back(key);
      }
    }
    return keys;
  }

  AgentId id() const { return params_.robot_id; }

  // Refresh only the sealed-mode bootstrap for an anchor whose publisher's
  // frame is now connected at the recipient. The Gaussian and its producer
  // identity are unchanged; no belief contraction or measurement is added here.
  void refreshMissingAnchorBootstraps(AgentId recipient,
                                     const KeySet& initialized_anchors) const {
    if (params_.persistent) return;
    auto& counts = sent_belief_counts_[recipient];
    for (auto& [endpoint, count] : counts) {
      const auto [key, producer] = endpoint;
      if (isRobotKey(key) && initialized_anchors.count(key) == 0 &&
          initialized_anchors.count(toRobotKey(producer)) != 0) {
        count = 0;
      }
    }
  }

  auto marginalizationFactors() const {
    if (not tmp_marginals_) {
      return GaussianFactorGraph{};
    }
    return tmp_marginals_->graph();
  }

  class MarginalsWithGraph : public gtsam::Marginals {
   public:
    MarginalsWithGraph(const NonlinearFactorGraph& factors,
                       const Values& values)
        : gtsam::Marginals(factors, values) {}

    const GaussianFactorGraph& graph() const { return graph_; }
  };

  enum class MarginalizationType { LOCAL, GLOBAL, FULL };
  void setMarginalizationGraph(MarginalizationType type) {
    if (params_.persistent && marginal_cache_.count(type)) {
      tmp_marginals_ = marginal_cache_.at(type);
      return;
    }
    auto factors = this->getFactorsUnsafe();
    size_t n_factors_removed = 0;
    size_t n_scale_priors_removed = 0;
    for (auto& factor : factors) {
      if (not factor) continue;
      if constexpr (std::is_same_v<VALUE, Similarity3>) {
        if (auto scale_prior =
                cbs::dynamic_pointer_cast<Sim3ScalePriorFactor>(factor);
            scale_prior &&
            scale_prior->role() == Sim3ScalePriorRole::AbsolutePose) {
          factor.reset();
          n_factors_removed++;
          n_scale_priors_removed++;
          continue;
        }
      }
      if (factor->keys().size() != 2) continue;
      switch (type) {
        case MarginalizationType::LOCAL:
          if (isPoseBeliefFactor(id(), factor) or
              isAnchorBeliefFactor(factor)) {
            factor.reset();
            n_factors_removed++;
          }
          break;
        case MarginalizationType::GLOBAL:
          if (isAnchorBeliefFactor(factor)) {
            factor.reset();
            n_factors_removed++;
          }
          break;
        case MarginalizationType::FULL:
          break;
      }
    }

    spdlog::debug(
        "BPSAM {}: Removed {} factors for marginalization, including {} "
        "absolute Sim3 scale priors",
        id(),
        n_factors_removed,
        n_scale_priors_removed);

    if (type == MarginalizationType::LOCAL or
        type == MarginalizationType::GLOBAL) {
      Values all_values = this->calculateEstimate();
      Values values;
      // add keys in factors
      for (auto const& factor : factors) {
        if (not factor) continue;
        for (auto const& key : factor->keys()) {
          values.insert_or_assign(key, all_values.at(key));
        }
      }

      try {
        tmp_marginals_ = std::make_shared<MarginalsWithGraph>(factors, values);
        if (params_.persistent) marginal_cache_[type] = tmp_marginals_;
      } catch (const std::exception& e) {
        factors.print();
        throw;
      }
    } else {
      tmp_marginals_.reset();
    }
  }

  // Cavity factorizations are valid only until the next optimizer update.
  mutable std::map<MarginalizationType, std::shared_ptr<MarginalsWithGraph>>
      marginal_cache_;

  std::map<AgentId, std::map<Key, double>> getDivergences() {
    // traver through the gbp graph and collect the Dxy of each key
    std::map<AgentId, std::map<Key, double>> divergences;
    for (auto const& [agent_id, graph] : gbp_graph_) {
      for (auto const& [key, var] : graph.vars()) {
        divergences[agent_id][key] = var->dxy();
      }
    }
    return divergences;
  }

  const Params& getParams() const { return params_; }

  bool saveLocalGraphG2o(const std::filesystem::path& filename,
                         bool remove_label) const {
    // traverse all factors and pick ones doesn't connect to anchors
    NonlinearFactorGraph local_graph;
    for (const auto& factor : this->getFactorsUnsafe()) {
      if (not factor) continue;
      bool is_local = true;
      for (const auto& key : factor->keys()) {
        if (isRobotKey(key)) {
          is_local = false;
          break;
        }
      }
      if (is_local) {
        local_graph.push_back(factor);
      }
    }

    // if remove char, then convert labeled symbols to normal symbols
    NonlinearFactorGraph rekeyed_graph;
    Values rekeyed_values;
    if (remove_label) {
      auto rekey = [](Key key) {
        auto labeled_key = LabeledSymbol(key);
        if (isPoseKey(labeled_key)) {
          return Symbol(labeled_key.chr(), labeled_key.index());
        } else {
          throw std::runtime_error(
              "shouln't reach here in BPSAM::saveLocalGraphG2o");
        }
      };

      std::map<Key, Key> rekey_map;
      for (const auto& key : local_graph.keys()) {
        rekey_map[key] = rekey(key);
      }

      rekeyed_graph = local_graph.rekey(rekey_map);

      for (const auto& [key, value] : this->calculateEstimate()) {
        if (rekey_map.find(key) != rekey_map.end()) {
          rekeyed_values.insert(rekey_map.at(key), value);
        }
      }
    } else {
      rekeyed_graph = local_graph;
      rekeyed_values = this->calculateEstimate();
    }

    // check if filename parent path exists, and throw error if not
    auto parent_path = filename.parent_path();
    if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
      throw std::runtime_error("BPSAM::saveLocalGraphG2o: Parent path " +
                               parent_path.string() + " does not exist.");
    }

    // check if filename exists, then remove it
    if (std::filesystem::exists(filename)) {
      std::filesystem::remove(filename);
    }

    cbs::writeG2oNoIndex(rekeyed_graph, rekeyed_values, filename.string());
    return true;
  }

 protected:
  Params params_;

  // map<"sending agent", graph>
  using AgentBPGraph = std::map<AgentId, gbp::Graph>;
  AgentBPGraph gbp_graph_;
  // map<"receiving agent", <"sending agent", graph>>
  mutable std::map<AgentId, AgentBPGraph> out_gbp_graph_;
  Values gbp_values_;    // gbp values
  Values T_r_r_values_;  // Trr values
  std::vector<std::pair<Key, Key>> new_gbp_keys_;
  std::set<Key> sim3_pose_scale_prior_keys_;
  std::set<Key> sim3_remote_copy_scale_gauge_keys_;

  mutable std::shared_ptr<MarginalsWithGraph> tmp_marginals_;

  // Bookkeeping: last_sent_beliefs_[requesting_robot][(belief_key,
  // producing_agent)] stores the Gaussian most recently returned by
  // getBeliefs() for that robot.
  mutable std::map<AgentId, std::map<std::pair<Key, AgentId>, gbp::Gaussian>>
      last_sent_beliefs_;

  // The receiver uses the first message for a new producer/key pair to
  // bootstrap its GBP node and deliberately does not accept it as a belief
  // factor. Track transmissions per requesting robot so the second message is
  // always sent before similarity filtering begins.
  mutable std::map<AgentId, std::map<std::pair<Key, AgentId>, size_t>>
      sent_belief_counts_;

  // Metric used by the robot-aware getBeliefs() overload to decide whether a
  // belief has changed enough to be worth resending.
  std::unique_ptr<gbp::BeliefMetric> belief_metric_;
};

using BPSAM = BPSAMT<gtsam::Pose3>;

using Sim3BPSAM = BPSAMT<gtsam::Similarity3>;
using ScaleBPSAM = Sim3BPSAM;

}  // namespace cbs

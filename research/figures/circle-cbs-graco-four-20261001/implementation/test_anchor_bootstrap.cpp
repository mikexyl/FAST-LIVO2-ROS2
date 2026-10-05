#include <cbs/bpsam/bpsam.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  cbs::BPSAM::Params params;
  params.robot_id = 'a';
  params.belief_similarity_threshold = .01;
  cbs::BPSAM solver(params);
  const auto a0 = cbs::toPoseKey('a', 0);
  const auto ra = cbs::toRobotKey('a'), rb = cbs::toRobotKey('b');
  const auto b0 = cbs::toPoseKey('b', 0), rd = cbs::toRobotKey('d');
  auto noise = gtsam::noiseModel::Isotropic::Sigma(6, .1);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(a0, gtsam::Pose3(), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(ra, a0, gtsam::Pose3(), noise);
  const gtsam::Pose3 a_from_b(gtsam::Rot3::Rz(.3), {3., -2., 1.});
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(a0, b0, a_from_b, noise);
  gtsam::Values values;
  values.insert(a0, gtsam::Pose3()); values.insert(ra, gtsam::Pose3());
  values.insert(b0, a_from_b); solver.update(graph, values);
  std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> messages;
  messages[b0].emplace('b', gbp::Gaussian(
      b0, gtsam::Vector6::Zero(), gtsam::Matrix6::Identity(), 1));
  solver.addBeliefs(messages); solver.update();
  solver.addBeliefs(messages); solver.update();
  require(solver.valueExists(rb), "publisher failed to establish anchor");
  solver.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::GLOBAL);
  const auto first = solver.getBeliefs({rb}, 'd', true);
  require(first.size() == 1, "first anchor bootstrap absent");
  require(solver.getBeliefs({rb}, 'd', true).size() == 1, "second bootstrap absent");
  require(solver.getBeliefs({rb}, 'd', true).empty(), "unchanged belief not filtered");
  // Until A's frame is initialized at D, this anchor cannot be attached there.
  solver.refreshMissingAnchorBootstraps('d', {rd});
  require(solver.getBeliefs({rb}, 'd', true).empty(), "unconnected producer forced reply");
  solver.refreshMissingAnchorBootstraps('d', {ra, rd});
  const auto retry = solver.getBeliefs({rb}, 'd', true);
  require(retry.size() == 1, "connected recipient cannot refresh missing anchor");
  const auto& before = first.at(rb).at('a');
  const auto& after = retry.at(rb).at('a');
  require(before.mu().isApprox(after.mu(), 1e-12) &&
              before.Sigma().isApprox(after.Sigma(), 1e-12),
          "bootstrap refresh changed the Gaussian");
  solver.getBeliefs({rb}, 'd', true);
  solver.refreshMissingAnchorBootstraps('d', {ra, rb, rd});
  require(solver.getBeliefs({rb}, 'd', true).empty(), "initialized anchor was resent");
  std::cout << "recipient-aware anchor bootstrap refresh: passed\n";
}

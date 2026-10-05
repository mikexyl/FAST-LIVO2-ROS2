#include <cbs/bpsam/bpsam.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>
#include <random>

namespace {
using Messages = std::map<gtsam::Key,
                          std::map<cbs::AgentId, gbp::Gaussian>>;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void preserve_initial_trajectory() {
  cbs::BPSAM::Params params;
  params.robot_id = 'a';
  params.persistent = true;
  cbs::BPSAM robot(params);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, .1);
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  const auto anchor = cbs::toRobotKey('a');
  const auto first = cbs::toPoseKey('a', 0);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      first, gtsam::Pose3(), noise);
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      anchor, first, gtsam::Pose3(), noise);
  values.insert(anchor, gtsam::Pose3());
  values.insert(first, gtsam::Pose3());
  gtsam::Pose3 previous;
  for (size_t i = 1; i < 20; ++i) {
    const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.02*i, -.03*i, .15*i),
                            {15.*i, -4.*i, 2.*i});
    const auto key = cbs::toPoseKey('a', i);
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        cbs::toPoseKey('a', i-1), key, previous.between(pose), noise);
    values.insert(key, pose);
    previous = pose;
  }
  const auto remote = cbs::toPoseKey('b', 7);
  const gtsam::Pose3 measurement(gtsam::Rot3::Rz(1.2), {12., 3., -8.});
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      cbs::toPoseKey('a', 19), remote, measurement, noise);
  // Remote initial values are intentionally absent: infer them from the
  // verified measurement and preserve the supplied local odometry exactly.
  // An unconnected cached seed must still be excluded from persistent ISAM.
  const auto orphan = cbs::toRobotKey('e');
  values.insert(orphan, gtsam::Pose3());
  robot.update(factors, values);
  require(factors.error(robot.calculateEstimate()) < 1e-12,
          "first update discarded valid trajectory initialization");
  require(robot.calculateEstimate<gtsam::Pose3>(remote).equals(
              previous * measurement, 1e-8),
          "new remote separator was not initialized from its local endpoint");
  require(!robot.valueExists(orphan), "unconnected seed reached ISAM");
}

void chain(bool random_stages, bool reverse_delivery, bool ring = false) {
  constexpr size_t count = 5;
  std::vector<std::unique_ptr<cbs::BPSAM>> robots;
  std::vector<gtsam::Pose3> truth;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, .1);
  gtsam::KeySet anchors;
  const auto adjacent = [ring](size_t i, size_t j) {
    const auto distance = std::abs(static_cast<int>(i)-static_cast<int>(j));
    return distance == 1 || (ring && distance == count-1);
  };
  for (size_t i = 0; i < count; ++i) {
    truth.emplace_back(gtsam::Rot3::RzRyRx(.07*i, -.03*i, .2*i),
                       gtsam::Point3(4.*i, -2.*i, .3*i));
    anchors.insert(cbs::toRobotKey('a' + i));
  }
  for (size_t i = 0; i < count; ++i) {
    const auto id = static_cast<cbs::AgentId>('a' + i);
    cbs::BPSAM::Params params;
    params.robot_id = id;
    params.persistent = true;
    robots.push_back(std::make_unique<cbs::BPSAM>(params));
    const auto own = cbs::toPoseKey(id, 0);
    const auto anchor = cbs::toRobotKey(id);
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        own, gtsam::Pose3(), noise);
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        anchor, own, gtsam::Pose3(), noise);
    values.insert(own, gtsam::Pose3());
    values.insert(anchor, gtsam::Pose3());
    for (size_t j = 0; j < count; ++j) {
      if (!adjacent(i, j)) continue;
      const auto remote = cbs::toPoseKey('a' + j, 0);
      const auto relative = truth[i].between(truth[j]);
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          own, remote, relative, noise);
      values.insert(remote, relative);
    }
    robots.back()->update(factors, values);
  }
  std::mt19937 rng(31);
  std::bernoulli_distribution pose_stage(.5);
  std::vector<size_t> stable_sizes;
  for (size_t step = 0; step < 80; ++step) {
    std::vector<std::pair<size_t, Messages>> deliveries;
    for (size_t i = 0; i < count; ++i) {
      const bool poses = random_stages ? pose_stage(rng) : step % 2 == 0;
      for (size_t j = 0; j < count; ++j) {
        if (!adjacent(i, j)) continue;
        robots[j]->setMarginalizationGraph(poses
            ? cbs::BPSAM::MarginalizationType::LOCAL
            : cbs::BPSAM::MarginalizationType::GLOBAL);
        auto response = robots[j]->getBeliefs(
            poses ? robots[i]->poseKeys() : anchors, 'a' + i, !poses);
        // A forwarded Gaussian keeps its original producer. A robot must
        // never turn its own returned belief into a new measurement.
        for (auto& [key, producers] : response) producers.erase('a' + i);
        deliveries.emplace_back(i, std::move(response));
      }
    }
    if (reverse_delivery) std::reverse(deliveries.begin(), deliveries.end());
    for (const auto& [i, response] : deliveries) robots[i]->addBeliefs(response);
    for (auto& robot : robots) robot->update();
    if (step == 60) {
      for (const auto& robot : robots)
        stable_sizes.push_back(robot->getFactorsUnsafe().nrFactors());
    }
  }
  for (size_t i = 0; i < count; ++i) {
    for (size_t j = 0; j < count; ++j) {
      const auto key = cbs::toRobotKey('a' + j);
      require(robots[i]->valueExists(key),
              "anchor did not propagate through the five-robot chain");
      require(robots[i]->calculateEstimate<gtsam::Pose3>(key).equals(
                  truth[i].between(truth[j]), 1e-5),
              "relayed anchor changed coordinate frame");
    }
    require(stable_sizes[i] == robots[i]->getFactorsUnsafe().nrFactors(),
            "unchanged anchor relays grew the factor graph");
  }
}

void preserve_producer_gaussian() {
  cbs::BPSAM::Params params;
  params.robot_id = 'b';
  params.persistent = true;
  cbs::BPSAM robot(params);
  const auto b0 = cbs::toPoseKey('b', 0), a0 = cbs::toPoseKey('a', 0);
  const auto rb = cbs::toRobotKey('b'), rc = cbs::toRobotKey('c');
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, .1);
  const gtsam::Pose3 b_from_a(gtsam::Rot3::Rz(.2), {3., 2., 1.});
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      b0, gtsam::Pose3(), noise);
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      rb, b0, gtsam::Pose3(), noise);
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      b0, a0, b_from_a, noise);
  gtsam::Values values;
  values.insert(b0, gtsam::Pose3());
  values.insert(rb, gtsam::Pose3());
  values.insert(a0, b_from_a);
  robot.update(factors, values);
  Messages poses;
  poses[a0].emplace('a', gbp::Gaussian(
      a0, gtsam::Vector6::Zero(), gtsam::Matrix6::Identity(), 1));
  robot.addBeliefs(poses);
  robot.update();
  const auto mean = gtsam::Pose3::Logmap(
      gtsam::Pose3(gtsam::Rot3::Rz(.3), {5., -2., 1.}));
  gbp::Gaussian original(rc, mean, .2*gtsam::Matrix6::Identity(), 1);
  for (size_t iteration = 0; iteration < 2; ++iteration) {
    Messages anchors;
    anchors[rc].emplace('a', original);
    robot.addBeliefs(anchors);
    robot.update();
    robot.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::GLOBAL);
    const auto relayed = robot.getBeliefs({rc}, 'd', true);
    require(relayed.size() == 1 && relayed.at(rc).size() == 1 &&
                relayed.at(rc).count('a') == 1,
            "relay lost its original producer identity");
    const auto& actual = relayed.at(rc).at('a');
    std::cout << "relay roundoff: mean=" << (actual.mu()-original.mu()).norm()
              << " covariance=" << (actual.Sigma()-original.Sigma()).norm() << std::endl;
    require(actual.mu().isApprox(original.mu(), 1e-12) &&
                actual.Sigma().isApprox(original.Sigma(), 1e-12),
            "relay changed the producer Gaussian or applied damping twice");
    require(robot.getBeliefs({rc}, 'd', true).empty(),
            "unchanged relay was sent repeatedly");
    require(robot.getBeliefs({rc}, 'a', true).empty(),
            "relay echoed a belief back to its producer");
    require(robot.getBeliefs({rc}, 'e', true).size() == 1,
            "different recipients shared bootstrap history");
    // Forwarding an already admitted producer update must not run a second
    // divergence gate; only newly computed local beliefs use that policy.
    original.mu()(3) += 10.;
    original.updateCanonical();
  }
}
}  // namespace

int main() {
  preserve_initial_trajectory();
  chain(false, false);
  chain(true, false);
  chain(true, true);
  chain(true, true, true);
  preserve_producer_gaussian();
  std::cout << "persistent anchor relay: chain, independent stages, reordered delivery passed\n";
}

# Multi-robot LiDAR SLAM development branches

This workspace supports EllipseLIO or FAST-LIVO2 odometry, ellipsoid BEV / raw
MapClosures retrieval, distributed loop evidence exchange, distributed PCM and
CBS pose-graph optimization. Optional `gtsam_points` registration factors augment
the verified pose factors. Evo evaluates trajectories; Rerun records and displays
maps and diagnostics. Benchmark adapters are published separately from our
frontend/backend path.

## Repositories

| Repository | Development branch |
| --- | --- |
| [FAST-LIVO2-ROS2](https://github.com/mikexyl/FAST-LIVO2-ROS2/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [ellipselio](https://github.com/mikexyl/ellipselio/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [cbs](https://github.com/mikexyl/cbs/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [cbs_ros](https://github.com/mikexyl/cbs_ros/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [livox_ros_driver2](https://github.com/mikexyl/livox_ros_driver2/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [rpg_vikit](https://github.com/mikexyl/rpg_vikit/tree/dev/multi-robot-lidar-slam) | `dev/multi-robot-lidar-slam` |
| [Swarm-SLAM](https://github.com/mikexyl/Swarm-SLAM/tree/dev/multi-robot-lidar-slam-comparison) | `dev/multi-robot-lidar-slam-comparison` |
| [cslam](https://github.com/mikexyl/cslam/tree/dev/multi-robot-lidar-slam-comparison) | `dev/multi-robot-lidar-slam-comparison` |

The previous development branches remain available. CBS, Livox message support,
and vikit reuse their existing committed changes on the new common branch.
EllipseLIO, Swarm-SLAM and cslam are forks created with GitHub CLI. Existing owned
repositories/forks are reused.

`multi_robot_lidar_slam.repos` at the wrapper root records the GitHub URLs and
branch names. Import it from an empty workspace's `src` directory with `vcs import`.
The Swarm-SLAM `src/cslam` entry is an independent nested repository; the umbrella
ignores `src/`. Its official-component benchmark instead builds its documented
pinned upstream source and does not apply the comparison branch's algorithm fixes.

MapClosures and `gtsam_points` are unmodified pinned dependencies, not additional
project forks. Their adapters live under `research/adapters/`; dependency commits
are documented in `assets.lock.json` and `MIXED-PGO.md`.

## Entry points and status

- [EllipseLIO frontend and ellipsoid BEVs](ELLIPSELIO.md)
- [Distributed CBS and PCM](DPGO.md)
- [Distributed registration factors](../../cbs_ros/docs/registration_factors.md)
- [Native online Rerun recorder](../scripts/ellipselio_live/README.md)
- [Experiment results and known failures](RESULTS-SUMMARY.md)
- [Retained artifact policy](figures/README.md)

These branches preserve research development and failed-run diagnostics. They do
not imply that the frontend succeeds on every sequence or that live research
export is safe with arbitrary executor scheduling. In particular, the Bob live
capture disables the separate synchronized exporter, as described in the recorder
README and `RESULTS-S3E-IMU-NOISE.md`.

## Publication validation (2026-09-18)

- Research Python suite: 81 passed, 10 opt-in tests skipped.
- Explicit native DDS integration suite: all 8 tests passed (including three-
  and four-robot registration factors, PCM and disconnected components).
- CBS and CBS ROS CTest suites: 5 + 5 passed, including native DDS PCM.
- Swarm comparison/evaluation utility tests: 7 passed.
- CBS ROS rebuilt successfully; changed Python sources passed syntax checking.
- Bob's full live recording passed Rerun 0.37.1 validation locally and on 148.

The ROS-sourced Python test invocation used `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
to avoid an incompatible system `launch_testing` pytest plugin. DDS tests used
`S3E_TEST_DDS=1`; no test bodies were disabled to obtain those eight passes.

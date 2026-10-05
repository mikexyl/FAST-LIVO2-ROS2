# Persistent CBS at 10-second intervals

PASS: common reference, published transforms and final geometry checks passed.

Tested persistent CBS on the frozen 121-submap, 13-loop GRACO A05/A07/G05/G06 graph on 148. Each robot was configured for one iteration every 10 seconds and 30 iterations. The run took 331.0 seconds, including startup, PCM, geometry preparation, scheduled waiting and shutdown.

| Robot | Iterations | Median interval [s] | Max solve [ms] | Max timer callback [ms] | Max belief service [ms] | Final reference | Owned GPU factors | Final geometry |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aerial05 | 30 | 9.969 | 3.635 | 461.860 | 67.583 | aerial05 | 10 | pass |
| aerial07 | 30 | 9.971 | 12.836 | 410.672 | 1.409 | aerial05 | 1 | pass |
| ground05 | 30 | 9.971 | 1.914 | 403.377 | 402.592 | aerial05 | 2 | pass |
| ground06 | 30 | 9.971 | 0.536 | 1.069 | 2.640 | aerial05 | 0 | pass |

The old persistent response path disabled the original CBS anchor relay. Incoming
anchor factors are excluded from the GLOBAL covariance graph, so anchors learned only through those
factors could not produce a local marginal and were silently omitted. Sparse chains exposed the bug;
the earlier fully connected three-robot tests did not.

The repair forwards cached anchor Gaussians with their original producer, mean, covariance and frame.
It does not label forwarded evidence as an independent local measurement or damp it a second time.
Repeated unchanged relays and echoes to the original producer are suppressed. Local-belief damping,
the two covariance exclusions and random stage selection remain unchanged.

A second bug discarded all supplied odometry initial poses in persistent mode: GTSAM's
FactorGraph::exists argument is a factor slot, not a pose key. The repair checks the graph's
variable-key set. The whole map no longer starts at identity, and new remote separators initialize
from their connected local measurement. Unconnected seeds remain excluded. The new long, rotated
trajectory regression fails before this correction and passes after it.

The relay-only intermediate was rejected: its labels reached A05, but the bad initialization
produced 87.899 m joint ATE and a 208.673 m published loop mismatch. This probe now also audits
published endpoint transforms using the existing configured alignment tolerances (2 m, 15 degrees),
independently of ground truth. Local geometry checks alone could not expose that error.

The new native five-robot chain test fails on the old core and passes after the repair. Tests also
cover independent stages, reordered delivery, a ring, producer/covariance preservation, duplicate
suppression and graph-size stability. Real DDS tests cover a growing five-robot chain with both
pose-only graphs and GPU registration factors.

Joint ATE is 0.954865 m, versus 0.961862 m for the retained coordinated run and 0.959933 m for the centralized control. Maximum published loop mismatch is 0.227704 m / 0.288652 degrees. These small ATE differences are descriptive, not an accuracy-improvement claim. Evaluation uses evo 1.36.5, 50 ms association, one shared rigid alignment, no scale fit and corrected GRACO IMU-reference ground truth. Ground truth is used only after optimization.


This rerun repairs persistent anchor relay and graph initialization, using the same configuration
as the earlier 10-second test. PCM, GPU registration settings, verified loops and point geometry are
unchanged. Both runs retain independent random stage selection; their random draws can differ.
Every transferred prepared endpoint matches the centralized evidence hashes.
No artificial host CPU, memory or GPU quota was imposed.

Each robot's timer callback is mutually exclusive. Peer belief requests remain asynchronous, with
one outstanding request per peer. The requested 10 seconds is a scheduling interval, not solve time.
Observed intervals are measured by 100 ms polling. The solve column times the native optimizer
update. Timer callbacks include local update and request dispatch, but exclude asynchronous response
latency. Belief-service timing includes lock waiting, marginalization and response packing. The
terminal timer callback is not present in the native timing CSV.

The same frozen graph is submitted once to optimizers that remain alive throughout the test. This
isolates cadence and persistent belief behavior; it is not a live frontend or growing-map replay.


Measured process-tree CPU: 55.93 s. Peak simultaneous process-tree RSS: 2.31 GiB (20 ms sampling; shared pages may count twice). CBS requests/responses: 22.91 KiB; PCM: 16.64 KiB; geometry exchange: 2.045 MiB. Expired belief requests: 0. Communication counts serialized application messages at the requester/sender once; it excludes discovery, network headers, retransmissions, reused retrieval and coordinator-local input/status traffic.


Verification: 28 native Python tests passed (one separately gated test skipped); eight core C++ and eight ROS C++ tests passed (one separately gated ROS test skipped). The local Python suite passed 21 tests with eight native-only skips. Source snapshots, patches, test logs, old-code regression failure and hashes are retained in `verification/`. These suites overlap; counts are not summed. [Verification summary](verification.json). All four references propagated by iteration 8 and remained A05 through iteration 30.

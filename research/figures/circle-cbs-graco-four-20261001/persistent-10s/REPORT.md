# Persistent CBS at 10-second intervals

FAIL: Persistent CBS final common-frame contract failed.

Tested persistent CBS on the frozen 121-submap, 13-loop GRACO A05/A07/G05/G06 graph on 148. Each robot was configured for one iteration every 10 seconds and 30 iterations. The run took 320.9 seconds, including startup, PCM, geometry preparation, scheduled waiting and shutdown.

| Robot | Iterations | Median interval [s] | Max solve [ms] | Max timer callback [ms] | Max belief service [ms] | Final reference | Owned GPU factors | Final geometry |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aerial05 | 30 | 9.970 | 1.737 | 435.363 | 48.369 | aerial05 | 10 | pass |
| aerial07 | 30 | 9.970 | 1.532 | 395.169 | 0.908 | aerial05 | 1 | pass |
| ground05 | 30 | 9.970 | 1.041 | 389.676 | 380.689 | aerial05 | 2 | pass |
| ground06 | 30 | 9.970 | 0.521 | 1.054 | 0.543 | aerial07 | 0 | pass |

Slowing the cadence did not establish a common frame across all four robots within
30 iterations. A05, A07 and G05 publish in A05's frame, while G06 still publishes in A07's frame.
The local geometry checks can pass despite this global inconsistency. No four-robot joint ATE is
reported because the common-frame gate failed; no external alignment repairs the result.

This reproduces the reference-propagation failure at a slower cadence. The logged computation fits
comfortably within 10 seconds, so optimizer callback overrun does not explain this run's failure.
Code inspection shows that the persistent GLOBAL marginal removes incoming anchor-belief factors
and persistent responses do not relay those anchors. This is consistent with failure to carry A05's
reference through the A05–A07–G05–G06 chain; repairing that mechanism is separate from this cadence test.


This test changes only the execution mode, cadence, settling count and timeout. The native
optimizer binaries, PCM policy, GPU registration settings, verified loops and point geometry match
the sealed experiment. Every transferred prepared endpoint matches the centralized evidence hashes.
No artificial host CPU, memory or GPU quota was imposed.

Each robot's timer callback is mutually exclusive. Peer belief requests remain asynchronous, with
one outstanding request per peer. The requested 10 seconds is a scheduling interval, not solve time.
Observed intervals are measured by 100 ms polling. The solve column times the native optimizer
update. Timer callbacks include local update and request dispatch, but exclude asynchronous response
latency. Belief-service timing includes lock waiting, marginalization and response packing. The
terminal timer callback is not present in the native timing CSV.

The same frozen graph is submitted once to optimizers that remain alive throughout the test. This
isolates cadence and persistent belief behavior; it is not a live frontend or growing-map replay.


Measured process-tree CPU: 55.11 s. Peak simultaneous process-tree RSS: 2.31 GiB (20 ms sampling; shared pages may count twice). CBS requests/responses: 39.30 KiB; PCM: 16.64 KiB; geometry exchange: 2.045 MiB. Expired belief requests: 0. Communication counts serialized application messages at the requester/sender once; it excludes discovery, network headers, retransmissions, reused retrieval and coordinator-local input/status traffic.

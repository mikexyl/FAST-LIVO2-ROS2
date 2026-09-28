# Measured communication benchmark — 28 September 2026

The campaign on workstation 148 compares the default point-cloud method with
unmodified native Swarm-SLAM, DCL-SLAM, DiSCo-SLAM and GAC-Mapping. Main scope:
GRACO ground (G01/G02/G03), aerial (A05/A07/A08), mixed (G05/G06/A07), and the
12 selected S3E groups (Laboratory 1–3, Library 1–2, Playground 1–3, Square 1–3,
Tunnel 1). Each native group replays at 1x, with a 120-second settling tail;
GAC uses the released sequential-session interaction and its existing one-hour
finishing allowance. GAC lacks the required camera inputs for the aerial/mixed
GRACO teams. These entries are unavailable, never zero-byte successes.

Our method reuses frozen, finite, chronological native frontend trajectories,
point BEVs and geometry. It releases snapshots at their relative sensor times,
then actually runs independent DDS retrieval/verification and native PCM/CBS.
This measures backend communication; it is not a fresh odometry or frontend
runtime benchmark. Each robot is isolated from its peers' stored geometry and
from ground truth. Inputs, code and configuration hashes are retained.

The new CBS scheduler captures each changed graph every 10 seconds on the common
descriptor-availability clock, including the final tail. It processes all queued
revisions, preserving 100 settling iterations, PCM and live GICP factors.
Graph capture continues during native optimization. No claim of ten-second
output latency is made; queue wait and input-to-publication lag are reported.
Sealed native sessions currently exchange retained geometry again in each
revision, and all those bytes are charged. Persistent cross-revision caching is
not assumed. The historical five-revision recordings remain unchanged.

## Accounting

- Charge serialized application payload per remote robot. Exclude sensor input,
  local same-robot exchanges, observer/visualization-only links, middleware
  discovery, IP/RTPS/TCP headers and retransmissions. This is not NIC traffic or
  a wireless-delivery benchmark. MB and GB are decimal.
- Our method: actual CDR sender counters for retrieval, verification and
  registration exchange; PCM counters; CBS client request+response counters,
  which count each service message once. A failed revision without saved native
  counters makes its reported subtotal a lower bound, not a complete total.
- ROS1 baselines: poll native publisher TCPROS connection counters before
  teardown. Subtract the verified four-byte per-message ROS framing. Sum
  reconnects; merge duplicate native processes belonging to one receiving robot
  using the largest node total. Also retain the unmerged connection total.
  A passive serialized subscriber is an independent cross-check. It can miss
  large bursts: the first DiSCo ground run demonstrated why publisher counters
  must be primary. Unknown/disconnected counter coverage is disclosed.
- Swarm-SLAM: a transparent RCL publication hook records exact serialized sizes
  and source node identity, including shared directed topics. Multiply by the
  native remote-robot subscriber topology. Early publications can precede
  discovery; their bytes are separately disclosed. The installed Humble observer
  lacks publisher IDs, so its ambiguous-topic omissions are not used as totals.
  The hook duplicates serialization but does not alter messages or algorithms;
  no claim of zero monitoring overhead or method runtime speedup is made.
- GAC-Mapping's released backend is centralized. Record its native
  frontend-to-backend clouds and odometry as a proxy in a separate column. Do not
  compare it as native peer traffic or infer zero communication from its lack
  of peer publishers.

Stop the entire group if native odometry is nonfinite or exceeds 20 m/s;
preserve the partial traffic and mark the group stopped. Do not rank an early
failure against a completed run by its smaller byte count. Independent groups
continue. No resource quotas are imposed on 148. Concurrent lanes are chosen
for throughput; bulk outputs stay there.

## Files and checks

Remote root: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/communication-benchmark148-20260928`.
`code-versions/` freezes each baseline run's monitors; `status/`, `logs/`,
`runs/`, native connection samples and sender traces retain provenance.
Smoke/setup-failed attempts are excluded from `full-v2` result tables.
`collect.py` writes a compact report with status, scope, components, byte
breakdown, and CBS cadence. Measurements without complete accounting remain
explicitly incomplete even if the native runner exits successfully.

Local verification: eight counter tests and eleven online scheduling/geometry
handoff tests passed. The native GRACO ground smoke test processed three CBS
revisions with one input captured while its predecessor was running; the first
regular interval was 10.016 seconds, with the final tail flushed immediately.

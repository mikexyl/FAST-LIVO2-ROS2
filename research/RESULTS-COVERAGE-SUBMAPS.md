# Observed-coverage submaps: implementation and local validation

2026-09-20. **Implemented as an opt-in strategy; the two-robot stability gate did
not pass.** Native and Python tests pass, and the diagnostic distributed pipeline
finishes, but aerial-08 has a 1.712 s gap between successful LiDAR updates. This
is an implementation check, not a full-sequence accuracy result or a replacement
for the historical temporal/spatial experiments.

The [policy](../scripts/recent_submaps/COVERAGE-SUBMAPS.md) counts occupied cells
from corrected native processed returns in a fixed gravity-horizontal grid.
Normal submap boundaries depend on observed area, newly observed area, and measured
active/successor overlap. Drone motion and elapsed time serve as secondary guards.
The grid is centered on observed returns; there is no drone-centered coverage crop.
Both maps build their own ellipsoids, and the completed map's exact member scans
supply registration evidence. Temporal remains the default.

The initial [coverage profile](../scripts/recent_submaps/coverage_submaps.yaml)
uses 1 m cells, starts a successor after 2,000 m² occupied / 1,000 m² new area,
and requests handover after 4,000 m² occupied / 2,000 m² new area. Handover also
requires 1,000 m² shared area, intersection/max(map areas) ≥0.25, and existing
odometry support checks. New area is measured from that map's first member scan.
Limits are 80 m horizontal displacement and 120 s age, with explicit recovery
when a hard guard cannot hand over to a supported successor.

The matching [backend profile](../scripts/recent_submaps/coverage_rollout.yaml)
keeps verification evidence and GICP sampling at 0.4 m. It removes adaptive
point-count coarsening from those two stages, addressing the
[confirmed preprocessing defect](RESULTS-GRACO-SPATIAL-VERIFICATION-DIAGNOSTIC.md).
Existing acceptance thresholds, PCM settings, and optional CBS registration-factor
settings are preserved. Coverage preparation requires the fixed-resolution profile;
evidence resolution and point counts travel through worker IPC and are checked.

**Verification.** The isolated native build passed both CTest executables, including
window/buffer ownership, covariance preservation, no self-matching, stationary new
views versus translated repeated views, gravity-plane projection, shared-cell
overlap, hard guards, and bounded writer backpressure. The writer test passed 80
immutable packets with 1.792 s backpressure and 4,160 KiB peak RSS growth.
There were 62 passing Python tests and six passing distributed integration tests
covering temporal, displacement, and coverage endpoints, worker isolation, causal
availability, and native PCM/CBS/GICP. A large identical-geometry regression case
passes the unchanged metric gate with fixed sampling. Python syntax and both
repository `git diff --check` checks pass.

**Local aerial check.** Fresh aerial-05 and aerial-08 replays ran concurrently at
1× for 120 seconds each, with the existing calibration, requested IMU noise,
reliable sensor input, and four-thread launchers. No ground truth was supplied.
The first sandboxed attempt could not access CUDA and had DDS socket errors; its
failure record is retained separately. The fresh run used working GPU/DDS access.

| Metric | aerial-05 | aerial-08 |
|---|---:|---:|
| Native poses, finite and chronological | 1,120 | 1,118 |
| Completed maps / inspection tails | 50 / 2 | 43 / 2 |
| Normal coverage handovers / recovery resets | 50 / 0 | 43 / 0 |
| Maximum successful-update gap | 0.320 s | **1.712 s — fail** |
| Maximum pose speed | 4.936 m/s | 4.268 m/s |
| Median occupied area | 4,028 m² | 4,011 m² |
| Minimum measured overlap ratio | 0.695 | 0.560 |
| Median member interval | 2.500 s | 4.480 s |
| Median / maximum scan processing time | 17.75 / 79.47 ms | 19.58 / 75.75 ms |
| Mapper peak RSS | 456.60 MiB | 448.50 MiB |

All 97 snapshots passed membership, anchor-transform, payload-hash, footprint,
first-observation and causal overlap checks. The 93 complete maps supplied 93 BEVs;
four tails were excluded. The recorded correspondence ages stay within the active
map's membership. All three Rerun files verify. Live analytics captured 1,106
messages per robot (98.75% / 98.93% of native updates); the native JSONL retains
every processed pose and is the source of stability measurements.

The aerial-08 gap spans native scans 277–293 (29.164–30.876 s after the first
initialized scan), with 15 intervening unsuccessful updates and zero eligible
features. Active submap 6 remained fixed throughout. Its handover occurred at
28.524 s; this observation does not rule out insufficient map maturity. The
cause of the matching loss remains unresolved. No correspondence threshold was
relaxed and no parameter sweep was performed.

**Diagnostic backend.** After preserving the failed stability verdict, the saved
maps completed MapClosures → distributed PCM → CBS in 12.814 s. Four proposals,
all intra-robot on aerial-08, failed the unchanged initial-overlap gate (overlap
0.0109–0.0263 versus required 0.10). Their transported and registration geometry
remained at 0.4 m, with 21,687–29,821 points, proving that the old 16k cap did not
silently coarsen them. Zero loops reached PCM; no loop registration factors were
created, and the robots remained disconnected. The synthetic integration tests,
rather than this zero-loop smoke, exercise accepted inter-robot PCM/CBS factors.
No ATE is reported.

All 93 rendered density images exactly reproduce cached MapClosures features.
Median feature counts are 105 / 86, with no map below five features. See the
[BEV gallery](figures/coverage-submaps/implementation/smoke/bevs/index.html) and
[overview](figures/coverage-submaps/implementation/smoke/bevs/overview.png).
The initial area thresholds often produce shorter maps than the temporal policy;
they do not guarantee better descriptors, contiguous scene coverage, or mature
odometry geometry. Coverage also precedes the backend's unchanged 80 m range
filter, which can remove some counted area.

**Retained evidence.** [Validation totals](figures/coverage-submaps/implementation/verification-results.json),
[native test log](figures/coverage-submaps/implementation/native-tests.log),
[frontend measurements](figures/coverage-submaps/implementation/smoke/frontend-summary.json),
[backend report](figures/coverage-submaps/implementation/smoke/report/report.json),
[source/config hashes](figures/coverage-submaps/implementation/smoke/source-hashes.json),
trajectories, images, Rerun recordings and source snapshots are retained under
`research/figures/coverage-submaps/implementation`. The
[retention manifest](figures/coverage-submaps/implementation/manifest.json)
verifies copied files. Original compressed member geometry and prepared clouds
remain at `.ros2/coverage-submaps-20260920/smoke` with an external geometry hash
manifest. No full four-flight coverage benchmark on 148 has been run.

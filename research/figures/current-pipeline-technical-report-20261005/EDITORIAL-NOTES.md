# Claim–evidence map and manuscript review

## Manuscript scope

The manuscript presents one validated pipeline: point-cloud multilayer retrieval on mature fixed circles, PCM, persistent distributed CBS with GPU VGICP factors, and submap reconstruction. Main results are from the simultaneous GRACO all-14 run. Immediate freezing is the focused publication-policy ablation. Separate native ground-trio communication measurements provide context with explicit workload limits.

No S3E accuracy is attributed to this exact configuration. The report does not describe superseded temporal/trajectory-centered submap methods, ellipsoid backend variants, staged solver trials or development chronology. Original experiment artifacts remain preserved outside the manuscript. This editorial record is not a manuscript section.

## Outline and paragraph roles

1. **Abstract and map:** define the problem, method, supported result and measurement scope.
2. **Problem and system design:** motivate spatial support; define placement, geometry publication, matching, factors, persistent optimization and reconstruction. Equations define coverage, evidence membership and the evaluation quantity.
3. **Experimental protocol:** identify data, concurrent execution, calibration and alignment rules.
4. **Online results and ablation:** quantify connectivity/accuracy and isolate the geometry-publication question. Separate the final settled output from during-playback availability.
5. **Communication:** account for the entire application payload before comparison context.
6. **Computation:** define timer scopes before reporting growth trends, latency and resources.
7. **Limitations and conclusion:** delimit claims and restate the tested outcome without adding new evidence.
8. **Reproducibility appendix:** record effective settings, validation and artifact provenance.

## Claim–evidence map

| Claim | Retained evidence | Boundary |
|---|---|---|
| All 14 robots share a frame during playback | `audit.json:timeline`, `evaluation.json:components` | Before the last bag ends, not before every bag ends |
| Final joint ATE is 0.933 m | `evaluation.json:joint_ate/aerial01` | One shared alignment, 45,397 matched poses, post-playback settling |
| 366 immutable circle endpoints meet the occupancy target | `audit.json:geometry`, `configuration.json:circle_cover` | Grid coverage is not evidence completeness; fixed capture union is not every raw scan |
| Maturation improves connectivity in this test | `immediate-freeze/` and main audit | One independent fresh replay per policy; not exactly frozen input or a universal causal claim |
| 110 admitted geometry pairs retain final support | `registration.json`, `pcm.json` | 111 pose loops; one geometry pair fails the initial gate; PCM singleton support is limited |
| Application traffic is 546.77 MB | `audit.json:communication` | Fan-out counted; no DDS/IP overhead or physical-radio claim |
| Communication compares favorably to DCL/DiSCo by assigned robot | `baseline-communication.json` | Different teams/workloads; Swarm uses less per assigned robot |
| Native local updates remain inexpensive at the tested graph sizes | `cbs-timing/raw-timing.json`, `summary.json`, CSVs | 842 nonterminal updates, maximum 61 local variables; not an asymptotic or end-to-end latency bound |
| Online pipeline RSS peaks at 42.34 GiB | `resources.json` | Simultaneous all-process RSS; shared pages may repeat; no per-robot/Jetson inference |
| Joint map has 13,353,004 representatives | `exports/export.json` | Visualization and readback validation are not geometric accuracy metrics |

## Five-dimension review

- **Clarity:** method equations and endpoint/time contracts distinguish spatial placement from pose optimization and evidence publication. Acronyms and timer scopes are introduced in context.
- **Structure:** one method precedes its protocol, result, focused ablation, cost analysis and limitations. No chronological experimental diary remains in the manuscript.
- **Evidence:** tables are regenerated from retained JSON; joint RMSE is checked against sample-weighted per-robot squared errors; all traffic terms sum exactly. Historical accuracy is not reused as current evidence.
- **Reproducibility:** the package includes effective per-robot configurations, runtime source/binary hashes, evaluation records, raw timing records and an offline build recipe. Bulk captures stay on 148. Hardware inventory is labeled post-run, not a contemporaneous utilization record.
- **Presentation and calibration of claims:** all 11 pages were rendered for visual review. Fonts, tables and chart axes remain readable. Scope limits accompany accuracy, baseline normalization, timing omissions and resource measurements. There are no claims of universal superiority, bounded lifelong graph size, physical-network performance or Jetson validation.

## Remaining scientific evidence needs

Matched-team baseline accuracy and communication under this exact setup, repeated executions, longer-horizon growth, and additional datasets remain necessary for broader paper claims. They are not supplied by the current report and do not justify carrying forward deprecated results.

# Persistent CBS+ implementation verification

The supplied draft's CBS+ backend is implemented as an opt-in mode across
`cbs`, `cbs_ros` and the multi-robot pipeline. The optimizer, received beliefs,
outgoing damping histories and native geometry remain alive as new graph
prefixes arrive. The existing sealed execution mode remains the default.

The [implementation specification](../../PERSISTENT-CBS.md) maps the draft's
algorithms to the code and documents configuration and limitations. Use
[`persistent_pipeline.yaml`](../../configs/persistent_pipeline.yaml) for the
live online launcher. Benchmarks and geometry remain on workstation 148 under
`/data3/mikexyl/swarm_s3e_ws/src/.ros2/persistent-cbs148-20260928`.

## Correctness checks

The tests exercise fixed Hellinger damping, lower/upper divergence gates,
first-message acceptance, per-recipient history, delayed anchor attachment,
retained beliefs across graph growth, correct pose/anchor cavity exclusions,
and cache invalidation. Native registration tests retain the exact factor
object and scale between revisions and reject changed endpoint metadata.

The DDS tests use three independently running robots. They add odometry,
connect the robots later, grow the graph, repeat an unchanged geometry revision,
and finish through an explicit final seal. They check stable process IDs,
increasing iteration counts, metric frame recovery, GPU evidence reuse, final
geometry support and clean shutdown. Suspending one robot's native process
also checks that other optimizers continue updating. A separate case changes
a PCM clique and verifies a fail-closed stop when a previously admitted loop
would be revoked. The live-capture adapter itself is tested with growing input.

Legacy sealed GPU tests run in normal and reversed robot order. Observed results
and retained remote log locations are listed in [verification.json](verification.json).
The final remote test logs, compact replay results, configuration and session
manifest were retrieved on 2026-09-30. Source hashes match the implementation
being committed. A fresh local precommit check passed 24 tests, with four DDS
tests skipped locally; distributed verification is recorded separately.

## Retained GRACO graph replay

This is a backend smoke test over three previously saved causal prefixes for
Aerial 05, 07 and 08. It uses the draft's requested 1 Hz local timer and unchanged
PCM/registration thresholds. It does not replay sensors, use ground truth, or
measure a new ATE. Prefixes are held for 10 seconds before the next input;
there are 10 final settling updates after the last admission.

| Original prefix | Poses | PCM-retained loops | Live GPU factors |
|---|---:|---:|---:|
| 001 | 6 | 2 | 2 |
| 020 | 69 | 48 | 47 |
| 040 | 103 | 106 | 100 |

The final graph has one common reference across all three robots. Geometry
factors belong to their canonical owners; final counts are 50, 50 and 0. All
admitted geometry factors must pass final support checks for completion. The
saved session manifest identifies the same six processes throughout the replay.
These counts were verified again in `graco-verified` after the final
delayed-anchor fix. The repeat completed with one shared component, unchanged
native process IDs across all prefixes, and successful final geometry checks
for all robots. See [the compact replay record](evidence/graco-final-repeat.json)
and [tested source hashes](evidence/tested-source-hashes.json).

These validation runs share the workstation with other tests. Their timing is
smoke-test timing, not an isolated performance benchmark. A requested 1 Hz
cadence is not a guarantee that every update finishes within one second.

## Scope and limitations

- The optimizer remains active within one process lifetime; this is not a
  crash-recovery checkpoint mechanism.
- Factors and cached endpoint geometry still grow with the accepted map.
- PCM admission and geometry-ready barriers remain in place for new revisions;
  CBS stages themselves are independent and asynchronous.
- A PCM decision that retracts an already-used measurement stops the persistent
  session. The draft specifies additive graph growth but no belief-retraction
  protocol. Such a run must not be reported as successful.
- No default-mode switch, full 14-robot experiment, new accuracy comparison,
  parameter sweep, or performance claim is part of this implementation check.

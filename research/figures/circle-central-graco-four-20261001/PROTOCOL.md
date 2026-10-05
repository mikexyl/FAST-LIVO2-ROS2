# Fixed spatial submaps and a centralized graph

This opt-in prototype extends `circle_cover.py` to the native point-cloud backend. EllipseLIO odometry and the normal distributed pipeline are unchanged. The selected experiment uses GRACO A05, A07, G05 and G06 on 148, without CPU/RAM/GPU quotas. The circle profile is 40 m radius, 99% occupied-cell coverage, 1 m grid, and 30% greatest-overlap target. Coverage takes priority over overlap.

## Geometry and graph frames

Placement replays only the growing union of retained native area snapshots. It does not contain every original scan. One fixed gravity-horizontal transform, taken from the first snapshot, is used for each robot. Stable point IDs deduplicate revisits and audit native coordinate consistency. Robot poses are not supplied to circle placement.

The footprint center is independent of the graph anchor. Each new circle gets an immutable endpoint `(robot_id, keyframe_id)` and the native IMU pose from its creation update as its anchor. Several circles may share an anchor timestamp. Same-robot nodes are joined in creation order by the existing odometry relative-pose constraints and configured noise. This is a graph of submap poses, not one rigid alignment per robot.

Geometry is frozen **once at the end of each capture**: every observed native representative within a circle's horizontal disk is included, with no height or age restriction. The payload is expressed in the creation anchor's IMU frame. The creation timestamp, graph-pose timestamp and geometry availability timestamp are separate. Geometry/descriptor availability is the seal timestamp, never retrospectively the circle's birth. Subsequent geometry revisions and online freezing are deferred. These runs therefore cannot demonstrate online publication latency.

Schema 6 (`strategy: fixed_spatial`) validates body/IMU frames, fixed horizontal transform, finite coordinates, immutable payload hashes, exact native point-ID membership, and disk containment. Existing area/temporal schemas remain intact. Overlapping same-robot native point-ID sets cannot supply independent loop measurements; the 30-second exclusion also remains.

## Backend and controls

Native point-cloud density BEVs, gravity alignment, five terrain-relative ORB layers, native HBST retrieval, pooled SE(2) matching, geometric vertical initialization and point-GICP verification retain the default settings. Full-height BEVs are saved for inspection. The legacy retrieval policy still uses terrain layers; a terrain support failure is visible and supplies no layered features.

All final sealed maps are published before staged retrieval. Each endpoint is queried once against previously indexed endpoints in deterministic robot/ID order. The existing two-candidate budget per query is retained and endpoint pairs are verified once. There is no online wall-time cooldown in this sealed batch. This scheduling is identical for the controls and is explicitly different from the normal online worker schedule.

The existing centralized pose solver applies component initialization, GNC-TLS loop selection and a selected-loop LM refit. The isolated native batch solver then keeps those pose constraints and adds the existing GLIM GPU VGICP wrapper to retained loops. Its immutable per-endpoint geometry, two adaptive voxel levels, fixed conservative information cap and final support/observability checks are reused. No CPU fallback is allowed for GPU mode. This centralized experiment replaces PCM/CBS; it does not measure distributed belief exchange or CBS cycle time.

This experiment sets `pgo.alignment_min_support: 1` to honor the user's singleton-loop policy: a verified singleton bridge can initialize a robot-pair alignment. The inherited two-distinct-endpoint gate disconnected the first circle graph despite verified ground bridges. This setting affects only the new experiment configuration; BEV, verification, factor-admission and global defaults remain unchanged.

The primary comparison uses identical 40 m radii:

- Incremental fixed circles, 99% occupied-cell coverage target.
- Trajectory-centered disks at the original retained snapshot cadence.

Both use the identical input union and end-of-capture geometry sealing. The trajectory control is allowed to miss 99% coverage; silently adding off-trajectory disks would change that method. An additional 80 m trajectory control restores the historical radius under the same centralized protocol. It separates radius from placement effects and is not a reproduction of historical CBS communication.

## Communication and computation

Each robot has a read-only evidence service. The server receives one serialized manifest containing poses, membership and ORB descriptors; requests selected geometry on demand; caches each endpoint; and sends optimized poses back. Verification geometry is reused for native registration preprocessing. No server-side direct read of another robot's original evidence store is used by this protocol.

The ledger records actual request/reply bytes, including session/configuration fields and the multiprocessing `Connection` length prefix, once per transfer. JSON/base64 array serialization deliberately reuses the existing evidence contract. These measurements describe this prototype's real application IPC streams, not Ethernet captures or estimates from compressed NPZ sizes. TCP/IP/DDS headers, frontend traffic and inspection-map downloads are excluded. This is an unoptimized centralized protocol; descriptor/member metadata can be repeated inside evidence packets.

Preparation, retrieval, verification/geometry transfer, pose selection, registration preparation and native optimization have separate timers. Parent/child CPU time is measured; peak RSS is the actual simultaneous process-tree sum sampled every 20 ms, not a sum of unrelated process peaks. RSS can count shared pages more than once. Whole-device GPU memory is sampled every 100 ms and includes unrelated activity/driver overhead; native endpoint counters cover geometry only. Neither memory estimate is inferred from point count.

The retained final comparison corrects the singleton initialization gate using `circle_central_graph_replay.py`. Sealed retrieval does not consume graph solutions: it reuses immutable preparation, retrieval/verification outcomes and evidence transfers, then runs a fresh graph and actual pose-delivery session for every control. Its headline time is a sum of separately measured capture stages and corrected graph stages, **not a new end-to-end latency measurement**. Resource tables for that final run cover the fresh graph stage only; full original capture resources remain separately retained. Communication combines actual pre-graph messages with actual corrected session/solution messages and excludes obsolete initializer delivery/shutdown messages. The report labels these scopes and does not add unrelated resource peaks.

Evaluation uses the retained full native trajectories, checked against the prepared anchor poses. Ground truth is read only by the evaluation module. GRACO provides IMU-reference positions; no additional lever-arm transform applies to this IMU estimate. Evo 1.36.5 associates timestamps within 50 ms and applies one rigid SE(3) alignment per connected component, with no scale. Disconnected components are not pooled.

Dense trajectory reconstruction averages correction poses for circles born at the same timestamp, then interpolates the resulting correction field. The maximum disagreement at each shared birth time is recorded. Map reconstruction uses each optimized submap pose: a native point is assigned to its closest circle center and emitted once. Points outside every footprint retain the nearest-circle correction and an explicit `uncovered` flag. They are never silently omitted or described as covered.

## Reproduction

Use `research/configs/circle_central_graco_four.yaml`, the retained native stores on 148, Python's existing research environment and the newer native MapClosures adapter with `include_image` support. A CUDA 12.9 / architecture 120 isolated batch build is required on 148. Orin requires its own ARM64/architecture 87 build.

```bash
cmake -S research/adapters/gtsam_points -B /fresh/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/gtsam-4.3 \
  -DGTSAM_POINTS_SOURCE_DIR=/path/to/pinned/gtsam_points \
  -DS3E_REGISTRATION_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DCMAKE_INSTALL_PREFIX=/fresh/.ros2/mixed-pgo-install
cmake --build /fresh/build --parallel
cmake --install /fresh/build
```

The build manifest at `bin/build.json` must contain binary/library hashes, the output of `--version`, and all adapter source hashes; `native_provenance` checks them before execution. `scripts/build_mixed_pgo.sh` creates that manifest automatically: set `S3E_MIXED_ROOT=/fresh`, `S3E_GTSAM_PREFIX`, `S3E_REGISTRATION_CUDA=ON` and `S3E_CUDA_ARCHITECTURES=120` before running it. It supports isolated checkouts regardless of their directory name. The experiment retains its isolated manifest under `.ros2/circle-central148-20260930/native-install/bin/` on 148.

```bash
PYTHONPATH=/path/to/new/native-mapclosures:research python -m s3e_pipeline.central_circle \
  --config research/configs/circle_central_graco_four.yaml --mode circle \
  --source /fresh --output /fresh/circle
PYTHONPATH=/path/to/new/native-mapclosures:research python -m s3e_pipeline.central_circle \
  --config research/configs/circle_central_graco_four.yaml --mode trajectory \
  --source /fresh --output /fresh/trajectory
PYTHONPATH=research python -m s3e_pipeline.circle_central_evaluation \
  --config research/configs/circle_central_graco_four.yaml \
  --run circle40=/fresh/circle --run trajectory40=/fresh/trajectory \
  --output /fresh/inspection
```

For a policy-only replay of a frozen capture, use a new directory:

```bash
PYTHONPATH=research python -m s3e_pipeline.circle_central_graph_replay \
  --config research/configs/circle_central_graco_four.yaml \
  --capture /fresh/immutable-detection-capture --source /fresh --output /fresh/corrected-graph
```

Use fresh output directories. Retain geometry on 148; retrieve the inspection gallery, numeric report, configuration/source/input hashes and test logs. Do not call a run successful if native final geometry checks fail. All three final controls connect the four robots and pass native final checks. Detailed results, the singleton-policy correction and exact measurement scopes are recorded in `RESULTS-CIRCLE-CENTRAL.md`.

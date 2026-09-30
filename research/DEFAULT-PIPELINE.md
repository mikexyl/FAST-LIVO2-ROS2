# Default evaluation pipeline

Point-cloud pipeline selected by the user on 2026-09-26; updated to the requested
GLIM GPU registration path on 2026-09-28. New evaluations use
[`configs/default_pipeline.yaml`](configs/default_pipeline.yaml) unless another
variant is requested explicitly.

1. Updated EllipseLIO with persistent-map odometry. Odometry submapping and
   accumulated-area odometry cropping are disabled.
2. Accumulated-area backend snapshots: all stored native map representatives
   within an 80 m horizontal radius, without an age or vertical range cutoff.
   The existing 20 m / 10 s snapshot cadence controls publication, not membership.
3. Gravity-aligned **point-cloud BEVs**. Full-height images and terrain-relative
   layers use the stored point representatives directly, without ellipsoid surface
   sampling. These are processed map points, not full-resolution raw LiDAR scans.
4. Multilayer MapClosures: pool distinct layer matches and estimate joint SE(2)
   consensus, then initialize height from geometry and verify with point GICP.
   The profile retains the benchmarked `retrieval_channels: legacy` policy:
   layered retrieval, with full-height images retained for inspection. It does
   not silently enable the separate experimental dual-channel retrieval policy.
5. Distributed PCM followed by CBS with **live GPU VGICP registration factors** for
   retained loops, alongside their pose constraints. Existing thresholds, evidence
   sampling, singleton-loop policy and information cap remain unchanged. The GPU
   factor uses GLIM's two-level adaptive voxel configuration and iSAM2
   relinearization threshold/caching policy. Point-GICP loop verification remains
   unchanged. See [GPU build and factor details](GLIM-GPU-CBS.md).
6. Assemble the map using **each submap's optimized CBS pose**. Correct dense
   trajectories using the existing anchor-pose correction procedure. This is not
   a single rigid transformation of each robot's entire map.

## Starting an evaluation

Copy the profile into a fresh experiment directory. Set the dataset, robot IDs,
output directory, ROS domain and dataset-specific evaluation fields. Use the
current mapper build and each robot's calibrated frontend configuration; preserve
its sensor extrinsics and noise settings. The example profile's S3E settings must
not replace GRACO ground/aerial calibration. GRACO evaluation uses each robot's
own LiDAR/IMU calibration and the released IMU reference trajectory; the unknown
S3E IMU-to-RTK offset remains an explicit evaluation limitation.

`scripts/recent_submaps/upstream_area_batch.py prepare --work NEW_DIRECTORY`
now selects this profile for its four supported S3E groups. Use `--backend-config`
to select a different complete profile. It applies `odometry.native_mapping` to
the calibrated frontend configuration. Other dataset launchers must copy these
defaults into their new, resolved experiment configuration; dated launchers that
replay frozen provenance intentionally retain their historical settings.

For an individual calibrated capture, `run_trial.py` needs both `--area-maps`
and `--persistent-odometry`, plus the current mapper executable/library. For
completed native snapshots, `python -m s3e_pipeline.recent_submaps --source
SNAPSHOTS --output NEW_PREPARED_DIRECTORY --config EXPERIMENT_CONFIG` prepares
point descriptors directly. Omitting `--config` selects the canonical profile.
The online snapshot preparer uses the same representation selection when given
that resolved configuration. Use the existing distributed `dpgo.run` backend and
`scripts/recent_submaps/rollout_report.py` for optimization and per-submap map
assembly; the older `s3e_pipeline.cli` consecutive-keyframe workflow is separate.

The `prepared-ROBOT/ellipsoid/` directory name is retained for compatibility with
the existing endpoint/artifact contract. Descriptor packets and preparation
summaries identify the actual representation as `point_cloud`; the directory
name is not evidence of ellipsoid rendering.

## Explicit alternatives and reproducibility

The complete previous CPU profile is preserved as
[`configs/cpu_gicp_pipeline.yaml`](configs/cpu_gicp_pipeline.yaml). GPU runs require
the CUDA CBS install (`.ros2/dpgo-gpu-install`, or explicit `CBS_OVERLAY`) and fail
if CUDA is unavailable. This change does not alter the frozen CPU communication
campaign or its reported measurements. The matched GPU checks are documented in
[the validation report](figures/glim-gpu-cbs148-20260928/REPORT.md).

Existing configurations without `backend.mapclosures.representation` preserve
their old ellipsoid behavior. Historical area/temporal profiles and frozen
results are unchanged. To choose sampled ellipsoid BEVs, set `representation:
ellipsoid` and `renderer: sampled_surface`; retain the exported-basis tolerance
appropriate to those snapshots. For the native full-ellipsoid backend, merge
[`configs/full_ellipsoid_backend.yaml`](configs/full_ellipsoid_backend.yaml),
which explicitly selects ellipsoid evidence and factors. Label alternatives in
reports rather than calling them the default.

Freeze source/configuration hashes and use new output directories. Evaluate with
evo 1.36.5, 50 ms association, rigid alignment and no scale fitting, with shared
alignment per connected component. Keep bulk geometry and recordings on 148,
without artificial resource quotas. Changing the default does not rerun or
relabel earlier experiments.

The initial default selection passed 29 focused native checks on 148, covering
direct point-descriptor agreement with the ablation computation, immutable area
membership, gravity/anchor metadata, multilayer matching and legacy ellipsoid
compatibility. The tested source copy and JUnit results are retained at
`/data3/mikexyl/swarm_s3e_ws/src/.ros2/point-cloud-default-20260926/` on 148.
This configuration change did not launch a new dataset benchmark.

## Online CBS update cadence (2026-09-28)

New online runs capture a changed causal graph prefix every **10 seconds**, using
the common wall clock of descriptor availability (`dpgo.update_interval_s`).
The former scheduler waited for a complete solve and then waited another 30
seconds; its five revisions were not five solver iterations.

Input capture now continues while CBS runs. Every captured revision is queued
and processed in order, including the final tail. Geometry is hard-linked,
not copied. Each revision retains distributed PCM, the registration exchange,
and the existing 100-iteration settling budget. A slow solve therefore causes
backlog instead of silently dropping graph updates. This is a 10-second **input
cadence**, not a claim of 10-second optimization latency. `epochs/events.jsonl`
and each `publication.json` record actual capture, queue wait, publication delay,
and missed capture deadlines. Reports must include these achieved timings.

This setting applies to `online_epochs`; a single frozen `dpgo.run` is still one
solve and must not be labelled as a 10-second online run. Historical captures
and configuration files remain unchanged.

An opt-in long-lived CBS+ mode is available in
[`persistent_pipeline.yaml`](configs/persistent_pipeline.yaml); see
[`PERSISTENT-CBS.md`](PERSISTENT-CBS.md) for the draft mapping, online entry point,
requested 1 Hz iterations, and the fail-closed PCM-retraction limitation.
The historical sealed execution mode remains the default.

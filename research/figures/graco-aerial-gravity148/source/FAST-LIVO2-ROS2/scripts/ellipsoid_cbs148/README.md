# EllipseLIO + ellipsoid MapClosures + PCM/CBS on 148

The fixed pipeline is **EllipseLIO → native ellipsoid surface BEVs → MapClosures → distributed PCM → CBS**, retaining the working live GICP factors. There is no centralized optimizer, MegaLoc, raw-BEV retrieval branch, sweep, or ground-truth input to estimation.

The queue discovers S3Ev1/v2 sequences already under `/mnt/data/s3e`, excludes Playground 1 per the prior user instruction, and uses each version's published LiDAR/IMU calibration. Odometry replays serially at 1x; the three isolated DDS loop workers and three native CBS optimizers run concurrently. One GPU job at a time renders ellipsoid surfaces. No CUDA MPS is needed for this serial GPU workload.

Native snapshots are selected causally from the current export at 1 m / 10 degrees / 2 seconds, after `MapIncremental`. Python independently checks every selected timestamp. The native estimator update equations are unchanged. The persistent fitted map is cropped at 80 m; surface spacing is 0.125 m, voxel size 0.25 m and density pixels 0.5 m. Verification still uses the raw trailing five-second cloud submap. Only ellipsoid descriptors enter retrieval.

A bounded SVD projection removes small float tensor-basis defects (maximum Gram error 0.001); reflection parity, centers and axes are retained. Larger defects fail the sequence. Adjustment magnitudes are recorded per keyframe. This prevents harmless native float roundoff from triggering the earlier overly tight orthogonality check without accepting materially distorted geometry.

PCM probability is 0.99 and the minimum clique size is two. Actual peer verdicts, retained/rejected loops and serialized communication are saved. The inherited live GICP configuration remains enabled. Synthetic native DDS tests exercise false closures, unknown robot alignments and disconnected components.

Evo 1.36.5 evaluates raw per-robot odometry and CBS output. CBS uses one shared rigid alignment per connected component, 50 ms association and no scale fitting. Missing timestamped GT is reported as unavailable, while maps and graph diagnostics remain available.

## Persistent execution

Remote workspace: `/data3/mikexyl/swarm_s3e_ws/src`.

Container `ellipsoid-cbs148-overnight` uses Docker's `unless-stopped` restart policy and a bind-mounted queue state. It survives SSH disconnection. On an external restart, completed/failed sequences are skipped; interrupted attempts remain archived and a fresh attempt is created. A file lock prevents duplicate queue controllers. Source, binary and configuration hashes are frozen at launch; a change stops the queue before another sequence. Timeouts terminate owned descendants, record a failure, then continue. Less than 100 GiB free stops the queue before another sequence.

```bash
ssh 148 python3 /data3/mikexyl/swarm_s3e_ws/src/FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/status.py
ssh 148 docker logs --tail 40 ellipsoid-cbs148-overnight
ssh 148 docker stop --time 60 ellipsoid-cbs148-overnight
```

Results: `.ros2/ellipsoid-cbs148/overnight-20260916/RESULTS.md`. Each attempt has its own logs, status, source/configuration hashes, native outputs, PCM decisions, evo evidence, report, plots and Rerun recording. Exact scan/pose exports from valid completed frontends are retained under each attempt’s `shared-odometry/` for the pending matched Swarm-SLAM comparison. Optional ellipsoid references are removed from that derived sensor-only manifest; sensor bytes, integer timestamps and poses are unchanged. Ellipsoid/submap/image caches and failed-run clouds are retired after evidence verification. Descriptors, selected BEVs, frame indices and poses remain. Incomplete geometry cannot pass cache validation.

## Isolated build

For a fixed parameter trial, pass `--config` with a complete experiment YAML and a fresh `--base` to `entrypoint.sh`. Optional `odometry.imu_noise` overrides all four native noise values after calibration loading; `odometry.native_sensor_qos: true` omits `input.reliable` from generated mapper YAML; `odometry.record_analytics: true` saves native diagnostics. Set the container's `ROS_DOMAIN_ID` to the config's `dpgo.ros_domain_id` to isolate concurrent experiments. Without these options, the existing queue behavior is preserved. The [requested S3E noise trial](../../research/RESULTS-S3E-IMU-NOISE.md) records its failed result and actual runtime configuration.

`build.sh` reuses pinned GTSAM 4.3 and builds CBS, cbs_ros, EllipseLIO, MapClosures and its inspection adapter. GTSAM Python 4.2 and Rerun 0.37.1 stay in separate environments. The source SDK 0.35 used by the disabled native CBS visualization is independent of the report recording version. The existing Humble image needs `libgraphviz-dev`; its obsolete `/usr/local/include/gtsam` directory is moved out of the include search path to prevent mixing GTSAM headers. Host ROS and unrelated services are unchanged. The deployment image ID and dependency provenance are retained with the run.

# Live EllipseLIO → Rerun

`../ellipselio_live_rerun.py` subscribes to a running EllipseLIO ROS node and writes
Rerun 0.37.1 data concurrently to a `.rrd` file and a gRPC stream. It does not read
saved poses or reconstruct scans from a bag. A ROS bag may provide the live
estimator's input.

## Native execution

Build this small launcher after sourcing ROS and the existing EllipseLIO install:

```bash
cmake -S FAST-LIVO2-ROS2/scripts/ellipselio_live -B .ros2/ellipse-live-launcher
cmake --build .ros2/ellipse-live-launcher -j2
```

It loads the installed `libellipselio_mapping.so` with a four-thread ROS executor.
Upstream's standalone launch also uses `component_container_mt`. No estimator
library is rebuilt. The existing default executable is unchanged.

This matters when `publish.map` is enabled: the native `PublishMap` callback sends
100 chunks with 100 ms sleeps for a 10-second map interval. A single-threaded
executor cannot process sensor callbacks while that publisher sleeps.

Use the robot's normal calibration/noise configuration with these publishers:

```yaml
publish:
  map: true
  scan: true
  markers: true
  odometry: true
  analytics: true
  tf: true
```

Start the recorder in a ROS-sourced Rerun 0.37.1 Python environment. Supply the
original bag start in integer nanoseconds and a new output directory:

```bash
.ros2/rerun-venv/bin/python FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py \
  --robot Bob --start-ns 1689768347996319770 \
  --output .ros2/bob-live-recording --grpc-port 9876
```

Wait for `READY`, then run the frontend with the same ROS domain:

```bash
bash FAST-LIVO2-ROS2/scripts/run_ellipselio.sh \
  --robot Bob --bag /data/s3e/S3Ev2/S3E_Library_2 \
  --mapping-config /path/to/Bob.yaml --output /path/to/new/frontend \
  --rate 1 --no-research-export \
  --mapper-executable .ros2/ellipse-live-launcher/ellipselio_mapping_mt
```

`--no-research-export` disables the separate MCAP/ellipsoid research exporter and
its completion service. This is intentional for native online visualization:
with concurrent IMU input the native LiDAR update can choose a different IMU
reference timestamp, triggering the research exporter's strict deskew/pose
association assertion. The assertion is preserved for research exports. This
live capture is not interchangeable with a frozen offline experiment.

Connect a viewer with `rerun rerun+http://127.0.0.1:9876/proxy`. After replay ends,
send SIGINT to the recorder to flush and close the file, then run
`rerun rrd verify /path/to/recording/live.rrd`.

## Recorded data and limits

- Native processed scans and chunked maps, already in the robot odometry frame.
- Exact native post-LiDAR TF poses, saved also in `post_lidar_poses.tum`.
- IMU-propagated odometry velocity/covariance and raw IMU diagnostics.
- Native residuals, feature counts, observability scores, and processing times.
- Native ellipsoid markers: upstream publishes a sparse subset, approximately
  one per 100 newly added map points; these are not the entire fitted map.

Display sampling caps scans at 6,000 points and each map chunk at 1,000 points.
Output ROS topics use best-effort QoS. `status.json` reports received counts,
rejected frames, map snapshot chunk counts, timestamps, and the first pose-speed
spike above 20 m/s. A partial snapshot is not a complete map. Analytics messages
have no header; their timing uses the latest observed `/clock`. Covariance is
shown in native published order, without interpreting it as edge uncertainty.

The `elapsed` timeline is seconds from bag start. `received_elapsed` preserves
message receipt timing for inspecting delayed map updates. The earlier Bob
Library 2 research run crossed its motion guard near 444.9 seconds; live reruns
may differ because of callback scheduling and export overhead.

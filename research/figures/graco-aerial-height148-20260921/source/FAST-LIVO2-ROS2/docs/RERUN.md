# S3E visualization with Rerun 0.37.1

This setup uses [Rerun 0.37.1](https://github.com/rerun-io/rerun/releases/tag/0.37.1), the latest stable release verified on 2026-09-10. It installs the SDK and matching viewer under `src/.ros2/rerun-venv/`. The existing system Rerun executable is unchanged.

## Run and stop

From `/home/mikexyl/workspaces/fast_livo2_ws/src`:

```bash
# Persistent session, including the full Alpha replay and the viewer.
FAST-LIVO2-ROS2/scripts/run_s3e.sh --background --rerun --robot Alpha

# Stop the managed replay, bridge, mapper, and viewer.
FAST-LIVO2-ROS2/scripts/run_s3e.sh --stop

# Alternatively, keep the supervisor in your terminal; Ctrl-C stops it.
FAST-LIVO2-ROS2/scripts/run_s3e.sh --rerun --robot Alpha
```

Bob and Carol use `--robot Bob` and `--robot Carol`. `--duration 30` bounds playback to 30 bag seconds. `--rerun-headless` records both formats without opening a window. Fast DDS is explicitly selected, with localhost networking and ROS domain 73 by default. The viewer listens on localhost port 9877; `--rerun-port` overrides it.

The persistent session's supervisor PID and console-log path are in `.ros2/active_s3e.json`. Each run saves its process IDs in `processes.json`. A successfully completed replay leaves the Rerun viewer open so you can inspect the recording. `--stop` also closes that viewer.

Closing the viewer during playback stops that replay and marks it incomplete. The MCAP and RRD recorded so far remain available; a severed viewer connection is handled during bridge shutdown.

## Official integration used

Rerun's current [ROS 2 guide](https://rerun.io/docs/howto/integrations/ros2-nav-turtlebot) describes a subscriber-based live integration; it does not claim a native DDS subscriber. Both older standalone official bridge repositories are archived or deprecated.

Here, ROS subscriptions forward serialized CDR messages in one-second MCAP batches through the official `rr.log_file_from_contents` importer. The complete stream is also saved as a ROS 2 MCAP. Rerun's [native ROS 2 decoders](https://rerun.io/docs/concepts/logging-and-ingestion/mcap/message-formats) provide:

- `PointCloud2` positions and extra fields, with `CoordinateFrame` from the ROS header.
- `TFMessage` as timestamped `Transform3D` parent/child frame relationships; `/tf_static` is static.
- `CameraInfo` as `Pinhole`, paired with the native image-plane frame of `CompressedImage`.
- IMU scalar series and GNSS `GeoPoints`.
- Schema reflection for odometry and additional fields, available for inspection.

The display uses `ros2_timestamp`, derived from sensor headers. MCAP log/publish times are also set to sensor time. The one-second batching introduces approximately one second of visualization latency; it does not change timestamps or mapper processing.

Two small display additions remain outside native decoding: odometry produces a breadcrumb trajectory, and PCL's packed float `rgb` field is reinterpreted as Rerun colors. The 0.37.1 importer retains that field but does not automatically assign it to `Points3D.colors`. The original cloud bytes remain intact in MCAP; positions and frames use the native decoder.

The SDK requires NumPy 2, so the bridge has an isolated environment. The ROS/OpenCV sensor adapter continues using the system Python and NumPy, avoiding the binary incompatibility with Humble's `cv_bridge`.

## Layout and calibration

- **Accumulated color map:** all registered colored scans and the trajectory up to the current timeline cursor.
- **Live camera and TF:** current cloud, camera frustum, and named frame axes.
- **Rectified camera / tracked features:** the mapper's annotated image, rectified for a consistent pinhole projection.
- **IMU / GNSS tabs:** native time series and geographic positions.

The visualization adapter rectifies a copy of `/rgb_img` and publishes `/s3e/camera/image_rect/compressed` plus `/s3e/camera/camera_info`. Intrinsics scale to the actual image size; distortion is removed and the output CameraInfo has zero distortion. The camera's static transform from `aft_mapped` (IMU) uses the same S3E extrinsics as the mapper. The mapper's input images and estimation remain unchanged.

## Saved data and setup

Each run adds `visualization.rrd`, `visualization.mcap`, `rerun_summary.json`, `rerun_bridge.log`, and `rerun_viewer.log` to the usual run directory. The summary reports message counts, native-import batches, queue backlog, and shutdown status. The RRD preserves the configured layout, trajectory, and color supplement. The MCAP preserves the selected ROS messages and can be opened with the native importer or other ROS tools.

```bash
# Open a saved visualization.
.ros2/rerun-venv/bin/rerun .ros2/runs/<run-directory>/visualization.rrd

# Install the pinned visualization environment if rebuilding elsewhere.
FAST-LIVO2-ROS2/scripts/setup_rerun.sh
FAST-LIVO2-ROS2/scripts/build_s3e.sh

# Focused color regression; native-decoder behavior is checked by replay.
source /opt/ros/humble/setup.bash
.ros2/rerun-venv/bin/python FAST-LIVO2-ROS2/tests/test_rerun_colors.py
```

The 20-second integration test in `S3E_Square_1_alpha_20260910_171048_737880` passed with 177 poses, 176 colored clouds, 176 rectified images/camera calibrations, matching dynamic TF counts, IMU and GNSS messages, and clean shutdown. Native RRD components for points, TF, pinhole calibration, images, IMU and geographic positions were inspected. The live viewer was also inspected using the official `rerun.experimental.ViewerClient.save_screenshot` API.

The longer live recording is `S3E_Square_1_alpha_20260910_171411_715470`. It contains about 435 bag seconds and 4,290 poses; the viewer disconnected before the end of Alpha's stream. All eight MCAP topics passed `rerun mcap check`. This run exposed the closed-viewer flush exception, which is now handled and covered by `tests/test_rerun_disconnect.py`. A screenshot of the working live layout is saved at `.ros2/rerun-live.png`.

# S3E Square 1 on ROS 2 Humble

This workspace extends the community [Robotic-Developer-Road FAST-LIVO2 Humble port](https://github.com/Robotic-Developer-Road/FAST-LIVO2/tree/humble). The original ROS 1 checkout remains in `../FAST-LIVO2`. [s3e.repos](../s3e.repos) selects this repository's `dev/fast-livo2-s3e-dpgo` branch and pins the modified dependencies, CBS and CBS ROS to published commits in the `mikexyl` forks. The Sophus revision is unchanged. CBS additionally requires the GTSAM/aria underlay documented in [DPGO.md](../research/DPGO.md).

## Run

For Rerun 0.37.1 visualization with native ROS 2 MCAP decoding, see [RERUN.md](RERUN.md).

From `/home/mikexyl/workspaces/fast_livo2_ws/src`:

```bash
# Full Alpha sequence with RViz (omit --rviz for headless playback).
FAST-LIVO2-ROS2/scripts/run_s3e.sh --robot Alpha --rviz

# A bounded test, measured in bag seconds.
FAST-LIVO2-ROS2/scripts/run_s3e.sh --robot Alpha --duration 60 --rate 1

# Other robots use their own camera and extrinsic calibration.
FAST-LIVO2-ROS2/scripts/run_s3e.sh --robot Bob
FAST-LIVO2-ROS2/scripts/run_s3e.sh --robot Carol
```

The default bag is `/data/s3e/S3E_Square_1`. `--bag /path/to/bag` overrides it. FAST-LIVO2 estimates one robot at a time; this setup does not fuse the three robots. Use one runner at a time because the upstream mapper shares output topics and debug files.

The runner sources ROS and the local install, waits for mapper subscriptions before playback, selects only that robot's LiDAR/IMU/left-image topics, publishes `/clock`, and stops its child processes on completion or Ctrl-C. It explicitly selects `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (Fast DDS), uses localhost, and defaults to `ROS_DOMAIN_ID=73` (an existing domain override is respected). Zenoh is installed on this machine but is not used by this runner. ROS 2 requires local DDS sockets; run outside environments that deny them.

Each run creates `.ros2/runs/S3E_Square_1_<robot>_<timestamp>/` containing:

- `trajectory.tum`: timestamp, IMU position, and quaternion (`qx qy qz qw`) in the gravity-aligned local world.
- `mapping.log` and `playback.log`.
- `mapping_config.yaml` and `camera_config.yaml`: copies of the settings used.
- `summary.json`: playback status, finite-pose checks, output counts, and trajectory continuity statistics.

The mapper publishes `/aft_mapped_to_init`, `/path`, `/cloud_registered`, and `/rgb_img`. The source copy of the trajectory is in `FAST-LIVO2-ROS2/Log/result/`. PCD accumulation is disabled by default; `/cloud_registered` carries the live map updates. The wrapper has no loop closure or multi-robot map merging.

## Build

```bash
FAST-LIVO2-ROS2/scripts/build_s3e.sh
```

Build, install, dependency-install, and colcon logs stay under `src/.ros2/`. The script explicitly selects the ROS 2 packages; an unrestricted `colcon build` over both FAST-LIVO2 checkouts would find duplicate `fast_livo` packages.

Prerequisites on this machine are Ubuntu 22.04, ROS 2 Humble, colcon, PCL, Eigen, OpenCV, Boost, fmt, `pcl_ros`, `pcl_conversions`, `cv_bridge`, `image_transport`, `demo_nodes_cpp`, and rosbag2. Sophus 1.22.10 is installed locally. Vikit uses the ROS 2 fork. Livox's original message definitions are built with `LIVOX_BUILD_MESSAGES_ONLY=ON`; no Livox hardware SDK is needed for this Velodyne bag.

The source dependencies live alongside this repository (`../Sophus`, `../rpg_vikit`, `../livox_ros_driver2`). Their revisions are in `s3e.repos`; the pinned Vikit and Livox forks already contain the required build fixes. Copy `livox_ros_driver2/package_ROS2.xml` to `livox_ros_driver2/package.xml` before building. The FAST-LIVO2 checkout itself contains the S3E integration and local fixes.

The files under `patches/` retain the adaptations for the original upstream
baselines: Vikit `4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50` and Livox
`4a1def929e5b59c7a8122d19fce6efba581ce9f7`. Apply them only when using those
unmodified baselines; the published fork commits already include them.

## Sensor configuration

The [S3E dataset](https://dapengfeng.github.io/S3E/) bag is already ROS 2 SQLite. No bag conversion or modification is required. This local sequence lasts approximately 460 seconds and includes Alpha, Bob, and Carol.

Calibration comes from `/data/s3e/Calibration/{alpha,bob,carol}.yaml`. Regenerate the configuration files with:

```bash
/usr/bin/python3 FAST-LIVO2-ROS2/scripts/generate_s3e_config.py
```

The dataset explicitly defines `Tic` as camera-to-IMU and `Tlc` as camera-to-LiDAR. Using destination/source notation:

```text
T_imu_lidar    = Tic * inverse(Tlc)
T_camera_lidar = inverse(Tlc)
```

`extrinsic_R/T` use `T_imu_lidar`; `Rcl/Pcl` use `T_camera_lidar`. The left camera uses the published pinhole intrinsics and four distortion coefficients, with FAST-LIVO2's internal image scale of 0.5 (1224×1024 becomes 612×512). No extra camera/IMU time shift is applied.

S3E clouds contain `x,y,z,intensity,ring,time`, with 16 rings. The float32 `time` is in **seconds relative to the original header**, often about −0.098 to +0.001 seconds. The adapter:

1. Moves the scan header to the earliest point.
2. Subtracts that offset from each point time, preserving absolute acquisition times.
3. Stably sorts the points by time and emits an unorganized cloud.
4. Decodes JPEG images to BGR8 without changing their timestamps.

`preprocess.velodyne_time_scale=1000.0` converts these seconds to FAST-LIVO2's internal milliseconds. The port's original default (0.001) is retained for other configurations. Reliable subscription QoS avoids losing fragments of the large sensor messages during playback.

The 16-beam configuration uses 2 m outer map voxels (subdivided by the existing voxel tree), image covariance 1000, a 0.2 m surface filter, 1 m blind radius, and a sliding local map. Larger outer voxels allow sparse distant surfaces to form planes; the reduced visual weighting helps with repetitive paving patterns. IMU covariance tuning starts from the port's Avia baseline. The dataset IMU noise densities are not copied directly into the algorithm's discrete covariance settings. This is an initial working setup, not an accuracy-tuned benchmark configuration.

## Local fixes and checks

- Buffered BGR8 images own their pixel storage after ROS callbacks return, preventing use-after-free crashes in OpenCV resize.
- On x86, the build keeps Eigen's SSE allocation convention compatible with Ubuntu's PCL libraries. Native AVX compilation caused an invalid free when destroying clouds allocated by PCL.
- A valid node is constructed before creating ImageTransport and reused by the mapper.
- Vikit libraries are linked through their CMake exports rather than hard-coded install paths. Its common package is built/exported with ament and the modern Sophus target.
- The squared LiDAR blind-range threshold is initialized from the configured range before preprocessing.
- Sensor output messages and TF use the estimated measurement timestamp rather than the current playback clock.
- The supervisor requests the bag player's best-effort `/clock` QoS, checks runtime failures, and saves separate results per run.

Run the timing regression checks after sourcing ROS:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 FAST-LIVO2-ROS2/tests/test_s3e_adapter.py
source .ros2/install/setup.bash
ctest --test-dir .ros2/build/fast_livo --output-on-failure
```

The C++ regressions verify the blind-range filter, point-time conversion, safe destruction of clouds allocated by PCL's voxel filter, and buffered image ownership after the ROS message is destroyed. The Python tests verify preservation of point data and acquisition times across organized clouds with padding, both byte orders, and rejection of unsupported timing fields/units. A sampled audit of 46 scans per robot across the full sequence also preserved acquisition times within 10 ns; observed scan spans were 0.0994–0.1010 seconds. Build and replay evidence is recorded in the local `.ros2/` logs and per-run summaries.

A saved run can also be plotted against the supplied GNSS positions:

```bash
/usr/bin/python3 FAST-LIVO2-ROS2/scripts/plot_s3e_trajectory.py .ros2/runs/<run-directory>
```

This writes `trajectory_check.png` and `gnss_position_check.json`. It fits a rigid transform without changing scale and excludes interpolation across large pose gaps. It is a position sanity check rather than a calibrated ATE benchmark: the GNSS antenna/IMU lever arm is not corrected, and the ground-truth file's identity quaternions are not used. Pass `--ground-truth /data/s3e/S3E_Square_1/bob_gt.txt` or `carol_gt.txt` for another robot.

## Validated on this machine (2026-09-10)

The final build passed both C++ regression executables and all four Python tests. All of these replays used Fast DDS at 1.5× and exited cleanly:

| Robot | Replay | Input clouds / images | Poses | Run directory under `.ros2/runs/` |
| --- | --- | --- | --- | --- |
| Alpha | Complete selected stream | 4,568 / 4,544 | 4,543 | `S3E_Square_1_alpha_20260910_162855_237755` |
| Bob | First 15 bag seconds | 128 / 127 | 107 | `S3E_Square_1_bob_20260910_162742_819632` |
| Carol | First 30 bag seconds | 278 / 278 | 273 | `S3E_Square_1_carol_20260910_162811_774314` |

Alpha's input counts match every selected cloud and image in the bag metadata. Its trajectory has finite values, strictly increasing timestamps, normalized quaternions, and a maximum adjacent pose displacement of 0.260 m. The estimated path length is 542.87 m. The GNSS position check matches 384 positions and gives 1.323 m RMSE, subject to the alignment and lever-arm limitations above. Its plot is `trajectory_check.png` in the Alpha run directory. Full-sequence validation of Bob and Carol has not been performed.

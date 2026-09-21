# aerial08: live EllipseLIO capture

Recorded directly from the running native ROS publishers. No saved poses or original-bag clouds are reconstructed.

Map/scan coordinates are the publisher's odom_ellipselio frame. Map chunks retain their common native snapshot stamp; receipt time is logged separately. Native map is published every 10 s in up to 100 chunks over roughly 10 s. A snapshot may be partial if best-effort output messages are lost or playback ends while chunks are in flight.

Display sampling: at most 6000 points per scan and 100000 per full map. Ellipsoids are the upstream sparse MarkerArray output (approximately one per 100 newly added map points), not the complete fitted map.

TF supplies exact post-LiDAR-update poses. Odometry supplies IMU-propagated velocity/covariance diagnostics. Analytics uses native sensor_stamp_ns when available; legacy publishers use approximate /clock timing. Covariance diagonals are shown in native published order; they are not relative-edge uncertainty.

Timeline elapsed = seconds from original bag start. The earlier run first exceeded 20 m/s near 444.9 s on this timeline; this rerun may differ.

# GRACO aerial pair: temporal EllipseLIO + ellipsoid BEV + PCM/CBS

**No common two-robot map was established. There were no accepted inter-robot loops; the CBS ATEs below use separate alignments for the two disconnected components. No joint two-robot ATE is reported.**

**Diagnostic result: aerial08 failed the successful-LiDAR-update gap criterion (1.384 s at the 30 s handover). Both captures completed, but this is not a passing frontend-stability trial. The original failed gate is retained; the backend ran with unchanged parameters to inspect the requested two-robot result.**

Local full-flight runs of aerial-05-40m and aerial-08-25m, treated as two independent robots. Both use the supplied aerial T_Imu_Lidar and IMU noise, reliable input, four-thread native launchers and separate ROS domains. Native temporal windows are 10 seconds with 5 seconds overlap.

Sensor staging preserved LiDAR/IMU CDR payloads and all original timestamps. Completed native member scans supply both ellipsoid descriptors and registration evidence. GT was made available only after optimization, for position evaluation using evo 1.36.5, 50 ms association and rigid alignment without scale. Raw fits are independent per robot; CBS uses a shared fit per connected component.

| Robot / flight | Raw ATE [m] | CBS ATE [m] | Max speed [m/s] | Max update gap [s] | Completed submaps |
|---|---:|---:|---:|---:|---:|
| aerial05 | 0.8272 | 0.8272 | 5.470 | 0.132 | 58 |
| aerial08 | 0.2187 | 0.0781 | 4.568 | 1.384 | 54 |

Connected components: [['aerial05'], ['aerial08']].
Shared-component CBS ATE: aerial05 = 0.8272 m, aerial08 = 0.0781 m.
Retained loops: 1; PCM rejected: 0; backend wall time: 12.22 s.

![Trajectories and native member-scan maps](report/trajectories-maps.png)

Both live recordings and the derived recording verified. The artifact audit checks descriptor/evidence membership, causal availability, anchor timestamps, same-robot exclusions and frozen sources. Full geometry, trajectories, evo output and logs are retained. This is one fixed-configuration trial, not an accuracy guarantee or parameter sweep.

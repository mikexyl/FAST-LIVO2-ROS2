# Square 2: measured Swarm-SLAM trajectory ATE

**Shared position ATE RMSE: 0.7359 m.** This is a measurement of the actual recorded Swarm trajectory; it is not a claim that the replay validation passed.

| Metric | Combined | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
| Dense corrected ATE (m) | 0.7359 | 0.9610 | 0.6813 | 0.5664 |
| GT-associated positions | 683 | 193 | 240 | 250 |

Evaluation uses evo 1.36.5, timestamp association within 50 ms, one shared SE(3) alignment across all three robots, no scale fitting and no additional per-robot alignment. Native optimized keyframe corrections are interpolated over the saved dense frontend odometry, using the existing evaluation protocol.

Direct evaluation of only the 1,403 native optimized keyframes gives **0.7066 m** over 121 GT-associated keyframes (Alpha/Bob/Carol: 12/52/57). This uses a different sample distribution and shared fit from the dense result; it should not be mixed with dense-trajectory comparison values.

The native result covers every observed keyframe. However, nine of 7,288 published scans were not acknowledged (Alpha/Bob/Carol: 2/3/4), and some selected timestamps differ from lossless replay. This affects the experimental comparison, not the mathematical ability to measure error of the resulting trajectory. The original replay failure and input audit remain unchanged.

The earlier report withheld ATE whenever that replay audit failed. That reporting rule was too restrictive: the value above is valid for this recorded run, while a controlled comparison still requires reliable replay. Square 2’s fresh paired BEV preparation also failed, so no paired accuracy winner is established.

GT orientations are placeholders and are unused; the antenna lever arm is uncorrected. This is position ATE, not full 6-DoF error.

The evo CLI independently reproduced the dense RMSE; it also matches the pooled per-robot squared errors. Ground-truth hashes match the archived inputs.

```bash
evo_ape kitti swarm/evo-actual-dense/Alpha-reference.kitti \
  swarm/evo-actual-dense/Alpha-estimate.kitti -a -r trans_part
```

[Machine-readable metrics and input hashes](swarm/actual-trajectory-ate.json) · [evo results](swarm/evo-actual-dense/Alpha-component-ape.zip) · [independent CLI reproduction](swarm/evo-actual-dense/cli-reproduction.zip) · [original replay audit](swarm/keyframe-input-audit.json)

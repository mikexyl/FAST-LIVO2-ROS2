#!/usr/bin/env python3
"""Render the retained frozen-graph GPU validation results, without recomputation."""
import csv
import html
import json
from pathlib import Path
import shutil
import statistics as stats

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PROJECT = Path(__file__).resolve().parents[2]
ROOT = PROJECT / 'research/figures/glim-gpu-cbs148-20260928'
PAPER = PROJECT / 'research/figures/s3e-graco-comparison-20260924'
NAMES = {'GRACO_aerial': 'GRACO A05/A07/A08', 'S3E_Library_1': 'S3E Library 1'}


def main():
    data = json.loads((ROOT / 'evidence/matched-v2/evaluated-results.json').read_text())
    entries = data['entries']
    rows = []
    for group, label in NAMES.items():
        for mode in ['gicp', 'vgicp_gpu']:
            chosen = [e for e in entries if e['group'] == group and e['variant'] == mode]
            ate = [next(iter(e['trajectory'].values()))['rmse_m'] for e in chosen]
            times = [e['wall_s'] for e in chosen]
            rows.append(dict(group=label, mode=mode, wall_median_s=stats.median(times),
                times_s=times, joint_ates_m=ate, factors=chosen[0]['accepted_factors'],
                max_rss_gib=max(e['simultaneous_process_tree_peak_kib'] for e in chosen) / 1024**2,
                whole_gpu_peak_mib=max(e['gpu_whole_device_peak_mib'] for e in chosen)))
    (ROOT / 'table.json').write_text(json.dumps(rows, indent=2) + '\n')
    with (ROOT / 'table.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    table = '| Group | Mode | Solve time, both repeats (s) | Joint ATE, both repeats (m) | Peak host RSS (GiB) | Whole GPU peak (MiB) |\n|---|---|---:|---:|---:|---:|'
    for row in rows:
        table += '\n| ' + ' | '.join([row['group'], row['mode'], ', '.join(f'{v:.2f}' for v in row['times_s']),
            ', '.join(f'{v:.4f}' for v in row['joint_ates_m']), f"{row['max_rss_gib']:.2f}", f"{row['whole_gpu_peak_mib']:.0f}"]) + ' |'
    text = '''# GLIM GPU registration path in CBS — validation on 148

The new default uses native CUDA VGICP registration factors and GLIM's relinearization/caching settings. All eight matched frozen-graph solves completed, retained one connected three-robot component, and passed the final geometry support checks. Median full-solve time improved from 42.42 to 19.91 s on GRACO (2.13×) and from 125.96 to 29.09 s on Library 1 (4.33×). Accuracy remained close across these controls; two repeats do not establish statistical equivalence.

'''+table+'''

The solve timer includes fresh-session preparation, PCM, registration exchange and the 100-iteration schedule at 10 Hz. This is a comparison of two CBS configurations, not a benchmark of GLIM itself or an isolated CUDA-kernel speedup. Odometry, frozen verified loop inputs, evidence budgets, conservative factor weighting and iteration budget are identical. Run order is CPU/GPU/GPU/CPU, serially per group. The RTX 5080/CUDA 12.9 measurements used no container resource quotas. Other authorized workstation jobs were not stopped, so these are workstation measurements rather than isolated hardware microbenchmarks.

## Accuracy and support

Dense trajectories were corrected using each optimized submap pose and evaluated after optimization with **evo 1.36.5**, 50 ms association, one shared rigid alignment per connected component and scale fixed to one. Ground truth never enters registration or CBS. GRACO uses its released IMU reference: all 8,983 dense poses associate. S3E Library 1 has only 1,031 associated samples among 12,299 dense poses; its sparse ground truth and unavailable IMU-to-RTK lever arms remain limitations. Small CPU/GPU differences should not be interpreted as a demonstrated accuracy gain or loss.

GRACO retains 106 pose loops and admits 100 geometry factors; Library 1 admits 260 geometry factors. Every admitted geometry factor passes the unchanged final inlier, overlap and observability checks. Initial rejection of unsupported geometry does not remove its PCM-retained companion pose constraint. The voxelized GPU objective differs from point GICP, so their optima need not coincide.

## What changed and why it is faster

The native `IntegratedVGICPFactorGPU` kernels from pinned gtsam_points v1.2.2 perform registration against two adaptive voxel levels (base 0.5–1.0 m, second level 2×). Endpoint points, covariances and voxel maps are uploaded once per session and shared across its factors. GPU factor linearizations are reused only when the relative pose is exactly identical. GLIM's 0.1 iSAM2 relinearization threshold and normal factor cache replace forced full relinearization in GPU mode. CPU and ellipsoid alternatives retain their old policy.

CBS still constructs outgoing cavity graphs using ordinary GTSAM. The wrapper executes native asynchronous CUDA work across each pair's voxel levels; it does not implement GLIM's whole-graph ISAM2Ext GPU hook. The information scale applies consistently to the nonlinear error and the full Hessian, gradient and constant. The adapter accounts for the pinned GPU cost's unhalved residual-sum convention; both pose gradients were checked numerically.

The remaining 20–29 s is **not ten-second online output latency**. Every fresh solve still pays startup/exchange/preparation costs and executes 100 scheduled iterations. The ten-second input-capture queue and persistent-session limitations are unchanged. Faster kernels alone do not eliminate that scheduling overhead.

## Memory scope

Host memory is the peak simultaneous sum of RSS over the experiment's unique worker/native/verifier process tree, sampled every 0.2 s; shared pages can be counted more than once. GPU memory is sampled whole-device allocation, including contexts, display and any other processes. CPU controls observed 81 MiB; GPU controls peaked at 1,230 and 1,874 MiB. These are not per-factor allocations or a Jetson memory prediction. Jetson Orin NX needs a separate ARM64/architecture-87 build and measurement; the x86/architecture-120 workstation build is not deployable there.

## Verification and activation

- Native checks: 8 passed; one standalone distributed-PCM test was environment-gated and skipped.
- Actual separate-process GPU DDS tests: normal and reversed robot order passed, including PCM rejection, geometry ownership, payload validation, common-frame poses and cleanup.
- Focused Python checks: 22 passed for GPU selection, preserved CPU settings, geometry exchange and online scheduling.
- All eight real frozen-graph controls completed; all admitted factors passed final geometry checks.
- The normal installed launcher is checked separately after pinning the exact GTSAM library used to build it. The runtime hook prevents ROS Humble's different same-SONAME GTSAM library from being selected accidentally.

The canonical pipeline now selects `vgicp_gpu`. The complete previous profile remains in `research/configs/cpu_gicp_pipeline.yaml`; existing frozen CPU communication experiments and historical results are unchanged. Selecting GPU mode without CUDA fails explicitly, without a CPU fallback. This validation covers two three-robot graphs, not a new all-14 or full-S3E rerun.

## Retained evidence

- [Raw evaluated results](evidence/matched-v2/evaluated-results.json), [table CSV](table.csv), [configuration/source/binary hashes](evidence/matched-v2/source-hashes.json), [inspection hashes](evidence/inspection-SHA256SUMS.json), [deployed source hashes](deployment-hashes.json).
- [Factor/build specification](../../GLIM-GPU-CBS.md), [default profile](../../configs/default_pipeline.yaml), [explicit CPU profile](../../configs/cpu_gicp_pipeline.yaml).
- Remote full evidence: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/gpu-cbs148-20260928/`. Geometry remains on 148. The `matched-v2` directory contains all eight runs, per-robot timing CSVs, registration support diagnostics and evo artifacts.
- Upstream references: [GLIM GPU configuration](https://github.com/koide3/glim/blob/master/config/config_global_mapping_gpu.json), [global mapping implementation](https://github.com/koide3/glim/blob/master/src/glim/mapping/global_mapping.cpp), [pinned gtsam_points](https://github.com/koide3/gtsam_points/tree/9d32e7dbecf6015560d84b4901d6b0a6f483ec46).
'''
    (ROOT / 'REPORT.md').write_text(text)
    plt.rcParams.update({'font.family': 'DejaVu Serif', 'font.size': 8, 'axes.spines.top': False,
        'axes.spines.right': False, 'pdf.fonttype': 42, 'ps.fonttype': 42})
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.55), layout='constrained')
    for ax, key, ylabel in zip(axes, ['times_s', 'joint_ates_m'], ['Full CBS solve (s)', 'Joint position ATE (m)']):
        for x, group in enumerate(NAMES.values()):
            for j, mode in enumerate(['gicp', 'vgicp_gpu']):
                values = next(row[key] for row in rows if row['group'] == group and row['mode'] == mode)
                pos = x + (-.17 if j == 0 else .17)
                ax.bar(pos, stats.median(values), width=.28, color=['#8899a6', '#147d92'][j],
                    label=['CPU GICP', 'GPU VGICP'][j] if x == 0 else None, alpha=.85)
                ax.scatter([pos-.03, pos+.03], values, color='#162631', s=12, zorder=3)
        ax.set_xticks([0, 1], ['GRACO aerial trio', 'S3E Library 1'])
        ax.set_ylabel(ylabel); ax.set_ylim(bottom=0); ax.grid(axis='y', alpha=.15); ax.set_axisbelow(True)
    axes[0].legend(fontsize=7, frameon=False, loc='upper left')
    for suffix in ['pdf', 'png']: fig.savefig(ROOT / ('gpu-comparison.'+suffix), dpi=300)
    plt.close(fig)
    header='<meta charset="utf-8"><title>GPU CBS validation</title><style>body{max-width:1080px;margin:40px auto;padding:0 20px;font:16px/1.6 system-ui;color:#193440}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:10px;text-align:right;border-bottom:1px solid #ddd}td:first-child,th:first-child{text-align:left}img{width:100%}a{color:#087d92}</style>'
    htmltable='<table><tr><th>Group</th><th>Mode</th><th>Solve repeats (s)</th><th>Joint ATE repeats (m)</th></tr>'
    for r in rows:htmltable+='<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in [r['group'],r['mode'],', '.join(f'{x:.2f}' for x in r['times_s']),', '.join(f'{x:.4f}' for x in r['joint_ates_m'])])+'</tr>'
    (ROOT / 'index.html').write_text(header+'<h1>GLIM GPU path in CBS</h1><p>Two matched three-robot graphs, two repeats per configuration. Eight completed solves; all robots connected. Unchanged 100-iteration schedule.</p><img src="gpu-comparison.png">'+htmltable+'</table><p>Dots show individual repeats. Bars show medians. Full solves still take 20–29 seconds; this is not ten-second publication latency. Library 1 has sparse ground truth. No Jetson deployment or all-14 GPU claim.</p><p><a href="REPORT.md">Full validation report</a> · <a href="table.csv">CSV</a> · <a href="evidence/matched-v2/evaluated-results.json">Measured data</a></p>')
    # The full historical paper bundle is optional; the standalone GPU report
    # and figure above remain reproducible from a source checkout alone.
    if not (PAPER / 'latex/main.tex').is_file():
        return
    shutil.copy2(ROOT / 'gpu-comparison.pdf', PAPER / 'output/pdf/gpu-comparison.pdf')
    tex=r'''\section{GPU registration in CBS}
\label{sec:gpu-cbs}
After the frozen CPU communication campaign was configured, we added a separate GPU path using GLIM's native gtsam\_points VGICP kernel family, two adaptive voxel levels and iSAM2 relinearization/caching settings. EllipseLIO, point-cloud BEVs, multilayer retrieval, point-GICP verification, PCM, geometry budgets and the conservative registration information cap are unchanged. The preceding communication results remain CPU measurements.

\begin{figure}[htbp]
\centering\includegraphics[width=\linewidth]{../output/pdf/gpu-comparison.pdf}
\caption{Matched frozen-final-graph CBS configurations on workstation 148 (RTX 5080). Bars are medians and dots show both repeats in CPU/GPU/GPU/CPU order. Both configurations execute 100 iterations at 10 Hz and retain one connected three-robot component. Runtime includes fresh-session preparation and exchanges. This measures the combined GPU objective and caching policy, not isolated kernel acceleration or GLIM end-to-end runtime.}
\end{figure}

The GRACO A05/A07/A08 graph completes in 42.40/42.44\,s on CPU versus 19.81/20.02\,s on GPU; Library~1 takes 125.58/126.33\,s versus 29.08/29.10\,s. The median ratios are 2.13 and 4.33. Shared-alignment position ATEs are 0.4846/0.4852\,m versus 0.4843/0.4851\,m for GRACO, and 1.6023/1.6188\,m versus 1.6173/1.6194\,m for Library~1. Evaluation uses evo~1.36.5, 50\,ms association and rigid alignment without scale. All 8,983 GRACO poses associate; Library~1 supplies only 1,031 matches among 12,299 dense poses, and its unknown IMU--RTK offset remains unresolved. Two repeats do not establish accuracy equivalence.

All 100 admitted GRACO and 260 Library~1 geometry factors pass final support checks. Eight native tests pass, one standalone PCM test is environment-gated, and two separate-process GPU tests pass with normal/reversed robot ordering and actual PCM rejection. Twenty-two focused Python checks also pass. The adapter scales the full nonlinear objective and Gaussian linearization consistently; both pose derivatives, rekeying, common-frame invariance and outgoing marginalization are checked.

Peak simultaneous process-tree RSS is 1.44/1.76\,GiB (CPU/GPU) for GRACO and 2.29/2.61\,GiB for Library~1. Whole-device allocation peaks at 1,230 and 1,874\,MiB during GPU runs, versus 81\,MiB for CPU controls; these samples include device contexts and any other processes. RSS can double-count shared pages. Workstation results do not predict Jetson unified-memory usage. Bulk evidence and source/configuration hashes remain on 148.

The GPU path is now the default for new evaluations; the full CPU profile and frozen historical runs are retained. The wrapper batches each pair's native voxel-level kernels inside ordinary GTSAM, without GLIM's whole-graph ISAM2Ext integration. Fresh-session startup, geometry exchanges and the fixed iteration schedule remain: 20--29\,s solves do not meet ten-second output latency, and no all-14 GPU or online-real-time claim is made.
\FloatBarrier
'''
    (PAPER / 'latex/sections/gpu-cbs.tex').write_text(tex)
    main = PAPER / 'latex/main.tex'; content = main.read_text()
    if r'\input{sections/gpu-cbs}' not in content:
        main.write_text(content.replace(r'\input{sections/communication}', r'\input{sections/communication}'+'\n'+r'\input{sections/gpu-cbs}'))


if __name__ == '__main__': main()

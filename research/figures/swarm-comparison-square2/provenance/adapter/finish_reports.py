#!/usr/bin/env python3
"""Finish reports and compact archives as independent native jobs exit."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
SEQUENCES=('square1','square2','laboratory1')
RESEARCH=ROOT/'FAST-LIVO2-ROS2/research'

def overview():
    lines=['# Swarm-SLAM LiDAR comparison','',
           'Three-robot S3E runs use fresh shared EllipseLIO exports. This compares the native Swarm LiDAR loop/optimizer path with our raw/ellipsoid MapClosures paths.',
           'Strict replay failures and preprocessing failures are retained. Unavailable ATEs are not replaced by historical results.','',
           '| Sequence | Strict Swarm replay | Exact selected inputs | Complete optimized coverage | Swarm ATE (m) | Raw BEV ATE (m) | Ellipsoid BEV ATE (m) |',
           '|---|---|---|---|---:|---:|---:|']
    def ate(v):
        rows=v.get('trajectory',{})
        if len(rows)!=1:return 'unavailable'
        row=next(iter(rows.values()))
        return f'{row["rmse_m"]:.4f}' if len(row['robots'])==3 else 'unavailable'
    for seq in SEQUENCES:
        p=RESEARCH/f'figures/swarm-comparison-{seq}/report.json'
        if not p.exists():
            lines.append(f'| {seq} | pending | pending | pending | pending | pending | pending |');continue
        report=json.loads(p.read_text());s=report['swarm'];b=report['ours']['branches']
        lines.append(f'| [{seq}](figures/swarm-comparison-{seq}/REPORT.md) | '+
            ('passed' if report['swarm_run']['success'] else 'failed')+f' | {s["selected_inputs_complete"]} | {s["optimized_coverage_complete"]} | {ate(s)} | {ate(b["raw"])} | {ate(b["ellipsoid"])} |')
    lines+=['','ATE is evaluated with evo 1.36.5 using one shared rigid alignment per component and no scale fitting.',
            'Laboratory 1 has no timestamped trajectory GT. Accepted native loops are recorded before GNC; native messages do not export GNC weights.',
            'A strict all-scan failure remains a failure even if the separate audit proves every selected keyframe and descriptor was delivered. Such a case can expose a usable keyframe result while failing transport validation.',
            'If selected inputs or native optimized coverage are incomplete, no full Swarm ATE is reported.', '',
            'Swarm uses native Scan Context and FPFH/TEASER++/ICP, with the documented transform-direction and GTSAM compatibility fixes.',
            'Native keyframes, map history and noise models differ from our paths. The reports disclose runtime and communication accounting differences.',
            'Setup: [Swarm-SLAM/s3e/README.md](../../Swarm-SLAM/s3e/README.md).']
    (RESEARCH/'RESULTS-SWARM-SLAM.md').write_text('\n'.join(lines)+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--retire-large',action='store_true');args=p.parse_args()
    os.chdir(ROOT)
    env=dict(os.environ,PYTHONPATH=str(RESEARCH),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/swarm-report-mpl')
    py=ROOT/'.ros2/research-venv/bin/python';rerun_py=ROOT/'.ros2/rerun-venv/bin/python'
    pending=set(SEQUENCES);failures={}
    while pending:
        for seq in sorted(pending):
            work=ROOT/'.ros2/swarm'/seq;output=RESEARCH/f'figures/swarm-comparison-{seq}'
            if (work/'report-complete.json').exists():pending.remove(seq);continue
            if not (work/'swarm/summary.json').exists():continue
            if not (output/'ours/report.json').exists() and not (work/'comparison-failure.json').exists():continue
            commands=[
                [py,'Swarm-SLAM/s3e/evaluate.py','--work',work,'--output',output],
                [py,'Swarm-SLAM/s3e/report.py',output],
                [rerun_py,'Swarm-SLAM/s3e/report.py',output,'--rerun'],
                [py,'Swarm-SLAM/s3e/archive.py','--work',work,'--output',output]+(['--retire-large'] if args.retire_large else [])]
            try:
                print(seq,'finalizing',flush=True)
                with (work/'finalize.log').open('w') as log:
                    for cmd in commands:subprocess.run([str(v) for v in cmd],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                (work/'report-complete.json').write_text(json.dumps(dict(report=str(output),finished_unix=time.time()))+'\n')
                print(seq,'report archived',flush=True)
            except Exception as exc:
                failures[seq]=repr(exc)
                (work/'report-failure.json').write_text(json.dumps(dict(error=repr(exc)))+'\n')
                print(seq,'report failure',repr(exc),flush=True)
            pending.remove(seq)
        overview()
        if pending:time.sleep(10)
    overview()
    if failures:raise RuntimeError(failures)
    print('All native attempts have finished; reports and compact evidence are retained.',flush=True)

if __name__=='__main__':main()

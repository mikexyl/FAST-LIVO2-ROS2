#!/usr/bin/env python3
"""Preserve a failed frontend attempt without inventing a Swarm result."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import sys
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Swarm-SLAM/s3e'))
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from frontend_quality import check_export
from s3e_pipeline.artifacts import file_hash,write_json
from s3e_pipeline.evaluation import ground_truth
from s3e_pipeline.evo_evaluation import evaluate


def main():
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--reason',required=True)
    args=p.parse_args();work=args.work.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    evidence=out/'provenance';evidence.mkdir(exist_ok=True)
    cfg=yaml.safe_load((work/'frontend/config.yaml').read_text());dataset=Path(cfg['dataset'])
    meta=yaml.safe_load((dataset/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start=meta['starting_time']['nanoseconds_since_epoch'];poses=[];truth={};quality={};captures={};hashes={};streams={}
    for robot in ('Alpha','Bob','Carol'):
        src=work/'frontend'/robot;dst=evidence/robot
        gt=dataset/f'{robot.lower()}_gt.txt';shutil.copy2(gt,evidence/gt.name);truth[robot]=ground_truth(gt)
        hashes[gt.name]=file_hash(gt)
        if not src.exists():captures[robot]=dict(started=False);continue
        dst.mkdir(exist_ok=True)
        summary=(json.loads((src/'summary.json').read_text()) if (src/'summary.json').exists()
                 else dict(success=False,error='Missing completed frontend summary'))
        captures[robot]=dict(summary,started=True)
        for name in ('summary.json','mapping_config.yaml','runtime.yaml','mapping.log','playback.log','divergence-diagnostics.json'):
            if (src/name).is_file():shutil.copy2(src/name,dst/name)
        for name in ('frames.jsonl','manifest.json'):
            path=src/'export'/name
            if not path.exists():continue
            hashes[f'{robot}/{name}']=file_hash(path)
            with path.open('rb') as inp,gzip.open(dst/(name+'.gz'),'wb') as output:shutil.copyfileobj(inp,output)
        if not (src/'export/manifest.json').exists():
            write_json(dst/'incomplete-export.json',dict(complete=False,reason=summary.get('error'),
                files={name:file_hash(src/'export'/name) for name in ('frames.jsonl','sensors.mcap')
                       if (src/'export'/name).is_file()},
                note='Interruption left no writer completion manifest; these are post-stop file hashes, not a completed sensor cache.'))
        if not (src/'export/frames.jsonl').exists():continue
        quality[robot]=check_export(src/'export');write_json(src/'quality.json',quality[robot])
        write_json(dst/'quality.json',quality[robot])
        # An interrupted robot is not evaluated as a complete sequence.
        if not summary.get('success'):continue
        rows=[json.loads(line) for line in (src/'export/frames.jsonl').read_text().splitlines()]
        streams[robot]=rows;poses.extend(dict(r,component=robot) for r in rows)
    metrics=evaluate(dict(poses=poses),truth,cfg['evaluation'],out/'raw-odometry-evo')
    audit_path=work/'alpha-imu-onset-audit.json'
    audit=json.loads(audit_path.read_text()) if audit_path.exists() else None
    if audit_path.exists():shutil.copy2(audit_path,evidence/audit_path.name)
    shutil.copy2(work/'frontend/config.yaml',evidence/'frontend-config.yaml')
    shutil.copytree(ROOT/'Swarm-SLAM/s3e',evidence/'adapter',dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copytree(ROOT/'.ros2/swarm/provenance',evidence/'setup',dirs_exist_ok=True)
    write_json(evidence/'sha256.json',hashes)
    result=dict(dataset=str(dataset),frontend='EllipseLIO',frontend_failure=args.reason,
        captures=captures,quality=quality,imu_onset_audit=audit,
        ours=dict(raw_odometry=metrics,not_run='Frontend failure; no MapClosures comparison run'),
        swarm_run=dict(started=False,success=False,complete=False,failure_stage='frontend',error=args.reason),
        swarm=dict(available=False,trajectory={},components={},reason='Not run on invalid frontend inputs'))
    write_json(out/'report.json',result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    onsets={}
    for robot,rows in streams.items():
        t=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
        xyz=np.array([np.array(r['T_world_body']).reshape(4,4)[:3,3] for r in rows])
        elapsed=(t-start)/1e9;steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1);speed=steps/(np.diff(t)/1e9)
        onsets[robot]={str(limit):(float(elapsed[np.flatnonzero(speed>limit)[0]+1])
                                  if np.any(speed>limit) else None) for limit in (5,20,100)}
        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True,layout='constrained')
        axes[0].plot(elapsed,np.linalg.norm(xyz-xyz[0],axis=1));axes[0].set_ylabel('Distance from start [m]')
        axes[1].semilogy(elapsed[1:],np.maximum(speed,1e-4));axes[1].axhline(20,color='red',ls='--',label='Motion sanity limit: 20 m/s')
        axes[1].set_ylabel('Scan-to-scan speed [m/s]');axes[1].set_xlabel('Bag elapsed time [s]');axes[1].legend()
        for ax in axes:ax.grid(alpha=.2)
        fig.suptitle(f'{dataset.name} · {robot} EllipseLIO divergence before Swarm-SLAM')
        fig.savefig(out/f'{robot}-divergence.png',dpi=180);fig.savefig(out/f'{robot}-divergence.pdf');plt.close(fig)
    incomplete=[r for r,c in captures.items() if c.get('started') and not c.get('success')]
    not_started=[r for r,c in captures.items() if not c.get('started')]
    lines=[f'# {dataset.name}: frontend failure on workstation 148','',
           '**Swarm-SLAM was not run on this sequence.** '+args.reason,'',
           'Original capture exit statuses are retained alongside separate motion-quality checks. '
           f'Incomplete captures: {", ".join(incomplete) or "none"}. Not started: {", ".join(not_started) or "none"}. '
           'Interrupted captures and missing robots are excluded from complete-sequence ATE.','',
           '| Completed capture | Raw position ATE [m] | GT matches | Maximum scan step [m] | Maximum speed [m/s] |',
           '|---|---:|---:|---:|---:|']
    for robot,m in metrics.items():
        q=quality[robot]
        lines.append(f'| {robot} | {m["rmse_m"]:.4f} | {m["samples"]} | {q["max_scan_step_m"]:.3f} | {q["max_speed_m_s"]:.3f} |')
    lines+=['','ATE is computed entirely with evo 1.36.5, 50 ms timestamp association, one rigid alignment '
            'per complete robot capture, and no scale fitting. No failing tail was trimmed to improve the score. '
            'Ground truth covers only part of the capture; unlabelled later divergence is not included in ATE. '
            'Supplied GT orientations are unused.','']
    if audit:
        lines += [f'The original IMU stream around 620–660 seconds contains **{audit["messages"]} messages**, '
                  f'a maximum timestamp gap of **{audit["max_header_gap_s"]*1000:.3f} ms**, '
                  f'**{audit["gaps_over_50ms"]} gaps over 50 ms**, and **{audit["nonmonotonic_intervals"]} non-increasing intervals. '
                  'This does not support a missing-IMU gap in the bag at the onset. It does not by itself prove the runtime delivery or identify the estimator’s internal failure mechanism.','']
    for robot,values in onsets.items():
        lines += [robot+' motion onsets (bag seconds): '+', '.join(f'above {limit} m/s at {value:.2f} s' for limit,value in values.items() if value is not None)+'.','']
    lines += ['The input guard now rejects finite but implausible '
              'ground-robot motion above 20 m/s before starting Swarm-SLAM. The guard does not use GT or modify the estimator.','',
              '[Machine-readable report](report.json), [evo evidence](raw-odometry-evo/README.md).','']
    for robot in streams:lines += [f'![{robot} motion diagnostics]({robot}-divergence.png)','']
    if audit:lines += ['[IMU onset audit](provenance/alpha-imu-onset-audit.json).','']
    result['motion_onsets_seconds']=onsets;write_json(out/'report.json',result)
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    write_json(work/'pipeline-disposition.json',dict(failure_stage='frontend',root_cause=args.reason,
        incomplete_captures=incomplete,not_started=not_started,
        action='Native Swarm skipped after frontend failure; diagnostics retained',report=str(out)))
    print(out/'REPORT.md')


if __name__=='__main__':main()

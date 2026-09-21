#!/usr/bin/env python3
"""Audit a completed motion-submap batch and compare it with retained temporal runs."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import traceback
import yaml
from motion_submap_batch_audit import audit, digest, read, rows


def number(value):
    return 'Unavailable' if value is None else f'{value:.4f}'


def raw_comparison_plot(root,groups):
    """Show matched per-robot raw ATEs; do not pool independently aligned errors."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    samples=[]
    for group in groups:
        for robot,current in group['raw'].items():
            previous=group['baseline_raw_ate'].get(robot)
            if previous is not None and current.get('rmse_m') is not None:
                label=group['name'].replace('S3E_','').replace('_',' ')+' / '+robot
                samples.append((label,previous,current['rmse_m']))
    if not samples:return None
    fig,ax=plt.subplots(figsize=(11,max(4,.48*len(samples))),layout='constrained')
    for i,(label,previous,current) in enumerate(samples):
        ax.barh(i-.18,previous,.34,color='#888888',label='Temporal 10 s / 5 s' if i==0 else None)
        ax.barh(i+.18,current,.34,color='#1976b4',label='Motion / overlap' if i==0 else None)
    ax.set_yticks(range(len(samples)),[r[0] for r in samples]);ax.invert_yaxis()
    ax.set_xlabel('Raw position ATE RMSE [m] — independent SE(3) fit per robot')
    ax.set_title('One retained temporal trial versus one fresh motion/overlap trial')
    ax.legend();ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    for suffix in ('png','pdf'):fig.savefig(root/f'raw-ate-comparison.{suffix}',dpi=180)
    plt.close(fig)
    return dict(matched_robots=len(samples),improved=sum(current<previous for _,previous,current in samples),
                worse=sum(current>previous for _,previous,current in samples),
                unchanged=sum(current==previous for _,previous,current in samples),
                limitation='Single trials with different concurrency; no general or causal accuracy claim')


def raw_metrics_after_failure(folder, plan):
    """Keep raw ATE available even when a backend cannot form a common frame."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/recent_submaps'))
    from rollout_report import native_rows
    from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth
    from s3e_pipeline.evaluation import trajectory_metrics, save_tum
    cfg=yaml.safe_load((folder/'config.yaml').read_text())
    output=folder/'raw-evaluation-after-backend-failure'
    if (output/'metrics.json').exists():
        previous=read(output/'metrics.json')
        return previous['raw'],previous['ground_truth']
    output.mkdir(exist_ok=True)
    dense=[]; robots=[]
    for robot,item in plan['trials'].items():
        path=Path(item['path'])/'frontend/native_updates.jsonl'
        if not path.exists() or not path.stat().st_size:continue
        track=native_rows(item['path'],robot);dense+=track;robots.append(robot)
        save_tum(output/f'{robot}-raw.tum',track)
    truth,status=evaluation_ground_truth(robots,Path(cfg['dataset']),dense,cfg['evaluation'])
    metrics=trajectory_metrics(dict(poses=dense),truth,cfg['evaluation'],output/'evo') if dense else {}
    (output/'metrics.json').write_text(json.dumps(dict(raw=metrics,ground_truth=status),indent=2)+'\n')
    return metrics,status


def finish(root,baseline):
    state=read(root/'status.json'); plans={g['name']:g for g in read(root/'plan.json')}
    for name,expected in read(root/'source-hashes.json').items():
        assert digest(Path('/workspace')/name)['sha256']==expected,name
    evidence=audit(root)
    result=dict(schema_version=1,strategy='motion_overlap',baseline=str(baseline),
                started_utc=state['started_utc'],finished_utc=state['finished_utc'],
                frozen_sources_verified=True,frontend_workers=state['frontend_workers'],groups=[])
    lines=['# Motion/overlap submap EllipseLIO: five-group comparison','',
           'This is the first fixed-configuration motion/overlap trial. Keyframes use 1 m translation, 10° rotation, or geometric overlap below 0.6. Maps request retirement at 20 m extent or 15 selected keyframes, with a growing successor and a geometric readiness check. Thirty seconds is a stale-geometry guard. Descriptor and registration evidence membership remain identical.','',
           'All 18 robot captures are fresh. Six 1× frontends run concurrently with four executor threads each, distinct ROS domains and no container CPU/RAM/swap quotas. Historical temporal trials used a different concurrency/resource configuration; this is not a repeated controlled ablation. MapClosures, registration, PCM and CBS settings are preserved.','',
           'Position ATE uses evo 1.36.5, 50 ms association, SE(3) alignment without scale. Each raw trajectory is aligned independently; CBS uses one fit per measured connected component, whose membership can differ across strategies. GT is read only by evaluation. Backend frame failures yield no valid shared CBS ATE. Raw ATE for a failed/incomplete frontend is diagnostic only.','',
           '| Group | Pipeline outcome | Connected robots / output frames | Retained loops | PCM rejected | Backend wall [s] |',
           '|---|---|---|---:|---:|---:|']
    details=[]
    for status in state['groups']:
        name=status['name'];plan=plans[name];folder=Path(plan['folder'])
        group=dict(name=name,status=status['status'],frontends=status['frontends'])
        result['groups'].append(group)
        report=read(folder/'report/report.json') if status['status']=='complete' else None
        if report:
            raw=report['raw'];gt=report['ground_truth'];cbs=report['cbs']
            connected=report['connectivity'];runtime=report['runtime'];pcm=report['pcm']
        else:
            group['failure']=read(folder/'failure.json')
            try:
                raw,gt=raw_metrics_after_failure(folder,plan)
            except Exception as error:
                group['raw_evaluation_failure']=dict(error=repr(error),traceback=traceback.format_exc())
                raw,gt={},{}
            cbs={}
            connected=read(folder/'connectivity.json') if (folder/'connectivity.json').exists() else {}
            runtime=read(folder/'dpgo/summary.json') if (folder/'dpgo/summary.json').exists() else {}
            pcm=read(folder/'dpgo/pcm.json') if (folder/'dpgo/pcm.json').exists() else {}
        group.update(raw=raw,cbs=cbs,ground_truth=gt,connectivity=connected,
                     runtime={k:v for k,v in runtime.items() if k!='robots'},
                     pcm={k:v for k,v in pcm.items() if k!='verdicts'},motion_diagnostics={})
        component_text='; '.join('+'.join(c) for c in connected.get('measured_components',[])) or 'Unavailable'
        if connected and not connected.get('common_frame_contract',False):component_text+='; inconsistent output frames'
        outcome='Complete' if report else 'Failed: '+group['failure']['stage']
        wall='—' if 'wall_s' not in runtime else f'{runtime["wall_s"]:.2f}'
        lines.append(f'| {name} | {outcome} | {component_text} | {runtime.get("loops","—")} | {pcm.get("excluded_loops","—")} | {wall} |')
        details += [f'## {name}','']
        if not report:details += [f'Failure: `{group["failure"]["error"]}`. Detailed logs remain in the group directory.','']
        if 'raw_evaluation_failure' in group:details += [f'Raw evaluation also failed: `{group["raw_evaluation_failure"]["error"]}`.','']
        details += ['| Robot | Temporal raw ATE [m] | Motion raw ATE [m] | Temporal CBS ATE [m] | Motion CBS ATE [m] | Frontend status |',
                    '|---|---:|---:|---:|---:|---|']
        previous=baseline/name
        previous_report=read(previous/'report/report.json') if (previous/'report/report.json').exists() else {}
        baseline_raw={r:v.get('rmse_m') for r,v in previous_report.get('raw',{}).items()}
        baseline_cbs={}
        for robot,item in plan['trials'].items():
            old_metrics=previous/'raw-evaluation-after-backend-failure'/robot/'metrics.json'
            if old_metrics.exists():baseline_raw[robot]=read(old_metrics).get('ate_rmse_m')
            old_component=previous_report.get('components',{}).get(robot)
            baseline_cbs[robot]=previous_report.get('cbs',{}).get(old_component,{}).get('per_robot',{}).get(robot,{}).get('statistics',{}).get('rmse')
            corrected=None
            if report:
                component=report['components'][robot]
                corrected=cbs.get(component,{}).get('per_robot',{}).get(robot,{}).get('statistics',{}).get('rmse')
            native=Path(item['path'])/'frontend/native_updates.jsonl'
            if native.exists():
                data=rows(native)
                group['motion_diagnostics'][robot]=dict(events=dict(Counter(r.get('submap_event') for r in data if r.get('submap_event'))),
                    max_correspondence_age_s=max((r['age_max_s'] for r in data if r['lidar_updated']),default=None),
                    max_selected_keyframes=max((r.get('selected_keyframes',0) for r in data),default=0),
                    max_extent_m=max((r.get('submap_extent_m',0) for r in data),default=0))
            details.append(f'| {robot} | {number(baseline_raw.get(robot))} | {number(raw.get(robot,{}).get("rmse_m"))} | {number(baseline_cbs.get(robot))} | {number(corrected)} | {status["frontends"][robot]["status"]} |')
        group['baseline_raw_ate']=baseline_raw
        group['baseline_cbs_ate']=baseline_cbs
        shared='; '.join(f'{c}: {number(v.get("rmse_m"))} m' for c,v in cbs.items()) or 'Unavailable'
        details += ['',f'Shared-component CBS ATE: {shared}.','']
        if report:details += [f'![Trajectories and maps]({name}/report/trajectories-maps.png)','']
        details += ['| Robot | Max speed [m/s] | Max LiDAR update gap [s] | Handovers | Stale recoveries | Completed / retrievable submaps | Max correspondence age [s] | Mapper peak RSS [MiB] |',
                    '|---|---:|---:|---:|---:|---:|---:|---:|']
        for robot,entry in evidence['groups'][name]['frontends'].items():
            if entry['status']!='complete':continue
            quality=entry['quality'];events=group['motion_diagnostics'][robot]['events']
            normal_handovers=sum(v for k,v in events.items() if k.startswith('handover_') and k!='handover_waiting_for_support')
            details.append(f'| {robot} | {quality["max_speed_m_s"]:.3f} | {quality["max_successful_update_gap_s"]:.3f} | {normal_handovers} | {events.get("stale_recovery",0)} | {entry["completed_submaps"]} / {entry["retrievable_submaps"]} | {number(entry["max_correspondence_age_s"])} | {entry["peak_mapper_rss_kib"]/1024:.1f} |')
        details += ['']
    result['raw_comparison']=raw_comparison_plot(root,result['groups'])
    if result['raw_comparison']:
        compared=result['raw_comparison']
        lines += ['',f'Raw ATE decreased on {compared["improved"]}/{compared["matched_robots"]} matched robots and increased on {compared["worse"]}. See the individual values; no pooled independently aligned ATE is reported.','',
                  '![Raw ATE comparison](raw-ate-comparison.png)','']
    lines += ['',*details,'## Retained evidence','',
              'Native window/member/anchor/hash validation and live Rerun verification are required for every admitted frontend. Successfully evaluated groups additionally retain verified derived Rerun recordings, raw/CBS trajectories, evo archives, map arrays, and descriptor/evidence/causality audits. Partial or failed outputs are kept with their failure status. All bulk geometry is retained.','',
              'Checks before launch: two native CTest cases (including motion policy and temporal compatibility), eight Python snapshot/endpoint cases, six scheduler/admission cases, two actual native retrieval/PCM/CBS integration cases, and two 120-second dataset smokes. No parameter sweep was performed.','',
              'The batch configuration, sources and binary hashes are frozen. `batch-summary.json` contains the numeric comparison, `retention-audit.json` contains immutable artifact hashes and validation, and each group retains its complete logs.','']
    result['reporting_source_hashes']={str(p):digest(p)['sha256'] for p in
                                      [Path(__file__),Path(__file__).with_name('motion_submap_batch_audit.py')]}
    (root/'batch-summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (root/'REPORT.md').write_text('\n'.join(lines))
    (root/'finalization.json').write_text(json.dumps(dict(status='complete',finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    parser.add_argument('--baseline',type=Path,default=Path('/history/recent-submaps/full-five-groups-parallel-20260919'))
    args=parser.parse_args()
    try:
        while not read(args.root/'status.json').get('finished_utc'):
            if read(args.root/'status.json')['phase']=='interrupted':raise RuntimeError('Batch interrupted')
            time.sleep(10)
        finish(args.root,args.baseline)
    except BaseException as error:
        (args.root/'finalization.json').write_text(json.dumps(dict(status='failed',error=repr(error),traceback=traceback.format_exc()),indent=2)+'\n')
        raise

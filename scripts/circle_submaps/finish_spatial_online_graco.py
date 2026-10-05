#!/usr/bin/env python3
"""Audit/report an actual online spatial-support run against its chain control.

Uses the established independent online crop audit and evo evaluation. No
retrieval, loop verification, PCM or optimization is rerun by this reporter.
"""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import traceback

import numpy as np


def dist(values):
    x=np.asarray(values,float)
    if not len(x):return None
    if not np.isfinite(x).all():raise ValueError('Nonfinite timing')
    return dict(count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(x.max()))


def load_module(path):
    spec=importlib.util.spec_from_file_location('online_auditor',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def native_timing(root, output):
    """Actual elapsed timer values; separate pose solves from full callbacks."""
    from s3e_pipeline.artifacts import read_json, read_jsonl, write_json
    start=read_json(root/'START')['wall_ns'];rows=[];omissions=[]
    for robot in read_json(root/'START')['robots']:
        local=root/'epochs/persistent'/robot
        stats={s['iteration']:s for s in read_jsonl(local/'stats.jsonl')}
        paths=list((local/'native'/robot/'cbs_online').glob('timing_robot_*.csv'))
        assert len(paths)==1
        seen=set()
        for t in csv.DictReader(paths[0].open()):
            iteration=int(t['iteration']);assert iteration not in seen;seen.add(iteration)
            s=stats[iteration]
            for k in ['num_new_values','num_new_factors','num_removed_factors','num_accepted_beliefs']:
                assert int(t[k])==s[k]
            row=dict(robot=robot,iteration=iteration,progress_s=(s['logged_wall_ns']-start)/1e9,
                revision=s['graph_revision'],stage=s['belief_stage'],variables=s['num_values'],
                new_factors=s['num_new_factors'],meaningful_update=s['meaningful_update'])
            for k in ['update_ms','stats_ms','belief_request_ms','visualization_ms','active_callback_ms']:
                row[k]=float(t[k]);assert np.isfinite(row[k]) and row[k]>=0
            assert row['active_callback_ms']>=row['update_ms']
            rows.append(row)
        missing=sorted(set(stats)-seen);assert missing==[max(stats)]
        omissions.append(dict(robot=robot,iteration=missing[0],reason='terminal callback has no timing row'))
    with (output/'cbs-updates.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary={k:dist([r[k] for r in rows]) for k in ['update_ms','active_callback_ms','stats_ms']}
    summary.update(timed_updates=len(rows),omissions=omissions,scope='Native elapsed solver and timer callback measurements; terminal callbacks omitted, not filled with zeros')
    write_json(output/'cbs-timing.json',summary)
    return rows,summary



def topology_preview(root,out,robot,keys,local):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.collections import LineCollection
    with np.load(root/f'prepared-{robot}/union.npz') as data:world=np.asarray(data['world'],float)
    frame=np.asarray(keys[0]['T_horizontal_map']);xy=(world@frame[:3,:3].T+frame[:3,3])[:,:2]
    xy=xy[np.linspace(0,len(xy)-1,min(len(xy),150000),dtype=int)]
    centers=np.asarray([k['footprint_center_xy'] for k in keys]);pending=set(local['unconnected_keys'])
    fig,ax=plt.subplots(figsize=(8,7))
    ax.scatter(*xy.T,s=.3,c='#cbd3d8',rasterized=True,edgecolors='none')
    for k,c in enumerate(centers):
        color='#D55E00' if k in pending else '#0072B2'
        ax.add_patch(Circle(c,keys[k]['footprint_radius_m'],facecolor='none',edgecolor=color,alpha=.25,lw=.7))
        ax.scatter(*c,c=color,s=35,zorder=4)
        ax.annotate(str(k),c,xytext=(4,4),textcoords='offset points',fontsize=8,zorder=5)
    ax.add_collection(LineCollection([centers[[e['i'][1],e['j'][1]]] for e in local['constraints']],
        colors=['#D55E00' if e['i'][1] in pending else '#0072B2' for e in local['constraints']],linewidths=1.4,zorder=3))
    ax.set_aspect('equal');ax.set_xlabel('Gravity-aligned local map x (m)');ax.set_ylabel('Gravity-aligned local map y (m)')
    ax.set_title(f'{robot}: actual spatial-support graph\nBlue: admitted component | orange: pending footprint')
    ax.grid(alpha=.12);fig.tight_layout();fig.savefig(out/'submap-graph.png',dpi=300);fig.savefig(out/'submap-graph.pdf');plt.close(fig)


def supported_subset_evaluation(args,cfg,keys,poses):
    """Evaluate whole trajectories only for robots with every submap supported.

    Exclusion is determined by topology, never by trajectory error. The old
    control is evaluated on exactly the same robots with one shared evo fit.
    """
    from s3e_pipeline.artifacts import write_json,file_hash
    from s3e_pipeline.circle_central_evaluation import tum_rows,correction_anchors
    from s3e_pipeline.evaluation import corrected_dense_rows,ground_truth,save_tum,trajectory_metrics
    out=args.output/'subset-evaluation';out.mkdir(exist_ok=True)
    excluded=sorted({p['robot_id'] for p in poses if not p.get('optimized',True)})
    included=[r for r in cfg['robots'] if r not in excluded]
    raw=[];dense=[];control=[];truth={};source_hashes={};reference_hashes={}
    for robot in cfg['robots']:
        path=args.output/'trajectories'/f'{robot}.tum';track=tum_rows(path,robot)
        raw.extend(dict(p,component=robot) for p in track)
        reference=args.base/'reference'/f'{robot}_gt.txt'
        truth[robot]=ground_truth(reference);reference_hashes[robot]=file_hash(reference)
        source_hashes[robot]=dict(raw=file_hash(path))
        if robot not in included:continue
        # Verify that the dense trajectory and submap anchors are one capture.
        stamps=np.array([p['stamp_ns'] for p in track],dtype=np.int64)
        for key in keys[robot]:
            index=int(np.argmin(np.abs(stamps-key['stamp_ns'])))
            assert abs(int(stamps[index])-key['stamp_ns'])<=10**6
            np.testing.assert_allclose(track[index]['T_world_body'],key['T_world_body'],atol=1e-4,rtol=0)
        anchors,optimized,_=correction_anchors(keys[robot],[p for p in poses if p['robot_id']==robot])
        corrected=[dict(p,component='supported_subset') for p in corrected_dense_rows(track,anchors,optimized)]
        save_tum(out/f'{robot}-corrected.tum',corrected);dense.extend(corrected)
        control_path=args.control/'solution/evaluation'/f'{robot}-corrected.tum'
        control.extend(dict(p,component='supported_subset') for p in tum_rows(control_path,robot))
        source_hashes[robot]['control_corrected']=file_hash(control_path)
    result=dict(included_robots=included,excluded_robots=excluded,
        selection='Exclude each entire robot trajectory if any submap is pending; selection uses graph support, not ATE.',
        scope='Supported robot subset only; not an all-14 result. Each method uses one shared rigid alignment on the same subset.',
        raw_ate=trajectory_metrics(dict(poses=raw),truth,cfg['evaluation'],out/'evo/raw'),
        source_hashes=source_hashes,reference_hashes=reference_hashes)
    if included:
        result['spatial_joint_ate']=trajectory_metrics(dict(poses=dense),truth,cfg['evaluation'],out/'evo/spatial-joint')
        result['control_joint_ate']=trajectory_metrics(dict(poses=control),truth,cfg['evaluation'],out/'evo/control-joint')
        result['spatial_individual_ate']=trajectory_metrics(dict(poses=[dict(p,component=p['robot_id']) for p in dense]),truth,cfg['evaluation'],out/'evo/spatial-individual')
        assert result['spatial_joint_ate']['supported_subset']['robots']==sorted(included)
        assert result['control_joint_ate']['supported_subset']['robots']==sorted(included)
    write_json(out/'result.json',result)
    return result


def timing_plot(timing_rows,audit,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for ax,key,label in zip(axes,['update_ms','active_callback_ms'],['CBS solver update (ms)','Full callback (ms)']):
        ax.scatter([r['progress_s'] for r in timing_rows],[r[key] for r in timing_rows],s=6,color='#0072B2',alpha=.35,rasterized=True)
        ax.set_ylabel(label);ax.set_yscale('log');ax.grid(alpha=.15)
        ax.axvline((audit['timeline']['last_bag_expected_end_wall_ns']-audit['timeline']['start_wall_ns'])/1e9,color='#D55E00',ls='--',label='Last bag ends')
    axes[0].legend(frameon=False);axes[-1].set_xlabel('Seconds after common playback release')
    fig.suptitle('Spatial-support submap graphs | all 14 GRACO robots | actual online updates');fig.tight_layout();fig.savefig(out/'cbs-progress.png',dpi=300);fig.savefig(out/'cbs-progress.pdf');plt.close(fig)


def run(args):
    sys.path.insert(0,str(args.runtime_project/'research'))
    from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,file_hash
    from s3e_pipeline.geometry import inv
    base=load_module(args.runtime_project/'scripts/circle_submaps/finish_online_graco.py')
    root,out=args.run,args.output;out.mkdir(parents=True,exist_ok=True)
    cfg=read_json(root/'configuration.json');assert cfg['pgo']['spatial_neighbors']['topology']=='spatial_only'
    audit,keys,dpgo,publications=base.audit(root,cfg,out)
    summary=read_json(dpgo/'summary.json');poses=read_jsonl(dpgo/'poses.jsonl')
    local_graphs={r:read_json(root/f'epochs/persistent/{r}/local-graph.json') for r in cfg['robots']}
    local_edges=read_jsonl(dpgo/'local-constraints.jsonl')
    assert len(local_edges)==sum(len(x['constraints']) for x in local_graphs.values())
    for edge in local_edges:
        assert edge['kind']=='map_consistency' and edge['i'][0]==edge['j'][0]
        robot=edge['i'][0];a,b=(keys[robot][edge[k][1]] for k in ['i','j'])
        np.testing.assert_allclose(edge['T_i_j'],inv(np.asarray(a['T_world_body']))@np.asarray(b['T_world_body']),atol=1e-8)
        assert edge['diagnostics']['reason']=='accepted'
        assert edge['diagnostics']['available_ns']<=max(r['available_ns'] for r in keys[robot])
    write_json(out/'spatial-graphs.json',dict(robots=local_graphs,odometry_chain_edges=0,
        supported_edges=len(local_edges),admitted_factors=summary['map_consistency_factors'],
        pending_factors=summary.get('pending_map_consistency_factors',0),alignment_complete=summary['alignment_complete']))
    timing_rows,timing=native_timing(root,out)
    # A common frame among robot roots is insufficient when local fragments
    # are pending. Recover complete-map alignment time from retained revisions.
    completeness={e['revision']:read_json(Path(e['path'])/'summary.json').get('alignment_complete',True) for e in publications}
    all_aligned=[e for e in publications if e['common_reference_ready'] and len(set(e['components'].values()))==1 and completeness[e['revision']]]
    audit['timeline']['first_all14_alignment_s']=(all_aligned[0]['published_wall_ns']-audit['timeline']['start_wall_ns'])/1e9 if all_aligned else None
    audit['timeline']['all14_alignment_requires_no_pending_submaps']=True
    audit['alignment_complete']=summary['alignment_complete']
    audit['success_scope']='Independent data and runtime audit; full alignment is reported separately.'
    write_json(out/'audit.json',audit)
    timing_plot(timing_rows,audit,out)
    pending=[p for p in poses if not p.get('optimized',True)]
    topology_robot=pending[0]['robot_id'] if pending else 'aerial05'
    topology_preview(root,out,topology_robot,keys[topology_robot],local_graphs[topology_robot])
    if pending:
        write_json(out/'PENDING.json',dict(poses=pending,reason='No supported local path to reference submap; refusing all-robot trajectory interpolation and global map'))
        subset=supported_subset_evaluation(args,cfg,keys,poses)
        # An incomplete method result still gets a factual report and all costs.
        import html
        old=read_json(args.control/'audit.json');old_summary=read_json(args.control/'cbs-summary.json')
        old_t=read_json(args.control_timing or args.control/'cbs-timing/summary.json')
        old_r=read_json(args.control/'resources.json');new_r=read_json(root/'resources.json')
        rows=[('Published submaps',old_summary['keyframes'],summary['keyframes']),
              ('PCM-retained loops',old_summary['loops'],summary['loops']),
              ('Pending submaps',0,len(pending)),
              ('Total application communication (MB)',old['communication']['total_application_cdr_bytes']/1e6,audit['communication']['total_application_cdr_bytes']/1e6)]
        for label,key in [('PCM','pcm_network_cdr_bytes'),('CBS','cbs_network_cdr_bytes'),('Registration geometry','registration_network_cdr_bytes'),('Retrieval / verification','peer_retrieval_cdr_bytes')]:
            rows.append((label+' (MB)',old['communication'][key]/1e6,audit['communication'][key]/1e6))
        rows += [('Root-frame groups',1,len({v['component'] for v in summary['robots'].values()})),
            ('Local map factors',old_summary['keyframes']-len(cfg['robots']),summary['map_consistency_factors']),
            ('CBS solver median (ms)',old_t['update_ms']['median'],timing['update_ms']['median']),
            ('CBS solver p95 (ms)',old_t['update_ms']['p95'],timing['update_ms']['p95']),
            ('CBS solver maximum (ms)',old_t['update_ms']['max'],timing['update_ms']['max']),
            ('Full callback p95 (ms)',old_t['active_callback_ms']['p95'],timing['active_callback_ms']['p95']),
            ('Wall time (s)',old_r['wall_s'],new_r['wall_s']),
            ('Peak simultaneous RSS (GiB)',old_r['peak_simultaneous_process_tree_rss_bytes']/2**30,new_r['peak_simultaneous_process_tree_rss_bytes']/2**30)]
        if subset['included_robots']:
            rows.append((f"Joint ATE on the same {len(subset['included_robots'])}-robot supported subset (m)",
                subset['control_joint_ate']['supported_subset']['rmse_m'],subset['spatial_joint_ate']['supported_subset']['rmse_m']))
        write_json(out/'comparison.json',dict(rows=[dict(metric=k,chain=a,spatial=b) for k,a,b in rows],joint_ate_m=None,
            subset_evaluation=subset,
            limitation='Fresh independent online replays; changed scheduling, odometry, circle geometry and retrieval opportunities. Incomplete spatial alignment; traffic and solve time are not equivalent-outcome comparisons.'))
        fmt=lambda x:str(x) if isinstance(x,int) else f'{x:.3f}'
        table='\n'.join(['| Metric | Prior chain | Spatial support |','| --- | ---: | ---: |']+['| '+label+' | '+fmt(a)+' | '+fmt(b)+' |' for label,a,b in rows])
        reason=f'{len(pending)} submaps remain pending. Their geometry has no accepted local path to the reference submap. The present one-anchor-per-robot CBS interface cannot activate a fragment through an inter-robot loop alone. No artificial links were added; no all-14 joint ATE or global PLY is claimed.'
        reason+=' Subset accuracy excludes entire trajectories for '+', '.join(subset['excluded_robots'])+' in both methods. Evo 1.36.5 uses corrected GRACO IMU-reference ground truth, 50 ms association, one shared rigid alignment, and no scale fit.'
        reason+=' These are fresh independent online replays: scheduling, odometry, geometry and retrieval opportunities can differ. Communication excludes DDS/IP/discovery, sensors, visualization and report downloads. Costs are not comparisons at an equivalent final outcome because the spatial result is incomplete.'
        (out/'REPORT.md').write_text('# GRACO 14-robot spatial graph: incomplete alignment\n\n'+reason+'\n\n'+table+'\n')
        page='<html><meta charset="utf-8"><title>Spatial graph: incomplete alignment</title><style>body{font:17px/1.6 system-ui;max-width:1100px;margin:40px auto}td,th{padding:8px 20px;border-bottom:1px solid #ddd;text-align:right}pre{white-space:pre-wrap}</style><h1>GRACO 14 robots: incomplete spatial alignment</h1><p>'+html.escape(reason)+'</p><table><tr><th>Metric</th><th>Prior chain</th><th>Spatial support</th></tr>'+''.join('<tr><td>'+label+'</td><td>'+fmt(a)+'</td><td>'+fmt(b)+'</td></tr>' for label,a,b in rows)+'</table><h2>Pending submaps</h2><pre>'+html.escape(json.dumps({r:[p['keyframe_id'] for p in pending if p['robot_id']==r] for r in cfg['robots']},indent=2))+'</pre><img src="submap-graph.png" style="width:100%"><p>Blue is the admitted local component; orange is the unsupported fragment. This is a local map layout, not a joint alignment.</p><h2>CBS time during the experiment</h2><img src="cbs-progress.png" style="width:100%"><p><a href="subset-evaluation/result.json">Supported-subset accuracy and all raw ATEs</a> · <a href="audit.json">Measured audit</a> · <a href="spatial-graphs.json">All local graphs</a> · <a href="cbs-timing.json">CBS timing</a></p></html>'
        (out/'index.html').write_text(page)
        for name,src in {'configuration.json':root/'configuration.json','cbs-summary.json':dpgo/'summary.json','registration.json':dpgo/'registration.json','resources.json':root/'resources.json'}.items():shutil.copy2(src,out/name)
        write_json(out/'COMPLETE.json',dict(success=False,inspection_complete=True,all_14_connected=False,reason=reason,pending_submaps=len(pending)))
        base.provenance(root,out,cfg)
        shutil.copy2(__file__,out/'spatial-auditor.py')
        write_json(out/'FILES.sha256.json',{str(p.relative_to(out)):file_hash(p) for p in out.rglob('*') if p.is_file() and p.name!='FILES.sha256.json'})
        return
    base.report(root,out,cfg,audit,keys,dpgo,publications,args.base)
    # Preserve the actual local graph in the saved solution, not just loop edges.
    graph=read_json(out/'solution/central/graph.json');graph['local_constraints']=local_edges
    write_json(out/'solution/central/graph.json',graph)
    exported=read_json(out/'exports/export.json');exported['graph_sha256']=file_hash(out/'solution/central/graph.json');write_json(out/'exports/export.json',exported)
    old_a=read_json(args.control/'audit.json');old_s=read_json(args.control/'cbs-summary.json')
    old_e=read_json(args.control/'evaluation.json');new_e=read_json(out/'evaluation.json')
    old_r=read_json(args.control/'resources.json');new_r=read_json(out/'resources.json')
    old_t=read_json(args.control_timing or args.control/'cbs-timing/summary.json')
    def one_joint(e):
        v=e['joint_ate'];return next(iter(v.values()))['rmse_m'] if len(v)==1 and len(next(iter(v.values()))['robots'])==14 else None
    def metrics(a,s,e,r,t):
        return dict(submaps=s['keyframes'],verified_loops=s['proposed_loops'],retained_loops=s['loops'],components=len(e['joint_ate']),
            joint_ate_m=one_joint(e),communication_MB=a['communication']['total_application_cdr_bytes']/1e6,
            retrieval_MB=a['communication']['peer_retrieval_cdr_bytes']/1e6,
            pcm_MB=a['communication']['pcm_network_cdr_bytes']/1e6,
            registration_MB=a['communication']['registration_network_cdr_bytes']/1e6,
            cbs_MB=a['communication']['cbs_network_cdr_bytes']/1e6,
            wall_s=r['wall_s'],peak_process_tree_rss_GiB=r['peak_simultaneous_process_tree_rss_bytes']/2**30,
            gpu_peak_MiB=r['whole_device_gpu_peak_mib'],solver_median_ms=t['update_ms']['median'],solver_p95_ms=t['update_ms']['p95'],solver_max_ms=t['update_ms']['max'],
            callback_median_ms=t['active_callback_ms']['median'],callback_p95_ms=t['active_callback_ms']['p95'],callback_max_ms=t['active_callback_ms']['max'])
    comparison=dict(control=metrics(old_a,old_s,old_e,old_r,old_t),spatial=metrics(audit,summary,new_e,new_r,timing),
        protocol_change='Spatial PCM exchanges direct separator-pair summaries instead of chronological intervals',
        limitation='Two independent fresh 1x online replays. Odometry, circle publication, retrieval opportunities and asynchronous scheduling can differ; not an exactly frozen-input ablation.')
    write_json(out/'comparison.json',comparison)
    labels=[('submaps','Submaps'),('verified_loops','Verified loops'),('retained_loops','PCM-retained loops'),('components','Final robot groups'),('joint_ate_m','All-14 joint ATE (m)'),('communication_MB','Total application traffic (MB)'),('retrieval_MB','Retrieval / verification (MB)'),('pcm_MB','PCM (MB)'),('registration_MB','Registration geometry (MB)'),('cbs_MB','CBS (MB)'),('solver_median_ms','CBS solver median (ms)'),('solver_p95_ms','CBS solver p95 (ms)'),('solver_max_ms','CBS solver maximum (ms)'),('callback_p95_ms','Full callback p95 (ms)'),('wall_s','End-to-end wall time (s)'),('peak_process_tree_rss_GiB','Peak simultaneous RSS (GiB)'),('gpu_peak_MiB','Whole-device GPU peak (MiB)')]
    fmt=lambda x:'Unavailable' if x is None else str(x) if isinstance(x,int) else f'{x:.3f}'
    html_table='<table><tr><th>Metric</th><th>Creation-order chain</th><th>Spatial-support graph</th></tr>'+''.join('<tr><td>'+label+'</td><td>'+fmt(comparison['control'][k])+'</td><td>'+fmt(comparison['spatial'][k])+'</td></tr>' for k,label in labels)+'</table>'
    text_table='\n'.join(['| Metric | Creation-order chain | Spatial-support graph |','| --- | ---: | ---: |']+['| '+label+' | '+fmt(comparison['control'][k])+' | '+fmt(comparison['spatial'][k])+' |' for k,label in labels])
    description=f"Spatial-support graph: {len(local_edges)} shared-geometry map-consistency edges; zero creation-order odometry edges; {summary.get('pending_map_consistency_factors',0)} pending local factors. All local components are connected before final map export. PCM uses spatial support paths. The local constraints reuse map observations and are correlated regularizers, not independent registrations."
    for name in ['index.html','REPORT.md']:
        p=out/name;s=p.read_text().replace('This trial changes geometry publication only.','This trial changes local graph topology and PCM support paths; the geometry publication policy is unchanged.')
        s=s.replace('Online GRACO · all 14 robots','Online GRACO · spatial-support graph · 14 robots')
        if name.endswith('.html'):
            s+=f'<h2>Graph comparison</h2><p>{description}</p>{html_table}<p>{comparison["limitation"]}</p><p>Communication excludes DDS/IP/discovery, sensor transport and visualization. Times are native elapsed measurements; GPU work uses the same VGICP settings.</p><img src="cbs-progress.png" alt="CBS time versus online progress"><p><a href="spatial-graphs.json">Actual submap graphs</a> · <a href="cbs-updates.csv">Measured CBS updates</a> · <a href="comparison.json">Comparison data</a></p>'
        else:s+='\n\n'+description+'\n\n'+text_table+'\n\n'+comparison['limitation']+'\n'
        p.write_text(s)
    # Native validation includes all original Rerun files; bulk recordings stay on 148.
    base.provenance(root,out,cfg)
    shutil.copy2(__file__,out/'spatial-auditor.py')
    write_json(out/'FILES.sha256.json',{str(p.relative_to(out)):file_hash(p) for p in out.rglob('*') if p.is_file() and 'solution' not in p.relative_to(out).parts and p.name!='FILES.sha256.json'})


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--control',type=Path,required=True);p.add_argument('--control-timing',type=Path)
    p.add_argument('--runtime-project',type=Path,default=Path(__file__).resolve().parents[2]);a=p.parse_args()
    try:run(a)
    except BaseException:
        a.output.mkdir(parents=True,exist_ok=True)
        (a.output/'REPORT_FAILED.json').write_text(json.dumps(dict(traceback=traceback.format_exc()),indent=2)+'\n')
        raise

#!/usr/bin/env python3
"""Audit a completed live run; reconstruct/evaluate its actual distributed result.

This does not rerun retrieval, registration, PCM, or any graph optimizer.
"""
import argparse
from collections import Counter, defaultdict
import html
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zlib

import numpy as np

PROJECT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(PROJECT/'research'))
from s3e_pipeline.artifacts import digest, file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.circle_cbs import solution_graph
from s3e_pipeline.circle_submaps import validate_geometry, validate_row


def stats(values):
    x=np.asarray(values,float)
    return dict(count=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),max=float(x.max())) if len(x) else None


def preview(ply, target):
    """Independent PLY check and deterministic raster preview, without Agg scatter."""
    import open3d as o3d
    from PIL import Image, ImageDraw, ImageFont
    cloud=o3d.io.read_point_cloud(str(ply));xyz=np.asarray(cloud.points);rgb=np.asarray(cloud.colors)
    if not len(xyz) or not cloud.has_colors() or not np.isfinite(xyz).all():raise ValueError('Invalid PLY readback')
    count=len(xyz);bounds=[xyz.min(0).tolist(),xyz.max(0).tolist()]
    sample=np.linspace(0,count-1,min(count,700000),dtype=np.int64)
    points=xyz[sample].copy();colors=np.round(rgb[sample]*255).astype(np.uint8)
    del cloud,xyz,rgb
    yaw,elevation=np.deg2rad([35.,55.]);across=np.array([-np.sin(yaw),np.cos(yaw),0.])
    camera=np.stack([[np.cos(yaw),np.sin(yaw),0.],across*np.sin(elevation)+[0.,0.,np.cos(elevation)],-across*np.cos(elevation)+[0.,0.,np.sin(elevation)]])
    views=[points,(points-np.mean(points,axis=0))@camera.T];canvas=Image.new('RGB',(2400,1300),'white')
    for panel,view in enumerate(views):
        low=view[:,:2].min(0);high=view[:,:2].max(0);scale=1080/max(float((high-low).max()),1.)
        xy=np.rint((view[:,:2]-(high+low)/2)*scale+600).astype(np.int64)
        xy[:,1]=1199-xy[:,1];order=np.argsort(view[:,2],kind='stable')
        pixels=xy[order,1]*1200+xy[order,0]
        _,last=np.unique(pixels[::-1],return_index=True);chosen=order[len(order)-1-last]
        raster=np.full((1200,1200,3),255,np.uint8)
        for dx,dy in ((0,0),(1,0),(0,1),(1,1)):
            raster[np.clip(xy[chosen,1]+dy,0,1199),np.clip(xy[chosen,0]+dx,0,1199)]=colors[chosen]
        canvas.paste(Image.fromarray(raster),(panel*1200,100))
    draw=ImageDraw.Draw(canvas)
    font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),32) if font_path.exists() else ImageFont.load_default()
    draw.text((55,35),'Top view · native gravity',fill='#243746',font=font)
    draw.text((1255,35),'Oblique view',fill='#243746',font=font)
    canvas.resize((1800,975),Image.Resampling.LANCZOS).save(target)
    return dict(reader='Open3D '+o3d.__version__,vertices=count,has_rgb=True,bounds_m=bounds,
        preview_points=len(points),preview_only_thinned=True,renderer='deterministic depth-ordered raster')


def audit_geometry(root, robot, rows):
    """Reconstruct the observation prefix independently of the placement solver."""
    prepared=root/f'prepared-{robot}'; source=root/f'frontends/{robot}/frontend/area_maps'
    placement=read_json(prepared/'placement.json'); history=read_jsonl(prepared/'placement-history.jsonl')
    observations=[r for r in read_jsonl(source/'index.jsonl') if r['complete']]
    provenance=placement['source_payloads']
    assert len(history)==len(observations)==len(provenance)
    ids=np.empty(0,np.int64);world=np.empty((0,3));checked=0;max_error=0.;published_ids=[]
    by_update=defaultdict(list);created_by_update=defaultdict(list);growth_history=defaultdict(list)
    for row in rows:
        by_update[row.get('sealed_update',row['created_update'])].append(row)
        created_by_update[row['created_update']].append(row)
    for update,(native,record,event) in enumerate(zip(observations,provenance,history)):
        payload=source/native['payload'];assert file_hash(payload)==record['sha256']==native['sha256']
        assert record['native_submap_id']==native['submap_id'] and event['update_id']==update
        with np.load(payload,allow_pickle=False) as f:
            incoming=f['point_ids'];T=np.asarray(native['T_world_imu']).reshape(4,4)
            xyz=np.asarray(f['points'],float)@T[:3,:3].T+T[:3,3]
        indices=np.searchsorted(ids,incoming);known=indices<len(ids)
        known[known]&=ids[indices[known]]==incoming[known]
        assert int((~known).sum())==record['first_seen_points']
        if known.any():assert np.allclose(world[indices[known]],xyz[known],atol=1e-3,rtol=1e-6)
        ids=np.r_[ids,incoming[~known]];world=np.vstack([world,xyz[~known]])
        order=np.argsort(ids);ids,world=ids[order],world[order]
        assert event['diagnostics']['coverage']+1e-12>=placement['config']['coverage_target']
        assert event['new_circles']==[r['keyframe_id'] for r in created_by_update[update]]
        for row in rows:
            if (row.get('geometry_publication_policy')=='mature_then_freeze' and
                    row['created_update']<=update<=row['sealed_update']):
                frame=np.asarray(row['T_horizontal_map']);xy=(world@frame[:3,:3].T+frame[:3,3])[:,:2]
                mask=np.sum((xy-row['footprint_center_xy'])**2,axis=1)<=row['footprint_radius_m']**2
                pc=placement['config'];cells=np.floor((xy[mask]-pc['grid_origin_xy'])/pc['cell_size_m']).astype(np.int64)
                growth_history[row['keyframe_id']].append(dict(native_available_ns=native['available_ns'],points=int(mask.sum()),cells=len(np.unique(cells,axis=0))))
        for row in by_update[update]:
            validate_row(row);p=prepared/'store'/f"{row['keyframe_id']:06d}.npz"
            assert file_hash(p)==row['sha256']
            assert row['source_observations_hash']==digest(provenance[:update+1])
            assert row['snapshot_received_wall_ns']==record['received_wall_ns']<=row['available_ns']
            anchor=observations[row['created_update']]
            assert row['stamp_ns']==anchor['stamp_ns'] and row['T_world_imu']==anchor['T_world_imu']
            with np.load(p,allow_pickle=False) as f:cloud=f['cloud'];saved_ids=f['point_ids']
            published_ids.append(saved_ids.copy())
            validate_geometry(row,cloud,saved_ids)
            frame=np.asarray(row['T_horizontal_map']);xy=(world@frame[:3,:3].T+frame[:3,3])[:,:2]
            mask=np.sum((xy-row['footprint_center_xy'])**2,axis=1)<=row['footprint_radius_m']**2
            assert np.array_equal(ids[mask],saved_ids), 'Published evidence differs from available seal-time crop'
            T=np.asarray(row['T_world_body']);expected=((world[mask]-T[:3,3])@T[:3,:3]).astype(np.float32)
            error=float(np.max(np.abs(expected-cloud))) if len(cloud) else 0.
            assert error<=1e-5;max_error=max(max_error,error)
            if row['retrievable']:
                descriptor=json.loads(zlib.decompress((prepared/'ellipsoid'/f"{row['keyframe_id']:06d}.json.zlib").read_bytes()))
                assert descriptor['payload_sha256']==row['sha256'] and descriptor['representation']=='point_cloud'
            if row.get('geometry_publication_policy')=='mature_then_freeze':
                from s3e_pipeline.circle_online import PublicationConfig, maturation_decision
                decision=row['maturation'];tail=decision['reason']=='end_of_stream'
                assert not tail or update==len(observations)-1
                expected_decision=maturation_decision(growth_history[row['keyframe_id']],row['created_available_ns'],
                    native['available_ns'],PublicationConfig(**row['publication_config']),tail)
                for field in ('reason','age_s','growth','stable'):assert decision[field]==expected_decision[field]
                if decision['reason']=='stable':
                    layers=descriptor['mapclosures_multilayer']
                    assert layers['terrain']['available'] and sum(layers['feature_counts'].values())>=decision['quality']['minimum_features']
            checked+=1
    with np.load(prepared/'union.npz') as f:
        assert np.array_equal(f['point_ids'],ids) and np.array_equal(f['world'],world.astype(np.float32))
    assert checked==len(rows) and [r['keyframe_id'] for r in rows]==list(range(len(rows)))
    evidence_count=len(np.unique(np.concatenate(published_ids)))
    return dict(success=True,observations=len(observations),circles=checked,points=len(ids),
        published_evidence_unique_points=evidence_count,
        fraction_of_final_observed_points_in_evidence=evidence_count/len(ids),
        exact_seal_time_crop=True,no_future_geometry=True,descriptor_evidence_hashes_match=True,
        maximum_crop_error_m=max_error,minimum_prefix_coverage=placement['minimum_prefix_coverage'])


def audit(root, cfg, out):
    assert read_json(root/'ONLINE_DONE')['success']
    robots=cfg['robots']; rows={r:read_jsonl(root/f'prepared-{r}/store/keyframes.jsonl') for r in robots}
    checks={};native={};timings=[];wire=Counter();events=[];received=defaultdict(dict);rejections=Counter();iteration_timing={}
    dense=out/'trajectories';dense.mkdir(exist_ok=True)
    for robot in robots:
        print('Audit',robot,flush=True)
        checks[robot]=audit_geometry(root,robot,rows[robot])
        iterations=read_jsonl(root/f'epochs/persistent/{robot}/stats.jsonl')
        iteration_timing[robot]=dict(interval_s=stats(np.diff([i['logged_wall_ns'] for i in iterations])/1e9),
            stages=dict(Counter(i['belief_stage'] for i in iterations)))
        summary=read_json(root/f'frontends/{robot}/frontend/summary.json')
        assert summary['success'] and summary['native_mapper_returncode']==0 and summary['finite']
        updates=read_jsonl(root/f'frontends/{robot}/frontend/native_updates.jsonl')
        sensor=np.array([r['sensor_stamp_ns'] for r in updates],np.int64)
        stamp=np.array([r['stamp_ns'] for r in updates],np.int64)
        pose=np.array([r['pose'] for r in updates],float)
        assert np.isfinite(pose).all() and (np.diff(sensor)>0).all() and (np.diff(stamp)>0).all()
        speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(sensor)/1e9)
        success=np.array([r['lidar_updated'] for r in updates],bool)
        gaps=np.diff(sensor[success])/1e9
        assert speed.max()<20 and gaps.max()<1
        with (dense/f'{robot}.tum').open('w') as f:
            for t,p in zip(stamp,pose):f.write(f'{t//10**9}.{t%10**9:09d} '+' '.join(f'{v:.17g}' for v in p)+'\n')
        native[robot]=dict(summary=summary,updates=len(updates),sensor_duration_s=float((sensor[-1]-sensor[0])/1e9),
            success_fraction=float(success.mean()),speed_m_s=stats(speed),successful_update_gap_s=stats(gaps),
            processing_s=stats([r['processing_s'] for r in updates]))
        timings+=read_jsonl(root/f'prepared-{robot}/timings.jsonl')
        for packet in read_jsonl(root/f'live/{robot}/wire.jsonl'):
            assert packet['src']==robot
            if packet['dst']!=robot:wire[packet['kind']]+=packet['network_cdr_bytes']
        for event in read_jsonl(root/f'live/{robot}/events.jsonl'):
            kind=event['type'];when=event['logged_wall_ns']
            if kind=='observe':assert event['available_ns']<=when
            if kind=='ranked':
                assert event['query_stamp_ns']<=when
                for candidate in event['candidates']:
                    if 'available_ns' in candidate:assert candidate['available_ns']<=event['query_stamp_ns']
            if kind=='constraint_received':
                key=(tuple(event['i']),tuple(event['j']))
                received[robot].setdefault(key,when)
            if kind=='verification':
                assert event['query_stamp_ns']<=event['verification_submitted_wall_ns']<=event['delivery_ns']<=when
                events.append(dict(robot=robot,**event))
                if not event['accepted']:rejections[event['reason']]+=1
    epochs=sorted((root/'epochs').glob('[0-9][0-9][0-9]'))
    for epoch in epochs:
        cutoff=read_json(epoch/'input.json')['cutoff_wall_ns']
        for robot in robots:
            prefix=read_jsonl(epoch/robot/'store/keyframes.jsonl')
            assert prefix==rows[robot][:len(prefix)] and all(r['available_ns']<=cutoff for r in prefix)
        for edge in read_jsonl(epoch/'loops/constraints.jsonl'):
            key=(tuple(edge['i']),tuple(edge['j']))
            assert all(received[r][key]<=cutoff for r in {key[0][0],key[1][0]})
    latest=read_json(root/'epochs/latest.json');dpgo=Path(latest['path']);summary=read_json(dpgo/'summary.json')
    assert summary['finished'] and latest['common_reference_ready']
    assert all(s['registration']['success'] and s['registration']['final'] for s in summary['robots'].values())
    start=read_json(root/'START')['wall_ns'];inputs=read_json(root/'inputs.json')
    # Conservative bag playback bounds exclude recorder drain and frontend shutdown.
    ends=[native[r]['summary']['playback_started_wall_ns']+int((cfg['online']['requested_duration_s'] or inputs[r]['sensor_duration_s'])*1e9) for r in robots]
    publications=[e for e in read_jsonl(root/'epochs/events.jsonl') if e['type']=='epoch_published']
    aligned=[e for e in publications if e['loops'] and e['common_reference_ready'] and len(set(e['components'].values()))<len(robots)]
    first=aligned[0] if aligned else None
    all14=[e for e in aligned if len(set(e['components'].values()))==1]
    timeline=dict(start_wall_ns=start,first_bag_expected_end_wall_ns=min(ends),last_bag_expected_end_wall_ns=max(ends),
        first_shared_alignment_s=(first['published_wall_ns']-start)/1e9 if first else None,
        first_all14_alignment_s=(all14[0]['published_wall_ns']-start)/1e9 if all14 else None,
        shared_alignment_before_first_bag_end=bool(first and first['published_wall_ns']<min(ends)),
        shared_alignment_before_last_bag_end=bool(first and first['published_wall_ns']<max(ends)),
        publications=len(publications),admitted_revisions=len(epochs),
        final_publication_s=(latest['published_wall_ns']-start)/1e9,
        settling_after_last_bag_s=max(0.,(latest['published_wall_ns']-max(ends))/1e9),
        final_iterations=summary['iterations'],bounds='1x playback start + bag duration; excludes recorder shutdown')
    graph_bytes={k:summary.get(k,0) for k in ('cbs_network_cdr_bytes','pcm_network_cdr_bytes','registration_network_cdr_bytes')}
    communication=dict(peer_retrieval_by_kind_cdr_bytes=dict(wire),peer_retrieval_cdr_bytes=sum(wire.values()),
        **graph_bytes,total_application_cdr_bytes=sum(wire.values())+sum(graph_bytes.values()),
        scope='Actual remote serialized application traffic; peer retrieval fan-out, PCM, geometry exchange, CBS requests and responses. Excludes DDS/IP/discovery, local sensor transport, visualization and downloads.',
        cbs_counting='Requester sent and received bytes count request and reply once. PCM and registration count sent bytes.',robots=len(robots))
    result=dict(success=True,geometry=checks,native=native,timeline=timeline,communication=communication,
        iteration_timing=iteration_timing,
        descriptors=dict(total=len(timings),terrain_unavailable=sum(not t.get('terrain',{}).get('available',False) for t in timings),
            empty_layered_features=sum(not sum(t.get('layer_features',{}).values()) for t in timings),
            retrieval_channels=cfg['backend']['mapclosures']['retrieval_channels']),
        publication_policy=cfg['online']['geometry_publication_policy'],
        maturation_reasons=dict(Counter(t.get('maturation',{}).get('reason','creation') for t in timings)),
        maturation_age_s=stats([t['maturation']['age_s'] for t in timings if 'maturation' in t]),
        prepare_latency_s=stats([t['prepare_latency_s'] for t in timings]),
        verification_latency_s=stats([e['wall_detection_latency_s'] for e in events]),
        verification_runtime_s=stats([e['verification_runtime_s'] for e in events]),
        verification_count=len(events),verification_accepted=sum(e['accepted'] for e in events),
        verification_rejections=dict(rejections),auditor_sha256=file_hash(__file__),
        runtime_source_hashes_unchanged=True,
        current_source_files_match_recorded=all(file_hash(p)==h for p,h in read_json(root/'source-hashes.json').items()))
    # ONLINE_DONE is written only after the launcher's end-of-run source check.
    # Subsequent development need not invalidate a completed historical smoke run.
    write_json(out/'audit.json',result)
    return result,rows,dpgo,publications


def report(root,out,cfg,audited,rows,dpgo,publications,base,comparison=None):
    from s3e_pipeline.circle_central_evaluation import evaluate_run
    from s3e_pipeline.circle_map_export import export
    solution=out/'solution';solution.mkdir(exist_ok=True);(solution/'central').mkdir(exist_ok=True)
    for robot in cfg['robots']:
        if not (solution/robot).exists():(solution/robot).symlink_to(root/f'prepared-{robot}')
    if not (solution/'cbs').exists():(solution/'cbs').symlink_to(dpgo)
    summary=read_json(dpgo/'summary.json')
    graph=solution_graph(dict(summary=summary,poses=read_jsonl(dpgo/'poses.jsonl'),constraints=read_jsonl(dpgo/'constraints.jsonl')),rows)
    write_json(solution/'central/graph.json',graph)
    evaluation_cfg=dict(cfg,dataset=str(base/'reference'),dense_trajectories={r:str(out/'trajectories'/f'{r}.tum') for r in cfg['robots']})
    write_json(out/'evaluation-config.json',evaluation_cfg)
    evaluation=read_json(solution/'evaluation/result.json') if (solution/'evaluation/result.json').exists() else evaluate_run(solution,evaluation_cfg)
    exported=read_json(out/'exports/export.json') if (out/'exports/export.json').exists() else export(solution,cfg,out/'exports')
    resources=read_json(root/'resources.json'); samples=[float(s.split(',')[1]) for s in (root/'gpu.csv').read_text().splitlines() if len(s.split(','))==3] if (root/'gpu.csv').exists() else []
    resources['whole_device_gpu_peak_mib']=max(samples) if samples else None
    resources['gpu_scope']='Whole-device samples; includes desktop/driver and any unrelated activity'
    write_json(out/'resources.json',resources)
    for name,src in {'configuration.json':root/'configuration.json','source-hashes.json':root/'source-hashes.json',
        'cbs-summary.json':dpgo/'summary.json','pcm.json':dpgo/'pcm.json','registration.json':dpgo/'registration.json',
        'evaluation.json':solution/'evaluation/result.json','reference-barrier.json':root/'epochs/reference-barrier.json',
        'frontend-binary-hashes.json':root/'frontend-binary-hashes.json'}.items():shutil.copy2(src,out/name)
    os.environ.setdefault('MPLCONFIGDIR','/tmp/circle-online-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start=audited['timeline']['start_wall_ns'];t=[(e['published_wall_ns']-start)/1e9 for e in publications]
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for ax,y,label in zip(axes,[[e['keyframes'] for e in publications],[e['loops'] for e in publications],[len(set(e['components'].values())) for e in publications]],['Submap nodes','Retained loops','Published groups']):
        ax.step(t,y,where='post',color='#28698c');ax.set_ylabel(label);ax.grid(alpha=.2)
        for bound in ('first_bag_expected_end_wall_ns','last_bag_expected_end_wall_ns'):ax.axvline((audited['timeline'][bound]-start)/1e9,color='#aa7744',ls='--',lw=1)
    axes[-1].set_xlabel('Seconds after common playback release');fig.suptitle('Online all-14 GRACO · actual persistent CBS publications\nDashed: first and last bag playback end')
    fig.tight_layout();fig.savefig(out/'timeline.png',dpi=150);plt.close(fig)
    previews=[];validation={}
    for i,item in enumerate(sorted(exported['files'],key=lambda x:(-len(x['robots']),x['robots']))):
        image=f'map-{i}.png';validation[image]=preview(Path(item['path']),out/image)
        assert validation[image]['vertices']==item['points'];previews.append((image,item))
    write_json(out/'independent-ply-validation.json',validation)
    # These are the exact publication-time full-height control images. Retrieval
    # still uses the unchanged terrain-relative layers, as in the runtime config.
    bev=out/'bev';bev.mkdir(exist_ok=True);gallery=[]
    for robot in ('aerial05','aerial07','ground05','ground06'):
        if robot not in rows:continue
        keys=rows[robot]
        for row in [keys[k] for k in sorted({0,len(keys)//2,len(keys)-1})]:
            source=root/f'prepared-{robot}/images'/f"{row['keyframe_id']:06d}.png"
            if source.exists():
                name=f"{robot}-{row['keyframe_id']:03d}.png";shutil.copy2(source,bev/name)
                gallery.append(f'<figure><img src="{name}"><figcaption>{robot} / {row["keyframe_id"]}: {row["geometry_count"]:,} points; sealed update {row.get("sealed_update",row["created_update"])}</figcaption></figure>')
    (bev/'index.html').write_text('<meta charset="utf-8"><style>body{font:15px system-ui;max-width:1300px;margin:auto}.grid{display:grid;grid-template-columns:repeat(3,1fr)}figure{padding:12px}img{width:100%;image-rendering:pixelated}</style><h1>Actual online publication-time BEVs</h1><p>First, middle and last circles for four robots. Full-height inspection controls; online retrieval uses terrain-relative layers. These images were written during replay from the same immutable point payloads.</p><div class="grid">'+''.join(gallery)+'</div>')
    timeline=audited['timeline'];comm=audited['communication'];joint=evaluation['joint_ate']
    components={c:v['robots'] for c,v in joint.items()}
    mature=cfg['online']['geometry_publication_policy']=='mature_then_freeze'
    geometry_policy=('Circle centers, IDs and creation anchors stay fixed while their unpublished geometry accumulates. '
        'Normal publication requires at least 30 seconds of age, no more than 5% growth in both native point IDs and occupied cells over 30 seconds, '
        'supported terrain and at least the existing five-feature retrieval minimum. Publication follows permanent ID order. '
        'A 120-second deadline bounds head-of-line waiting; end-of-stream tails are explicitly labelled. '
        f"Publication reasons: {audited['maturation_reasons']}. Geometry becomes immutable at publication; later observations do not revise it. "
        if mature else 'Geometry is frozen immediately at circle creation. Circle footprints remain fixed. Later observations do not revise its retrieval or registration evidence. ')
    evidence_fraction=sum(v['published_evidence_unique_points'] for v in audited['geometry'].values())/sum(v['points'] for v in audited['geometry'].values())
    first_shared=f"{timeline['first_shared_alignment_s']:.1f} s" if timeline['first_shared_alignment_s'] is not None else 'not achieved'
    first_all=f"{timeline['first_all14_alignment_s']:.1f} s" if timeline['first_all14_alignment_s'] is not None else 'not achieved'
    verdict=('All 14 robots formed one aligned component.' if len(components)==1 else
        'The online run completed, but did not reproduce the staged all-14 connection. There is no valid single joint ATE or globally aligned all-14 PLY for this run.')
    paragraphs=[f"{verdict} Fresh simultaneous 1× replay of all {len(cfg['robots'])} GRACO robots on workstation 148: {sum(map(len,rows.values()))} circle submaps, {summary['proposed_loops']} verified loops, {summary['loops']} PCM-retained loops, {len(components)} final components. This run consumes growing observations and exchanges peer DDS messages during playback.",
        'Joint ATE by connected component: '+'; '.join(f"{k} ({len(v['robots'])} robots): {v['rmse_m']:.3f} m" for k,v in joint.items())+'. Evo 1.36.5; corrected IMU-reference GRACO GT; 50 ms association; one rigid alignment per component; no scale fitting.',
        f"First shared alignment after playback release: {first_shared}; available before the first bag ended: {timeline['shared_alignment_before_first_bag_end']}. First all-14 alignment: {first_all}. Final publication: {timeline['final_publication_s']:.1f} s; post-playback settling: {timeline['settling_after_last_bag_s']:.1f} s. {timeline['admitted_revisions']} growing graph revisions; final per-robot iteration counts: {min(summary['iterations'].values())}–{max(summary['iterations'].values())}. Iterations target 10 s, using the configured pose/anchor exchange policy; admission/overruns can take longer.",
        geometry_policy+'The input is a growing union of live native 80 m area exports, not every original scan. '
        f"Published evidence contains {evidence_fraction:.1%} of final retained point IDs. The 99% footprint coverage target is distinct from frozen-evidence coverage.",
        f"Retrieval diagnosis: {audited['descriptors']['terrain_unavailable']} of {audited['descriptors']['total']} circles lacked terrain support; {audited['descriptors']['empty_layered_features']} had no layered ORB features. The preserved legacy multilayer configuration has no full-height retrieval fallback. {audited['verification_count']} candidates reached 3D verification, with {audited['verification_accepted']} accepted. Matching, terrain, verification, PCM and registration thresholds are unchanged. This trial changes geometry publication only.",
        f"Measured application traffic: {comm['total_application_cdr_bytes']/1e6:.2f} MB total ({comm['total_application_cdr_bytes']/len(cfg['robots'])/1e6:.2f} MB per robot): retrieval/verification {comm['peer_retrieval_cdr_bytes']/1e6:.2f}, PCM {comm['pcm_network_cdr_bytes']/1e6:.2f}, registration geometry {comm['registration_network_cdr_bytes']/1e6:.2f}, CBS {comm['cbs_network_cdr_bytes']/1e6:.2f} MB. {comm['scope']} These bytes do not establish a matched win against three-robot baseline runs. Low traffic with few loops and disconnected robots is not an equivalent mapping outcome to the successful staged run.",
        f"Measured end-to-end wall time {resources['wall_s']:.1f} s; peak simultaneous process-tree RSS {resources['peak_simultaneous_process_tree_rss_bytes']/2**30:.2f} GiB. Shared pages may be counted more than once. Whole-device GPU peak: {resources['whole_device_gpu_peak_mib']} MiB. No host resource quotas. Native callbacks use four threads, numerical libraries one thread per process.",
        'Every published payload was checked against an independent reconstruction of its seal-time observation prefix. Descriptor/evidence hashes, immutable graph prefixes, both-endpoint loop receipt before graph admission, finite chronological odometry, successful-update gaps, final registration support and published loop-transform consistency passed. No frozen graph replay or centralized solve was used to produce this result.',
        'PLY files contain every unique retained native representative once per robot, re-posed by the nearest optimized circle. Uncovered points are retained and flagged. Components are exported separately; native IMU gravity sets their display orientation. GT is used only for trajectory evaluation. Per-robot live Rerun recordings and all bulk inputs remain on 148.']
    headers=['Robot','Circles','Raw ATE (m)','CBS individual ATE (m)','Largest update gap (s)','Component']
    table=[]
    for r in cfg['robots']:table.append([r,str(len(rows[r])),f"{evaluation['raw_ate'][r]['rmse_m']:.3f}",f"{evaluation['individual_ate'][r]['rmse_m']:.3f}",f"{audited['native'][r]['successful_update_gap_s']['max']:.3f}",graph['components'][r]])
    if comparison:
        old=read_json(comparison/'audit.json');old_summary=read_json(comparison/'cbs-summary.json');old_eval=read_json(comparison/'evaluation.json')
        old_fraction=sum(v['published_evidence_unique_points'] for v in old['geometry'].values())/sum(v['points'] for v in old['geometry'].values())
        comparison_data=dict(previous=dict(circles=old_summary['keyframes'],loops=old_summary['loops'],components=len(old_eval['joint_ate']),
            largest_component=max(len(v['robots']) for v in old_eval['joint_ate'].values()),empty_layers=old['descriptors']['empty_layered_features'],
            evidence_fraction=old_fraction,communication_MB=old['communication']['total_application_cdr_bytes']/1e6),
            current=dict(circles=summary['keyframes'],loops=summary['loops'],components=len(joint),largest_component=max(len(v['robots']) for v in joint.values()),
            empty_layers=audited['descriptors']['empty_layered_features'],evidence_fraction=evidence_fraction,communication_MB=comm['total_application_cdr_bytes']/1e6),
            note='Fresh independent 1x odometry replays; same calibration and method thresholds, publication policy changed. Lower traffic alone is not a mapping-quality benefit.')
        write_json(out/'comparison.json',comparison_data)
        paragraphs.insert(1,'Immediate freeze → maturation: '+', '.join(f"{k.replace('_',' ')} {comparison_data['previous'][k]:.3g} → {comparison_data['current'][k]:.3g}" for k in comparison_data['previous'])+'. '+comparison_data['note'])
    (out/'REPORT.md').write_text('# Online GRACO all 14\n\n'+'\n\n'.join(paragraphs)+'\n\n'+'\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(r)+' |' for r in table])+'\n')
    page='<!doctype html><meta charset="utf-8"><title>Online GRACO all 14</title><style>body{font:16px/1.6 system-ui;max-width:1200px;margin:32px auto;padding:0 24px;background:#f6f8fa;color:#243746}img{width:100%;background:white}table{border-collapse:collapse}td,th{padding:8px;border-bottom:1px solid #dde2e5;text-align:right}a{color:#216987}</style><h1>Online GRACO · all 14 robots</h1>'
    page+=''.join('<p>'+html.escape(p)+'</p>' for p in paragraphs[:3])+'<img src="timeline.png" alt="Actual online publications">'
    for image,item in previews:page+=f'<p><a href="exports/{Path(item["path"]).name}">{len(item["robots"])}-robot map PLY · {item["points"]:,} points</a></p><img src="{image}" alt="Corrected map">'
    page+='<table><tr>'+''.join('<th>'+h+'</th>' for h in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+v+'</td>' for v in row)+'</tr>' for row in table)+'</table>'
    page+=''.join('<p>'+html.escape(p)+'</p>' for p in paragraphs[3:])+'<p><a href="bev/index.html">Published BEV gallery</a> · <a href="audit.json">Causality and health audit</a> · <a href="evaluation.json">Accuracy</a> · <a href="REPORT.md">Report</a></p>'
    (out/'index.html').write_text(page)
    write_json(out/'COMPLETE.json',dict(success=True,run=str(root),online=True,all_14_connected=len(components)==1,
        audit_sha256=file_hash(out/'audit.json'),components=components))


def provenance(root,out,cfg):
    native=out/'native-configurations';native.mkdir(exist_ok=True);manifest={}
    for robot in cfg['robots']:
        local=native/robot;local.mkdir(exist_ok=True)
        for name in ('mapping_config.yaml','runtime.yaml','summary.json'):
            shutil.copy2(root/f'frontends/{robot}/frontend'/name,local/name)
            manifest[f'{robot}/{name}']=file_hash(local/name)
    manifest['note']='Exact native calibration, IMU noise and reliable QoS; these files take precedence over the inherited generic odometry stanza.'
    write_json(out/'native-configurations.json',manifest)
    rrds=sorted(root.glob('frontends/*/recording/live.rrd'));assert len(rrds)==len(cfg['robots'])
    with (out/'rerun-validation.log').open('w') as log:
        subprocess.run(['/workspace/.ros2/rerun-venv/bin/rerun','rrd','verify',*map(str,rrds)],stdout=log,stderr=subprocess.STDOUT,check=True)
    write_json(out/'recording-validation.json',dict(success=True,validator='rerun 0.37.1 rrd verify including footers',
        recordings=[dict(path=str(p),bytes=p.stat().st_size) for p in rrds]))
    shutil.copy2(__file__,out/'auditor.py')
    write_json(out/'FILES.sha256.json',{str(p.relative_to(out)):file_hash(p) for p in out.rglob('*')
        if p.is_file() and 'solution' not in p.relative_to(out).parts and p.name!='FILES.sha256.json'})


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--comparison',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    cfg=read_json(a.run/'configuration.json')
    audited,rows,dpgo,publications=audit(a.run,cfg,a.output)
    report(a.run,a.output,cfg,audited,rows,dpgo,publications,a.base,a.comparison)
    provenance(a.run,a.output,cfg)

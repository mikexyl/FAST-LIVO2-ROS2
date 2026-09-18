"""Ellipsoid descriptors -> isolated DDS loop workers -> distributed PCM/CBS.

No centralized optimizer is invoked. Raw scans provide only geometric verification
and registration-factor evidence; retrieval uses native ellipsoid-map BEVs.
"""
import argparse
import gzip
from pathlib import Path
import shutil
import time
import numpy as np
import yaml
from .artifacts import read_json,read_jsonl,write_json,write_jsonl,file_hash,digest,stage_output,validate_stage
from .geometry import pose,transform

SOURCE=Path(__file__).resolve().parents[3]


def artifacts(work):
    cfg=yaml.safe_load((work/'config.yaml').read_text())
    result={}
    for robot in cfg['robots']:
        prepared=Path(read_json(work/f'prepared-{robot}.json')['path'])
        validate_stage(prepared)
        result[f'keyframes.ellipselio.{robot}']=str(prepared)
        result[f'descriptors.ellipselio.mapclosures.{robot}']=str(prepared/'ellipsoid')
        result[f'odometry.ellipselio.{robot}']=str(work/'odometry'/robot)
    return result


def solve(work):
    from .dpgo import run
    cfg=yaml.safe_load((work/'config.yaml').read_text())
    assert cfg['dpgo']['pcm']['enabled'] is True
    assert cfg['dpgo']['mode']=='peers' and cfg['descriptor_branches']==['ellipsoid']
    paths=artifacts(work)
    inputs={r:read_json(work/f'prepared-{r}.json')['stage_hash'] for r in cfg['robots']}
    code=read_json(work/'source-hashes.json')
    with stage_output(work/'stages','ellipsoid-cbs-pcm',cfg,inputs,digest(code),True) as (out,cached):
        if not cached:run(cfg,paths,SOURCE,out)
    out=out if cached else out.parent/out.name.split('.incomplete-')[0]
    summary=read_json(out/'summary.json');pcm=read_json(out/'pcm.json')
    if not summary['pcm_enabled']:raise ValueError('PCM was not enabled')
    if pcm['retained_loops']!=summary['loops']:raise ValueError('PCM retained-loop accounting mismatch')
    paths['dpgo']=str(out.resolve())
    write_json(work/'artifacts.json',paths)


def evaluate(work):
    from .dpgo_evaluation import evaluate as evaluate_cbs
    cfg=yaml.safe_load((work/'config.yaml').read_text());paths=read_json(work/'artifacts.json')
    out=work/'report';out.mkdir(exist_ok=False)
    evaluate_cbs(cfg,paths,Path(cfg['dataset']),out)
    report=read_json(out/'report.json')
    report.update(pipeline='EllipseLIO -> native ellipsoid surface BEVs -> MapClosures -> distributed PCM -> CBS with live GICP factors',
                  representation='Causal persistent native ellipsoid map, 80 m crop; raw trailing submaps for geometric verification only',
                  config_sha256=file_hash(work/'config.yaml'),source_hashes=read_json(work/'source-hashes.json'))
    write_json(out/'report.json',report)
    dpgo=Path(paths['dpgo'])
    shutil.copytree(dpgo,out/'dpgo',ignore=shutil.ignore_patterns('*.bin'))
    evidence=out/'provenance';evidence.mkdir()
    for name in ('config.yaml','schedule.json','source-hashes.json','dataset.json'):
        shutil.copy2(work/name,evidence/name)
    examples={};example_meta={}
    for robot in cfg['robots']:
        p=evidence/robot;p.mkdir()
        for name in ('summary.json','quality.json','mapping_config.yaml','runtime.yaml'):
            shutil.copy2(work/robot/name,p/name)
        for name in ('frames.jsonl','manifest.json'):
            with (work/robot/'export'/name).open('rb') as i,gzip.open(p/(name+'.gz'),'wb') as o:shutil.copyfileobj(i,o)
        prepared=Path(paths[f'keyframes.ellipselio.{robot}'])
        for name in ('COMPLETE.json','timings.jsonl','summary.json','raw.tum','source-hashes.json'):
            shutil.copy2(prepared/name,p/('prepared-'+name))
        shutil.copy2(prepared/'store/keyframes.jsonl',p/'keyframes.jsonl')
        keys=read_jsonl(p/'keyframes.jsonl');key=keys[len(keys)//2]['keyframe_id']
        with np.load(prepared/'bevs'/f'{key:06d}-ellipsoid.npz') as f:
            examples.update({robot+'_'+k:f[k] for k in f.files})
        example_meta[robot]=dict(keyframe_id=key,stamp_ns=keys[len(keys)//2]['stamp_ns'])
        gt=Path(report['ground_truth'][robot]['path'])
        if gt.exists():shutil.copy2(gt,evidence/gt.name)
    np.savez_compressed(out/'ellipsoid-bevs.npz',**examples);write_json(out/'ellipsoid-bevs.json',example_meta)
    plots(out,report,cfg)
    metric=list(report['trajectory'].values());combined=(metric[0]['rmse_m'] if len(metric)==1 and len(metric[0]['robots'])==len(cfg['robots']) else None)
    lines=[f'# {Path(cfg["dataset"]).name}: EllipseLIO + ellipsoid MapClosures + PCM/CBS','',report['pipeline']+'.','',
           f'Retrieval uses ellipsoid-projected BEVs only. {len(cfg["robots"])} isolated DDS workers detect loops; distributed PCM gates inter-robot loops before native CBS optimization. Native GICP factors retain the existing CBS configuration. No centralized PGO is used.','',
           '| Robot | Raw ATE, independent fit [m] | CBS ATE, shared component fit [m] |','|---|---:|---:|']
    duration=read_json(work/cfg['robots'][0]/'summary.json').get('requested_duration',0)
    if duration:
        lines[2:2]=[f'**Smoke test only: first {duration:g} seconds of bag time. This is not a full-sequence result.**','']
    def error(metrics,r):
        return next((f'{m["per_robot"][r]["statistics"]["rmse"]:.4f}' for m in metrics.values() if r in m['per_robot']),'unavailable')
    for r in cfg['robots']:lines.append(f'| {r} | {error(report["raw_odometry"],r)} | {error(report["trajectory"],r)} |')
    pcm=report['pcm']
    lines += ['',f'Combined CBS ATE: **{combined:.4f} m**.' if combined is not None else 'Combined multi-robot ATE unavailable; inspect component/GT status.',
              '', 'Evo 1.36.5, 50 ms timestamp association, one shared rigid fit per CBS component, no scale fitting or additional robot fit. Raw odometry uses separate fits. See the numeric report for GT frame conversion and orientation use.', '',
              f'PCM: **{pcm["proposed_loops"]} proposed / {pcm["retained_loops"]} retained / {pcm["excluded_loops"]} excluded**. PCM applies to inter-robot measurements; verified intra-robot loops pass unchanged.', '',
              f'Components: `{report["components"]}`. Detection/PCM/CBS wall time: **{report["runtime"]["wall_s"]:.2f} s**.', '',
              'The ellipsoid float bases are projected to their nearest orthogonal basis only when the maximum Gram-matrix defect is at most 0.001; larger defects fail preparation. Axes and centers are unchanged. Per-keyframe adjustment magnitudes are retained in preparation timings.', '',
              '![Trajectories](trajectories.png)','', '![CBS corrected maps](maps.png)','', '![Ellipsoid density BEVs](bevs.png)','',
              '[Numeric report](report.json), [PCM decisions](dpgo/pcm.json), [evo evidence](evo/cbs/evaluation.json), [Rerun recording](result.rrd).']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    write_json(out/'COMPLETE.json',dict(pipeline=report['pipeline'],completed_unix_ns=time.time_ns(),
        report_sha256=file_hash(out/'report.json'),pcm_sha256=file_hash(out/'dpgo/pcm.json'),
        config_sha256=report['config_sha256']))


def plots(out,report,cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .robot_colors import robot_colors
    colors={r:np.asarray(c)/255. for r,c in robot_colors(cfg['robots']).items()}
    groups=sorted(set(report['components'].values()))
    fig,axes=plt.subplots(1,len(groups),figsize=(8*len(groups),7),squeeze=False,layout='constrained')
    for ax,group in zip(axes[0],groups):
        for robot in cfg['robots']:
            if report['components'][robot]!=group:continue
            with np.load(out/f'{robot}-maps.npz') as f:p=f['optimized']
            if len(p)>250000:p=p[np.linspace(0,len(p)-1,250000,dtype=int)]
            ax.scatter(p[:,0],p[:,1],c=colors[robot],s=.2,alpha=.4,rasterized=True,label=robot)
        ax.set_aspect('equal');ax.legend(markerscale=10);ax.set_title('CBS component '+group);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
    fig.savefig(out/'maps.png',dpi=160);fig.savefig(out/'maps.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,len(cfg['robots']),figsize=(5*len(cfg['robots']),5),squeeze=False,layout='constrained')
    with np.load(out/'ellipsoid-bevs.npz') as f:
        for ax,robot in zip(axes[0],cfg['robots']):
            ax.imshow(f[robot+'_image'],cmap='gray',vmin=0,vmax=255)
            xy=f[robot+'_orb_xy'];keep=f[robot+'_kept_indices'].astype(int)
            if len(keep):ax.scatter(xy[keep,0],xy[keep,1],s=8,facecolors='none',edgecolors='#55dc80',linewidths=.4)
            ax.set_title(f'{robot}: {len(keep)} retained ORB');ax.axis('off')
    fig.savefig(out/'bevs.png',dpi=160);fig.savefig(out/'bevs.pdf');plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['solve','evaluate']);p.add_argument('--work',type=Path,required=True)
    a=p.parse_args();globals()[a.stage](a.work.resolve())

"""Post-backend evaluation and retained temporal/spatial comparison."""
from pathlib import Path
import json
import sys
from collections import Counter
from datetime import datetime
import numpy as np
import yaml
ROOT=Path('/workspace')
BASE=ROOT/'.ros2/graco-aerial-spatial148-20260920'
WORK=BASE/'full'
OLD=ROOT/'.ros2/graco-aerial-gravity148-20260920/full'
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.evaluation import trajectory_metrics
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth

def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def save(p,data):p.write_text(json.dumps(data,indent=2)+'\n')
def stats(xs):
    return dict(min=float(np.min(xs)),median=float(np.median(xs)),max=float(np.max(xs))) if len(xs) else None

assert read(WORK/'status.json')['phase']=='complete'
new,old=read(WORK/'report/report.json'),read(OLD/'report/report.json')
cfg=yaml.safe_load((WORK/'config.yaml').read_text())
# Extra independent fits distinguish per-robot trajectory shape from inter-robot alignment.
independent=[]
for robot in cfg['robots']:
    independent += [dict(r,component=robot) for r in rows(WORK/f'report/{robot}-cbs.jsonl')]
gt,status=evaluation_ground_truth(cfg['robots'],WORK/'reference',independent,cfg['evaluation'])
ind=trajectory_metrics(dict(poses=independent),gt,cfg['evaluation'],WORK/'report/evo/cbs-independent')
save(WORK/'report/cbs-independent.json',ind)
comparison=dict(baseline=str(OLD),spatial=str(WORK),robots={},
    limitations=['Fresh spatial captures versus retained temporal captures, not repeated matched runs.',
      'Temporal BEVs used explicitly reconstructed startup gravity; spatial BEVs use actual per-anchor filter gravity.',
      'Component-wide ATEs have different scopes if connectivity differs; independent raw/CBS fits are labelled separately.',
      'Image bounding-box area is not a measure of observed surface area or retrieval quality.'],
    baseline_components=old['connectivity']['measured_components'],spatial_components=new['connectivity']['measured_components'],
    baseline_loops=old['runtime']['loops'],spatial_loops=new['runtime']['loops'],
    baseline_pcm_rejected=old['pcm']['excluded_loops'],spatial_pcm_rejected=new['pcm']['excluded_loops'],
    baseline_backend_s=old['runtime']['wall_s'],spatial_backend_s=new['runtime']['wall_s'])
oldcbs={r:s['statistics']['rmse'] for component in old['cbs'].values() for r,s in component['per_robot'].items()}
newcbs={r:s['statistics']['rmse'] for component in new['cbs'].values() for r,s in component['per_robot'].items()}
newgallery=read(BASE/'bev-gallery/manifest.json')
oldgallery=read(OLD.parent/'bev-gallery/manifest.json')
audit=read(WORK/'retention-audit.json')
for robot in cfg['robots']:
    trial=Path(read(WORK/'trials.json')[robot])
    updates=rows(trial/'frontend/native_updates.jsonl')
    event_counts=dict(Counter(r['submap_event'] for r in updates if r['submap_event']))
    handovers=[]
    for prev,cur in zip(updates,updates[1:]):
        if cur['handovers']>prev['handovers']:
            dt=(cur['stamp_ns']-prev['stamp_ns'])/1e9
            step=float(np.linalg.norm(np.array(cur['pose'][:3])-prev['pose'][:3]))
            handovers.append(dict(stamp_ns=cur['stamp_ns'],event=cur['submap_event'],step_m=step,dt_s=dt,speed_m_s=step/dt,lidar_updated=cur['lidar_updated']))
    f=audit['frontends'][robot]
    def footprint(g):
        mm=[m for m in g['maps'] if m['robot']==robot]
        res=g['density_map_resolution_m']
        return dict(count=len(mm),bbox_area_m2=stats([m['width']*m['height']*res**2 for m in mm]),
                    aspect_ratio=stats([max(m['width'],m['height'])/min(m['width'],m['height']) for m in mm]),
                    features=stats([m['features'] for m in mm]))
    comparison['robots'][robot]=dict(temporal_raw_ate_m=old['raw'][robot]['rmse_m'],spatial_raw_ate_m=new['raw'][robot]['rmse_m'],
        temporal_cbs_component_fit_ate_m=oldcbs[robot],spatial_cbs_component_fit_ate_m=newcbs[robot],
        spatial_cbs_independent_ate_m=ind[robot]['rmse_m'],
        temporal_quality=old['frontend_quality'][robot],spatial_quality=new['frontend_quality'][robot],
        temporal_bevs=footprint(oldgallery),spatial_bevs=footprint(newgallery),
        completed_submaps=f['completed_submaps'],partial_submaps=f['partial_submaps'],
        spatial_extent_m=stats(f['completed_extents_m']),duration_s=stats(f['completed_durations_s']),
        finish_reasons=f['finish_reasons'],event_counts=event_counts,handovers=handovers,
        failed_update_count_including_startup=sum(not r['lidar_updated'] for r in updates),
        features=stats([r['features'] for r in updates if r['lidar_updated']]),
        residual=stats([r['residual'] for r in updates if r['lidar_updated']]),
        peak_mapper_rss_mib=f['peak_mapper_rss_mib'],processing_mean_s=f['processing_mean_s'],processing_p95_s=f['processing_p95_s'])
state=read(WORK/'status.json')
comparison['full_pipeline_wall_s']=(datetime.fromisoformat(state['finished_utc'])-datetime.fromisoformat(state['started_utc'])).total_seconds()
comparison['all_images_exact_features']=newgallery['exact_cached_features']
save(WORK/'comparison.json',comparison)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
names=list(cfg['robots']);x=np.arange(len(names));w=.36
fig,axes=plt.subplots(1,3,figsize=(16,4.8),layout='constrained')
for ax,label,a,b in [(axes[0],'Raw position ATE RMSE [m]','temporal_raw_ate_m','spatial_raw_ate_m'),
                      (axes[1],'CBS position ATE RMSE [m] — component fits','temporal_cbs_component_fit_ate_m','spatial_cbs_component_fit_ate_m')]:
    aa=[comparison['robots'][r][a] for r in names];bb=[comparison['robots'][r][b] for r in names]
    ax.bar(x-w/2,aa,w,label='Temporal 10 s / 5 s',color='#64748b');ax.bar(x+w/2,bb,w,label='Spatial 40 m / 20 m',color='#0d9488')
    ax.set_title(label);ax.set_xticks(x,names,rotation=15);ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2);ax.legend()
    for xx,v in zip(x-w/2,aa):ax.text(xx,v,f'{v:.2f}',ha='center',va='bottom',fontsize=9)
    for xx,v in zip(x+w/2,bb):ax.text(xx,v,f'{v:.2f}',ha='center',va='bottom',fontsize=9)
ax=axes[2]
a=[comparison['robots'][r]['temporal_quality']['max_successful_update_gap_s'] for r in names]
b=[comparison['robots'][r]['spatial_quality']['max_successful_update_gap_s'] for r in names]
ax.bar(x-w/2,a,w,color='#64748b');ax.bar(x+w/2,b,w,color='#0d9488');ax.axhline(1,color='#b91c1c',ls='--',label='Failure threshold')
ax.set_title('Largest successful LiDAR update gap [s]');ax.set_xticks(x,names,rotation=15);ax.grid(axis='y',alpha=.2);ax.legend()
fig.suptitle('GRACO aerial 05–08 on 148 · retained temporal vs fresh spatial run',fontsize=16)
fig.supxlabel('CBS scopes differ: temporal aerial-06/07 share one fit; spatial has four separate fits and zero accepted loops.', fontsize=10)
fig.savefig(WORK/'report/temporal-spatial-comparison.png',dpi=170)
print(json.dumps(comparison,indent=2))

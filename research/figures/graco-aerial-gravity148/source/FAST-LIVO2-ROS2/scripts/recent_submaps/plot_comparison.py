#!/usr/bin/env python3
"""Static report figures from saved native diagnostics and evo-aligned poses."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
a=p.parse_args();names=['baseline','enabled']
if (a.report/'repeat/metrics.json').exists():names.append('repeat')
trial_names=dict(baseline='bob-baseline',enabled='bob-enabled',repeat='bob-enabled-repeat')
fig,axes=plt.subplots(3,2,figsize=(14,11),layout='constrained')
for name in names:
    trial=a.evidence/trial_names[name]
    rows=[json.loads(line) for line in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
    stamp=np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)
    elapsed=(stamp-stamp[0])*1e-9
    pose=np.array([r['pose'] for r in rows]);pose_stamp=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(pose_stamp)*1e-9)
    axes[0,0].plot(elapsed[1:],speed,label=name,lw=.8)
    for ax,field,scale in [(axes[0,1],'features',1),(axes[1,0],'age_max_s',1),(axes[1,1],'residual',1),(axes[2,0],'processing_s',1000)]:
        values=[r[field]*scale if r[field] is not None and (field!='residual' or r['lidar_updated']) else np.nan for r in rows]
        ax.plot(elapsed,values,label=name,lw=.8)
    memory=[json.loads(line) for line in (trial/'memory.jsonl').read_text().splitlines()]
    times=np.array([r['wall_ns'] for r in memory],dtype=np.int64)
    axes[2,1].plot((times-times[0])*1e-9,[r.get('VmRSS',0)/1024 for r in memory],label=name)
for ax,title in zip(axes.flat,['Native pose speed [m/s]','Matched features','Maximum correspondence source age [s]',
                              'Successful-update residual [m]','Native processing [ms]','Mapper RSS [MiB]']):
    ax.set_title(title);ax.set_xlabel('Elapsed time [s]');ax.grid(alpha=.2);ax.legend()
axes[0,0].set_yscale('symlog',linthresh=2);axes[0,0].axhline(20,color='red',ls='--',lw=1)
axes[1,0].axhline(10,color='gray',ls='--',lw=1)
fig.savefig(a.report/'diagnostics.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,len(names),figsize=(7*len(names),6),layout='constrained',squeeze=False)
for ax,name in zip(axes[0],names):
    estimate=np.loadtxt(a.report/name/'evo/Bob-estimate-aligned.tum');reference=np.loadtxt(a.report/name/'evo/Bob-reference-matched.tum')
    origin=reference[0,1:3]
    ax.plot(reference[:,1]-origin[0],reference[:,2]-origin[1],color='black',ls='--',label='Position GT')
    ax.plot(estimate[:,1]-origin[0],estimate[:,2]-origin[1],label=name)
    metrics=json.loads((a.report/name/'metrics.json').read_text())
    ax.set_title(f"{name}: {metrics['ate_rmse_m']:.4f} m ATE"+(' (failed)' if not metrics['gate_pass'] else ''))
    ax.axis('equal');ax.set_xlabel('Aligned x [m]');ax.set_ylabel('Aligned y [m]');ax.legend();ax.grid(alpha=.2)
fig.savefig(a.report/'trajectories.png',dpi=170);plt.close(fig)

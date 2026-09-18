#!/usr/bin/env python3
"""Reproduce the retained integration-fixture figure from native messages."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('output',type=Path);args=p.parse_args()
output=args.output
data=json.loads((output/'native/optimized.json').read_text())
tracks={robot:np.array([[e['pose']['position'][k] for k in 'xyz']
                        for e in sorted(packet['estimates'],key=lambda e:e['key']['keyframe_id'])])
        for robot,packet in data.items()}
fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
for robot,name,color in [('0','Alpha','#df554f'),('1','Bob','#3b97dc'),('2','Carol','#52ac76')]:
    xyz=tracks[robot]
    axes[0].plot(xyz[:,0],xyz[:,1],label=name,color=color)
for a,b,label in [('0','1','Alpha–Bob'),('0','2','Alpha–Carol'),('1','2','Bob–Carol')]:
    axes[1].plot(np.linalg.norm(tracks[a]-tracks[b],axis=1)*100,label=label)
axes[0].set_aspect('equal',adjustable='datalim')
axes[0].set_xlabel('X [m]');axes[0].set_ylabel('Y [m]')
axes[0].set_title('Native optimized trajectories\nStraight corridor segment')
axes[1].set_xlabel('Corresponding keyframe');axes[1].set_ylabel('Position disagreement [cm]')
axes[1].set_title('Identical inputs; no ground truth')
for ax in axes:ax.legend();ax.grid(alpha=.2)
fig.suptitle('Workstation 148 · three-robot integration test (not benchmark ATE)')
fig.savefig(output/'smoke.png',dpi=180);fig.savefig(output/'smoke.pdf')

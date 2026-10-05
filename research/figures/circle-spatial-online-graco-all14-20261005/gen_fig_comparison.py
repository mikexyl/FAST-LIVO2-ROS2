#!/usr/bin/env python3
"""Render retained evo trajectories; this script never reruns optimization."""
from pathlib import Path
import html
import json
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT=Path(__file__).resolve().parent
read=lambda name:json.loads((ROOT/name).read_text())
evaluation=read('subset-evaluation/result.json')
audit=read('audit.json')
robots=evaluation['included_robots']
excluded=', '.join(evaluation['excluded_robots'])
palette=['#0072B2','#D55E00','#009E73','#CC79A7','#8F5B31','#6B5BA6',
         '#3B7B84','#BB6677','#8D8C2F','#366C3B','#A45952','#555555']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'pdf.fonttype':42,
    'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.12})
folder=ROOT/'subset-evaluation/evo'
load=lambda mode,robot,suffix:np.loadtxt(folder/mode/f'{robot}-{suffix}.tum')[:,1:4]
references=[load('spatial-joint',r,'reference-matched') for r in robots]
origin=np.concatenate(references).mean(axis=0)
fig,axes=plt.subplots(1,2,figsize=(11,5.8),sharex=True,sharey=True)
for ax,mode,label,key in zip(axes,['control-joint','spatial-joint'],
        ['Previous creation-order chain','Spatial-support graph'],['control_joint_ate','spatial_joint_ate']):
    for robot,reference,color in zip(robots,references,palette):
        ref=reference-origin
        ax.plot(ref[:,0],ref[:,1],color='#bfc3c7',linewidth=2.0,zorder=1)
        xyz=load(mode,robot,'estimate-aligned')-origin
        ax.plot(xyz[:,0],xyz[:,1],color=color,linewidth=.85,alpha=.9,zorder=2)
    error=evaluation[key]['supported_subset']['rmse_m']
    ax.set_title(f'{label}\nShared 12-robot ATE: {error:.3f} m')
    ax.set_aspect('equal',adjustable='box');ax.set_xlabel('x relative to pooled GT centroid (m)')
axes[0].set_ylabel('y relative to pooled GT centroid (m)')
handles=[Line2D([],[],color=c,lw=1.7,label=r.replace('aerial','A').replace('ground','G')) for r,c in zip(robots,palette)]
handles.append(Line2D([],[],color='#bfc3c7',lw=2,label='Ground truth'))
fig.legend(handles=handles,ncol=7,loc='lower center',frameon=False,bbox_to_anchor=(.5,.045))
fig.suptitle('Same supported 12-robot subset in both runs',fontweight='bold')
fig.text(.5,.013,'Entire A03 and A08 trajectories excluded because their spatial graphs contain pending submaps. This is not an all-14 result.',ha='center',fontsize=8)
fig.tight_layout(rect=(0,.15,1,.96))
for ext in ('png','pdf'):fig.savefig(ROOT/f'supported-subset-trajectories.{ext}',dpi=300)
plt.close(fig)

scope=('These are fresh independent online replays; scheduling, odometry, geometry and retrieval opportunities can differ. '
       'Communication excludes DDS/IP/discovery, sensors, visualization and report downloads. '
       'The spatial result is incomplete, so costs are not comparisons at an equivalent final outcome.')
health=(f"All 14 native replays completed without divergence. Maximum speed: "
        f"{max(v['speed_m_s']['max'] for v in audit['native'].values()):.3f} m/s; longest successful-LiDAR-update gap: "
        f"{max(v['successful_update_gap_s']['max'] for v in audit['native'].values()):.3f} s. "
        'All 14 Rerun recordings passed verification. The independent audit checked sealed geometry membership, '
        'immutable payloads, causal availability and native calibration. All robot reference frames connected; '
        'three local submaps did not. No chronological fallback edges were inserted.')
rows=[]
for robot in audit['native']:
    raw=evaluation['raw_ate'][robot]['rmse_m']
    result=evaluation['spatial_individual_ate'].get(robot)
    corrected=f"{result['rmse_m']:.3f}" if result else 'Not evaluated: pending submaps'
    rows.append((robot,f'{raw:.3f}',corrected))
headers=('Robot','Raw individual ATE (m)','Spatial corrected individual ATE (m)')
table='<table><tr>'+''.join('<th>'+x+'</th>' for x in headers)+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(x)+'</td>' for x in row)+'</tr>' for row in rows)+'</table>'
section=('<section id="validated-comparison"><h2>What completed</h2><p>'+health+'</p><p>'+scope+'</p>'
    '<h2>CBS computation during the experiment</h2><img src="cbs-progress.png" style="width:100%">'
    '<p>810 measured updates across all robots. Solver time is separate from the full callback; terminal callbacks without timer rows are omitted. '
    '<a href="cbs-progress.pdf">Vector plot</a> · <a href="cbs-updates.csv">Measured updates</a>.</p>'
    '<h2>Matched supported-subset trajectories</h2><img src="supported-subset-trajectories.png" style="width:100%">'
    '<p>Both panels use one shared rigid evo alignment for the same 12 robots. The plotted view is horizontal; ATE uses all three position coordinates. '
    'Subset selection depends only on graph connectivity, not on error. '
    '<a href="supported-subset-trajectories.pdf">Vector figure</a>.</p><h2>Individual errors</h2>'+table+
    '<p>Individual ATEs each use a separate rigid fit. They do not measure inter-robot alignment.</p></section>')
page=ROOT/'index.html';text=page.read_text()
if 'id="validated-comparison"' in text:
    text=re.sub(r'<section id="validated-comparison">.*?</section>',lambda _:section,text,flags=re.S)
else:text=text.replace('<h2>Pending submaps</h2>',section+'<h2>Pending submaps</h2>')
page.write_text(text)
report=ROOT/'REPORT.md'
report.write_text(report.read_text().split('\n\n## Validation and comparison scope')[0])
with report.open('a') as stream:
    stream.write('\n\n## Validation and comparison scope\n\n'+health+'\n\n'+scope+'\n\n')
    stream.write('![Matched supported-subset trajectories](supported-subset-trajectories.png)\n\n')
    stream.write('![CBS measured update and callback time](cbs-progress.png)\n\n')
    stream.write('## Individual errors\n\n| '+' | '.join(headers)+' |\n| --- | ---: | ---: |\n')
    for row in rows:stream.write('| '+' | '.join(row)+' |\n')
    stream.write('\nIndividual errors each use a separate rigid fit. They do not measure inter-robot alignment.\n')

"""Measured A05 topology; coordinates are fixed footprint centers, not pose origins."""
import json
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

ROOT=Path(__file__).resolve().parent
d=json.loads((ROOT/'graph.json').read_text())
old=json.loads((ROOT.parent/'circle-mature-online-graco-all14-20261001/submap-graph-a05/graph-data.json').read_text())
assert old['sources']['keyframes']['sha256']==d['input_sha256']
cells=np.asarray(old['planner']['occupied_cells'])+.5
centers=np.asarray([r['footprint_center_xy'] for r in d['footprints']])
pairs=[(e['i'][1],e['j'][1]) for e in d['constraints']]
assert (27,28) not in pairs and (28,29) not in pairs and (18,28) in pairs
assert d['odometry_edges']==0 and d['component_count']==1
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':10,
    'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(1,2,figsize=(9.2,5.25),sharex=True,sharey=True)
for ax,new in zip(axes,(False,True)):
    ax.scatter(cells[:,0],cells[:,1],s=.25,c='#d7dfe2',edgecolors='none',rasterized=True,zorder=0)
    for xy in centers:
        ax.add_patch(Circle(xy,40,facecolor='none',edgecolor='#9cb3be',linewidth=.35,alpha=.40,zorder=1))
    connections=pairs if new else list(zip(range(len(centers)-1),range(1,len(centers))))
    ax.add_collection(LineCollection([centers[[a,b]] for a,b in connections],
        colors='#0072B2' if new else '#82919a',linewidths=1.25,alpha=.8,zorder=2))
    focus=(18,28) if new else (27,28)
    ax.plot(*centers[list(focus)].T,color='#009E73' if new else '#D55E00',lw=2.8,zorder=3)
    ax.scatter(*centers.T,s=22,color='#0072B2',edgecolors='white',linewidths=.55,zorder=4)
    for k,xy in enumerate(centers):
        ax.annotate(str(k),xy,xytext=(4,4),textcoords='offset points',fontsize=8 if k in(18,27,28) else 7,
            fontweight='bold' if k in (18,27,28) else 'normal',color='#162e3c',zorder=5)
    ax.set_aspect('equal');ax.set_xlabel('Horizontal map x (m)');ax.grid(alpha=.12)
axes[0].set_ylabel('Horizontal map y (m)')
axes[0].set_title('(a) Creation-order chain\n29 edges; includes unsupported 27-28',pad=12)
axes[1].set_title(f"(b) Shared-geometry graph\n{len(pairs)} edges; 14 cycles; one component",pad=12)
fig.legend(handles=[Line2D([0],[0],color='#D55E00',lw=2,label='Removed: 27-28'),
    Line2D([0],[0],color='#009E73',lw=2,label='Supported: 18-28 (5,821 shared IDs)'),
    Line2D([0],[0],marker='o',color='none',markerfacecolor='#0072B2',label='Fixed circle center')],
    loc='lower center',bbox_to_anchor=(.5,.085),ncol=3,frameon=False)
fig.text(.5,.042,'GRACO A05 | 30 fixed circles | R = 40 m | no chronological fallback | inter-robot loops omitted',ha='center')
fig.text(.5,.012,'Layout shows circle centers. Pose variables retain creation-anchor frames. Saved-prefix construction, not a new optimized result.',ha='center',fontsize=7,color='#56646e')
fig.subplots_adjust(left=.07,right=.98,top=.90,bottom=.23,wspace=.14)
(ROOT/'output/pdf').mkdir(parents=True,exist_ok=True)
fig.savefig(ROOT/'graph-comparison.png',dpi=300,bbox_inches='tight')
fig.savefig(ROOT/'output/pdf/graph-comparison.pdf',bbox_inches='tight')
counts=Counter(x['reason'] for x in d['diagnostics']['decisions'])
html=f'''<!doctype html><meta charset="utf-8"><title>A05 spatial-support submap graph</title>
<style>body{{font:17px/1.65 system-ui;max-width:1150px;margin:45px auto;padding:0 25px;color:#223943;background:#f7f9fa}}
h1{{font-size:30px}}img{{width:100%;background:white;border:1px solid #dde5e8;border-radius:10px}}
.numbers{{display:flex;gap:30px;padding:18px 24px;background:white;border-radius:10px}}b{{color:#0072B2}}
a{{color:#166182}}table{{border-collapse:collapse}}td,th{{padding:8px 24px 8px 0;text-align:left}}</style>
<h1>A05: spatial support replaces the creation-order chain</h1>
<p>Submap <b>28 connects only to 18</b>, which shares 5,821 native point IDs. The 224.7-m link from 27 to 28 is removed. All 30 nodes remain in one locally connected graph.</p>
<div class="numbers"><span><b>43</b> supported edges</span><span><b>0</b> odometry-chain edges</span><span><b>14</b> cycles</span><span><b>1</b> component</span></div>
<p><img src="graph-comparison.png" alt="A05 old chain and corrected spatial graph"></p>
<p>Nodes are drawn at fixed circle centers to expose neighborhood relationships. Actual variables retain creation-anchor IMU frames. This replays the saved online publication prefixes; it is not a new sensor replay or optimized accuracy result.</p>
<table><tr><th>Construction on 148</th><th>Measured value</th></tr>
<tr><td>All 30 prefix updates</td><td>{d['construction_wall_s']:.3f} s</td></tr>
<tr><td>Slowest prefix</td><td>{max(x['construction_s'] for x in d['prefixes'])*1000:.1f} ms</td></tr>
<tr><td>Fresh process peak resident memory</td><td>{d['fresh_process_peak_rss_kib']/1024:.1f} MiB</td></tr>
<tr><td>Peak cached geometry arrays</td><td>{d['diagnostics']['peak_cached_cloud_bytes']/1e6:.2f} MB</td></tr>
<tr><td>Candidate decisions</td><td>{counts['insufficient_shared_geometry']} insufficient support; {counts['degree_budget']} cycle-budget exclusions</td></tr>
<tr><td>Supported forest bridges exceeding normal degree budget</td><td>{sum(e['diagnostics']['degree_budget_exception'] for e in d['constraints'])}</td></tr></table>
<p><b>Disconnected regions:</b> they stay pending with separate component IDs until a supported local path reaches the reference submap. No fake links or priors are added. The present CBS interface does not independently optimize disconnected fragments; joint-map export refuses to label them globally aligned.</p>
<p><b>Evidence and cost:</b> these are correlated map-consistency regularizers, not independent registrations. Shared support geometry stays local. Spatial PCM exchanges direct separator-pair summaries, which can increase communication; total CBS time and network usage require a matched run.</p>
<p><a href="verification.json">Implementation verification</a> · <a href="graph.json">All graph edges, prefix diagnostics and hashes</a> · <a href="output/pdf/graph-comparison.pdf">Vector figure</a></p>'''
(ROOT/'index.html').write_text(html)

"""A05 measured topology in fixed footprint-center layout (not pose origins)."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

ROOT = Path(__file__).resolve().parent
d = json.loads((ROOT/'graph.json').read_text())
old_path = ROOT.parent/'circle-mature-online-graco-all14-20261001/submap-graph-a05/graph-data.json'
old = json.loads(old_path.read_text())
assert old['sources']['keyframes']['sha256'] == d['input_sha256']
cells = np.asarray(old['planner']['occupied_cells']) + .5
centers = np.asarray([r['footprint_center_xy'] for r in d['footprints']])
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':10,
    'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig, axes = plt.subplots(1, 2, figsize=(9.2, 5.1), sharex=True, sharey=True)
colors = {'odometry':'#71828c', 'map_consistency':'#c16b25'}
for ax, added in zip(axes, (False, True)):
    ax.scatter(cells[:,0],cells[:,1],s=.25,c='#d3dcdf',edgecolors='none',rasterized=True,zorder=0)
    for xy in centers:
        ax.add_patch(Circle(xy,40,facecolor='none',edgecolor='#9cb3be',linewidth=.35,alpha=.45,zorder=1))
    for kind in colors:
        if kind == 'map_consistency' and not added: continue
        segments = [centers[[e['i'][1],e['j'][1]]] for e in d['constraints'] if e['kind']==kind]
        ax.add_collection(LineCollection(segments,colors=colors[kind],linewidths=1.1 if kind=='odometry' else 1.6,
            linestyles='solid' if kind=='odometry' else 'dashed',alpha=.85,zorder=2))
    ax.scatter(centers[:,0],centers[:,1],s=23,color='#185574',edgecolors='white',linewidths=.6,zorder=3)
    for k,xy in enumerate(centers):
        ax.annotate(str(k),xy,xytext=(4,4),textcoords='offset points',fontsize=7,color='#163f52',zorder=4)
    ax.set_aspect('equal'); ax.set_xlabel('Horizontal map x (m)')
    ax.grid(alpha=.12)
axes[0].set_ylabel('Horizontal map y (m)')
axes[0].set_title('(a) Chronological backbone\n30 nodes · 29 edges · no local cycles',pad=12)
axes[1].set_title('(b) Sparse spatial graph\n29 backbone + 33 consistency edges · 33 cycles',pad=12)
fig.legend(handles=[Line2D([0],[0],color=colors['odometry'],label='Odometry backbone'),
    Line2D([0],[0],color=colors['map_consistency'],ls='--',label='Shared-geometry consistency'),
    Line2D([0],[0],marker='o',color='none',markerfacecolor='#185574',label='Fixed footprint center')],
    loc='lower center',bbox_to_anchor=(.5,.075),ncol=3,frameon=False)
fig.text(.5,.035,'A05 · R = 40 m · at most 3 spatial neighbors per node · inter-robot loops omitted',ha='center',fontsize=8)
fig.text(.5,.005,'Graph layout uses footprint centers; optimized variables retain their original creation-anchor IMU frames.',
    ha='center',fontsize=7,color='#586671')
fig.subplots_adjust(left=.07,right=.98,top=.9,bottom=.22,wspace=.14)
(ROOT/'output/pdf').mkdir(parents=True,exist_ok=True)
fig.savefig(ROOT/'graph-comparison.png',dpi=300,bbox_inches='tight')
fig.savefig(ROOT/'output/pdf/graph-comparison.pdf',bbox_inches='tight')
html = f'''<!doctype html><meta charset="utf-8"><title>A05 sparse spatial graph</title>
<style>body{{font:17px/1.65 system-ui;max-width:1150px;margin:45px auto;padding:0 25px;color:#223943;background:#f7f9fa}}
h1{{font-size:30px}} img{{width:100%;background:white;border:1px solid #dde5e8;border-radius:10px}}
.numbers{{display:flex;gap:30px;padding:18px 24px;background:white;border-radius:10px}} b{{color:#a65b21}}
a{{color:#166182}} table{{border-collapse:collapse}} td,th{{padding:8px 24px 8px 0;text-align:left}}</style>
<h1>A05: sparse spatial connections between circle submaps</h1>
<p>The original 29-edge backbone is retained. An additional <b>33 map-consistency factors</b> connect nonconsecutive overlapping crops supported by shared native point IDs.</p>
<div class="numbers"><span><b>30</b> submaps</span><span><b>33</b> new local cycles</span><span><b>3</b> maximum spatial degree</span><span><b>0</b> new peer geometry bytes</span></div>
<p><img src="graph-comparison.png" alt="A05 chronological versus sparse spatial graph"></p>
<p>Nodes are drawn at circle centers to show spatial relationships. The actual pose variables remain in creation-anchor IMU frames. These panels show topology, not a new optimized solution.</p>
<table><tr><th>Construction on 148</th><th>Measured value</th></tr>
<tr><td>All 30 publication-prefix updates</td><td>{d['construction_wall_s']:.3f} s</td></tr>
<tr><td>Slowest prefix update</td><td>{max(x['construction_s'] for x in d['prefixes'])*1000:.1f} ms</td></tr>
<tr><td>Peak resident memory, fresh inspection process</td><td>{d['fresh_process_peak_rss_kib']/1024:.1f} MiB</td></tr>
<tr><td>Shared-point support</td><td>4,660–46,303 IDs per accepted edge</td></tr>
<tr><td>Candidate exclusions</td><td>16 insufficient support; 11 degree limit</td></tr></table>
<p>Construction runs only on each robot's local sealed geometry. Overall communication and CBS update time still require matched measurements: new constraints can change beliefs and solver work.</p>
<p><b>Evidence caveat.</b> Shared crops are correlated. These conservatively weighted constraints preserve map consistency; they are not independent loop detections and do not independently correct raw odometry drift.</p>
<p><b>Implementation verification:</b> 42 native Python/ROS tests and 17 C++ tests passed. The growing three-robot GPU test retained 21 local consistency factors and nine inter-robot registration pairs, reached a shared frame, and passed final geometry validation. See <a href="verification.json">verification and binary hashes</a>. A new all-14 dataset accuracy run has not been performed.</p>
<p><a href="output/pdf/graph-comparison.pdf">Vector figure</a> · <a href="graph.json">Measured graph, input/source hashes and all decisions</a></p>'''
(ROOT/'index.html').write_text(html)

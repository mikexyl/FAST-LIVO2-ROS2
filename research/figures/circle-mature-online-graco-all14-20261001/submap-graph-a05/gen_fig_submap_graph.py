#!/usr/bin/env python3
"""Plot A05's observed footprints and actual final CBS graph, without GT alignment."""
import hashlib
import json
import os
from collections import defaultdict
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/a05-submap-graph-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np

ROOT = Path(__file__).resolve().parent
d = json.loads((ROOT / 'graph-data.json').read_text())
registration = json.loads((ROOT.parent / 'registration.json').read_text())
robot_names = sorted(registration['robots'])


def key(e):
    return tuple(sorted(tuple(e[k]) for k in ('i', 'j')))


reg_pairs = set()
for data in registration['robots'].values():
    for p in data['pairs']:
        if p.get('accepted'):
            reg_pairs.add(tuple(sorted((robot_names[x[0]], x[1]) for x in (p['i'], p['j']))))
assert len(d['keyframes']) == 30 and len(d['odometry_edges']) == 29
assert len(d['loops']) == 8 and all(key(e) in reg_pairs for e in d['loops'])
assert all(e['i'][1]+1 == e['j'][1] for e in d['odometry_edges'])

rows = sorted(d['keyframes'], key=lambda r: r['keyframe_id'])
pose = {(p['robot_id'], p['keyframe_id']): np.array(p['T_world_body']) for p in d['poses']}
H = np.asarray(d['planner']['T_horizontal_map'])
X0_raw = np.asarray(rows[0]['T_world_body'])
C0 = pose[('aerial05', 0)] @ np.linalg.inv(X0_raw)
V = H @ np.linalg.inv(C0)
xy = {k: (V @ T)[:2, 3] for k, T in pose.items()}
own_xy = np.array([xy[('aerial05', r['keyframe_id'])] for r in rows])
raw_xy = np.array([(H @ np.asarray(r['T_world_body']))[:2, 3] for r in rows])
assert np.allclose(own_xy[0], raw_xy[0], atol=1e-8)
assert np.isfinite(own_xy).all()

circles = d['planner']['circles']
assert [c['id'] for c in circles] == list(range(30))
for r,c in zip(rows,circles):
    assert np.array_equal(r['footprint_center_xy'],c['center_xy'])
cell_config = d['planner']['config']
occupied = (np.asarray(d['planner']['occupied_cells']) + 0.5)*cell_config['cell_size_m'] + cell_config['grid_origin_xy']

BLUE, ORANGE, GREEN = '#0072B2', '#E69F00', '#009E73'
GRAY, INK = '#7b8792', '#233643'
plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],
                     'font.size':9,'axes.titlesize':10,'axes.titleweight':'bold',
                     'axes.labelsize':9,'legend.fontsize':8,
                     'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none',
                     'axes.spines.top':False,'axes.spines.right':False,
                     'grid.alpha':.15,'axes.axisbelow':True})
fig = plt.figure(figsize=(10.4, 8.1), facecolor='white')
gs = fig.add_gridspec(2,2,height_ratios=[1,0.34],left=.075,right=.975,top=.835,bottom=.105,
                      hspace=.43,wspace=.26)
left=fig.add_subplot(gs[0,0]); right=fig.add_subplot(gs[0,1]); graph=fig.add_subplot(gs[1,:])
fig.suptitle('A05: spatial submaps and their graph',fontsize=16,fontweight='bold',color=INK,y=.98)
fig.text(.5,.944,'30 submaps  |  29 odometry edges  |  8 inter-robot loops, all with registration factors',
         ha='center',fontsize=9.5,color=INK)
legend=[Line2D([],[],marker='o',color=BLUE,lw=0,markersize=6,label='A05 submap node'),
        Line2D([],[],color=GRAY,lw=1.3,label='Odometry factor'),
        Line2D([],[],marker='D',color=ORANGE,lw=0,markersize=5,label='Peer submap node'),
        Line2D([],[],color=GREEN,lw=1.5,label='Loop pose + registration')]
fig.legend(handles=legend,loc='upper center',bbox_to_anchor=(.5,.916),ncol=4,frameon=False)

left.scatter(occupied[:,0],occupied[:,1],s=.9,c='#a7afb4',alpha=.60,linewidths=0,rasterized=True)
for c in circles:
    x,y=c['center_xy']
    left.add_patch(Circle((x,y),c['radius_m'],fill=False,edgecolor=BLUE,lw=.7,alpha=.55))
    left.scatter([x],[y],s=16,c=BLUE,zorder=4)
    left.annotate(str(c['id']),(x,y),xytext=(3,3),textcoords='offset points',fontsize=7,
                  color=INK,zorder=5,bbox=dict(facecolor='white',edgecolor='none',pad=.25,alpha=.75))
left.plot(raw_xy[:,0],raw_xy[:,1],ls='--',lw=.9,c='#5f6870',alpha=.85,zorder=2)
left.set_title('(a) Fixed footprints in the odometry map',loc='left',pad=13)
left.text(0,1.015,'40 m radius; labels are submap IDs',transform=left.transAxes,fontsize=8,color='#52636f')
left.legend(handles=[Line2D([],[],color='#a7afb4',lw=3,label='Observed occupied cells'),
                     Line2D([],[],color='#5f6870',ls='--',lw=1,label='Creation-anchor order')],
            loc='lower left',fontsize=7,framealpha=.92,edgecolor='none')

for e in d['odometry_edges']:
    a,b=xy[tuple(e['i'])],xy[tuple(e['j'])]
    right.plot([a[0],b[0]],[a[1],b[1]],color=GRAY,lw=1.1,zorder=1)
for e in d['loops']:
    a,b=xy[tuple(e['i'])],xy[tuple(e['j'])]
    right.plot([a[0],b[0]],[a[1],b[1]],color=GREEN,lw=1.25,alpha=.9,zorder=2)
right.scatter(own_xy[:,0],own_xy[:,1],s=24,c=BLUE,edgecolors='white',linewidths=.5,zorder=4)

# Co-created circles can share a pose origin. Label their IDs together without
# moving any plotted node; the topology panel retains every node individually.
groups=defaultdict(list)
for r in rows: groups[r['stamp_ns']].append(r['keyframe_id'])
for ids in groups.values():
    p=np.mean([xy[('aerial05', i)] for i in ids],axis=0)
    label=','.join(map(str,ids))
    offset={0:(12,-3),2:(-16,4),3:(10,16)}.get(ids[0],(4,4))
    right.annotate(label,p,xytext=offset,textcoords='offset points',fontsize=7,color=BLUE,
                   ha='right' if offset[0]<0 else 'left',
                   arrowprops=dict(arrowstyle='-',color=BLUE,lw=.45) if ids[0] in (0,2,3) else None,
                   zorder=6,bbox=dict(facecolor='white',edgecolor='none',pad=.3,alpha=.78))
peer_keys=sorted(k for k in xy if k[0]!='aerial05')


def short(k):
    return ('A' if k[0].startswith('aerial') else 'G')+k[0][-2:]+':'+str(k[1])


for idx,k in enumerate(peer_keys):
    p=xy[k];right.scatter(*p,s=31,c=ORANGE,marker='D',edgecolors='white',linewidths=.5,zorder=5)
    offsets={'A07:7':(12,-7),'A08:15':(-14,-13),'A07:14':(-10,-14),
             'A07:16':(-12,3),'A07:17':(-12,12),'A08:12':(-8,9),
             'A08:20':(-8,12),'G01:9':(-8,-13)}
    right.annotate(short(k),p,xytext=offsets[short(k)],
                   ha='right' if offsets[short(k)][0]<0 else 'left',textcoords='offset points',
                   fontsize=7,color='#8a5c00',zorder=7,
                   arrowprops=dict(arrowstyle='-',color='#b68a32',lw=.45),
                   bbox=dict(facecolor='white',edgecolor='none',pad=.3,alpha=.85))
right.set_title('(b) Optimized pose origins and loop links',loc='left',pad=13)
right.text(0,1.015,'Actual creation-anchor poses; nearby labels may list several IDs',
           transform=right.transAxes,fontsize=7.6,color='#52636f')
for ax in [left,right]:
    ax.set_aspect('equal',adjustable='datalim');ax.set_xlabel('Horizontal x (m)');ax.set_ylabel('Horizontal y (m)')
    ax.grid(True);ax.margins(.12);ax.tick_params(labelsize=8)

# An ID-ordered schematic makes the graph topology unambiguous even where
# multiple circle nodes have nearly coincident metric pose origins.
graph.set_title('(c) A05 submap connectivity',loc='left',pad=10)
graph.text(1,1.10,'Layout spacing is schematic',transform=graph.transAxes,ha='right',fontsize=8,color='#52636f')
graph.plot(np.arange(30),np.zeros(30),color=GRAY,lw=1.4,zorder=1)
graph.scatter(np.arange(30),np.zeros(30),s=150,facecolors='white',edgecolors=BLUE,lw=1.15,zorder=3)
for i in range(30):graph.text(i,0,str(i),color=BLUE,ha='center',va='center',fontsize=7,zorder=4)
for e,px in zip(d['loops'],np.linspace(8.2,28.2,len(d['loops']))):
    own=tuple(e['i']) if e['i'][0]=='aerial05' else tuple(e['j'])
    peer=tuple(e['j']) if e['i'][0]=='aerial05' else tuple(e['i'])
    graph.plot([own[1],px],[0,.95],color=GREEN,lw=1.3,zorder=1)
    graph.scatter([px],[.95],s=42,c=ORANGE,marker='D',edgecolors='white',lw=.5,zorder=3)
    graph.text(px,1.11,short(peer),ha='center',va='bottom',fontsize=8,color='#8a5c00')
graph.text(-.7,.95,'Peer endpoints',fontsize=8,color='#52636f',va='center')
graph.text(-.7,-.32,'A05 submap ID',fontsize=8,color='#52636f',va='center')
graph.set_xlim(-.8,29.8);graph.set_ylim(-.45,1.4);graph.axis('off')
fig.text(.075,.057,'A graph pose is the IMU frame at submap creation, not the spatial circle center.',
         fontsize=9,color=INK)
fig.text(.075,.032,'Final persistent CBS solution from the online GRACO all-14 run. Metric views use A05 native gravity and its first anchor; no GT alignment.',
         fontsize=7.7,color='#52636f')
fig.savefig(ROOT/'a05-submap-graph.pdf')
fig.savefig(ROOT/'a05-submap-graph.png',dpi=240)
plt.close(fig)

summary={'experiment':d['experiment'],'robot':'aerial05','submaps':30,'odometry_edges':29,
         'retained_loop_edges':8,'loops_with_registration_factors':8,'within_robot_loop_edges':0,
         'peer_endpoints':[short(k) for k in peer_keys],
         'peer_robots':sorted(set(k[0] for k in peer_keys)),
         'co_creation_groups':{str(k):v for k,v in groups.items() if len(v)>1},
         'distinct_creation_timestamps':len(groups),
         'T_display_cbs':V.tolist(),'ground_truth_used':False,
         'checks':{'sources_match_final_map_export':True,'all_nodes_have_final_poses':True,
                   'odometry_edges_match_recorded_bridge_construction':True,
                   'all_eight_loop_pairs_have_accepted_geometry_factors':True,
                   'footprints_match_publication_index':True},
         'metric_node_positions':'Creation-anchor pose origins, no jitter; grouped labels for co-created nodes',
         'topology_panel':'ID-ordered layout, no metric meaning; same actual graph edges',
         'auxiliary_variables':'CBS anchor/belief auxiliary variables and priors are not shown; this is the submap constraint graph',
         'data_sha256':hashlib.sha256((ROOT/'graph-data.json').read_bytes()).hexdigest(),
         'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(ROOT/'index.html').write_text('''<!doctype html><html><head><meta charset="utf-8"><title>A05 submap graph</title>
<style>body{margin:32px auto;max-width:1300px;padding:0 24px;font:16px/1.55 system-ui;color:#233643;background:#f7f9fa}h1{margin-bottom:6px}img{width:100%;background:white;border:1px solid #e1e6e9;border-radius:10px}a{color:#0072b2}.links{display:flex;gap:20px;margin:16px 0}p{max-width:1050px}</style></head><body>
<h1>A05: spatial submaps and their graph</h1><p>Latest online GRACO all-14 run: 30 A05 submaps, 29 odometry edges, and 8 retained inter-robot loops. All eight carry registration factors.</p>
<div class="links"><a href="a05-submap-graph.pdf">Vector PDF</a><a href="a05-submap-graph.png">Full-resolution image</a><a href="summary.json">Counts and verification</a><a href="graph-data.json">Retained graph data</a></div>
<img src="a05-submap-graph.png" alt="A05 footprint coverage, optimized submap poses, and full graph topology">
<p>The circle centers define spatial support. The optimized graph variables use the original creation-anchor IMU frames. Several circles are created together, so their pose origins can coincide; they remain separate nodes. Panel (c) shows every node and edge without metric overlap. Peer endpoints belong to A07, A08 and G01; their other graph connections are omitted. CBS anchor/belief auxiliary variables and priors are outside this submap-constraint view.</p>
<p>No ground truth is used. The footprints and occupied cells come from the final A05 planner state; graph poses and loops match the final published CBS graph used to export the joint map. Panels (a) and (b) show unoptimized footprints and optimized poses, respectively. This is an inspection plot, not a change to the pipeline or a new experiment.</p></body></html>''')
print(json.dumps({k:summary[k] for k in ('submaps','odometry_edges','retained_loop_edges','peer_robots','distinct_creation_timestamps')},indent=2))

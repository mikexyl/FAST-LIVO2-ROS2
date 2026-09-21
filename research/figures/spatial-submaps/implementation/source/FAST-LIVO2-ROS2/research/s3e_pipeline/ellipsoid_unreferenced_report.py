"""Visualize graph components when no timestamped trajectory GT is available."""
from pathlib import Path
import json
import numpy as np
from .ellipsoid_full_report import ROBOTS,COLORS,read,rows


def component_robots(report,branch):
    assignment=report['branches'][branch]['components']
    return {c:[r for r in ROBOTS if assignment[r]==c] for c in sorted(set(assignment.values()))}


def figures(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output=Path(output);report=read(output/'report.json');tracks={}
    groups={b:component_robots(report,b) for b in ('raw','ellipsoid')}
    for branch in groups:
        poses=rows(output/branch/'poses.jsonl')
        for robot in ROBOTS:
            tracks[f'{branch}_{robot}']=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3]
                for r in poses if r['robot_id']==robot])
    def panels():
        fig,axes=plt.subplots(max(map(len,groups.values())),2,figsize=(12,4.5*max(map(len,groups.values()))),
                              squeeze=False,layout='constrained')
        for col,branch in enumerate(('raw','ellipsoid')):
            for row in range(len(axes)):
                ax=axes[row,col]
                if row>=len(groups[branch]):ax.set_visible(False);continue
                component=list(groups[branch])[row];robots=groups[branch][component]
                ax.set_title(f'{branch.capitalize()} BEVs · component '+ ' + '.join(robots))
                ax.set_xlabel('X in component frame [m]');ax.set_ylabel('Y in component frame [m]')
                ax.set_aspect('equal');ax.grid(alpha=.2)
        return fig,axes
    sequence=Path(report['dataset']).name.removeprefix('S3E_').replace('_',' ')
    fig,axes=panels()
    for col,branch in enumerate(('raw','ellipsoid')):
        for row,(component,robots) in enumerate(groups[branch].items()):
            ax=axes[row,col]
            for robot in robots:
                p=tracks[f'{branch}_{robot}'];ax.plot(p[:,0],p[:,1],color=COLORS[robot],lw=1.2,label=robot)
            ax.legend()
    fig.suptitle(sequence+' · optimized trajectories\nNo trajectory GT alignment; disconnected components are shown separately')
    fig.savefig(output/'trajectories.png',dpi=180);fig.savefig(output/'trajectories.pdf');plt.close(fig)
    np.savez_compressed(output/'trajectory-view.npz',origin=np.zeros(3),**tracks)
    (output/'visualization.json').write_text(json.dumps(dict(gt_available=False,components=groups,
        coordinates='Independent PGO component frames; no alignment between components or to GT',report=report),indent=2)+'\n')
    if (output/'maps.npz').exists():
        with np.load(output/'maps.npz') as f:maps={k:f[k] for k in f.files}
        fig,axes=panels()
        for col,branch in enumerate(('raw','ellipsoid')):
            for row,(component,robots) in enumerate(groups[branch].items()):
                ax=axes[row,col]
                center_z=float(np.median(np.concatenate([tracks[f'{branch}_{r}'][:,2] for r in robots])))
                for robot in robots:
                    p=maps[f'{branch}_{robot}'];p=p[np.abs(p[:,2]-center_z)<=.5]
                    ax.scatter(p[:,0],p[:,1],c=COLORS[robot],s=.3,alpha=.6,rasterized=True,label=robot)
                ax.legend(markerscale=12)
        fig.suptitle(sequence+' · corrected maps from identical raw keyframe scans\n1 m slice around each component’s median trajectory height; no GT alignment')
        fig.savefig(output/'maps.png',dpi=180);fig.savefig(output/'maps.pdf');plt.close(fig)
    if (output/'bev-examples.npz').exists():
        selection=read(output/'bev-examples.json');fig,axes=plt.subplots(2,3,figsize=(13,8),layout='constrained')
        with np.load(output/'bev-examples.npz') as f:
            for row,branch in enumerate(('raw','ellipsoid')):
                for col,robot in enumerate(ROBOTS):
                    prefix=f'{branch}_{robot}_';im=f[prefix+'image'];xy=f[prefix+'orb_xy'];keep=f[prefix+'kept_indices'].astype(int)
                    ax=axes[row,col];ax.imshow(im,cmap='gray',vmin=0,vmax=255)
                    if len(keep):ax.scatter(xy[keep,0],xy[keep,1],s=10,facecolors='none',edgecolors='#55dc80',linewidths=.5)
                    ax.set_title(f'{robot} · {branch} · {len(keep)} retained ORB');ax.axis('off')
        fig.suptitle(sequence+' · density BEVs with retained ORB features\nMiddle keyframe of each robot; same 0.5 m pixels, views shown at individual extents')
        fig.savefig(output/'bevs.png',dpi=180);fig.savefig(output/'bevs.pdf');plt.close(fig)


def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    output=Path(output);report=read(output/'report.json')
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    with np.load(output/'trajectory-view.npz') as f:tracks={k:f[k] for k in f.files}
    maps={}
    if (output/'maps.npz').exists():
        with np.load(output/'maps.npz') as f:maps={k:f[k] for k in f.files}
    groups={b:component_robots(report,b) for b in ('raw','ellipsoid')}
    columns=[]
    for branch in groups:
        columns.append(rrb.Vertical(*[rrb.Spatial3DView(name=f'{branch} · '+ ' + '.join(robots),origin=f'/{branch}/{component}')
                                     for component,robots in groups[branch].items()]))
    rr.init(Path(report['dataset']).name+'_ellipsoid_BEV_components');rr.save(str(output/'trajectories.rrd'))
    views=rrb.Horizontal(*columns)
    if (output/'bev-examples.npz').exists():
        bevs=rrb.Vertical(*[rrb.Horizontal(*[rrb.Spatial2DView(name=f'{branch} {robot}',origin=f'/bevs/{branch}/{robot}')
                                             for robot in ROBOTS]) for branch in groups])
        views=rrb.Tabs(views,bevs)
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(views,
        rrb.TextDocumentView(name='Results and limitations',origin='/report'),row_shares=[.8,.2]),collapse_panels=True))
    for branch in groups:
        for component,robots in groups[branch].items():
            base=f'/{branch}/{component}';rr.log(base,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
            for robot in robots:
                color=[int(COLORS[robot][i:i+2],16) for i in (1,3,5)]
                rr.log(base+'/'+robot+'/trajectory',rr.LineStrips3D([tracks[f'{branch}_{robot}']],colors=color,radii=.035),static=True)
                if f'{branch}_{robot}' in maps:
                    rr.log(base+'/'+robot+'/map',rr.Points3D(maps[f'{branch}_{robot}'],colors=[*color,130],radii=.025),static=True)
    if (output/'bev-examples.npz').exists():
        with np.load(output/'bev-examples.npz') as f:
            for branch in groups:
                for robot in ROBOTS:
                    base=f'/bevs/{branch}/{robot}';prefix=f'{branch}_{robot}_'
                    rr.log(base,rr.Image(f[prefix+'image']),static=True)
                    xy=f[prefix+'orb_xy'];keep=f[prefix+'kept_indices'].astype(int)
                    if len(keep):rr.log(base+'/features',rr.Points2D(xy[keep],colors=[80,240,120],radii=1.2),static=True)
    rr.log('/report',rr.TextDocument((output/'REPORT.md').read_text(),media_type=rr.MediaType.MARKDOWN),static=True)
    rr.get_data_recording().flush()

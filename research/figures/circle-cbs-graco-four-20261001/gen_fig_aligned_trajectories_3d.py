"""Render aligned dense trajectories, preserving one shared component transform."""
from pathlib import Path
import hashlib
import json
import os

import numpy as np
import plotly.graph_objects as go
from plotly.offline import get_plotlyjs

ROOT = Path(__file__).resolve().parent
COLORS = {'aerial05':'#cb5c57','aerial07':'#dc9f32','ground05':'#378ca4','ground06':'#6554a4'}
NAMES = {'aerial05':'A05 · aerial','aerial07':'A07 · aerial','ground05':'G05 · ground','ground06':'G06 · ground'}
MODES = {'circle40':'Fixed circles · 40 m','trajectory40':'Trajectory-centered · 40 m',
         'trajectory80':'Trajectory-centered · 80 m'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    entries=json.loads((ROOT/'comparison.json').read_text());data={};sources={}
    for mode in MODES:
        entry=entries[mode];data[mode]={};sources[mode]={}
        assert len(set(entry['evaluation']['components'].values())) == 1
        component=next(iter(entry['evaluation']['components'].values()))
        A=np.asarray(entry['evaluation']['joint_ate'][component]['alignment_SE3'])
        assert np.allclose(A[:3,:3].T@A[:3,:3],np.eye(3),atol=1e-8)
        assert np.isclose(np.linalg.det(A[:3,:3]),1.)
        for robot in COLORS:
            path=ROOT/mode/robot/'corrected.tum';track=np.loadtxt(path)
            assert np.isfinite(track).all() and np.all(np.diff(track[:,0])>0)
            xyz=track[:,1:4]@A[:3,:3].T+A[:3,3]
            data[mode][robot]=(xyz,track[:,0]-track[0,0])
            sources[mode][robot]={'path':str(path.relative_to(ROOT)),'sha256':sha(path),
                'samples':len(track),'alignment_SE3':A.tolist()}
    all_xyz=np.concatenate([xyz for robots in data.values() for xyz,_ in robots.values()])
    lo=all_xyz.min(0);hi=all_xyz.max(0);span=hi-lo
    lo-=np.maximum(span*.045,[1.,1.,2.]);hi+=np.maximum(span*.045,[1.,1.,2.]);span=hi-lo
    ratio=span/max(span)
    fig=go.Figure()
    for mode,robots in data.items():
        for robot,(xyz,elapsed) in robots.items():
            group=mode+'-'+robot;visible=mode=='circle40'
            fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],mode='lines',
                name=NAMES[robot],legendgroup=group,visible=visible,
                line=dict(color=COLORS[robot],width=6,dash='solid' if robot.startswith('aerial') else 'dash'),
                customdata=elapsed[:,None],
                hovertemplate=NAMES[robot]+'<br>Time %{customdata[0]:.1f} s<br>East %{x:.2f} m<br>North %{y:.2f} m<br>z %{z:.2f} m<extra></extra>'))
            for index,symbol,label in [(0,'circle-open','Start'),(-1,'square','End')]:
                p=xyz[index]
                fig.add_trace(go.Scatter3d(x=[p[0]],y=[p[1]],z=[p[2]],mode='markers',
                    name=NAMES[robot]+' · '+label,legendgroup=group,showlegend=False,visible=visible,
                    marker=dict(size=5,color=COLORS[robot],symbol=symbol),
                    hovertemplate=NAMES[robot]+' · '+label+'<extra></extra>'))
    eye=dict(x=1.5,y=-1.65,z=.95)
    ranges=[dict(range=[float(lo[k]),float(hi[k])],backgroundcolor='#f7f9fc',gridcolor='#dfe5ed',
                 zerolinecolor='#dfe5ed',showbackground=True) for k in range(3)]
    ratio_dict=dict(x=float(ratio[0]),y=float(ratio[1]),z=float(ratio[2]))
    modes=list(MODES)
    fig.update_layout(template='plotly_white',margin=dict(l=5,r=5,t=135,b=15),
        font=dict(family='Arial, sans-serif',size=13,color='#253443'),
        title=dict(text='CBS · four aligned robots · '+MODES['circle40'],x=.02,y=.99),
        legend=dict(orientation='h',x=.5,xanchor='center',y=1.01,yanchor='bottom',groupclick='togglegroup'),
        scene=dict(xaxis=dict(title='East [m]',**ranges[0]),yaxis=dict(title='North [m]',**ranges[1]),
            zaxis=dict(title='Aligned z [m]',**ranges[2]),aspectmode='manual',aspectratio=ratio_dict,
            camera=dict(eye=eye,up=dict(x=0,y=0,z=1)),dragmode='orbit'),
        uirevision='aligned-graco',updatemenus=[
            dict(type='dropdown',x=.01,y=1.16,xanchor='left',yanchor='top',buttons=[
                dict(label=MODES[m],method='update',args=[dict(visible=[i//12==j for i in range(36)]),
                    dict(title=dict(text='CBS · four aligned robots · '+MODES[m],x=.02,y=.99))]) for j,m in enumerate(modes)]),
            dict(type='buttons',direction='right',x=.39,y=1.16,xanchor='left',buttons=[
                dict(label='True scale',method='relayout',args=[{'scene.aspectratio':ratio_dict}]),
                dict(label='Height ×3',method='relayout',args=[{'scene.aspectratio':dict(ratio_dict,z=3*ratio_dict['z'])}])]),
            dict(type='buttons',direction='right',x=.7,y=1.16,xanchor='left',buttons=[
                dict(label='Oblique',method='relayout',args=[{'scene.camera':dict(eye=eye,up=dict(x=0,y=0,z=1))}]),
                dict(label='Top',method='relayout',args=[{'scene.camera':dict(eye=dict(x=0,y=0,z=2.5),up=dict(x=0,y=1,z=0))}]),
                dict(label='Side',method='relayout',args=[{'scene.camera':dict(eye=dict(x=0,y=-2.5,z=.03),up=dict(x=0,y=0,z=1))}])])])
    plot=fig.to_html(full_html=False,include_plotlyjs=False,div_id='trajectories-3d',
        config=dict(displaylogo=False,responsive=True,toImageButtonOptions=dict(format='png',filename='graco-aligned-trajectories-3d',scale=2)))
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>GRACO aligned trajectories in 3D</title><style>body{margin:0;background:#fff;color:#253443;font:15px system-ui}header{padding:20px 28px 6px}h1{font-size:23px;margin:0 0 8px}p{margin:6px 0;line-height:1.5}a{color:#286a9c}.plotly-graph-div{height:calc(100vh - 155px)!important;min-height:580px}small{color:#536476}</style>
<header><h1>GRACO · CBS submap graph trajectories in 3D</h1><p>Drag to rotate · scroll to zoom · click a robot in the legend to hide it.</p>
<small>One shared rigid alignment for all four robots. All trajectory samples retained. True metric scale by default; “Height ×3” exaggerates vertical display only. Open circles mark starts; squares mark ends.</small>
<p><a href="index.html">Comparison gallery</a> · <a href="aligned-trajectories-3d.png">Static view</a> · <a href="aligned-trajectories-3d.pdf">Vector PDF</a></p></header>'''
    (ROOT/'aligned-trajectories-3d.html').write_text(page+'<script>'+get_plotlyjs()+'</script>'+plot+'</html>')

    os.environ.setdefault('MPLCONFIGDIR','/tmp/circle-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    # This machine has an older system mpl_toolkits namespace. Select the 3D
    # toolkit shipped beside the active matplotlib; do not change installation.
    import mpl_toolkits
    matching=Path(matplotlib.__file__).parent.parent/'mpl_toolkits'
    if (matching/'mplot3d').is_dir():mpl_toolkits.__path__=[str(matching)]
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],'font.size':10,
        'pdf.fonttype':42,'legend.frameon':False,'savefig.dpi':300})
    fig=plt.figure(figsize=(10.5,6.4));ax=fig.add_subplot(111,projection='3d')
    for robot,(xyz,_) in data['circle40'].items():
        ax.plot(*xyz.T,color=COLORS[robot],lw=1.8,ls='-' if robot.startswith('aerial') else '--',label=NAMES[robot])
        ax.scatter(*xyz[0],s=24,marker='o',facecolors='white',edgecolors=COLORS[robot],depthshade=False)
        ax.scatter(*xyz[-1],s=22,marker='s',color=COLORS[robot],depthshade=False)
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),
        xlabel='East [m]',ylabel='North [m]',zlabel='Aligned z [m]')
    ax.set_box_aspect(span,zoom=1.35);ax.view_init(elev=25,azim=-64)
    # The oblique 3D box extends beyond the rectangular 2D axes patch. Every
    # sample is within the explicit metric limits, so retain projected lines
    # and endpoint markers outside that patch instead of visually cutting them.
    for artist in [*ax.lines,*ax.collections]:artist.set_clip_on(False)
    ax.xaxis.pane.set_facecolor('#f7f9fc');ax.yaxis.pane.set_facecolor('#f7f9fc');ax.zaxis.pane.set_facecolor('#f7f9fc')
    ax.set_zticks([0,20,40]);ax.tick_params(labelsize=8)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,1.01),ncol=4)
    fig.suptitle('CBS submap graph · fixed circles, 40 m',y=.97,fontsize=14)
    fig.text(.5,.07,'True metric scale · one shared rigid alignment · open circles: start, squares: end',ha='center',fontsize=9,color='#536476')
    fig.subplots_adjust(left=.01,right=.97,top=.9,bottom=.13)
    for ext in ('png','pdf'):fig.savefig(ROOT/f'aligned-trajectories-3d.{ext}')
    plt.close(fig)
    (ROOT/'3d-view-provenance.json').write_text(json.dumps(dict(
        source_sha256=sha(__file__),comparison_sha256=sha(ROOT/'comparison.json'),
        sources=sources,default_mode='circle40',default_height_exaggeration=1,
        placement='one joint evaluation SE(3) transform per connected component, shared by every robot',
        no_smoothing_or_subsampling=True),indent=2)+'\n')
    print('Saved standalone interactive HTML, PNG, PDF and trajectory provenance')


if __name__=='__main__':main()

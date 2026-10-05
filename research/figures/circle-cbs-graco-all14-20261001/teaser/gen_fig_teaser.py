#!/usr/bin/env python3
"""Reproducible orthographic teaser from the validated map and CBS trajectories.

Run with the project research Python environment. No new alignment, smoothing,
or ground-truth transform is applied. PDF map is rasterized; lines/text are vector.
"""
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/graco-circle-teaser-mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.patheffects as pe
import numpy as np

HERE = Path(__file__).resolve().parent
REPORT = HERE.parent
DTYPE = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),
                  ('green','u1'),('blue','u1'),('robot_id','u1'),
                  ('submap_id','<u4'),('uncovered','u1')])


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(2**20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    metadata = json.loads((REPORT/'export.json').read_text())
    assert len(metadata['files']) == 1
    info = metadata['files'][0]
    assert len(info['robots']) == 14
    ply = REPORT/'exports'/Path(info['path']).name
    assert sha256(ply) == info['sha256']
    with ply.open('rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError('Missing PLY header')
            header.append(line.decode('ascii').strip())
            if line.strip() == b'end_header':
                break
        offset = f.tell()
    assert 'format binary_little_endian 1.0' in header
    assert f"element vertex {info['points']}" in header
    assert ply.stat().st_size == offset+info['points']*DTYPE.itemsize
    data = np.memmap(ply, mode='r', dtype=DTYPE, offset=offset, shape=(info['points'],))
    # Identical deterministic display subset and height draw order to report top view.
    subset = data[np.linspace(0,len(data)-1,min(len(data),350000),dtype=np.int64)]
    subset = subset[np.argsort(subset['z'])]
    rgb = np.stack([subset[k] for k in ('red','green','blue')],1)/255.
    bounds = np.array([[data['x'].min(),data['y'].min()], [data['x'].max(),data['y'].max()]])
    level = np.asarray(info['T_level_cbs'])
    tracks = {}; hashes = {}; gaps = {}
    for robot in info['robots']:
        path = HERE/'inputs'/f'{robot}-corrected.tum'
        a = np.loadtxt(path)
        assert a.shape[1] == 8 and len(a)>1 and np.isfinite(a).all()
        assert np.all(np.diff(a[:,0])>0)
        xyz = a[:,1:4]@level[:3,:3].T + level[:3,3]
        # Preserve all recorded positions; never bridge a missing-data interval.
        breaks = np.flatnonzero(np.diff(a[:,0])>1.0)+1
        tracks[robot] = np.split(xyz, breaks)
        gaps[robot] = int(len(breaks)); hashes[robot] = sha256(path)
        bounds[0] = np.minimum(bounds[0],xyz[:,:2].min(0))
        bounds[1] = np.maximum(bounds[1],xyz[:,:2].max(0))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.5,
        'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none',
        'path.simplify':False,'savefig.facecolor':'white'})
    limits = bounds + np.array([[-10.,-10.],[10.,10.]])
    width = 3.5
    height = width*np.diff(limits[:,1])[0]/np.diff(limits[:,0])[0]
    fig = plt.figure(figsize=(width,height),facecolor='white')
    ax = fig.add_axes([.008,.008,.984,.984])
    ax.set_xlim(limits[:,0]); ax.set_ylim(limits[:,1])
    ax.set_aspect('equal'); ax.set_axis_off()
    ax.scatter(subset['x'],subset['y'],c=rgb,s=.032,linewidths=0,
               rasterized=True,zorder=1)
    styles = {'aerial':dict(linestyle=(0,(3.0,1.5))),
              'ground':dict(linestyle='-')}
    for robot,segments in tracks.items():
        style = styles['aerial' if robot.startswith('aerial') else 'ground']
        color = np.asarray(metadata['colors_rgb'][robot],dtype=float)/255.
        for xyz in segments:
            ax.plot(xyz[:,0],xyz[:,1],**style,color=color,linewidth=.48,alpha=1.,
                solid_capstyle='round',dash_capstyle='round',zorder=3,
                path_effects=[pe.Stroke(linewidth=.95,foreground='white',alpha=.85),pe.Normal()])
    handles = [Line2D([0],[0],**styles[k],color='#29343a',linewidth=.85,label=k.title()) for k in styles]
    ax.legend(handles=handles,loc='upper right',bbox_to_anchor=(.988,.99),
        frameon=False,handlelength=2.6,handletextpad=.7,labelspacing=.55,
        borderaxespad=0,fontsize=7.2)
    # Metric bar is in the same equal-aspect data coordinates as map and paths.
    x = limits[0,0]+19.; y = limits[0,1]+35.; length = 100.
    ax.plot([x,x+length],[y,y],color='#29343a',lw=1.1,zorder=5)
    ax.plot([x,x],[y-3,y+3],color='#29343a',lw=.8,zorder=5)
    ax.plot([x+length,x+length],[y-3,y+3],color='#29343a',lw=.8,zorder=5)
    ax.text(x+length/2,y+8,'100 m',ha='center',va='bottom',fontsize=7.5,color='#29343a')
    stem = HERE/'graco14-circle-topview-teaser'
    fig.savefig(stem.with_suffix('.pdf'),dpi=900)
    fig.savefig(stem.with_suffix('.png'),dpi=600)
    fig.savefig(HERE/'preview.png',dpi=300)
    plt.close(fig)
    manifest = dict(map_sha256=info['sha256'],export_metadata_sha256=sha256(REPORT/'export.json'),
        source_script_sha256=sha256(Path(__file__)),trajectory_sha256=hashes,
        robots=info['robots'],map_points=info['points'],display_points=len(subset),
        original_map_colors=True,trajectory_colors_rgb=metadata['colors_rgb'],
        trajectory_color_policy='Exact corresponding robot RGB from PLY export metadata',
        orientation='Identical to report orthographic top view',
        trajectory_transform=level.tolist(),ground_truth_used=False,trajectory_smoothing=False,
        gaps_over_1s=gaps,scale_bar_m=length,scale_bar_endpoints_xy=[[x,y],[x+length,y]],
        figure_size_inches=[width,height],pdf_map_dpi=900,png_dpi=600,
        pdf_trajectory_and_text='vector',limits_xy=limits.tolist())
    (HERE/'provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(size_inches=[width,height],robots=len(tracks),gaps=gaps)))


if __name__ == '__main__':
    main()

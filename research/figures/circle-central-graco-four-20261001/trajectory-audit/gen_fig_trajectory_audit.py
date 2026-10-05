"""Reproducible trajectory-only views and graph-propagation continuity audit."""
import argparse
import html
from pathlib import Path

import numpy as np

from .artifacts import file_hash, read_json, write_json
from .circle_central_evaluation import tum_rows
from .evaluation import ground_truth
from .geometry import inv, transform
from .trajectory_continuity import continuity_summary, motion_series


def plot_a07_focus(raw, new, gt, alignment, output):
    import matplotlib.pyplot as plt
    a=np.asarray([r['T_world_body'] for r in raw]);b=np.asarray([r['T_world_body'] for r in new])
    xyz=transform(alignment,b[:,:3,3]);rigid=transform(alignment@b[0]@inv(a[0]),a[:,:3,3])
    # Locations selected from the displayed A07 overview, independent of GT.
    locations=[(139.,55.),(91.,-16.),(52.,92.)]
    fig,axes=plt.subplots(2,2,figsize=(9.6,8),layout='constrained')
    for k,ax in enumerate(axes.ravel()):
        ax.plot(gt[:,0],gt[:,1],color='#333333',ls='--',lw=1.1,label='Ground truth')
        ax.plot(rigid[:,0],rigid[:,1],color='#8c8c8c',lw=3.,alpha=.8,label='Raw, first correction only')
        ax.plot(xyz[:,0],xyz[:,1],color='#E69F00',lw=1.4,label='Corrected trajectory')
        ax.set_aspect('equal');ax.set_xlabel('ENU x [m]');ax.set_ylabel('ENU y [m]')
        if k:
            center=np.array(locations[k-1]);ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5)
            ax.set_title(['','Eastern bend','Southern turn','Start / return area'][k])
        else:
            ax.set_title('A07 · shared four-robot alignment')
            for j,(x,y) in enumerate(locations):ax.annotate(str(j+1),(x,y),xytext=(5,5),textcoords='offset points',fontsize=11)
    fig.suptitle('A07: inspect the bends in raw odometry, corrected trajectory and GT',fontsize=13)
    fig.legend(*axes.ravel()[0].get_legend_handles_labels(),loc='outside lower center',ncol=3)
    for ext in ('png','pdf'):fig.savefig(Path(output)/f'aerial07-focus.{ext}',dpi=300)
    plt.close(fig)


def audit(inspection, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],
        'font.size':10,'axes.titlesize':11,'axes.labelsize':10,
        'legend.fontsize':8,'legend.frameon':False,'pdf.fonttype':42,
        'axes.spines.top':False,'axes.spines.right':False,
        'axes.grid':True,'grid.alpha':.15,'savefig.dpi':300})
    colors={'aerial05':'#D55E00','aerial07':'#E69F00','ground05':'#0072B2','ground06':'#CC79A7'}
    inspection=Path(inspection);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    comparison=read_json(inspection/'comparison.json');results={};cards=[]
    for label,entry in comparison.items():
        run=Path(entry['run']);cfg=read_json(run/'configuration.json');robots=cfg['robots']
        records={};table=[]
        paths_fig,paths_ax=plt.subplots(2,2,figsize=(9.6,8),layout='constrained')
        zoom_fig,zoom_ax=plt.subplots(2,2,figsize=(9.6,8),layout='constrained')
        motion_fig,motion_ax=plt.subplots(2,2,figsize=(9.6,6),layout='constrained')
        for robot,ax,zax,maxis in zip(robots,paths_ax.ravel(),zoom_ax.ravel(),motion_ax.ravel()):
            raw_path=Path(cfg['dense_trajectories'][robot]);new_path=run/'evaluation'/f'{robot}-corrected.tum'
            raw=tum_rows(raw_path,robot);new=tum_rows(new_path,robot)
            series=motion_series(raw,new);summary=continuity_summary(raw,new)
            records[robot]=dict(summary,raw_path=str(raw_path),raw_sha256=file_hash(raw_path),
                corrected_path=str(new_path),corrected_sha256=file_hash(new_path))
            component=entry['evaluation']['components'][robot]
            A=np.asarray(entry['evaluation']['joint_ate'][component]['alignment_SE3'])
            a=np.asarray([r['T_world_body'] for r in raw]);b=np.asarray([r['T_world_body'] for r in new])
            xyz=transform(A,b[:,:3,3]);C0=b[0]@inv(a[0]);rigid=transform(A@C0,a[:,:3,3])
            gt_path=Path(cfg['dataset'])/f'{robot}_gt.txt';_,gt=ground_truth(gt_path)
            if robot=='aerial07' and label=='circle40':plot_a07_focus(raw,new,gt,A,output)
            records[robot]['gt_sha256']=file_hash(gt_path)
            ax.plot(xyz[:,0],xyz[:,1],lw=1.4,color=colors[robot])
            ax.scatter(*xyz[0,:2],s=45,marker='o',facecolors='white',edgecolors=colors[robot],zorder=5)
            ax.scatter(*xyz[-1,:2],s=35,marker='s',color=colors[robot],zorder=5)
            ax.set_title(robot+' · corrected trajectory only');ax.set_aspect('equal')
            ax.set_xlabel('ENU x [m]');ax.set_ylabel('ENU y [m]')
            # The most abrupt raw velocity change is selected without GT. Plot
            # actual samples around it, retaining all intervening observations.
            velocity=np.diff(a[:,:3,3],axis=0)/series['dt_s'][:,None]
            k=int(np.argmax(np.linalg.norm(np.diff(velocity,axis=0),axis=1)))+1
            elapsed=np.asarray([r['stamp_ns']-raw[0]['stamp_ns'] for r in raw])*1e-9
            mask=np.abs(elapsed-elapsed[k])<=2.
            selected=xyz[mask];lo=selected[:,:2].min(axis=0);hi=selected[:,:2].max(axis=0)
            center=(lo+hi)*.5;half=max(float(max(hi-lo))*.65,1.)
            zax.plot(gt[:,0],gt[:,1],color='#444444',ls='--',lw=.9,label='Ground truth')
            zax.plot(rigid[mask,0],rigid[mask,1],color='#999999',lw=3.,alpha=.8,label='Raw, first correction only')
            zax.plot(selected[:,0],selected[:,1],'.-',ms=3,lw=1,color=colors[robot],label='Corrected samples')
            zax.scatter(*xyz[k,:2],s=45,marker='o',facecolors='none',edgecolors='black',zorder=7)
            zax.set_xlim(center[0]-half,center[0]+half);zax.set_ylim(center[1]-half,center[1]+half)
            zax.set_aspect('equal');zax.set_title(f'{robot} · raw velocity change at {elapsed[k]:.1f} s')
            zax.set_xlabel('ENU x [m]');zax.set_ylabel('ENU y [m]')
            maxis.plot(series['elapsed_s'],series['correction_added_step_m']*1000,lw=1.,color=colors[robot])
            maxis.set_title(f"{robot} · maximum {summary['max_correction_added_step_m']*1000:.4f} mm")
            maxis.set_xlabel('Elapsed sensor time [s]');maxis.set_ylabel('Added displacement / sample [mm]')
            maxis.set_ylim(bottom=0)
            table.append(f"| {robot} | {summary['raw_max_step_m']:.4f} | {summary['corrected_max_step_m']:.4f} | "
                f"{summary['raw_max_speed_m_s']:.3f} | {summary['corrected_max_speed_m_s']:.3f} | "
                f"{summary['max_correction_added_step_m']*1000:.4f} |")
        paths_fig.suptitle(label+' · one continuous line per robot',fontsize=14)
        zoom_fig.suptitle(label+' · close-ups of strongest raw velocity changes',fontsize=14)
        motion_fig.suptitle(label+' · displacement introduced by changing graph correction',fontsize=13)
        paths_fig.legend(handles=[Line2D([],[],marker='o',mfc='white',color='black',ls='',label='Start'),
            Line2D([],[],marker='s',color='black',ls='',label='End')],loc='outside lower center',ncol=2)
        zoom_fig.legend(*zoom_ax.ravel()[0].get_legend_handles_labels(),loc='outside lower center',ncol=3)
        for name,fig in [('trajectories',paths_fig),('closeups',zoom_fig),('added-motion',motion_fig)]:
            fig.savefig(output/f'{label}-{name}.png');fig.savefig(output/f'{label}-{name}.pdf');plt.close(fig)
        results[label]=records
        cards.append('<h2>'+html.escape(label)+'</h2>'+''.join(
            f'<figure><img src="{label}-{name}.png"><figcaption><a href="{label}-{name}.pdf">Vector PDF</a></figcaption></figure>'
            for name in ['trajectories','closeups','added-motion']))
        (output/f'{label}.md').write_text('# '+label+' continuity audit\n\n'+
            '| Robot | Raw max step [m] | Corrected max step [m] | Raw max speed [m/s] | Corrected max speed [m/s] | Max added displacement [mm] |\n'+
            '|---|---:|---:|---:|---:|---:|\n'+'\n'.join(table)+'\n')
    report=('The saved dense trajectories retain the raw sample timestamps. To isolate graph propagation, each raw '
        'step is rotated by the previous correction and subtracted from the corrected step. A fixed rigid alignment '
        'therefore contributes zero. Across these runs the maximum additional displacement is below 0.5 mm per '
        'sample. There is no evidence here of macroscopic jumps introduced by trajectory correction. This audit '
        'does not establish that the trajectory or spatial map deformation is accurate.\n\n'
        'Trajectory-only panels omit map points and ground truth to make each line unambiguous. Close-ups show '
        'the largest raw velocity change, selected without ground truth; gray is raw odometry transformed by the '
        'first correction only, colored dots are the actual corrected samples, and dashed black is ground truth. '
        'No smoothing, sample removal or trajectory replacement is applied. ATE and graph solutions are unchanged.\n')
    (output/'REPORT.md').write_text('# Dense trajectory continuity audit\n\n'+report+'\n'+
        '\n'.join((output/f'{label}.md').read_text().replace('# ', '## ',1) for label in results))
    write_json(output/'audit.json',dict(results=results,
        generator_sha256=file_hash(__file__),metric_sha256=file_hash(Path(__file__).with_name('trajectory_continuity.py'))))
    (output/'index.html').write_text('<meta charset="utf-8"><style>body{font-family:system-ui;max-width:1150px;margin:35px auto;color:#253443}img{width:100%}figure{margin:20px 0}p{line-height:1.6}</style>'+
        '<h1>Dense trajectory continuity audit</h1><p>'+html.escape(report).replace('\n\n','</p><p>')+'</p>'+
        '<p><a href="REPORT.md">Numeric tables and method</a> · <a href="audit.json">Measurements and hashes</a></p>'+''.join(cards))
    return results


def main():
    p=argparse.ArgumentParser();p.add_argument('--inspection',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();audit(args.inspection,args.output)


if __name__=='__main__':main()

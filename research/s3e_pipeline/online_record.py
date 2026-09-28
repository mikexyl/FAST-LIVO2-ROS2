"""Record actual online arrivals, never replay a final optimized map backwards.

Consumes live TF capture, immutable snapshots, and published backend revisions.
Writes Rerun as it runs and RGB frames to a concurrently running FFmpeg encoder.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from .artifacts import read_json
from .online_io import JsonlTail, Journal, atomic_json
from .online_display import corrections, corrected, component_level
from .robot_colors import robot_colors
from .online_video import group_video, video_fonts


class TumTail:
    def __init__(self,path): self.path=path; self.offset=0; self.pending=''
    def read(self):
        if not self.path.exists(): return []
        with self.path.open() as f: f.seek(self.offset); text=f.read(); self.offset=f.tell()
        lines=(self.pending+text).split('\n'); self.pending=lines.pop(); rows=[]
        for line in lines:
            if not line or line.startswith('#'): continue
            parts=line.split(); seconds,fraction=parts[0].split('.')
            rows.append([int(seconds)*10**9+int(fraction.ljust(9,'0')),*map(float,parts[1:])])
        return rows


class Recorder:
    def __init__(self,work,robots,fps=5):
        self.work=work; self.robots=robots; self.fps=fps; self.out=work/'demo'
        self.colors=robot_colors(robots); self.rows={r:[] for r in robots}; self.poses={r:[] for r in robots}
        self.chunks={r:[] for r in robots}; self.ids={r:set() for r in robots}
        self.row_tails={r:JsonlTail(work/f'prepared-{r}/store/keyframes.jsonl') for r in robots}
        self.pose_tails={r:TumTail(work/f'frontends/{r}/recording/post_lidar_poses.tum') for r in robots}
        self.estimates={r:{} for r in robots}; self.components={r:r for r in robots}; self.revision=0
        self.loops=[]; self.latest=None; self.frame=0; self.last_full=-1; self.main=None
        self.map_versions={}; self.layout=None
        self.journal=Journal(self.out/'events.jsonl'); self.frames=Journal(self.out/'frames.jsonl')
        self.fonts=video_fonts()
        rr.init('GRACO | online ground + aerial swarm'); rr.save(str(self.out/'online.rrd'))
        rr.log('/about',rr.TextDocument('Fresh concurrent 1× ROS2 bag replay. Persistent EllipseLIO odometry; accumulated-area multilayer ellipsoid BEVs; peer DDS retrieval and verification; periodic distributed PCM/CBS with registration factors.\n\nAll logs use receipt/publication wall time. Original sensor timestamps remain in evidence. Maps are displayed separately until CBS connects them. No ground truth is read. Display geometry is sampled native processed map points; not raw scans. Optimization revises the displayed map only when its result becomes available.'),static=True)
        self.pipe=(self.out/'video.rgb').open('wb',buffering=0)
        atomic_json(self.out/'READY',dict(wall_ns=time.time_ns(),rerun=rr.__version__))

    def poll(self):
        changed=False
        for r in self.robots:
            poses=self.pose_tails[r].read()
            self.poses[r].extend(poses)
            if poses: self.journal.append(dict(type='poses_received',robot=r,poses=poses))
            for row in self.row_tails[r].read():
                if row['available_ns']>time.time_ns(): raise ValueError('Future display snapshot')
                key=row['keyframe_id']
                if key!=len(self.rows[r]): raise ValueError('Noncontiguous display input')
                native=self.work/f'frontends/{r}/frontend/area_maps'/row['payload']
                with np.load(native,allow_pickle=False) as payload:
                    points=payload['points']; ids=payload['point_ids']
                    mask=np.fromiter((int(x) not in self.ids[r] for x in ids),dtype=bool,count=len(ids))
                    self.ids[r].update(map(int,ids)); points=points[mask]
                # Only the display is thinned; every native evidence point remains stored.
                if len(points):
                    _,selection=np.unique(np.floor(points/1.).astype(np.int32),axis=0,return_index=True)
                    points=points[selection].astype(np.float32)
                T=np.asarray(row['T_world_body']); world=points@T[:3,:3].T+T[:3,3]
                self.chunks[r].append((key,row['stamp_ns'],world)); self.rows[r].append(row); changed=True
                self.journal.append(dict(type='map_displayed',robot=r,keyframe_id=key,
                    descriptor_available_ns=row['available_ns'],display_points=len(points)))
        latest=self.work/'epochs/latest.json'
        if latest.exists():
            value=read_json(latest)
            if value['revision']>self.revision:
                if value['published_wall_ns']>time.time_ns(): raise ValueError('Future optimizer output')
                self.revision=value['revision']; self.latest=value; path=Path(value['path'])
                poses=JsonlTail(path/'poses.jsonl').read(); self.estimates={r:{} for r in self.robots}
                for pose in poses:
                    r=pose['robot_id']; self.estimates[r][pose['keyframe_id']]=pose['T_world_body']
                    self.components[r]=pose['component']
                self.loops=JsonlTail(path/'constraints.jsonl').read(); changed=True
                self.journal.append(dict(type='revision_displayed',**value,components=self.components))
        return changed

    def scene(self):
        groups={g:[r for r in self.robots if self.components[r]==g] for g in set(self.components.values())}
        main=min(groups,key=lambda g:(-len(groups[g]),g))
        levels={g:component_level(self.rows[g],self.estimates[g]) for g in groups}
        tracks={}; clouds={}; transforms={}
        for r in self.robots:
            anchors,delta=corrections(self.rows[r],self.estimates[r]); transforms[r]=(anchors,delta)
            L=levels[self.components[r]][:3,:3]
            pose=self.poses[r]
            tracks[r]=(corrected(np.array([x[1:4] for x in pose]),np.array([x[0] for x in pose]),anchors,delta)@L.T
                       if pose else np.empty((0,3)))
            parts=[corrected(x,np.full(len(x),stamp,dtype=np.int64),anchors,delta) for _,stamp,x in self.chunks[r] if len(x)]
            clouds[r]=np.concatenate(parts)@L.T if parts else np.empty((0,3))
        return groups,main,tracks,clouds,levels

    def record_scene(self,elapsed,scene,full):
        groups,main,tracks,clouds,levels=scene
        layout=tuple(sorted(self.components.items()))
        if self.layout!=layout:
            rr.log('/world',rr.Clear(recursive=True)); self.map_versions={}
            views=[rrb.Spatial3DView(
                name=('Global group · all robots aligned' if len(groups)==1 else
                      f'{g} · {len(rs)} robot(s)'), origin=f'/world/{g}')
                for g,rs in sorted(groups.items(),key=lambda item:(-len(item[1]),item[0]))]
            rr.send_blueprint(rrb.Blueprint(views[0] if len(views)==1 else
                rrb.Grid(*views,grid_columns=4 if len(views)>8 else 3 if len(views)>4 else 2)))
            self.main=main; self.layout=layout
        for g in groups: rr.log(f'/world/{g}',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
        for r in self.robots:
            root=f'/world/{self.components[r]}/{r}'; track=tracks[r]
            if len(track):
                rr.log(root+'/position',rr.Points3D(track[-1:],colors=self.colors[r],radii=.8,labels=[r]))
                if full: rr.log(root+'/trajectory',rr.LineStrips3D([track],colors=self.colors[r],radii=.13))
            version=(self.revision,len(self.rows[r]))
            if self.map_versions.get(r)!=version and len(clouds[r]):
                rr.log(root+'/map',rr.Points3D(clouds[r],colors=self.colors[r],radii=.05)); self.map_versions[r]=version
        if full:
            for g in groups:
                segments=[]
                for edge in self.loops:
                    i,j=edge['i'],edge['j']
                    if self.components[i[0]]!=g: continue
                    if i[1] not in self.estimates[i[0]] or j[1] not in self.estimates[j[0]]: continue
                    segments.append([levels[g][:3,:3]@np.array(self.estimates[r][k])[:3,3] for r,k in (i,j)])
                if segments: rr.log(f'/world/{g}/loops',rr.LineStrips3D(segments,colors=[240,190,85],radii=.04))
        status=f'ONLINE · {elapsed:.1f} s\n\n{len(self.robots)} concurrent robots\n{len(groups)} connected components\n{len(self.loops)} PCM-retained loops\nCBS revision {self.revision}\n{sum(map(len,self.rows.values()))} available area maps\n\n'+('\n'.join(f'{r}: {len(self.poses[r])} poses, {len(self.rows[r])} maps' for r in self.robots))
        rr.log('/status',rr.TextDocument(status))

    def video(self,elapsed,scene):
        return group_video(self,elapsed,scene)

    def run(self):
        while not (self.work/'START').exists(): time.sleep(.05)
        start=read_json(self.work/'START')['wall_ns']; last_image=None; cached=None
        try:
            while True:
                changed=self.poll(); now=time.time_ns(); elapsed=(now-start)/1e9
                rr.set_time('online_elapsed',duration=elapsed)
                full=changed or elapsed-self.last_full>=1.
                if full or cached is None: cached=self.scene(); self.last_full=elapsed
                self.record_scene(elapsed,cached,full)
                target=int(elapsed*self.fps)
                if target>=self.frame:
                    current=self.video(elapsed,cached)
                    while self.frame<target and last_image is not None:
                        self.pipe.write(last_image.tobytes()); self.frame+=1
                    self.pipe.write(current.tobytes()); self.frame+=1; last_image=current
                    self.frames.append(dict(frame=self.frame-1,elapsed_s=elapsed,revision=self.revision,
                        largest_component=len(cached[0][cached[1]]),loops=len(self.loops)))
                    if changed or int(elapsed)%30==0: current.save(self.out/'preview.jpg',quality=92)
                atomic_json(self.out/'progress.json',dict(elapsed_s=elapsed,frames=self.frame,revision=self.revision,
                    poses={r:len(x) for r,x in self.poses.items()},submaps={r:len(x) for r,x in self.rows.items()}))
                if (self.work/'STOP_RECORDING').exists(): break
                time.sleep(.15)
        finally:
            self.pipe.close(); rr.get_data_recording().flush(); rr.disconnect()
            self.journal.close(); self.frames.close()
            if last_image is not None: last_image.save(self.out/'final.png')
            atomic_json(self.out/'summary.json',dict(frames=self.frame,fps=self.fps,revisions=self.revision,
                components=self.components,retained_loops=len(self.loops),finished_wall_ns=time.time_ns(),
                ground_truth_used=False,recording='online.rrd',video='online.mp4'))


if __name__=='__main__':
    import yaml
    p=argparse.ArgumentParser(); p.add_argument('--work',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True); a=p.parse_args()
    Recorder(a.work,yaml.safe_load(a.config.read_text())['robots']).run()

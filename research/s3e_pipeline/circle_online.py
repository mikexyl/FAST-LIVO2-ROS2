"""Live fixed circles with opt-in geometry maturation before immutable publication.

Only an append-only native index is consumed. No end-of-capture map, saved
trajectory, future geometry, or ground truth is available to this worker.
"""
import argparse
from dataclasses import asdict, dataclass
from pathlib import Path
import time
import zlib

import numpy as np
import yaml

from .area_maps import AreaHistoryAudit
from .artifacts import canonical, digest, file_hash, read_json, write_json
from .backends import pack_array
from .circle_cover import CircleCoverConfig, CircleCoverPlanner, placement_metrics
from .circle_submaps import SCHEMA, PREPROCESSING, id_ranges, validate_geometry, validate_row
from .gravity_bev import gravity_ground, submap_gravity
from .multilayer_mapclosures import describe_layers
from .online_io import JsonlTail, Journal, atomic_json


@dataclass(frozen=True)
class PublicationConfig:
    mode: str = 'freeze_at_creation'
    minimum_age_s: float = 30.
    stability_window_s: float = 30.
    maximum_growth_fraction: float = .05
    maximum_wait_s: float = 120.

    def __post_init__(self):
        if self.mode not in ('freeze_at_creation','mature_then_freeze'):
            raise ValueError('Unknown circle publication mode')
        values=[self.minimum_age_s,self.stability_window_s,self.maximum_growth_fraction,self.maximum_wait_s]
        if not np.isfinite(values).all() or min(values)<0 or self.stability_window_s<=0:
            raise ValueError('Invalid maturation parameters')
        if self.maximum_wait_s<max(self.minimum_age_s,self.stability_window_s):
            raise ValueError('Maturation deadline precedes minimum age/stability window')


def maturation_decision(history, created_ns, now_ns, config, final=False):
    """Sensor-time decision. Both occupied-cell and unique-point growth must settle."""
    age=(now_ns-created_ns)/1e9
    old=[h for h in history if h['native_available_ns']<=now_ns-round(config.stability_window_s*1e9)]
    growth=None
    if old:
        growth={k:(history[-1][k]-old[-1][k])/max(old[-1][k],1) for k in ('points','cells')}
    stable=bool(growth and all(v<=config.maximum_growth_fraction for v in growth.values()))
    reason=('creation' if config.mode=='freeze_at_creation' else 'end_of_stream' if final else
        'deadline' if age>=config.maximum_wait_s else
        'stable' if age>=config.minimum_age_s and stable else 'growing')
    return dict(reason=reason,age_s=age,growth=growth,stable=stable,
        stability_window_s=config.stability_window_s,maximum_growth_fraction=config.maximum_growth_fraction)


class GrowingCircles:
    """Permanent centers and immutable geometry using only observations so far."""
    def __init__(self, config, publication=None):
        self.config=config; self.audit=AreaHistoryAudit(); self.planner=None
        self.publication=publication or PublicationConfig()
        self.ids=np.empty(0,np.int64); self.world=np.empty((0,3))
        self.sources=[]; self.rows=[]; self.minimum_coverage=1.; self.placement_s=0.
        self.pending={};self.anchors={};self.last_native=None;self.next_publish=0;self.decisions=[]

    def add(self, row, data, source_hash, received_ns):
        self.audit.add(row,data)
        if self.planner is None:
            self.robot=row['robot_id']; self.gravity=np.asarray(row['gravity_world_m_s2'])
            self.frame=gravity_ground(self.gravity); self.begin=row['begin_ns']
            self.planner=CircleCoverPlanner(self.config,self.frame)
        elif row['robot_id']!=self.robot:
            raise ValueError('Foreign robot observation')
        T=np.asarray(row['T_world_imu']).reshape(4,4)
        ids=np.asarray(data['point_ids'],np.int64)
        world=np.asarray(data['points'],float)@T[:3,:3].T+T[:3,3]
        indices=np.searchsorted(self.ids,ids); known=indices<len(self.ids)
        known[known]&=self.ids[indices[known]]==ids[known]
        if known.any() and not np.allclose(self.world[indices[known]],world[known],atol=1e-3,rtol=1e-6):
            raise ValueError('Native point identity changed')
        new=world[~known]; self.ids=np.r_[self.ids,ids[~known]]
        self.world=np.vstack([self.world,new]); order=np.argsort(self.ids,kind='stable')
        self.ids,self.world=self.ids[order],self.world[order]
        self.sources.append(dict(native_submap_id=row['submap_id'],sha256=source_hash,
            anchor_ns=row['stamp_ns'],native_available_ns=row['available_ns'],
            received_wall_ns=received_ns,first_seen_points=len(new)))
        previous=self.planner.circles; started=time.perf_counter()
        update=self.planner.update(new,len(self.sources)-1)
        elapsed=time.perf_counter()-started; self.placement_s+=elapsed
        if self.planner.circles[:len(previous)]!=previous:
            raise ValueError('Published circle moved')
        coverage=update.diagnostics['coverage']; self.minimum_coverage=min(self.minimum_coverage,coverage)
        if len(self.ids) and coverage+1e-12<self.config.coverage_target:
            raise ValueError('Online circle coverage target missed')
        for circle in update.new_circles:
            self.pending[circle.id]=[];self.anchors[circle.id]=dict(row)
        self.last_native=dict(row)
        xy=self.planner.project(self.world)
        for key,counts in self.pending.items():
            circle=self.planner.circles[key]
            mask=np.sum((xy-circle.center_xy)**2,axis=1)<=self.config.radius_m**2
            cells=np.floor((xy[mask]-self.config.grid_origin_xy)/self.config.cell_size_m).astype(np.int64)
            counts.append(dict(native_available_ns=row['available_ns'],points=int(mask.sum()),cells=len(np.unique(cells,axis=0))))
        # Legacy mode remains byte-for-byte equivalent in geometry membership.
        created=self.release() if self.publication.mode=='freeze_at_creation' else []
        history=dict(update_id=len(self.sources)-1,native_available_ns=row['available_ns'],
            received_wall_ns=received_ns,observed_points=len(self.ids),circle_count=len(self.planner.circles),
            new_circles=[c.id for c in update.new_circles],placement_s=elapsed,diagnostics=update.diagnostics)
        return created,history

    def release(self, quality=None, final=False):
        """Publish in permanent ID/anchor order; a bounded wait prevents head blocking.

        The quality callback only inspects the current crop. It cannot see future
        observations. Ready later circles continue accumulating behind the head.
        """
        if self.last_native is None:return []
        current=self.last_native;received_ns=self.sources[-1]['received_wall_ns']
        xy=self.planner.project(self.world);created=[]
        while self.next_publish in self.pending:
            key=self.next_publish;circle=self.planner.circles[key];row=self.anchors[key]
            decision=maturation_decision(self.pending[key],row['available_ns'],current['available_ns'],self.publication,final)
            if decision['reason']=='growing':break
            mask=np.sum((xy-circle.center_xy)**2,axis=1)<=self.config.radius_m**2
            T=np.asarray(row['T_world_imu']).reshape(4,4)
            cloud=((self.world[mask]-T[:3,3])@T[:3,:3]).astype(np.float32)
            item=dict(schema_version=SCHEMA,strategy='fixed_spatial',placement_mode='circle',
                robot_id=self.robot,submap_id=key,keyframe_id=key,
                stamp_ns=row['stamp_ns'],available_ns=current['available_ns'],seal_ns=current['available_ns'],
                last_member_ns=current['available_ns'],begin_ns=self.begin,end_ns=current['anchor_sensor_ns'],
                created_update=circle.created_update,created_available_ns=row['available_ns'],
                anchor_source_keyframe=row['submap_id'],T_world_body=T.tolist(),T_world_imu=T.ravel().tolist(),
                T_horizontal_map=self.frame.tolist(),footprint_center_xy=list(circle.center_xy),
                footprint_radius_m=self.config.radius_m,frame='creation_anchor_imu',
                body_frame=self.robot+'/imu',cloud_frame=self.robot+'/imu',world_frame=self.robot+'/odom_ellipselio',
                gravity_world_m_s2=self.gravity.tolist(),gravity_imu_m_s2=(T[:3,:3].T@self.gravity).tolist(),
                gravity_source='fixed first live native snapshot gravity',geometry_count=len(cloud),
                geometry_id_ranges=id_ranges(self.ids[mask]),member_scan_ids=[],
                membership_semantics='stable native point IDs; immutable published crop',
                geometry_source='growing union of live native area observations available at sealing',
                geometry_preprocessing=PREPROCESSING,geometry_publication_policy=self.publication.mode,
                image_available=False,retrievable=len(cloud)>=30,complete=True,
                source_observations_hash=digest(self.sources),config_hash=digest(asdict(self.config)),
                snapshot_received_wall_ns=received_ns,native_available_ns=current['available_ns'],
                sealed_update=len(self.sources)-1,publication_config=asdict(self.publication))
            if self.publication.mode=='freeze_at_creation':
                item['last_member_ns']=row['last_member_ns']
            ready,diagnostic=(quality(item,cloud) if quality else (True,dict(status='legacy_creation')))
            decision.update(quality=diagnostic,quality_ready=ready,keyframe_id=key,sealed_update=len(self.sources)-1)
            if decision['reason']=='stable' and not ready:
                decision['reason']='waiting_for_features';self.decisions.append(decision);break
            if self.publication.mode=='mature_then_freeze' and quality is None:
                raise ValueError('Mature publication requires a quality assessment')
            item['maturation']=decision
            created.append((item,cloud,self.ids[mask].copy()))
            self.decisions.append(decision);del self.pending[key];self.next_publish+=1
        return created

    def finish(self, output):
        if self.planner is None:raise ValueError('No initialized map observations')
        np.savez_compressed(output/'union.npz',world=self.world.astype(np.float32),point_ids=self.ids)
        self.planner.save(output/'planner-state.json')
        if self.pending:raise ValueError('Unsealed circles at shutdown')
        xy=self.planner.project(self.world); memberships=np.zeros(len(xy),np.int32)
        for c in self.planner.circles:memberships+=np.sum((xy-c.center_xy)**2,axis=1)<=self.config.radius_m**2
        summary=dict(mode='circle',robot_id=self.robot,config=asdict(self.config),
            footprints=len(self.planner.circles),observations=len(self.sources),points=len(self.ids),
            metrics=placement_metrics(self.planner.cell_centers,[c.center_xy for c in self.planner.circles],self.config),
            minimum_prefix_coverage=self.minimum_coverage,actual_point_coverage=float(np.mean(memberships>0)),
            mean_point_membership=float(np.mean(memberships)),unassigned_points=int(np.sum(memberships==0)),
            placement_s=self.placement_s,source_hash=digest(self.sources),source_payloads=self.sources,
            publication_config=asdict(self.publication),
            geometry_policy='immutable seal-time crop; no geometry revisions or future observations')
        write_json(output/'placement.json',summary)
        return summary


def descriptors(engine, row, cloud):
    ground,alignment=submap_gravity(row); started=time.perf_counter()
    full=engine.describe(cloud,ground=ground,include_image=True)
    full_s=time.perf_counter()-started; started=time.perf_counter()
    layers=describe_layers(engine,cloud,cloud,ground)
    layers_s=time.perf_counter()-started
    descriptor=dict(mapclosures={k:pack_array(full[k]) for k in ('ground','xy','bits')},
        mapclosures_features=len(full['xy']),mapclosures_multilayer=layers,
        representation='point_cloud',renderer='point_cloud',projection_alignment=alignment,
        submap_id=row['submap_id'],member_scan_ids=[],payload_sha256=row.get('sha256'))
    return descriptor,np.asarray(full['image'],np.uint8),dict(full_s=full_s,layers_s=layers_s,
        full_features=len(full['xy']),layer_features=layers['feature_counts'],terrain=layers['terrain'])


def describe(engine, row, cloud, output, prepared=None):
    from PIL import Image
    descriptor,image,timing=prepared if prepared is not None else descriptors(engine,row,cloud)
    descriptor=dict(descriptor,payload_sha256=row['sha256'])
    key=row['keyframe_id']
    (output/'ellipsoid'/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(descriptor)))
    Image.fromarray(image).save(output/'images'/f'{key:06d}.png')
    return timing


def run(source, output, config):
    import s3e_mapclosures_native as native
    source,output=Path(source),Path(output);output.mkdir(parents=True,exist_ok=False)
    for folder in ('store','ellipsoid','images'):(output/folder).mkdir()
    opts=config['backend']['mapclosures']
    if opts['representation']!='point_cloud' or native.upstream_commit!=opts['upstream_commit']:
        raise ValueError('Incorrect online renderer/native MapClosures revision')
    engine=native.MapClosures(opts['density_map_resolution'],opts['density_threshold'],opts['hamming_distance_threshold'])
    growing=GrowingCircles(CircleCoverConfig(**config['circle_cover']),
        PublicationConfig(**config.get('circle_publication',{})))
    tail=JsonlTail(source/'index.jsonl');seen=[];timings=[];started=time.time_ns()
    history=Journal(output/'placement-history.jsonl')
    with (output/'store/keyframes.jsonl').open('xb',buffering=0) as index:
        def publish(created, cached):
            for row,cloud,ids in created:
                key=row['keyframe_id'];payload=output/'store'/f'{key:06d}.npz'
                np.savez_compressed(payload,cloud=cloud,point_ids=ids)
                row['sha256']=file_hash(payload)
                row['cbs_evidence_voxel_m']=config['backend']['evidence']['voxel_m']
                validate_geometry(row,cloud,ids)
                timing=describe(engine,row,cloud,output,cached.get(key)) if row['retrievable'] else dict(full_s=0.,layers_s=0.)
                row.update(available_ns=time.time_ns(),availability_clock='unix_wall_descriptor_publication')
                validate_row(row); index.write(canonical(row)+b'\n');growing.rows.append(row)
                timing.update(keyframe_id=key,available_ns=row['available_ns'],
                    snapshot_received_wall_ns=row['snapshot_received_wall_ns'],
                    prepare_latency_s=(row['available_ns']-row['snapshot_received_wall_ns'])/1e9,
                    maturation=row['maturation'],created_update=row['created_update'],sealed_update=row['sealed_update'])
                timings.append(timing)

        def release(final=False):
            cached={}
            def quality(row,cloud):
                started=time.perf_counter()
                if len(cloud)<30:
                    return False,dict(terrain_available=False,layer_features=0,
                        minimum_features=opts['inliers_threshold'],reason='insufficient_points',runtime_s=0.)
                prepared=descriptors(engine,row,cloud);cached[row['keyframe_id']]=prepared
                d=prepared[2];features=sum(d['layer_features'].values())
                ready=bool(d['terrain']['available'] and features>=opts['inliers_threshold'])
                return ready,dict(terrain_available=d['terrain']['available'],layer_features=features,
                    minimum_features=opts['inliers_threshold'],runtime_s=time.perf_counter()-started)
            created=growing.release(quality,final)
            publish(created,cached)

        atomic_json(output/'READY',dict(wall_ns=time.time_ns()))
        try:
            while True:
                additions=tail.read()
                for native_row in additions:
                    seen.append(native_row);received=time.time_ns()
                    if not native_row['complete']:continue
                    path=source/native_row['payload']
                    if path.parent!=source or file_hash(path)!=native_row['sha256']:
                        raise ValueError('Native snapshot hash/path mismatch')
                    with np.load(path,allow_pickle=False) as data:
                        created,event=growing.add(native_row,data,native_row['sha256'],received)
                    history.append(event)
                    if growing.publication.mode=='freeze_at_creation':publish(created,{})
                    else:release()
                    atomic_json(output/'progress.json',dict(submaps=len(growing.rows),observations=len(growing.sources),
                        pending_circles=len(growing.pending),points=len(growing.ids),wall_ns=time.time_ns()))
                manifest=source/'manifest.json'
                if manifest.exists() and not additions:
                    final=read_json(manifest)
                    if not final['complete'] or file_hash(source/'index.jsonl')!=final['index_sha256']:
                        raise ValueError('Incomplete native manifest')
                    tail.finish()
                    if [__import__('json').loads(x) for x in (source/'index.jsonl').read_text().splitlines()]!=seen:
                        raise ValueError('Native index changed while streaming')
                    if growing.pending:release(final=True)
                    placement=growing.finish(output)
                    from .artifacts import write_jsonl
                    write_jsonl(output/'timings.jsonl',timings)
                    write_jsonl(output/'maturation-decisions.jsonl',growing.decisions)
                    atomic_json(output/'summary.json',dict(complete=True,submaps=len(growing.rows),
                        source_rows=len(seen),source_index_sha256=final['index_sha256'],placement=placement,
                        started_wall_ns=started,finished_wall_ns=time.time_ns(),ground_truth_used=False,
                        publication_policy=growing.publication.mode))
                    return
                time.sleep(.05)
        finally:history.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    a=p.parse_args();run(a.source,a.output,yaml.safe_load(a.config.read_text()))

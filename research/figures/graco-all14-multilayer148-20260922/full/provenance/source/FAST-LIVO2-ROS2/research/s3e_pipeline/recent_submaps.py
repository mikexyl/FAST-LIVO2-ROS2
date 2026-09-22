"""Prepare native completed submaps for existing MapClosures/PCM/CBS workers."""
import argparse
import hashlib
import json
from pathlib import Path
import zlib
import time
import numpy as np
from .artifacts import canonical, read_jsonl, write_jsonl
from .backends import pack_array
from .geometry import voxel_downsample


def prepare(source, output, config, gravity_path=None):
    import s3e_mapclosures_native as native
    from .ellipsoid_cuda import SurfaceSampler
    source, output = Path(source), Path(output)
    manifest=json.loads((source/'manifest.json').read_text())
    if not manifest['complete'] or hashlib.sha256((source/'index.jsonl').read_bytes()).hexdigest()!=manifest['index_sha256']:
        raise ValueError('Unverified submap index')
    sidecar = json.loads(Path(gravity_path).read_text()) if gravity_path is not None else None
    if sidecar is not None and sidecar['source_index_sha256'] != manifest['index_sha256']:
        raise ValueError('Gravity reconstruction does not match submap index')
    alignment = config['backend']['mapclosures'].get('projection_alignment', 'gravity')
    if alignment not in ('gravity', 'local_ground') or (sidecar is not None and alignment != 'gravity'):
        raise ValueError('Invalid submap projection alignment')
    source_rows=read_jsonl(source/'index.jsonl')
    is_area=any(row.get('strategy')=='area' for row in source_rows)
    if is_area and any(row.get('strategy')!='area' for row in source_rows):
        raise ValueError('Do not mix accumulated area maps and recent submaps')
    if any(row.get('strategy') in ('coverage','area') for row in source_rows):
        backend=config['backend']; evidence=backend.get('evidence',{}); registration=backend.get('registration',{})
        if (evidence.get('sampling')!='fixed' or registration.get('sampling')!='fixed' or
                not 0<evidence.get('voxel_m',0)<=registration.get('voxel_m',0)):
            raise ValueError('Coverage submaps require fixed-resolution evidence and registration; use coverage_rollout.yaml')
        if is_area and (evidence.get('max_range_m',80.) is not None or registration.get('max_range_m',80.) is not None):
            raise ValueError('Area snapshots must not be cropped by a 3D range filter; use area_rollout.yaml')
    if is_area and alignment!='gravity':raise ValueError('Area maps require gravity-horizontal projection')
    output.mkdir(parents=True,exist_ok=False)
    store=output/'store'; descriptors=output/'ellipsoid'
    store.mkdir(); descriptors.mkdir()
    mc=config['backend']['mapclosures']; rows=[]; timings=[]; started=time.monotonic()
    sampler=None if is_area else SurfaceSampler()
    if is_area:
        from .area_maps import AreaHistoryAudit,render_area_surface
        audit=AreaHistoryAudit()
    try:
        for row in source_rows:
            if not row['complete'] or not row['retrievable']:continue
            path=source/row['payload']
            if path.parent!=source or hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
                raise ValueError('Snapshot payload hash mismatch')
            if row['available_ns']<row['last_member_ns'] or row['stamp_ns']>row['available_ns']:
                raise ValueError('Noncausal submap')
            frame_start=time.monotonic();basis_diagnostic=None;sampling=None;multilayer=None
            if alignment == 'gravity':
                from .gravity_bev import submap_gravity
                ground, projection = submap_gravity(row, sidecar)
            else:
                ground, projection = None, dict(source='legacy native local-ground fit')
            key=len(rows)
            with np.load(path,allow_pickle=False) as data:
                points=data['points']; e=data['ellipsoids']
                if is_area:
                    audit.add(row,data)
                    cloud=points.copy() # Preserve every selected native map representative.
                else:
                    cloud=voxel_downsample(points,.25)
                    cloud=cloud[np.linalg.norm(cloud,axis=1)<=80.]
                np.savez_compressed(store/f'{key:06d}.npz',cloud=cloud,scan=cloud)
                basis=e[:,6:].reshape(-1,3,3)
                engine=native.MapClosures(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
                surface=np.empty((0,3))
                if len(e):
                    if config.get('ellipsoid_basis_roundoff_tolerance') is not None:
                        from .ellipsoid_bev import normalize_exported_basis
                        basis,basis_diagnostic=normalize_exported_basis(basis,config['ellipsoid_basis_roundoff_tolerance'])
                    surface,sampling=(render_area_surface(e[:,:3],e[:,3:6],basis,ground,row['area_radius_m'])
                        if is_area else sampler.render(e[:,:3],e[:,3:6],basis))
                    features=engine.describe(surface) if ground is None else engine.describe(surface, ground=ground)
                else:
                    features=dict(ground=np.eye(4) if ground is None else ground,
                                  xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8))
                if mc.get('multilayer',{}).get('enabled',False):
                    if alignment!='gravity':raise ValueError('Multilayer BEVs require gravity-horizontal projection')
                    from .multilayer_mapclosures import describe_layers
                    multilayer=describe_layers(engine,points,surface,features['ground'])
            packed=dict(mapclosures={k:pack_array(features[k]) for k in ('ground','xy','bits')},
                        mapclosures_features=len(features['xy']),representation='ellipsoid',
                        submap_id=row['submap_id'],member_scan_ids=row['member_scan_ids'],payload_sha256=row['sha256'],
                        projection_alignment=projection)
            if multilayer is not None:packed['mapclosures_multilayer']=multilayer
            (descriptors/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(packed)))
            item=dict(row,keyframe_id=key,T_world_body=np.asarray(row['T_world_imu']).reshape(4,4).tolist(),
                      body_frame=row['robot_id']+'/imu',world_frame=row['robot_id']+'/odom_ellipselio',
                      cloud_frame=row['robot_id']+'/imu',image_available=False,camera=None,
                      submap_start_ns=row['begin_ns'],submap_end_ns=row['stamp_ns'],
                      submap_points=len(cloud),geometry_preprocessing=('causal accumulated area map in snapshot IMU frame'
                        if is_area else 'causal completed native submap in anchor IMU frame'))
            rows.append(item)
            timings.append(dict(keyframe_id=key,submap_id=row['submap_id'],available_ns=row['available_ns'],
                                runtime_s=time.monotonic()-frame_start,ellipsoids=len(e),features=len(features['xy']),
                                evidence_points=len(cloud),sampling=sampling,basis_roundoff=basis_diagnostic,
                                projection_alignment=projection,
                                multilayer=None if multilayer is None else {k:v for k,v in multilayer.items() if k!='layers'}))
        write_jsonl(store/'keyframes.jsonl',rows)
        write_jsonl(output/'timings.jsonl',timings)
        (output/'summary.json').write_text(json.dumps(dict(submaps=len(rows),wall_s=time.monotonic()-started,
            source_index_sha256=manifest['index_sha256'],source=str(source),ground_truth_used=False,
            projection_alignment=alignment,
            gravity_sidecar_sha256=hashlib.sha256(Path(gravity_path).read_bytes()).hexdigest() if sidecar else None),indent=2)+'\n')
    finally:
        if sampler is not None:sampler.close()
    if not rows:raise ValueError('No completed retrievable submaps')
    return rows


if __name__=='__main__':
    import yaml
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--gravity-sidecar',type=Path)
    args=parser.parse_args()
    prepare(args.source,args.output,yaml.safe_load(args.config.read_text()),args.gravity_sidecar)

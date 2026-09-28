"""Opt-in primitive-only snapshot preparation; never requests an NPZ point array."""
import hashlib
import time
import zlib
import numpy as np
from .artifacts import canonical,file_hash
from .backends import pack_array
from .ellipsoid_registration import validate, bounded_ellipsoids
from .ellipsoid_raster import rasterize, terrain_from_ellipsoids
from .multilayer_mapclosures import NAMES, VERSION


def evidence_policy(options):
    return dict(voxel_m=float(options.get('ellipsoid_voxel_m', .4)),
                max_count=int(options.get('ellipsoid_max_count', 50000)))


def prepared_evidence(data, row, options):
    """A prepared immutable subset is shared verbatim by verification and CBS."""
    policy = evidence_policy(options)
    if row.get('ellipsoid_evidence_policy') != policy:
        raise ValueError('Ellipsoid evidence preprocessing differs from preparation')
    evidence = validate(data['ellipsoids'])
    if len(evidence)>policy['max_count'] or hashlib.sha256(evidence.astype('<f8').tobytes()).hexdigest()!=row['ellipsoid_evidence_sha256']:
        raise ValueError('Prepared ellipsoid evidence hash/count mismatch')
    return evidence


def prepare_row(source, row, key, store, descriptors, config, *, sidecar=None, **unused):
    import s3e_mapclosures_native as native
    from .gravity_bev import submap_gravity
    start=time.perf_counter(); backend=config['backend']; mc=backend['mapclosures']
    if backend['registration'].get('method')!='ellipsoid':raise ValueError('Ellipsoid backend requires ellipsoid verification')
    if row.get('strategy')!='area' or mc.get('projection_alignment','gravity')!='gravity':
        raise ValueError('Ellipsoid backend requires gravity-aligned accumulated area snapshots')
    renderer=mc.get('renderer','sampled_surface')
    if renderer not in ('sampled_surface','geometric_coverage'):raise ValueError('Unknown ellipsoid renderer')
    path=source/row['payload']
    if path.parent!=source or file_hash(path)!=row['sha256']:
        raise ValueError('Snapshot payload hash mismatch')
    if row['available_ns']<row['last_member_ns'] or row['stamp_ns']>row['available_ns']:
        raise ValueError('Noncausal snapshot')
    ground,projection=submap_gravity(row,sidecar)
    with np.load(path,allow_pickle=False) as data:
        e=np.array(data['ellipsoids'],dtype=np.float64,copy=True)
    basis_diagnostic=None
    if config.get('ellipsoid_basis_roundoff_tolerance') is not None and len(e):
        from .ellipsoid_bev import normalize_exported_basis
        basis,basis_diagnostic=normalize_exported_basis(e[:,6:].reshape(-1,3,3),config['ellipsoid_basis_roundoff_tolerance'])
        e[:,6:]=basis.reshape(-1,9)
    e=validate(e);policy=evidence_policy(backend['registration'])
    evidence=bounded_ellipsoids(e,policy['voxel_m'],policy['max_count'])
    np.savez_compressed(store/f'{key:06d}.npz',ellipsoids=evidence)
    coefficients,terrain=terrain_from_ellipsoids(e,ground)
    layers_enabled=mc.get('multilayer',{}).get('enabled',False)
    engine=native.MapClosures(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    empty=lambda:dict(ground=ground,xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8))
    packets={name:empty() for name in ('fullheight',*NAMES)}
    rendering={};images={};origins={};feature_start=0.;feature_s=0.
    if renderer=='geometric_coverage':
        result=rasterize(e,ground,row['area_radius_m'],mc['density_map_resolution'],mc['density_threshold'],
                         coefficients if layers_enabled else None,mc.get('raster_device','cpu'))
        images=result['images'];rendering=result['diagnostics']
        feature_start=time.perf_counter()
        for name,image in images.items():
            packets[name]=engine.describe_image(image,result['lower_bound'],result['resolution'],ground)
        feature_s=time.perf_counter()-feature_start
        origins={name+'_lower_bound':result['lower_bound'] for name in images}
    else:
        from .area_maps import render_area_surface
        from .multilayer_bev import relative_heights,layer_masks
        t=time.perf_counter()
        surface,rendering=(render_area_surface(e[:,:3],e[:,3:6],e[:,6:].reshape(-1,3,3),ground,row['area_radius_m'])
                           if len(e) else (np.empty((0,3)),{}))
        rendering=dict(rendering or {},total_s=time.perf_counter()-t)
        feature_start=time.perf_counter()
        if len(surface)>=30:packets['fullheight']=engine.describe(surface,ground=ground,include_image=True)
        if layers_enabled and coefficients is not None:
            level=surface@ground[:3,:3].T+ground[:3,3]
            for name,mask in layer_masks(relative_heights(level,coefficients)).items():
                if mask.sum()>=30:packets[name]=engine.describe(surface[mask],ground=ground,include_image=True)
        feature_s=time.perf_counter()-feature_start
        rendering['density_generation_s']=sum(p.get('density_generation_s',0.) for p in packets.values())
        rendering['surface_generation_s']=rendering['total_s']
        rendering['total_s']+=rendering['density_generation_s']
        for name,packet in packets.items():
            if 'image' in packet:
                images[name]=packet['image'];origins[name+'_lower_bound']=packet['lower_bound']
    # Small images and ORB packets survive even when terrain is unsupported.
    inspection=descriptors.parent/'rendering';inspection.mkdir(exist_ok=True)
    np.savez_compressed(inspection/f'{key:06d}.npz',**images,**origins)
    pack=lambda p:{k:pack_array(p[k]) for k in ('ground','xy','bits')}
    packed=dict(mapclosures=pack(packets['fullheight']),mapclosures_features=len(packets['fullheight']['xy']),
                representation='ellipsoid',renderer=renderer,submap_id=row['submap_id'],
                member_scan_ids=row['member_scan_ids'],payload_sha256=row['sha256'],projection_alignment=projection)
    if layers_enabled:
        packed['mapclosures_multilayer']=dict(version=VERSION,terrain=terrain,
            feature_counts={name:len(packets[name]['xy']) for name in NAMES},
            layers={name:pack(packets[name]) for name in NAMES})
    (descriptors/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(packed)))
    item=dict(row,keyframe_id=key,T_world_body=np.asarray(row['T_world_imu']).reshape(4,4).tolist(),
        body_frame=row['robot_id']+'/imu',world_frame=row['robot_id']+'/odom_ellipselio',
        cloud_frame=row['robot_id']+'/imu',image_available=False,camera=None,
        submap_start_ns=row['begin_ns'],submap_end_ns=row['stamp_ns'],
        submap_points=0,ellipsoid_count=len(evidence),ellipsoid_evidence_policy=policy,
        ellipsoid_evidence_sha256=hashlib.sha256(evidence.astype('<f8').tobytes()).hexdigest(),
        geometry_preprocessing='causal accumulated area map in snapshot IMU frame')
    timing=dict(keyframe_id=key,submap_id=row['submap_id'],available_ns=row['available_ns'],
        runtime_s=time.perf_counter()-start,ellipsoids=len(e),evidence_ellipsoids=len(evidence),
        renderer=renderer,rendering=rendering,descriptor_stage_s=feature_s,
        feature_extraction_s=sum(p.get('feature_extraction_s',0.) for p in packets.values()),
        native_feature_timing_available=all('feature_extraction_s' in p for p in packets.values() if len(p['xy'])),
        basis_roundoff=basis_diagnostic,terrain=terrain,
        features={name:len(p['xy']) for name,p in packets.items()},point_arrays_loaded=False)
    return item,timing

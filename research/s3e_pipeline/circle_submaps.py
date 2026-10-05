"""Opt-in, sealed spatial footprints over a causal union of retained area observations.

Placement is incremental; geometry is frozen once at end of capture. This is a
staged integration experiment, not an online geometry-publication policy.
"""
from dataclasses import asdict
from pathlib import Path
import time
import zlib

import numpy as np

from .artifacts import canonical, digest, file_hash, read_jsonl, write_json, write_jsonl
from .backends import pack_array
from .circle_cover import Circle, CircleCoverPlanner, placement_metrics
from .circle_cover_replay import PreparedObservations
from .gravity_bev import submap_gravity
from .multilayer_mapclosures import describe_layers

SCHEMA = 6
PREPROCESSING = 'sealed fixed spatial footprint; native representatives in creation-anchor IMU frame'


def id_ranges(ids):
    ids = np.asarray(ids, dtype=np.int64)
    if len(ids) == 0: return []
    if np.any(np.diff(ids) <= 0): raise ValueError('Unordered or duplicate point IDs')
    cuts = np.r_[0, np.flatnonzero(np.diff(ids) != 1)+1, len(ids)]
    return [[int(ids[a]), int(ids[b-1])+1] for a, b in zip(cuts[:-1], cuts[1:])]


def validate_row(row):
    # Sensor time identifies immutable geometry; wall time governs live delivery.
    live = row.get('availability_clock') == 'unix_wall_descriptor_publication'
    available = row.get('native_available_ns', -1) if live else row.get('available_ns', -1)
    if live and (row.get('geometry_publication_policy') not in ('freeze_at_creation','mature_then_freeze') or
                 row.get('available_ns', -1) < row.get('snapshot_received_wall_ns', 0)):
        raise ValueError('Invalid live spatial publication')
    if row.get('geometry_publication_policy')=='mature_then_freeze':
        if (row.get('sealed_update',-1)<row['created_update'] or
            available<row['created_available_ns'] or
            row.get('maturation',{}).get('reason') not in ('stable','deadline','end_of_stream') or
            (row['maturation']['reason']=='stable' and not row['maturation'].get('quality_ready'))):
            raise ValueError('Invalid mature spatial publication')
    if (row.get('schema_version') != SCHEMA or row.get('strategy') != 'fixed_spatial' or
        row.get('frame') != 'creation_anchor_imu' or row.get('cloud_frame') != row.get('body_frame') or
        not row.get('body_frame') or row.get('geometry_preprocessing') != PREPROCESSING or
        available < max(row['stamp_ns'], row['last_member_ns']) or
        available != row['seal_ns'] or not row.get('complete') or
        row['placement_mode'] not in ('circle', 'trajectory') or row['footprint_radius_m'] <= 0):
        raise ValueError('Invalid sealed spatial endpoint/frame/availability')
    T = np.asarray(row['T_world_body'], dtype=float)
    frame = np.asarray(row['T_horizontal_map'], dtype=float)
    CircleCoverPlanner._validate_frame(T); CircleCoverPlanner._validate_frame(frame)
    if not np.array_equal(T, np.asarray(row['T_world_imu']).reshape(4,4)):
        raise ValueError('Body/IMU anchor mismatch')
    if not np.isfinite(row['footprint_center_xy']).all(): raise ValueError('Invalid footprint')
    ranges = row['geometry_id_ranges']
    if any(a < 0 or b <= a or (k and a <= ranges[k-1][1]) for k, (a, b) in enumerate(ranges)):
        raise ValueError('Invalid point-ID ranges')
    if sum(b-a for a, b in ranges) != row['geometry_count']: raise ValueError('Geometry membership mismatch')
    submap_gravity(row)


def validate_geometry(row, cloud, ids):
    validate_row(row)
    cloud=np.asarray(cloud);ids=np.asarray(ids)
    if cloud.shape!=(row['geometry_count'],3) or not np.isfinite(cloud).all() or id_ranges(ids)!=row['geometry_id_ranges']:
        raise ValueError('Payload/member-ID mismatch')
    T=np.asarray(row['T_world_body']);frame=np.asarray(row['T_horizontal_map'])
    world=cloud@T[:3,:3].T+T[:3,3]
    xy=(world@frame[:3,:3].T+frame[:3,3])[:,:2]
    if np.any(np.linalg.norm(xy-row['footprint_center_xy'],axis=1)>row['footprint_radius_m']+1e-3):
        raise ValueError('Geometry outside fixed footprint')


def build(store, output, config, mode):
    """Same growing observations, final seal and radius for both placement modes."""
    if mode not in ('circle', 'trajectory'): raise ValueError('Unknown placement mode')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    (output/'store').mkdir(); observations = PreparedObservations(store)
    planner = CircleCoverPlanner(config, observations.frame)
    footprints = []; anchors = []; history = []; placement_s = 0.
    for update_id, (row, new) in enumerate(observations):
        started = time.perf_counter()
        if mode == 'circle':
            update = planner.update(new, update_id)
            footprints = list(planner.circles)
            anchors.extend([row]*len(update.new_circles))
            diagnostics = update.diagnostics
        else:
            # Keep the original sensor-time snapshot cadence, changing only R.
            p = np.asarray(row['T_world_body'])[:3, 3]
            xy = (observations.frame[:3, :3]@p+observations.frame[:3, 3])[:2]
            footprints.append(Circle(len(footprints), tuple(xy), config.radius_m, update_id)); anchors.append(row)
            cells = np.unique(np.floor(((observations.world@observations.frame[:3, :3].T+
                observations.frame[:3, 3])[:, :2]-config.grid_origin_xy)/config.cell_size_m).astype(np.int64), axis=0)
            diagnostics = placement_metrics((cells+.5)*config.cell_size_m+config.grid_origin_xy,
                [c.center_xy for c in footprints], config)
        elapsed = time.perf_counter()-started; placement_s += elapsed
        history.append(dict(update_id=update_id, available_ns=row['available_ns'],
            observed_points=len(observations.ids), circle_count=len(footprints), placement_s=elapsed,
            diagnostics=diagnostics))
    horizontal = observations.world@observations.frame[:3, :3].T+observations.frame[:3, 3]
    rows = []; memberships = np.zeros(len(horizontal), dtype=np.int32)
    seal = observations.rows[-1]['available_ns']
    source_hash = digest(dict(index=file_hash(observations.index), payloads=observations.payloads))
    for key, (circle, anchor) in enumerate(zip(footprints, anchors)):
        mask = np.sum((horizontal[:, :2]-circle.center_xy)**2, axis=1) <= config.radius_m**2
        memberships += mask
        T = np.asarray(anchor['T_world_body'])
        cloud = (observations.world[mask]-T[:3, 3])@T[:3, :3]
        path = output/'store'/f'{key:06d}.npz'
        np.savez_compressed(path, cloud=cloud.astype(np.float32), point_ids=observations.ids[mask])
        gravity = observations.rows[0]['gravity_world_m_s2']
        row = dict(schema_version=SCHEMA, strategy='fixed_spatial', placement_mode=mode,
            robot_id=observations.robot_id, submap_id=key, keyframe_id=key,
            stamp_ns=anchor['stamp_ns'], available_ns=seal, seal_ns=seal, last_member_ns=seal,
            begin_ns=observations.rows[0]['begin_ns'], end_ns=seal,
            created_update=circle.created_update, created_available_ns=anchor['available_ns'],
            anchor_source_keyframe=anchor['keyframe_id'], T_world_body=T.tolist(), T_world_imu=T.ravel().tolist(),
            T_horizontal_map=observations.frame.tolist(), footprint_center_xy=list(circle.center_xy),
            footprint_radius_m=config.radius_m, frame='creation_anchor_imu',
            body_frame=anchor['body_frame'], cloud_frame=anchor['body_frame'], world_frame=observations.world_frame,
            gravity_world_m_s2=list(gravity), gravity_imu_m_s2=(T[:3, :3].T@gravity).tolist(),
            gravity_source='fixed first retained snapshot native gravity; transformed to creation anchor',
            geometry_count=len(cloud), geometry_id_ranges=id_ranges(observations.ids[mask]),
            member_scan_ids=[], membership_semantics='stable native point IDs; scan membership is unavailable',
            geometry_source='growing union of retained native area snapshots; not all original scans or raw points',
            geometry_preprocessing=PREPROCESSING, image_available=False,
            retrievable=len(cloud)>=30, complete=True, sha256=file_hash(path),
            source_observations_hash=source_hash, config_hash=digest(asdict(config)))
        validate_geometry(row, cloud.astype(np.float32), observations.ids[mask]); rows.append(row)
    write_jsonl(output/'store/keyframes.jsonl', rows)
    np.savez_compressed(output/'union.npz', world=observations.world.astype(np.float32), point_ids=observations.ids)
    cells = np.unique(np.floor((horizontal[:, :2]-config.grid_origin_xy)/config.cell_size_m).astype(np.int64), axis=0)
    metrics = placement_metrics((cells+.5)*config.cell_size_m+config.grid_origin_xy,
        [c.center_xy for c in footprints], config)
    summary = dict(mode=mode, robot_id=observations.robot_id, config=asdict(config),
        footprints=len(footprints), observations=len(observations.rows), points=len(horizontal),
        actual_point_coverage=float(np.mean(memberships>0)), mean_point_membership=float(np.mean(memberships)),
        unassigned_points=int(np.sum(memberships==0)), placement_s=placement_s, metrics=metrics,
        source_hash=source_hash, source_index=str(observations.index), source_payloads=observations.payloads,
        geometry_policy='one immutable end-of-capture crop per fixed footprint; no online latency claim')
    write_json(output/'placement.json', summary); write_jsonl(output/'placement-history.jsonl', history)
    if mode == 'circle': planner.save(output/'planner-state.json')
    return summary


def prepare(output, backend):
    """Native point-density BEVs and unchanged terrain-relative multilayer ORB."""
    import s3e_mapclosures_native as native
    from PIL import Image
    output = Path(output); (output/'descriptors').mkdir(); (output/'images').mkdir()
    opts = backend['mapclosures']
    if opts['representation'] != 'point_cloud' or opts['upstream_commit'] != native.upstream_commit:
        raise ValueError('Point-cloud backend/native revision mismatch')
    engine = native.MapClosures(opts['density_map_resolution'], opts['density_threshold'], opts['hamming_distance_threshold'])
    timings = []
    for row in read_jsonl(output/'store/keyframes.jsonl'):
        validate_row(row)
        path = output/'store'/f"{row['keyframe_id']:06d}.npz"
        if file_hash(path) != row['sha256']: raise ValueError('Modified geometry')
        with np.load(path, allow_pickle=False) as f: cloud = np.asarray(f['cloud'], dtype=float)
        if not row['retrievable']: continue
        ground, alignment = submap_gravity(row); started = time.perf_counter()
        full = engine.describe(cloud, ground=ground, include_image=True)
        full_s = time.perf_counter()-started; started = time.perf_counter()
        layers = describe_layers(engine, cloud, cloud, ground)
        layers_s = time.perf_counter()-started
        descriptor = dict(mapclosures={k:pack_array(full[k]) for k in ('ground','xy','bits')},
            mapclosures_features=len(full['xy']), mapclosures_multilayer=layers,
            representation='point_cloud', renderer='point_cloud', projection_alignment=alignment,
            submap_id=row['submap_id'], member_scan_ids=[], payload_sha256=row['sha256'])
        (output/'descriptors'/f"{row['keyframe_id']:06d}.json.zlib").write_bytes(zlib.compress(canonical(descriptor)))
        Image.fromarray(np.asarray(full['image'], dtype=np.uint8)).save(output/'images'/f"{row['keyframe_id']:06d}.png")
        timings.append(dict(keyframe_id=row['keyframe_id'], full_s=full_s, layers_s=layers_s,
            full_features=len(full['xy']), layer_features=layers['feature_counts'], terrain=layers['terrain'],
            native_timing={k:full[k] for k in full if k.endswith('_s')}))
    write_jsonl(output/'preparation.jsonl', timings)
    return dict(full_s=sum(t['full_s'] for t in timings), layers_s=sum(t['layers_s'] for t in timings))

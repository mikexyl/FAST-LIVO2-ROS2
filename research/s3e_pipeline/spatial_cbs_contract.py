"""Immutable fixed-footprint provenance shared by verification and CBS transport."""
import numpy as np
from .artifacts import digest, file_hash
from .circle_submaps import validate_row, validate_geometry
from .registration import bounded_cloud


def endpoint(row):
    validate_row(row)
    voxel = row.get('cbs_evidence_voxel_m')
    if not isinstance(voxel, (int,float)) or not np.isfinite(voxel) or voxel <= 0:
        raise ValueError('Missing fixed spatial verification preprocessing')
    return dict(robot=row['robot_id'], key=row['keyframe_id'], stamp_ns=row['stamp_ns'],
        available_ns=row['available_ns'], cloud_frame=row['cloud_frame'],
        source_sha256=row['sha256'], row_sha256=digest(row), evidence_voxel_m=voxel)


def prepared_cloud(path, data, row):
    endpoint(row)
    if file_hash(path) != row['sha256']:
        raise ValueError('Modified fixed spatial geometry')
    validate_geometry(row, data['cloud'], data['point_ids'])
    # Centralized verification exported this same 0.4 m cloud. Registration
    # performs its normal second-stage voxel/cap on identical inputs.
    return bounded_cloud(data['cloud'], row['cbs_evidence_voxel_m'], None, None)[0]

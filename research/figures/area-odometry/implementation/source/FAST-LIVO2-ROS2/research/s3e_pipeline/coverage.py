"""Audit observed-return footprints, independent of the native scheduler."""
import numpy as np


def validate_coverage(row, data):
    if row.get('strategy') != 'coverage':
        raise ValueError('Invalid coverage strategy')
    size = float(row['coverage_cell_size_m'])
    origin = np.asarray(row['coverage_origin_world'], dtype=float)
    axes = np.array([row['coverage_x_world'], row['coverage_y_world']], dtype=float)
    if not np.isfinite(size) or size <= 0 or origin.shape != (3,) or axes.shape != (2, 3):
        raise ValueError('Invalid coverage grid')
    if not np.isfinite(origin).all() or not np.isfinite(axes).all() or not np.allclose(axes @ axes.T, np.eye(2), atol=1e-8):
        raise ValueError('Invalid coverage frame')
    cells = data['coverage_cells']
    if cells.shape != (row['coverage_cell_count'], 3) or cells.dtype.kind not in 'iu':
        raise ValueError('Invalid coverage cell payload')
    saved = {(int(x), int(y)): int(first) for x, y, first in cells}
    if len(saved) != len(cells) or not set(saved.values()).issubset(row['member_scan_ids']):
        raise ValueError('Invalid coverage cell membership')
    counts = row['member_point_counts']
    if len(counts) != len(row['member_scan_ids']) or any(type(x) is not int or x < 0 for x in counts) or sum(counts) != len(data['points']):
        raise ValueError('Coverage member point counts mismatch')
    T = np.asarray(row['T_world_imu']).reshape(4, 4)
    # Native coordinates were float32 before the anchor transform. Allow only
    # cell-boundary ambiguity from that serialization, never arbitrary missing cells.
    world = data['points'].astype(float) @ T[:3, :3].T + T[:3, 3]
    tol = 16*np.finfo(np.float32).eps*max(1., np.max(np.abs(world), initial=0), np.max(np.abs(T[:3, 3])))/size
    possible_first = {}; certain_first = {}; offset = 0
    for scan, count in zip(row['member_scan_ids'], counts):
        xy = (world[offset:offset+count]-origin) @ axes.T / size
        offset += count
        lo, hi = np.floor(xy-tol).astype(np.int64), np.floor(xy+tol).astype(np.int64)
        certain = lo[np.all(lo == hi, axis=1)]
        for c in np.unique(certain, axis=0):
            certain_first.setdefault(tuple(c), scan)
        for xx, yy in ((lo[:, 0], lo[:, 1]), (lo[:, 0], hi[:, 1]), (hi[:, 0], lo[:, 1]), (hi[:, 0], hi[:, 1])):
            for c in np.unique(np.column_stack([xx, yy]), axis=0):
                possible_first.setdefault(tuple(c), scan)
    if not set(certain_first).issubset(saved) or not set(saved).issubset(possible_first):
        raise ValueError('Coverage cells do not match observed geometry')
    if any(first < possible_first[c] or first > certain_first.get(c, first) for c, first in saved.items()):
        raise ValueError('Coverage first-observation membership mismatch')
    seed = sum(first == min(saved.values()) for first in saved.values()) if saved else 0
    if seed != row['coverage_seed_cells']:
        raise ValueError('Coverage seed mismatch')
    if not np.isclose(len(saved)*size**2, row['occupied_area_m2']) or not np.isclose((len(saved)-seed)*size**2, row['new_area_m2']):
        raise ValueError('Coverage area mismatch')
    if row['finish_reason'] == 'coverage' and (row['occupied_area_m2'] < row['target_area_m2'] or row['new_area_m2'] < row['required_new_area_m2']):
        raise ValueError('Premature coverage handover')
    return saved


def validate_overlaps(rows, cells):
    indexed = {r['submap_id']: r for r in rows if r['schema_version'] == 4}
    for row in indexed.values():
        other = row['overlap_successor_id']
        if other < 0:
            if row['complete'] and not row['finish_reason'].endswith('_recovery'):
                raise ValueError('Missing coverage successor at handover')
            if row['shared_area_m2'] != 0 or row['overlap_ratio'] != 0:
                raise ValueError('Coverage overlap without successor')
            continue
        if other not in indexed:
            if row['shared_area_m2'] == 0 and row['overlap_ratio'] == 0:
                continue  # Empty successor has no snapshot.
            raise ValueError('Missing coverage successor')
        successor = indexed[other]
        for field in ('coverage_origin_world', 'coverage_x_world', 'coverage_y_world', 'coverage_cell_size_m'):
            if row[field] != successor[field]:
                raise ValueError('Coverage grids cannot be intersected')
        a = set(cells[row['submap_id']])
        b = {c for c, first in cells[other].items() if first <= row['member_scan_ids'][-1]}
        shared = len(a & b); ratio = shared/max(len(a), len(b)) if a or b else 0
        if not np.isclose(shared*row['coverage_cell_size_m']**2, row['shared_area_m2']) or not np.isclose(ratio, row['overlap_ratio']):
            raise ValueError('Coverage overlap mismatch')
        if row['complete'] and not row['finish_reason'].endswith('_recovery'):
            if row['shared_area_m2'] < row['min_shared_area_m2'] or ratio < row['min_overlap_ratio']:
                raise ValueError('Unsupported coverage overlap at handover')

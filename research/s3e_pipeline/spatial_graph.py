"""Append-only spatial map-consistency factors, not independent loop evidence.

Only sealed local crops are inputs. Shared native point IDs establish support;
the measurement preserves their original map relationship. No ICP, peer reads,
ground truth, or additional GPU registration factor is involved.
"""
from collections import OrderedDict
from dataclasses import asdict, dataclass
from pathlib import Path
import time

import numpy as np

from .artifacts import digest, file_hash
from .circle_submaps import validate_geometry, validate_row
from .geometry import adjoint, constraint, inv, pose, transform

# cbs_ros/pose_graph_factor.h: explicitly local, bypasses PCM loop proposals
# and the registration exchange. An older native bridge rejects this type.
MAP_CONSISTENCY_EDGE_TYPE = 100


@dataclass(frozen=True)
class SpatialGraphConfig:
    enabled: bool = False
    max_degree: int = 3
    max_new_edges: int = 2
    min_shared_points: int = 100
    min_shared_fraction: float = .10  # fraction of the smaller sealed crop
    min_second_axis_std_m: float = 1.
    max_shared_discrepancy_m: float = .005
    information_fraction: float = .25
    support_sample_points: int = 2048
    cache_clouds: int = 4

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('spatial_neighbors.enabled must be Boolean')
        for name in ('max_degree', 'max_new_edges', 'min_shared_points',
                     'support_sample_points', 'cache_clouds'):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f'Invalid spatial graph {name}')
        if self.max_new_edges > self.max_degree or self.support_sample_points < 3:
            raise ValueError('Invalid spatial degree or support sample budget')
        for name in ('min_shared_fraction', 'information_fraction'):
            if not np.isfinite(getattr(self, name)) or not 0 < getattr(self, name) <= 1:
                raise ValueError(f'Invalid spatial graph {name}')
        for name in ('min_second_axis_std_m', 'max_shared_discrepancy_m'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'Invalid spatial graph {name}')


def shared_count(a, b):
    """Intersect compressed ID ranges without expanding either crop."""
    i = j = count = 0
    while i < len(a) and j < len(b):
        count += max(0, min(a[i][1], b[j][1]) - max(a[i][0], b[j][0]))
        if a[i][1] <= b[j][1]:
            i += 1
        else:
            j += 1
    return count


def path_covariance(rows, i, j, edge_covariance):
    """Compose the raw chain's right-tangent noise into endpoint j's frame."""
    end = pose(rows[j]['T_world_body'])
    covariance = np.zeros((6, 6))
    for k in range(i, j):
        suffix = inv(pose(rows[k+1]['T_world_body'])) @ end
        A = adjoint(inv(suffix))
        covariance += A @ edge_covariance @ A.T
    return (covariance + covariance.T) / 2


class SpatialGraph:
    def __init__(self, robot, store, pgo):
        self.robot = robot
        self.store = Path(store)
        self.cfg = SpatialGraphConfig(**pgo.get('spatial_neighbors', {}))
        sigma = np.array([np.deg2rad(pgo['odometry_rotation_sigma_deg'])]*3 +
                         [pgo['odometry_translation_sigma_m']]*3)
        if not np.isfinite(sigma).all() or np.any(sigma <= 0):
            raise ValueError('Invalid odometry covariance')
        self.edge_covariance = np.diag(sigma**2)
        self.hashes = []
        self.rows = []
        self.edges = []
        self.degree = []
        self.decisions = []
        self.cache = OrderedDict()
        self.peak_cached_cloud_bytes = 0
        self.payload_reads = 0
        self.elapsed_s = 0.

    def cloud(self, row):
        key = row['keyframe_id']
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        path = self.store / f'{key:06d}.npz'
        if file_hash(path) != row['sha256']:
            raise ValueError('Modified spatial support geometry')
        with np.load(path, allow_pickle=False) as data:
            xyz = np.asarray(data['cloud'], dtype=float)
            ids = np.asarray(data['point_ids'])
        validate_geometry(row, xyz, ids)
        self.payload_reads += 1
        while len(self.cache) >= self.cfg.cache_clouds:
            self.cache.popitem(last=False)
        self.cache[key] = (xyz, ids)
        self.peak_cached_cloud_bytes = max(self.peak_cached_cloud_bytes,
            sum(x.nbytes + ids.nbytes for x, ids in self.cache.values()))
        return xyz, ids

    def support(self, a, b, count):
        xa, ia = self.cloud(a)
        xb, ib = self.cloud(b)
        ids, pa, pb = np.intersect1d(ia, ib, assume_unique=True, return_indices=True)
        if len(ids) != count:
            raise ValueError('Shared point membership disagrees with sealed metadata')
        relative = inv(pose(a['T_world_body'])) @ pose(b['T_world_body'])
        # Validate ALL common points; bound the temporary transformed array.
        discrepancy = 0.
        for first in range(0, count, self.cfg.support_sample_points):
            s = slice(first, first+self.cfg.support_sample_points)
            delta = xa[pa[s]] - transform(relative, xb[pb[s]])
            discrepancy = max(discrepancy, float(np.linalg.norm(delta, axis=1).max()))
        if discrepancy > self.cfg.max_shared_discrepancy_m:
            raise ValueError('Same native point ID has inconsistent map coordinates')
        sample = np.linspace(0, count-1, min(count, self.cfg.support_sample_points), dtype=int)
        points = xa[pa[sample]]
        centered = points - points.mean(axis=0)
        std = np.sqrt(np.maximum(0, np.linalg.eigvalsh(centered.T @ centered / len(points))))
        return dict(shared_discrepancy_max_m=discrepancy, shared_axis_std_m=std.tolist(),
                    support_samples=len(points), supported=bool(std[1] >= self.cfg.min_second_axis_std_m))

    def update(self, rows):
        start = time.perf_counter()
        hashes = [digest(row) for row in rows]
        if len(rows) < len(self.rows) or hashes[:len(self.hashes)] != self.hashes:
            raise ValueError('Changed spatial graph prefix')
        if not self.cfg.enabled:
            return []
        for j in range(len(self.rows), len(rows)):
            b = rows[j]
            validate_row(b)
            if b['robot_id'] != self.robot or b['keyframe_id'] != j:
                raise ValueError('Spatial graph requires consecutive owned submaps')
            if j and (b['available_ns'] < rows[j-1]['available_ns'] or
                      b['stamp_ns'] < rows[j-1]['stamp_ns']):
                raise ValueError('Spatial graph must follow causal publication order')
            if j and (not np.array_equal(b['T_horizontal_map'], rows[0]['T_horizontal_map']) or
                      b.get('world_frame') != rows[0].get('world_frame')):
                raise ValueError('Spatial graph horizontal/map frame changed')
            self.degree.append(0)
            candidates = []
            for i, a in enumerate(self.rows):
                # Adjacent nodes already have the unchanged odometry constraint.
                if i == j-1:
                    continue
                distance = float(np.linalg.norm(np.asarray(a['footprint_center_xy']) - b['footprint_center_xy']))
                if distance >= a['footprint_radius_m'] + b['footprint_radius_m']:
                    continue
                count = shared_count(a['geometry_id_ranges'], b['geometry_id_ranges'])
                fraction = count / max(1, min(a['geometry_count'], b['geometry_count']))
                record = dict(i=[self.robot, i], j=[self.robot, j], shared_points=count,
                    shared_fraction=fraction, center_distance_m=distance,
                    available_ns=max(a['available_ns'], b['available_ns']))
                if count < self.cfg.min_shared_points or fraction < self.cfg.min_shared_fraction:
                    self.decisions.append(dict(record, reason='insufficient_shared_geometry'))
                else:
                    candidates.append(((-fraction, -count, distance, i), record))
            for _, record in sorted(candidates, key=lambda item: item[0]):
                i = record['i'][1]
                if self.degree[j] >= self.cfg.max_new_edges or self.degree[i] >= self.cfg.max_degree:
                    self.decisions.append(dict(record, reason='degree_budget'))
                    continue
                quality = self.support(rows[i], b, record['shared_points'])
                record.update(quality)
                if not quality['supported']:
                    self.decisions.append(dict(record, reason='weak_shared_geometry'))
                    continue
                # Shared crops are correlated. This is a regularization budget,
                # never a covariance estimate from N supposedly independent points.
                covariance = path_covariance(rows, i, j, self.edge_covariance)
                weight = self.cfg.information_fraction / self.cfg.max_degree
                information = weight * np.linalg.inv(covariance)
                information = (information + information.T) / 2
                record.update(reason='accepted', source_hashes=[rows[i]['sha256'], b['sha256']],
                    information_weight=weight, path_edges=j-i,
                    evidence='shared native point IDs; correlated local map consistency')
                self.edges.append(constraint(record['i'], record['j'],
                    inv(pose(rows[i]['T_world_body'])) @ pose(b['T_world_body']), information,
                    kind='map_consistency', accepted=True, diagnostics=record))
                self.decisions.append(record)
                self.degree[i] += 1
                self.degree[j] += 1
            self.rows.append(b)
            self.hashes.append(hashes[j])
        self.elapsed_s += time.perf_counter() - start
        return list(self.edges)

    def diagnostics(self):
        return dict(schema_version=1, robot=self.robot, config=asdict(self.cfg),
            submaps=len(self.rows), map_consistency_factors=len(self.edges),
            additional_degree=self.degree, decisions=self.decisions,
            construction_s=self.elapsed_s, payload_reads=self.payload_reads,
            peak_cached_cloud_bytes=self.peak_cached_cloud_bytes,
            geometry_network_bytes=0,
            communication_scope='robot-local construction only; CBS belief traffic measured separately',
            uncertainty_policy='heuristic regularizer; correlated with the odometry backbone')

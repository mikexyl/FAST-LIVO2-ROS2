"""Standalone incremental disk cover of occupied map cells, independent of poses."""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp, minimize
from scipy.sparse import csc_matrix, csr_matrix, eye, hstack, vstack
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class CircleCoverConfig:
    radius_m: float = 80.
    cell_size_m: float = 1.
    grid_origin_xy: tuple = (0., 0.)
    coverage_target: float = .95
    overlap_target: float = .30
    overlap_tolerance: float = .10
    candidate_spacing_fraction: float = .25
    solver_time_limit_s: float = 10.
    refinement_max_iterations: int = 100

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key == 'grid_origin_xy':
                if np.asarray(value).shape != (2,) or not np.isfinite(value).all():
                    raise ValueError('Invalid fixed grid origin')
                object.__setattr__(self, key, tuple(map(float, value)))
                continue
            if isinstance(value, bool) or not np.isfinite(value):
                raise ValueError(f'Invalid {key}')
        if (self.radius_m <= 0 or self.cell_size_m <= 0 or
                not 0 < self.coverage_target <= 1 or
                not 0 <= self.overlap_target <= 1 or
                not 0 <= self.overlap_tolerance <= 1 or
                not 0 < self.candidate_spacing_fraction <= 1 or
                self.solver_time_limit_s <= 0 or
                not isinstance(self.refinement_max_iterations, int) or
                self.refinement_max_iterations <= 0):
            raise ValueError('Invalid circle-cover configuration')


@dataclass(frozen=True)
class Circle:
    id: int
    center_xy: tuple
    radius_m: float
    created_update: int


@dataclass(frozen=True)
class CoverUpdate:
    new_circles: tuple
    diagnostics: dict


def disk_overlap(distance, radius):
    """Intersection of equal disks / area of one disk; tangent disks share zero."""
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('Invalid radius')
    distance = np.asarray(distance, dtype=float)
    if not np.isfinite(distance).all() or np.any(distance < 0):
        raise ValueError('Invalid separation')
    x = np.clip(distance / (2*radius), 0, 1)
    return (2*np.arccos(x)-2*x*np.sqrt(np.maximum(0, 1-x*x))) / np.pi


def covered_mask(points_xy, centers_xy, radius):
    points = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    centers = np.asarray(centers_xy, dtype=float).reshape(-1, 2)
    if not len(centers):
        return np.zeros(len(points), dtype=bool)
    return cKDTree(centers).query(points, k=1)[0] <= radius + 1e-10*max(1., radius)


def candidate_centers(uncovered, spacing):
    """Deterministic farthest-point net, seeded by the lexicographically first cell."""
    uncovered = np.asarray(uncovered, dtype=float).reshape(-1, 2)
    if not len(uncovered):
        return uncovered.copy()
    # Sorted input makes point order and duplicate observations irrelevant.
    uncovered = np.unique(uncovered, axis=0)
    distance2 = np.full(len(uncovered), np.inf)
    chosen = []
    index = 0
    while True:
        center = uncovered[index]
        chosen.append(center)
        distance2 = np.minimum(distance2, np.sum((uncovered-center)**2, axis=1))
        index = int(np.argmax(distance2))
        if distance2[index] <= spacing*spacing + 1e-12:
            break
    return np.asarray(chosen)


def coverage_table(points, candidates, radius):
    tree = cKDTree(points)
    columns = tree.query_ball_point(candidates, radius+1e-10*max(1., radius), return_sorted=True)
    indptr = np.r_[0, np.cumsum([len(v) for v in columns])]
    indices = np.concatenate(columns).astype(np.int32) if indptr[-1] else np.empty(0, np.int32)
    return csc_matrix((np.ones(len(indices)), indices, indptr), shape=(len(points), len(candidates)))


def overlap_cost(new_centers, old_centers, radius, target):
    new = np.asarray(new_centers).reshape(-1, 2)
    old = np.asarray(old_centers).reshape(-1, 2)
    centers = np.vstack([old, new])
    if len(centers) < 2:
        return 0.
    distances = np.linalg.norm(new[:, None]-centers[None, :], axis=2)
    distances[np.arange(len(new)), len(old)+np.arange(len(new))] = np.inf
    nearest = distances.min(axis=1)
    intersects = nearest < 2*radius
    # No overlap requirement for an isolated footprint.
    return float(np.sum((disk_overlap(nearest[intersects], radius)-target)**2))


def placement_metrics(points_xy, centers_xy, config):
    points = np.asarray(points_xy).reshape(-1, 2)
    centers = np.asarray(centers_xy).reshape(-1, 2)
    tree = cKDTree(points) if len(points) else None
    members = [np.asarray(tree.query_ball_point(c, config.radius_m+1e-10*max(1., config.radius_m)), dtype=int)
               if tree else np.empty(0, dtype=int) for c in centers]
    memberships = np.zeros(len(points), dtype=np.int32)
    for indices in members:
        memberships[indices] += 1
    pairs = []
    nearest = []
    for i, a in enumerate(centers):
        other = [(float(np.linalg.norm(a-b)), j) for j, b in enumerate(centers) if i != j]
        distance, neighbor = min(other) if other else (np.inf, None)
        fraction = float(disk_overlap(distance, config.radius_m)) if distance < 2*config.radius_m else 0.
        nearest.append(dict(circle_id=i, neighbor_id=neighbor if fraction > 0 else None,
                            overlap_fraction=fraction, isolated=fraction == 0))
        for j in range(i+1, len(centers)):
            distance = float(np.linalg.norm(a-centers[j]))
            if distance >= 2*config.radius_m:
                continue
            shared = len(np.intersect1d(members[i], members[j], assume_unique=True))
            denominator = min(len(members[i]), len(members[j]))
            pairs.append(dict(i=i, j=j, center_distance_m=distance,
                              circle_overlap_fraction=float(disk_overlap(distance, config.radius_m)),
                              shared_cells=shared,
                              shared_occupied_fraction=shared/denominator if denominator else None))
    covered = int(np.count_nonzero(memberships))
    neighbor_values = [v['overlap_fraction'] for v in nearest if not v['isolated']]
    return dict(occupied_cells=len(points), covered_cells=covered,
                coverage=covered/len(points) if len(points) else 1., circle_count=len(centers),
                cell_memberships=int(memberships.sum()),
                mean_memberships_per_covered_cell=float(memberships.sum()/covered) if covered else 0.,
                nearest_neighbor_overlap=nearest, intersecting_pairs=pairs,
                nearest_overlap_median=float(np.median(neighbor_values)) if neighbor_values else None,
                nearest_overlap_in_band_fraction=float(np.mean(np.abs(np.asarray(neighbor_values)-config.overlap_target)
                                                              <= config.overlap_tolerance)) if neighbor_values else None)


class CircleCoverPlanner:
    def __init__(self, config=None, T_horizontal_map=None):
        self.config = config or CircleCoverConfig()
        self._frame = np.array(np.eye(4) if T_horizontal_map is None else T_horizontal_map, dtype=float, copy=True)
        self._validate_frame(self._frame)
        self._cells = np.empty((0, 2), dtype=np.int64)
        self._circles = ()
        self._last_update = -1

    @staticmethod
    def _validate_frame(frame):
        if (frame.shape != (4, 4) or not np.isfinite(frame).all() or
                not np.allclose(frame[3], [0, 0, 0, 1]) or
                not np.allclose(frame[:3, :3]@frame[:3, :3].T, np.eye(3), atol=1e-8) or
                not np.isclose(np.linalg.det(frame[:3, :3]), 1., atol=1e-8)):
            raise ValueError('Expected one finite proper horizontal-frame transform')

    @property
    def circles(self):
        return self._circles

    @property
    def cells(self):
        return self._cells.copy()

    @property
    def cell_centers(self):
        return (self._cells+.5)*self.config.cell_size_m + self.config.grid_origin_xy

    @property
    def T_horizontal_map(self):
        return self._frame.copy()

    def project(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('Expected finite map points with shape (N,3)')
        return (points @ self._frame[:3, :3].T + self._frame[:3, 3])[:, :2]

    def point_coverage(self, points):
        xy = self.project(points)
        mask = covered_mask(xy, [c.center_xy for c in self.circles], self.config.radius_m)
        return dict(points=len(xy), covered_points=int(mask.sum()), coverage=float(mask.mean()) if len(xy) else 1.)

    def _greedy(self, table, needed, candidates, old):
        covered = np.zeros(table.shape[0], dtype=bool)
        selected = []
        while int(covered.sum()) < needed:
            gain = np.asarray(table.T @ (~covered).astype(float)).ravel()
            if selected:
                gain[selected] = -1
            best = np.flatnonzero(gain == gain.max())
            if gain[best[0]] <= 0:
                raise RuntimeError('Candidate net cannot cover required cells')
            index = min(best, key=lambda k: (overlap_cost(candidates[[k]], np.vstack([old, candidates[selected]]),
                                                           self.config.radius_m, self.config.overlap_target), int(k)))
            selected.append(int(index))
            covered[table.indices[table.indptr[index]:table.indptr[index+1]]] = True
        return selected

    def _solve(self, table, needed, greedy):
        count, candidates = table.shape
        objective = np.r_[np.ones(candidates), np.zeros(count)]
        constraint = vstack([
            hstack([-table, eye(count, format='csc')]),
            csr_matrix(np.r_[np.zeros(candidates), np.ones(count)].reshape(1, -1)),
            csr_matrix(np.r_[np.ones(candidates), np.zeros(count)].reshape(1, -1))], format='csc')
        lower = np.r_[np.full(count, -np.inf), needed, -np.inf]
        upper = np.r_[np.zeros(count), np.inf, len(greedy)]
        started = time.perf_counter()
        try:
            result = milp(objective, integrality=np.r_[np.ones(candidates), np.zeros(count)],
                          bounds=Bounds(0, 1), constraints=LinearConstraint(constraint, lower, upper),
                          options=dict(time_limit=self.config.solver_time_limit_s, mip_rel_gap=0., presolve=True))
            selected = np.flatnonzero(result.x[:candidates] > .5).tolist() if result.x is not None else []
            feasible = (bool(selected) and len(selected) <= len(greedy) and
                        int(np.count_nonzero(np.asarray(table[:, selected].sum(axis=1)).ravel())) >= needed)
            finite = lambda x: float(x) if x is not None and np.isfinite(x) else None
            diagnostics = dict(status=int(result.status), message=str(result.message),
                               optimal_candidate_count=bool(result.status == 0 and feasible),
                               mip_gap=finite(getattr(result, 'mip_gap', None)),
                               dual_bound=finite(getattr(result, 'mip_dual_bound', None)),
                               objective=finite(result.fun), selected_count=len(selected),
                               fallback=not feasible, greedy_count=len(greedy))
        except Exception as error:
            selected = []
            diagnostics = dict(status=-1, message=str(error), optimal_candidate_count=False,
                               mip_gap=None, dual_bound=None, objective=None,
                               selected_count=0, fallback=True, greedy_count=len(greedy))
        diagnostics['wall_s'] = time.perf_counter()-started
        return (selected if diagnostics['fallback'] is False else greedy), diagnostics

    def _refine(self, new, old, uncovered, needed):
        started = time.perf_counter()
        config = self.config
        original_cost = overlap_cost(new, old, config.radius_m, config.overlap_target)
        if original_cost < 1e-15:
            return new, dict(applied=False, success=True, message='No overlap deviation to refine',
                             cost_before=original_cost, cost_after=original_cost, wall_s=0.)
        distance = np.linalg.norm(uncovered[:, None]-new[None], axis=2)
        assignment = np.argmin(distance, axis=1)
        nearest = distance[np.arange(len(uncovered)), assignment]
        eligible = np.flatnonzero(nearest <= config.radius_m+1e-10*max(1., config.radius_m))
        # Protect the necessary cells with largest boundary margin. Ties retain sorted cell order.
        protected = eligible[np.argsort(nearest[eligible], kind='stable')[:needed]]
        groups = [uncovered[protected[assignment[protected] == k]] for k in range(len(new))]
        active = [k for k, group in enumerate(groups) if len(group)]
        def slack(x):
            centers = x.reshape(-1, 2)
            return np.array([config.radius_m**2-np.max(np.sum((groups[k]-centers[k])**2, axis=1)) for k in active])
        def jacobian(x):
            centers = x.reshape(-1, 2); result = np.zeros((len(active), x.size))
            for row, k in enumerate(active):
                delta = centers[k]-groups[k]
                index = int(np.argmax(np.sum(delta**2, axis=1)))
                result[row, 2*k:2*k+2] = -2*delta[index]
            return result
        bounds = []
        for k, center in enumerate(new):
            bounds.extend([(float(v-config.radius_m), float(v+config.radius_m)) if k in active else (float(v), float(v))
                           for v in center])
        try:
            result = minimize(lambda x: overlap_cost(x.reshape(-1, 2), old, config.radius_m, config.overlap_target),
                              new.ravel(), method='SLSQP', bounds=bounds,
                              constraints=dict(type='ineq', fun=slack, jac=jacobian),
                              options=dict(maxiter=config.refinement_max_iterations, ftol=1e-10))
            proposed = result.x.reshape(-1, 2)
            cost = overlap_cost(proposed, old, config.radius_m, config.overlap_target)
            valid = (bool(result.success) and np.isfinite(proposed).all() and
                     cost <= original_cost+1e-12 and slack(result.x).min(initial=0.) >= -1e-6 and
                     int(covered_mask(uncovered, proposed, config.radius_m).sum()) >= needed)
            diagnostics = dict(applied=bool(valid and not np.array_equal(proposed, new)), success=bool(result.success),
                               message=str(result.message), cost_before=original_cost,
                               cost_after=cost if valid else original_cost,
                               iterations=int(result.nit), wall_s=time.perf_counter()-started)
            return (proposed if valid else new), diagnostics
        except Exception as error:
            return new, dict(applied=False, success=False, message=str(error), cost_before=original_cost,
                             cost_after=original_cost, wall_s=time.perf_counter()-started)

    def update(self, new_points, update_id, *, T_horizontal_map=None):
        started = time.perf_counter()
        if T_horizontal_map is not None and not np.array_equal(np.asarray(T_horizontal_map), self._frame):
            raise ValueError('Horizontal frame cannot change during a growing-map session')
        if (not isinstance(update_id, (int, np.integer)) or isinstance(update_id, (bool, np.bool_)) or
                update_id <= self._last_update or update_id < 0):
            raise ValueError('Expected strictly increasing nonnegative integer update IDs')
        xy = self.project(new_points)
        scaled = np.floor((xy-self.config.grid_origin_xy)/self.config.cell_size_m)
        if len(scaled) and np.max(np.abs(scaled)) >= 2**52:
            raise ValueError('Cell coordinates exceed exact floating-point indexing range')
        cells = np.unique(np.vstack([self._cells, scaled.astype(np.int64)]), axis=0)
        points = (cells+.5)*self.config.cell_size_m+self.config.grid_origin_xy
        old = np.asarray([c.center_xy for c in self.circles]).reshape(-1, 2)
        covered = covered_mask(points, old, self.config.radius_m)
        required = int(np.ceil(self.config.coverage_target*len(points)-1e-12))
        needed = max(0, required-int(covered.sum()))
        diagnostics = dict(update_id=int(update_id), added_cells=len(cells)-len(self._cells),
                           coverage_before=float(covered.mean()) if len(points) else 1.,
                           required_newly_covered_cells=needed, candidate_count=0,
                           solver=None, refinement=None, pruned_circles=0)
        new = np.empty((0, 2))
        if needed:
            uncovered = points[~covered]
            candidates = candidate_centers(uncovered, self.config.radius_m*self.config.candidate_spacing_fraction)
            table = coverage_table(uncovered, candidates, self.config.radius_m)
            greedy = self._greedy(table, needed, candidates, old)
            selected, diagnostics['solver'] = self._solve(table, needed, greedy)
            new = candidates[selected].copy()
            new, diagnostics['refinement'] = self._refine(new, old, uncovered, needed)
            diagnostics['candidate_count'] = len(candidates)
            for k in range(len(new)-1, -1, -1):
                reduced = np.delete(new, k, axis=0)
                if int(covered_mask(uncovered, reduced, self.config.radius_m).sum()) >= needed:
                    new = reduced
                    diagnostics['pruned_circles'] += 1
            new = new[np.lexsort((new[:, 1], new[:, 0]))]
        centers = np.vstack([old, new])
        if int(covered_mask(points, centers, self.config.radius_m).sum()) < required:
            raise RuntimeError('Placement did not satisfy coverage; state not committed')
        additions = tuple(Circle(len(self.circles)+i, tuple(map(float, center)), self.config.radius_m, int(update_id))
                          for i, center in enumerate(new))
        # All expensive work and acceptance checks precede the state transaction.
        metrics = placement_metrics(points, centers, self.config)
        diagnostics.update(metrics)
        diagnostics['wall_s'] = time.perf_counter()-started
        self._cells, self._circles, self._last_update = cells, self.circles+additions, int(update_id)
        return CoverUpdate(additions, diagnostics)

    def state(self):
        return dict(schema_version=1, config=asdict(self.config), T_horizontal_map=self._frame.tolist(),
                    occupied_cells=self._cells.tolist(), circles=[asdict(c) for c in self.circles],
                    last_update_id=self._last_update)

    def save(self, path):
        path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
        state = self.state()
        raw = json.dumps(state, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        envelope = dict(state=state, sha256=hashlib.sha256(raw).hexdigest())
        temporary = path.with_name(path.name+'.partial')
        temporary.write_text(json.dumps(envelope, sort_keys=True, allow_nan=False)+'\n')
        temporary.replace(path)

    @classmethod
    def load(cls, path):
        envelope = json.loads(Path(path).read_text()); state = envelope['state']
        raw = json.dumps(state, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        if hashlib.sha256(raw).hexdigest() != envelope['sha256'] or state['schema_version'] != 1:
            raise ValueError('Invalid circle-cover state or checksum')
        planner = cls(CircleCoverConfig(**state['config']), state['T_horizontal_map'])
        cells = np.asarray(state['occupied_cells'])
        if not cells.size:
            cells = np.empty((0, 2), dtype=np.int64)
        if cells.ndim != 2 or cells.shape[1] != 2 or cells.dtype.kind not in 'iu':
            raise ValueError('Invalid occupied cells')
        if cells.size and np.max(np.abs(cells.astype(float))) >= 2**52:
            raise ValueError('Invalid saved cell coordinate range')
        if not np.array_equal(cells, np.unique(cells, axis=0)):
            raise ValueError('State cells must be sorted and unique')
        last = state['last_update_id']
        if not isinstance(last, int) or isinstance(last, bool) or last < -1:
            raise ValueError('Invalid last update')
        circles = []
        for index, value in enumerate(state['circles']):
            c = Circle(**dict(value, center_xy=tuple(value['center_xy'])))
            if (type(c.id) is not int or c.id != index or len(c.center_xy) != 2 or not np.isfinite(c.center_xy).all() or
                    c.radius_m != planner.config.radius_m or not isinstance(c.created_update, int) or
                    isinstance(c.created_update, bool) or
                    not 0 <= c.created_update <= last or
                    (circles and c.created_update < circles[-1].created_update)):
                raise ValueError('Invalid saved circle')
            circles.append(c)
        if len(cells) and (last < 0 or not circles):
            raise ValueError('Occupied state has no published cover')
        planner._cells, planner._circles, planner._last_update = cells.astype(np.int64), tuple(circles), last
        required = int(np.ceil(planner.config.coverage_target*len(cells)-1e-12))
        if covered_mask(planner.cell_centers, [c.center_xy for c in circles], planner.config.radius_m).sum() < required:
            raise ValueError('Saved state fails its coverage target')
        return planner

"""Causal, standalone replay of retained accumulated-area snapshot observations.

No odometry rerun, descriptors, ground truth, or graph optimization is involved.
Run with: PYTHONPATH=research python -m s3e_pipeline.circle_cover_replay --help
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import platform
import resource
import shutil
import threading
import time

import numpy as np
import psutil
import scipy
import yaml

from .circle_cover import CircleCoverConfig, CircleCoverPlanner, covered_mask, placement_metrics
from .gravity_bev import gravity_ground


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


class PreparedObservations:
    """Union stable geometry IDs; freeze each point at its first available observation.

    Prepared cloud rows must preserve the original ordered native point-ID ranges.
    Revisited coordinates are audited within float32 export precision, rather than
    allowing round-trip jitter to create additional grid cells.
    """
    def __init__(self, store):
        self.store = Path(store)
        self.index = self.store/'keyframes.jsonl'
        self.rows = sorted([json.loads(v) for v in self.index.read_text().splitlines() if v.strip()],
                           key=lambda row: (row['available_ns'], row['keyframe_id']))
        if not self.rows:
            raise ValueError('No retained snapshots')
        self.frame = gravity_ground(self.rows[0]['gravity_world_m_s2'])
        self.world_frame = self.rows[0]['world_frame']
        self.robot_id = self.rows[0]['robot_id']
        self.ids = np.empty(0, dtype=np.int64)
        self.world = np.empty((0, 3))
        self.payloads = []
        self.max_repeat_delta_m = 0.

    def __iter__(self):
        previous_available = -1
        previous_key = -1
        for row in self.rows:
            if (row['available_ns'] <= previous_available or row['keyframe_id'] <= previous_key or
                    row['available_ns'] < max(row['stamp_ns'], row['last_member_ns']) or
                    row['world_frame'] != self.world_frame or row['robot_id'] != self.robot_id or
                    row['strategy'] != 'area' or row['frame'] != 'snapshot_anchor_imu' or
                    row['cloud_frame'] != row['body_frame'] or not row['complete']):
                raise ValueError('Noncausal, inconsistent, or incomplete retained snapshot')
            path = self.store/f"{row['keyframe_id']:06d}.npz"
            with np.load(path, allow_pickle=False) as data:
                cloud = np.asarray(data['cloud'], dtype=float)
            T = np.asarray(row['T_world_body'], dtype=float).reshape(4, 4)
            CircleCoverPlanner._validate_frame(T)
            ranges = row['geometry_id_ranges']
            if any(a < 0 or b <= a or (k and a <= ranges[k-1][1]) for k, (a, b) in enumerate(ranges)):
                raise ValueError('Point-ID ranges must be positive, ordered and coalesced')
            ids = np.concatenate([np.arange(a, b, dtype=np.int64) for a, b in ranges]) if ranges else np.empty(0, np.int64)
            if cloud.shape != (row['geometry_count'], 3) or len(cloud) != len(ids) or not np.isfinite(cloud).all():
                raise ValueError('Prepared native geometry/ID membership mismatch')
            if len(ids) and ids[-1] >= row['archive_point_count']:
                raise ValueError('Point ID exceeds available map archive')
            world = cloud@T[:3, :3].T+T[:3, 3]
            indices = np.searchsorted(self.ids, ids)
            known = indices < len(self.ids)
            known[known] &= self.ids[indices[known]] == ids[known]
            if known.any():
                delta = np.abs(self.world[indices[known]]-world[known])
                self.max_repeat_delta_m = max(self.max_repeat_delta_m, float(delta.max()))
                if not np.allclose(self.world[indices[known]], world[known], atol=1e-3, rtol=1e-6):
                    raise ValueError('Persistent point coordinates changed beyond export precision')
            new = world[~known]
            self.ids = np.r_[self.ids, ids[~known]]
            self.world = np.vstack([self.world, new])
            order = np.argsort(self.ids, kind='stable')
            self.ids, self.world = self.ids[order], self.world[order]
            self.payloads.append(dict(keyframe_id=row['keyframe_id'], available_ns=row['available_ns'],
                                      file=str(path), sha256=sha256(path), native_points=len(ids),
                                      first_seen_points=len(new)))
            previous_available, previous_key = row['available_ns'], row['keyframe_id']
            yield row, new


class MemorySampler:
    """Actual simultaneous resident memory; never sum separate process peaks."""
    def __init__(self):
        self.peak = 0; self.peak_children = 0; self.samples = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        process = psutil.Process()
        while not self.stop.is_set():
            children = process.children(recursive=True)
            total = process.memory_info().rss
            for child in children:
                try:
                    total += child.memory_info().rss
                except psutil.NoSuchProcess:
                    pass
            self.peak = max(self.peak, total)
            self.peak_children = max(self.peak_children, len(children)); self.samples += 1
            self.stop.wait(.02)

    def __enter__(self):
        self.thread.start(); return self

    def __exit__(self, *args):
        self.stop.set(); self.thread.join()


def replay(store, output, config, make_gallery=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output/'states').mkdir(); (output/'inspection').mkdir()
    observations = PreparedObservations(store)
    planner = CircleCoverPlanner(config, observations.frame)
    baseline = []
    history = []
    states = []
    start = time.perf_counter()
    with MemorySampler() as memory:
        for update_id, (row, new) in enumerate(observations):
            old = planner.circles
            update = planner.update(new, update_id)
            if planner.circles[:len(old)] != old or update.diagnostics['coverage'] < config.coverage_target:
                raise RuntimeError('Acceptance invariant violated')
            baseline_center = planner.project(np.asarray(row['area_center_world']).reshape(1, 3))[0]
            baseline.append(baseline_center.tolist())
            before = time.perf_counter()
            original = placement_metrics(planner.cell_centers, baseline, config)
            original['audit_wall_s'] = time.perf_counter()-before
            original['point_coverage'] = float(covered_mask(planner.project(observations.world), baseline, config.radius_m).mean())
            update.diagnostics['point_coverage'] = planner.point_coverage(observations.world)
            planner.save(output/'states'/f'{update_id:03d}.json')
            restored = CircleCoverPlanner.load(output/'states'/f'{update_id:03d}.json')
            if restored.state() != planner.state():
                raise RuntimeError('Persisted state differs from live state')
            history.append(dict(update_id=update_id, keyframe_id=row['keyframe_id'],
                                available_ns=row['available_ns'], elapsed_sensor_s=(row['available_ns']-observations.rows[0]['available_ns'])*1e-9,
                                native_unique_points=len(observations.ids), new_circles=[asdict(c) for c in update.new_circles],
                                planner=update.diagnostics, trajectory_centered=original,
                                planner_core_array_bytes=planner.cells.nbytes+planner.T_horizontal_map.nbytes,
                                current_process_rss_bytes=psutil.Process().memory_info().rss))
            states.append(planner.state())
            print(f"update {update_id+1}/{len(observations.rows)}: {len(planner.cells)} cells, "
                  f"{len(planner.circles)} circles, {update.diagnostics['coverage']:.4%} covered, "
                  f"{update.diagnostics['wall_s']:.3f}s", flush=True)
    process_hwm = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024  # Linux KiB
    source = {str(Path(__file__).name): sha256(__file__), 'circle_cover.py': sha256(Path(__file__).with_name('circle_cover.py')),
              'gravity_bev.py': sha256(Path(__file__).with_name('gravity_bev.py'))}
    provenance = dict(schema_version=1, method='incremental fixed-radius circle footprints',
                      branch='dev/incremental-circle-submaps', published_base='1eec4b98c51267398d91d8ad953cdbc472951262',
                      ground_truth_used=False, source_sha256=source, config=asdict(config),
                      metadata=dict(path=str(observations.index), sha256=sha256(observations.index)),
                      payloads=observations.payloads, T_horizontal_map=observations.frame.tolist(),
                      robot_id=observations.robot_id, world_frame=observations.world_frame,
                      python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__, hostname=platform.node(),
                      map_semantics='growing union of retained prepared native area snapshots; not every original scan',
                      point_identity='stable native geometry IDs; coordinates frozen at first available observation',
                      maximum_reobserved_coordinate_delta_m=observations.max_repeat_delta_m,
                      baseline='original trajectory-centered radii and centers, reprojected into the same fixed first-snapshot gravity frame',
                      observation_count=len(history), final_unique_native_points=len(observations.ids),
                      replay_wall_s_before_plotting=time.perf_counter()-start,
                      memory=dict(peak_sampled_simultaneous_process_tree_rss_bytes=memory.peak,
                                  process_high_water_rss_bytes_before_plotting=process_hwm,
                                  sampling_period_s=.02, samples=memory.samples, max_child_processes=memory.peak_children,
                                  scope='fresh replay process, input union, solver, state-history validation; excludes gallery plotting'))
    write_json(output/'inspection'/'provenance.json', provenance)
    write_json(output/'inspection'/'history.json', history)
    write_json(output/'inspection'/'placement-history.json', [dict(update_id=k, circles=s['circles']) for k, s in enumerate(states)])
    planner.save(output/'final-state.json')
    write_json(output/'inspection'/'summary.json', dict(config=asdict(config), final=history[-1],
        minimum_prefix_coverage=min(h['planner']['coverage'] for h in history),
        total_added_circles=len(planner.circles), solver_updates=sum(h['planner']['solver'] is not None for h in history),
        proven_candidate_minimum_updates=sum(bool(h['planner']['solver'] and h['planner']['solver']['optimal_candidate_count']) for h in history),
        greedy_fallback_updates=sum(bool(h['planner']['solver'] and h['planner']['solver']['fallback']) for h in history),
        mean_update_s=float(np.mean([h['planner']['wall_s'] for h in history])),
        median_update_s=float(np.median([h['planner']['wall_s'] for h in history])),
        p95_update_s=float(np.percentile([h['planner']['wall_s'] for h in history], 95)),
        max_update_s=max(h['planner']['wall_s'] for h in history), memory=provenance['memory']))
    if make_gallery:
        render_gallery(output/'inspection', states, history, np.asarray(baseline), config)
    return history


def render_gallery(output, states, history, baseline, config):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle as CirclePatch
    output = Path(output)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    origin = np.asarray(config.grid_origin_xy)
    final_cells = (np.asarray(states[-1]['occupied_cells'])+.5)*config.cell_size_m+origin
    # Final extents are used only for post-replay presentation, never placement.
    all_centers = np.vstack([baseline, [c['center_xy'] for c in states[-1]['circles']]])
    lo = np.minimum(final_cells.min(axis=0), all_centers.min(axis=0)-config.radius_m)-8
    hi = np.maximum(final_cells.max(axis=0), all_centers.max(axis=0)+config.radius_m)+8
    for k, state in enumerate(states):
        xy = (np.asarray(state['occupied_cells'])+.5)*config.cell_size_m+origin
        circles = state['circles']
        centers = np.asarray([c['center_xy'] for c in circles])
        fig, axes = plt.subplots(1, 2, figsize=(13, 6.2), constrained_layout=True)
        for ax, cc, title in [(axes[0], centers, 'Incremental map coverage'), (axes[1], baseline[:k+1], 'Existing trajectory-centered footprints')]:
            mask = covered_mask(xy, cc, config.radius_m)
            ax.scatter(xy[mask, 0], xy[mask, 1], s=.6, color='#8796a3', rasterized=True)
            ax.scatter(xy[~mask, 0], xy[~mask, 1], s=3, color='#e07025', rasterized=True, label='Uncovered cells')
            for j, center in enumerate(cc):
                is_new = (circles[j]['created_update'] == k) if ax is axes[0] else j == k
                color = '#168a76' if is_new else '#375875'
                ax.add_patch(CirclePatch(center, config.radius_m, fill=False, ec=color, lw=1.8 if is_new else .8, alpha=.9 if is_new else .6))
                ax.plot(*center, '+', color=color, ms=5)
                if ax is axes[0]:
                    ax.text(center[0]+2, center[1]+2, str(j), color=color, fontsize=8)
            ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='Horizontal x [m]', ylabel='Horizontal y [m]',
                   title=f'{title}\n{len(cc)} circles · {mask.mean():.2%} cell coverage', aspect='equal')
            ax.grid(alpha=.12)
        fig.suptitle(f"GRACO A05 · retained snapshot {k+1}/{len(states)} · {history[k]['elapsed_sensor_s']:.1f}s since first available snapshot\n"
                     'Green: newly published circles  |  Blue: fixed previous circles  |  Orange: uncovered cells', fontsize=12)
        fig.savefig(output/f'prefix-{k:03d}.png', dpi=135); plt.close(fig)
    times = np.asarray([h['elapsed_sensor_s'] for h in history])
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for field, ax, title in [('circle_count', axes[0, 0], 'Published footprints'), ('coverage', axes[0, 1], 'Occupied-cell coverage'),
                             ('mean_memberships_per_covered_cell', axes[1, 0], 'Cell memberships / covered cell'),
                             ('nearest_overlap_median', axes[1, 1], 'Median greatest disk overlap')]:
        for method, color, label in [('planner', '#168a76', 'Incremental circles'), ('trajectory_centered', '#375875', 'Trajectory-centered')]:
            values = [h[method][field] for h in history]
            ax.plot(times, [np.nan if v is None else v for v in values], color=color, label=label, drawstyle='steps-post')
        if field == 'coverage': ax.axhline(config.coverage_target, color='#e07025', ls='--', lw=1); ax.set_ylim(.8, 1.005)
        if field == 'nearest_overlap_median': ax.axhspan(config.overlap_target-config.overlap_tolerance, config.overlap_target+config.overlap_tolerance, color='#168a76', alpha=.12); ax.set_ylim(0, 1)
        ax.set(title=title, xlabel='Time since first available snapshot [s]'); ax.grid(alpha=.15)
    axes[0, 0].legend(frameon=False)
    fig.savefig(output/'timeline.png', dpi=150); fig.savefig(output/'timeline.pdf'); plt.close(fig)
    final = history[-1]
    figure_links = json.dumps([dict(image=f'prefix-{k:03d}.png', update=k+1,
                                   time=h['elapsed_sensor_s'], circles=h['planner']['circle_count'],
                                   coverage=h['planner']['coverage'], seconds=h['planner']['wall_s']) for k, h in enumerate(history)])
    html = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A05 incremental circle placement</title><style>body{font:17px system-ui;max-width:1250px;margin:35px auto;padding:0 20px;color:#213442;background:#f5f7f9}h1{font-size:30px}img{width:100%;background:white;border-radius:8px}input{width:100%}button{padding:8px 16px}a{color:#087a67}.card{background:white;padding:22px;border-radius:10px;margin:20px 0}.hint{color:#526576;font-size:15px}</style>
<h1>GRACO A05 · incremental spatial footprints</h1><p>Radius 80 m · 95% occupied-cell coverage · desired greatest disk overlap 30% ±10 percentage points.</p>
<p class="hint">Causal growing union of 34 retained prepared area snapshots, not every original scan. Gravity is fixed from the first snapshot. No trajectory or future map enters circle placement. Published circles remain fixed.</p>
<div class="card"><button id="prev">Previous</button> <button id="play">Play</button> <button id="next">Next</button>
<p id="status" aria-live="polite"></p><input id="slider" aria-label="Available snapshot" type="range" min="0" max="33" value="33"><img id="map" alt="Growing observed map and incremental versus trajectory-centered circles"></div>
<div class="card"><h2>Timeline</h2><img src="timeline.png" alt="Circle count, coverage, repeated membership and overlap timelines"></div>
<p class="hint">The overlap band is a secondary objective. Coverage and fewer added circles take priority; deviations are retained in the diagnostics. Counts are certified only within that update’s candidate set when HiGHS proves optimality. Footprint geometry update/freeze policy is deferred.</p>
<p><a href="summary.json">Summary</a> · <a href="history.json">All update/solver/overlap diagnostics</a> · <a href="placement-history.json">Fixed center history</a> · <a href="provenance.json">Source and input hashes</a> · <a href="timeline.pdf">Timeline PDF</a></p>
<script>const frames=FRAMES;const slider=document.getElementById('slider');let timer=null;
function show(){const f=frames[Number(slider.value)];document.getElementById('map').src=f.image;document.getElementById('status').textContent=`Snapshot ${f.update}/${frames.length} · ${f.time.toFixed(1)} s · ${f.circles} circles · ${(100*f.coverage).toFixed(2)}% coverage · ${f.seconds.toFixed(3)} s planning`;}
slider.max=frames.length-1;slider.value=frames.length-1;slider.oninput=show;
document.getElementById('prev').onclick=()=>{slider.value=Math.max(0,Number(slider.value)-1);show()};document.getElementById('next').onclick=()=>{slider.value=Math.min(frames.length-1,Number(slider.value)+1);show()};
document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}else{slider.value=0;show();document.getElementById('play').textContent='Pause';timer=setInterval(()=>{if(Number(slider.value)>=frames.length-1){clearInterval(timer);timer=null;document.getElementById('play').textContent='Play'}else{slider.value=Number(slider.value)+1;show()}},800)}};show();</script></html>'''
    (output/'index.html').write_text(html.replace('FRAMES', figure_links))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'configs'/'circle_cover.yaml')
    parser.add_argument('--cell-size', type=float, help='Grid-resolution sensitivity control only')
    parser.add_argument('--no-gallery', action='store_true')
    args = parser.parse_args()
    config = CircleCoverConfig(**yaml.safe_load(args.config.read_text()))
    if args.cell_size is not None:
        config = replace(config, cell_size_m=args.cell_size)
    replay(args.store, args.output, config, not args.no_gallery)
    shutil.copyfile(args.config, args.output/'inspection'/'requested-config.yaml')


if __name__ == '__main__':
    main()

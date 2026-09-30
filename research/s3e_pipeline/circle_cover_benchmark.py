"""Fresh-process planner versus original-footprint runtime/RSS controls.

Neither control retains full occupancy history or renders figures. Both load
the same causal input union and audit coverage and point coverage each update.
This measures the standalone footprint stage, not the existing LiDAR pipeline.
"""
import argparse
from dataclasses import asdict, replace
from pathlib import Path
import platform
import resource
import time

import numpy as np
import scipy
import yaml

from .circle_cover import CircleCoverConfig, CircleCoverPlanner, placement_metrics, covered_mask
from .circle_cover_replay import PreparedObservations, MemorySampler, write_json, sha256


def benchmark(store, mode, config, output):
    if mode not in ('planner', 'trajectory_centered'):
        raise ValueError('Unknown footprint mode')
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    observations = PreparedObservations(store)
    planner = CircleCoverPlanner(config, observations.frame)
    metric_config = replace(config, radius_m=observations.area_radius_m) if mode == 'trajectory_centered' else config
    cells = np.empty((0, 2), dtype=np.int64)
    centers = []
    records = []
    started = time.perf_counter()
    with MemorySampler() as memory:
        for update_id, (row, new) in enumerate(observations):
            t = time.perf_counter()
            if mode == 'planner':
                report = planner.update(new, update_id).diagnostics
                xy = planner.cell_centers
                current = [c.center_xy for c in planner.circles]
            else:
                projected = planner.project(new)
                inserted = np.floor((projected-config.grid_origin_xy)/config.cell_size_m).astype(np.int64)
                cells = np.unique(np.vstack([cells, inserted]), axis=0)
                xy = (cells+.5)*config.cell_size_m+config.grid_origin_xy
                centers.append(planner.project(np.asarray(row['area_center_world']).reshape(1, 3))[0].tolist())
                current = centers
                report = placement_metrics(xy, current, metric_config)
            stage_wall = time.perf_counter()-t
            point_coverage = float(covered_mask(planner.project(observations.world), current, metric_config.radius_m).mean())
            records.append(dict(update_id=update_id, available_ns=row['available_ns'],
                                occupied_cells=len(xy), circle_count=len(current), coverage=report['coverage'],
                                point_coverage=point_coverage, stage_wall_s=stage_wall,
                                solver=report.get('solver'), mean_memberships=report['mean_memberships_per_covered_cell']))
    output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(mode=mode, config=asdict(config), updates=records,
                  footprint_radius_m=metric_config.radius_m,
                  hostname=platform.node(), python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                  mean_stage_s=float(np.mean([r['stage_wall_s'] for r in records])),
                  p95_stage_s=float(np.percentile([r['stage_wall_s'] for r in records], 95)),
                  max_stage_s=max(r['stage_wall_s'] for r in records),
                  total_replay_wall_s=time.perf_counter()-started,
                  peak_sampled_simultaneous_process_tree_rss_bytes=memory.peak,
                  process_high_water_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                  max_child_processes=memory.peak_children, sampling_period_s=.02,
                  source_sha256={name: sha256(Path(__file__).with_name(name)) for name in
                                 ('circle_cover_benchmark.py', 'circle_cover.py', 'circle_cover_replay.py', 'gravity_bev.py')},
                  metadata_sha256=sha256(observations.index), input_payloads=observations.payloads,
                  scope='one fresh process per mode; identical native input union; no occupancy history, state I/O or plotting',
                  timing_scope='grid insertion plus footprint placement and cell/overlap audit; excludes input I/O and point audit',
                  limitation='standalone footprint stage; not native LiDAR/submap/MapClosures memory or runtime')
    write_json(output, result)
    print(f'{mode}: {records[-1]["circle_count"]} circles; mean {result["mean_stage_s"]:.4f}s; '
          f'peak RSS {memory.peak/2**20:.1f}MiB', flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['planner', 'trajectory_centered'], required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'configs'/'circle_cover.yaml')
    args = parser.parse_args()
    benchmark(args.store, args.mode, CircleCoverConfig(**yaml.safe_load(args.config.read_text())), args.output)


if __name__ == '__main__':
    main()

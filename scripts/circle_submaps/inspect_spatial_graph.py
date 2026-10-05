#!/usr/bin/env python3
"""Replay sealed local prefixes; retain compact spatial-graph inspection data."""
import argparse
from dataclasses import asdict
from pathlib import Path
import resource
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.artifacts import file_hash, read_jsonl, write_json
from s3e_pipeline.cbs_bridge import local_graph
from s3e_pipeline.spatial_graph import SpatialGraph


def run(store, output):
    output.mkdir(parents=True, exist_ok=False)
    rows = read_jsonl(store/'keyframes.jsonl')
    if not rows:
        raise ValueError('Empty spatial store')
    robot = rows[0]['robot_id']
    pgo = dict(odometry_rotation_sigma_deg=1., odometry_translation_sigma_m=.15,
               spatial_neighbors=dict(enabled=True))
    graph = SpatialGraph(robot, store, pgo)
    prefixes = []; old = []; start = time.perf_counter()
    for n in range(1, len(rows)+1):
        before = time.perf_counter()
        edges = graph.update(rows[:n])
        if edges[:len(old)] != old:
            raise AssertionError('An admitted spatial edge changed')
        old = list(edges)
        prefixes.append(dict(submaps=n, available_ns=rows[n-1]['available_ns'],
            odometry_edges=0, components=graph.diagnostics()['component_count'], map_consistency_edges=len(edges),
            additional_degree_max=max(graph.degree), construction_s=time.perf_counter()-before))
    nodes, edges = local_graph(rows, robot, pgo, spatial_graph=graph)
    result = dict(robot=robot, nodes=len(nodes), odometry_edges=0,
        map_consistency_edges=len(graph.edges), local_cycle_rank=graph.diagnostics()['cycle_rank'],
        component_count=graph.diagnostics()['component_count'],
        construction_wall_s=time.perf_counter()-start,
        fresh_process_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        scope='sealed publication-prefix graph construction only; no optimization or new sensor replay',
        input_index=str(store/'keyframes.jsonl'), input_sha256=file_hash(store/'keyframes.jsonl'),
        source_hashes={p.name:file_hash(p) for p in [Path(__file__),
            Path(__file__).resolve().parents[2]/'research/s3e_pipeline/spatial_graph.py',
            Path(__file__).resolve().parents[2]/'research/s3e_pipeline/cbs_bridge.py']},
        configuration=dict(pgo, spatial_neighbors=asdict(graph.cfg)),
        diagnostics=graph.diagnostics(), prefixes=prefixes, constraints=edges,
        footprints=[{k:r[k] for k in ('keyframe_id','stamp_ns','available_ns','footprint_center_xy',
            'footprint_radius_m','T_world_body','T_horizontal_map')} for r in rows])
    write_json(output/'graph.json', result)
    print({k:result[k] for k in ('robot','nodes','odometry_edges','map_consistency_edges',
        'construction_wall_s','fresh_process_peak_rss_kib')}, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--store', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.store, args.output)

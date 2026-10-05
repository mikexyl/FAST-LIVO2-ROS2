#!/usr/bin/env python3
"""Measure persistent spatial CBS at a slow cadence, preserving failed results."""
import argparse
import csv
import json
from pathlib import Path
import resource
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'research'))
from s3e_pipeline.artifacts import file_hash, read_json, write_json
from s3e_pipeline.circle_cbs import prepare, solution_graph, evidence_comparison
from s3e_pipeline.circle_cover_replay import MemorySampler
from s3e_pipeline.persistent_cbs import PersistentSession, save_snapshot


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(count=0)
    return dict(count=len(values), minimum=float(values.min()),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                maximum=float(values.max()))


def run(args):
    config = yaml.safe_load(args.config.read_text())
    config['dpgo'].update(execution='persistent', persistent_loop_rate_hz=1 / args.interval,
                          persistent_settle_iterations=args.iterations,
                          timeout_s=max(600., args.interval * args.iterations + 180.))
    stores, rows, loops = prepare(args.capture, args.output, config)
    write_json(args.output / 'probe-source.json', dict(sha256=file_hash(__file__),
        interval_s=args.interval, iterations=args.iterations,
        scope='Frozen verified graph in persistent optimizers; no frontend replay',
        scheduling='Native mutually exclusive periodic callback; asynchronous peer requests'))
    session = None
    latest = {}
    seen = {}
    observations = {r: [] for r in config['robots']}
    snapshot = None
    failure = None
    start = time.monotonic()
    cpu = time.process_time()
    children = resource.getrusage(resource.RUSAGE_CHILDREN)
    with MemorySampler() as memory, (args.output / 'iteration-timeline.jsonl').open('w') as log:
        try:
            session = PersistentSession(config, args.source, args.output / 'cbs/session', stores)
            while not session.ready():
                if time.monotonic() - start > 60:
                    raise TimeoutError('Persistent startup timeout')
                time.sleep(.1)
            session.submit(rows, loops, final=True)
            while True:
                for robot in config['robots']:
                    path = session.output / robot / 'estimate.json'
                    if not path.exists():
                        continue
                    value = read_json(path)
                    latest[robot] = value
                    status = value['status']
                    identity = status['iteration'], status['finished']
                    if seen.get(robot) == identity:
                        continue
                    seen[robot] = identity
                    event = dict(robot=robot, observed_elapsed_s=time.monotonic() - start,
                        iteration=status['iteration'], finished=status['finished'],
                        component=status['component'], reference_available=status['reference_available'],
                        cbs_last_stats=status['cbs_last_stats'])
                    observations[robot].append(event)
                    log.write(json.dumps(event, sort_keys=True) + '\n'); log.flush()
                    print(robot, identity, status['component'], flush=True)
                # Enforces the existing final shared-frame and geometry checks.
                snapshot = session.snapshot()
                if snapshot and snapshot['summary']['finished']:
                    break
                time.sleep(.1)
        except Exception as error:
            failure = dict(type=type(error).__name__, message=str(error))
        finally:
            if session:
                session.close()
    elapsed = time.monotonic() - start
    end = resource.getrusage(resource.RUSAGE_CHILDREN)
    # Refresh the final atomic outputs after all owned processes have stopped.
    for robot in config['robots']:
        path = args.output / 'cbs/session' / robot / 'estimate.json'
        if path.exists():
            latest[robot] = read_json(path)
    write_json(args.output / 'final-robot-estimates.json', latest)
    timing = {}
    for robot, events in observations.items():
        first = {}
        for event in events:
            first.setdefault(event['iteration'], event['observed_elapsed_s'])
        gaps = [first[b] - first[a] for a, b in zip(sorted(first), sorted(first)[1:]) if b == a + 1]
        native = args.output / 'cbs/session' / robot / 'native' / robot / 'cbs_online'
        values = dict(observed_iteration_interval_s=distribution(gaps))
        for pattern, columns in [
            ('timing_robot_*.csv', ['update_ms', 'active_callback_ms', 'belief_request_ms']),
            ('belief_service_timing_robot_*.csv', ['get_beliefs_ms', 'callback_ms', 'lock_wait_ms'])]:
            paths = list(native.glob(pattern))
            if paths:
                with paths[0].open() as f:
                    records = list(csv.DictReader(f))
                for column in columns:
                    if records and column in records[0]:
                        values[column] = distribution([float(r[column]) for r in records])
        timing[robot] = values
    result = dict(success=failure is None, failure=failure, iterations_requested=args.iterations,
        interval_requested_s=args.interval, timing=timing,
        robots={r:v['status'] for r,v in latest.items()},
        resources=dict(wall_s=elapsed, parent_cpu_s=time.process_time()-cpu,
            child_cpu_s=end.ru_utime+end.ru_stime-children.ru_utime-children.ru_stime,
            peak_simultaneous_process_tree_rss_bytes=memory.peak),
        timing_notes='Observed cadence has 100 ms polling uncertainty. Callback timings exclude asynchronous response latency; terminal callbacks are absent from native timing CSV.')
    if session and latest:
        result['evidence'] = evidence_comparison(session.output, args.central_registration)
    write_json(args.output / 'result.json', result)
    if failure is not None:
        write_json(args.output / 'FAILED.json', failure)
        print('FAILED:', failure, flush=True)
        return
    save_snapshot(snapshot, args.output / 'cbs')
    (args.output / 'central').mkdir()
    graph = solution_graph(snapshot, rows)
    write_json(args.output / 'central/graph.json', graph)
    # A reference label and owner-local geometry checks cannot establish that
    # different robots published mutually consistent transforms. Audit the
    # published endpoints with the configured alignment tolerances, without GT.
    limits = dict(translation_m=config['pgo']['alignment_translation_m'],
                  rotation_deg=config['pgo']['alignment_rotation_deg'])
    bad = [r for r in graph['published_loop_residuals']
           if any(not np.isfinite(r[k]) or r[k] > limit for k, limit in limits.items())]
    audit = dict(success=not bad, limits=limits, failed_loops=bad,
                 scope='Published loop transform sanity check; no ground truth')
    write_json(args.output / 'published-transform-audit.json', audit)
    from s3e_pipeline.circle_central_evaluation import evaluate_run
    evaluate_run(args.output, config)
    if bad:
        result.update(success=False, failure=dict(type='PublishedTransformConsistency',
            message='Published robot transforms violate configured alignment tolerances'))
        write_json(args.output / 'result.json', result)
        write_json(args.output / 'FAILED.json', result['failure'])
        return
    write_json(args.output / 'COMPLETE.json', dict(success=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('capture', 'config', 'source', 'output', 'central-registration'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--interval', type=float, default=10.)
    parser.add_argument('--iterations', type=int, default=30)
    args = parser.parse_args()
    if not np.isfinite(args.interval) or args.interval <= 0 or args.iterations < 1:
        parser.error('Require positive finite interval and positive iterations')
    run(args)

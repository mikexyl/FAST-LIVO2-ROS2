"""Staged centralized submap graph with measured robot/server byte streams.

Robot processes own their stores. The server receives descriptors/poses once,
requests verification geometry on demand, caches it, and returns optimized poses.
The benchmark counts actual serialized Pipe messages (including framing), not
NPZ sizes or hypothetical broadcasts. TCP/DDS overhead is outside this scope.
"""
import argparse
from collections import defaultdict
from dataclasses import asdict
import json
import multiprocessing as mp
from pathlib import Path
import resource
import subprocess
import sys
import time
import traceback
import zlib

import numpy as np
import yaml

from .artifacts import canonical, digest, file_hash, read_json, read_jsonl, write_json, write_jsonl
from .backends import create, pack_payload, unpack_payload
from .circle_cover import CircleCoverConfig
from .circle_cover_replay import MemorySampler
from .circle_submaps import build, prepare, validate_row, validate_geometry
from .distributed import eligible_observation, select_branches
from .geometry import canonical_edge, constraint
from .mixed_pgo import native_provenance, refine_graph, settings
from .pgo import optimize
from .registration import bounded_cloud


def serve(connection, root, robot, cfg, session):
    """One robot's read-only evidence service; no remote store paths accepted."""
    try:
        root = Path(root); rows = read_jsonl(root/'store/keyframes.jsonl')
        descriptors = {}; backend_hash = digest(cfg)
        for key, row in enumerate(rows):
            validate_row(row)
            if row['robot_id'] != robot or row['keyframe_id'] != key: raise ValueError('Endpoint identity mismatch')
            if row['retrievable']:
                descriptor = json.loads(zlib.decompress((root/'descriptors'/f'{key:06d}.json.zlib').read_bytes()))
                if descriptor['payload_sha256'] != row['sha256']: raise ValueError('Descriptor/evidence mismatch')
                descriptors[key] = descriptor
        connection.send_bytes(canonical(dict(ready=True, session=session, config_hash=backend_hash)))
        while True:
            request = json.loads(connection.recv_bytes())
            if request['session'] != session or request['config_hash'] != backend_hash: raise ValueError('Session/config mismatch')
            op = request['op']
            if op == 'manifest':
                response = dict(rows=rows, descriptors=descriptors)
            elif op == 'geometry':
                key = request['key']
                if type(key) is not int or not 0 <= key < len(rows): raise ValueError('Invalid endpoint')
                row = rows[key]; path = root/'store'/f'{key:06d}.npz'
                if file_hash(path) != row['sha256']: raise ValueError('Modified immutable geometry')
                with np.load(path, allow_pickle=False) as f:
                    points = np.asarray(f['cloud'], dtype=float)
                    validate_geometry(row,points,f['point_ids'])
                xyz, voxel = bounded_cloud(points, cfg['evidence']['voxel_m'], None, None)
                response = pack_payload(dict(row=row, cloud=xyz, image=b'', descriptor=descriptors[key],
                    evidence_preprocessing=dict(sampling_policy='fixed', requested_voxel_m=cfg['evidence']['voxel_m'],
                        effective_voxel_m=voxel, input_points=len(points), output_points=len(xyz), max_range_m=None)))
            elif op == 'solution':
                returned = request['poses']
                if {r['keyframe_id'] for r in returned} != set(range(len(rows))): raise ValueError('Incomplete solution')
                if any(r['robot_id'] != robot for r in returned): raise ValueError('Wrong solution owner')
                write_jsonl(root/'solution.jsonl', returned)
                response = dict(solution_sha256=file_hash(root/'solution.jsonl'), count=len(returned))
            elif op == 'close':
                connection.send_bytes(canonical(dict(closed=True, session=session))); break
            else: raise ValueError('Unknown operation')
            connection.send_bytes(canonical(dict(session=session, config_hash=backend_hash, result=response)))
    except BaseException:
        try: connection.send_bytes(canonical(dict(error=traceback.format_exc())))
        except (OSError, EOFError): pass
    finally: connection.close()


class RobotClient:
    def __init__(self, root, robot, cfg, ledger):
        self.robot = robot; self.config_hash = digest(cfg); self.session = digest([str(root), self.config_hash])
        self.ledger = ledger; self.closed = False
        context = mp.get_context('spawn'); self.connection, child = context.Pipe()
        self.process = context.Process(target=serve, args=(child, str(root), robot, cfg, self.session))
        self.process.start(); child.close()
        try:
            ready = self.receive('ready')
            if ready != dict(ready=True, session=self.session, config_hash=self.config_hash): raise ValueError('Invalid ready barrier')
        except BaseException:
            if self.process.is_alive(): self.process.terminate()
            self.process.join(5); self.connection.close(); raise

    def record(self, kind, direction, data):
        # multiprocessing.Connection uses a signed 32-bit length prefix below 2 GiB.
        framing = 4 if len(data) <= 0x7fffffff else 12
        self.ledger.append(dict(robot_id=self.robot, kind=kind, direction=direction,
            payload_bytes=len(data), framing_bytes=framing, transmitted_bytes=len(data)+framing))

    def receive(self, kind):
        if not self.connection.poll(300): raise TimeoutError('Robot service timeout')
        data = self.connection.recv_bytes(); self.record(kind, 'robot_to_server', data)
        result = json.loads(data)
        if 'error' in result: raise RuntimeError(result['error'])
        if result.get('session') != self.session: raise ValueError('Wrong response session')
        return result

    def call(self, op, **body):
        data = canonical(dict(op=op, session=self.session, config_hash=self.config_hash, **body))
        self.record(op, 'server_to_robot', data); self.connection.send_bytes(data)
        response = self.receive(op)
        if response['config_hash'] != self.config_hash: raise ValueError('Wrong response configuration')
        return response['result']

    def close(self):
        if self.closed: return
        self.closed = True
        try:
            if self.process.is_alive():
                data = canonical(dict(op='close', session=self.session, config_hash=self.config_hash))
                self.record('close', 'server_to_robot', data); self.connection.send_bytes(data); self.receive('close')
                self.process.join(10)
        finally:
            if self.process.is_alive(): self.process.terminate(); self.process.join(5)
            self.connection.close()


def communication_summary(ledger):
    by_kind = defaultdict(int); by_robot = defaultdict(int); by_direction = defaultdict(int)
    for row in ledger:
        by_kind[row['kind']] += row['transmitted_bytes']; by_robot[row['robot_id']] += row['transmitted_bytes']
        by_direction[row['direction']] += row['transmitted_bytes']
    return dict(total_bytes=sum(r['transmitted_bytes'] for r in ledger), messages=len(ledger),
        by_kind=dict(by_kind), by_robot=dict(by_robot), by_direction=dict(by_direction),
        scope='actual serialized robot/server IPC messages plus Connection framing; no TCP/IP, DDS, map-visualization upload or odometry replay traffic',
        caching='one verification geometry transfer per requested endpoint; reused by native registration preprocessing')


def centralized(roots, cfg, source, output, *, solve_graph=True):
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    (output/'server-stores').mkdir(); ledger = []; clients = {}; matchers = {}; rows = []; descriptors = {}
    events = []; loops = []; payloads = {}; verified = set(); stage = {}; started = time.perf_counter()
    usage0 = resource.getrusage(resource.RUSAGE_CHILDREN)
    with MemorySampler() as memory:
        try:
            for robot, root in sorted(roots.items()):
                client = RobotClient(root, robot, cfg['backend'], ledger); clients[robot] = client
                manifest = client.call('manifest'); rows.extend(manifest['rows'])
                descriptors[robot] = {int(k):v for k,v in manifest['descriptors'].items()}
                matchers[robot] = create(cfg['backend'])
                server_store = output/'server-stores'/robot; server_store.mkdir()
                write_jsonl(server_store/'keyframes.jsonl', manifest['rows'])
            lookup = {(r['robot_id'],r['keyframe_id']):r for r in rows}
            stage['publication_s'] = time.perf_counter()-started
            retrieval_s = verification_s = 0.
            def geometry(endpoint):
                if endpoint not in payloads:
                    packet = clients[endpoint[0]].call('geometry', key=endpoint[1])
                    value = unpack_payload(packet); row = value['row']
                    if (row['robot_id'],row['keyframe_id']) != endpoint or row != lookup[endpoint]: raise ValueError('Geometry identity mismatch')
                    payloads[endpoint] = value
                    np.savez_compressed(output/'server-stores'/endpoint[0]/f'{endpoint[1]:06d}.npz', cloud=value['cloud'])
                return payloads[endpoint]
            # All robots seal their own final geometry before staged retrieval.
            # One query per endpoint, the existing two-hypothesis per-query budget,
            # no wall-time cooldown: a sealed batch has no online sensor clock.
            ordered = sorted(rows, key=lambda r:(r['robot_id'],r['keyframe_id']))
            published = set()
            for query in ordered:
                q = (query['robot_id'],query['keyframe_id']); ranked = []
                if not query['retrievable']: published.add(q); continue
                began = time.perf_counter()
                body = dict(stamp_ns=max(r['available_ns'] for r in rows), anchor_ns=query['stamp_ns'],
                    strategy=query['strategy'], geometry_id_ranges=query['geometry_id_ranges'])
                for robot, matcher in sorted(matchers.items()):
                    index = {key:d for key,d in descriptors[robot].items() if (robot,key) in published and
                        eligible_observation(lookup[robot,key], body, robot==q[0], round(cfg['loops']['same_robot_exclusion_s']*1e9))}
                    proposals = matcher.retrieve(descriptors[q[0]][q[1]], index, cfg['loops']['top_k'])
                    ranked.extend((robot, p) for p in proposals if p['eligible'])
                ranked.sort(key=lambda p:(-p[1]['score'],p[0],p[1]['keyframe_id']))
                candidates, rejections = select_branches(ranked, cfg['loops']['branch_verification_limits'], {}, 0, 0)
                retrieval_s += time.perf_counter()-began
                events.append(dict(type='retrieval', query=list(q), candidates=[dict(robot=r,**p) for r,p in ranked], rejections=rejections))
                for robot, proposal in candidates:
                    c = (robot,proposal['keyframe_id']); pair = tuple(sorted((q,c)))
                    if pair in verified: continue
                    verified.add(pair); began = time.perf_counter()
                    result = matchers[robot].verify(geometry(q), geometry(c), proposal)
                    elapsed = time.perf_counter()-began; verification_s += elapsed
                    events.append(dict(type='verification', query=list(q), candidate=list(c), seconds=elapsed, **result))
                    if result['accepted']:
                        loops.append(canonical_edge(constraint(list(q), list(c), result['T_i_j'], result['information'],
                            kind='loop', accepted=True, backend='mapclosures', diagnostics=result)))
                published.add(q)
                print(f"query {q}: {len(ranked)} eligible, {len(candidates)} verified slots, {len(loops)} accepted total", flush=True)
            stage.update(retrieval_s=retrieval_s, verification_and_geometry_transfer_s=verification_s)
            write_jsonl(output/'events.jsonl', events); write_jsonl(output/'constraints.jsonl', loops)
            if not solve_graph:
                # The distributed circle runner consumes only verified loops.
                # Do not initialize, select loops with, or invoke a central PGO.
                return dict(stage=stage, graph_nodes=len(rows), proposed_loops=len(loops),
                    verification_count=len(verified), downloaded_endpoints=len(payloads),
                    backend='sealed descriptor retrieval and point-GICP verification only',
                    graph_solver='none; measurements passed to distributed PCM/CBS')
            began = time.perf_counter(); graph = optimize(rows, loops, cfg['pgo']); stage['pose_graph_s'] = time.perf_counter()-began
            write_json(output/'pose-graph.json', graph)
            registrations = settings(cfg['central_registration'])
            began = time.perf_counter()
            graph = refine_graph(graph, {r:output/'server-stores'/r for r in roots}, registrations, source, output)
            stage['registration_graph_s'] = time.perf_counter()-began
            write_json(output/'graph.json', graph)
            for robot, client in clients.items(): client.call('solution', poses=[r for r in graph['poses'] if r['robot_id']==robot])
        finally:
            active_error=sys.exc_info()[0];shutdown_errors=[]
            for service in [*matchers.values(),*clients.values()]:
                try:service.close()
                except Exception as error:shutdown_errors.append(str(error))
            write_jsonl(output/'communication.jsonl', ledger)
            if shutdown_errors:
                write_json(output/'shutdown-errors.json',shutdown_errors)
                if active_error is None:raise RuntimeError('Service shutdown failed: '+str(shutdown_errors))
    usage1 = resource.getrusage(resource.RUSAGE_CHILDREN)
    summary = dict(stage=stage, wall_s=time.perf_counter()-started, graph_nodes=len(rows),
        proposed_loops=len(loops), verification_count=len(verified), downloaded_endpoints=len(payloads),
        retained_loops=sum(f['kind']=='loop' and f['solver_weight']>0 for f in graph['factors']),
        registration_factors=sum(f['kind']=='registration' for f in graph['factors']), components=graph['components'],
        communication=communication_summary(ledger), peak_simultaneous_process_tree_rss_bytes=memory.peak,
        memory_sample_interval_s=.02, child_cpu_s=usage1.ru_utime+usage1.ru_stime-usage0.ru_utime-usage0.ru_stime,
        backend='existing centralized GNC-TLS pose selection then native joint LM + GLIM GPU VGICP; no CBS/DDS cycles',
        scheduling='sealed staged inputs; no online cooldown/latency claim; two candidates per query')
    write_json(output/'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True); parser.add_argument('--mode', choices=['circle','trajectory'], required=True)
    parser.add_argument('--output', required=True); parser.add_argument('--source', required=True)
    parser.add_argument('--prepared', help='Reuse immutable prepared maps; measure a fresh centralized backend only')
    args = parser.parse_args(); cfg = yaml.safe_load(Path(args.config).read_text())
    output = Path(args.output); output.mkdir(parents=True, exist_ok=False)
    write_json(output/'configuration.json', cfg)
    code=Path(__file__).resolve().parent
    write_json(output/'source-hashes.json',dict(python={str(p.relative_to(code)):file_hash(p)
        for p in sorted(code.glob('*.py'))}, native=native_provenance(args.source)))
    roots = {}; summaries = {}; began = time.perf_counter(); cpu0=time.process_time()
    usage0=resource.getrusage(resource.RUSAGE_CHILDREN)
    with (output/'gpu.csv').open('w') as gpu_log:
        observer=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,memory.used,utilization.gpu',
            '--format=csv,noheader,nounits','-lms','100'],stdout=gpu_log,stderr=subprocess.DEVNULL)
        try:
            with MemorySampler() as total_memory:
                with MemorySampler() as memory:
                    for robot, source_store in cfg['stores'].items():
                        root = output/robot; roots[robot] = root
                        t = time.perf_counter()
                        if args.prepared:
                            previous=Path(args.prepared).resolve()/robot;root.mkdir()
                            for name in ('store','descriptors','images','union.npz','placement.json','placement-history.jsonl'):
                                (root/name).symlink_to(previous/name)
                            placement=read_json(root/'placement.json')
                            if placement['mode']!=args.mode or placement['config']!=asdict(CircleCoverConfig(**cfg['circle_cover'])):
                                # JSON tuples become lists; compare canonical forms.
                                if canonical(placement['config'])!=canonical(asdict(CircleCoverConfig(**cfg['circle_cover']))) or placement['mode']!=args.mode:
                                    raise ValueError('Prepared placement profile differs')
                            summaries[robot]=read_json(Path(args.prepared)/'preparation-summary.json')['robots'][robot]
                        else:
                            placement = build(source_store, root, CircleCoverConfig(**cfg['circle_cover']), args.mode)
                            prep = prepare(root, cfg['backend'])
                            summaries[robot] = dict(placement=placement, preparation=prep, wall_s=time.perf_counter()-t)
                        print(f"prepared {robot}: {placement['footprints']} maps; coverage {placement['metrics']['coverage']:.4%}", flush=True)
                write_json(output/'preparation-summary.json', dict(robots=summaries, wall_s=time.perf_counter()-began,
                    peak_simultaneous_process_tree_rss_bytes=memory.peak,
                    reused=bool(args.prepared), original_preparation_root=args.prepared,
                    timing_scope='input setup only; per-robot preparation timers describe the original capture' if args.prepared else 'fresh preparation'))
                centralized(roots, cfg, args.source, output/'central')
        finally:
            observer.terminate(); observer.wait(timeout=10)
            usage1=resource.getrusage(resource.RUSAGE_CHILDREN)
            write_json(output/'resources.json',dict(wall_s=time.perf_counter()-began,
                parent_cpu_s=time.process_time()-cpu0,
                child_cpu_s=usage1.ru_utime+usage1.ru_stime-usage0.ru_utime-usage0.ru_stime,
                peak_simultaneous_process_tree_rss_bytes=total_memory.peak,
                rss_scope='sum of RSS at each 20 ms sample; shared pages can be counted in multiple processes',
                gpu_scope='whole device, sampled every 100 ms; includes driver and any unrelated GPU users'))


if __name__ == '__main__': main()

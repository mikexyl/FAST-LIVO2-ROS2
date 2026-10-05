#!/usr/bin/env python3
"""Fresh circle placement, point BEVs, verified loops, persistent CBS and PLY.

Reuses retained native odometry observations. Geometry is sealed at capture end;
there is no centralized graph initialization or optimization in this runner.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing as mp
from pathlib import Path
import resource
import shutil
import sys
import time
import traceback

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'research'))
from s3e_pipeline.artifacts import digest, file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.circle_cover import CircleCoverConfig
from s3e_pipeline.circle_cover_replay import MemorySampler
from s3e_pipeline.circle_submaps import build, prepare
from s3e_pipeline.central_circle import centralized, communication_summary
from s3e_pipeline.circle_cbs import run as run_cbs


def prepare_robot(robot, store, output, config):
    started = time.perf_counter()
    placement = build(store, output, CircleCoverConfig(**config['circle_cover']), 'circle')
    features = prepare(output, config['backend'])
    return robot, dict(placement=placement, preparation=features,
                      wall_s=time.perf_counter()-started)


def run(args):
    config = yaml.safe_load(args.config.read_text())
    if (config['backend']['mapclosures']['representation'] != 'point_cloud' or
            config['dpgo']['execution'] != 'persistent'):
        raise ValueError('This runner requires point-cloud BEVs and persistent CBS')
    output = args.output; output.mkdir(parents=True, exist_ok=False)
    capture = output / 'capture'; capture.mkdir()
    write_json(output / 'configuration.json', config)
    write_json(capture / 'configuration.json', config)
    code = Path(__file__).resolve().parents[2] / 'research/s3e_pipeline'
    write_json(output / 'source-hashes.json', dict(runner=file_hash(__file__),
        python={p.name:file_hash(p) for p in sorted(code.glob('*.py'))},
        odometry_reused=True, map_input='growing union of retained native area snapshots'))
    def status(phase, **extra):
        write_json(output / 'status.json', dict(phase=phase, updated=time.time(), **extra))
        print(phase, extra, flush=True)
    start = time.perf_counter(); cpu = time.process_time()
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    try:
        with MemorySampler() as memory:
            status('circle-placement-and-point-BEVs', robots=len(config['robots']))
            started = time.perf_counter(); summaries = {}
            roots = {r:capture/r for r in config['robots']}
            if args.prepared:
                previous=read_json(args.prepared/'configuration.json')
                if any(previous[k]!=config[k] for k in ('robots','backend','circle_cover')):
                    raise ValueError('Prepared capture configuration differs')
                for robot,root in roots.items():
                    root.mkdir()
                    for name in ('store','descriptors','images','union.npz','placement.json','placement-history.jsonl'):
                        (root/name).symlink_to(args.prepared.resolve()/robot/name)
                preparation=read_json(args.prepared/'preparation-summary.json')
                preparation.update(reused_after_worker_crash=True,source=str(args.prepared),
                    recovery_input_setup_s=time.perf_counter()-started)
                write_json(capture/'preparation-summary.json',preparation)
            # Independent robots run concurrently; no host CPU/RAM/GPU quotas.
            else:
                with ProcessPoolExecutor(max_workers=len(roots), mp_context=mp.get_context('spawn')) as pool:
                    jobs = [pool.submit(prepare_robot, r, config['stores'][r], roots[r], config) for r in roots]
                    for job in as_completed(jobs):
                        robot, summary = job.result(); summaries[robot] = summary
                        status('circle-placement-and-point-BEVs', completed=list(summaries),
                               robot=robot, circles=summary['placement']['footprints'])
                write_json(capture/'preparation-summary.json',dict(robots=summaries,
                    wall_s=time.perf_counter()-started, reused=False, parallel_robots=len(roots)))
            status('multilayer-retrieval-and-point-verification')
            started = time.perf_counter()
            detection = centralized(roots, config, args.source, capture/'central', solve_graph=False)
            detection.update(wall_s=time.perf_counter()-started,
                communication=communication_summary(read_jsonl(capture/'central/communication.jsonl')))
            write_json(capture/'central/summary.json', detection)
            status('distributed-PCM-and-persistent-CBS', verified_loops=detection['proposed_loops'])
            run_cbs(capture, config, args.source, output/'solution')
            status('exporting-corrected-map')
            from s3e_pipeline.circle_map_export import export
            exported = export(output/'solution', config, output/'exports')
            status('complete', maps=exported['files'], components=exported['components'])
            write_json(output/'COMPLETE.json',dict(success=True, config_sha256=digest(config),
                all_robots_connected=len(set(exported['components'].values()))==1,
                exports=exported['files']))
    except BaseException:
        status('failed', traceback=traceback.format_exc())
        raise
    finally:
        end = resource.getrusage(resource.RUSAGE_CHILDREN)
        write_json(output/'resources.json',dict(wall_s=time.perf_counter()-start,
            parent_cpu_s=time.process_time()-cpu,
            child_cpu_s=end.ru_utime+end.ru_stime-usage.ru_utime-usage.ru_stime,
            peak_simultaneous_process_tree_rss_bytes=memory.peak,
            scope='Fresh circle placement through export; retained odometry replay excluded'))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('config','source','output'): p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--prepared',type=Path,help='Resume from immutable circle preparation after a worker failure')
    run(p.parse_args())

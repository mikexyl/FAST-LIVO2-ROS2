#!/usr/bin/env python3
"""Fresh four-flight spatial EllipseLIO / ellipsoid-BEV / PCM / CBS on 148."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import shutil
from collections import Counter
import os
from pathlib import Path
import sys
import threading
import traceback

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parent
BASE = ROOT / '.ros2/graco-aerial-spatial148-20260920'
INPUT_BASE = ROOT / '.ros2/graco-aerial-four148-20260920'
PYTHON = ROOT / '.ros2/research-venv/bin/python'
RERUN = ROOT / '.ros2/rerun-venv/bin'
LIBRARY = BASE / 'install/ellipselio/lib/libellipselio_mapping.so'
LAUNCHER = BASE / 'launcher/ellipselio_mapping_mt'
FLIGHTS = dict(aerial05='aerial-05-40m', aerial06='aerial-06-20m',
               aerial07='aerial-07-25m', aerial08='aerial-08-25m')
sys.path.insert(0, str(ROOT / 'FAST-LIVO2-ROS2/research'))
from graco_aerial import call, now, read, save, sha
from full_batch import quality


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def export_truth(work):
    """Copy the verified historical references only after backend completion."""
    source = INPUT_BASE/'full/reference'
    shutil.copytree(source, work/'reference')
    provenance = read(source/'source-hashes.json')
    for robot in FLIGHTS:
        assert sha(work/'reference'/f'{robot}_gt.txt') == provenance[robot]['output_sha256']
    save(work/'reference/reuse-provenance.json', dict(source=str(source), evaluation_only=True,
        copied_after_backend=True, source_manifest_sha256=sha(source/'source-hashes.json')))


def audit(work, frozen):
    import zlib
    keys = {}
    result = dict(frontends={}, descriptor_evidence_memberships=0, causal_ranked_events=0)
    for robot, trial_path in read(work / 'trials.json').items():
        trial = Path(trial_path)
        updates = rows(trial / 'frontend/native_updates.jsonl')
        source = rows(trial / 'frontend/submaps/index.jsonl')
        complete = [r for r in source if r['complete'] and r['retrievable']]
        prepared = rows(work / f'prepared-{robot}/store/keyframes.jsonl')
        assert len(complete) == len(prepared)
        for i, (original, key) in enumerate(zip(complete, prepared)):
            assert key['keyframe_id'] == i
            assert all(key[n] == v for n, v in original.items() if n != 'keyframe_id')
            descriptor = json.loads(zlib.decompress((work / f'prepared-{robot}/ellipsoid/{i:06d}.json.zlib').read_bytes()))
            for name, value in [('submap_id', original['submap_id']),
                                ('member_scan_ids', original['member_scan_ids']),
                                ('payload_sha256', original['sha256'])]:
                assert descriptor[name] == value
            keys[robot, i] = key
            result['descriptor_evidence_memberships'] += 1
        assert all(r['age_max_s'] <= 120.000001 for r in updates if r['lidar_updated'])
        assert all(r['schema_version'] == 3 and r['strategy'] == 'spatial' for r in source)
        assert all(r['gravity_source'] == 'ellipselio_filter_at_anchor' for r in source)
        profile = yaml.safe_load((SCRIPTS/'spatial_submaps.yaml').read_text())
        assert all(all(np.isclose(r[n], profile['spatial'][n]) for n in ('radius_m', 'overlap_m', 'max_age_s'))
                   and r['distance_metric'] == 'horizontal' for r in source)
        current_config = yaml.safe_load((work/'configs'/f'{robot}.yaml').read_text())
        old_config = yaml.safe_load((INPUT_BASE/'full/configs'/f'{robot}.yaml').read_text())
        assert current_config['/**']['ros__parameters']['mapping'].pop('submaps') == profile
        old_config['/**']['ros__parameters']['mapping'].pop('submaps')
        assert current_config == old_config, 'Calibration or frontend settings changed unexpectedly'
        assert 'verified without error' in (work / f'{robot}-recording-verification.log').read_text()
        memory = rows(trial / 'memory.jsonl')
        result['frontends'][robot] = dict(quality=read(work / f'{robot}-quality.json'),
            validation=read(trial / 'artifact-validation.json'),
            completed_submaps=len(complete), partial_submaps=len(source)-len(complete),
            max_correspondence_age_s=max(r['age_max_s'] for r in updates if r['lidar_updated']),
            processing_mean_s=float(np.mean([r['processing_s'] for r in updates])),
            processing_p95_s=float(np.quantile([r['processing_s'] for r in updates], .95)),
            peak_mapper_rss_mib=max(r['VmHWM'] for r in memory if 'VmHWM' in r)/1024,
            finish_reasons=dict(Counter(r['finish_reason'] for r in complete)),
            completed_extents_m=[r['extent_m'] for r in complete],
            completed_durations_s=[(r['end_ns']-r['begin_ns'])/1e9 for r in complete],
            live_rrd_sha256=sha(trial / 'recording/live.rrd'))
    for robot in FLIGHTS:
        for event in rows(work / f'dpgo/{robot}/events.jsonl'):
            if event['type'] != 'ranked':
                continue
            assert event['query_stamp_ns'] == keys[tuple(event['query'])]['available_ns'] <= event['delivery_ns']
            assert all(keys[event['candidate_robot'], c['keyframe_id']]['available_ns'] <= event['delivery_ns']
                       for c in event['candidates'])
            result['causal_ranked_events'] += 1
    for edge in rows(work / 'dpgo/constraints.jsonl'):
        a, b = keys[tuple(edge['i'])], keys[tuple(edge['j'])]
        if edge['i'][0] == edge['j'][0]:
            assert not set(a['member_scan_ids']).intersection(b['member_scan_ids'])
            assert abs(a['stamp_ns']-b['stamp_ns']) >= 30*10**9
    graph = rows(work / 'dpgo/poses.jsonl')
    assert len(graph) == len(keys)
    for node in graph:
        assert node['stamp_ns'] == keys[node['robot_id'], node['keyframe_id']]['stamp_ns']
        assert np.isfinite(node['T_world_body']).all()
    assert 'verified without error' in (work / 'recording-verification.log').read_text()
    assert all(sha(Path(p)) == h for p, h in frozen.items())
    result.update(frozen_sources_verified=True, derived_rerun_verified=True,
                  derived_rerun_sha256=sha(work / 'report/result.rrd'), all_bulk_geometry_retained=True)
    save(work / 'retention-audit.json', result)


def run(work):
    work.mkdir(parents=True, exist_ok=False)
    state = dict(phase='preflight', started_utc=now(), host=os.uname().nodename, frontend_workers=4,
                 robots={r: dict(status='queued') for r in FLIGHTS})
    lock = threading.Lock()
    def mark(phase=None, robot=None, **fields):
        with lock:
            if phase:
                state['phase'] = phase
            (state['robots'][robot] if robot else state).update(fields)
            state['updated_utc'] = now()
            save(work / 'status.json', state)
    try:
        assert (BASE / 'BUILD_COMPLETE').exists()
        assert read(INPUT_BASE / 'staging-status.json')['phase'] == 'complete'
        import s3e_mapclosures_native
        from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
        sampler = SurfaceSampler()
        try:
            sampler.render(np.array([[0., 0., 5.]]), np.array([[.2, .2, .2]]), np.eye(3)[None])
        finally:
            sampler.close()
        watched = {p.resolve() for folder in (ROOT/'ellipselio', SCRIPTS,
            ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline') for p in folder.rglob('*')
            if p.is_file() and p.suffix in ('.py', '.cpp', '.h', '.hpp', '.msg', '.yaml', '.sh', '.txt')}
        watched.update([LIBRARY, LAUNCHER, ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',
            ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py', ROOT/'FAST-LIVO2-ROS2/src/research_export.cpp',
            ROOT/'FAST-LIVO2-ROS2/include/research_export.h', ROOT/'FAST-LIVO2-ROS2/scripts/run_dpgo_ros.sh',
            ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so', Path(s3e_mapclosures_native.__file__),
            ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so', ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node'])
        frozen = {str(p): sha(p) for p in sorted(watched)}
        save(work/'source-hashes.json', frozen)
        save(work/'calibration-hashes.json', {str(p): sha(p) for p in (INPUT_BASE/'calibration').glob('*.yaml')})
        cfg = yaml.safe_load((SCRIPTS/'rollout.yaml').read_text())
        cfg.update(robots=list(FLIGHTS), dataset=str(work/'reference'), output_root=str(work),
                   experiment_name='GRACO aerial 05/06/07/08 on 148, 40 m / 20 m horizontal spatial submaps')
        cfg['dpgo'].update(ros_domain_id=208, timeout_s=1800)
        cfg['backend']['mapclosures']['local_map'] = 'Completed native spatial submaps; 40 m radius, 20 m overlap, horizontal extent; last-member IMU frame'
        cfg['evaluation'].update(ground_truth_format='graco_imu_enu', expected_connected_robots=list(FLIGHTS),
            trajectory_limitation='GRACO RTK/INS T_Base_Imu positions; original timestamps; rigid alignment without scale')
        (work/'config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        (work/'configs').mkdir()
        (work/'frontends').mkdir()
        trials = {r: str(work/'frontends'/r) for r in FLIGHTS}
        save(work/'trials.json', trials)
        for robot in FLIGHTS:
            mapping = yaml.safe_load((INPUT_BASE/'full/configs'/f'{robot}.yaml').read_text())
            p = mapping['/**']['ros__parameters']
            p.pop('research', None)
            p['mapping']['submaps'] = yaml.safe_load((SCRIPTS/'spatial_submaps.yaml').read_text())
            p['publish'] = dict(map=True, scan=True, markers=True, odometry=True, analytics=True, tf=True)
            (work/'configs'/f'{robot}.yaml').write_text(yaml.safe_dump(mapping, sort_keys=False))
        def frontend(item):
            index, robot = item
            try:
                sensors = INPUT_BASE/'inputs'/robot
                meta = yaml.safe_load((sensors/'metadata.yaml').read_text())['rosbag2_bagfile_information']
                duration = meta['duration']['nanoseconds']/1e9
                env = dict(os.environ, ROS_DOMAIN_ID=str(200+2*index))
                mark(robot=robot, status='recording', sensor_duration_s=duration, ros_domain_id=200+2*index)
                call(['/usr/bin/python3', SCRIPTS/'run_trial.py', robot, '--robot', robot, '--enabled', '--submap-strategy', 'spatial',
                      '--bag', sensors, '--mapping-config', work/'configs'/f'{robot}.yaml',
                      '--output-root', work/'frontends', '--mapper-executable', LAUNCHER,
                      '--mapping-library', LIBRARY], work/f'{robot}-frontend.log', duration+240, env)
                trial = Path(trials[robot])
                mark(robot=robot, status='validating')
                call([PYTHON, SCRIPTS/'validate.py', trial], work/f'{robot}-validation.log', 300, env)
                call(['/usr/bin/python3', SCRIPTS/'sensor_coverage.py', trial], work/f'{robot}-coverage.log', 120, env)
                call([RERUN/'rerun', 'rrd', 'verify', trial/'recording/live.rrd'],
                     work/f'{robot}-recording-verification.log', 300, env)
                q = quality(trial)
                save(work/f'{robot}-quality.json', q)
                assert q['full_completion'] and q['finite'] and q['chronological'] and q['max_speed_m_s'] <= 20
                assert read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
                mark(robot=robot, status='complete', quality=q, diagnostic_only=not q['passed'])
            except Exception as error:
                save(work/f'{robot}-failure.json', dict(error=repr(error), traceback=traceback.format_exc()))
                mark(robot=robot, status='failed', error=repr(error))
        mark('frontends')
        with ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(frontend, enumerate(FLIGHTS)))
        assert all(r['status'] == 'complete' for r in state['robots'].values()), 'Frontend completion/validity failure'
        qualities = {r: read(work/f'{r}-quality.json') for r in FLIGHTS}
        diagnostic = any(not q['passed'] for q in qualities.values())
        save(work/'frontend-gate.json', dict(passed=not diagnostic, quality=qualities,
            diagnostic_backend=diagnostic, policy='Completed finite chronological captures with speed <=20 m/s '
            'and verified native artifacts can enter the requested backend diagnostically; update gaps >=1 s remain failed gates.'))
        assert all(sha(Path(p)) == h for p, h in frozen.items())
        mark('descriptors', diagnostic_only=diagnostic)
        artifacts = {}
        for robot in FLIGHTS:
            mark(active_robot=robot)
            output = work/f'prepared-{robot}'
            call([PYTHON, '-m', 's3e_pipeline.recent_submaps', '--source', Path(trials[robot])/'frontend/submaps',
                  '--output', output, '--config', work/'config.yaml'], work/f'{robot}-descriptors.log', 1800)
            artifacts[f'keyframes.ellipselio.{robot}'] = str(output)
            artifacts[f'descriptors.ellipselio.mapclosures.{robot}'] = str(output/'ellipsoid')
        save(work/'artifacts.json', artifacts)
        mark('distributed_pcm_cbs')
        call([PYTHON, Path(__file__), 'backend', '--work', work], work/'backend.log', cfg['dpgo']['timeout_s']+120)
        mark('evaluation')
        export_truth(work)
        call([PYTHON, SCRIPTS/'rollout_report.py', '--work', work, '--trials', work/'trials.json'], work/'evaluation.log', 1800)
        report = read(work/'report/report.json')
        report.update(diagnostic_only=diagnostic, frontend_quality=qualities)
        save(work/'report/report.json', report)
        call([RERUN/'python', SCRIPTS/'rollout_rerun.py', work], work/'rerun.log', 300)
        call([RERUN/'rerun', 'rrd', 'verify', work/'report/result.rrd'], work/'recording-verification.log', 300)
        mark('auditing')
        audit(work, frozen)
        mark('complete', finished_utc=now(), frozen_sources_verified=True,
             raw_ate={r: x['rmse_m'] for r, x in report['raw'].items()},
             shared_cbs_ate={r: x['rmse_m'] for r, x in report['cbs'].items()},
             connectivity=report['connectivity'], loops=report['runtime']['loops'], pcm_rejected=report['pcm']['excluded_loops'])
    except Exception as error:
        save(work/'failure.json', dict(stage=state['phase'], error=repr(error), traceback=traceback.format_exc()))
        mark('failed', finished_utc=now(), error=repr(error))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['run', 'backend'])
    parser.add_argument('--work', type=Path, default=BASE/'full')
    args = parser.parse_args()
    if args.stage == 'run':
        run(args.work.resolve())
    else:
        from s3e_pipeline.dpgo import run as backend
        (args.work/'dpgo').mkdir(exist_ok=False)
        backend(yaml.safe_load((args.work/'config.yaml').read_text()), read(args.work/'artifacts.json'), ROOT, args.work/'dpgo')

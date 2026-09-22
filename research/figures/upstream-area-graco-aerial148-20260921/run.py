"""Fresh GRACO aerial 05–08, updated persistent odometry and accumulated area backend."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import sqlite3
import sys
import threading
import time
import traceback
import zlib
import numpy as np
import yaml
ROOT=Path('/workspace');BASE=ROOT/'.ros2/upstream-area-graco-aerial148-20260921'
PREV=ROOT/'.ros2/upstream-area-s3e-20260921';OLD=ROOT/'.ros2/graco-aerial-four148-20260920'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(SCRIPTS),str(ROOT/'FAST-LIVO2-ROS2/research'),str(PREV)]
import upstream_area_batch as u
u.LIBRARY=PREV/'capacity-run/install/ellipselio/lib/libellipselio_mapping.so'
u.LAUNCHER=PREV/'capacity-run/launcher/ellipselio_mapping_mt'
from upstream_area_batch import call,read,save,sha,quality,PYTHON,RERUN
from consolidate import capture
from s3e_pipeline.area_maps import shared_ids
FLIGHTS=dict(aerial05='aerial-05-40m',aerial06='aerial-06-20m',aerial07='aerial-07-25m',aerial08='aerial-08-25m')

def rows(path):return [json.loads(l) for l in Path(path).read_text().splitlines()]
def now():return time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
def sources():
    result=u.sources();result[str(Path(__file__).relative_to(ROOT))]=sha(Path(__file__))
    result[str(BASE.relative_to(ROOT)/'env.sh')]=sha(BASE/'env.sh')
    return result


def prepare(work):
    work.mkdir(parents=True,exist_ok=False);(work/'configs').mkdir();(work/'frontends').mkdir()
    assert sha(u.LIBRARY)==read(PREV/'capacity-run/finished.json')['library_sha256']
    assert read(PREV/'capacity-cuda/test-result.json')['bitwise_centroid_equality']
    assert read(OLD/'staging-status.json')['phase']=='complete'
    cfg=yaml.safe_load((SCRIPTS/'area_rollout.yaml').read_text())
    cfg.update(robots=list(FLIGHTS),dataset=str(work/'reference'),output_root=str(work),
               experiment_name='GRACO aerial 05–08: updated persistent EllipseLIO and accumulated-area MapClosures')
    cfg['dpgo'].update(ros_domain_id=220,timeout_s=1800)
    cfg['evaluation'].update(ground_truth_format='graco_imu_enu',expected_connected_robots=list(FLIGHTS),
        trajectory_limitation='GRACO RTK/INS T_Base_Imu positions; original timestamps; rigid alignment without scale')
    inputs={};configs={};trials={}
    for robot in FLIGHTS:
        bag=OLD/'inputs'/robot
        assert (bag/'COMPLETE.json').exists()
        meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        assert {t['topic_metadata']['name'] for t in meta['topics_with_message_count']}=={'/velodyne/points','/gnss/imu'}
        with sqlite3.connect(f'file:{bag/"sensors.db3"}?mode=ro',uri=True) as db:
            assert {t[0] for t in db.execute('SELECT name FROM topics')}=={'/velodyne/points','/gnss/imu'}
            assert dict(db.execute('SELECT t.name,count(*) FROM messages m JOIN topics t ON t.id=m.topic_id GROUP BY t.name'))==read(OLD/'staging-status.json')['robots'][robot]['counts']
        inputs[robot]=dict(path=str(bag),sha256={p.name:sha(p) for p in bag.iterdir() if p.is_file()},
            sensor_duration_s=meta['duration']['nanoseconds']/1e9,conversion_audit=read(OLD/f'{robot}-conversion-audit.json'),
            reuse_scope='Previously validated sensor conversion only; odometry and backend captures are fresh')
        mapping=yaml.safe_load((OLD/'full/configs'/f'{robot}.yaml').read_text());p=mapping['/**']['ros__parameters']
        p.pop('research',None);p['mapping']['submaps']=dict(enabled=False)
        p['mapping']['area_maps']=dict(yaml.safe_load((SCRIPTS/'area_maps.yaml').read_text()),odometry=False)
        assert p['input']['reliable'] and p['imu']['rate']==125
        configs[robot]=p['imu'];trials[robot]=str(work/'frontends'/robot)
        (work/'configs'/f'{robot}.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
    cfg['odometry']['imu_noise']=dict((k,configs['aerial05'][k]) for k in ('acc_noise','gyr_noise','acc_bias','gyr_bias'))
    assert all(all(v[k]==configs['aerial05'][k] for k in cfg['odometry']['imu_noise']) for v in configs.values())
    (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    save(work/'trials.json',trials);save(work/'inputs.json',inputs)
    save(work/'calibration-hashes.json',{str(p):sha(p) for p in (OLD/'calibration').glob('*.yaml')})
    frozen=sources();save(work/'source-hashes.json',frozen);save(BASE/'source-hashes.json',frozen)
    return cfg,trials,inputs,frozen


def audit(work,frozen):
    cfg=yaml.safe_load((work/'config.yaml').read_text());trials=read(work/'trials.json');keys={}
    result=dict(frontends={},descriptor_evidence_memberships=0,causal_ranked_events=0,same_robot_loop_checks=0)
    for robot,path in trials.items():
        trial=Path(path);stats=capture(trial)
        log=work/f'{robot}-recording-verification.log'
        if (trial/'recording/entity-parts/result.json').exists():
            recording=read(trial/'recording/entity-parts/result.json')
            assert recording['partitioned_footer_and_data_verified'] and recording['normalized_complete_data_comparison_passed']
            assert recording['source_sha256']==sha(trial/'recording/live.rrd')
            stats['live_recording']=recording
        else:
            assert 'verified without error' in log.read_text()
            stats['live_recording']=dict(path=str(trial/'recording/live.rrd'),sha256=sha(trial/'recording/live.rrd'),verified=True)
        original=[x for x in rows(trial/'frontend/area_maps/index.jsonl') if x['complete'] and x['retrievable']]
        prepared=rows(work/f'prepared-{robot}/store/keyframes.jsonl');assert len(original)==len(prepared)
        for i,(area,key) in enumerate(zip(original,prepared)):
            assert key['keyframe_id']==i and all(key[k]==v for k,v in area.items() if k!='keyframe_id')
            packet=json.loads(zlib.decompress((work/f'prepared-{robot}/ellipsoid/{i:06d}.json.zlib').read_bytes()))
            assert packet['member_scan_ids']==area['member_scan_ids'] and packet['payload_sha256']==area['sha256']
            assert packet['submap_id']==area['submap_id'] and key['submap_points']==area['geometry_count']
            keys[robot,i]=key;result['descriptor_evidence_memberships']+=1
        stats['descriptor_preparation']=read(work/f'prepared-{robot}/summary.json');result['frontends'][robot]=stats
    for robot in FLIGHTS:
        for event in rows(work/f'dpgo/{robot}/events.jsonl'):
            if event['type']!='ranked':continue
            assert event['query_stamp_ns']==keys[tuple(event['query'])]['available_ns']<=event['delivery_ns']
            assert all(keys[event['candidate_robot'],c['keyframe_id']]['available_ns']<=event['delivery_ns'] for c in event['candidates'])
            result['causal_ranked_events']+=1
    for edge in rows(work/'dpgo/constraints.jsonl'):
        a,b=keys[tuple(edge['i'])],keys[tuple(edge['j'])]
        if edge['i'][0]==edge['j'][0]:
            assert abs(a['stamp_ns']-b['stamp_ns'])>=30*10**9
            assert not shared_ids(a['geometry_id_ranges'],b['geometry_id_ranges'])
            result['same_robot_loop_checks']+=1
    graph=rows(work/'dpgo/poses.jsonl');assert len(graph)==len(keys)
    for node in graph:
        assert node['stamp_ns']==keys[node['robot_id'],node['keyframe_id']]['stamp_ns'] and np.isfinite(node['T_world_body']).all()
    assert 'verified without error' in (work/'recording-verification.log').read_text()
    assert sources()==frozen
    for info in read(work/'inputs.json').values():
        assert all(sha(Path(info['path'])/p)==h for p,h in info['sha256'].items())
    result.update(frozen_sources_verified=True,input_bags_unchanged=True,derived_rerun_verified=True,
        derived_rerun_sha256=sha(work/'report/result.rrd'),all_bulk_geometry_retained=True,
        processing_timing_scope='Native undistortion, LiDAR update, map insertion and area snapshot/export; excludes input synchronization and ROS publication',
        upstream_revision='6506f46f1947b4ef86cfba402f11f10a6ef520ee',
        source_integration='Same verified persistent-map integration and octree/CUDA storage fixes as S3E batch; no estimator or backend threshold changes')
    save(work/'retention-audit.json',result)
    return result


def run(work):
    started=time.monotonic();cfg,trials,inputs,frozen=prepare(work)
    state=dict(phase='frontends',started_utc=now(),frontend_workers=4,robots={r:dict(status='queued') for r in FLIGHTS})
    lock=threading.Lock()
    def mark(phase=None,robot=None,**values):
        with lock:
            if phase:state['phase']=phase
            (state['robots'][robot] if robot else state).update(values);state['updated_utc']=now();save(work/'status.json',state)
            print(phase or robot,values,flush=True)
    try:
        mark('frontends')
        def frontend(item):
            index,robot=item;trial=Path(trials[robot]);env=dict(os.environ,ROS_DOMAIN_ID=str(224+2*index))
            try:
                mark(robot=robot,status='recording',ros_domain_id=224+2*index)
                call(['/usr/bin/python3',SCRIPTS/'run_trial.py',robot,'--robot',robot,'--area-maps','--persistent-odometry',
                    '--bag',inputs[robot]['path'],'--mapping-config',work/'configs'/f'{robot}.yaml',
                    '--output-root',work/'frontends','--mapper-executable',u.LAUNCHER,'--mapping-library',u.LIBRARY],
                    work/f'{robot}-frontend.log',inputs[robot]['sensor_duration_s']+300,env)
                mark(robot=robot,status='validating')
                call([PYTHON,SCRIPTS/'validate.py',trial],work/f'{robot}-validation.log',600,env)
                call(['/usr/bin/python3',SCRIPTS/'sensor_coverage.py',trial],work/f'{robot}-coverage.log',120,env)
                q=quality(trial);save(work/f'{robot}-quality.json',q)
                assert q['full_completion'] and q['finite'] and q['chronological'] and q['max_speed_m_s']<=20,q
                assert read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
                try:call([RERUN/'rerun','rrd','verify',trial/'recording/live.rrd'],work/f'{robot}-recording-verification.log',300,env)
                except RuntimeError:
                    assert 'TooManyTables' in (work/f'{robot}-recording-verification.log').read_text()
                    call([PYTHON,PREV/'partition_recording.py',trial],work/f'{robot}-recording-partition.log',1800,env)
                mark(robot=robot,status='complete',quality=q)
            except Exception as exc:
                save(work/f'{robot}-failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
                mark(robot=robot,status='failed',error=repr(exc))
        with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(frontend,enumerate(FLIGHTS)))
        assert all(v['status']=='complete' for v in state['robots'].values()),'Frontend completion/validity failure'
        qualities={r:read(work/f'{r}-quality.json') for r in FLIGHTS};diagnostic=any(not q['passed'] for q in qualities.values())
        save(work/'frontend-gate.json',dict(passed=not diagnostic,quality=qualities,diagnostic_backend=diagnostic,
            policy='Completed finite chronological captures with speed <=20 m/s may enter backend diagnostically; gaps >=1 s remain failed stability checks'))
        assert sources()==frozen
        mark('descriptors',diagnostic_only=diagnostic)
        def descriptor(robot):
            target=work/f'prepared-{robot}'
            call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',Path(trials[robot])/'frontend/area_maps',
                '--output',target,'--config',work/'config.yaml'],work/f'{robot}-descriptors.log',1800)
        with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(descriptor,FLIGHTS))
        artifacts={}
        for r in FLIGHTS:
            artifacts[f'keyframes.ellipselio.{r}']=str(work/f'prepared-{r}')
            artifacts[f'descriptors.ellipselio.mapclosures.{r}']=str(work/f'prepared-{r}/ellipsoid')
        save(work/'artifacts.json',artifacts);mark('distributed_pcm_cbs')
        call([PYTHON,SCRIPTS/'upstream_area_batch.py','backend','--work',work],work/'backend.log',cfg['dpgo']['timeout_s']+120)
        mark('evaluation')
        # First GT access: after optimization. Reuse hash-verified original IMU-frame reference exports.
        truth=read(OLD/'full/reference/source-hashes.json')
        shutil.copytree(OLD/'full/reference',work/'reference')
        for r,item in truth.items():assert sha(work/'reference'/f'{r}_gt.txt')==item['output_sha256']
        call([PYTHON,SCRIPTS/'rollout_report.py','--work',work,'--trials',work/'trials.json'],work/'evaluation.log',1800)
        report=read(work/'report/report.json');report.update(diagnostic_only=diagnostic,frontend_quality=qualities);save(work/'report/report.json',report)
        call([RERUN/'python',SCRIPTS/'rollout_rerun.py',work],work/'rerun.log',300)
        call([RERUN/'rerun','rrd','verify',work/'report/result.rrd'],work/'recording-verification.log',300)
        mark('auditing');audit(work,frozen)
        mark('complete',finished_utc=now(),wall_s=time.monotonic()-started,raw_ate={r:x['rmse_m'] for r,x in report['raw'].items()},
            individual_cbs_ate={r:x['rmse_m'] for r,x in report['cbs_individual'].items()},
            shared_cbs_ate={r:x['rmse_m'] for r,x in report['cbs'].items()},connectivity=report['connectivity'],
            loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'])
    except Exception as exc:
        save(work/'failure.json',dict(stage=state['phase'],error=repr(exc),traceback=traceback.format_exc()))
        mark('failed',error=repr(exc));raise

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,default=BASE/'full');run(p.parse_args().work)

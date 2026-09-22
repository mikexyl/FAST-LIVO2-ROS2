"""Backend admission uses sensor validity, never raw ATE or GT availability."""
import importlib.util
import json
from pathlib import Path
import pytest


def module():
    path=Path(__file__).resolve().parents[2]/'scripts/recent_submaps/full_batch.py'
    spec=importlib.util.spec_from_file_location('full_submap_batch_test',path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


def trial(root,dt=.1,speed=1.,complete=True):
    (root/'frontend').mkdir()
    (root/'frontend/summary.json').write_text(json.dumps(dict(success=complete,requested_duration=0)))
    rows=[dict(stamp_ns=10**9+round(i*dt*1e9),sensor_stamp_ns=10**9+round(i*dt*1e9),
               pose=[i*dt*speed,0,0,0,0,0,1],lidar_updated=i>0) for i in range(12)]
    (root/'frontend/native_updates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))


def test_backend_admission_does_not_gate_on_accuracy(tmp_path):
    trial(tmp_path)
    assert module().quality(tmp_path)['passed']  # No GT or ATE artifact exists.
    (tmp_path/'evaluation').mkdir()
    (tmp_path/'evaluation/metrics.json').write_text(json.dumps(dict(ate_rmse_m=1000.,gate_pass=False)))
    result=module().quality(tmp_path)
    assert result['passed'] and result['ground_truth_used'] is False


@pytest.mark.parametrize('kwargs',[dict(dt=1.1),dict(speed=21.),dict(complete=False)])
def test_backend_rejects_unstable_or_incomplete_frontend(tmp_path,kwargs):
    trial(tmp_path,**kwargs)
    assert not module().quality(tmp_path)['passed']


@pytest.mark.parametrize('workers',[3,6])
def test_parallel_frontends_have_distinct_domains_and_finish_before_backend(tmp_path,monkeypatch,workers):
    import threading
    import yaml
    batch=module();folder=tmp_path/'group';folder.mkdir()
    robots=list('ABCDEF'[:workers])
    trials={r:dict(path=str(folder/'frontends'/r),reused=False,bag='sensor-bag',config='mapping.yaml',duration_s=1) for r in robots}
    (tmp_path/'plan.json').write_text(json.dumps([dict(name='group',folder=str(folder),trials=trials)]))
    (tmp_path/'source-hashes.json').write_text('{}')
    (folder/'config.yaml').write_text(yaml.safe_dump(dict(dpgo=dict(timeout_s=1))))
    monkeypatch.setattr(batch,'sources',lambda:{})
    monkeypatch.setattr(batch,'quality',lambda _:dict(passed=True,ground_truth_used=False))
    monkeypatch.setattr(batch.signal,'signal',lambda *_:None)
    barrier=threading.Barrier(workers);domains=set();done=set();lock=threading.Lock()
    def call(command,log,timeout,env=None):
        cmd=list(map(str,command))
        if cmd[1].endswith('run_trial.py'):
            robot=cmd[2]
            with lock:domains.add(env['ROS_DOMAIN_ID'])
            barrier.wait(timeout=5)
            path=Path(trials[robot]['path']);path.mkdir(parents=True,exist_ok=True)
            (path/'sensor-coverage.json').write_text('{"full_selected_sensor_tail_reached":true}')
            with lock:done.add(robot)
        elif 's3e_pipeline.recent_submaps' in cmd:
            assert done==set(robots)
            raise RuntimeError('Synthetic backend stop after validating scheduling')
    monkeypatch.setattr(batch,'call',call)
    batch.run(tmp_path,workers=workers)
    assert domains=={str(90+2*i) for i in range(workers)}
    state=json.loads((tmp_path/'status.json').read_text())
    assert all(v['status']=='complete' for v in state['groups'][0]['frontends'].values())
    assert state['groups'][0]['failed_stage']=='descriptors'

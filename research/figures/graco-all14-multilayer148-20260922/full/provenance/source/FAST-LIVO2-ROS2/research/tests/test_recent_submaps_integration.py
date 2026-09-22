"""Use actual isolated retrieval and optional native PCM/CBS with submap endpoints."""
import os
from pathlib import Path
import pytest


@pytest.mark.parametrize('strategy',['temporal','spatial','coverage','area'])
def test_submap_three_worker_retrieval_is_causal(tmp_path,monkeypatch,strategy):
    import test_mapclosures as fixture
    original_rows=fixture.write_jsonl;original_json=fixture.write_json
    def rows(path,values):
        if Path(path).name=='keyframes.jsonl':
            values=[dict(r,submap_id=0,strategy=strategy,member_scan_ids=list(range(50)),sha256='source',
                         begin_ns=0,end_ns=2*10**9,available_ns=6*10**9,retrievable=True) for r in values]
        return original_rows(path,values)
    def descriptor(path,value):
        if Path(path).name=='000000.json':
            value=dict(value,submap_id=0,member_scan_ids=list(range(50)),payload_sha256='source')
        return original_json(path,value)
    monkeypatch.setattr(fixture,'write_jsonl',rows);monkeypatch.setattr(fixture,'write_json',descriptor)
    if strategy in ('coverage','area'):
        import copy
        cfg=copy.deepcopy(fixture.CFG)
        cfg['backend']['evidence']=dict(sampling='fixed',voxel_m=.4)
        cfg['backend']['registration'].update(sampling='fixed',voxel_m=.4)
        if strategy=='area':
            cfg['backend']['evidence']['max_range_m']=None
            cfg['backend']['registration']['max_range_m']=None
        monkeypatch.setattr(fixture,'CFG',cfg)
    fixture.test_native_three_worker_replay_cold_determinism_and_lidar_only(tmp_path,True)
    events=fixture.read_jsonl(tmp_path/'first/events.jsonl')
    assert all(e['query_stamp_ns']>=6*10**9 for e in events if 'query_stamp_ns' in e)


@pytest.mark.skipif(os.environ.get('S3E_TEST_DDS')!='1',reason='Requires native CBS DDS overlay')
@pytest.mark.parametrize('strategy',['temporal','spatial','coverage','area'])
def test_submap_endpoints_native_pcm_cbs_registration(tmp_path,monkeypatch,strategy):
    import test_cbs_integration as fixture
    original=fixture.write_jsonl
    def rows(path,values):
        if Path(path).name=='keyframes.jsonl':
            values=[dict(r,submap_id=r['keyframe_id'],strategy=strategy,available_ns=r['stamp_ns']+200000000,
                         begin_ns=r['stamp_ns']-10000000000,end_ns=r['stamp_ns']+100000000,
                         member_scan_ids=list(range(r['keyframe_id']*50,r['keyframe_id']*50+100)),retrievable=True)
                    for r in values]
        return original(path,values)
    monkeypatch.setattr(fixture,'write_jsonl',rows)
    if strategy=='area':
        original_run=fixture.run
        def run(cfg,*args,**kwargs):
            cfg['dpgo']['registration_factors']['max_range_m']=None
            return original_run(cfg,*args,**kwargs)
        monkeypatch.setattr(fixture,'run',run)
    fixture.exercise_native(tmp_path,False,True,registration=True)

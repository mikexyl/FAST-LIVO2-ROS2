from pathlib import Path
import sys
import numpy as np
import yaml
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import s3e_mapclosures_native as native
from s3e_pipeline.backends import create,pack_array
from s3e_pipeline.distributed import replay,select_branches
from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,write_jsonl
from s3e_pipeline.geometry import inv,transform,pose
from scipy.spatial.transform import Rotation

CFG=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/square1-mapclosures.yaml').read_text())

def features():
    rng=np.random.default_rng(17)
    return dict(xy=rng.uniform(-80,80,(60,2)),bits=rng.integers(0,256,(60,32),dtype=np.uint8),ground=np.eye(4))

def descriptor(f,visual):
    return dict(visual=pack_array(np.asarray(visual,dtype=np.float32)),
                mapclosures={k:pack_array(v) for k,v in f.items()})

def test_native_pose_direction_ground_frames_and_causal_filter():
    engine=native.MapClosures(.5,.05,50);ref=features();q=features()
    planar=np.eye(4);planar[:3,:3]=Rotation.from_euler('z',.43).as_matrix();planar[:3,3]=[13,-7,0]
    q['xy']=ref['xy']@planar[:2,:2].T+planar[:2,3]/.5
    q['ground'][:3,:3]=Rotation.from_euler('xy',[.03,-.04]).as_matrix();q['ground'][2,3]=1.2
    ref['ground'][:3,:3]=Rotation.from_euler('xy',[-.02,.05]).as_matrix();ref['ground'][2,3]=.8
    truth=inv(q['ground'])@planar@ref['ground'];engine.add(4,ref)
    result=engine.query(q,[4],20)[0]
    assert result['inliers']==60 and result['valid_pose']
    assert np.allclose(result['T_i_j'],truth,atol=1e-5)
    assert np.allclose(engine.pair(ref,q)['T_i_j'],inv(truth),atol=1e-5)
    assert engine.query(q,[],20)==[]
    assert np.array_equal(engine.query(q,[4],20)[0]['T_i_j'],result['T_i_j'])

def test_independent_lidar_retrieval_despite_orthogonal_visuals():
    backend=create(CFG['backend']);f=features()
    q=descriptor(f,[1,0]);candidate=descriptor(f,[0,1])
    result=backend.retrieve(q,{7:candidate},20)[0]
    assert result['visual_similarity']==0 and result['sources']==['mapclosures']
    assert result['eligible'] and result['mapclosures_hypothesis']['inliers']==60

def test_empty_density_features_are_a_reportable_failure():
    engine=native.MapClosures(.5,.05,50)
    # A uniform plane has no useful density features, not an indexing crash.
    x,y=np.meshgrid(np.arange(20.),np.arange(20.));cloud=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    f=engine.describe(cloud);assert len(f['xy'])==0
    engine.add(0,f);assert engine.query(f,[0],20)==[]
    assert engine.pair(f,f)['inliers']==0

def test_independent_budgets_and_union_deduplication():
    candidates=[('Bob',dict(keyframe_id=0,sources=['mapclosures'],branch_scores={'mapclosures':10})),
                ('Carol',dict(keyframe_id=1,sources=['megaloc'],branch_scores={'megaloc':.9}))]
    selected,_=select_branches(candidates,{'mapclosures':1,'megaloc':1},{},10**9,2*10**9)
    assert len(selected)==2
    both=[('Bob',dict(keyframe_id=2,sources=['mapclosures','megaloc'],branch_scores={'mapclosures':20,'megaloc':.95}))]
    selected,_=select_branches(both,{'mapclosures':1,'megaloc':1},{},10**9,2*10**9)
    assert len(selected)==1 and selected[0][1]['selected_branches']==['mapclosures','megaloc']

@pytest.mark.parametrize('lidar_only',[False,True])
def test_native_three_worker_replay_cold_determinism_and_lidar_only(tmp_path,lidar_only):
    stores={};descs={};rng=np.random.default_rng(12)
    parts=[]
    for axis in range(3):
        points=rng.uniform(-5,5,(1000,3));points[:,axis]=0;parts.append(points)
    cloud=np.vstack(parts).astype(np.float32)
    for i,robot in enumerate(['Alpha','Bob','Carol']):
        store=tmp_path/robot;store.mkdir();stores[robot]=store
        root=tmp_path/(robot+'-descriptors');root.mkdir();descs[robot]=root
        row=dict(robot_id=robot,keyframe_id=0,stamp_ns=10**9,T_world_body=np.eye(4).tolist())
        if lidar_only:row['image_available']=False
        write_jsonl(store/'keyframes.jsonl',[row]);np.savez_compressed(store/'000000.npz',cloud=cloud,scan=cloud)
        if not lidar_only:(store/'000000.png').write_bytes(b'inspection-only')
        desc=descriptor(features(),np.eye(3)[i])
        if lidar_only:del desc['visual']
        write_json(root/'000000.json',desc)
    cfg=dict(CFG['loops'],robots=list(stores));backend=dict(CFG['backend'],read_paths=[native.__file__])
    if lidar_only:
        backend['name']='mapclosures';backend.pop('fusion')
        cfg['branch_verification_limits']={'mapclosures':1}
    # Separate empty caches establish cold determinism, then reuse one cache.
    first_backend=dict(backend,verification_cache=str(tmp_path/'cache-first'))
    second_backend=dict(backend,verification_cache=str(tmp_path/'cache-second'))
    replay(stores,descs,first_backend,cfg,tmp_path/'first');replay(stores,descs,second_backend,cfg,tmp_path/'second')
    events=read_jsonl(tmp_path/'first/events.jsonl');verified=[e for e in events if e['type']=='verification']
    assert len(verified)==3 and all(e['accepted'] and e['retrieval_sources']==['mapclosures'] for e in verified)
    assert all(not e['verification_cache_hit'] for e in verified)
    assert len(read_jsonl(tmp_path/'first/constraints.jsonl'))==2
    for name in ('constraints.jsonl','exchanges.jsonl'):
        assert (tmp_path/'first'/name).read_bytes()==(tmp_path/'second'/name).read_bytes()
    messages=read_jsonl(tmp_path/'first/exchanges.jsonl')
    assert read_json(tmp_path/'first/summary.json')['network_bytes']==sum(e['network_bytes'] for e in messages)
    replay(stores,descs,first_backend,cfg,tmp_path/'cached')
    assert all(e['verification_cache_hit'] for e in read_jsonl(tmp_path/'cached/events.jsonl') if e['type']=='verification')
    for name in ('constraints.jsonl','exchanges.jsonl'):
        assert (tmp_path/'cached'/name).read_bytes()==(tmp_path/'first'/name).read_bytes()

def test_pgo_stage_needs_no_native_backend_and_preserves_inputs(tmp_path,monkeypatch):
    import builtins,copy
    from argparse import Namespace
    from s3e_pipeline.artifacts import stage_output,stage_path,file_hash
    from s3e_pipeline.cli import run
    artifacts={}
    for robot in CFG['robots']:
        settings={'robot':robot}
        with stage_output(tmp_path/'input','keyframes',settings,{},'fixture') as (out,_):
            (out/'store').mkdir()
            write_jsonl(out/'store/keyframes.jsonl',[dict(robot_id=robot,keyframe_id=0,stamp_ns=10**9,T_world_body=np.eye(4).tolist())])
        artifacts[f'keyframes.livo.{robot}']=str(stage_path(tmp_path/'input','keyframes',settings,{},'fixture'))
    with stage_output(tmp_path/'input','loops',{}, {},'fixture') as (out,_):
        write_jsonl(out/'constraints.jsonl',[])
    artifacts['loops.livo.megaloc_mapclosures']=str(stage_path(tmp_path/'input','loops',{}, {},'fixture'))
    registry=tmp_path/'inputs.json';write_json(registry,dict(artifacts=artifacts))
    config=copy.deepcopy(CFG);config['output_root']=str(tmp_path/'output')
    cfg_path=tmp_path/'config.yaml';cfg_path.write_text(yaml.safe_dump(config))
    before={str(p):(file_hash(p),p.stat().st_mtime_ns) for p in (tmp_path/'input').rglob('*') if p.is_file()}
    original_import=builtins.__import__
    def guarded_import(name,*args,**kwargs):
        if name=='s3e_mapclosures_native':raise AssertionError('PGO imported a descriptor backend')
        return original_import(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded_import)
    args=Namespace(config=cfg_path,input_run=registry,stage=['pgo'],resume=True)
    result=run(args);run(args)
    graph=read_json(Path(read_json(result)['artifacts']['pgo.livo.megaloc_mapclosures'])/'graph.json')
    assert len(set(graph['components'].values()))==3
    assert before=={str(p):(file_hash(p),p.stat().st_mtime_ns) for p in (tmp_path/'input').rglob('*') if p.is_file()}

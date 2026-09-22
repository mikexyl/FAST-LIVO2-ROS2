"""Resume valid native captures after recording-format failures, preserving original trials."""
import argparse
import json
from pathlib import Path
import sys
import time
import traceback
import yaml

ROOT=Path('/workspace');BASE=ROOT/'.ros2/upstream-area-s3e-20260921'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path.insert(0,str(SCRIPTS))
from upstream_area_batch import call,read,save,sha,sources,quality,PYTHON,RERUN


def frozen():
    expected=read(BASE/'full/source-hashes.json')
    assert read(BASE/'capacity-cuda/test-result.json')['bitwise_centroid_equality']
    expected['.ros2/ellipsoid-cuda/libellipsoid_surface.so']=sha(BASE/'capacity-cuda/libellipsoid_surface.so')
    return expected


def run(name):
    source=BASE/'full'/name;folder=BASE/'recovered-cuda'/name
    assert sources()==frozen()
    trials=read(source/'trials.json')
    if name=='S3E_Campus_Road_2':trials['Carol']=str(BASE/'capacity-run/frontends/Carol')
    qualities={r:quality(Path(p)) for r,p in trials.items()}
    assert all(v['passed'] for v in qualities.values()),qualities
    assert all(read(Path(p)/'sensor-coverage.json')['full_selected_sensor_tail_reached'] for p in trials.values())
    assert all(read(Path(p)/'artifact-validation.json')['accumulated_area_membership_verified'] for p in trials.values())
    folder.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((source/'config.yaml').read_text());cfg['output_root']=str(folder)
    (folder/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    save(folder/'trials.json',trials)
    save(folder/'source-hashes.json',frozen())
    save(folder.parent/'source-hashes.json',frozen())
    provenance=dict(cuda_storage_growth=read(BASE/'capacity-cuda/test-result.json'),
        cuda_source_sha256=sha(BASE/'capacity-cuda/surface.cu'),original_batch=str(BASE/'full'),trials=trials,quality=qualities,
        reason='Resume geometrically valid captures after live recording footer-format failures; Road 2 Carol is a separate octree-storage-growth rerun',
        estimator_thresholds_changed=False,backend_thresholds_changed=False,ground_truth_used=False,
        configs={r:sha(Path(p)/'frontend/runtime.yaml') for r,p in trials.items()},
        source_indices={r:sha(Path(p)/'frontend/area_maps/index.jsonl') for r,p in trials.items()})
    save(folder/'capture-provenance.json',provenance)
    state=dict(status='running',stage='descriptors',started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    def mark(stage,**extra):state.update(stage=stage,**extra);save(folder/'status.json',state);print(name,stage,flush=True)
    try:
        mark('descriptors');artifacts={}
        for robot,trial in trials.items():
            mark('descriptors',robot=robot)
            target=folder/f'prepared-{robot}'
            call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',Path(trial)/'frontend/area_maps',
                '--output',target,'--config',folder/'config.yaml'],folder/f'{robot}-descriptors.log',1800)
            artifacts[f'keyframes.ellipselio.{robot}']=str(target)
            artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(target/'ellipsoid')
        save(folder/'artifacts.json',artifacts);mark('distributed_pcm_cbs')
        call([PYTHON,SCRIPTS/'upstream_area_batch.py','backend','--work',folder],folder/'backend.log',cfg['dpgo']['timeout_s']+120)
        mark('evaluation')
        call([PYTHON,SCRIPTS/'rollout_report.py','--work',folder,'--trials',folder/'trials.json'],folder/'evaluation.log',3600)
        call([RERUN/'python',SCRIPTS/'rollout_rerun.py',folder],folder/'rerun.log',300)
        call([RERUN/'rerun','rrd','verify',folder/'report/result.rrd'],folder/'recording-verification.log',300)
        assert sources()==frozen()
        mark('complete',status='complete',finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    except Exception as exc:
        mark('failed',status='failed',error=repr(exc),traceback=traceback.format_exc())


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('groups',nargs='+')
    for name in p.parse_args().groups:run(name)

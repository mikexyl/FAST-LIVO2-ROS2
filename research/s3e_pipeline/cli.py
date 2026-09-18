"""One reproducible S3E visual/LiDAR loop pipeline, with reusable stages."""
import argparse
import base64
import copy
import os
from pathlib import Path
import subprocess
import time
import zlib
import json
import math
import numpy as np
import yaml
from .frontends import frontend_name, ellipse_mapping, ellipse_provenance
from .artifacts import (digest,file_hash,read_json,read_jsonl,write_json,write_jsonl,
                        canonical,stage_output,stage_path,validate_stage,invalidate_dependents)

SOURCE=Path(os.environ.get('S3E_SOURCE_ROOT',Path(__file__).resolve().parents[3]))
REPO=SOURCE/'FAST-LIVO2-ROS2'
CODE_ROOT=Path(__file__).resolve().parents[1]



def resolved(path):
    p=Path(path)
    return p if p.is_absolute() else SOURCE/p


def odometry_mapping(robot, overrides):
    """Resolve existing ROS parameters without modifying shared robot YAMLs."""
    mapping=yaml.safe_load((REPO/f'config/s3e/{robot.lower()}.yaml').read_text())
    params=mapping['/**']['ros__parameters']
    for key,value in overrides.items():
        parts=key.split('.'); parent=params
        for part in parts[:-1]:
            if not isinstance(parent,dict) or part not in parent:
                raise ValueError(f'Unknown odometry mapping parameter: {key}')
            parent=parent[part]
        if not isinstance(parent,dict) or parts[-1] not in parent:
            raise ValueError(f'Unknown odometry mapping parameter: {key}')
        if type(value) is not type(parent[parts[-1]]):
            raise ValueError(f'Wrong type for odometry mapping parameter: {key}')
        if isinstance(value,float) and not math.isfinite(value):
            raise ValueError(f'Non-finite odometry mapping parameter: {key}')
        parent[parts[-1]]=copy.deepcopy(value)
    return mapping


def code_hash(stage):
    names=['cli.py','artifacts.py','frontends.py']
    names+=dict(odometry=['mcap_writer.py'],keyframes=['data.py','geometry.py'],
        descriptors=['backends.py','mapclosures.py','data.py','model_worker.py'],
        loops=['backends.py','mapclosures.py','registration.py','geometry.py','data.py','distributed.py','verification.py','isolation.py'],
        pgo=['pgo.py','mixed_pgo.py','registration.py','geometry.py'],evaluate=['evaluation.py','evo_evaluation.py','visualize.py','data.py','geometry.py'],
        dpgo=['dpgo.py','ros_dpgo_worker.py','registration_exchange.py','mixed_pgo.py','cbs_bridge.py','distributed.py','verification.py','isolation.py','backends.py','mapclosures.py','registration.py','geometry.py','data.py'],
        dpgo_evaluate=['dpgo_evaluation.py','dpgo_visualize.py','evaluation.py','evo_evaluation.py','visualize.py','cbs_bridge.py','geometry.py','data.py'],
        inspect=['mapclosures_inspection.py','mapclosures_rerun.py','backends.py','registration.py','geometry.py'])[stage]
    hashes={name:file_hash(Path(__file__).with_name(name)) for name in names}
    if stage=='odometry':
        for p in [*sorted((REPO/'src').glob('*.cpp')),*sorted((REPO/'include').rglob('*.h')),
                  REPO/'scripts/run_s3e.py',REPO/'scripts/s3e_adapter.py',REPO/'launch/mapping_s3e.launch.py',
                  REPO/'scripts/run_ellipselio.py',REPO/'scripts/run_ellipselio.sh']:
            hashes[str(p.relative_to(REPO))]=file_hash(p)
    if stage in ('descriptors','loops','inspect'):
        hashes.update({str(p.relative_to(CODE_ROOT)):file_hash(p) for p in sorted((CODE_ROOT/'adapters/mapclosures').glob('*')) if p.is_file()})
    if stage == 'pgo':
        hashes.update({str(p.relative_to(CODE_ROOT)):file_hash(p) for p in sorted((CODE_ROOT/'adapters/gtsam_points').rglob('*')) if p.is_file()})
    return digest(hashes)


def verification_code_hash():
    return digest({p:file_hash(Path(__file__).with_name(p)) for p in
        ['backends.py','mapclosures.py','registration.py','geometry.py','artifacts.py']})


def selected_inputs(artifacts,robots,METHOD,FRONTEND='livo'):
    LOOPS=f'loops.{FRONTEND}.{METHOD}';PGO=f'pgo.{FRONTEND}.{METHOD}'
    labels={f'{stage}.{FRONTEND}.{r}' for stage in ['odometry','keyframes'] for r in robots}
    labels|={f'descriptors.{FRONTEND}.{method}.{r}' for method in ['megaloc',METHOD] for r in robots}
    labels|={LOOPS,PGO,'evaluate','inspect','dpgo','dpgo_evaluate'}
    chosen={k:v for k,v in artifacts.items() if k in labels}
    if LOOPS not in chosen or PGO not in chosen:chosen.pop('evaluate',None)
    return chosen


def check_lineage(artifacts,robots,METHOD,FRONTEND='livo'):
    LOOPS=f'loops.{FRONTEND}.{METHOD}';PGO=f'pgo.{FRONTEND}.{METHOD}'
    hashes={k:validate_stage(v)['stage_hash'] for k,v in artifacts.items() if k!='evaluate'}
    for robot in robots:
        k=f'keyframes.{FRONTEND}.{robot}';v=f'descriptors.{FRONTEND}.megaloc.{robot}';d=f'descriptors.{FRONTEND}.{METHOD}.{robot}'
        expected={k:{'odometry':hashes[f'odometry.{FRONTEND}.{robot}']},d:{'keyframes':hashes[k]}}
        if METHOD=='megaloc_mapclosures':
            expected[v]={'keyframes':hashes[k]};expected[d]['visual']=hashes[v]
        for label,inputs in expected.items():
            if read_json(Path(artifacts[label])/'COMPLETE.json')['identity']['inputs']!=inputs:
                raise ValueError(f'Invalid input lineage: {label}')
    inputs={r:dict(keyframes=hashes[f'keyframes.{FRONTEND}.{r}'],descriptors=hashes[f'descriptors.{FRONTEND}.{METHOD}.{r}']) for r in robots}
    if read_json(Path(artifacts[LOOPS])/'COMPLETE.json')['identity']['inputs']!=inputs:raise ValueError('Invalid loop lineage')
    expected=dict(keyframes={r:hashes[f'keyframes.{FRONTEND}.{r}'] for r in robots},loops=hashes[LOOPS])
    if read_json(Path(artifacts[PGO])/'COMPLETE.json')['identity']['inputs']!=expected:raise ValueError('Invalid graph lineage')
    return hashes


def run(args):
    cfg=yaml.safe_load(args.config.read_text());robots=cfg['robots']
    FRONTEND=frontend_name(cfg)
    METHOD=cfg['backend']['name'];LOOPS=f'loops.{FRONTEND}.{METHOD}';PGO=f'pgo.{FRONTEND}.{METHOD}'
    if METHOD not in ('megaloc_mapclosures','mapclosures'):raise ValueError('Expected MegaLoc + MapClosures or MapClosures-only')
    if FRONTEND=='ellipselio' and METHOD!='mapclosures':raise ValueError('EllipseLIO currently requires LiDAR-only MapClosures')
    if FRONTEND=='ellipselio' and 'inspect' in args.stage:raise ValueError('RGB inspection requires camera exports; use dpgo_evaluate for EllipseLIO maps and registration overlays')
    if cfg.get('schema_version')!=2 or robots!=['Alpha','Bob','Carol']:raise ValueError('Expected the single three-robot S3E configuration')
    offsets=cfg['odometry'].get('start_offsets_s',{})
    if not isinstance(offsets,dict) or set(offsets)-set(robots) or any(not math.isfinite(float(v)) or float(v)<0 for v in offsets.values()):
        raise ValueError('Odometry start_offsets_s must map robot names to finite nonnegative bag offsets')
    overrides=cfg['odometry'].get('mapping_overrides',{})
    if (not isinstance(overrides,dict) or set(overrides)-set(robots) or
            any(not isinstance(v,dict) or any(not isinstance(k,str) for k in v) for v in overrides.values())):
        raise ValueError('Odometry mapping_overrides must map robot names to parameter dictionaries')
    if FRONTEND=='ellipselio' and overrides:raise ValueError('EllipseLIO mapping_overrides are not supported')
    mappings={r:(ellipse_mapping(SOURCE,r) if FRONTEND=='ellipselio' else odometry_mapping(r,overrides.get(r,{}))) for r in robots}
    replay_robots=getattr(args,'odometry_robots',None) or robots
    if set(replay_robots)-set(robots):raise ValueError('Unknown odometry robot')
    dataset=resolved(cfg['dataset'])
    if dataset.name not in ('S3E_Square_1','S3E_Square_2','S3E_Playground_1','S3E_Playground_2','S3E_Library_1','S3E_Laboratory_1','S3E_Campus_Road_1'):raise ValueError('Unsupported dataset')
    if any(s in ('all','odometry') for s in args.stage) and not (dataset/'metadata.yaml').is_file():
        raise FileNotFoundError(dataset/'metadata.yaml')
    root=resolved(cfg['output_root']);root.mkdir(parents=True,exist_ok=True)
    previous_run=read_json(args.input_run) if args.input_run else None
    if previous_run and frontend_name(previous_run.get('config',{}))!=FRONTEND:raise ValueError('Input run belongs to a different odometry frontend')
    previous_dataset=previous_run.get('config',{}).get('dataset') if previous_run else None
    if previous_dataset and resolved(previous_dataset)!=dataset:
        raise ValueError('Input run belongs to a different S3E sequence')
    artifacts=selected_inputs(previous_run['artifacts'],robots,METHOD,FRONTEND) if previous_run else {}
    for robot in robots:
        odometry_input=artifacts.get(f'odometry.{FRONTEND}.{robot}')
        if odometry_input and resolved(read_json(Path(odometry_input)/'run/summary.json')['bag'])!=dataset:
            raise ValueError('Input odometry belongs to a different S3E sequence')
        will_replay=any(s in ('all','odometry') for s in args.stage) and robot in replay_robots
        if odometry_input and not will_replay and read_json(Path(odometry_input)/'run/summary.json').get('start_offset_s',0.)!=float(offsets.get(robot,0.)):
            raise ValueError(f'Frozen odometry start offset differs for {robot}; replay it')
        if odometry_input and not will_replay:
            saved=Path(odometry_input)/'run/mapping_config.yaml'
            if not saved.is_file() or yaml.safe_load(saved.read_text())!=mappings[robot]:
                raise ValueError(f'Frozen odometry mapping configuration differs for {robot}; replay it')
    prefix='inspection-run' if args.stage==['inspect'] else 'run'
    registry=root/f'{prefix}-{digest(cfg)[:16]}.json'
    def get(label):
        p=Path(artifacts[label]);m=validate_stage(p)
        return p,m['stage_hash']
    def execute(stage,label,settings,inputs,operation):
        code=code_hash(stage)
        with stage_output(root,stage,settings,inputs,code,args.resume) as (out,cached):
            print(f'{label}: {"cached" if cached else "running"} {out}',flush=True)
            if not cached:operation(out)
        out=stage_path(root,stage,settings,inputs,code)
        previous=artifacts.get(label)
        if previous and Path(previous)!=out:
            invalidate_dependents(artifacts,validate_stage(previous,verify_files=False)['stage_hash'])
        artifacts[label]=str(out)
        write_json(registry,dict(schema_version=1,artifacts=artifacts,config=cfg,research_source=str(CODE_ROOT)))
        return out
    backend=dict(cfg['backend'],name=METHOD,command=[],read_paths=[])
    default_stages=['odometry','descriptors','dpgo','dpgo_evaluate'] if 'dpgo' in cfg else ['odometry','descriptors','loops','pgo','evaluate']
    stages=default_stages if args.stage==['all'] else args.stage
    if any(s in ('descriptors','loops') for s in stages):
        import s3e_mapclosures_native as native
        backend['native_binary_sha256']=file_hash(native.__file__)
        backend['adapter_code_hash']=digest({p.name:file_hash(p) for p in sorted((CODE_ROOT/'adapters/mapclosures').glob('*')) if p.is_file()})
    for stage in stages:
        if stage=='odometry':
            inputs={p.name:file_hash(p) for p in sorted(dataset.glob('*.db3'))}
            inputs['metadata.yaml']=file_hash(dataset/'metadata.yaml')
            for robot in robots:
                if robot not in replay_robots:
                    if f'odometry.{FRONTEND}.{robot}' not in artifacts:
                        raise ValueError(f'No frozen odometry for skipped robot {robot}; provide --input-run or replay it')
                    previous,_=get(f'odometry.{FRONTEND}.{robot}')
                    if read_json(previous/'run/summary.json').get('start_offset_s',0.)!=float(offsets.get(robot,0.)):
                        raise ValueError(f'Frozen odometry start offset differs for {robot}; replay it')
                    print(f'odometry.{FRONTEND}.{robot}: reusing validated input {previous}',flush=True)
                    continue
                offset=float(offsets.get(robot,0.))
                settings=dict(robot=robot,**{k:v for k,v in cfg['odometry'].items() if k not in ('start_offsets_s','mapping_overrides')})
                settings['mapping_config']=mappings[robot]
                if FRONTEND=='livo':settings['camera_config_sha256']=file_hash(REPO/f'config/s3e/{robot.lower()}_camera.yaml')
                else:settings['native']=ellipse_provenance(SOURCE)
                if offset:settings['start_offset_s']=offset
                def odometry(out):
                    config_path=out/'mapping_config.yaml'
                    config_path.write_text(yaml.safe_dump(mappings[robot],sort_keys=False))
                    if FRONTEND=='ellipselio':
                        subprocess.run(['bash',str(REPO/'scripts/run_ellipselio.sh'),'--robot',robot,
                            '--bag',str(dataset),'--rate',str(cfg['odometry']['rate']),
                            '--output',str(out/'run'),'--start-offset',str(offset),
                            '--mapping-config',str(config_path)],check=True,cwd=SOURCE)
                    else:
                        subprocess.run([str(REPO/'scripts/run_s3e.sh'),'--robot',robot,'--namespace',robot,
                            '--bag',str(dataset),'--rate',str(cfg['odometry']['rate']),'--export','--output',str(out/'run'),
                            '--start-offset',str(offset),
                            '--mapping-config',str(config_path),
                            '--export-python',str(SOURCE/'.ros2/research-venv/bin/python')],check=True,cwd=SOURCE)
                    summary=read_json(out/'run/summary.json')
                    if not summary['success']:raise ValueError('Odometry failed')
                    write_json(out/'summary.json',summary)
                execute(stage,f'odometry.{FRONTEND}.{robot}',settings,inputs,odometry)
        elif stage=='descriptors':
            from .data import keyframes,LocalStore
            from .backends import create,JsonWorker,pack_array
            model=None
            try:
                for robot in robots:
                    odom,oh=get(f'odometry.{FRONTEND}.{robot}')
                    keylabel=f'keyframes.{FRONTEND}.{robot}'
                    if keylabel not in artifacts:
                        camera=yaml.safe_load((odom/'run/camera_config.yaml').read_text())['/**']['ros__parameters'] if FRONTEND=='livo' else None
                        execute('keyframes',keylabel,dict(robot=robot,**cfg['keyframes']),{'odometry':oh},
                            lambda out:keyframes(odom/'run/export',out/'store',cfg['keyframes'],camera))
                    keys,kh=get(keylabel);manifest=read_json(keys/'COMPLETE.json')
                    if manifest['identity']['inputs']!={'odometry':oh} or manifest['identity']['config']!=dict(robot=robot,**cfg['keyframes']):
                        raise ValueError('Frozen keyframes do not match configuration')
                    store=LocalStore(keys/'store',robot);visual=None;vh=None
                    if METHOD=='megaloc_mapclosures':
                        visual_label=f'descriptors.{FRONTEND}.megaloc.{robot}'
                        if visual_label not in artifacts:
                            vcfg=cfg['visual'];checkpoint=resolved(vcfg['checkpoint'])
                            if file_hash(checkpoint)!=vcfg['checkpoint_sha256']:raise ValueError('MegaLoc checkpoint hash mismatch')
                            if model is None:
                                model=JsonWorker([str(resolved(vcfg['python'])),str(CODE_ROOT/'s3e_pipeline/model_worker.py'),
                                    '--source',str(resolved(vcfg['source'])),'--checkpoint',str(checkpoint),
                                    '--gpu-lock',str(resolved(vcfg['gpu_lock']))])
                            def visual(out):
                                timings=[]
                                for row in store.rows:
                                    key=row['keyframe_id'];start=time.monotonic()
                                    image=(store.root/f'{key:06d}.png').read_bytes()
                                    desc=model.call(op='describe',image=base64.b64encode(image).decode())
                                    (out/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(desc)))
                                    timings.append(dict(keyframe_id=key,seconds=time.monotonic()-start))
                                write_jsonl(out/'timing.jsonl',timings)
                                write_json(out/'summary.json',dict(frames=len(timings),runtime_s=sum(t['seconds'] for t in timings),model=model.call(op='stats')))
                            execute(stage,visual_label,vcfg,{'keyframes':kh},visual)
                        visual,vh=get(visual_label)
                        vm=read_json(visual/'COMPLETE.json')['identity']
                        if vm['inputs']!={'keyframes':kh}:raise ValueError('MegaLoc/keyframe mismatch')
                        provenance=vm['config']
                        checkpoint_hash=provenance.get('checkpoint_sha256') or provenance.get('file_hashes',{}).get(str(resolved(cfg['visual']['checkpoint'])))
                        if checkpoint_hash!=cfg['visual']['checkpoint_sha256']:raise ValueError('Frozen MegaLoc checkpoint mismatch')
                    def fused(out):
                        instance=create(backend);timings=[]
                        for row in store.rows:
                            key=row['keyframe_id'];start=time.monotonic()
                            with np.load(store.root/f'{key:06d}.npz') as f:scan=f['cloud']
                            desc=instance.describe(row,scan,b'')
                            if visual is not None:
                                v=json.loads(zlib.decompress((visual/f'{key:06d}.json.zlib').read_bytes()))
                                desc['visual']=pack_array(np.asarray(v['global'],dtype=np.float32))
                            (out/f'{key:06d}.json.zlib').write_bytes(zlib.compress(canonical(desc)))
                            timings.append(dict(keyframe_id=key,seconds=time.monotonic()-start,features=desc.get('mapclosures_features')))
                            if key%100==0:print(f'{robot}: {key+1}/{len(store.rows)} LiDAR descriptors',flush=True)
                        write_jsonl(out/'timing.jsonl',timings)
                        write_json(out/'summary.json',dict(frames=len(timings),runtime_s=sum(t['seconds'] for t in timings),visual_reused=visual is not None))
                    execute(stage,f'descriptors.{FRONTEND}.{METHOD}.{robot}',dict(preprocessing=backend['mapclosures'],native_binary_sha256=backend.get('native_binary_sha256'),visual=vh),dict(keyframes=kh,**({'visual':vh} if visual is not None else {})),fused)
            finally:
                if model:model.close()
        elif stage=='loops':
            from .distributed import replay
            stores={};descs={};inputs={}
            for robot in robots:
                k,kh=get(f'keyframes.{FRONTEND}.{robot}');d,dh=get(f'descriptors.{FRONTEND}.{METHOD}.{robot}')
                dm=read_json(d/'COMPLETE.json')['identity']
                if dm['inputs'].get('keyframes')!=kh:raise ValueError('Loop descriptor/keyframe mismatch')
                if dm['config']['preprocessing']!=backend['mapclosures']:raise ValueError('LiDAR preprocessing mismatch')
                if dm['config']['native_binary_sha256']!=backend['native_binary_sha256']:raise ValueError('Native descriptor binary mismatch')
                stores[robot]=k/'store';descs[robot]=d;inputs[robot]=dict(keyframes=kh,descriptors=dh)
            cache=root/'verification-cache'/digest(dict(backend=backend,inputs=inputs,code=verification_code_hash()))
            cache.mkdir(parents=True,exist_ok=True)
            execute(stage,LOOPS,dict(backend=backend,loops=cfg['loops']),inputs,
                    lambda out:replay(stores,descs,dict(backend,verification_cache=str(cache)),dict(cfg['loops'],robots=robots),out))
        elif stage=='pgo':
            from .pgo import optimize
            from .mixed_pgo import settings as registration_settings, native_provenance, refine_graph
            registration=registration_settings(cfg['pgo'].get('registration_factors',{}))
            provenance=native_provenance(SOURCE) if registration['enabled'] else None
            loop,lh=get(LOOPS);rows=[];hashes={};stores={}
            for robot in robots:
                p,h=get(f'keyframes.{FRONTEND}.{robot}');rows+=read_jsonl(p/'store/keyframes.jsonl');hashes[robot]=h
                stores[robot]=p/'store'
            settings=dict(cfg['pgo'],registration_factors=registration,native=provenance) if registration['enabled'] else cfg['pgo']
            def pgo(out):
                graph=optimize(rows,read_jsonl(loop/'constraints.jsonl'),cfg['pgo'])
                if registration['enabled']:
                    graph=refine_graph(graph,stores,registration,SOURCE,out,provenance)
                write_json(out/'graph.json',graph)
            execute(stage,PGO,settings,dict(keyframes=hashes,loops=lh),pgo)
        elif stage=='evaluate':
            from .evaluation import evaluate
            inputs=check_lineage(artifacts,robots,METHOD,FRONTEND)
            execute(stage,'evaluate',cfg['evaluation'],inputs,lambda out:evaluate(cfg,artifacts,dataset,out))
        elif stage=='dpgo':
            from .dpgo import run as run_dpgo, native_provenance
            labels=[f'keyframes.{FRONTEND}.{r}' for r in robots]
            labels += [LOOPS] if cfg['dpgo'].get('mode','peers')=='frozen' else [f'descriptors.{FRONTEND}.{METHOD}.{r}' for r in robots]
            inputs={label:get(label)[1] for label in labels}
            if cfg['dpgo'].get('mode','peers')=='peers':
                import s3e_mapclosures_native as native
                for robot in robots:
                    dm=read_json(Path(artifacts[f'descriptors.{FRONTEND}.{METHOD}.{robot}'])/'COMPLETE.json')['identity']
                    if dm['inputs']['keyframes']!=inputs[f'keyframes.{FRONTEND}.{robot}']:
                        raise ValueError('DDS descriptor/keyframe mismatch')
                    if dm['config']['preprocessing']!=cfg['backend']['mapclosures'] or dm['config']['native_binary_sha256']!=file_hash(native.__file__):
                        raise ValueError('DDS descriptor preprocessing or native binary mismatch')
            provenance=native_provenance(SOURCE)
            settings=dict(dpgo=cfg['dpgo'],backend=cfg['backend'],loops=cfg['loops'],pgo=cfg['pgo'],native=provenance)
            execute(stage,'dpgo',settings,inputs,lambda out:run_dpgo(cfg,artifacts,SOURCE,out))
        elif stage=='dpgo_evaluate':
            from .dpgo_evaluation import evaluate as evaluate_dpgo
            labels=['dpgo',*[f'{s}.{FRONTEND}.{r}' for s in ('keyframes','odometry') for r in robots]]
            if PGO in artifacts:labels.append(PGO)
            inputs={label:get(label)[1] for label in labels}
            inputs['position_ground_truth']={r:file_hash(dataset/f'{r.lower()}_gt.txt') if (dataset/f'{r.lower()}_gt.txt').is_file() else None for r in robots}
            execute(stage,'dpgo_evaluate',cfg['evaluation'],inputs,lambda out:evaluate_dpgo(cfg,artifacts,dataset,out))
        elif stage=='inspect':
            from .mapclosures_inspection import build
            import s3e_mapclosures_inspection as native
            labels=[LOOPS,*[f'keyframes.{FRONTEND}.{r}' for r in robots],
                    *[f'descriptors.{FRONTEND}.{METHOD}.{r}' for r in robots]]
            inputs={label:get(label)[1] for label in labels}
            settings=dict(mapclosures=backend['mapclosures'],evidence=backend['evidence'],
                native_binary_sha256=file_hash(native.__file__),rerun_version=cfg['evaluation']['rerun_version'])
            execute(stage,'inspect',settings,inputs,lambda out:build(cfg,artifacts,out))
        else:raise ValueError(f'Unknown stage {stage}')
    print(f'Run registry: {registry}',flush=True)
    return registry


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    r=sub.add_parser('run');r.add_argument('--stage',nargs='+',choices=['all','odometry','descriptors','loops','pgo','evaluate','inspect','dpgo','dpgo_evaluate'],required=True)
    r.add_argument('--config',type=Path,required=True);r.add_argument('--input-run',type=Path)
    r.add_argument('--odometry-robots',nargs='+',choices=['Alpha','Bob','Carol'],
                   help='Replay only these robots; reuse other validated odometry from --input-run')
    r.add_argument('--resume',action='store_true');args=p.parse_args();run(args)


if __name__=='__main__':main()

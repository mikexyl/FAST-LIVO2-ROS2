#!/usr/bin/env python3
"""Matched frozen CPU GICP / GLIM GPU VGICP distributed CBS measurements.

No frontend replay, new retrieval or parameter search. Each run uses the exact
same immutable endpoint stores and verified constraints. Run from the isolated
GPU source tree, with its ROS/GTSAM overlay sourced.
"""
import argparse,copy,csv,json,os,subprocess,sys,threading,time,traceback
from pathlib import Path
import yaml

SOURCE=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(SOURCE/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.dpgo import run,native_provenance
from s3e_pipeline.artifacts import file_hash,read_json,read_jsonl
from s3e_pipeline.online_io import atomic_json

def summarize(out):
    summary=read_json(out/'dpgo/summary.json'); details=[]
    for robot,record in summary['robots'].items():
        reg=record.get('registration') or {};entry=dict(robot=robot,wall_s=record['wall_s'],
            factors=reg.get('registration_factor_count',0),preparation_s=reg.get('preparation_s',0),
            linearizations=sum(p.get('linearizations',0) for p in reg.get('pairs',[])),
            gpu_batches=sum(p.get('gpu_linearization_batches',0) for p in reg.get('pairs',[])),
            exact_cache_hits=sum(p.get('exact_cache_hits',0) for p in reg.get('pairs',[])))
        for pattern in ['timing_robot_*.csv','belief_service_timing_robot_*.csv']:
            path=next((out/'dpgo'/robot/'native').rglob(pattern)); rows=list(csv.DictReader(path.open()))
            entry[pattern]={k:sum(float(r[k]) for r in rows)/1000 for k in rows[0] if k.endswith('_ms')} if rows else {}
        details.append(entry)
    return dict(wall_s=summary['wall_s'],loops=summary['loops'],
        simultaneous_process_tree_peak_kib=summary.get('simultaneous_process_tree_peak_rss_kib'),robots=details)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',default=['GRACO_aerial','S3E_Library_1'])
    p.add_argument('--order',nargs='+',default=['gicp','vgicp_gpu','vgicp_gpu','gicp']);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);entries=[]
    atomic_json(a.output/'source-hashes.json',native_provenance(SOURCE))
    originals=Path('/workspace/.ros2/communication-benchmark148-20260928/runs')
    for group in a.cases:
        original=originals/('ours-'+group+'-full-v2');epoch=original/'epochs'/('040' if group=='GRACO_aerial' else '045')
        request=read_json(epoch/'request.json'); base=yaml.safe_load((original/'config.yaml').read_text())
        for index,variant in enumerate(a.order):
            out=a.output/(group+'-'+str(index)+'-'+variant);out.mkdir();(out/'dpgo').mkdir()
            cfg=copy.deepcopy(base);cfg['dpgo'].update(mode='frozen',ros_domain_id=209,retain_registration_transport=False)
            cfg['dpgo']['registration_factors']['factor']=variant
            (out/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
            atomic_json(out/'input.json',dict(request=request,original=str(epoch),
                hashes={str(path):file_hash(path) for path in [epoch/'request.json',epoch/'loops/constraints.jsonl',original/'config.yaml']}))
            entry=dict(group=group,variant=variant,index=index,path=str(out),phase='running');entries.append(entry)
            atomic_json(a.output/'progress.json',dict(entries=entries,phase='running'))
            gpu=[];stop=threading.Event()
            def sample():
                while not stop.is_set():
                    result=subprocess.run(['nvidia-smi','--query-gpu=memory.used,utilization.gpu','--format=csv,noheader,nounits'],capture_output=True,text=True)
                    gpu.append(dict(wall_ns=time.time_ns(),global_device_sample=result.stdout.strip()))
                    stop.wait(.25)
            sampler=threading.Thread(target=sample);sampler.start()
            try:
                run(cfg,request['artifacts'],SOURCE,out/'dpgo')
                entry.update(phase='complete',**summarize(out))
            except Exception as error:
                entry.update(phase='failed',error=repr(error),traceback=traceback.format_exc())
                raise
            finally:
                stop.set();sampler.join();atomic_json(out/'gpu-samples.json',dict(scope='Whole GPU allocation/utilization, including any other processes; not per-process allocation',samples=gpu))
                atomic_json(a.output/'progress.json',dict(entries=entries,phase=entry['phase']))
    atomic_json(a.output/'progress.json',dict(entries=entries,phase='complete'))

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Backend-only replay of retained causal prefixes; never loads ground truth."""
import argparse
import copy
from pathlib import Path
import sys
import time
import yaml
SOURCE=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(SOURCE/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.artifacts import read_json,read_jsonl,file_hash
from s3e_pipeline.online_io import atomic_json
from s3e_pipeline.persistent_cbs import PersistentSession,save_snapshot


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--revisions',type=int,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--hold-s',type=float,default=10.)
    p.add_argument('--domain',type=int,default=171);a=p.parse_args()
    if a.revisions!=sorted(set(a.revisions)) or a.hold_s<=0:raise ValueError('Expected ordered unique revisions and positive hold')
    a.output.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((a.capture/'config.yaml').read_text())
    cfg=copy.deepcopy(cfg);cfg['dpgo'].update(execution='persistent',persistent_loop_rate_hz=1.,
        persistent_settle_iterations=10,ros_domain_id=a.domain,timeout_s=600)
    cfg['dpgo']['registration_factors']['factor']='vgicp_gpu'
    (a.output/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    final=a.capture/'epochs'/f'{a.revisions[-1]:03d}'
    request=read_json(final/'request.json')['artifacts']
    stores={r:Path(request[f'keyframes.ellipselio.{r}'])/'store' for r in cfg['robots']}
    session=PersistentSession(cfg,SOURCE,a.output/'session',stores)
    records=[]
    try:
        start=time.monotonic()
        while not session.ready():
            if time.monotonic()-start>60:raise TimeoutError('Startup')
            time.sleep(.1)
        for index,revision in enumerate(a.revisions):
            epoch=a.capture/'epochs'/f'{revision:03d}'
            inputs=read_json(epoch/'request.json')['artifacts']
            rows={r:read_jsonl(Path(inputs[f'keyframes.ellipselio.{r}'])/'store/keyframes.jsonl') for r in cfg['robots']}
            edges=read_jsonl(Path(inputs['loops.ellipselio.mapclosures'])/'constraints.jsonl')
            terminal=index==len(a.revisions)-1; start=time.monotonic()
            session.submit(rows,edges,terminal)
            first=None; snapshot=None
            while snapshot is None or (terminal and not snapshot['summary']['finished']) or (not terminal and time.monotonic()-start<a.hold_s):
                snapshot=session.snapshot()
                if snapshot is not None:
                    if first is None:first=time.monotonic()-start
                    save_snapshot(snapshot,a.output/f'{revision:03d}')
                time.sleep(.2)
            records.append(dict(capture_revision=revision,session_revision=session.revision,
                input_sha256=file_hash(epoch/'input.json'),first_applied_s=first,
                elapsed_s=time.monotonic()-start,summary=snapshot['summary']))
            atomic_json(a.output/'progress.json',dict(state='complete' if terminal else 'running',records=records))
    except BaseException as error:
        atomic_json(a.output/'progress.json',dict(state='failed',error=str(error),records=records))
        raise
    finally:session.close()


if __name__=='__main__':main()

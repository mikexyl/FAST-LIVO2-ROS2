"""Consume completed native snapshots during capture, before the final manifest."""
import argparse
import fcntl
import json
from pathlib import Path
import time
from contextlib import contextmanager
from .artifacts import canonical, file_hash
from .area_maps import AreaHistoryAudit
from .online_io import JsonlTail, atomic_json
from .recent_submaps import prepare_row


@contextmanager
def preparation_slot(directory, count):
    """Limit simultaneous heavy preparations, not CPU or memory available to them."""
    held=None
    while held is None:
        for i in range(count):
            stream=(directory/f'prepare-{i}.lock').open('a')
            try: fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB); held=stream; break
            except BlockingIOError: stream.close()
        if held is None: time.sleep(.05)
    try: yield
    finally: fcntl.flock(held,fcntl.LOCK_UN); held.close()


def run(source, output, config, slots, concurrency):
    source,output=Path(source),Path(output)
    backend=config['backend']; mc=backend['mapclosures']
    if mc.get('projection_alignment')!='gravity': raise ValueError('Live area maps require gravity projection')
    for name in ('evidence','registration'):
        if backend[name].get('sampling')!='fixed' or backend[name].get('max_range_m',80.) is not None:
            raise ValueError('Live area maps require fixed evidence without a 3D range crop')
    if not 0<backend['evidence']['voxel_m']<=backend['registration']['voxel_m']:
        raise ValueError('Invalid evidence sampling')
    store=output/'store'; descriptors=output/'ellipsoid'
    store.mkdir(parents=True,exist_ok=True); descriptors.mkdir(exist_ok=True)
    tail=JsonlTail(source/'index.jsonl'); audit=AreaHistoryAudit(); key=0; rows=0
    started=time.time_ns(); seen=[]
    with (store/'keyframes.jsonl').open('xb',buffering=0) as index, (output/'timings.jsonl').open('xb',buffering=0) as timings:
        atomic_json(output/'READY',dict(wall_ns=time.time_ns()))
        while True:
            additions=tail.read()
            for row in additions:
                rows+=1; seen.append(row); received=time.time_ns()
                if not row['complete'] or not row['retrievable']: continue
                if row.get('strategy')!='area': raise ValueError('Expected accumulated area map')
                with preparation_slot(slots,concurrency):
                    item,timing=prepare_row(source,row,key,store,descriptors,config,audit=audit)
                # Sensor anchor stays unchanged. Availability is descriptor publication,
                # on a common wall clock, because bags are from different sessions.
                item['native_available_ns']=item['available_ns']
                item['snapshot_received_wall_ns']=received
                item['available_ns']=time.time_ns()
                item['availability_clock']='unix_wall_descriptor_publication'
                timing.update(available_ns=item['available_ns'],snapshot_received_wall_ns=received,
                              prepare_latency_s=(item['available_ns']-received)/1e9)
                index.write(canonical(item)+b'\n'); timings.write(canonical(timing)+b'\n'); key+=1
                atomic_json(output/'progress.json',dict(submaps=key,last_available_ns=item['available_ns']))
            manifest=source/'manifest.json'
            if manifest.exists() and not additions:
                final=json.loads(manifest.read_text())
                if not final['complete'] or file_hash(source/'index.jsonl')!=final['index_sha256']:
                    raise ValueError('Final native manifest failed verification')
                tail.finish()
                if [json.loads(x) for x in (source/'index.jsonl').read_text().splitlines()]!=seen:
                    raise ValueError('Native index changed during streaming')
                atomic_json(output/'summary.json',dict(submaps=key,source_rows=rows,complete=True,
                    source_index_sha256=final['index_sha256'],started_wall_ns=started,
                    finished_wall_ns=time.time_ns(),ground_truth_used=False))
                return
            time.sleep(.05)


if __name__=='__main__':
    import yaml
    p=argparse.ArgumentParser(); p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--config',type=Path,required=True)
    p.add_argument('--slots',type=Path,required=True); p.add_argument('--concurrency',type=int,default=3)
    a=p.parse_args(); run(a.source,a.output,yaml.safe_load(a.config.read_text()),a.slots,a.concurrency)

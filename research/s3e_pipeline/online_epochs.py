"""Periodic distributed PCM/CBS over causal prefixes while capture keeps running.

The current native PCM and GICP exchange seal immutable batches. Each online
revision therefore uses a fresh native session, rather than mutating a sealed
graph or disabling verification. No estimation is done by this launcher.
"""
import argparse
import copy
import os
import math
import multiprocessing
import signal
from collections import deque
from pathlib import Path
import time
from .artifacts import digest, read_json, write_jsonl
from .online_io import JsonlTail, Journal, atomic_json


def prefix(work,robots,cutoff=None,previous=()):
    rows={r:[x for x in JsonlTail(work/f'prepared-{r}/store/keyframes.jsonl').read()
             if cutoff is None or x['available_ns']<=cutoff] for r in robots}
    if any(not x for x in rows.values()): return None
    received={}; edges={}
    for r in robots:
        path=work/f'live/{r}/constraints.json'
        value=read_json(path) if path.exists() else {'constraints':[],'available_wall_ns':0}
        received[r]={}
        if cutoff is not None and value['available_wall_ns']>cutoff: continue
        for edge in value['constraints']:
            key=(tuple(edge['i']),tuple(edge['j']))
            if all(endpoint[1]<len(rows[endpoint[0]]) for endpoint in key):
                received[r][key]=edge; edges[key]=edge
    # An in-flight replacement cannot withdraw an already agreed measurement.
    # Retain that version until both endpoints agree on its replacement.
    agreed={(tuple(e['i']),tuple(e['j'])):e for e in previous}
    if any(endpoint[1]>=len(rows[endpoint[0]]) for key in agreed for endpoint in key):
        raise ValueError('Online graph prefix regressed')
    for key,edge in sorted(edges.items()):
        if all(key in received[r] and digest(received[r][key])==digest(edge) for r in {key[0][0],key[1][0]}):
            agreed[key]=edge
    return rows,[agreed[key] for key in sorted(agreed)]


class Cadence:
    """Deadlines advance independently of optimizer completion; no fictitious backfill."""
    def __init__(self, interval, now):
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError('CBS update interval must be finite and positive')
        self.interval, self.next = interval, now

    def take(self, now, final=False):
        if not final and now < self.next: return None
        scheduled = self.next
        missed = max(0, math.floor((now-scheduled)/self.interval))
        self.next = scheduled+(missed+1)*self.interval
        return dict(deadline_lateness_s=max(0.,now-scheduled), missed_deadlines=missed)


def solve_epoch(epoch, source, config):
    """One isolated native PCM/CBS session; the parent keeps capturing new prefixes."""
    from .dpgo import run as optimize
    request=read_json(epoch/'request.json')
    with (epoch/'solver.log').open('w',buffering=1) as log:
        os.dup2(log.fileno(),1); os.dup2(log.fileno(),2)
        optimize(config,request['artifacts'],source,epoch/'dpgo')


def freeze_epoch(work, root, revision, rows, edges, identity, cutoff, cadence):
    epoch=root/f'{revision:03d}'; epoch.mkdir()
    frozen=epoch/'loops'; frozen.mkdir(); write_jsonl(frozen/'constraints.jsonl',edges)
    artifacts={'loops.ellipselio.mapclosures':str(frozen)}
    for robot,track in rows.items():
        local=epoch/robot; store=local/'store'; store.mkdir(parents=True)
        write_jsonl(store/'keyframes.jsonl',track)
        for row in track:
            name=f"{row['keyframe_id']:06d}.npz"
            os.link(work/f'prepared-{robot}/store'/name,store/name)
        artifacts[f'keyframes.ellipselio.{robot}']=str(local)
    (epoch/'dpgo').mkdir()
    atomic_json(epoch/'input.json',dict(revision=revision,cutoff_wall_ns=cutoff,
        rows={r:len(x) for r,x in rows.items()},loops=len(edges),identity=identity,
        method='periodic distributed PCM/CBS; immutable causal input prefix',**cadence))
    atomic_json(epoch/'request.json',dict(artifacts=artifacts))
    return epoch


def run(work,source,config,interval=None):
    execution = config.get('dpgo', {}).get('execution', 'sealed')
    if execution == 'persistent':
        from .persistent_online import run as persistent_run
        return persistent_run(work,source,config,interval)
    if execution != 'sealed': raise ValueError('dpgo.execution must be sealed or persistent')
    interval=float(interval if interval is not None else config.get('dpgo',{}).get('update_interval_s',10.))
    cadence=Cadence(interval,time.monotonic())
    robots=config['robots']; root=work/'epochs'; root.mkdir(exist_ok=False)
    events=Journal(root/'events.jsonl'); previous=None; accepted=[]; revision=0
    queue=deque(); active=None; capture_done=False; published_count=0; max_pending=0
    cfg=copy.deepcopy(config); cfg['dpgo']['mode']='frozen'
    # A dedicated domain is supplied by the caller. Never override it with the
    # live retrieval domain or a hard-coded domain shared by another experiment.
    context=multiprocessing.get_context('spawn')
    atomic_json(root/'READY',dict(wall_ns=time.time_ns(),update_interval_s=interval,
        clock='common wall descriptor availability',overrun_policy='queue every changed causal prefix; no dropped revisions'))
    try:
        while True:
            if active is not None and not active['process'].is_alive():
                active['process'].join()
                if active['process'].exitcode != 0:
                    raise RuntimeError(f"CBS revision {active['input']['revision']} failed; inspect {active['epoch']}/solver.log")
                epoch=active['epoch']; inp=active['input']; published=time.time_ns()
                completed=dict(revision=inp['revision'],path=str(epoch/'dpgo'),
                    cutoff_wall_ns=inp['cutoff_wall_ns'],published_wall_ns=published,
                    started_wall_ns=active['started'],queue_wait_s=(active['started']-inp['cutoff_wall_ns'])/1e9,
                    input_to_publication_s=(published-inp['cutoff_wall_ns'])/1e9,
                    inputs=str(epoch/'input.json'),loops=read_json(epoch/'dpgo/summary.json')['loops'])
                atomic_json(epoch/'publication.json',completed); atomic_json(root/'latest.json',completed)
                events.append(dict(type='epoch_published',**completed)); published_count+=1; active=None
            done=all((work/f'live/{r}/DONE').exists() for r in robots)
            tick=None if capture_done else cadence.take(time.monotonic(),final=done)
            if tick is not None:
                value=prefix(work,robots,previous=accepted)
                if value is not None:
                    rows,edges=value; cutoff=time.time_ns(); accepted=edges
                    identity=digest(dict(counts={r:len(x) for r,x in rows.items()},edges=edges))
                    if identity!=previous and edges:
                        revision+=1
                        epoch=freeze_epoch(work,root,revision,rows,edges,identity,cutoff,
                            dict(tick,target_interval_s=interval,final=done))
                        queue.append(epoch); previous=identity; max_pending=max(max_pending,len(queue))
                        events.append(dict(type='epoch_queued',revision=revision,cutoff_wall_ns=cutoff,
                            submaps=sum(map(len,rows.values())),loops=len(edges),pending=len(queue),**tick))
                elif done:
                    raise ValueError('Online capture ended without initialized input for every robot')
                if done: capture_done=True
            if active is None and queue:
                epoch=queue.popleft(); inp=read_json(epoch/'input.json'); started=time.time_ns()
                process=context.Process(target=solve_epoch,args=(epoch,source,cfg))
                process.start(); active=dict(epoch=epoch,input=inp,process=process,started=started)
                events.append(dict(type='epoch_started',revision=inp['revision'],cutoff_wall_ns=inp['cutoff_wall_ns'],
                    queue_wait_s=(started-inp['cutoff_wall_ns'])/1e9,pending=len(queue)))
            if capture_done and active is None and not queue:
                atomic_json(root/'DONE',dict(revisions=revision,published_revisions=published_count,
                    finished_wall_ns=time.time_ns(),final_loops=len(accepted),no_verified_loops=not accepted,
                    target_interval_s=interval,max_pending_revisions=max_pending,
                    clock='common wall descriptor availability',termination='all captured revisions processed'))
                return
            time.sleep(.05)
    finally:
        if active is not None and active['process'].is_alive():
            os.kill(active['process'].pid,signal.SIGINT)
            active['process'].join(timeout=15)
            if active['process'].is_alive(): active['process'].kill(); active['process'].join()
        events.close()


if __name__=='__main__':
    import yaml
    p=argparse.ArgumentParser(); p.add_argument('--work',type=Path,required=True)
    p.add_argument('--source',type=Path,default=Path('/workspace'))
    p.add_argument('--config',type=Path,required=True); p.add_argument('--interval',type=float,default=None)
    a=p.parse_args(); run(a.work,a.source,yaml.safe_load(a.config.read_text()),a.interval)

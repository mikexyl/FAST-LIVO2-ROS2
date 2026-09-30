"""Causal online prefixes applied to one continuing CBS+ session per robot."""
from pathlib import Path
import time
from .artifacts import digest
from .online_io import Journal, atomic_json
from .online_epochs import Cadence, prefix, freeze_epoch
from .persistent_cbs import PersistentSession, save_snapshot


def run(work, source, config, interval=None):
    work = Path(work); robots = config['robots']
    interval = float(interval if interval is not None else config['dpgo'].get('update_interval_s',10.))
    cadence = Cadence(interval,time.monotonic()); root = work/'epochs'; root.mkdir()
    events = Journal(root/'events.jsonl'); session = None; accepted = []; pending = None
    captured_final = False; identity = None; epoch = None; published = None; last_poll = 0.
    coalesced = 0; ready_started = time.monotonic()
    try:
        session = PersistentSession(config,source,root/'persistent',
            {r:work/f'prepared-{r}/store' for r in robots})
        atomic_json(root/'READY',dict(mode='persistent',update_interval_s=interval,
            overrun_policy='coalesce complete growing prefixes while admission is busy; retain every measurement'))
        while True:
            session.check(); now = time.monotonic()
            done = all((work/f'live/{r}/DONE').exists() for r in robots)
            tick = None if captured_final else cadence.take(now,final=done)
            if tick is not None:
                value = prefix(work,robots,previous=accepted)
                if value is not None:
                    rows,edges = value; accepted = edges
                    new_id = digest(dict(rows=rows,edges=edges))
                    if new_id != identity or done:
                        if pending: coalesced += 1
                        pending = (rows,edges,new_id,time.time_ns(),dict(tick,final=done,target_interval_s=interval))
                        identity = new_id
                elif done: raise ValueError('Capture ended without initialized input for every robot')
                captured_final = done
            if now-last_poll > .2:
                last_poll = now
                snapshot = session.snapshot() if session.revision else None
                key = None if snapshot is None else digest({k:snapshot['summary'][k] for k in ('revision','iterations','finished')})
                if snapshot and key != published:
                    published = key; save_snapshot(snapshot,epoch/'dpgo')
                    publication = dict(revision=session.revision,path=str(epoch/'dpgo'),
                        cutoff_wall_ns=metadata['cutoff_wall_ns'],published_wall_ns=time.time_ns(),
                        loops=snapshot['summary']['loops'],inputs=str(epoch/'input.json'),mode='persistent',
                        iterations=snapshot['summary']['iterations'])
                    atomic_json(epoch/'publication.json',publication); atomic_json(root/'latest.json',publication)
                    events.append(dict(type='epoch_published',**publication))
                    if snapshot['summary']['finished']:
                        atomic_json(root/'DONE',dict(revisions=session.revision,mode='persistent',
                            coalesced_prefixes=coalesced,final_loops=len(accepted),finished_wall_ns=time.time_ns()))
                        return
            if pending and session.submitted is None and session.ready():
                rows,edges,key,cutoff,detail = pending; pending = None
                epoch = freeze_epoch(work,root,session.revision+1,rows,edges,key,cutoff,detail)
                metadata = dict(cutoff_wall_ns=cutoff)
                session.submit(rows,edges,final=detail['final'])
                events.append(dict(type='epoch_started',revision=session.revision,cutoff_wall_ns=cutoff,
                    mode='persistent',coalesced_prefixes=coalesced))
            if not session.revision and now-ready_started > config['dpgo'].get('timeout_s',600):
                raise TimeoutError('Persistent CBS startup/input timeout')
            time.sleep(.05)
    except BaseException as error:
        atomic_json(root/'FAILED',dict(error=str(error),mode='persistent',wall_ns=time.time_ns()))
        raise
    finally:
        if session: session.close()
        events.close()

"""Deterministic delivery scheduler; retrieval and verification run only in workers."""
import base64
from collections import defaultdict,deque
import heapq
import itertools
import multiprocessing as mp
from pathlib import Path
import resource
import time

from .artifacts import canonical, digest, read_json, read_jsonl, write_json, write_jsonl
from .backends import create, pack_payload, unpack_payload
from .data import LocalStore
from .geometry import canonical_edge, constraint
from .isolation import restrict_reads, worker_paths
from .verification import Verifier,VerificationProcess


def available_ns(row):
    return row.get('available_ns', row['stamp_ns'])


def eligible_observation(row, query, same_robot, exclusion_ns):
    if not row.get('retrievable', True) or available_ns(row) > query['stamp_ns']:
        return False
    if not same_robot:
        return True
    anchor = query.get('anchor_ns', query['stamp_ns'])
    if anchor - row['stamp_ns'] < exclusion_ns:
        return False
    if 'begin_ns' in query and 'begin_ns' in row:
        if max(query['begin_ns'], row['begin_ns']) < min(query['end_ns'], row['end_ns']):
            return False
    return True


def select_branches(candidates,limits,last,stamp,cooldown_ns):
    """Independent raw-score budgets; coalesce a pair selected by both branches."""
    chosen={};rejected=[]
    for branch,limit in limits.items():
        ranked=sorted(((r,x) for r,x in candidates if branch in x.get('sources',[])),
                      key=lambda p:(-p[1]['branch_scores'][branch],p[0],p[1]['keyframe_id']))
        count=0
        for robot,item in ranked:
            endpoint=(robot,item['keyframe_id']);reason=None
            if count>=limit:reason='verification_budget'
            elif stamp-last.get((robot,branch),-10**30)<cooldown_ns:reason='verification_cooldown'
            if reason:
                rejected.append(dict(type='rejection',reason=reason,branch=branch,candidate=list(endpoint)))
                continue
            selected=chosen.setdefault(endpoint,dict(item,selected_branches=[]))
            selected['selected_branches'].append(branch);count+=1;last[robot,branch]=stamp
    return [(r,item) for (r,k),item in chosen.items()],rejected


class Worker:
    def __init__(self,robot,store,descriptors,backend,cfg):
        self.robot=robot; self.store=LocalStore(store,robot);self.descriptors=Path(descriptors)
        self.backend=create(backend);self.cfg=cfg;self.index={};self.seen=set();self.requested=set()
        self.backend_name=backend['name']
        self.backend_config=backend;self.verifier=None;self.pending=deque();self.completed=[]
        self.candidate_replies={};self.last_verification={}
        self.queue_size=cfg.get('verification_queue_size',2)
        if type(self.queue_size) is not int or not 0<=self.queue_size<=8:
            raise ValueError('verification_queue_size must be 0..8 (0 selects serial verification)')
        self.verifier_rss=0

    def finish_verification(self,event,now,reply,preparation_s):
        body=event['body'];q=body['query_id'];c=body['candidate_id'];src=event['src']
        result=reply['result']
        result.update(type='verification',query=[self.robot,q],candidate=[src,c],score=body['score'],
            query_stamp_ns=body['query_stamp_ns'],delivery_ns=now,
            simulated_detection_latency_s=(now-body['query_stamp_ns'])/1e9,
            verification_runtime_s=reply['seconds']+preparation_s,verification_cache_hit=reply['cache_hit'])
        outgoing=[]
        if result['accepted']:
            diagnostics={k:v for k,v in result.items() if k not in ('verification_runtime_s','verification_cache_hit')}
            edge=constraint([self.robot,q],[src,c],result['T_i_j'],result['information'],
                accepted=True,kind='loop',backend=self.backend_name,diagnostics=diagnostics)
            outgoing.append(dict(src=self.robot,dst='optimizer',kind='constraint',body=canonical_edge(edge)))
        if self.queue_size:self.verifier_rss=max(self.verifier_rss,reply['max_rss_kib'])
        return dict(id=event['_verification_id'],records=[result],outgoing=outgoing)

    def drain_one(self):
        event,now,preparation_s,future=self.pending.popleft()
        self.completed.append(self.finish_verification(event,now,future.result(),preparation_s))

    def collect(self,wait=False):
        while self.pending and (wait or self.pending[0][3].done()):self.drain_one()
        result=self.completed;self.completed=[];return result

    def close(self):
        try:
            if isinstance(self.verifier,VerificationProcess):self.verifier.close()
        finally:self.backend.close()

    def descriptor(self,key):
        row=self.store.row(key)
        import json,zlib
        path=self.descriptors/f'{key:06d}.json.zlib'
        descriptor=json.loads(zlib.decompress(path.read_bytes())) if path.exists() else read_json(self.descriptors/f'{key:06d}.json')
        if 'submap_id' in row and (descriptor.get('submap_id')!=row['submap_id'] or
            descriptor.get('member_scan_ids')!=row['member_scan_ids'] or descriptor.get('payload_sha256')!=row['sha256']):
            raise ValueError('Submap descriptor/evidence membership mismatch')
        return descriptor

    def payload(self,key):
        if key not in self.seen:raise ValueError('Future payload requested')
        row,cloud,image=self.store.payload(key)
        # Geometry is requested only for selected descriptor proposals.
        from .registration import bounded_cloud
        evidence=self.backend_config.get('evidence',{})
        sampling=evidence.get('sampling','adaptive')
        if sampling not in ('adaptive','fixed'):raise ValueError('Unknown evidence sampling policy')
        if row.get('strategy')=='coverage' and (sampling!='fixed' or self.backend_config['registration'].get('sampling')!='fixed'):
            raise ValueError('Coverage submaps require fixed-resolution verification')
        budget=None if sampling=='fixed' else evidence.get('max_points',16000)
        original_points=len(cloud)
        cloud,resolution=bounded_cloud(cloud,evidence.get('voxel_m',.5),budget,80.)
        preprocessing=dict(sampling_policy=sampling,requested_voxel_m=evidence.get('voxel_m',.5),
                           effective_voxel_m=resolution,input_points=original_points,output_points=len(cloud),max_range_m=80.)
        image=b''
        return dict(row=row,cloud=cloud,image=image,descriptor=self.descriptor(key),evidence_preprocessing=preprocessing)

    def handle(self,event,now):
        outgoing=[];records=[]
        def send(dst,kind,body):outgoing.append(dict(src=self.robot,dst=dst,kind=kind,body=body))
        kind=event['kind'];body=event['body']
        if kind=='observe':
            key=body['keyframe_id'];row=self.store.row(key)
            if available_ns(row)!=now or not row.get('retrievable',True) or key in self.seen:raise ValueError('Invalid observation schedule')
            self.seen.add(key);self.index[key]=self.descriptor(key)
            q=dict(query_id=key,stamp_ns=now,anchor_ns=row['stamp_ns'],descriptor=self.index[key])
            q.update({k:row[k] for k in ('begin_ns','end_ns') if k in row})
            for robot in self.cfg['robots']:send(robot,'query',q)
        elif kind=='query':
            src=event['src'];q=body['query_id'];stamp=body['stamp_ns']
            eligible={k:v for k,v in self.index.items() if eligible_observation(self.store.row(k), body,
                src==self.robot, round(self.cfg['same_robot_exclusion_s']*1e9))}
            ranked=self.backend.retrieve(body['descriptor'],eligible,self.cfg['top_k'])
            send(src,'candidates',dict(query_id=q,query_stamp_ns=stamp,candidates=ranked))
        elif kind=='candidates':
            q=body['query_id'];src=event['src']
            if q not in self.seen:raise ValueError('Reply for unseen query')
            records.append(dict(type='ranked',query=[self.robot,q],candidate_robot=src,
                query_stamp_ns=body['query_stamp_ns'],delivery_ns=now,candidates=body['candidates']))
            replies=self.candidate_replies.setdefault(q,{})
            if src in replies:raise ValueError('Duplicate candidate response')
            replies[src]=body
            if len(replies)<len(self.cfg['robots']):return outgoing,records
            del self.candidate_replies[q]
            candidates=sorted(((r,item) for r,b in replies.items() for item in b['candidates']),
                              key=lambda x:(-x[1]['score'],x[0],x[1]['keyframe_id']))
            selected,rejected=select_branches(candidates,self.cfg['branch_verification_limits'],
                self.last_verification,body['query_stamp_ns'],round(self.cfg.get('min_verification_interval_s',2.)*1e9))
            records.extend(dict(r,query=[self.robot,q]) for r in rejected)
            for src,item in candidates:
                if not item['eligible']:
                    records.append(dict(type='rejection',reason=item['rejection_reason'],
                        query=[self.robot,q],candidate=[src,item['keyframe_id']]))
            for src,item in selected:
                key=item['keyframe_id'];self.requested.add((q,src,key))
                send(src,'payload_request',dict(query_id=q,candidate_id=key,query_stamp_ns=body['query_stamp_ns'],
                    score=item['score'],proposal=item))
            return outgoing,records
        elif kind=='payload_request':
            key=body['candidate_id']
            if key not in self.seen or available_ns(self.store.row(key))>body['query_stamp_ns']:
                records.append(dict(type='rejection',reason='future_payload',candidate=[self.robot,key]));return outgoing,records
            send(event['src'],'payload_response',dict(**body,payload=pack_payload(self.payload(key))))
        elif kind=='payload_response':
            q=body['query_id'];c=body['candidate_id'];src=event['src']
            if (q,src,c) not in self.requested:raise ValueError('Unrequested payload')
            candidate_row=body['payload']['row']
            if candidate_row['robot_id']!=src or candidate_row['keyframe_id']!=c or available_ns(candidate_row)>body['query_stamp_ns']:
                raise ValueError('Invalid candidate evidence identity')
            if self.verifier is None:
                self.verifier=VerificationProcess(self.backend_config,self.cfg.get('worker_timeout_s',300)) if self.queue_size else Verifier(self.backend_config,self.backend)
            if self.queue_size and len(self.pending)>=self.queue_size:self.drain_one()
            start=time.monotonic();packet=dict(query=pack_payload(self.payload(q)),candidate=body['payload'])
            if 'proposal' in body:packet['proposal']=body['proposal']
            preparation_s=time.monotonic()-start
            # Retain only timing/identity here; the bounded IPC future owns evidence.
            metadata=dict(event,body={k:v for k,v in body.items() if k!='payload'})
            if self.queue_size:self.pending.append((metadata,now,preparation_s,self.verifier.submit(packet)))
            else:self.completed.append(self.finish_verification(metadata,now,self.verifier.verify(packet),preparation_s))
        else:raise ValueError(f'Unknown worker event {kind}')
        return outgoing,records


def worker_main(conn,robot,store,descriptors,backend,cfg):
    worker=None
    try:
        restrict_reads(worker_paths(store,descriptors,backend))
        worker=Worker(robot,store,descriptors,backend,cfg)
        conn.send_bytes(canonical({'ready':True,'robot':robot}))
        import json
        while True:
            data=json.loads(conn.recv_bytes())
            if data['op']=='close':
                completed=worker.collect(wait=True);worker.close()
                conn.send_bytes(canonical(dict(closed=True,completed=completed,verifier_max_rss_kib=worker.verifier_rss)))
                worker=None;break
            outgoing,records=worker.handle(data['event'],data['now'])
            conn.send_bytes(canonical(dict(outgoing=outgoing,records=records,
                completed=worker.collect(),verifier_max_rss_kib=worker.verifier_rss,
                max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)))
    except BaseException as exc:
        import traceback
        try:conn.send_bytes(canonical({'error':str(exc),'traceback':traceback.format_exc()}))
        except (BrokenPipeError,EOFError):pass
    finally:
        if worker:
            try:worker.close()
            except Exception:pass
        conn.close()


def replay(stores,descriptor_roots,backend,cfg,output):
    import json,zlib,hashlib
    cfg=dict(cfg,verification_queue_size=cfg.get('verification_queue_size',2))
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    ctx=mp.get_context('spawn');connections={};processes={};rss={};verifier_rss={};counter=itertools.count();heap=[]
    rows={robot:read_jsonl(Path(path)/'keyframes.jsonl') for robot,path in stores.items()}
    epoch=min(available_ns(r[0]) for r in rows.values())
    accepted={};record_batches=[];wire=[]
    pending_verifications={};optimizer_messages=[];accepted_proposals=set();submitted=0;verified=0
    observed=0;handled=0;wall_start=time.monotonic();last_progress=wall_start
    total_observations=sum(len(track) for track in rows.values())
    def receive(robot,draining=False):
        timeout=cfg.get('worker_timeout_s',300)*(cfg.get('verification_queue_size',2)+1 if draining else 1)
        if not connections[robot].poll(timeout):
            raise TimeoutError(f'Worker {robot} timed out')
        result=json.loads(connections[robot].recv_bytes())
        if 'error' in result:raise RuntimeError(f'{robot}: {result["traceback"]}')
        return result
    def encode(message):return zlib.compress(canonical(dict(schema_version=1,**message)))
    def account(message,encoded,identifier,sent,delivery):
        is_network=message['src']!=message['dst']
        wire.append(dict(id=identifier,src=message['src'],dst=message['dst'],kind=message['kind'],
            sent_ns=sent,delivery_ns=delivery,serialized_bytes=len(encoded),network_bytes=len(encoded) if is_network else 0,
            sha256=hashlib.sha256(encoded).hexdigest(),query_id=message['body'].get('query_id'),
            candidate_id=message['body'].get('candidate_id')))
    def collect(result,robot):
        nonlocal verified
        verifier_rss[robot]=max(verifier_rss.get(robot,0),result.get('verifier_max_rss_kib',0))
        for completed in result.get('completed',[]):
            order,identifier=pending_verifications.pop(completed['id'])
            if order[2]!=robot:raise ValueError('Verification returned to the wrong robot')
            record_batches.append((order,completed['records']));verified+=1
            if len(completed['outgoing'])>1:raise ValueError('Multiple constraints for one verification')
            for message in completed['outgoing']:
                # There is no optimizer feedback. Its dedicated directed links can
                # be evaluated after geometry completes, in their original logical
                # send order, without delaying or reordering any retrieval event.
                if (message['src'],message['dst'],message['kind'])!=(robot,'optimizer','constraint'):
                    raise ValueError('Asynchronous verification may only emit optimizer constraints')
                optimizer_messages.append((order,identifier,message,encode(message)))
                edge=message['body'];accepted_proposals.add((tuple(edge['i']),tuple(edge['j'])))
    def progress(now,phase='replay_and_verification'):
        nonlocal last_progress
        last_progress=time.monotonic()
        write_json(output/'progress.json',dict(phase=phase,observations=observed,total_observations=total_observations,
            handled_events=handled,pending_events=len(heap),accepted=len(accepted_proposals),
            verification_submitted=submitted,verification_completed=verified,verification_pending=len(pending_verifications),
            replay_s=(now-epoch)/1e9,wall_s=time.monotonic()-wall_start))
        print(f'Loop replay: {observed}/{total_observations} observations, {handled} events, '
              f'{verified}/{submitted} verifications, {len(accepted_proposals)} constraints',flush=True)
    try:
        for robot in sorted(stores):
            robot_backend=dict(backend,requesting_robot=robot)
            if backend.get('verification_cache'):
                cache=Path(backend['verification_cache'])/robot;cache.mkdir(parents=True,exist_ok=True)
                robot_backend['local_cache']=str(cache)
                robot_backend['read_paths']=[*backend.get('read_paths',[]),str(cache)]
            parent,child=ctx.Pipe();connections[robot]=parent
            proc=ctx.Process(target=worker_main,args=(child,robot,str(stores[robot]),str(descriptor_roots[robot]),robot_backend,cfg))
            proc.start();child.close();processes[robot]=proc
            receive(robot)
            for row in rows[robot]:
                heapq.heappush(heap,(available_ns(row),0,robot,row['keyframe_id'],next(counter),
                    dict(src=robot,dst=robot,kind='observe',body={'keyframe_id':row['keyframe_id']})))
        while heap:
            now,priority,destination,frame,seq,event=heapq.heappop(heap)
            order=(now,priority,destination,frame,seq)
            if isinstance(event, str):
                event_path=Path(event)
                event=json.loads(zlib.decompress(event_path.read_bytes()))
                event_path.unlink()
            handled+=1
            if event['kind']=='observe':observed+=1
            if handled%1000==0 or (event['kind']=='observe' and observed%50==0) or time.monotonic()-last_progress>=30:
                progress(now)
            if event['kind']=='payload_response':
                # Reserve a deterministic message ID even if verification rejects.
                # Serial and concurrent execution therefore have identical IDs,
                # wire bytes, delivery times and reciprocal-edge tie breaking.
                event['_verification_id']=seq
                pending_verifications[seq]=(order,next(counter));submitted+=1
            robot=event['dst']
            connections[robot].send_bytes(canonical(dict(op='event',event=event,now=now)))
            result=receive(robot);record_batches.append((order,result['records']))
            collect(result,robot);rss[robot]=max(rss.get(robot,0),result['max_rss_kib'])
            for message in result['outgoing']:
                if message['dst']=='optimizer':raise ValueError('Optimizer message bypassed verification accounting')
                encoded=encode(message)
                delivery=now
                identifier=next(counter)
                account(message,encoded,identifier,now,delivery)
                # Spool pending payloads to bound RAM while routing serialized evidence.
                # Final transcripts retain the size and SHA256 of the exact wire bytes.
                wire_dir=output/'wire';wire_dir.mkdir(exist_ok=True)
                event_path=wire_dir/f'{identifier:09d}.json.zlib'
                event_path.write_bytes(encoded)
                heapq.heappush(heap,(delivery,1,message['dst'],0,identifier,str(event_path)))
        progress(now,'draining_verification')
        for robot in sorted(connections):
            connections[robot].send_bytes(canonical({'op':'close'}));collect(receive(robot,draining=True),robot)
            processes[robot].join(timeout=10)
            if processes[robot].exitcode!=0:raise RuntimeError('Worker failed to exit cleanly')
        if pending_verifications:raise ValueError('Missing verification results')
        arrivals=[]
        for order,identifier,message,encoded in sorted(optimizer_messages,key=lambda item:item[0]):
            sent=order[0];delivery=sent
            account(message,encoded,identifier,sent,delivery)
            arrivals.append((delivery,identifier,message['body']))
        for delivery,identifier,edge in sorted(arrivals,key=lambda item:(item[0],item[1])):
            key=(tuple(edge['i']),tuple(edge['j']))
            if key in accepted:
                record_batches.append(((delivery,1,'optimizer',0,identifier),
                    [dict(type='rejection',reason='duplicate_constraint',i=edge['i'],j=edge['j'])]))
            else:accepted[key]=edge
        handled+=len(arrivals)
        progress(max([now,*[item[0] for item in arrivals]]),'complete')
    finally:
        for conn in connections.values():conn.close()
        for proc in processes.values():
            if proc.is_alive():proc.terminate()
            proc.join(timeout=5)
    records=[record for _,batch in sorted(record_batches,key=lambda item:item[0]) for record in batch]
    write_jsonl(output/'events.jsonl',records);write_jsonl(output/'exchanges.jsonl',sorted(wire,key=lambda item:item['id']))
    write_jsonl(output/'constraints.jsonl',[accepted[k] for k in sorted(accepted)])
    write_json(output/'summary.json',dict(robots=sorted(stores),
        network_bytes=sum(w['network_bytes'] for w in wire),messages=len(wire),accepted=len(accepted),
        wall_runtime_s=time.monotonic()-wall_start,
        worker_max_rss_kib=rss,filesystem_isolation='Linux Landlock',
        verifier_max_rss_kib=verifier_rss,verification_queue_size=cfg.get('verification_queue_size',2),
        verification_submitted=submitted,verification_completed=verified,
        verification_execution=('bounded robot-owned processes; causal retrieval order; optimizer-only results'
                                if cfg.get('verification_queue_size',2) else 'serial within retrieval workers'),
        wire_codec='zlib-compressed canonical JSON v1, including envelope',
        latency_semantics='unrestricted deterministic delivery; compute runtime reported separately'))

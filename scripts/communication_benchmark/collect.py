#!/usr/bin/env python3
"""Collect lightweight measured results, retaining failure and accounting scope."""
import collections,json,time
from pathlib import Path
from batch import GROUPS
from launch import ROOT,save
from meter import native_logical_totals

def read(path,default=None):
 return json.loads(path.read_text()) if path.exists() else default
def lines(path):
 if not path.exists():return []
 return [json.loads(s) for s in path.read_text().splitlines() if s.strip().endswith('}')]
def partition(robots,links):
 parent={r:r for r in robots}
 def root(r):
  while parent[r]!=r:r=parent[r]
  return r
 for i,j in links:
  if i in parent and j in parent:parent[root(j)]=root(i)
 groups=collections.defaultdict(list)
 for r in robots:groups[root(r)].append(r)
 return sorted(groups.values(),key=lambda x:(-len(x),x))

def native_connectivity(method,out,robots,summary):
 links=[];detail={};loops=None
 if method=='dcl':
  completed={r:sum(e['robot']==r and e['state']==7 for e in read(out/'state-events.json',[])) for r in robots}
  kept=set()
  for p in out.with_name(out.name+'-pcm').glob('consistent_loop_closures_*.txt'):
   for line in p.read_text().splitlines():
    v=line.split()
    if len(v)!=2:continue
    pair=tuple(map(int,v));i,j=[(x>>56)-ord('a') for x in pair]
    if 0<=i<len(robots) and 0<=j<len(robots) and i!=j:
     kept.add(tuple(sorted(pair)))
     if completed[robots[i]] and completed[robots[j]]:links.append((robots[i],robots[j]))
  loops=len(kept);detail=dict(completed_optimizer_cycles=completed,loop_count_scope='latest PCM-kept endpoint union; not all-run loop count')
 elif method=='disco':
  transforms=read(out/'transforms.json',{})
  for r in robots[1:]:
   if r in transforms:links.append((robots[0],r))
  loops=summary.get('loop_messages');detail['loop_count_scope']='native loop messages; not deduplicated accepted constraints'
 elif method=='swarm':
  optimized=read(out/'optimized.json',{});groups=collections.defaultdict(list)
  for i,r in enumerate(robots):
   if str(i) in optimized:groups[optimized[str(i)]['origin_robot_id']].append(r)
  for group in groups.values():links.extend(zip(group,group[1:]))
  loops=summary.get('accepted_inter');detail.update(coverage=summary.get('coverage'),loop_count_scope='accepted inter-robot messages')
 else:
  detail['connectivity_scope']='await native sequential-session evaluation; no peer-network interpretation'
  return None,None,detail
 return partition(robots,links),loops,detail

def collect_one(method,group,attempt):
 tag=method+'-'+group+'-'+attempt;status=read(ROOT/'status'/(tag+'.json'),{})
 row=dict(method=method,group=group,attempt=attempt,status=status.get('phase','pending'),bytes=None,valid_total=False,components=None)
 if not status:
  if method=='gac' and group in ['GRACO_aerial','GRACO_mixed']:row.update(status='excluded',reason='Native camera input absent')
  return row
 row['run_status']=status;robots=status.get('robots',[]);out=ROOT/'runs'/tag
 row['robots']=robots;row['output']=str(out)
 if method=='ours':
  categories=collections.Counter();live_events=[]
  for robot in robots:
   for msg in lines(out/f'live/{robot}/wire.jsonl'):
    kind=msg['kind'];category='retrieval' if kind=='query' else 'verification_geometry' if kind=='payload_response' else 'retrieval_control'
    categories[category]+=msg['network_cdr_bytes']
   live_events+=lines(out/f'live/{robot}/events.jsonl')
  summaries=list((out/'epochs').glob('*/dpgo/summary.json'));epoch_inputs=list((out/'epochs').glob('*/input.json'))
  for file in summaries:
   summary=read(file)
   for key,label in [('registration_network_cdr_bytes','registration_geometry'),('pcm_network_cdr_bytes','pcm'),('cbs_network_cdr_bytes','cbs_beliefs'),('loop_network_cdr_bytes','epoch_control')]:categories[label]+=summary[key]
  row.update(bytes=sum(categories.values()),categories=dict(categories),measurement='native serialized CDR sender/service counters',
   complete_epochs=len(summaries),captured_epochs=len(epoch_inputs),
   valid_total=status['phase']=='complete' and len(summaries)==len(epoch_inputs),
   scope='Live retrieval plus every completed immutable PCM/CBS revision; failed/in-flight native revision traffic may be missing and is a lower bound.')
  epoch_events=lines(out/'epochs/events.jsonl');queued=[x for x in epoch_events if x['type']=='epoch_queued'];published=[x for x in epoch_events if x['type']=='epoch_published']
  row['cadence']=dict(target_s=10.,capture_intervals_s=[(b['cutoff_wall_ns']-a['cutoff_wall_ns'])/1e9 for a,b in zip(queued,queued[1:])],
   publications=[dict(revision=x['revision'],delay_s=x['input_to_publication_s'],queue_s=x['queue_wait_s'],wall_ns=x['published_wall_ns']) for x in published],
   missed_capture_deadlines=sum(x['missed_deadlines'] for x in queued))
  latest=read(out/'epochs/latest.json')
  if latest:
   final=ROOT/'runs'/tag/'epochs'/('%03d'%latest['revision'])/'dpgo';poses=lines(final/'poses.jsonl')
   if poses:
    components=collections.defaultdict(set)
    for p in poses:components[p['component']].add(p['robot_id'])
    row['components']=sorted([sorted(v) for v in components.values()],key=lambda x:(-len(x),x))
   summary=read(final/'summary.json',{});row['loops']=summary.get('loops');row['pcm_rejected']=read(final/'pcm.json',{}).get('excluded_loops')
   row['component_scope']='latest published revision; final only when status is complete'
  row['verification']=dict(collections.Counter(e['reason'] for e in live_events if e['type']=='verification'))
  row['raw_audit']=read(out/'raw-odometry-audit.json',{})
 else:
  summary=read(out/'summary.json',read(out/'progress.json',{}));row['native_summary']=summary
  row['status']='complete' if status.get('phase')=='finished' and summary.get('phase')=='complete' else ('failed' if status.get('phase') in ['finished','setup_failed'] else status['phase'])
  if status['phase']=='excluded':row['status']='excluded'
  observed=read(out/'communication-measured.json',{})
  if method=='swarm':
   measured=read(out/'communication-publications.json',{})
   if measured:row.update(bytes=measured['total_peer_payload_bytes'],categories=measured['by_category'],valid_total=measured['complete_probe'],
      measurement='native RCL serialized publication probe',scope=measured['scope'],
      accounting=dict(serialization_errors=measured['serialization_errors'],unmapped_topics=measured['unmapped_topics'],before_discovery_bytes=measured['before_first_graph_discovery_bytes'],observer_bytes=measured['observer_payload_bytes']))
  else:
   bus=read(out/'communication-native-bus.json',{})
   if bus.get('connections'):
    measured=native_logical_totals(bus['connections'],method)
    row.update(bytes=measured['total_peer_payload_bytes'],categories=measured['by_category'],
      measurement='native TCPROS publisher counters minus four-byte message framing',valid_total=bool(observed.get('finished')) and not observed.get('observer_errors'),
      scope='Native serialized message-body bytes per publisher/topic/remote robot; reconnects summed, duplicate receiving processes merged by maximum node total. Excludes local/observer links and TCP/IP framing.',
      accounting=dict(native_connection_bytes=measured['native_all_peer_connection_bytes'],observer_bytes=observed.get('total_peer_payload_bytes'),observer_errors=observed.get('observer_errors')))
   if method=='gac':row['scope']='Centralized native frontend-to-backend input proxy; not native interrobot traffic. '+row.get('scope','')
  row['components'],row['loops'],row['connectivity_details']=native_connectivity(method,out,robots,summary)
  row['divergence']=read(out/'divergence.json')
  if row['divergence']:row['status']='stopped_divergence'
  elif status.get('timed_out'):row['status']='timeout'
  row['wall_s']=summary.get('wall_s')
 return row

if __name__=='__main__':
 selections=read(ROOT/'attempt-selection.json',{})
 rows=[]
 for group in GROUPS:
  for method in ['ours','swarm','dcl','disco','gac']:
   selection=selections.get(method+':'+group,{})
   row=collect_one(method,group,selection.get('attempt','full-v2'))
   if selection:
    row['attempt_selection']=selection
    row['previous_attempt']=collect_one(method,group,selection['previous_attempt'])
   rows.append(row)
 lanes=[read(p) for p in sorted((ROOT/'status').glob('lane-*-full-v2.json'))]
 for attempt in sorted({s['attempt'] for s in selections.values()}):
  lanes.extend(read(p) for p in sorted((ROOT/'status').glob('lane-*-'+attempt+'.json')))
 result=dict(schema_version=1,collected_unix=time.time(),rows=rows,lanes=lanes,
  campaign_complete=len(lanes)>=6 and all(x['phase']=='finished' for x in lanes)
   and all(x['status'] in ['complete','failed','excluded','stopped_divergence','timeout'] for x in rows),
  protocol='Three matched GRACO trios and 12 selected S3E trios; native baseline methods unchanged; ours uses frozen default point-cloud frontend evidence replayed at 1x and ten-second CBS input capture. Bulk retained on 148.')
 save(ROOT/'report/measured-results.json',result)
 print(collections.Counter(x['status'] for x in rows))

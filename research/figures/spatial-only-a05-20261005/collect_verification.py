from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
root=Path('/workspace/.ros2/spatial-only148-20261005')
source=root/'source'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def tests(p):
    x=ET.parse(p).getroot(); suites=[x] if x.tag=='testsuite' else list(x.iter('testsuite'))
    values={k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}
    assert not values['failures'] and not values['errors'], values
    values['passed']=values['tests']-values['skipped']
    return values
result=dict(scope='Implementation checks plus A05 saved-publication-prefix topology; no new dataset accuracy benchmark',
    native_cpp={k:tests(root/f'{k}.xml') for k in ('core','ros')},native_python=tests(root/'python.xml'),
    source_hashes={},binary_hashes={},native_scenarios={})
for package,paths in {
    'FAST-LIVO2-ROS2':['research/s3e_pipeline','research/tests','scripts/circle_submaps'],
    'cbs':['include/cbs/robust','src/robust','tests/test_inter_robot_pcm.cpp'],
    'cbs_ros':['include/cbs_ros/distributed_pcm.h','src/distributed_pcm.cpp','src/cbs_ros_node.cpp','msg/PcmPacket.msg','tests/test_distributed_pcm.cpp']}.items():
    for name in paths:
        p=source/package/name
        for q in ([p] if p.is_file() else sorted(p.rglob('*'))):
            if q.is_file() and '__pycache__' not in q.parts:
                result['source_hashes'][str(q.relative_to(source))]=sha(q)
for name in ['cbs/lib/libcbs.so','cbs_ros/lib/cbs_ros/cbs_ros_node']:
    p=source/'.ros2/cbs-spatial-install'/name
    result['binary_hashes'][name]=sha(p)
for pattern in ['test_native_growing_graph*','test_native_pending_component*']:
    for test in sorted((root/'test-work').glob(pattern)):
        if test.is_symlink():continue
        summaries={p.parent.name:json.loads(p.read_text()) for p in (test/'session').glob('*/summary.json')}
        result['native_scenarios'][test.name]={r:{k:s.get(k) for k in ['revision','finished','loops','map_consistency_factors','alignment_complete','unconnected_submaps','registration_network_cdr_bytes','pcm_network_cdr_bytes']} for r,s in summaries.items()}
        assert summaries and all(s['finished'] and s['alignment_complete'] and s['registration']['success'] for s in summaries.values())
result['a05']=json.loads((root/'a05-final/graph.json').read_text())
result['a05']={k:result['a05'][k] for k in ['nodes','odometry_edges','map_consistency_edges','local_cycle_rank','component_count','input_sha256','construction_wall_s']}
(root/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
print({k:v for k,v in result.items() if k not in ('source_hashes','binary_hashes','native_scenarios')})

"""Verify retained experimental artifacts without changing estimation outputs."""
from pathlib import Path
import hashlib,json,time,shutil,xml.etree.ElementTree as ET

B=Path(__file__).resolve().parent;W=B/'full'
read=lambda p:json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def check(p,value):
    assert sha(p)==value, f'Hash changed: {p}'

assert read(W/'status.json')['phase']=='complete'
runtime=read(W/'runtime-audit.json')
assert runtime['frozen_sources_verified'] and runtime['graph_anchors_verified']
for p,h in read(W/'source-hashes.json').items():check(Path(p),h)
for p,h in read(W/'frontend-binaries.json').items():check(Path(p),h)
robots=list(read(W/'trials.json'));assert len(robots)==14
input_files=0
for r,item in read(W/'inputs.json').items():
    for relative,h in item['sha256'].items():check(Path(item['path'])/relative,h);input_files+=1
for values in read(W/'calibration-hashes.json').values():
    for p,h in values.items():check(Path(p),h)
gallery=read(W/'gallery/summary.json');maps=gallery['maps']
assert len(maps)==runtime['descriptor_evidence_memberships']==504
assert gallery['cached_orb_reproduced'] and not gallery['ground_truth_used']
for m in maps:
    assert m['cached_orb_reproduced']
    check(W/'gallery'/m['image'],m['image_sha256'])
    check(W/f"prepared-{m['robot']}/ellipsoid/{m['key']:06d}.json.zlib",m['descriptor_sha256'])
for r in robots:
    assert read(W/f'{r}-quality.json')['passed']
    assert '1 file verified without error.' in (W/f'{r}-recording-verification.log').read_text()
    assert (W/f'frontends/{r}/recording/live.rrd').stat().st_size>0
    for p,h in read(W/f'frontends/{r}/source-hashes.json').items():
        if p.startswith('ellipselio/') or p.endswith(('selected_mapping.yaml','libellipselio_mapping.so','ellipselio_mapping_mt','run_ellipselio.py','ellipselio_live_rerun.py','submap_writer.py')):check(Path('/workspace')/p,h)
assert '1 file verified without error.' in (W/'recording-verification.log').read_text()
assert '1 file verified without error.' in (W/'gravity-recording-verification.log').read_text()
# Keep initial fixture failures visible while reporting the latest result per test.
tests={};test_runs=[]
for name in ['tests.xml','tests-fixed.xml','tests-distributed.xml']:
    p=B/name
    root=ET.parse(p).getroot();cases=list(root.iter('testcase'))
    test_runs.append(dict(file=name,sha256=sha(p),tests=len(cases),failures=sum(any(c.tag in ('failure','error') for c in t) for t in cases)))
    for t in cases:
        name=t.attrib['name']
        # The new multilayer flag renamed the two existing parameterized cases.
        stem='test_native_three_worker_replay_cold_determinism_and_lidar_only'
        if name in (stem+'[False]',stem+'[True]'):name=name[:-1]+'-False]'
        tests[t.attrib['classname']+'::'+name]=not any(c.tag in ('failure','error','skipped') for c in t)
assert all(tests.values())
assert read(B/'smoke14-v2/verified.json')
references=read(W/'reference/source-hashes.json')
assert references['ground_truth_for_estimation'] is False
for r,row in references['files'].items():check(W/f'reference/{r}_gt.txt',row['sha256'])
report=read(W/'report/report.json')
assert all(report['ground_truth'][r]['available'] for r in robots)
assert set(report['raw'])==set(report['cbs_individual'])==set(robots)
out=dict(complete=True,verified_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
    robots=robots,sensor_files_rehashed=input_files,source_hashes_unchanged=True,calibration_hashes_unchanged=True,
    frontend_estimator_sources_and_binaries_and_selected_configs_unchanged=True,
    descriptor_evidence_memberships=504,gallery_images_and_descriptors_rehashed=504,cached_orb_reproduced=True,
    live_recordings_verified=14,result_recording_verified=True,gravity_level_recording_verified=True,latest_tests_passed=len(tests),test_runs=test_runs,
    synthetic_14_worker_preflight=read(B/'smoke14-v2/verified.json'),
    ground_truth_for_estimation=False,ground_truth_available_for_evaluation=14,
    bulk_geometry_retained=True,previous_experiments_preserved=True,
    retained_remote_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-all14-multilayer148-20260922')
(W/'retention-audit.json').write_text(json.dumps(out,indent=2)+'\n')
provenance=W/'provenance';provenance.mkdir(exist_ok=True)
for pattern in ['*.py','*.xml','*.sh','*.json','*.txt','*.log']:
    for p in B.glob(pattern):shutil.copy2(p,provenance/p.name)
for name in ['source','native']:shutil.copytree(B/name,provenance/name,dirs_exist_ok=True)
shutil.copy2(B/'smoke14-v2/verified.json',provenance/'smoke14-verified.json')
paths=[p for folder in [W/'report',W/'diagnostics',W/'gallery',W/'reference',W/'provenance',W/'configs'] for p in folder.rglob('*') if p.is_file()]
paths += [p for p in W.iterdir() if p.is_file() and p.suffix in ('.md','.json','.yaml') and p.name!='artifact-hashes.json']
paths += [W/'dpgo'/n for n in ['summary.json','pcm.json','registration.json','poses.jsonl','constraints.jsonl','proposed-constraints.jsonl']]
(W/'artifact-hashes.json').write_text(json.dumps({str(p.relative_to(W)):sha(p) for p in sorted(set(paths))},indent=2)+'\n')
print(json.dumps(out,indent=2),flush=True)

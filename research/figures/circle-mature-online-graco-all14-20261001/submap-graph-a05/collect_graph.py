#!/usr/bin/env python3
"""Collect compact, immutable graph and footprint records; no point-cloud transfer."""
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REMOTE_CODE = r'''
from pathlib import Path
import hashlib,json
p=Path('/data3/mikexyl/swarm_s3e_ws/src/.ros2/circle-mature-online148-20261001')
files={
 'graph':'inspection/solution/central/graph.json',
 'keyframes':'full/prepared-aerial05/store/keyframes.jsonl',
 'planner':'full/prepared-aerial05/planner-state.json',
 'input':'full/epochs/persistent/aerial05/input/13.json',
 'bridge':'source/FAST-LIVO2-ROS2/research/s3e_pipeline/cbs_bridge.py',
}
blobs={k:(p/v).read_bytes() for k,v in files.items()}
g=json.loads(blobs['graph']); planner=json.loads(blobs['planner'])['state']
rows=[json.loads(s) for s in blobs['keyframes'].splitlines()]
inp=json.loads(blobs['input'])
fields=('robot_id','keyframe_id','stamp_ns','created_update','T_world_body','T_horizontal_map',
        'footprint_center_xy','footprint_radius_m','geometry_count','sha256','seal_ns')
loops=[{k:e[k] for k in ('i','j','kind','accepted','T_i_j')} for e in g['constraints']
       if e['i'][0]=='aerial05' or e['j'][0]=='aerial05']
endpoints={tuple(e[k]) for e in loops for k in ('i','j')}
poses=[r for r in g['poses'] if r['robot_id']=='aerial05' or (r['robot_id'],r['keyframe_id']) in endpoints]
# The recorded bridge constructs odometry factors between successive input rows.
assert 'for a, b in zip(rows, rows[1:])' in blobs['bridge'].decode()
assert len(inp['rows'])==len(rows)==30 and inp['final']
out={
 'experiment':'circle-mature-online148-20261001/full','robot':'aerial05',
 'sources':{k:{'path':str(p/files[k]),'sha256':hashlib.sha256(v).hexdigest()} for k,v in blobs.items()},
 'graph_solver':g['solver'],'graph_total_nodes':len(g['poses']),'graph_total_loops':len(g['constraints']),
 'poses':poses,'loops':loops,'keyframes':[{k:r[k] for k in fields} for r in rows],
 'odometry_edges':[{'i':['aerial05',a['keyframe_id']],'j':['aerial05',b['keyframe_id']],
                    'kind':'odometry'} for a,b in zip(inp['rows'],inp['rows'][1:])],
 'planner':{k:planner[k] for k in ('T_horizontal_map','circles','occupied_cells','config')},
 'odometry_provenance':'Successive rows of actual final revision 13 input; construction verified against frozen runtime cbs_bridge.py',
 'ground_truth_used':False,
}
print(json.dumps(out,separators=(',',':')))
'''

result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                         '148', 'python3', '-'], input=REMOTE_CODE, text=True,
                        capture_output=True, check=True)
data = json.loads(result.stdout)
export = json.loads((ROOT.parent / 'exports/export.json').read_text())
assert data['sources']['graph']['sha256'] == export['graph_sha256']
assert data['sources']['keyframes']['sha256'] == export['robots']['aerial05']['submap_index_sha256']
(ROOT / 'graph-data.json').write_text(json.dumps(data, separators=(',', ':')) + '\n')
print(json.dumps({'local_bytes': (ROOT / 'graph-data.json').stat().st_size,
                  'poses_including_loop_neighbors': len(data['poses']),
                  'a05_nodes': len(data['keyframes']), 'odometry_edges': len(data['odometry_edges']),
                  'loops_involving_a05': len(data['loops']),
                  'loop_endpoints': [[e['i'], e['j']] for e in data['loops']],
                  'source_hashes_match_final_map_export': True}, indent=2))

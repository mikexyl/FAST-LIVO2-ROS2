#!/usr/bin/env python3
"""Audit the duplicate-stream integration fixture without ground truth."""
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Swarm-SLAM/s3e'))
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from evaluate import audit_keyframes,pose_dict


def main():
    work=ROOT/'.ros2/swarm148/smoke';native=work/'swarm'
    def read(p):return json.loads(p.read_text())
    def rows(p):return [json.loads(line) for line in p.read_text().splitlines()]
    summary=read(native/'summary.json');inputs=read(native/'inputs.json')
    source=rows(work/'frontend/Alpha/export/frames.jsonl')
    source=[r for r in source if r['stamp_ns']<=source[0]['stamp_ns']+round(inputs['duration']*1e9)]
    robots=('Alpha','Bob','Carol')
    keys={r:rows(native/f'{r}-keyframes.jsonl') for r in robots}
    audit=audit_keyframes({r:source for r in robots},keys,.5)
    packets=read(native/'optimized.json');tracks={}
    for robot,packet in packets.items():
        tracks[robot]={e['key']['keyframe_id']:np.array(pose_dict(e['pose'])) for e in packet['estimates']}
    errors=[]
    for a,b in (('0','1'),('0','2'),('1','2')):
        for key in tracks[a].keys() & tracks[b].keys():
            errors.append(float(np.linalg.norm(tracks[a][key][:3,3]-tracks[b][key][:3,3])))
    result=dict(fixture='Same Alpha input supplied to three native robots; no GT used.',
                exact_selected_inputs=audit,all_scans_processed=summary['input_replay_complete'],
                same_origin=len({v['origin_robot_id'] for v in packets.values()})==1,
                compared_pairs=len(errors),max_corresponding_position_difference_m=max(errors),
                rms_corresponding_position_difference_m=float(np.sqrt(np.mean(np.square(errors)))),
                native_success=summary['success'])
    result['passed']=bool(result['native_success'] and result['same_origin']
                          and all(v['exact_selected_timestamps_and_poses'] for v in audit.values())
                          and result['max_corresponding_position_difference_m']<.5)
    (native/'integration-check.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()

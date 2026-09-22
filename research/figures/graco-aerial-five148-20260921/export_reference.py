"""Read A04 reference only after the five-robot backend has completed."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
BASE=Path(__file__).resolve().parent
status=json.loads((BASE/'backend-status.json').read_text())
assert status['phase']=='awaiting_reference_export' and status['backend_complete']
audit=json.loads((BASE/'aerial04-conversion-audit.json').read_text())
source=Path(audit['source']);output=BASE/'reference-aerial04'
output.mkdir(exist_ok=False)
h=hashlib.sha256()
with source.open('rb') as f:
    for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
assert h.hexdigest()==audit['source_sha256']
destination=output/'aerial04_gt.txt';previous=None;count=0
with AnyReader([source]) as reader,destination.open('x') as stream:
    connections=[c for c in reader.connections if c.topic=='/gnss/ground_truth']
    assert len(connections)==1 and connections[0].msgtype=='nav_msgs/msg/Odometry'
    for connection,_,raw in reader.messages(connections=connections):
        m=reader.deserialize(raw,connection.msgtype)
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        assert previous is None or stamp>previous
        previous=stamp;p=m.pose.pose.position;q=m.pose.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w];assert np.isfinite(values).all()
        stream.write(f'{stamp//10**9}.{stamp%10**9:09d} '+' '.join(f'{v:.17g}' for v in values)+'\n');count+=1
info=dict(source=str(source),source_sha256=audit['source_sha256'],topic='/gnss/ground_truth',rows=count,
    output_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),evaluation_only=True,
    backend_completed_before_reference_access=True)
(output/'source-hashes.json').write_text(json.dumps(dict(aerial04=info),indent=2)+'\n')
print(json.dumps(info))

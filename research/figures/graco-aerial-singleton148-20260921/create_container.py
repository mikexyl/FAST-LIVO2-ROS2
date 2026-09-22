"""Run on 148: reproduce the verified environment with fresh output/source mounts."""
import json,subprocess
from pathlib import Path
ROOT=Path('/data3/mikexyl/swarm_s3e_ws/src')
B=ROOT/'.ros2/graco-aerial-singleton148-20260921'
old=json.loads(subprocess.check_output(['docker','inspect','ellipselio-graco-height148-20260921']))[0]
cmd=['docker','run','-d','--name','ellipselio-graco-singleton148-20260921','--network','host','--ipc','host',
     '--gpus','all','--security-opt','label=disable','--user',old['Config']['User'],'--entrypoint','sleep']
for m in old['Mounts']:
    source=m['Source']
    if m['Destination']=='/workspace/FAST-LIVO2-ROS2':source=str(B/'source/FAST-LIVO2-ROS2')
    cmd+=['-v',source+':'+m['Destination']+('' if m['RW'] else ':ro')]
cmd+=['-v',str(ROOT/'.ros2/graco-aerial-five148-20260921')+':/workspace/.ros2/graco-aerial-five148-20260921:ro']
cmd+=['-v',str(ROOT/'.ros2/graco-aerial-height148-20260921')+':/workspace/.ros2/graco-aerial-height148-20260921:ro',old['Config']['Image'],'infinity']
subprocess.run(cmd,check=True)
info=json.loads(subprocess.check_output(['docker','inspect','ellipselio-graco-singleton148-20260921']))[0]
limits=info['HostConfig']
assert limits['NanoCpus']==limits['Memory']==limits['CpuQuota']==0 and limits['CpusetCpus']==''
(B/'container-resources.json').write_text(json.dumps(dict(image=info['Image'],host_config=limits,mounts=info['Mounts']),indent=2)+'\n')

import subprocess,json
from pathlib import Path
B=Path(__file__).resolve().parent
old=json.loads(subprocess.check_output(['docker','inspect','ellipselio-graco-singleton148-20260921']))[0]
name='ellipselio-graco-all14-multilayer148-20260922'
args=['docker','create','--name',name,'--network','host','--ipc','host','--gpus','all','--security-opt','label=disable','--user',old['Config']['User'],'--entrypoint','sleep']
for m in old['Mounts']:
 src=m['Source'];dest=m['Destination'];ro=not m['RW']
 if dest=='/workspace/FAST-LIVO2-ROS2':src=str(B/'source/FAST-LIVO2-ROS2')
 if dest=='/opt/upstream-area-mapclosures':src=str(B/'native');Path(src).mkdir(exist_ok=True)
 args+=['-v',src+':'+dest+(':ro' if ro else '')]
args += [old['Config']['Image'],'infinity']
subprocess.run(args,check=True)
d=json.loads(subprocess.check_output(['docker','inspect',name]))[0]
h=d['HostConfig'];assert all(h[k]==0 for k in ['NanoCpus','Memory','CpuQuota']) and not h['CpusetCpus']
(B/'container-resources.json').write_text(json.dumps(d,indent=2))
subprocess.run(['docker','start',name],check=True)

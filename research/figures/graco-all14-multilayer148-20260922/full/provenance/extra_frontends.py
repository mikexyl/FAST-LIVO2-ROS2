"""Run newly validated A01–03 independently of the active first capture queue."""
from pathlib import Path
B=Path(__file__).resolve().parent
source=(B/'frontends.py').read_text()
source=source.replace("ROBOTS=[f'aerial{i:02}' for i in range(1,9)]+[f'ground{i:02}' for i in range(1,7)]","ROBOTS=['aerial01','aerial02','aerial03']")
for name in ['frontend-status.json','inputs.json','calibration-hashes.json','trials.json','frontend-binaries.json']:
 source=source.replace("W/'"+name+"'","W/'extra-"+name+"'")
source=source.replace('max_workers=4','max_workers=3').replace('concurrent_robots=4','concurrent_robots=3')
exec(compile(source,str(B/'frontends.py'),'exec'))

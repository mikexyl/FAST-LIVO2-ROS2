import json,sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
B=Path(__file__).resolve().parent
sys.path.insert(0,str(B/'source/FAST-LIVO2-ROS2/research'))
from s3e_pipeline.graco import stage_sensors
bags={'ground06':'/data/graco/ground-06_ros2','aerial06':'/data/graco/aerial-06-20m_full_ros2'}
(B/'inputs').mkdir(exist_ok=True)
def stage(item):
    robot,bag=item
    print('staging',robot,flush=True)
    result=stage_sensors(bag,B/'inputs'/robot)
    print(robot,result['counts'],flush=True)
    return robot,result
with ThreadPoolExecutor(max_workers=2) as pool:result=dict(pool.map(stage,bags.items()))
(B/'staging.json').write_text(json.dumps(result,indent=2)+'\n')

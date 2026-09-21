"""Wait for the batch, then render and audit actual cached spatial BEVs."""
from pathlib import Path
import json,subprocess,time,traceback
B=Path('/workspace/.ros2/graco-aerial-spatial148-20260920')
W=B/'full'
state=dict(phase='waiting_for_pipeline')
def save(): (B/'postprocess-status.json').write_text(json.dumps(state,indent=2)+'\n')
def run(command,log):
    with (B/log).open('x') as f: subprocess.run([str(v) for v in command],stdout=f,stderr=subprocess.STDOUT,check=True)
save()
try:
    deadline=time.monotonic()+3600
    while True:
        status=json.loads((W/'status.json').read_text())
        if status['phase']=='complete':break
        if status['phase']=='failed':raise RuntimeError('Main pipeline failed')
        if time.monotonic()>deadline:raise TimeoutError('Pipeline completion wait timed out')
        time.sleep(5)
    state['phase']='rendering_verified_bevs';save()
    py='/workspace/.ros2/research-venv/bin/python'
    run([py,'/workspace/FAST-LIVO2-ROS2/scripts/recent_submaps/render_spatial_bevs.py','--work',W,'--output',B/'bev-gallery'],'gallery.log')
    state['phase']='comparing_temporal_results';save()
    run([py,B/'analyze.py'],'comparison.log')
    state['phase']='complete';save()
except Exception as e:
    state.update(phase='failed',error=repr(e),traceback=traceback.format_exc());save();raise

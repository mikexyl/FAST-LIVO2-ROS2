import json
from pathlib import Path
import sys
import time
import psutil

ROOT=Path('/workspace');BASE=ROOT/'.ros2/upstream-area-s3e-20260921';CAP=BASE/'capacity-run'
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path.insert(0,str(SCRIPTS))
from upstream_area_batch import call,read,save,quality,PYTHON,RERUN,sha
trial=CAP/'frontends/Carol'
pid=read(trial/'frontend/processes.json')['mapper']
if Path(f'/proc/{pid}/maps').exists():
    loaded=[r for r in Path(f'/proc/{pid}/maps').read_text().splitlines() if 'libellipselio_mapping.so' in r]
    assert loaded and all('/capacity-run/install/' in r for r in loaded)
    (trial/'loaded-library.txt').write_text('\n'.join(loaded)+'\n')
    parent=psutil.Process(pid).parent().parent()
    assert 'run_trial.py' in ' '.join(parent.cmdline())
    deadline=time.monotonic()+2400
    while parent.is_running() and parent.status()!=psutil.STATUS_ZOMBIE:
        assert time.monotonic()<deadline,'Capacity rerun completion timed out'
        time.sleep(5)
assert read(trial/'frontend/summary.json')['success']
call([PYTHON,SCRIPTS/'validate.py',trial],CAP/'Carol-validation.log',600)
call(['/usr/bin/python3',SCRIPTS/'sensor_coverage.py',trial],CAP/'Carol-coverage.log',120)
result=quality(trial);save(CAP/'Carol-quality.json',result)
assert result['passed'] and read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']
footer=True
try:call([RERUN/'rerun','rrd','verify',trial/'recording/live.rrd'],CAP/'Carol-recording-verification.log',300)
except RuntimeError:
    assert 'TooManyTables' in (CAP/'Carol-recording-verification.log').read_text()
    footer=False
    call([RERUN/'rerun','rrd','verify','--check-footers=false',trial/'recording/live.rrd'],CAP/'Carol-recording-stream-verification.log',300)
save(CAP/'finished.json',dict(quality=result,recording_data_verified=True,recording_footer_verified=footer,
    library_sha256=sha(CAP/'install/ellipselio/lib/libellipselio_mapping.so'),
    initial_failure_preserved=str(BASE/'full/S3E_Campus_Road_2/frontends/Carol')))
print(json.dumps(read(CAP/'finished.json')),flush=True)

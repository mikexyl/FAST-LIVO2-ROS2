"""Convert A04 sensors only, then verify every measurement against ROS1."""
from pathlib import Path
import sys,json,subprocess,time,traceback
ROOT=Path(__file__).resolve().parents[2]
BASE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'),str(ROOT/'FAST-LIVO2-ROS2/research')]
from graco_aerial148 import verify_conversion
from s3e_pipeline.graco import stage_sensors,TOPICS
from graco_aerial import sha,save
source=Path('/data/graco/aerial-04-40m.bag')
converted=BASE/'converted/aerial04'
state=dict(phase='converting',source=str(source),ground_truth_used=False,started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
def mark(phase,**kw):
    state.update(phase=phase,**kw);save(BASE/'staging-status.json',state);print(phase,flush=True)
try:
    assert not converted.exists()
    converted.parent.mkdir(exist_ok=True)
    mark('converting')
    with (BASE/'conversion.log').open('w') as log:
        subprocess.run([str(Path(sys.executable).parent/'rosbags-convert'),'--src',str(source),'--dst',str(converted),
            '--dst-storage','sqlite3','--dst-version','5','--include-topic',*TOPICS],stdout=log,stderr=subprocess.STDOUT,check=True)
    mark('verifying_conversion')
    audit=verify_conversion(source,converted)
    audit.update(source=str(source),source_bytes=source.stat().st_size,source_sha256=sha(source))
    save(BASE/'aerial04-conversion-audit.json',audit)
    mark('staging_validated_sensors')
    info=stage_sensors(converted,BASE/'inputs/aerial04')
    mark('complete',counts=info['counts'],source_sha256=audit['source_sha256'],finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
except Exception as e:
    mark('failed',error=repr(e),traceback=traceback.format_exc());raise

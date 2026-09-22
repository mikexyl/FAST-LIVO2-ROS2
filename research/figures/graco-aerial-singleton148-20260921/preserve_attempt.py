"""Preserve the failed endpoint-order attempt before a fresh backend run."""
import json,shutil
from pathlib import Path
B=Path('/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-singleton148-20260921')
assert json.loads((B/'full/status.json').read_text())['phase']=='failed'
target=B/'failed-endpoint-order'
target.mkdir(exist_ok=False)
(B/'full').rename(target/'full')
for name in ('run.log','source-hashes.json'):
    (B/name).rename(target/name)
shutil.copytree(B/'source',target/'source')
(target/'README.md').write_text('The first singleton run stopped because native numeric robot order differed from Python name order for A04/A08. No evaluation or successful result was produced. Full logs, configuration and original frozen source are preserved.\n')

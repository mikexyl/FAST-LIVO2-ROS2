import json
from pathlib import Path
import yaml
from s3e_pipeline.dpgo import run

root=Path(__file__).resolve().parents[2]
work=Path(__file__).resolve().parent/'verification'
cfg=yaml.safe_load((work/'config.yaml').read_text())
artifacts=json.loads((work/'artifacts.json').read_text())
out=work/'dpgo';out.mkdir(exist_ok=False)
run(cfg,artifacts,root,out)

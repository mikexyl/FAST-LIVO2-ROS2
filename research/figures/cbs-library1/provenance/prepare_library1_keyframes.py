"""Prepare a completed robot's keyframe cache while the next robot replays."""
import os, sys
from pathlib import Path
import yaml
source=Path.cwd();os.environ['S3E_SOURCE_ROOT']=str(source)
import json
registry=json.loads(Path(sys.argv[2]).read_text());sys.path.insert(0,registry['research_source'])
from s3e_pipeline.artifacts import stage_output,validate_stage
from s3e_pipeline.cli import code_hash
from s3e_pipeline.data import keyframes
robot=sys.argv[1];cfg=registry['config'];stage=Path(registry['artifacts'][f'odometry.livo.{robot}']);manifest=validate_stage(stage)
camera=yaml.safe_load((stage/'run/camera_config.yaml').read_text())['/**']['ros__parameters']
with stage_output(source/cfg['output_root'],'keyframes',dict(robot=robot,**cfg['keyframes']),dict(odometry=manifest['stage_hash']),code_hash('keyframes'),True) as (out,cached):
 print(robot,'keyframes',out,'cached',cached,flush=True)
 if not cached:
  rows=keyframes(stage/'run/export',out/'store',cfg['keyframes'],camera)
  print(robot,'keyframes complete',len(rows),flush=True)

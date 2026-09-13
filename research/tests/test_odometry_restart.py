"""A per-robot bag restart must replace only its dependent artifacts."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from s3e_pipeline import cli
from s3e_pipeline.artifacts import read_json, stage_output, stage_path, write_json


def test_restart_bob_reuses_other_robots_and_invalidates_its_descriptors(tmp_path, monkeypatch):
    cfg = yaml.safe_load((Path(__file__).parents[1]/'configs/playground1-cbs.yaml').read_text())
    dataset=tmp_path/'S3E_Playground_1';dataset.mkdir()
    (dataset/'metadata.yaml').write_text('{}')
    root=tmp_path/'run';cfg.update(dataset=str(dataset),output_root=str(root))
    artifacts={}; old_paths={}
    for robot in cfg['robots']:
        settings=dict(robot=robot,rate=1.)
        with stage_output(root,'odometry',settings,{},'old') as (out,_):
            (out/'run').mkdir()
            summary=dict(bag=str(dataset),success=True)
            write_json(out/'run/summary.json',summary);write_json(out/'summary.json',summary)
        old=stage_path(root,'odometry',settings,{},'old');old_paths[robot]=old
        artifacts[f'odometry.livo.{robot}']=str(old)
        old_hash=read_json(old/'COMPLETE.json')['stage_hash']
        with stage_output(root,'keyframes',dict(robot=robot),dict(odometry=old_hash),'old') as (out,_):pass
        keys=stage_path(root,'keyframes',dict(robot=robot),dict(odometry=old_hash),'old')
        artifacts[f'keyframes.livo.{robot}']=str(keys)
    original=dict(config=dict(cfg,odometry=dict(rate=1.)),artifacts=artifacts)
    registry=tmp_path/'input.json';write_json(registry,original)
    config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump(cfg))
    commands=[]
    def replay(command,**kwargs):
        commands.append(command)
        out=Path(command[command.index('--output')+1]);out.mkdir()
        write_json(out/'summary.json',dict(bag=str(dataset),success=True,
                   start_offset_s=float(command[command.index('--start-offset')+1])))
    monkeypatch.setattr(cli.subprocess,'run',replay)
    args=SimpleNamespace(config=config,input_run=registry,stage=['odometry'],resume=True,odometry_robots=['Bob'])
    cli.run(args)
    assert len(commands)==1 and commands[0][commands[0].index('--robot')+1]=='Bob'
    assert commands[0][commands[0].index('--start-offset')+1]=='22.0'
    result=read_json(next(root.glob('run-*.json')))['artifacts']
    for robot in ('Alpha','Carol'):
        assert result[f'odometry.livo.{robot}']==str(old_paths[robot])
        assert result[f'keyframes.livo.{robot}']==artifacts[f'keyframes.livo.{robot}']
    assert result['odometry.livo.Bob']!=str(old_paths['Bob'])
    assert 'keyframes.livo.Bob' not in result
    assert read_json(old_paths['Bob']/'run/summary.json')==dict(bag=str(dataset),success=True)
    # Applying the offset configuration to the old exports cannot silently
    # enter descriptors or optimization without the requested odometry replay.
    args.stage=['descriptors']
    with pytest.raises(ValueError,match='start offset differs for Bob'):
        cli.run(args)


@pytest.mark.parametrize('offset',[-1,float('nan'),float('inf')])
def test_invalid_start_offset_rejected_before_work(tmp_path,offset):
    cfg=yaml.safe_load((Path(__file__).parents[1]/'configs/playground1-cbs.yaml').read_text())
    cfg['odometry']['start_offsets_s']['Bob']=offset
    path=tmp_path/'config.yaml';path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='finite nonnegative'):
        cli.run(SimpleNamespace(config=path))

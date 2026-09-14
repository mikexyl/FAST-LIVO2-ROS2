"""Sequence boundaries and unavailable GT must not produce spurious accuracy."""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s3e_pipeline.artifacts import write_json
from s3e_pipeline.cli import run
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth, comparison_plot
from s3e_pipeline.evaluation import trajectory_metrics


def test_unmatched_and_missing_gt_still_plot_disconnected_trajectories(tmp_path):
    # Laboratory 1 supplies endpoint IDs 0/1, not sensor epoch timestamps.
    (tmp_path/'alpha_gt.txt').write_text('0 1 2 3 0 0 0 1\n1 2 3 4 0 0 0 1\n')
    robots = ['Alpha', 'Bob']
    rows = [dict(robot_id=r, component=r, stamp_ns=1662375655*10**9+k*10**9,
                 T_world_body=np.eye(4).tolist()) for r in robots for k in range(4)]
    gt, status = evaluation_ground_truth(robots, tmp_path, rows, {'gt_max_gap_s': 2})
    assert status['Alpha']['reason'] == 'no_timestamp_overlap_with_sensor_trajectory'
    assert status['Bob']['reason'] == 'missing_or_invalid_position_ground_truth'
    assert not any(v['available'] for v in status.values())
    metrics = trajectory_metrics({'poses': rows}, gt, {'gt_max_gap_s': 2})
    assert metrics == {}
    report = dict(trajectory=metrics, centralized_reference=None, components={r:r for r in robots}, ground_truth=status)
    comparison_plot(dict(robots=robots, dataset='S3E_Laboratory_1'), rows, [], gt, report, tmp_path)
    assert (tmp_path/'trajectories.png').stat().st_size > 1000
    values = list(csv.DictReader((tmp_path/'metrics.csv').open()))
    assert len(values) == 2 and all(v['position_rmse_m'] == '' and v['samples'] == '0' for v in values)


def test_timestamp_matched_gt_keeps_shared_rigid_metric(tmp_path):
    epoch = 1662375655
    xyz = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,1]], dtype=float)
    (tmp_path/'alpha_gt.txt').write_text('\n'.join(f'{epoch+k} {p[0]} {p[1]} {p[2]} 0 0 0 1' for k,p in enumerate(xyz)))
    rows = []
    for k, p in enumerate(xyz):
        T = np.eye(4); T[:3,3] = p + [10,20,30]
        rows.append(dict(robot_id='Alpha', component='Alpha', stamp_ns=(epoch+k)*10**9, T_world_body=T.tolist()))
    gt, status = evaluation_ground_truth(['Alpha'], tmp_path, rows, {'gt_max_gap_s':2})
    assert status['Alpha']['available'] and status['Alpha']['matched_samples'] == 4
    metrics = trajectory_metrics({'poses':rows}, gt, {'gt_max_gap_s':2})['Alpha']
    assert metrics['rmse_m'] < 1e-12 and metrics['scale'] == 1


def test_evo_shared_alignment_and_native_result_archives(tmp_path):
    from evo.core import metrics as evo_metrics
    from evo.main_ape import ape
    from evo.tools.file_interface import load_res_file, read_kitti_poses_file
    from s3e_pipeline.evo_evaluation import evaluate
    epoch = 1662375655
    xyz = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,1]], dtype=float)
    rows = []; truth = {}
    for robot in ['Alpha', 'Bob']:
        stamps = (epoch+np.arange(4))*10**9
        truth[robot] = (stamps, xyz)
        for k, p in enumerate(xyz):
            T = np.eye(4); T[:3,3] = p + [10+(robot == 'Bob')*2,20,30]
            rows.append(dict(robot_id=robot, component='Alpha', stamp_ns=int(stamps[k]+10_000_000), T_world_body=T.tolist()))
    result = evaluate({'poses':rows}, truth, {'evo_max_diff_s':.05}, tmp_path)['Alpha']
    # Separate robot fits would incorrectly erase this two-metre map misalignment.
    assert result['rmse_m'] == pytest.approx(1.)
    assert result['samples'] == 8
    saved = load_res_file(tmp_path/'Alpha-component-ape.zip')
    direct = ape(read_kitti_poses_file(tmp_path/'Alpha-reference.kitti'),
                 read_kitti_poses_file(tmp_path/'Alpha-estimate.kitti'),
                 evo_metrics.PoseRelation.translation_part, align=True, correct_scale=False)
    assert saved.stats == direct.stats
    assert np.allclose(saved.np_arrays['error_array'], direct.np_arrays['error_array'])
    assert load_res_file(tmp_path/'Bob-ape.zip').stats['rmse'] == pytest.approx(1.)
    assert evaluate({'poses':rows}, truth, {'evo_max_diff_s':.001}) == {}


def test_evo_degenerate_alignment_reported_as_unavailable(tmp_path):
    from s3e_pipeline.evo_evaluation import evaluate
    rows = [dict(robot_id='Alpha', component='Alpha', stamp_ns=k*10**9,
                 T_world_body=np.eye(4).tolist()) for k in range(4)]
    truth = {'Alpha': (np.arange(4)*10**9, np.zeros((4,3)))}
    assert evaluate({'poses':rows}, truth, {}, tmp_path) == {}
    import json
    status = json.loads((tmp_path/'evaluation.json').read_text())
    assert 'alignment failed' in status['components']['Alpha']['reason']


@pytest.mark.parametrize('config_name,sequence', [
    ('square2-cbs.yaml', 'S3E_Square_2'),
    ('library1-cbs.yaml', 'S3E_Library_1'),
    ('playground1-cbs.yaml', 'S3E_Playground_1'),
    ('playground2-cbs.yaml', 'S3E_Playground_2'),
    ('laboratory1-cbs.yaml', 'S3E_Laboratory_1'),
    ('campus-road1-cbs.yaml', 'S3E_Campus_Road_1'),
])
def test_different_sequence_registry_is_rejected(tmp_path, config_name, sequence):
    configs = Path(__file__).resolve().parents[1]/'configs'
    cfg = yaml.safe_load((configs/config_name).read_text())
    square = yaml.safe_load((configs/'square1-cbs.yaml').read_text())
    if sequence == 'S3E_Playground_1':
        square['odometry']['start_offsets_s'] = {'Bob': 22.0}
        # Excluded historical sequence retains its pre-PCM configuration.
        square['dpgo'].pop('pcm')
    if sequence in ('S3E_Playground_2', 'S3E_Square_2', 'S3E_Library_1'):
        square['odometry']['mapping_overrides'] = {r:{'vio.img_point_cov':100} for r in cfg['robots']}
    assert {k:v for k,v in cfg.items() if k not in ('dataset','output_root')} == {k:v for k,v in square.items() if k not in ('dataset','output_root')}
    dataset = tmp_path/sequence; dataset.mkdir(); (dataset/'metadata.yaml').write_text('{}')
    cfg.update(dataset=str(dataset), output_root=str(tmp_path/'output'))
    config = tmp_path/'config.yaml'; config.write_text(yaml.safe_dump(cfg))
    registry = tmp_path/'run.json'; write_json(registry, dict(config=square, artifacts={}))
    with pytest.raises(ValueError, match='different S3E sequence'):
        run(SimpleNamespace(config=config, input_run=registry, stage=['dpgo'], resume=True))

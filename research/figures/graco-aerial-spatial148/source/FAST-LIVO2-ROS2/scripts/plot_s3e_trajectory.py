#!/usr/bin/env python3
"""Evaluate one saved S3E trajectory with evo translation APE."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evo.tools import file_interface

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'research'))
from s3e_pipeline.evaluation import ground_truth
from s3e_pipeline.evo_evaluation import evaluate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--ground-truth', type=Path, default=Path('/data/s3e/S3E_Square_1/alpha_gt.txt'))
    parser.add_argument('--max-diff', type=float, default=.05, help='evo timestamp association tolerance in seconds')
    args = parser.parse_args()
    estimate = file_interface.read_tum_trajectory_file(args.run_directory/'trajectory.tum')
    rows = [dict(robot_id='robot', component='robot', stamp_ns=round(t*1e9), T_world_body=T.tolist())
            for t, T in zip(estimate.timestamps, estimate.poses_se3)]
    output = args.run_directory/'evo'
    metrics = evaluate({'poses':rows}, {'robot':ground_truth(args.ground_truth)},
                       {'evo_max_diff_s':args.max_diff}, output)
    report = dict(ground_truth=str(args.ground_truth), engine='evo 1.36.5', metrics=metrics,
                  status='available' if metrics else 'unavailable: see evo/evaluation.json',
                  note='Translation APE only; no scale fitting; supplied GT orientations and antenna lever arm unused.')
    (args.run_directory/'gnss_position_check.json').write_text(json.dumps(report, indent=2)+'\n')
    fig, axes = plt.subplots(1, 2, figsize=(11,5), layout='constrained')
    if metrics:
        saved = file_interface.load_res_file(output/'robot-ape.zip', load_trajectories=True)
        for name, trajectory in saved.trajectories.items():
            xyz = trajectory.positions_xyz
            axes[0].plot(xyz[:,0], xyz[:,1], label=name)
        axes[1].plot(saved.np_arrays['seconds_from_start'], saved.np_arrays['error_array'])
        axes[1].set_title(f'evo translation APE RMSE {saved.stats["rmse"]:.3f} m')
    else:
        xyz = estimate.positions_xyz
        axes[0].plot(xyz[:,0], xyz[:,1], label='FAST-LIVO2 (local frame)')
        axes[1].text(.5, .5, 'APE unavailable: no usable matched ground truth', ha='center', transform=axes[1].transAxes)
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title=args.ground_truth.parent.name)
    axes[0].set_aspect('equal'); axes[0].legend(); axes[0].grid(alpha=.2)
    axes[1].set(xlabel='Time from first matched pose (s)', ylabel='Translation APE (m)')
    axes[1].grid(alpha=.2)
    fig.savefig(args.run_directory/'trajectory_check.png', dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

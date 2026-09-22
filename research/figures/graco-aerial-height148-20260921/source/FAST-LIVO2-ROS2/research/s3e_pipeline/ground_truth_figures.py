"""Render released timestamped multi-robot position GT in one common frame."""
import argparse
from pathlib import Path

import numpy as np
from evo.tools.file_interface import read_tum_trajectory_file

from .artifacts import file_hash, write_json
from .report_figures import COLORS, ROBOTS, configure, plt, Line2D, save


def render(dataset, output, gap_s=2., dpi=350):
    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    if dataset.name not in ('S3E_Square_1', 'S3E_Square_2', 'S3E_Playground_1', 'S3E_Playground_2', 'S3E_Library_1', 'S3E_Campus_Road_1'):
        raise ValueError('Expected a supported sequence with timestamped position GT')
    tracks = {r: read_tum_trajectory_file(dataset/f'{r.lower()}_gt.txt') for r in ROBOTS}
    for robot, track in tracks.items():
        if (track.num_poses < 3 or not np.isfinite(track.positions_xyz).all()
                or not np.isfinite(track.timestamps).all() or np.any(np.diff(track.timestamps) <= 0)):
            raise ValueError(f'{robot}: invalid timestamped position trajectory')
    origin = tracks['Alpha'].positions_xyz[0].copy()
    configure()
    fig, ax = plt.subplots(figsize=(8.2, 5.8))
    inputs = {}
    for robot, track in tracks.items():
        gaps = np.flatnonzero(np.diff(track.timestamps) > gap_s) + 1
        xyz = track.positions_xyz - origin
        path = np.insert(xyz, gaps, np.nan, axis=0)
        ax.plot(path[:, 0], path[:, 1], color=COLORS[robot], lw=1.4, label=robot)
        ax.scatter(*xyz[0, :2], c=COLORS[robot], s=38, marker='o',
                   edgecolors='white', linewidths=.7, zorder=4)
        ax.scatter(*xyz[-1, :2], c=COLORS[robot], s=42, marker='X',
                   edgecolors='white', linewidths=.7, zorder=4)
        source = dataset/f'{robot.lower()}_gt.txt'
        inputs[robot] = dict(path=str(source), sha256=file_hash(source), samples=track.num_poses,
                             first_stamp_s=float(track.timestamps[0]), last_stamp_s=float(track.timestamps[-1]),
                             gaps_over_threshold=len(gaps))
    handles, labels = ax.get_legend_handles_labels()
    handles += [Line2D([], [], color='#50565b', marker=marker, ls='', label=label)
                for marker, label in [('o', 'First GT sample'), ('X', 'Last GT sample')]]
    ax.legend(handles=handles, ncol=3, frameon=False, loc='upper left', bbox_to_anchor=(0., -.13))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    ax.set(xlabel="East from Alpha's first GT position (m)",
           ylabel="North from Alpha's first GT position (m)")
    ax.set_title(dataset.name.replace('_', ' ') + ' — multi-robot position ground truth', loc='left', pad=12)
    fig.text(.5, .03, f'One shared origin; no rotation or scale fitting. GT gaps greater than {gap_s:g} s remain unconnected.\n'
             'Markers denote the available GT endpoints, which may not cover the full sensor recording.',
             ha='center', fontsize=8, color='#50565b')
    fig.subplots_adjust(left=.11, right=.97, bottom=.25, top=.91)
    output.mkdir(parents=True, exist_ok=True)
    save(fig, output, 'ground_truth_trajectories', dpi)
    write_json(output/'ground_truth.json', dict(schema_version=1, dataset=str(dataset), inputs=inputs,
        coordinate_origin_world_m=origin.tolist(), projection='XY', gap_threshold_s=gap_s,
        alignment='shared translation only, subtract first Alpha GT position', scale=1,
        parser='evo.tools.file_interface.read_tum_trajectory_file', orientation_ground_truth_used=False,
        generator_sha256=file_hash(Path(__file__)), dpi=dpi,
        files={f'ground_truth_trajectories.{ext}': file_hash(output/f'ground_truth_trajectories.{ext}')
               for ext in ('png', 'pdf')}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gap-s', type=float, default=2.)
    parser.add_argument('--dpi', type=int, default=350)
    args = parser.parse_args()
    if args.gap_s <= 0 or args.dpi <= 0:
        parser.error('gap-s and dpi must be positive')
    render(args.dataset, args.output, args.gap_s, args.dpi)


if __name__ == '__main__':
    main()

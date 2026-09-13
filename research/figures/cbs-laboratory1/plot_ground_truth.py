"""Plot the released Laboratory 1 GT endpoints, without inventing trajectories."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 's3e-report-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('/data/s3e/S3E_Laboratory_1'))
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'ground_truth')
    args = parser.parse_args()
    if args.dataset.name != 'S3E_Laboratory_1':
        parser.error('This figure describes the endpoint-only S3E_Laboratory_1 release.')
    palette = {'Alpha': '#dc514b', 'Bob': '#2684bc', 'Carol': '#269b69'}
    records, inputs = [], []
    for robot in palette:
        source = args.dataset / f'{robot.lower()}_gt.txt'
        # Read only the supplied record ID and position. Alpha's orientation
        # columns contain a missing separator; orientation is neither used nor repaired.
        fields = [line.split() for line in source.read_text().splitlines() if line.strip()]
        if [row[0] for row in fields] != ['0', '1']:
            raise ValueError(f'{source}: expected start/end record IDs 0 and 1')
        for row in fields:
            records.append(dict(robot=robot, endpoint='start' if row[0] == '0' else 'end',
                                record_id=int(row[0]), x=float(row[1]), y=float(row[2]), z=float(row[3])))
        inputs.append(dict(path=str(source.resolve()), sha256=sha256(source), records=len(fields)))
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'pdf.fonttype': 42, 'savefig.facecolor': 'white'})
    fig, ax = plt.subplots(figsize=(9.1, 4.4))
    for row in records:
        start = row['endpoint'] == 'start'
        ax.scatter(row['y'], row['z'], c=palette[row['robot']], s=65,
                   marker='o' if start else 'X', edgecolors='white', linewidths=.8, zorder=3)
        ax.annotate(f"{row['robot']} {row['endpoint']}", (row['y'], row['z']),
                    xytext=(9, 0), textcoords='offset points', va='center', fontsize=9)
    ax.set(xlabel='Y (source units)', ylabel='Z (source units)',
           xlim=(-2320, 1530), ylim=(-1060, 290))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=.22, linewidth=.5)
    ax.set_title('S3E Laboratory 1 — ground-truth start/end positions', loc='left', pad=15)
    handles = [Line2D([], [], color=color, marker='s', ls='', label=robot)
               for robot, color in palette.items()]
    handles += [Line2D([], [], color='#444444', marker=marker, ls='', label=label)
                for marker, label in [('o', 'Start (record 0)'), ('X', 'End (record 1)')]]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .087),
               ncol=5, frameon=False, fontsize=9)
    fig.text(.5, .033, 'Y–Z projection in the source frame; no coordinate conversion or estimator alignment.\n'
             'Only the six released endpoint positions are shown. Intermediate ground-truth paths are unavailable.',
             ha='center', fontsize=8.5, color='#51565c')
    fig.subplots_adjust(left=.09, right=.985, top=.88, bottom=.24)
    artifacts = []
    for extension in ('png', 'pdf'):
        path = args.output / f'ground_truth_endpoints.{extension}'
        fig.savefig(path, dpi=350, bbox_inches='tight', pad_inches=.08)
        artifacts.append(dict(path=path.name, sha256=sha256(path)))
    plt.close(fig)
    table = args.output / 'positions.csv'
    with table.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    artifacts.append(dict(path=table.name, sha256=sha256(table)))
    metadata = dict(schema_version=1, dataset='S3E_Laboratory_1',
                    ground_truth_type='start_end_positions_only',
                    source='https://huggingface.co/datasets/PengYu-Team/S3E/blob/main/README.md#known-issues',
                    inputs=inputs, positions=records, projection=['y', 'z'],
                    units='source units; no unit conversion assumed',
                    alignment='none', timestamp_association='not applicable to endpoint IDs',
                    orientation='ignored', connected_paths=False, dpi=350,
                    script_sha256=sha256(Path(__file__)), matplotlib_version=matplotlib.__version__,
                    artifacts=artifacts)
    (args.output / 'figure.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Saved 6 ground-truth endpoint positions to {args.output}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Summarize frozen frontend retries without modifying their verdicts."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def report(evidence, output):
    state = json.loads((evidence / 'status.json').read_text())
    if not state.get('finished_utc'):
        raise ValueError('Wait for the complete serial queue')
    cases = {r['name']: r for r in json.loads((evidence / 'cases.json').read_text())}
    output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(17, 10), layout='constrained')
    summary = []
    for ax, row in zip(axes.flat, state['cases']):
        name = row['name']; case = cases[name]; trial = evidence / name
        destination = output / name
        evaluation = trial / 'evaluation-graco-reference' if (trial / 'evaluation-graco-reference').exists() else trial / 'evaluation'
        if evaluation.exists():
            shutil.copytree(evaluation, destination, dirs_exist_ok=True)
        for filename in ('artifact-validation.json', 'sensor-coverage.json', 'recording-verification.log', 'source-hashes.json'):
            if (trial / filename).exists():
                destination.mkdir(exist_ok=True)
                shutil.copy2(trial / filename, destination / filename)
        metrics_path = destination / 'metrics.json'
        metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
        metrics.pop('handovers', None)
        frontend_ok = all(metrics.get(k, False) for k in ('full_completion', 'motion_pass', 'update_continuity_pass'))
        rmse = metrics.get('ate_rmse_m')
        summary.append(dict(name=name, robot=case['robot'], frontend_success=frontend_ok,
                            accuracy_available=rmse is not None, metrics=metrics,
                            validation_exit=row.get('validation_exit'),
                            recording_verification_exit=row.get('recording_verification_exit')))
        aligned = destination / f"evo/{case['robot']}-estimate-aligned.tum"
        if aligned.exists():
            estimate = np.loadtxt(aligned)
            reference = np.loadtxt(destination / f"evo/{case['robot']}-reference-matched.tum")
            origin = reference[0, 1:3]
            ax.plot(reference[:, 1]-origin[0], reference[:, 2]-origin[1], 'k--', lw=1, label='Position GT')
            ax.plot(estimate[:, 1]-origin[0], estimate[:, 2]-origin[1], lw=1, label='Submap EllipseLIO')
            detail = f'ATE {rmse:.3f} m'
        elif (destination / 'native-trajectory.tum').exists():
            estimate = np.loadtxt(destination / 'native-trajectory.tum')
            ax.plot(estimate[:, 1], estimate[:, 2], label='Native trajectory')
            detail = 'ATE unavailable; native frame'
        else:
            detail = 'No evaluable trajectory'
        ax.set_title(f"{name}\n{detail}", fontsize=10)
        ax.axis('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(alpha=.2)
        if ax.lines: ax.legend(fontsize=8)
    for ax in list(axes.flat)[len(summary):]: ax.set_visible(False)
    fig.savefig(output / 'trajectories.png', dpi=160); plt.close(fig)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    for filename in ('cases.json', 'status.json'):
        shutil.copy2(evidence / filename, output / filename)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(report(args.evidence, args.output), indent=2))

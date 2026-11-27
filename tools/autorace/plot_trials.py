"""Compare error, command and motion using outputs from run_trial.py."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('runs', type=Path, nargs='+')
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    error, angular, trajectory, heading = axes.ravel()
    stats = {}
    for run in args.runs:
        with (run / 'trace.csv').open() as source:
            data = np.array([list(map(float, row.values())) for row in csv.DictReader(source)])
        if data.ndim != 2 or not len(data):
            raise ValueError(f'No samples in {run}')
        t, e, w, v, x, y, yaw = data.T
        valid = np.isfinite(e)
        absolute = np.abs(e[valid])
        if not len(absolute):
            raise ValueError(f'No tracked frames in {run}')
        label = run.name
        error.plot(t, e, label=label)
        angular.plot(t, w, label=label)
        trajectory.plot(x, y, label=label)
        heading.plot(t, np.degrees(np.unwrap(yaw) - yaw[0]), label=label)
        stats[label] = dict(duration_s=float(t[-1]), samples=len(data),
                            mean_absolute_error=float(np.mean(absolute)),
                            max_absolute_error=float(np.max(absolute)),
                            stop_samples=int(np.count_nonzero(v == 0)),
                            distance_m=float(np.sum(np.hypot(np.diff(x), np.diff(y)))),
                            initial_pose=[float(x[0]), float(y[0]), float(yaw[0])],
                            end_pose=[float(x[-1]), float(y[-1]), float(yaw[-1])])
    error.set(title='Lane center error', xlabel='Simulation time (s)', ylabel='Half-frame units')
    angular.set(title='Steering command', xlabel='Simulation time (s)', ylabel='rad/s')
    trajectory.set(title='Odometry', xlabel='x (m)', ylabel='y (m)')
    trajectory.set_aspect('equal', adjustable='datalim')
    heading.set(title='Heading change', xlabel='Simulation time (s)', ylabel='degrees')
    for axis in axes.ravel():
        axis.grid(alpha=0.25)
        axis.legend()
    fig.savefig(args.output, dpi=135)
    print(json.dumps(stats, indent=2))


if __name__ == '__main__':
    main()

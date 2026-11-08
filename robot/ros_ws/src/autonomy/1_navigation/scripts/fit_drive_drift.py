#!/usr/bin/env python3
"""Fit straight/omni zero-turn yaw coupling from a recorded CSV; no ROS/hardware."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def fit_recording(path, gain=0.75):
    path = Path(path)
    rows = list(csv.DictReader(path.open()))
    keys = ('t', 'cmd_age', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'odom_vx', 'odom_vy', 'odom_wz')
    a = np.array([[float(row[key]) for key in keys] for row in rows])
    if len(a) < 200:
        raise ValueError('need at least 200 samples')
    valid = np.isfinite(a).all(axis=1)
    remove_applied_rotation = bool(rows and 'applied_wz' in rows[0])
    if remove_applied_rotation:
        applied = np.array([[float(row[k]) for k in ('applied_age', 'applied_wz')] for row in rows])
        valid &= (np.isfinite(applied).all(axis=1) & (applied[:, 0] >= 0) & (applied[:, 0] <= .2))
        # A verification recording may already contain compensation. Estimate the
        # intrinsic bias rather than fitting the smaller residual and forgetting it.
        a[:, 7] -= applied[:, 1]
    a = a[valid]
    sources = np.array([row['source'] for row, keep in zip(rows, valid) if keep])
    dt = np.diff(a[:, 0])
    if len(a) < 200 or np.any(dt <= 0):
        raise ValueError('insufficient samples or nonmonotonic timestamps')
    acceleration = np.linalg.norm(np.diff(a[:, 2:4], axis=0), axis=1) / dt
    acceleration = np.r_[acceleration[0], np.convolve(acceleration, np.ones(15)/15, mode='same')]
    speed = np.linalg.norm(a[:, 2:4], axis=1)
    keep = ((sources == 'teleop') & (a[:, 1] >= 0) & (a[:, 1] <= 0.2) &
            (np.abs(a[:, 4]) <= 0.005) & (speed >= 0.10) & (speed <= 0.32) &
            (np.linalg.norm(a[:, 5:7], axis=1) >= 0.06) & (acceleration < 0.25))
    a = a[keep]
    x, y = a[:, 2:4], a[:, 7]
    if len(a) < 200 or np.linalg.matrix_rank(x) < 2 or np.linalg.cond(x) > 10:
        raise ValueError('need independent forward and sideways data')
    coefficients = np.linalg.lstsq(x, y, rcond=None)[0]
    rms = lambda v: float(np.sqrt(np.mean(v*v)))
    validations = []
    splits = [a[:, 0] < a[0, 0] + .6 * (a[-1, 0] - a[0, 0]),
              (np.floor(a[:, 0] / 2).astype(int) % 2) == 0]
    for name, train in zip(('first_60_percent', 'alternating_2_second_blocks'), splits):
        if min(sum(train), sum(~train)) < 100 or np.linalg.matrix_rank(x[train]) < 2:
            raise ValueError('insufficient held-out data')
        c = np.linalg.lstsq(x[train], y[train], rcond=None)[0]
        baseline = rms(y[~train])
        residual = rms(y[~train] - gain * (x[~train] @ c))
        improvement = 1 - residual / baseline if baseline else 0
        validations.append(dict(split=name, baseline_rms_rad_s=baseline,
                                predicted_residual_rms_rad_s=residual, rms_reduction=improvement))
        if improvement < 0.20:
            raise ValueError(f'{name}: less than 20% held-out RMS improvement')
    return dict(source=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                total_samples=len(rows), selected_samples=len(a),
                removed_applied_rotation=remove_applied_rotation,
                model='measured_wz ~= vx_gain * commanded_vx + vy_gain * commanded_vy',
                vx_gain_rad_m=float(coefficients[0]), vy_gain_rad_m=float(coefficients[1]),
                compensation_gain=gain, max_correction_rad_s=0.07,
                model_speed_limit_m_s=0.30, active_speed_range_m_s=[0.10, 0.45],
                baseline_rms_rad_s=rms(y),
                predicted_residual_rms_rad_s=rms(y - gain * (x @ coefficients)),
                held_out=validations,
                limits='Retrospective prediction assuming unit angular response; not a physical '
                       'verification of compensation or a replacement for module calibration.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.csv
    if source is None:
        recordings = list((Path(__file__).resolve().parents[1] / 'recordings').glob('drive_drift_*.csv'))
        if not recordings:
            parser.error('no drive drift recordings')
        source = max(recordings, key=lambda p: p.stat().st_mtime_ns)
    report = fit_recording(source)
    output = args.output or source.with_name(source.stem + '_fit.json')
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

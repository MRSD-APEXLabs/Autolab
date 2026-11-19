#!/usr/bin/env python3
"""Fit straight/omni zero-turn yaw coupling from a recorded CSV; no ROS/hardware."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

# The drift builds for ~1.5 s after a start from rest (recordings/drive_drift_20261119_004551);
# the steady part is the fitted model, station feedback handles the build-up.
SETTLE_S = 1.2
FIT_SPEED = (0.10, 0.45)  # covers the 0.40 m/s back-out and 0.36-0.40 m/s docking-start legs


def heading_rate(t, yaw, half_window=0.15):
    """Gyro yaw rate: local slope of the unwrapped /odom pose heading (Pigeon 2).

    Not the /odom twist: Phoenix computes that from wheel kinematics, and the wheels
    scrub, so it showed only ~1/4 of the real drift and the earlier fit was too small.
    """
    heading = np.unwrap(yaw)
    lo = np.searchsorted(t, t - half_window, side='left')
    hi = np.searchsorted(t, t + half_window, side='right')
    rate = np.full(len(t), np.nan)
    for i, (start, end) in enumerate(zip(lo, hi)):
        if end - start >= 5:
            rate[i] = np.polyfit(t[start:end] - t[i], heading[start:end], 1)[0]
    return rate


def time_moving(t, speed, sources):
    """Seconds since the current commanded teleop motion started from rest; -1 when not moving."""
    out, start = np.full(len(t), -1.0), None
    for i in range(len(t)):
        if sources[i] != 'teleop' or speed[i] < 0.08:
            start = None
            continue
        if start is None or t[i] - t[i-1] > 0.25:
            start = t[i]
        out[i] = t[i] - start
    return out


def fit_recording(path, gain=0.75):
    path = Path(path)
    rows = list(csv.DictReader(path.open()))
    keys = ('t', 'cmd_age', 'cmd_vx', 'cmd_vy', 'cmd_wz', 'odom_vx', 'odom_vy', 'yaw')
    a = np.array([[float(row[key]) for key in keys] for row in rows])
    if len(a) < 200:
        raise ValueError('need at least 200 samples')
    valid = np.isfinite(a).all(axis=1)
    applied_wz = np.zeros(len(a))
    remove_applied_rotation = bool(rows and 'applied_wz' in rows[0])
    if remove_applied_rotation:
        applied = np.array([[float(row[k]) for k in ('applied_age', 'applied_wz')] for row in rows])
        valid &= (np.isfinite(applied).all(axis=1) & (applied[:, 0] >= 0) & (applied[:, 0] <= .2))
        applied_wz = applied[:, 1]
    a, applied_wz = a[valid], applied_wz[valid]
    sources = np.array([row['source'] for row, keep in zip(rows, valid) if keep])
    dt = np.diff(a[:, 0])
    if len(a) < 200 or np.any(dt <= 0):
        raise ValueError('insufficient samples or nonmonotonic timestamps')
    # A verification recording may already contain compensation. Estimate the
    # intrinsic bias rather than fitting the smaller residual and forgetting it.
    measured = heading_rate(a[:, 0], a[:, 7]) - applied_wz
    acceleration = np.linalg.norm(np.diff(a[:, 2:4], axis=0), axis=1) / dt
    acceleration = np.r_[acceleration[0], np.convolve(acceleration, np.ones(15)/15, mode='same')]
    speed = np.linalg.norm(a[:, 2:4], axis=1)
    keep = ((sources == 'teleop') & (a[:, 1] >= 0) & (a[:, 1] <= 0.2) &
            (np.abs(a[:, 4]) <= 0.005) & (speed >= FIT_SPEED[0]) & (speed <= FIT_SPEED[1]) &
            (np.linalg.norm(a[:, 5:7], axis=1) >= 0.06) & (acceleration < 0.25) &
            (time_moving(a[:, 0], speed, sources) >= SETTLE_S) & np.isfinite(measured))
    a = a[keep]
    x, y = a[:, 2:4], measured[keep]
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
                model='gyro_wz ~= vx_gain * commanded_vx + vy_gain * commanded_vy',
                measured_signal='gyro heading rate (slope of /odom pose yaw), not the wheel twist',
                settle_s=SETTLE_S, fit_speed_range_m_s=list(FIT_SPEED),
                vx_gain_rad_m=float(coefficients[0]), vy_gain_rad_m=float(coefficients[1]),
                compensation_gain=gain, max_correction_rad_s=0.15,
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

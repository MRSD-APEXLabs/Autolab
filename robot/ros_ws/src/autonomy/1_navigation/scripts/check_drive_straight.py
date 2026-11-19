#!/usr/bin/env python3
"""Record commanded versus measured yaw while the operator drives straight.

Subscribes only. Never starts the drivetrain, publishes commands, or changes gains.
The fit describes commands ENTERING the bridge versus Phoenix odometry. It cannot
separate bridge filtering, module calibration, wheel slip and gyro error.
"""

import argparse
import csv
import datetime
import json
import math
from pathlib import Path
import time

import numpy as np


FIELDS = ('t', 'source', 'cmd_age', 'cmd_vx', 'cmd_vy', 'cmd_wz',
          'x', 'y', 'yaw', 'odom_vx', 'odom_vy', 'odom_wz')
APPLIED_FIELDS = ('applied_age', 'applied_vx', 'applied_vy', 'applied_wz')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def analyse(rows, settle=0.75, min_duration=2.0):
    """Independent steady teleop passes, then an optional small offline bias fit."""
    groups, current = [], []
    for row in rows:
        finite = all(math.isfinite(row[k]) for k in FIELDS if k != 'source')
        valid = (finite and row['source'] == 'teleop' and 0 <= row['cmd_age'] <= 0.2 and
                 0.08 <= abs(row['cmd_vx']) <= 1.0 and abs(row['cmd_vy']) <= 0.04 and
                 abs(row['cmd_wz']) <= 0.005 and abs(row['odom_vx']) >= 0.03 and
                 row['cmd_vx'] * row['odom_vx'] > 0)
        same_pass = (current and row['t'] - current[-1]['t'] <= 0.25 and
                     row['t'] > current[-1]['t'] and
                     row['cmd_vx'] * current[0]['cmd_vx'] > 0 and
                     abs(row['cmd_vx'] - current[0]['cmd_vx']) <=
                     max(0.03, 0.15 * abs(current[0]['cmd_vx'])))
        if current and (not valid or not same_pass):
            groups.append(current)
            current = []
        if valid:
            current.append(row)
    if current:
        groups.append(current)
    passes = []
    for group in groups:
        group = [r for r in group if r['t'] >= group[0]['t'] + settle]
        if len(group) < 3 or group[-1]['t'] - group[0]['t'] < min_duration:
            continue
        t = np.array([r['t'] for r in group])
        t -= t[0]
        heading = np.unwrap([r['yaw'] for r in group])
        yaw_rate = float(np.polyfit(t, heading, 1)[0])
        distance = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                       for a, b in zip(group, group[1:]))
        if distance < 0.15:
            continue
        mean_vx = float(np.mean([r['cmd_vx'] for r in group]))
        dx, dy = group[-1]['x'] - group[0]['x'], group[-1]['y'] - group[0]['y']
        lateral = -dx * math.sin(group[0]['yaw']) + dy * math.cos(group[0]['yaw'])
        passes.append({
            'direction': 'forward' if mean_vx > 0 else 'reverse',
            'start_s': group[0]['t'], 'duration_s': float(t[-1]),
            'command_vx_m_s': mean_vx,
            'command_wz_rad_s': float(np.mean([r['cmd_wz'] for r in group])),
            'measured_vx_m_s': float(np.mean([r['odom_vx'] for r in group])),
            'measured_yaw_rate_rad_s': yaw_rate,
            'reported_odom_wz_rad_s': float(np.mean([r['odom_wz'] for r in group])),
            'heading_change_deg': math.degrees(float(heading[-1] - heading[0])),
            'distance_m': distance, 'lateral_displacement_m': lateral,
            'yaw_deg_per_m': math.degrees(float(heading[-1] - heading[0])) / distance,
        })
    fit = None
    if len(passes) >= 3:
        vx = np.array([p['command_vx_m_s'] for p in passes])
        wz = np.array([p['measured_yaw_rate_rad_s'] for p in passes])
        if float(np.ptp(vx)) >= 0.08:
            design = np.column_stack((np.ones(len(vx)), vx))
            intercept, slope = np.linalg.lstsq(design, wz, rcond=None)[0]
            residual = wz - design @ np.array([intercept, slope])
            fit = {'model': 'measured_wz ~= intercept + vx_gain * commanded_vx',
                   'intercept_rad_s': float(intercept), 'vx_gain_rad_m': float(slope),
                   'pass_residual_rms_rad_s': float(np.sqrt(np.mean(residual ** 2))),
                   'applied': False}
    return {'samples': len(rows), 'steady_passes': passes, 'candidate_bias_fit': fit,
            'limits': 'Odometry-based estimate, not external ground truth or module calibration. '
                      'The fit is diagnostic only; no compensation is applied.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--output', type=Path, help='Output prefix for .csv and .json files')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error('--duration must be between 1 and 600 seconds')
    prefix = args.output or (Path(__file__).resolve().parents[1] / 'recordings' /
                            ('drive_drift_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    prefix.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation keeps an earlier recording from being overwritten.
    csv_path, json_path = Path(str(prefix) + '.csv'), Path(str(prefix) + '.json')
    if csv_path.exists() or json_path.exists():
        parser.error(f'output already exists: {prefix}')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node('drive_straight_recorder', enable_rosout=False, start_parameter_services=False)
    rows = []
    latest = {'command': None, 'time': -math.inf, 'source': 'unknown',
              'applied': None, 'applied_time': -math.inf}
    started = time.monotonic()
    live = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def command(msg):
        latest['command'], latest['time'] = msg, time.monotonic()

    def applied(msg):
        latest['applied'], latest['applied_time'] = msg, time.monotonic()

    def odometry(msg):
        now = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        yaw = (math.atan2(2*(q.w*q.z + q.x*q.y), norm - 2*(q.y*q.y + q.z*q.z))
               if math.isfinite(norm) and abs(norm - 1) < 0.1 else math.nan)
        cmd = latest['command']
        twist = msg.twist.twist
        rows.append(dict(zip(FIELDS, (
            now - started, latest['source'], now - latest['time'],
            cmd.linear.x if cmd else math.nan, cmd.linear.y if cmd else math.nan,
            cmd.angular.z if cmd else math.nan, p.x, p.y, yaw,
            twist.linear.x, twist.linear.y, twist.angular.z))))
        actual = latest['applied']
        rows[-1].update(zip(APPLIED_FIELDS, (now - latest['applied_time'],
            actual.linear.x if actual else math.nan, actual.linear.y if actual else math.nan,
            actual.angular.z if actual else math.nan)))

    node.create_subscription(Twist, '/cmd_vel', command, live)
    node.create_subscription(Twist, '/swerve/applied_cmd_vel', applied, live)
    node.create_subscription(Odometry, '/odom', odometry, live)
    node.create_subscription(String, '/cmd_vel_mux/active_source',
                             lambda msg: latest.update(source=msg.data), latched)
    print(f'Recording for {args.duration:g} seconds. This tool sends NO drive commands.\n'
          'Use the pad in open space: steady straight forward/reverse passes at two speeds,\n'
          'at least 3 seconds each, rotation stick centered. Pause between passes.\n'
          f'Output: {prefix}.csv / .json', flush=True)
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    with csv_path.open('x', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS + APPLIED_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    report = analyse(rows)
    with json_path.open('x') as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
    print(f'Found {len(report["steady_passes"])} steady straight passes.')
    for p in report['steady_passes']:
        print(f'  {p["direction"]}: command {p["command_vx_m_s"]:+.2f} m/s, '
              f'yaw {math.degrees(p["measured_yaw_rate_rad_s"]):+.2f} deg/s, '
              f'{p["yaw_deg_per_m"]:+.2f} deg/m')
    if report['candidate_bias_fit'] is not None:
        print('Candidate yaw-bias fit saved for review; no compensation applied.')
    print(f'Report: {json_path}')
    return 0 if rows else 1


if __name__ == '__main__':
    raise SystemExit(main())

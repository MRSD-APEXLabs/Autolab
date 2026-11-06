#!/usr/bin/env python3
"""Bench checks for the wrist F/T sensor: is the plate's weight visible in Fz, and what does a lift do on its own?

The grasp hold check expects Fz to change by the plate's weight (0.60-0.69 N, about 61-70 g) over the lift. On
2026-11-05 it read 0.04 N with the plate in the grasp. These checks separate the possible causes:

  static  Arm still. Zero, read, load the gripper by hand (plate, or a known weight), read, unload, read.
          Loaded minus unloaded is what the sensor sees of the load without any motion. Compare with a scale.
  lift    Zero at the current pose, lift `--lift` m, lower back, `--repeats` times. With an empty gripper the
          after-lift change should be ~0; a shift here (cable pull, arm configuration) adds to or cancels the
          plate's weight in the real hold check. With a plate held (`--grip`), it is that hold check by itself.
  drift   Arm still. Zero, then one reading per second: how far the sensor wanders over a grasp's duration.
  gripper Interactive gripper control: o = open, a number = close to it (0..1000), r = read position + FSRs.

`--grip N` on static/lift opens the gripper, prompts you to place the plate, closes to N, and (static, or lift
with --open-after) prompts you before opening again, so you can hold the plate while it releases.

Both ft_ext_force (what the servo uses: the controller's tool-load compensation subtracted) and ft_raw_force are
shown, so a payload-compensation change shows up as a difference between the two.

  python3 tools/ft_check.py static [--ip 192.168.1.236] [--execute --grip 295]
  python3 tools/ft_check.py drift --seconds 30
  python3 tools/ft_check.py lift --execute [--grip 295 [--open-after]] [--lift 0.10] [--repeats 4]
  python3 tools/ft_check.py gripper --execute

`lift`, `gripper` and `--grip` move the arm or the gripper: they need --execute, and nothing else may command the arm
meanwhile (xarm_ros2 trajectories, teleop, a visual servo run). The arm is handed back in mode 1 at the end.
Needs the xArm SDK (xarm-python-sdk) and numpy, as the node does.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visual_servoing.arm import XArm  # noqa: E402

G = 9.81


def read_raw(arm):
    values = getattr(arm._require(), 'ft_raw_force', None)
    if values is None or len(values) < 6:
        return None
    values = [float(v) for v in values[:6]]
    return values if np.isfinite(values).all() else None


def sample(arm, seconds, interval):
    """Median and spread of ft_ext_force and ft_raw_force over `seconds`."""
    ext, raw = [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = arm.ft_force()
        if value is not None:
            ext.append(value)
        value = read_raw(arm)
        if value is not None:
            raw.append(value)
        time.sleep(interval)
    if not ext:
        raise RuntimeError('no valid ft_ext_force reads: is the sensor enabled and the report socket up?')
    ext = np.asarray(ext)
    raw = np.asarray(raw) if raw else None
    return {'ext': np.median(ext, axis=0), 'ext_std': ext.std(axis=0), 'n': len(ext),
            'raw': np.median(raw, axis=0) if raw is not None else None}


def show(label, s, ref=None):
    f, sd = s['ext'], s['ext_std']
    line = (f'{label:<14} F=[{f[0]:+.3f} {f[1]:+.3f} {f[2]:+.3f}] N (std {sd[0]:.3f} {sd[1]:.3f} {sd[2]:.3f}) '
            f'T=[{f[3]:+.4f} {f[4]:+.4f} {f[5]:+.4f}] N m  n={s["n"]}')
    if s['raw'] is not None:
        line += f'  raw Fz {s["raw"][2]:+.3f}'
    print(line)
    if ref is not None:
        d = f[:3] - ref['ext'][:3]
        print(f'{"":<14} change: dF=[{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] N  |dFz| {abs(d[2]):.3f} N '
              f'= {abs(d[2]) / G * 1000:.1f} g  |dF| {np.linalg.norm(d):.3f} N = {np.linalg.norm(d) / G * 1000:.1f} g')


def zero(arm):
    arm.ft_enable()
    arm.ft_zero()
    time.sleep(0.3)


def gripper_report(arm):
    print(f'gripper position {arm.gripper_position()}, fsr {arm.gripper_fsr()}')


def gripper_close(arm, args, prompt):
    """Close to --grip after `prompt` (None: no gripper control)."""
    if args.grip is None:
        return
    input(prompt)
    arm.gripper_set(args.grip, settle=args.grip_settle)
    gripper_report(arm)


def gripper_open(arm, args, prompt):
    if args.grip is None:
        return
    input(prompt)
    arm.gripper_open()
    time.sleep(args.grip_settle)
    gripper_report(arm)


def cmd_gripper(arm, args):
    """Interactive: o = open, a number = close to it (0..1000), r = read position and FSRs, q = quit."""
    arm.gripper_setup()
    gripper_report(arm)
    while True:
        cmd = input('gripper [o | 0..1000 | r | q]: ').strip().lower()
        if cmd in ('q', 'quit', 'exit'):
            return
        if cmd in ('o', 'open'):
            arm.gripper_open()
            time.sleep(args.grip_settle)
        elif cmd.isdigit():
            arm.gripper_set(int(cmd), settle=args.grip_settle)
        elif cmd not in ('r', ''):
            print('o to open, a number to close to, r to read, q to quit')
            continue
        gripper_report(arm)


def cmd_static(arm, args):
    if args.grip is not None:
        arm.gripper_setup()
        arm.gripper_open()
        time.sleep(args.grip_settle)
    zero(arm)
    unloaded = sample(arm, args.window, args.interval)
    show('unloaded', unloaded)
    if args.grip is None:
        input('Load the gripper (plate or known weight, hanging free, nothing else touching), then Enter... ')
    else:
        gripper_close(arm, args, f'Hold the plate between the open fingers, Enter to close to {args.grip}, '
                                 'then let go once it holds... ')
        input('Plate hanging free, nothing else touching? Enter to measure... ')
    time.sleep(args.settle)
    loaded = sample(arm, args.window, args.interval)
    show('loaded', loaded, unloaded)
    if args.grip is None:
        input('Remove the load, then Enter... ')
    else:
        gripper_open(arm, args, 'Hold the plate, Enter to open the gripper... ')
        input('Plate removed? Enter to measure... ')
    time.sleep(args.settle)
    after = sample(arm, args.window, args.interval)
    show('unloaded again', after, unloaded)
    print('loaded vs the mean of both unloaded reads:')
    mean = {'ext': (unloaded['ext'] + after['ext']) / 2}
    show('loaded', loaded, mean)


def cmd_drift(arm, args):
    zero(arm)
    first = None
    for i in range(int(args.seconds)):
        s = sample(arm, 1.0, args.interval)
        first = first or s
        show(f't={i + 1:>3d} s', s, first if i else None)


def cmd_lift(arm, args):
    if not args.execute:
        sys.exit('lift moves the arm: add --execute (and make sure nothing else commands it)')
    arm.position_mode()
    if args.grip is not None:
        arm.gripper_setup()
        arm.gripper_open()
        time.sleep(args.grip_settle)
        gripper_close(arm, args, f'Place the plate between the fingers (or leave empty), Enter to close to '
                                 f'{args.grip}... ')
    start = arm.flange_pose()
    zero(arm)
    print(f'zeroed at the start pose; lifting {args.lift * 1000:.0f} mm at {args.speed:.0f} mm/s, '
          f'{args.repeats} times')
    changes, returns = [], []
    try:
        for i in range(args.repeats):
            time.sleep(args.settle)
            before = sample(arm, args.window, args.interval)
            show(f'[{i + 1}] before', before)
            up = start.copy()
            up[2, 3] += args.lift
            arm.move_flange(up, args.speed)
            time.sleep(args.settle)
            lifted = sample(arm, args.window, args.interval)
            show(f'[{i + 1}] lifted', lifted, before)
            arm.move_flange(start, args.speed)
            time.sleep(args.settle)
            back = sample(arm, args.window, args.interval)
            show(f'[{i + 1}] back down', back, before)
            changes.append(lifted['ext'][2] - before['ext'][2])
            returns.append(back['ext'][2] - before['ext'][2])
    finally:
        arm.hand_back()
    if args.open_after:
        gripper_open(arm, args, 'Hold the plate, Enter to open the gripper... ')
    changes, returns = np.asarray(changes), np.asarray(returns)
    print(f'lift dFz: mean {changes.mean():+.3f} N (std {changes.std():.3f}) = {changes.mean() / G * 1000:+.1f} g; '
          f'back-down dFz: mean {returns.mean():+.3f} N (std {returns.std():.3f})')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('check', choices=('static', 'lift', 'drift', 'gripper'))
    parser.add_argument('--ip', default='192.168.1.236')
    parser.add_argument('--execute', action='store_true', help='allow arm and gripper motion')
    parser.add_argument('--window', type=float, default=2.0, help='s of reads per measurement')
    parser.add_argument('--interval', type=float, default=0.02, help='s between reads')
    parser.add_argument('--settle', type=float, default=1.5, help='s to wait after a move or a load change')
    parser.add_argument('--seconds', type=float, default=30.0, help='drift: how long')
    parser.add_argument('--lift', type=float, default=0.10, help='lift: m (the servo lifts 0.10)')
    parser.add_argument('--speed', type=float, default=50.0, help='lift: mm/s (the servo lifts at 50)')
    parser.add_argument('--repeats', type=int, default=4)
    parser.add_argument('--grip', type=int, default=None,
                        help='static/lift: open, then close the gripper on the plate to this (0..1000)')
    parser.add_argument('--grip-settle', type=float, default=4.0, help='s after a gripper command')
    parser.add_argument('--open-after', action='store_true', help='lift: open the gripper at the end (prompts)')
    args = parser.parse_args()
    if (args.grip is not None or args.check == 'gripper') and not args.execute:
        sys.exit('the gripper moves: add --execute')
    if args.open_after and args.grip is None:
        sys.exit('--open-after needs --grip')

    arm = XArm(args.ip, read_only=not args.execute)
    arm.connect()
    try:
        {'static': cmd_static, 'lift': cmd_lift, 'drift': cmd_drift, 'gripper': cmd_gripper}[args.check](arm, args)
    finally:
        arm.disconnect()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Bench tool for the two-finger tilt probe: jog the closed-finger probe onto the plate, see which finger touched
from the wrist torque, and run the tilt-and-retouch seek that levels the fingers with the plate.

Why tilt: the table leans towards the robot, so with the tool straight down one closed fingertip lands up to ~5 mm
before the other. The seek finds the tilt about the flange x axis (the axis across the finger line; the fingers close
along flange y) at which both fingertips touch together. The grasp then keeps that tilt.

How the seek works (no finger geometry needed):
  1. touch: read the in-air F/T baseline, descend at --speed until |Fz - baseline| > --contact-n for --contact-count
     reads, halt, then read the settled wrench in contact. tilt torque t = --torque-sign * dT[--torque-axis].
  2. |t| < --torque-deadband: both fingers touch, done.
  3. otherwise lift --lift mm, tilt by sign(t) * step about the fingertip centre (lowering the high finger), touch
     again. When t changes sign the step halves; below --min-step the seek ends between the two last tilts.
  4. --max-tilt bounds the total tilt from straight down.
The baseline is re-read in the air before every touch: tilting moves the gripper's weight onto the torque reading by
about as much as the contact itself.

Calibrating the torque sign first (`watch`, arm still, fingers closed, in the air): push one fingertip straight up by
hand and note the sign of the tilt torque; push the other. With the default --torque-sign 1 a press on the flange +y
finger should read t < 0 ("+y finger"). If the label is wrong, flip --torque-sign. `tilt +1` lowers the +y finger.

Commands (`help` lists them):
  s | status          tip position, tilt from straight down, gripper target, F/T now
  ft                  median F/T over --window s
  watch [s]           live Fz and tilt torque at 10 Hz for s seconds (default 10): torque sign calibration
  zero                re-zero the F/T sensor (only in the air, nothing touching)
  close [N] | open    gripper to N (default --probe-close) / open, waiting the travel time
  z <mm>              move the fingertip centre up (+) or down (-) in base z
  tilt <deg>          rotate about flange x through the fingertip centre (+ lowers the +y finger); lift first
  level               back to tool straight down at the current yaw, same fingertip position
  touch               one probe down from here (at most --probe-depth mm)
  seek                touch, then tilt and re-touch until balanced
  grasp [N]           after a touch/seek: lift --lift, open, re-zero, lower --grip-depth below the last touch along the
                      tilted tool axis, close to N (default --grip-close) and wait
  lift [mm]           straight up (default 100)
  q | quit            hand the arm back (mode 1) and exit

Everything that moves needs --execute, and nothing else may command the arm meanwhile (xarm_ros2, teleop, a servo
run). Without --execute only status/ft/watch work. Each touch is appended to --log (CSV) when given.

  python3 tools/probe_tilt_check.py --execute [--ip 192.168.1.236] [--log probe_tilt.csv] [--report-type real]

Needs numpy and the xArm SDK; PyYAML to read the fingertip from config/visual_servo.yaml (else the CAD fingertip).
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visual_servoing.arm import XArm, flange_for_tip  # noqa: E402
from visual_servoing.mount import Mount, matrix_rpy, rpy_matrix  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'visual_servo.yaml'


def load_mount(path):
    try:
        import yaml
    except ImportError:
        print('PyYAML missing: using the CAD fingertip (mount.py defaults)')
        return Mount()
    with open(path) as f:
        data = yaml.safe_load(f)
    params = dict(data['/**']['ros__parameters'].get('mount', {}))
    return Mount.from_params(params)


def axis_rotation(axis, angle):
    """Rodrigues: rotation by `angle` about unit `axis`."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


class Bench:
    def __init__(self, arm, mount, args):
        self.arm, self.mount, self.args = arm, mount, args
        self.flange_T_tip = mount.flange_T_tip
        self.last_touch = None    # dict of the last touch
        self.log = None
        if args.log:
            new = not Path(args.log).exists()
            self.log = open(args.log, 'a', newline='')
            self.writer = csv.writer(self.log)
            if new:
                self.writer.writerow(['time', 'label', 'tilt_deg', 'tip_x_mm', 'tip_y_mm', 'contact_z_mm', 'dfz_n',
                                      'dtx_nm', 'dty_nm', 'tilt_torque_nm', 'finger'])

    # -- geometry -------------------------------------------------------------------------------------
    def tip(self, pose=None):
        pose = self.arm.flange_pose() if pose is None else pose
        return (pose @ self.flange_T_tip)[:3, 3]

    @staticmethod
    def yaw(pose):
        return float(np.arctan2(pose[1, 0], pose[0, 0]))

    def tilt(self, pose=None):
        """(about flange x, about flange y) in degrees, relative to straight down at the current yaw."""
        pose = self.arm.flange_pose() if pose is None else pose
        level = rpy_matrix(np.pi, 0.0, self.yaw(pose))
        roll, pitch, _ = matrix_rpy(level.T @ pose[:3, :3])
        return float(np.degrees(roll)), float(np.degrees(pitch))

    def tilted(self, pose, degrees):
        """`pose` rotated by `degrees` about its own x axis through the fingertip centre."""
        r = axis_rotation(pose[:3, 0], np.radians(degrees))
        p = self.tip(pose)
        out = pose.copy()
        out[:3, :3] = r @ pose[:3, :3]
        out[:3, 3] = p + r @ (pose[:3, 3] - p)
        return out

    # -- sensing --------------------------------------------------------------------------------------
    def ft(self, seconds=None):
        seconds = self.args.window if seconds is None else seconds
        reads, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            value = self.arm.ft_force()
            if value is not None:
                reads.append(value)
            time.sleep(0.02)
        if not reads:
            raise RuntimeError('no F/T reads: sensor enabled?')
        return np.median(np.asarray(reads), axis=0)

    def tilt_torque(self, wrench_change):
        return self.args.torque_sign * float(wrench_change[3 + self.args.torque_axis])

    def finger(self, t):
        if abs(t) < self.args.torque_deadband:
            return 'both'
        return '+y finger' if t < 0 else '-y finger'

    # -- motion ---------------------------------------------------------------------------------------
    def need_execute(self):
        if not self.args.execute:
            raise RuntimeError('this moves the arm or gripper: restart with --execute')

    def move(self, pose, speed=None):
        self.need_execute()
        self.arm.move_flange(pose, self.args.move_speed if speed is None else speed)

    def move_z(self, mm, speed=None):
        pose = self.arm.flange_pose()
        pose[2, 3] += mm / 1000.0
        self.move(pose, speed)

    def gripper(self, value):
        self.need_execute()
        start = self.arm.gripper_position()
        travel = abs(value - start) if start is not None else 1000
        if not self.arm.gripper_set(value, settle=0.0):
            raise RuntimeError(f'gripper set {value} failed')
        wait = travel / self.args.gripper_rate + self.args.gripper_margin
        print(f'gripper {start} -> {value}: waiting {wait:.1f} s')
        time.sleep(wait)

    def zero(self):
        self.arm.ft_zero()
        time.sleep(0.3)

    # -- commands -------------------------------------------------------------------------------------
    def status(self):
        pose = self.arm.flange_pose()
        tx, ty = self.tilt(pose)
        tip = self.tip(pose) * 1000.0
        w = self.ft(0.3)
        print(f'tip [{tip[0]:.1f} {tip[1]:.1f} {tip[2]:.1f}] mm  tilt about flange x {tx:+.2f} deg '
              f'(about y {ty:+.2f})  yaw {np.degrees(self.yaw(pose)):.1f} deg  gripper {self.arm.gripper_position()}')
        self.show('F/T now', w)

    def show(self, label, w):
        print(f'{label:<10} F=[{w[0]:+.2f} {w[1]:+.2f} {w[2]:+.2f}] N  T=[{w[3]:+.4f} {w[4]:+.4f} {w[5]:+.4f}] N m')

    def watch(self, seconds):
        base = self.ft(0.5)
        print(f'baseline taken; tilt torque = {self.args.torque_sign:+d} * dT[{"xyz"[self.args.torque_axis]}]. '
              'Press one fingertip straight up...')
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            w = self.arm.ft_force()
            if w is not None:
                d = np.asarray(w) - base
                t = self.tilt_torque(d)
                print(f'  dFz {d[2]:+6.2f} N  tilt torque {t:+.4f} N m  dTy {d[4]:+.4f}  -> {self.finger(t)}')
            time.sleep(0.1)

    def touch(self, label='touch'):
        """Descend until contact. Returns the touch record, or None without contact within --probe-depth."""
        self.need_execute()
        a = self.args
        time.sleep(a.settle)
        base = self.ft()
        start = self.arm.flange_pose()
        target = start.copy()
        target[2, 3] -= a.probe_depth / 1000.0
        deadline = time.monotonic() + a.probe_depth / a.speed * 2.0 + 2.0
        self.arm.move_flange(target, a.speed, wait=False)
        first, count = None, 0
        try:
            while True:
                pose = self.arm.flange_pose()
                w = self.arm.ft_force()
                if w is None:
                    raise RuntimeError('F/T read failed during the touch')
                if abs(w[2] - base[2]) > a.contact_n:
                    first = first if count else pose
                    count += 1
                    if count >= max(1, a.contact_count):
                        self.arm.halt()
                        break
                else:
                    count = 0
                    if np.linalg.norm(pose[:3, 3] - target[:3, 3]) < 0.0005:
                        print(f'no contact within {a.probe_depth:.1f} mm')
                        return None
                if time.monotonic() > deadline:
                    raise RuntimeError('touch did not reach its target')
                time.sleep(0.02)
        except BaseException:
            self.arm.halt()
            raise
        time.sleep(a.settle)
        d = self.ft() - base
        t = self.tilt_torque(d)
        tip = self.tip(first)
        tx, _ = self.tilt(first)
        rec = {'pose': first, 'tilt': tx, 'tip': tip, 'dw': d, 't': t, 'finger': self.finger(t)}
        print(f'{label}: tilt {tx:+.2f} deg  contact z {tip[2] * 1000:.1f} mm  dFz {d[2]:+.2f} N  '
              f'tilt torque {t:+.4f} N m (dTx {d[3]:+.4f} dTy {d[4]:+.4f})  -> {rec["finger"]}')
        if self.log:
            self.writer.writerow([f'{time.time():.2f}', label, f'{tx:.3f}', f'{tip[0] * 1000:.1f}',
                                  f'{tip[1] * 1000:.1f}', f'{tip[2] * 1000:.2f}', f'{d[2]:.3f}', f'{d[3]:.5f}',
                                  f'{d[4]:.5f}', f'{t:.5f}', rec['finger']])
            self.log.flush()
        self.last_touch = rec
        return rec

    def seek(self):
        a = self.args
        rec = self.touch('seek 0')
        if rec is None:
            return
        step, prev, history = a.step, None, [rec]
        for i in range(1, a.max_iter + 1):
            t = rec['t']
            if abs(t) < a.torque_deadband:
                print(f'balanced at tilt {rec["tilt"]:+.2f} deg after {i - 1} retouches')
                return
            s = 1.0 if t > 0 else -1.0
            if prev is not None and s != prev:
                step /= 2.0
                if step < a.min_step:
                    print(f'torque flipped with step < {a.min_step} deg: both fingers touch between '
                          f'{history[-2]["tilt"]:+.2f} and {rec["tilt"]:+.2f} deg; using {rec["tilt"]:+.2f}')
                    return
            prev = s
            pose = self.arm.flange_pose()
            current, _ = self.tilt(pose)
            if abs(current + s * step) > a.max_tilt:
                print(f'cap: tilt {current + s * step:+.2f} deg exceeds --max-tilt {a.max_tilt}; stopping at '
                      f'{current:+.2f} deg ({rec["finger"]} still touches alone)')
                return
            self.move_z(a.lift, a.speed)
            self.move(self.tilted(self.arm.flange_pose(), s * step), a.tilt_speed)
            rec = self.touch(f'seek {i}')
            if rec is None:
                print('lost the plate during the seek')
                return
            history.append(rec)
        print(f'--max-iter {a.max_iter} reached at tilt {rec["tilt"]:+.2f} deg')

    def grasp(self, value):
        a = self.args
        if self.last_touch is None:
            raise RuntimeError('touch or seek first')
        contact_tip = self.last_touch['tip']
        self.move_z(a.lift, a.speed)
        self.gripper(0)
        self.zero()
        pose = self.arm.flange_pose()
        down = pose[:3, 2]                       # flange z: towards the fingers, along the tilted tool
        # place the fingertip centre grip_depth below the touched height along the tool axis
        tip_now = self.tip(pose)
        reach = (tip_now[2] - (contact_tip[2] - a.grip_depth / 1000.0)) / max(-down[2], 1e-6)
        target = pose.copy()
        target[:3, 3] += down * reach
        print(f'lowering {reach * 1000:.1f} mm along the tool axis to {a.grip_depth:.1f} mm below the touch')
        self.move(target, a.speed)
        self.gripper(value)
        self.show('closed', self.ft())

    def level(self):
        pose = self.arm.flange_pose()
        self.move(flange_for_tip(self.tip(pose), self.yaw(pose), self.flange_T_tip), self.args.tilt_speed)


HELP = __doc__[__doc__.index('Commands'):__doc__.index('Everything that moves')]


def repl(bench):
    a = bench.args
    while True:
        try:
            line = input('probe> ').strip().split()
        except EOFError:
            return
        if not line:
            continue
        cmd, rest = line[0].lower(), line[1:]
        num = (lambda default: float(rest[0]) if rest else default)
        try:
            if cmd in ('q', 'quit', 'exit'):
                return
            elif cmd in ('h', 'help', '?'):
                print(HELP)
            elif cmd in ('s', 'status'):
                bench.status()
            elif cmd == 'ft':
                bench.show('F/T', bench.ft())
            elif cmd == 'watch':
                bench.watch(num(10.0))
            elif cmd == 'zero':
                bench.zero()
                print('zeroed')
            elif cmd == 'close':
                bench.gripper(int(num(a.probe_close)))
            elif cmd == 'open':
                bench.gripper(0)
            elif cmd == 'z':
                mm = num(0.0)
                if abs(mm) > a.max_jog:
                    raise RuntimeError(f'|z| above --max-jog {a.max_jog} mm')
                bench.move_z(mm)
            elif cmd == 'tilt':
                deg = num(0.0)
                current, _ = bench.tilt()
                if abs(current + deg) > a.max_tilt:
                    raise RuntimeError(f'tilt would reach {current + deg:+.1f} deg, above --max-tilt {a.max_tilt}')
                bench.move(bench.tilted(bench.arm.flange_pose(), deg), a.tilt_speed)
                bench.status()
            elif cmd == 'level':
                bench.level()
                bench.status()
            elif cmd == 'touch':
                bench.touch()
            elif cmd == 'seek':
                bench.seek()
            elif cmd == 'grasp':
                bench.grasp(int(num(a.grip_close)))
            elif cmd == 'lift':
                bench.move_z(num(100.0), a.lift_speed)
            else:
                print(f'unknown command {cmd!r}; help lists them')
        except KeyboardInterrupt:
            print('interrupted: halting')
            if a.execute:
                bench.arm.halt()
        except Exception as exc:
            print(f'error: {exc}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ip', default='192.168.1.236')
    p.add_argument('--execute', action='store_true', help='allow arm and gripper motion')
    p.add_argument('--config', default=str(CONFIG), help='visual_servo.yaml for the fingertip (mount.*)')
    p.add_argument('--log', default=None, help='append every touch to this CSV')
    p.add_argument('--report-type', default=None, choices=('normal', 'rich', 'real'),
                   help='xArm SDK report socket the F/T is read from (default: SDK default). `real` should be '
                        'the fast one; check with `watch` that values change every line')
    # probe (servo defaults: grasp.probe_*, gripper.ft_contact_*)
    p.add_argument('--probe-close', type=int, default=550)
    p.add_argument('--probe-depth', type=float, default=15.0, help='mm a touch may descend')
    p.add_argument('--speed', type=float, default=8.0, help='mm/s for touches and lifts (final_speed)')
    p.add_argument('--contact-n', type=float, default=2.0, help='|dFz| N that counts as contact')
    p.add_argument('--contact-count', type=int, default=3)
    p.add_argument('--lift', type=float, default=2.0, help='mm up before each tilt step / before the grasp opens')
    # tilt seek
    p.add_argument('--torque-axis', type=int, default=0, choices=(0, 1, 2), help='0 Tx, 1 Ty, 2 Tz')
    p.add_argument('--torque-sign', type=int, default=1, choices=(-1, 1))
    p.add_argument('--torque-deadband', type=float, default=0.01, help='N m: |tilt torque| below = both touch')
    p.add_argument('--step', type=float, default=1.0, help='deg, first tilt step')
    p.add_argument('--min-step', type=float, default=0.25, help='deg; a sign flip below this ends the seek')
    p.add_argument('--max-tilt', type=float, default=15.0, help='deg from straight down')
    p.add_argument('--max-iter', type=int, default=15)
    p.add_argument('--tilt-speed', type=float, default=10.0, help='mm/s for tilt moves')
    # grasp
    p.add_argument('--grip-depth', type=float, default=7.5, help='mm below the touch (probe_grip_depth)')
    p.add_argument('--grip-close', type=int, default=350, help='gripper value for `grasp`')
    p.add_argument('--lift-speed', type=float, default=50.0)
    # misc
    p.add_argument('--move-speed', type=float, default=20.0, help='mm/s for z jogs')
    p.add_argument('--max-jog', type=float, default=50.0, help='mm per z jog')
    p.add_argument('--window', type=float, default=0.5, help='s of F/T reads per median')
    p.add_argument('--settle', type=float, default=0.3, help='s still before a baseline / contact read')
    p.add_argument('--gripper-rate', type=float, default=80.0, help='units/s (probe_gripper_rate)')
    p.add_argument('--gripper-margin', type=float, default=1.0, help='s (probe_gripper_margin)')
    args = p.parse_args()

    mount = load_mount(args.config)
    print(mount.describe())
    arm = XArm(args.ip, read_only=not args.execute, report_type=args.report_type)
    arm.connect()
    try:
        if args.execute:
            arm.position_mode()
            arm.gripper_setup()
        arm.ft_enable()
        print('Fingers closed? Gripper in the air? Then `zero` before the first touch. `help` lists commands.')
        bench = Bench(arm, mount, args)
        repl(bench)
    finally:
        try:
            if args.execute:
                arm.hand_back()
        finally:
            arm.disconnect()


if __name__ == '__main__':
    main()

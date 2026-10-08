"""One visual-servo grasp run: level the tool, servo to a hover pose above the plate, descend, close on the
FSRs, lift, return. The Camera-Edge ZEDYOLOServo._execute_run sequence, driven by 3D plate estimates.

ServoRunner knows nothing about ROS. The caller feeds it detections with observe(); start() runs one grasp
on a worker thread and abort() stops it. Every arm call goes through `arm` (arm.XArm, or a fake in the
tests); with dry_run the arm is only read, and the run servoes "in place": the plate estimate, the target
and the command it would send are computed and reported until abort() or align_timeout.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import logging
import threading
import time

import numpy as np

from .arm import ArmError, flange_for_tip, level_pose
from .control import (Gains, PlateFilter, PoseHistory, finger_angle_offset, grasp_yaw, hover_target, plate_in_base,
                      servo_command)
from .mount import Mount

log = logging.getLogger(__name__)


@dataclass
class GripperConfig:
    """The Camera-Edge FSR grasp (config.py): close in steps, nudge sideways while only one finger presses."""
    close_start: int = 50              # GRIPPER_CLOSE
    step: int = 20                     # GRIPPER_STEP
    max_close: int = 350
    samples: int = 100                 # N_SAMPLES
    rate: float = 2.0                  # LOOP_HZ
    target_pressure: tuple = (15, -1)  # one finger above this and not the other: nudge (Camera-Edge: 250, 120)
    stop_pressure: tuple = (25, -1)    # all FSRs in use above this: done; negative leaves one out (Camera-Edge: 370, 140)
    final_squeeze: int = 30
    nudge_mm: float = 1.0              # NUDGE_MM
    nudge_speed: float = 30.0          # mm/s
    use_fsr: bool = False              # False: close to the gripper's position + close_offset, no FSR feedback
    close_offset: int = 350            # gripper units; 350 held the plate on the arm (2026-09-22)
    close_settle: float = 4.0          # s for the fingers to close before lifting (1.5 lifted mid-close)
    tare_samples: int = 5              # FSR reads at the hover, gripper open; the pressures count from their median (0: raw)


@dataclass
class ServoConfig:
    dry_run: bool = True
    control_rate: float = 50.0          # Hz
    command_duration: float = 0.25      # s the arm keeps a velocity command before zeroing it
    level_at_start: bool = True         # roll 180, pitch 0 before servoing (Camera-Edge move_to_start_pose)
    level_speed: float = 100.0          # mm/s
    hover_height: float = 0.08          # m, fingertips above the plate's top face at the end of the servo
    grasp_offset: tuple = (0.0, 0.0)    # m, fingertip target shift in the grasp frame (x: along the plate's long side, y: finger axis)
    finger_axis: str = 'y'              # flange axis the fingers close along
    settle_count: int = 5               # consecutive in-tolerance detections that end the servo
    max_image_age: float = 0.5          # s, older detections are ignored
    hold_timeout: float = 0.5           # s without a fresh detection: zero velocity
    lost_timeout: float = 5.0           # s without a detection: the attempt fails
    align_timeout: float = 60.0         # s per servo attempt (0 = none)
    max_travel: float = 0.25            # m, the fingertip target must stay this close to the fingertip at the start
    min_tip_z: float = -0.08            # m, base z floor for any fingertip target
    plate_size: tuple = (0.12776, 0.08548)   # m (long, short): SBS footprint
    size_tolerance: float = 0.35        # reject estimates whose measured size is off by more (0 = off)
    min_conf: float = 0.25              # detections below this count only with depth and a close size match:
    weak_size_tolerance: float = 0.12   # both sides within this fraction of plate_size
    filter_alpha: float = 0.3
    filter_gate: float = 0.03           # m
    grasp: bool = True                  # False: stop at the hover pose
    grasp_depth: float = -0.002         # m below the top face (0.005 hit the tray on the arm: 7 mm higher held)
    approach_speed: float = 30.0        # mm/s, hover to `final_segment` above the grasp height
    final_segment: float = 0.02         # m
    final_speed: float = 8.0            # mm/s
    lift: float = 0.10                  # m after closing
    lift_speed: float = 50.0            # mm/s
    max_retries: int = 3                # GRASP_MAX_RETRIES
    retry_lift: float = 0.05            # m, GRASP_RETRY_LIFT_MM
    return_to_start: bool = True        # back to the pose the run started from (Camera-Edge _initial_pose)
    return_speed: float = 100.0         # mm/s
    gains: Gains = field(default_factory=Gains)
    gripper: GripperConfig = field(default_factory=GripperConfig)


class Aborted(Exception):
    pass


class RunFailed(Exception):
    pass


class ServoRunner:
    def __init__(self, arm, mount: Mount, config: ServoConfig, clock=time.time, sleep=time.sleep, prepare=None):
        """`clock` must be the clock of the image stamps (ROS time = time.time on the robot computer).
        `prepare(check_abort)`, if given, runs first on the run's thread (e.g. switch the camera hub to the
        wrist camera); an exception from it fails the run."""
        self.arm, self.mount, self.config = arm, mount, config
        self.prepare = prepare
        self.clock, self.sleep = clock, sleep
        self.history = PoseHistory()
        self.plate = PlateFilter(config.filter_alpha, config.filter_gate)
        self._lock = threading.Lock()
        self._pending = []          # detections waiting for the control loop: (stamp, detections, intrinsics)
        self._abort = threading.Event()
        self._thread = None
        self._on_done = None
        self._status = {'phase': 'idle', 'running': False, 'result': None, 'message': '', 'dry_run': config.dry_run}

    # -- API -----------------------------------------------------------------------------------------
    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self, on_done=None):
        """Start a run; False if one is already running."""
        if self.running:
            return False
        self._abort.clear()
        self._on_done = on_done
        with self._lock:
            self._pending.clear()
            self._status = {'phase': 'starting', 'running': True, 'result': None, 'message': '',
                            'dry_run': self.config.dry_run, 'attempt': 0, 'started': self.clock()}
        self._thread = threading.Thread(target=self._run, name='visual_servo', daemon=True)
        self._thread.start()
        return True

    def abort(self, wait=0.0):
        self._abort.set()
        if wait and self._thread is not None:
            self._thread.join(wait)

    def status(self):
        with self._lock:
            status = dict(self._status)
        return status

    def latest_pose(self):
        """The newest base_T_flange of the running servo loop, or None."""
        return self.history.latest() if self.running else None

    def observe(self, stamp, detections, intrinsics):
        """Detections (camera_perception Detection, most confident first) of the image captured at `stamp`."""
        if not self.running:
            return
        with self._lock:
            self._pending.append((float(stamp), list(detections), intrinsics))
            del self._pending[:-4]

    # -- helpers -------------------------------------------------------------------------------------
    def _set(self, **values):
        with self._lock:
            self._status.update(values)

    def _check_abort(self):
        if self._abort.is_set():
            raise Aborted()

    def _wait(self, seconds):
        end = self.clock() + seconds
        while self.clock() < end:
            self._check_abort()
            self.sleep(min(0.05, max(0.0, end - self.clock())))

    def _yaw(self, base_T_flange):
        return float(np.arctan2(base_T_flange[1, 0], base_T_flange[0, 0]))

    def _tip(self, base_T_flange):
        return (base_T_flange @ self.mount.flange_T_tip)[:3, 3]

    def _absorb_detections(self):
        """Turn queued detections into plate estimates, with the arm pose at each image's capture time."""
        with self._lock:
            pending, self._pending = self._pending, []
        used = 0
        now = self.clock()
        cfg = self.config
        for stamp, detections, intrinsics in pending:
            if not detections:
                continue
            if now - stamp > cfg.max_image_age:
                self._drop('stale', now - stamp)
                continue
            base_T_flange = self.history.at(stamp)
            if base_T_flange is None:
                self._drop('no_pose', now - stamp)
                continue
            base_T_camera = base_T_flange @ self.mount.flange_T_camera
            estimates = (plate_in_base(det, intrinsics, base_T_camera, stamp, cfg.plate_size) for det in detections)
            estimate = next((e for e in estimates if e is not None and self._plausible(e)), None)
            if estimate is None:
                self._drop('implausible', now - stamp)
                continue
            if self.plate.update(estimate):
                used += 1
                self._set(plate_source=estimate.source, plate_conf=round(estimate.conf, 3))
            else:
                self._drop('outlier', now - stamp)
        return used

    def _drop(self, reason, age):
        """Count a detection the servo didn't use, by reason, for the status (with the image's age when dropped)."""
        with self._lock:
            dropped = dict(self._status.get('dropped', {}))    # a new dict: status() copies are read unlocked
            dropped[reason] = dropped.get(reason, 0) + 1
            self._status['dropped'] = dropped
            self._status['last_drop'] = f'{reason}, image {age * 1000:.0f} ms old'

    def _plausible(self, estimate):
        """Whether a detection may steer the servo. Clutter next to the plate can drop YOLO's score far below
        min_conf (0.90 -> 0.03 on the arm, a game controller beside the plate) while the box stays on the plate, so
        a weak detection counts if the depth measures the plate's size on both sides."""
        cfg = self.config
        errors = [abs(measured / nominal - 1.0) for measured, nominal in zip(estimate.size, cfg.plate_size)]
        if estimate.conf >= cfg.min_conf:
            ok = cfg.size_tolerance <= 0.0 or errors[0] <= cfg.size_tolerance
        else:
            ok = estimate.source != 'size' and max(errors) <= cfg.weak_size_tolerance
        if not ok:
            self._set(rejected=f'size {estimate.size[0] * 1000:.0f} x {estimate.size[1] * 1000:.0f} mm, '
                               f'conf {estimate.conf:.2f}')
        return ok

    def _target(self, base_T_flange):
        """(fingertip target, yaw) for the current plate estimate, checked against the limits."""
        cfg = self.config
        yaw = grasp_yaw(self.plate.short_axis, self._yaw(base_T_flange), cfg.finger_axis)
        return self._offset_target(hover_target(self.plate.center, cfg.hover_height), yaw), yaw

    def _offset_target(self, target, yaw):
        cfg = self.config
        if any(cfg.grasp_offset):
            # grasp frame: y along the fingers, x across them, both horizontal
            finger = yaw + finger_angle_offset(cfg.finger_axis)
            along = np.array([np.cos(finger), np.sin(finger)])
            across = np.array([along[1], -along[0]])
            target = target.copy()
            target[:2] += cfg.grasp_offset[0] * across + cfg.grasp_offset[1] * along
        self._limit(target)
        return target

    def _limit(self, target):
        cfg = self.config
        if target[2] < cfg.min_tip_z:
            raise RunFailed(f'fingertip target z {target[2] * 1000:.0f} mm is below min_tip_z {cfg.min_tip_z * 1000:.0f} mm')
        travel = float(np.linalg.norm(target - self._start_tip))
        if travel > cfg.max_travel:
            raise RunFailed(f'fingertip target is {travel * 1000:.0f} mm from the start (max_travel '
                            f'{cfg.max_travel * 1000:.0f} mm): plate misdetected or out of reach')

    # -- the run ---------------------------------------------------------------------------------------
    def _run(self):
        cfg = self.config
        start_pose = None
        self._controlled_end = False    # return to the start only after a run that ended as planned
        try:
            if self.prepare is not None:
                self._set(phase='selecting camera')
                self.prepare(self._check_abort)
            self.arm.connect()
            start_pose = self.arm.flange_pose()
            self._start_tip = self._tip(start_pose)
            self._set(phase='preparing', start_tip_mm=[round(v * 1000, 1) for v in self._start_tip])
            if not cfg.dry_run:
                self.arm.position_mode()
                if cfg.grasp:
                    self.arm.gripper_setup()
                    self.arm.gripper_open()
                if cfg.level_at_start:
                    self.arm.move_flange(level_pose(start_pose), cfg.level_speed)
            for attempt in range(cfg.max_retries + 1):
                self._set(attempt=attempt + 1)
                self._align()
                if cfg.dry_run:
                    break
                if not cfg.grasp:
                    self._set(result='aligned', message='at the hover pose (grasp disabled)')
                    break
                if self._grasp():
                    self._set(result='grasped', message=f'grasped on attempt {attempt + 1}')
                    break
                if attempt < cfg.max_retries:
                    log.info('grasp failed: retry %d/%d', attempt + 1, cfg.max_retries)
                    self._retry_lift()
                else:
                    self._set(result='failed', message=f'no grasp after {cfg.max_retries + 1} attempts')
            self._controlled_end = True
        except Aborted:
            self._set(result='aborted', message='stopped')
        except (RunFailed, ArmError) as exc:
            log.error('visual servo failed: %s', exc)
            self._set(result='failed', message=str(exc))
        except Exception as exc:   # report, then leave the arm in a state xarm_ros2 can take over
            log.exception('visual servo crashed')
            self._set(result='failed', message=f'{type(exc).__name__}: {exc}')
        finally:
            self._finish(start_pose)

    def _finish(self, start_pose):
        cfg = self.config
        self.arm.stop()
        if not cfg.dry_run and self.arm.connected:
            try:
                if cfg.return_to_start and start_pose is not None and self._controlled_end:
                    self._set(phase='returning')
                    self.arm.position_mode()
                    self.arm.move_flange(start_pose, cfg.return_speed)
            except Exception as exc:
                log.warning('return to the start pose failed: %s', exc)
                self._set(message=(self.status().get('message') or '') + f'; return failed: {exc}')
            try:
                self.arm.hand_back()
            except Exception as exc:
                log.warning('hand back (mode 1) failed: %s', exc)
        if self.status().get('result') is None:
            self._set(result='stopped' if cfg.dry_run else 'done')
        self._set(phase='idle', running=False, finished=self.clock())
        if self._on_done is not None:
            try:
                self._on_done(self.status())
            except Exception:
                log.exception('on_done callback failed')

    def _align(self):
        """Velocity servo to the hover pose. Returns when the fingertip has held the target for settle_count
        detections; in a dry run, only on abort or align_timeout."""
        cfg = self.config
        self.plate.reset()
        self.history.clear()
        with self._lock:
            self._pending.clear()
        self._set(phase='servoing', settled=0)
        if not cfg.dry_run:
            self.arm.velocity_mode()
        period = 1.0 / cfg.control_rate
        started = self.clock()
        last_seen = started
        settled = 0
        last_count = started
        next_tick = started
        while True:
            self._check_abort()
            now = self.clock()
            pose = self.arm.flange_pose()
            self.history.add(now, pose)
            fresh = self._absorb_detections()
            if self.plate.last_stamp is not None:
                last_seen = max(last_seen, self.plate.last_stamp)
            if self.plate.center is not None:
                # the plate is still and its estimate is in the base frame: frames YOLO misses reuse the last one.
                # Without fresh detections, settling counts every hold_timeout instead of per detection
                target, yaw = self._target(pose)
                cmd = servo_command(pose, self.mount.flange_T_tip, self.arm.flange_T_tcp, target, yaw, cfg.gains)
                if fresh or now - last_count >= cfg.hold_timeout:
                    settled = settled + 1 if cmd.within else 0
                    last_count = now
                self._set(stale_s=round(now - self.plate.last_stamp, 2))
                self._set(error_mm=[round(v * 1000, 1) for v in cmd.error_xyz],
                          error_yaw_deg=round(float(np.degrees(cmd.error_yaw)), 2),
                          command_mm_s=[round(v * 1000, 1) for v in cmd.v_tcp], command_wz=round(cmd.wz, 3),
                          plate_mm=[round(v * 1000, 1) for v in self.plate.center],
                          target_mm=[round(v * 1000, 1) for v in target], settled=settled,
                          observations=self.plate.count)
                if settled >= cfg.settle_count and not cfg.dry_run:
                    self.arm.stop()
                    return
                if not cfg.dry_run:
                    self.arm.velocity(cmd.v_tcp, cmd.wz, cfg.command_duration)
            else:
                self.arm.stop()
                settled = 0
                if now - last_seen > cfg.lost_timeout:
                    raise RunFailed(f'no plate detection for {cfg.lost_timeout:.1f} s')
            if cfg.align_timeout > 0.0 and now - started > cfg.align_timeout:
                if cfg.dry_run:
                    return
                raise RunFailed(f'servo did not converge in {cfg.align_timeout:.0f} s')
            next_tick += period
            delay = next_tick - self.clock()
            if delay > 0:
                self.sleep(delay)
            else:
                next_tick = self.clock()

    def _grasp(self):
        """Descend onto the plate, close on the FSRs, lift. True if both fingers ended above threshold."""
        cfg, gcfg = self.config, self.config.gripper
        self._set(phase='descending')
        self.arm.stop()
        self.arm.position_mode()
        pose = self.arm.flange_pose()
        yaw = grasp_yaw(self.plate.short_axis, self._yaw(pose), cfg.finger_axis)
        grasp_tip = self.plate.center.copy()
        grasp_tip[2] -= cfg.grasp_depth
        grasp_tip = self._offset_target(grasp_tip, yaw)
        above = grasp_tip + np.array([0.0, 0.0, cfg.final_segment])
        base = self._fsr_baseline()
        self._set(grasp_tip_mm=[round(v * 1000, 1) for v in grasp_tip])
        self._check_abort()
        self.arm.move_flange(flange_for_tip(above, yaw, self.mount.flange_T_tip), cfg.approach_speed)
        self._check_abort()
        self.arm.move_flange(flange_for_tip(grasp_tip, yaw, self.mount.flange_T_tip), cfg.final_speed)

        self._set(phase='closing')
        if not gcfg.use_fsr:
            current = self.arm.gripper_position()
            value = min((current or 0) + gcfg.close_offset, 1000)
            self.arm.gripper_set(value, settle=gcfg.close_settle)
            self._set(gripper=value, gripper_before=current, fsr=list(self.arm.gripper_fsr()))
            self._set(phase='lifting')
            lifted = self.arm.flange_pose()
            lifted[2, 3] += cfg.lift
            self.arm.move_flange(lifted, cfg.lift_speed)
            log.info('closed the gripper %s -> %d without FSR feedback, lifted', current, value)
            return True
        value = gcfg.close_start
        self.arm.gripper_set(value)
        fsr = (None, None)
        for i in range(gcfg.samples):
            self._check_abort()
            fsr = self.arm.gripper_fsr()
            self._set(fsr=list(fsr), gripper=value)
            if fsr[0] is None:
                self._wait(1.0 / gcfg.rate)
                continue
            rise = (fsr[0] - base[0], fsr[1] - base[1])
            if self._pressed(rise):
                self.arm.gripper_set(value + gcfg.final_squeeze)
                break
            hit1, hit2 = rise[0] > gcfg.target_pressure[0], rise[1] > gcfg.target_pressure[1]
            if all(t >= 0 for t in gcfg.stop_pressure) and hit1 != hit2:
                # one finger touches, the other not: slide towards the free finger. Camera-Edge's nudge(-1 mm)
                # for FSR1 took the finger axis as if roll were 0, so it moved along flange +y: kept as tuned.
                self._nudge(gcfg.nudge_mm if hit1 else -gcfg.nudge_mm, yaw)
                value += gcfg.step // 4
            else:
                value = min(value + gcfg.step, gcfg.max_close)
            self.arm.gripper_set(value)

        self._set(phase='lifting')
        pose = self.arm.flange_pose()
        lifted = pose.copy()
        lifted[2, 3] += cfg.lift
        self.arm.move_flange(lifted, cfg.lift_speed)
        fsr = self.arm.gripper_fsr()
        self._set(fsr=list(fsr))
        ok = fsr[0] is not None and self._pressed((fsr[0] - base[0], fsr[1] - base[1]))
        log.info('grasp check after lift: fsr=%s (baseline %s) -> %s', fsr, base, 'held' if ok else 'not held')
        return ok

    def _pressed(self, rise):
        """Every FSR in use rose above its stop pressure. A negative stop pressure leaves that FSR out: on the arm
        FSR2 read 0 with the plate held (2026-09-22), so both-fingers-pressed never came and nudging can't balance."""
        used = [(r, t) for r, t in zip(rise, self.config.gripper.stop_pressure) if t >= 0]
        return bool(used) and all(r > t for r, t in used)

    def _fsr_baseline(self):
        """Median FSR readings with the gripper open and nothing between the fingers. An idle FSR need not read 0:
        on the arm FSR1 idled at ~360 and FSR2 at 0 (2026-09-22), so raw readings would count as a touch."""
        reads = [self.arm.gripper_fsr() for _ in range(max(0, self.config.gripper.tare_samples))]
        reads = [r for r in reads if r[0] is not None]
        base = tuple(int(v) for v in np.median(np.array(reads), axis=0)) if reads else (0, 0)
        self._set(fsr_baseline=list(base))
        log.info('FSR baseline %s from %d reads', base, len(reads))
        return base

    def _nudge(self, signed_mm, yaw):
        """Move `signed_mm` along the finger axis (flange y for finger_axis 'y')."""
        finger = yaw + finger_angle_offset(self.config.finger_axis)
        pose = self.arm.flange_pose()
        moved = pose.copy()
        moved[:2, 3] += signed_mm / 1000.0 * np.array([np.cos(finger), np.sin(finger)])
        self.arm.move_flange(moved, self.config.gripper.nudge_speed)

    def _retry_lift(self):
        cfg = self.config
        self._set(phase='retrying')
        self.arm.position_mode()
        pose = self.arm.flange_pose()
        pose[2, 3] += cfg.retry_lift
        self.arm.move_flange(pose, cfg.level_speed)
        self.arm.gripper_open()

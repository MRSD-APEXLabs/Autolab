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

from .arm import ArmError, GRIPPER_FSR_CUTOFF, flange_for_tip, level_pose
from .control import (Gains, PlateFilter, PoseHistory, finger_angle_offset, grasp_yaw, hover_target, plate_in_base,
                      servo_command)
from .mount import Mount

log = logging.getLogger(__name__)


@dataclass
class GripperConfig:
    """Close on tared FSR pressure; unload an active pad while centring the plate."""
    close_start: int = 50              # GRIPPER_CLOSE
    step: int = 10                     # replacement pads need smaller closing steps
    max_close: int = 350
    samples: int = 100                 # N_SAMPLES
    rate: float = 2.0                  # LOOP_HZ
    target_pressure: tuple = (60, 60) # early contact above each finger's unloaded baseline
    stop_pressure: tuple = (150, 150) # both fingers must press; a negative threshold disables that finger
    final_squeeze: int = 40            # additional close command after both fingers reach grip pressure
    final_squeeze_settle: float = 1.0 # seconds for the final adjustment immediately before lifting
    nudge_mm: float = 0.5
    nudge_speed: float = 5.0           # mm/s
    use_fsr: bool = True              # False: close to the gripper's position + close_offset, no FSR feedback
    close_offset: int = 350            # gripper units; 350 held the plate on the arm (2026-09-22)
    close_settle: float = 4.0          # s for the fingers to close before lifting (1.5 lifted mid-close)
    tare_samples: int = 25             # FSR reads at the hover, gripper open; pressures count from their median (0: raw)
    fsr_polarity: tuple = (1, 1)       # +1: contact raises ADC; -1: contact lowers ADC
    fsr_samples: int = 5              # median of this many reads for each contact decision
    fsr_sample_interval: float = 0.05 # seconds between independent ADC readings
    fsr_nudge_sign: tuple = (1, -1)   # direction along flange finger axis that unloads each pad
    max_nudge_mm: float = 5.0         # maximum displacement from the initial grasp position
    max_nudges: int = 10              # bound oscillations / an unresponsive second sensor
    # The wrist F/T sensor is read and logged at every grasp decision; use_ft lets it act: it overrides the FSR
    # nudge direction, stops the final descent on contact and decides the hold check after the lift
    use_ft: bool = False
    ft_samples: int = 5               # median of this many reads (fsr_sample_interval apart) per decision
    ft_finger_axis: int = 1           # sensor force component along the finger axis (0 fx, 1 fy, 2 fz)
    ft_finger_sign: int = 1           # lateral = sign * F[axis] > 0: a + nudge (fsr_nudge_sign) unloads it
    ft_lateral_deadband_n: float = 1.0  # below this the wrist feels no sideways push
    ft_mm_per_n: float = 0.0          # > 0: nudge this many mm per N of lateral force, in [nudge_mm, 4 nudge_mm]
    ft_contact_n: float = 5.0         # |Fz| above this during the final descent: the fingertips touched
    ft_contact_backoff: float = 0.002  # m back up after a descent contact
    ft_hold_min_n: float = 0.4        # Fz change over the lift that counts as holding the plate


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
    max_retries: int = 0                # one attempt per start command; retries explicitly opt in
    retry_lift: float = 0.05            # m, GRASP_RETRY_LIFT_MM
    return_to_start: bool = False       # keep the lifted pose; returning is an explicit option
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
        self._ft_ok = False         # the F/T sensor was zeroed for this grasp and still reads
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
                    if cfg.gripper.use_fsr:
                        self._wait(cfg.gripper.close_settle)  # finish opening before the unloaded tare
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
                    count = cfg.max_retries + 1
                    self._set(result='failed', message=f'no grasp after {count} attempt' + ('s' if count != 1 else ''))
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
        """Descend onto the plate, close on the FSRs, lift. True if the FSRs (with use_ft, the wrist Fz) say the
        plate is held. The F/T sensor is logged at every decision; with use_ft it also steers the nudges."""
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
        self._ft_setup()
        base = self._fsr_baseline() if gcfg.use_fsr else (0, 0)
        self._set(grasp_tip_mm=[round(v * 1000, 1) for v in grasp_tip])
        self._check_abort()
        self.arm.move_flange(flange_for_tip(above, yaw, self.mount.flange_T_tip), cfg.approach_speed)
        self._check_abort()
        self._descend(flange_for_tip(grasp_tip, yaw, self.mount.flange_T_tip), cfg.final_speed)

        self._set(phase='closing')
        if not gcfg.use_fsr:
            current = self.arm.gripper_position()
            value = min((current or 0) + gcfg.close_offset, 1000)
            self.arm.gripper_set(value, settle=gcfg.close_settle)
            self._set(gripper=value, gripper_before=current, fsr=list(self.arm.gripper_fsr()))
            ft_before = self._ft_read()
            self._ft_log('before lift', ft_before)
            self._set(phase='lifting')
            lifted = self.arm.flange_pose()
            lifted[2, 3] += cfg.lift
            self.arm.move_flange(lifted, cfg.lift_speed)
            log.info('closed the gripper %s -> %d without FSR feedback, lifted', current, value)
            return self._ft_hold(ft_before, True)
        value = gcfg.close_start
        self.arm.gripper_set(value)
        fsr = (None, None)
        nudge_total, nudges = 0.0, 0
        last_nudged_finger = None
        for i in range(gcfg.samples):
            self._check_abort()
            fsr = self._sample_fsr(gcfg.fsr_samples)
            self._set(fsr=list(fsr), gripper=value)
            rise = self._fsr_pressure(fsr, base)
            self._set(fsr_pressure=list(rise))
            if self._pressed(rise):
                break
            finger, lateral = self._ft_balance(self._unbalanced_finger(rise), rise)
            if finger is not None:
                # A switch to the opposite pad means the previous nudge unloaded
                # the first jaw. With a fixed opening the controller can bounce
                # between the pads forever. Take up a quarter step of slack only
                # after that switch, while below the firmware force cutoff, then
                # re-read both sensors before another correction.
                if (last_nudged_finger is not None and finger != last_nudged_finger
                        and max(fsr) <= GRIPPER_FSR_CUTOFF and value < gcfg.max_close):
                    previous = value
                    value = min(value + max(1, gcfg.step // 4), gcfg.max_close)
                    if not self.arm.gripper_set(value):
                        raise ArmError('gripper balance adjustment command failed')
                    self._set(gripper=value, balance_close_increment=value - previous)
                    last_nudged_finger = None
                    continue
                # Move the loaded jaw away from the plate, bringing the other jaw
                # closer. Hold the opening while centring; further squeezing here
                # increases the force on the finger that already touches.
                signed_mm = self._nudge_mm(lateral) * gcfg.fsr_nudge_sign[finger]
                nudge_total += signed_mm
                nudges += 1
                if abs(nudge_total) > gcfg.max_nudge_mm or nudges > gcfg.max_nudges:
                    raise RunFailed('FSR contact did not balance within the gripper nudge limits: '
                                    f'pressure={rise}, gripper={value}, nudges={nudges - 1}')
                self._nudge(signed_mm, yaw)
                last_nudged_finger = finger
                self._set(nudge_fsr=finger + 1, nudge_mm=signed_mm, nudge_total_mm=nudge_total, nudges=nudges)
                self._wait(1.0 / gcfg.rate)
                after = self._sample_fsr(gcfg.fsr_samples)
                after_rise = self._fsr_pressure(after, base)
                self._set(fsr=list(after), fsr_pressure=list(after_rise),
                          nudge_pressure_before=list(rise), nudge_pressure_after=list(after_rise))
                log.info('FSR%d nudge %+.1f mm: pressure %s -> %s, gripper %d',
                         finger + 1, signed_mm, rise, after_rise, value)
                self._ft_log(f'after nudge {signed_mm:+.1f} mm', self._ft_read(), pressure=after_rise)
                # A plate resting on the base can follow the loaded jaw. Even
                # with the correct unloading direction, a fixed opening then
                # never brings the opposite pad into contact. Take up a small
                # amount of slack only after a measured reduction in load.
                if (not self._pressed(after_rise)
                        and self._unbalanced_finger(after_rise) == finger
                        and rise[finger] - after_rise[finger] >= max(20, .1 * rise[finger])
                        and max(after) <= GRIPPER_FSR_CUTOFF and value < gcfg.max_close):
                    previous = value
                    value = min(value + max(1, gcfg.step // 4), gcfg.max_close)
                    if not self.arm.gripper_set(value):
                        raise ArmError('gripper balance adjustment command failed')
                    self._set(gripper=value, balance_close_increment=value - previous)
                    log.info('FSR load reduced: taking up slack, gripper %d -> %d', previous, value)
                    last_nudged_finger = None
                continue
            value = min(value + gcfg.step, gcfg.max_close)
            self.arm.gripper_set(value)
        else:
            self._set(fsr_contact_confirmed=False)
            log.warning('FSR thresholds not reached: retaining the gripper command without a lift')
            return False

        self._set(fsr_contact_confirmed=True, phase='grip_settling',
                  grip_contact_fsr=list(fsr), grip_contact_pressure=list(rise))
        self._ft_log('contact confirmed', self._ft_read(), pressure=rise)
        self._wait(gcfg.close_settle)
        settled_fsr = self._sample_fsr(gcfg.fsr_samples)
        self._set(fsr_before_squeeze=list(settled_fsr),
                  pressure_before_squeeze=list(self._fsr_pressure(settled_fsr, base)))
        self._ft_log('before squeeze', self._ft_read(), pressure=self._fsr_pressure(settled_fsr, base))
        self._squeeze_before_lift()
        prelift_fsr = self._sample_fsr(gcfg.fsr_samples)
        prelift_pressure = self._fsr_pressure(prelift_fsr, base)
        self._set(fsr_before_lift=list(prelift_fsr), pressure_before_lift=list(prelift_pressure))
        log.info('Grip contact %s; settled %s; after squeeze %s (baseline %s)',
                 fsr, settled_fsr, prelift_fsr, base)
        ft_before = self._ft_read()
        self._ft_log('before lift', ft_before, pressure=prelift_pressure)
        if not self._held_after_lift(prelift_pressure):
            raise RunFailed(f'grip lost before lift after final squeeze: pressure={prelift_pressure}')
        self._check_abort()
        self._set(phase='lifting')
        pose = self.arm.flange_pose()
        lifted = pose.copy()
        lifted[2, 3] += cfg.lift
        self.arm.move_flange(lifted, cfg.lift_speed)
        fsr = self._sample_fsr(gcfg.fsr_samples)
        rise = self._fsr_pressure(fsr, base)
        self._set(fsr=list(fsr), fsr_pressure=list(rise))
        # Both pads confirmed contact before lifting. Lifting changes how the
        # plate loads the pads; one pad can unload while the opposing pad still
        # carries the plate. Require retained pressure on at least one enabled
        # pad instead of incorrectly treating that redistribution as a lost grip.
        ok = self._held_after_lift(rise)
        self._set(hold_contact_confirmed=ok,
                  hold_fsr_active=[t >= 0 and r > t for r, t in zip(rise, gcfg.stop_pressure)])
        log.info('grasp check after lift: fsr=%s (baseline %s) -> %s', fsr, base, 'held' if ok else 'not held')
        return self._ft_hold(ft_before, ok)

    def _squeeze_before_lift(self):
        """After the main grip settles, send and verify the final increment before moving the arm."""
        cfg = self.config.gripper
        self._check_abort()
        before = self.arm.gripper_position()
        if before is None or not 0 <= before <= cfg.max_close:
            raise ArmError(f'cannot verify gripper setpoint before final squeeze: {before}')
        target = min(before + cfg.final_squeeze, cfg.max_close)
        self._set(phase='squeezing', gripper_before_squeeze=before, gripper_squeeze_target=target,
                  gripper_squeeze_delta=target - before, squeeze_command_verified=False)
        if target > before:
            if not self.arm.gripper_set(target, settle=0.0):
                raise ArmError('final gripper squeeze command failed')
        readback = self.arm.gripper_position()
        if readback != target:
            raise ArmError(f'final gripper squeeze readback mismatch: requested {target}, read {readback}')
        self._set(gripper=target, gripper_after_squeeze=readback, squeeze_command_verified=True)
        log.info('PRE-LIFT SQUEEZE: gripper %d -> %d (%+d), register readback %d',
                 before, target, target - before, readback)
        self._wait(cfg.final_squeeze_settle)

    def _pressed(self, rise):
        """Every enabled finger exceeds its stop pressure; a negative threshold disables that finger."""
        used = [(r, t) for r, t in zip(rise, self.config.gripper.stop_pressure) if t >= 0]
        return bool(used) and all(r > t for r, t in used)

    def _held_after_lift(self, rise):
        """After both pads confirmed the grasp, retain contact on either enabled pad."""
        return any(t >= 0 and r > t for r, t in zip(rise, self.config.gripper.stop_pressure))

    def _unbalanced_finger(self, pressure):
        """Which finger needs unloading; also balance when both touch but only one has enough force."""
        cfg = self.config.gripper
        if any(t < 0 for t in cfg.stop_pressure):
            return None  # balancing requires two working sensors
        for thresholds in (cfg.stop_pressure, cfg.target_pressure):
            active = [p > t for p, t in zip(pressure, thresholds)]
            if active[0] != active[1]:
                return 0 if active[0] else 1
        return None

    def _fsr_baseline(self):
        """Median unloaded readings at the hover pose. Mounting preload makes the idle ADC values nonzero."""
        count = self.config.gripper.tare_samples
        base = self._sample_fsr(count) if count > 0 else (0, 0)
        self._set(fsr_baseline=list(base))
        log.info('FSR baseline %s from %d reads', base, count)
        return base

    def _fsr_pressure(self, fsr, baseline):
        """Positive contact signal in ADC counts, with each finger's measured polarity."""
        return tuple((value - zero) * polarity for value, zero, polarity in
                     zip(fsr, baseline, self.config.gripper.fsr_polarity))

    def _sample_fsr(self, count):
        """Reject isolated spikes and fail a grasp if a majority of sensor replies are invalid."""
        reads = []
        for i in range(count):
            self._check_abort()
            reading = self.arm.gripper_fsr()
            if all(v is not None and 0 <= v <= 1023 for v in reading):
                reads.append(reading)
            if i + 1 < count:
                self._wait(self.config.gripper.fsr_sample_interval)
        if len(reads) <= count // 2:
            raise RunFailed(f'FSR sampling failed: {len(reads)}/{count} valid replies')
        return tuple(int(v) for v in np.median(reads, axis=0))

    # -- F/T sensor --------------------------------------------------------------------------------------
    def _ft_setup(self):
        """Switch the sensor on and zero it at the hover with the gripper open, beside the FSR tare."""
        self._ft_ok = False
        try:
            self.arm.ft_enable()
            raw = self.arm.ft_force()
            self.arm.ft_zero()
        except (ArmError, AttributeError) as exc:
            self._ft_failed(f'F/T sensor setup failed: {exc}')
            return
        self._ft_ok = True
        if raw is not None:
            log.info('F/T zero: raw F=[%.2f %.2f %.2f] N T=[%.3f %.3f %.3f] N m before zeroing', *raw)
        self._set(ft_raw_at_zero=[round(v, 3) for v in raw] if raw is not None else None)

    def _ft_failed(self, message):
        """With use_ft the grasp relies on the sensor and fails; without, the F/T log stops for this grasp."""
        if self.config.gripper.use_ft:
            raise RunFailed(message)
        log.warning('%s: continuing without F/T (use_ft off)', message)
        self._ft_ok = False
        self._set(ft_error=message)

    def _ft_read(self):
        """Median wrench of ft_samples reads, or None when the sensor is unavailable (use_ft off)."""
        if not self._ft_ok:
            return None
        cfg = self.config.gripper
        reads = []
        for i in range(cfg.ft_samples):
            self._check_abort()
            try:
                value = self.arm.ft_force()
            except ArmError:
                value = None
            if value is not None:
                reads.append(value)
            if i + 1 < cfg.ft_samples:
                self._wait(cfg.fsr_sample_interval)
        if len(reads) <= cfg.ft_samples // 2:
            self._ft_failed(f'F/T sampling failed: {len(reads)}/{cfg.ft_samples} valid reads')
            return None
        return [float(v) for v in np.median(reads, axis=0)]

    def _lateral(self, wrench):
        cfg = self.config.gripper
        return cfg.ft_finger_sign * wrench[cfg.ft_finger_axis]

    def _ft_log(self, phase, wrench, decision='', pressure=None):
        """One log line (and the status fields) per grasp decision, whether or not use_ft is on."""
        if wrench is None:
            return
        lateral = self._lateral(wrench)
        self._set(ft=[round(v, 3) for v in wrench], ft_lateral=round(lateral, 3), ft_phase=phase,
                  ft_decision=decision)
        log.info('F/T %s: F=[%.2f %.2f %.2f] N T=[%.3f %.3f %.3f] N m lateral=%+.2f N%s%s', phase, *wrench, lateral,
                 f' fsr_pressure={list(pressure)}' if pressure is not None else '', f' -> {decision}' if decision else '')

    def _ft_balance(self, finger, pressure):
        """Cross-check the FSR balancing choice (`finger` to unload, None: none) against the sideways wrist force.
        Returns the finger to unload, F/T's choice with use_ft, and the lateral force (None without F/T)."""
        cfg = self.config.gripper
        wrench = self._ft_read()
        if wrench is None:
            return finger, None
        lateral = self._lateral(wrench)
        ft_finger = None
        if abs(lateral) > cfg.ft_lateral_deadband_n:
            ft_finger = cfg.fsr_nudge_sign.index(1 if lateral > 0 else -1)
        if finger == ft_finger:
            decision = 'balanced' if finger is None else 'agree'
        else:
            if finger is None:
                decision = 'disagree: F/T-only imbalance'
            elif ft_finger is None:
                decision = 'disagree: no lateral force'
            else:
                decision = 'disagree: opposite sides'
            action = 'close' if ft_finger is None else f'unload FSR{ft_finger + 1}'
            decision += f', F/T override: {action}' if cfg.use_ft else f', would: {action}'
        self._ft_log('balance', wrench, decision, pressure)
        return (ft_finger if cfg.use_ft else finger), lateral

    def _nudge_mm(self, lateral):
        cfg = self.config.gripper
        if not cfg.use_ft or cfg.ft_mm_per_n <= 0.0 or lateral is None:
            return cfg.nudge_mm
        return float(np.clip(cfg.ft_mm_per_n * abs(lateral), cfg.nudge_mm, 4 * cfg.nudge_mm))

    def _descend(self, target, speed):
        """Final descent onto the plate. With use_ft, stop on a wrist Fz spike (the tray) and back off."""
        cfg = self.config.gripper
        if not (cfg.use_ft and self._ft_ok):
            self.arm.move_flange(target, speed)
            self._ft_log('grasp height', self._ft_read())
            return
        start = self.arm.flange_pose()
        deadline = self.clock() + np.linalg.norm(target[:3, 3] - start[:3, 3]) * 1000.0 / speed * 2.0 + 2.0
        self.arm.move_flange(target, speed, wait=False)
        try:
            while True:
                self._check_abort()
                wrench = self.arm.ft_force()
                if wrench is None:
                    raise RunFailed('F/T read failed during the final descent')
                if abs(wrench[2]) > cfg.ft_contact_n:
                    self.arm.halt()
                    backed = self.arm.flange_pose()
                    backed[2, 3] += cfg.ft_contact_backoff
                    self.arm.move_flange(backed, speed)
                    self._set(ft_contact=True, ft_contact_tip_mm=[round(v * 1000, 1) for v in self._tip(backed)])
                    self._ft_log('descent contact', wrench, f'stopped, backed off {cfg.ft_contact_backoff * 1000:.1f} mm')
                    return
                if np.linalg.norm(self.arm.flange_pose()[:3, 3] - target[:3, 3]) < 0.0005:
                    break
                if self.clock() > deadline:
                    raise RunFailed('final descent did not reach the grasp height')
                self.sleep(cfg.fsr_sample_interval)
        except BaseException:
            try:
                self.arm.halt()
            except Exception as exc:
                log.warning('halting the descent failed: %s', exc)
            raise
        self._set(ft_contact=False)
        self._ft_log('grasp height', self._ft_read())

    def _ft_hold(self, before, fsr_held):
        """After the lift: Fz must change by the plate's weight. Returns the grasp verdict, F/T's with use_ft."""
        cfg = self.config.gripper
        after = self._ft_read()
        if before is None or after is None:
            return fsr_held
        delta = abs(after[2] - before[2])
        held = delta >= cfg.ft_hold_min_n
        verdict = 'held' if held else 'not held'
        if held == fsr_held:
            decision = f'agree: {verdict}'
        else:
            decision = f'disagree: FSR {"held" if fsr_held else "not held"}, ' + (
                f'F/T override: {verdict}' if cfg.use_ft else f'would: {verdict}')
        self._set(ft_hold_delta_n=round(delta, 3), ft_hold=held)
        self._ft_log(f'after lift (|dFz| {delta:.2f} N)', after, decision)
        return held if cfg.use_ft else fsr_held

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
        if cfg.gripper.use_fsr:
            self._wait(cfg.gripper.close_settle)

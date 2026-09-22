"""swerve_bridge: /cmd_vel -> CTRE phoenix6 swerve drivetrain -> /odom + TF odom->base_footprint.

THE ONLY HARDWARE-FACING NODE of this stack.  Everything here is organised around one
question: "what happens to the motors if this goes wrong?".

Layers, from pure to dirty:
  1. resolve_target()   pure; decides Hardware vs Simulation BEFORE phoenix6 is imported, because
                        phoenix6 picks its native library at the first native call and DEFAULTS TO
                        REAL HARDWARE on this machine when CTR_TARGET is unset or misspelt.
  2. CommandGovernor    pure; clamp, steer stabiliser, speed-up-only slew, arming state machine
                        (cmd timeout, e-stop, zero-before-re-enable).  No ROS, no phoenix6 ->
                        exhaustively unit-tested.
  3. Drivetrain         the only code that imports phoenix6 (lazily) and owns every native object.
  4. ControlLoop        one tick governor -> drivetrain plus the fault policy (any exception ->
                        zero request, enable not fed, keep running).  ROS-free, tested with a fake.
  5. SwerveBridge       thin rclpy node: one timer, one mutually exclusive callback group, one
                        executor thread -> set_control() is only ever called from one thread.

Motor safety model (verified in ref/phoenix6_api.md):
  * phoenix6 swerve requests are LATCHED natively: the last request keeps driving for as long as
    the enable signal is fed.  So whenever the bridge is not armed it sends an explicit ZERO
    request AND stops feeding enable (Talons go neutral/brake ~0.1 s later), and it sends the zero
    request once more before it starts feeding enable again.
  * The native odometry thread is what applies a request to the modules.  If the drivetrain
    state stops advancing (hung thread, dead CAN device) no request - not even zero - gets
    through, so the bridge stops feeding enable until the state advances again.
  * A /cmd_vel is aged by the time it waited in the middleware queue, so an executor stall or a
    slow drivetrain start-up cannot turn a stale command into a fresh one.
  * If this process dies without any cleanup (SIGKILL, power), enable simply lapses after
    ENABLE_FEED_TIMEOUT_S.  That is the last line of defence, which is why the timeout is short.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
import enum
import importlib.util
import math
import os
import signal
import sys
import threading
import time
import traceback
from typing import Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.subscription import Subscription
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# =================================================================================================
# 1. Target resolution (pure)
# =================================================================================================

TARGET_HARDWARE = 'Hardware'       # exact, case-sensitive strings that phoenix6 compares against
TARGET_SIMULATION = 'Simulation'
FORCE_SIM_ENV = 'SWERVE_NAV_FORCE_SIM'
SITE_PACKAGES_ENV = 'PHOENIX6_SITE_PACKAGES'
_FORCE_SIM_OFF_VALUES = ('', '0', 'false', 'no', 'off')

EXIT_ERROR = 1
EXIT_HARDWARE_REFUSED = 2


class HardwareRefusedError(RuntimeError):
    """hardware:=true was requested but the environment forbids touching real hardware."""


class TargetVerificationError(RuntimeError):
    """The phoenix6 that actually got loaded is not the target that was decided."""


def resolve_target(hardware_param: bool, env: Mapping[str, str]) -> str:
    """Decide the phoenix6 native target.  Must run BEFORE anything imports phoenix6.

    Simulation is the default and can never be refused.  Hardware is refused when the environment
    says otherwise; every ambiguous case resolves AWAY from the real motors.
    """
    if not isinstance(hardware_param, bool):
        # A string such as "false" is truthy; never let that select the real robot.
        raise TypeError(f'hardware must be a bool, got {type(hardware_param).__name__}')
    if not hardware_param:
        return TARGET_SIMULATION
    force_sim = env.get(FORCE_SIM_ENV, '').strip().lower()
    if force_sim not in _FORCE_SIM_OFF_VALUES:
        raise HardwareRefusedError(
            f'hardware:=true refused: {FORCE_SIM_ENV}={env[FORCE_SIM_ENV]!r} forces simulation '
            '(this is a sandboxed/test environment)')
    ctr_target = env.get('CTR_TARGET')
    if ctr_target is not None and ctr_target != TARGET_HARDWARE:
        raise HardwareRefusedError(
            f'hardware:=true refused: the environment says CTR_TARGET={ctr_target!r}. '
            'Unset CTR_TARGET (or set it to "Hardware") if you really mean the real robot')
    return TARGET_HARDWARE


def resolve_site_packages(param_value: str, env: Mapping[str, str]) -> str:
    """Return the phoenix6 directory; env PHOENIX6_SITE_PACKAGES overrides the parameter."""
    return os.path.abspath(os.path.expanduser(env.get(SITE_PACKAGES_ENV) or param_value))


def drive_request_type_name(param_value: str) -> str:
    """Map the drive_request_type parameter to the phoenix6 DriveRequestType member name."""
    names = {'velocity': 'VELOCITY', 'open_loop': 'OPEN_LOOP_VOLTAGE'}
    if param_value not in names:
        raise ValueError(f'drive_request_type must be one of {sorted(names)}, got {param_value!r}')
    return names[param_value]


def check_native_target(target: str, is_simulation: bool, ctre_libraries: Iterable[str]) -> None:
    """Raise unless the natively loaded phoenix6 matches the decided target."""
    if is_simulation != (target == TARGET_SIMULATION):
        raise TargetVerificationError(
            f'target {target!r} requested but phoenix6 reports is_simulation()={is_simulation}')
    if target == TARGET_SIMULATION:
        not_sim = sorted(lib for lib in ctre_libraries if 'Sim' not in os.path.basename(lib))
        if not_sim:
            raise TargetVerificationError(
                f'simulation requested but non-simulation CTRE libraries are mapped: {not_sim}')


def mapped_ctre_libraries(maps_path: str = '/proc/self/maps') -> List[str]:
    with open(maps_path) as maps:
        return sorted({line.split()[-1] for line in maps if 'libCTRE' in line})


def queue_delay_s(now_ns: int, message_info: Optional[Mapping[str, object]]) -> float:
    """Seconds a message waited in the middleware queue before its callback ran (0 if unknown).

    rclpy hands a message over only when the executor gets to it.  After an executor stall, or
    the seconds a hardware drivetrain needs to come up while the subscription already exists, a
    long-stale /cmd_vel would otherwise be time-stamped "now" and drive the robot for
    cmd_timeout.  The caller back-dates the command by this delay instead.
    ``now_ns`` and the timestamps are wall-clock ns: a clock step in between can only make the
    command look older (dropped: fail-safe) or give a negative delay (clamped to 0).
    """
    info = message_info or {}
    # Reception time first (same host clock); some RMWs only fill in the source timestamp.
    for key in ('received_timestamp', 'source_timestamp'):
        stamp_ns = info.get(key)
        if isinstance(stamp_ns, int) and not isinstance(stamp_ns, bool) and stamp_ns > 0:
            return max(0.0, (now_ns - stamp_ns) * 1e-9)
    return 0.0


# =================================================================================================
# 2. Command governor (pure: no ROS, no phoenix6)
# =================================================================================================

# Absolute ceilings for the speed parameters: a typo in a launch file (10.0 instead of 1.0) must
# fail loudly at start-up instead of producing an indoor robot that does 10 m/s.
HARD_MAX_LINEAR_SPEED = 2.0    # m/s
HARD_MAX_ANGULAR_SPEED = 3.0   # rad/s
MAX_CMD_TIMEOUT_S = 1.0


def _require_finite_positive(name: str, value: float, upper: float = math.inf) -> None:
    if not (isinstance(value, float) and math.isfinite(value) and 0.0 < value <= upper):
        bound = '' if math.isinf(upper) else f' and <= {upper}'
        raise ValueError(f'{name} must be a finite float > 0{bound}, got {value!r}')


@dataclass(frozen=True)
class GovernorLimits:
    max_linear_speed: float      # m/s, applied to the NORM of (vx, vy)
    max_angular_speed: float     # rad/s
    max_linear_accel: float      # m/s^2, speed-up only
    max_angular_accel: float     # rad/s^2, speed-up only
    cmd_timeout: float           # s
    max_dt: float = 0.1          # s; a stalled timer must not turn into one giant slew step

    def __post_init__(self) -> None:
        _require_finite_positive('max_linear_speed', self.max_linear_speed, HARD_MAX_LINEAR_SPEED)
        _require_finite_positive('max_angular_speed', self.max_angular_speed,
                                 HARD_MAX_ANGULAR_SPEED)
        _require_finite_positive('max_linear_accel', self.max_linear_accel)
        _require_finite_positive('max_angular_accel', self.max_angular_accel)
        _require_finite_positive('cmd_timeout', self.cmd_timeout, MAX_CMD_TIMEOUT_S)
        _require_finite_positive('max_dt', self.max_dt)


class ArmState(enum.Enum):
    DISARMED = 'disarmed'   # zero request every tick, enable NOT fed
    PRIMED = 'primed'       # arm condition just became true: one more zero request, no enable
    ARMED = 'armed'         # governed command + enable fed


class CommandVerdict(enum.Enum):
    ACCEPTED = 'accepted'
    IGNORED_E_STOP = 'ignored_e_stop'
    REJECTED_NON_FINITE = 'rejected_non_finite'


@dataclass(frozen=True)
class DriveCommand:
    vx: float
    vy: float
    wz: float
    enable: bool    # True: send this request AND feed enable.  False: velocities are zero.


STOP_COMMAND = DriveCommand(0.0, 0.0, 0.0, enable=False)


def clamp_norm(x: float, y: float, limit: float) -> Tuple[float, float]:
    """Scale (x, y) so its norm is at most ``limit``, keeping the direction."""
    peak = max(abs(x), abs(y))
    if peak <= limit / math.sqrt(2.0):
        return x, y     # the norm cannot exceed the limit; also covers (0, 0)
    # Normalise by the peak first so huge-but-finite inputs cannot overflow hypot() to inf.
    ux, uy = x / peak, y / peak
    unit_norm = math.hypot(ux, uy)              # in [1, sqrt(2)]
    if peak * unit_norm <= limit:
        return x, y
    return ux / unit_norm * limit, uy / unit_norm * limit


def clamp_abs(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def slew_speed_up_only(prev: Sequence[float], target: Sequence[float],
                       max_step: float) -> Tuple[float, ...]:
    """Move from ``prev`` towards ``target``; only the speeding-up part is rate limited.

    Slowing down along the current direction of motion is never delayed (a stop must be
    immediate), so the move is split in two: first drop, for free, to the point of the segment
    [0, prev] closest to the target, then approach the target by at most ``max_step``.
    A reversal therefore passes through zero at once and ramps up in the new direction.
    """
    pp = sum(p * p for p in prev)
    keep = 0.0
    if pp > 0.0:
        keep = max(0.0, min(1.0, sum(t * p for t, p in zip(target, prev)) / pp))
    base = [keep * p for p in prev]
    delta = [t - b for t, b in zip(target, base)]
    distance = math.sqrt(sum(d * d for d in delta))
    scale = 1.0 if distance <= max_step else max_step / distance
    return tuple(b + scale * d for b, d in zip(base, delta))


MAX_STEER_HOLD_ANGLE = 0.35    # rad (20 deg): more would visibly bend every commanded path


class SteerStabiliser:
    """Keeps the azimuths still while the command only jitters.

    All four module angles depend only on the DIRECTION of the twist (vx, vy, radius * wz), not
    on its size, and a twist and its exact opposite need the same angles (the modules reverse
    their drive instead of turning).  So the wobble is removed by stabilising that direction:

      * a target direction within ``hold_angle`` of the current one changes nothing (hysteresis:
        controller noise no longer reaches the steer motors);
      * while the robot is moving, the direction follows a larger change at no more than
        ``max_rate``; from rest it jumps straight to the new direction, so starts are not delayed;
      * the output keeps the command's full speed while the direction lags by less than
        FULL_SPEED_LAG (damping the azimuths must not cost drive), and fades to zero as the lag
        approaches 90 deg.  It is never faster than the command, a zero command passes as zero
        and a slowdown is never delayed.
    """

    FULL_SPEED_LAG = 0.5    # rad

    def __init__(self, hold_angle: float, max_rate: float, radius: float) -> None:
        if not (isinstance(hold_angle, float) and 0.0 <= hold_angle <= MAX_STEER_HOLD_ANGLE):
            raise ValueError(f'steer_hold_angle must be a float in [0, {MAX_STEER_HOLD_ANGLE}] '
                             f'rad, got {hold_angle!r}')
        _require_finite_positive('steer_max_rate', max_rate)
        _require_finite_positive('steer_radius', radius)
        self._hold_angle = hold_angle
        self._max_rate = max_rate
        self._radius = radius
        self._direction: Optional[Tuple[float, float, float]] = None

    def reset(self) -> None:
        self._direction = None

    def apply(self, vx: float, vy: float, wz: float, at_rest: bool,
              dt: float) -> Tuple[float, float, float]:
        target = (vx, vy, wz * self._radius)
        norm = math.sqrt(sum(c * c for c in target))
        if norm < 1e-9:
            return 0.0, 0.0, 0.0    # stopped: the modules hold, and so does the direction
        unit = tuple(c / norm for c in target)
        if self._direction is None:
            self._direction = unit
            return vx, vy, wz
        dot = sum(a * b for a, b in zip(self._direction, unit))
        ref = self._direction if dot >= 0.0 else tuple(-c for c in self._direction)
        cross = (ref[1] * unit[2] - ref[2] * unit[1],
                 ref[2] * unit[0] - ref[0] * unit[2],
                 ref[0] * unit[1] - ref[1] * unit[0])
        angle = math.atan2(math.sqrt(sum(c * c for c in cross)), abs(dot))   # [0, pi/2]
        lag = angle
        if angle > self._hold_angle:
            step = angle - self._hold_angle
            if not at_rest:
                step = min(step, self._max_rate * dt)
            # Rotate ref towards unit by ``step`` along the great circle (slerp).
            sin_angle = math.sin(angle)
            k_ref = math.sin(angle - step) / sin_angle
            k_unit = math.sin(step) / sin_angle
            ref = tuple(k_ref * r + k_unit * u for r, u in zip(ref, unit))
            length = math.sqrt(sum(c * c for c in ref))
            ref = tuple(c / length for c in ref)
            lag = angle - step
        self._direction = ref
        along = norm * min(1.0, max(0.0, math.cos(lag)) / math.cos(self.FULL_SPEED_LAG))
        return along * ref[0], along * ref[1], along * ref[2] / self._radius


class CommandGovernor:
    """Turns the latest /cmd_vel + e-stop + time into what the drivetrain may do this tick.

    Arm condition ("armed" in the contract): NOT e_stop AND a command younger than cmd_timeout
    exists AND the caller does not inhibit.  Transitions:

        DISARMED --arm condition--> PRIMED --arm condition still true--> ARMED
        any state --arm condition false / fault()--> DISARMED

    PRIMED outputs zero without enable, so the natively LATCHED request is guaranteed to be zero
    at the moment enable is fed again.  Times are seconds on any monotonic clock.
    """

    def __init__(self, limits: GovernorLimits, steer: Optional[SteerStabiliser] = None,
                 rest_linear: float = 0.0, rest_angular: float = 0.0) -> None:
        self._limits = limits
        self._steer = steer
        # At or below these output speeds the drivetrain holds its module angles (phoenix6
        # deadbands), so the next direction may be taken at once.
        self._rest_linear = rest_linear
        self._rest_angular = rest_angular
        self._state = ArmState.DISARMED
        self._e_stop = False
        self._command: Optional[Tuple[float, float, float]] = None
        self._command_time = 0.0
        self._output = (0.0, 0.0, 0.0)
        self._last_update: Optional[float] = None
        self._rejected_commands = 0

    @property
    def state(self) -> ArmState:
        return self._state

    @property
    def e_stop(self) -> bool:
        return self._e_stop

    @property
    def rejected_commands(self) -> int:
        return self._rejected_commands

    def command_age(self, now: float) -> Optional[float]:
        return None if self._command is None else now - self._command_time

    def on_command(self, vx: float, vy: float, wz: float, now: float) -> CommandVerdict:
        if not all(math.isfinite(v) for v in (vx, vy, wz, now)):
            # NaN/inf means the sender is broken: treat as zero and demand a new, sane command.
            self._rejected_commands += 1
            self.fault()
            return CommandVerdict.REJECTED_NON_FINITE
        if self._e_stop:
            # Nothing received during an e-stop may become active when it is released.
            return CommandVerdict.IGNORED_E_STOP
        self._command = (vx, vy, wz)
        self._command_time = now
        return CommandVerdict.ACCEPTED

    def on_e_stop(self, active: bool) -> None:
        self._e_stop = bool(active)
        if self._e_stop:
            self.fault()

    def fault(self) -> None:
        """Forget the command and disarm; only a NEW command can arm again (through PRIMED)."""
        self._command = None
        self._disarm()

    def update(self, now: float, inhibit: bool = False) -> DriveCommand:
        if not math.isfinite(now):
            self.fault()
            return STOP_COMMAND
        dt = 0.0
        if self._last_update is not None:
            dt = max(0.0, min(now - self._last_update, self._limits.max_dt))
        self._last_update = now

        if self._command is not None:
            age = now - self._command_time
            if not 0.0 <= age < self._limits.cmd_timeout:
                self._command = None    # a stale command is gone for good, it can never revive
        if self._command is None or self._e_stop or inhibit:
            self._disarm()
            return STOP_COMMAND
        if self._state is ArmState.DISARMED:
            self._state = ArmState.PRIMED
            return STOP_COMMAND

        self._state = ArmState.ARMED
        vx, vy, wz = self._command
        vx, vy = clamp_norm(vx, vy, self._limits.max_linear_speed)
        wz = clamp_abs(wz, self._limits.max_angular_speed)
        if self._steer is not None:
            at_rest = (math.hypot(*self._output[:2]) <= self._rest_linear
                       and abs(self._output[2]) <= self._rest_angular)
            vx, vy, wz = self._steer.apply(vx, vy, wz, at_rest, dt)
        out_vx, out_vy = slew_speed_up_only(self._output[:2], (vx, vy),
                                            self._limits.max_linear_accel * dt)
        (out_wz,) = slew_speed_up_only(self._output[2:], (wz,),
                                       self._limits.max_angular_accel * dt)
        self._output = (out_vx, out_vy, out_wz)
        return DriveCommand(out_vx, out_vy, out_wz, enable=True)

    def _disarm(self) -> None:
        self._state = ArmState.DISARMED
        self._output = (0.0, 0.0, 0.0)   # the slew always restarts from rest
        if self._steer is not None:
            self._steer.reset()


# =================================================================================================
# 3. Odometry / status helpers (pure)
# =================================================================================================

STATIONARY_LINEAR_SPEED = 0.02    # m/s   measured speed below which the robot counts as at rest
STATIONARY_ANGULAR_SPEED = 0.02   # rad/s


@dataclass(frozen=True)
class OdomSample:
    valid: bool               # safe to publish and to base decisions on
    timestamp: float          # s, phoenix6 (CLOCK_MONOTONIC) timebase; 0.0 before the first update
    age: float                # s since the state was captured
    x: float
    y: float
    yaw: float
    vx: float                 # robot frame
    vy: float
    wz: float
    successful_daqs: int
    failed_daqs: int
    odometry_period: float


def odometry_usable(timestamp: float, successful_daqs: int, odometry_valid: bool,
                    values: Iterable[float]) -> bool:
    """Return False during phoenix6 start-up and whenever a NaN would poison TF/SLAM."""
    return (math.isfinite(timestamp) and timestamp != 0.0 and successful_daqs > 0
            and odometry_valid and all(math.isfinite(v) for v in values))


class OdometryWatchdog:
    """Tells whether the drivetrain state is still being produced by successful acquisitions.

    get_state_copy() keeps returning the last state, "valid" flag included, when the native
    odometry thread hangs or every acquisition fails (dead CAN device).  That same thread is
    what applies the swerve request to the modules, so in that condition a new request - a zero
    request included - would never reach the motors while enable was still being fed, and
    Nav2 / field-centric teleop would be steering on a frozen pose.  Progress therefore means:
    a valid, recent state whose successful_daqs counter has moved.
    """

    def __init__(self, stale_after: float) -> None:
        self._stale_after = stale_after
        self._daqs: Optional[int] = None
        self._last_progress: Optional[float] = None

    def observe(self, sample: Optional[OdomSample], now: float) -> None:
        if sample is None or not sample.valid or not sample.age < self._stale_after:
            return
        if sample.successful_daqs != self._daqs:    # "!=": a counter reset is progress as well
            self._daqs = sample.successful_daqs
            self._last_progress = now

    def alive(self, now: float) -> bool:
        return (self._last_progress is not None
                and 0.0 <= now - self._last_progress < self._stale_after)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def diagonal_covariance(diagonal: Sequence[float], name: str) -> List[float]:
    if len(diagonal) != 6 or not all(math.isfinite(v) and v >= 0.0 for v in diagonal):
        raise ValueError(f'{name} must be 6 finite non-negative numbers, got {list(diagonal)!r}')
    covariance = [0.0] * 36
    covariance[::7] = [float(v) for v in diagonal]
    return covariance


def reset_odometry_refusal(state: ArmState, sample: Optional[OdomSample],
                           command_pending: bool = False) -> Optional[str]:
    """Return why ~/reset_odometry must be refused right now, or None when it is allowed.

    A pose jump while something is driving on that pose (Nav2, field-centric teleop) would send
    the robot somewhere unintended, hence: only when disarmed AND measurably at rest.
    ``command_pending``: a command was accepted but the next tick has not armed on it yet.
    ``sample``: pass None when the odometry is not alive (its speeds would be frozen values).
    """
    if state is not ArmState.DISARMED:
        return f'drivetrain is {state.value}: stop commanding /cmd_vel first'
    if command_pending:
        return 'a /cmd_vel command is about to arm the drivetrain: stop commanding /cmd_vel first'
    if sample is None or not sample.valid:
        return 'no valid odometry, cannot verify that the robot is stationary'
    if (math.hypot(sample.vx, sample.vy) > STATIONARY_LINEAR_SPEED
            or abs(sample.wz) > STATIONARY_ANGULAR_SPEED):
        return (f'robot is moving (v={math.hypot(sample.vx, sample.vy):.3f} m/s, '
                f'wz={sample.wz:.3f} rad/s)')
    return None


@dataclass(frozen=True)
class BridgeStatus:
    hardware: bool
    state: ArmState
    e_stop: bool
    enabled: bool
    cmd_age: Optional[float]
    odometry_ready: bool
    sample: Optional[OdomSample]
    fault_count: int
    last_fault: str
    rejected_commands: int
    sim_alive: Optional[bool]     # None on hardware
    drive_request_type: str
    sim_error: str = ''           # why the simulation stepping thread died


LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2


def summarise_status(status: BridgeStatus) -> Tuple[int, str, List[Tuple[str, str]]]:
    """(level, message, key/values) for /swerve/status."""
    if not status.odometry_ready:
        level, message = LEVEL_ERROR, 'odometry not valid or not advancing: driving is inhibited'
    elif status.sim_alive is False:
        level, message = LEVEL_ERROR, f'simulation stepping thread died: {status.sim_error}'
    elif status.e_stop:
        level, message = LEVEL_WARN, 'e-stop active'
    elif status.last_fault:
        level, message = LEVEL_WARN, f'{status.fault_count} control fault(s), last: ' \
                                     f'{status.last_fault}'
    else:
        level, message = LEVEL_OK, status.state.value
    sample = status.sample
    values = [
        ('mode', 'hardware' if status.hardware else 'sim'),
        ('enabled', str(status.enabled)),
        ('armed', str(status.state is ArmState.ARMED)),
        ('state', status.state.value),
        ('e_stop', str(status.e_stop)),
        ('cmd_age', 'none' if status.cmd_age is None else f'{status.cmd_age:.3f}'),
        ('odometry_ready', str(status.odometry_ready)),
        ('successful_daqs', str(sample.successful_daqs if sample else 0)),
        ('failed_daqs', str(sample.failed_daqs if sample else 0)),
        ('odometry_period', f'{sample.odometry_period:.4f}' if sample else 'none'),
        ('state_age', f'{sample.age:.4f}' if sample else 'none'),
        ('control_faults', str(status.fault_count)),
        ('rejected_commands', str(status.rejected_commands)),
        ('drive_request_type', status.drive_request_type),
    ]
    if status.sim_alive is not None:
        values.append(('sim_thread_alive', str(status.sim_alive)))
    return level, message, values


# =================================================================================================
# 4. Drivetrain: the only phoenix6 user
# =================================================================================================

# Enable must be re-fed within this window or the Talons go neutral (brake).  Short on purpose:
# it bounds how long the motors stay live after this process hangs or is SIGKILLed.
ENABLE_FEED_TIMEOUT_S = 0.1


@contextlib.contextmanager
def _no_bytecode() -> Iterator[None]:
    """Never drop .pyc files into the user's (read-only by agreement) venv / generated folder."""
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        yield
    finally:
        sys.dont_write_bytecode = previous


class Drivetrain:
    """Owns the phoenix6 swerve drivetrain.  NOT thread-safe: use from one thread only."""

    def __init__(self, *, target: str, site_packages: str, tuner_constants_dir: str,
                 work_dir: str, odometry_hz: float, drive_request_type: str,
                 deadband: float, rotational_deadband: float, diagnostics: bool, logger) -> None:
        if target not in (TARGET_HARDWARE, TARGET_SIMULATION):
            raise ValueError(f'invalid target {target!r}')
        request_type_name = drive_request_type_name(drive_request_type)
        self._simulation = target == TARGET_SIMULATION
        self._closed = False
        self._sim = None

        # FIRST, before any phoenix6 / tuner_constants import: phoenix6 reads this variable at
        # its first native call, and anything but the exact strings falls back to HARDWARE.
        os.environ['CTR_TARGET'] = target

        # tuner_constants opens ./logs/example.hoot relative to the cwd at import time and the
        # simulation target writes ./ctre_sim: keep both out of wherever the node was started.
        work_dir = os.path.abspath(os.path.expanduser(work_dir))
        os.makedirs(work_dir, exist_ok=True)
        os.chdir(work_dir)

        with _no_bytecode():
            # Appended, not prepended: the directory may also hold pip/pygame, which must never
            # shadow the system packages that ROS uses.
            if site_packages not in sys.path:
                sys.path.append(site_packages)
            import phoenix6
            from phoenix6 import swerve, unmanaged, utils

            package_root = os.path.realpath(site_packages) + os.sep
            if not os.path.realpath(phoenix6.__file__).startswith(package_root):
                raise TargetVerificationError(
                    f'phoenix6 was imported from {phoenix6.__file__}, not from {site_packages}')
            if self._simulation or not diagnostics:
                # The diagnostics server (TCP 1250, Tuner X) is OFF by default, on hardware too.
                # It used to be left running here "because the user works with Tuner X", and on
                # this robot its start is what immediately precedes the 50 Hz control timer going
                # silent: /odom and TF stop, nothing is logged, and every Nav2 transform lookup
                # fails with "extrapolation into the future".  Three hardware runs froze 0.6 s,
                # 7.2 s and 0.95 s after this server came up; simulation, which has always
                # disabled it, has never reproduced it.  Set phoenix_diagnostics:=true for a
                # Tuner X session - but then do not expect navigation to survive.
                unmanaged.set_phoenix_diagnostics_start_time(-1)
            # The first native call above/below is what loads the libraries: verify them now,
            # before tuner_constants (which opens the CAN bus object) is imported.
            check_native_target(target, bool(utils.is_simulation()), mapped_ctre_libraries())

            constants_module = self._load_tuner_constants(tuner_constants_dir)

        tuner = constants_module.TunerConstants
        modules = [tuner.front_left, tuner.front_right, tuner.back_left, tuner.back_right]
        self._unmanaged = unmanaged
        self._utils = utils
        # ONE long-lived request (with_* mutate and return it).
        #
        # The deadbands are NOT cosmetic.  A swerve module's ANGLE comes from atan2() of its
        # velocity vector, so as the commanded chassis velocity approaches zero the angle becomes
        # numerically meaningless: a command that jitters around zero (which is exactly what a
        # controller produces while the robot is not making progress) swings the azimuths through
        # large random angles while the drive speed stays negligible.  The steer motors track it
        # eagerly (k_p 100) and the wheels just wander.  Below the deadband phoenix6 zeroes the
        # request and the modules HOLD their angle instead, which is what we want.
        # Set both to 0.0 to pass every command through unchanged.
        self._request = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                getattr(swerve.SwerveModule.DriveRequestType, request_type_name))
            .with_deadband(float(deadband))
            .with_rotational_deadband(float(rotational_deadband)))
        # python float on purpose: phoenix6 26.3.0 rejects an int odometry frequency.
        self._drivetrain = constants_module.TunerSwerveDrivetrain(
            tuner.drivetrain_constants, float(odometry_hz), modules)
        try:
            # Constructing the devices can map further libraries: verify again.
            libraries = mapped_ctre_libraries()
            check_native_target(target, bool(utils.is_simulation()), libraries)
            self.stop()     # replace the drivetrain's initial Idle request by an explicit zero
            if self._simulation:
                from swerve_navigation.phoenix_sim import PySimSwerve
                self._sim = PySimSwerve(self._drivetrain, modules)
                self._sim.start()
            logger.info(
                f'phoenix6 {target} target up: {len(libraries)} CTRE libraries mapped, odometry '
                f'{self._drivetrain.get_odometry_frequency():.0f} Hz, drive request '
                f'{request_type_name}, work_dir {work_dir}')
        except BaseException:
            # The caller never gets a reference to this object: clean up the native side here.
            self.shutdown(logger)
            raise

    @staticmethod
    def _load_tuner_constants(directory: str):
        # Loaded by file path instead of putting the directory on sys.path: that folder also
        # contains the user's motion scripts (final.py, test.py, robot.py), which must never
        # become importable from this process.
        path = os.path.join(os.path.abspath(os.path.expanduser(directory)), 'tuner_constants.py')
        spec = importlib.util.spec_from_file_location('tuner_constants', path)
        if spec is None or spec.loader is None or not os.path.isfile(path):
            raise FileNotFoundError(f'tuner_constants.py not found at {path}')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    @property
    def simulation(self) -> bool:
        return self._simulation

    def sim_alive(self) -> Optional[bool]:
        return None if self._sim is None else self._sim.is_alive()

    def sim_error(self) -> str:
        return '' if self._sim is None or self._sim.error is None else repr(self._sim.error)

    def enabled(self) -> bool:
        return bool(self._unmanaged.get_enable_state())

    def drive(self, vx: float, vy: float, wz: float) -> None:
        """Apply a velocity request and keep the motors enabled for ENABLE_FEED_TIMEOUT_S."""
        # Request first: if it raises, enable is not fed for this tick.
        self._apply(vx, vy, wz)
        self._unmanaged.feed_enable(ENABLE_FEED_TIMEOUT_S)

    def stop(self) -> None:
        """Latch a zero request and deliberately do NOT feed enable."""
        self._apply(0.0, 0.0, 0.0)

    def _apply(self, vx: float, vy: float, wz: float) -> None:
        self._require_open()
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            raise ValueError(f'non-finite drive request ({vx}, {vy}, {wz})')
        # set_control() must be called every time: mutating the request alone changes nothing.
        self._drivetrain.set_control(
            self._request
            .with_velocity_x(float(vx))
            .with_velocity_y(float(vy))
            .with_rotational_rate(float(wz)))

    def read(self) -> OdomSample:
        self._require_open()
        state = self._drivetrain.get_state_copy()    # thread-safe snapshot; get_state() is not
        pose, speeds = state.pose, state.speeds
        timestamp = float(state.timestamp)
        values = (float(pose.x), float(pose.y), float(pose.rotation().radians()),
                  float(speeds.vx), float(speeds.vy), float(speeds.omega))
        successful_daqs = int(state.successful_daqs)
        return OdomSample(
            valid=odometry_usable(timestamp, successful_daqs,
                                  bool(self._drivetrain.is_odometry_valid()), values),
            timestamp=timestamp,
            age=max(0.0, float(self._utils.get_current_time_seconds()) - timestamp),
            x=values[0], y=values[1], yaw=values[2],
            vx=values[3], vy=values[4], wz=values[5],
            successful_daqs=successful_daqs,
            failed_daqs=int(state.failed_daqs),
            odometry_period=float(state.odometry_period))

    def tare(self) -> None:
        """Zero the odometry pose.  Caller guarantees: disarmed and stationary."""
        self._require_open()
        self._drivetrain.tare_everything()

    def shutdown(self, logger) -> None:
        """Zero request -> feed_enable(0) -> stop sim thread -> close.  Idempotent.

        Every step runs even if an earlier one failed: a failing zero request must not prevent
        the disable, and nothing may touch the modules once close() destroyed the native object.
        """
        if self._closed:
            return
        steps = [('zero request', self.stop),
                 ('disable (feed_enable 0)', lambda: self._unmanaged.feed_enable(0))]
        if self._sim is not None:
            steps.append(('stop simulation thread', self._sim.stop))
        steps.append(('close drivetrain', self._drivetrain.close))
        for description, step in steps:
            try:
                step()
                logger.info(f'shutdown: {description} done')
            except Exception as exc:
                logger.error(f'shutdown: {description} FAILED: {exc!r}')
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError('drivetrain is closed')


class ControlLoop:
    """One control tick: governor -> drivetrain, with the fault policy.  ROS-free.

    ``drivetrain`` needs drive(vx, vy, wz) and stop(); ``report_fault`` receives a message.
    Policy for ANY exception in the control path: forget the command and disarm (so that moving
    again takes a NEW command and a new zero-only tick), try to latch a zero request, and do not
    feed enable.  If even the zero request fails the robot is still safe, because enable is no
    longer fed and the Talons go neutral within ENABLE_FEED_TIMEOUT_S; the next tick retries.
    Nothing is ever raised to the caller: the timer (and with it the watchdog) must keep running.
    """

    def __init__(self, governor: CommandGovernor, drivetrain, report_fault) -> None:
        self._governor = governor
        self._drivetrain = drivetrain
        self._report_fault = report_fault
        self.fault_count = 0
        self.last_fault = ''

    def step(self, now: float, inhibit: bool) -> None:
        try:
            command = self._governor.update(now, inhibit=inhibit)
            if command.enable:
                self._drivetrain.drive(command.vx, command.vy, command.wz)
            else:
                self._drivetrain.stop()
        except Exception as exc:
            self._on_fault(exc)

    def _on_fault(self, exc: Exception) -> None:
        self._governor.fault()
        self.fault_count += 1
        self.last_fault = repr(exc)
        message = f'CONTROL FAULT #{self.fault_count}: {exc!r} -> zero request, enable not fed'
        try:
            self._drivetrain.stop()
        except Exception as stop_exc:
            message += f'; the zero request failed as well ({stop_exc!r}): relying on the ' \
                       'enable timeout, retrying next tick'
        try:
            self._report_fault(message)
        except Exception:
            pass    # a broken logger must not break the safety path


# =================================================================================================
# 5. ROS node
# =================================================================================================

MIN_RATE_HZ = 20.0     # at least two control ticks per ENABLE_FEED_TIMEOUT_S, or enable flickers
MAX_RATE_HZ = 250.0
# The native odometry thread is also what applies the swerve request to the modules, so its
# period is control latency - for the zero request as well.  0.0 selects the CTRE default
# (250 Hz on CAN FD, 100 Hz on CAN 2.0).
MIN_ODOMETRY_HZ = 50.0
MAX_ODOMETRY_HZ = 1000.0
STATUS_PERIOD_S = 1.0
ODOMETRY_STALE_S = 0.25    # drivetrain state not advancing for this long -> driving is inhibited
TARE_SETTLE_S = 0.1        # resets take effect within one odometry period; do not publish the jump
CLOCK_STALL_WARN_S = 1.0   # phoenix6 timebase not advancing for this long -> warn, keep publishing
CONTROL_TICK_STALL_S = 0.5  # control timer not firing for this long -> say so from the spin loop
HEARTBEAT_PERIOD_S = 5.0   # one "alive" line this often: the loop is cheap, silence is expensive
PUBLISH_STALL_S = 0.5      # /odom not published for this long -> say so, and say WHY
QUEUE_DELAY_WARN_S = 0.1   # a /cmd_vel that waited this long for its callback is worth a warning


def validate_rates(rate_hz: float, odometry_hz: float) -> None:
    if not (math.isfinite(rate_hz) and MIN_RATE_HZ <= rate_hz <= MAX_RATE_HZ):
        raise ValueError(f'rate_hz must be within [{MIN_RATE_HZ}, {MAX_RATE_HZ}], got {rate_hz}')
    if odometry_hz != 0.0 and not (math.isfinite(odometry_hz)
                                   and MIN_ODOMETRY_HZ <= odometry_hz <= MAX_ODOMETRY_HZ):
        raise ValueError(f'odometry_hz must be 0.0 (CTRE default) or within '
                         f'[{MIN_ODOMETRY_HZ}, {MAX_ODOMETRY_HZ}], got {odometry_hz}')


HARDWARE_BANNER = (
    '=' * 78,
    '  SWERVE BRIDGE IS IN HARDWARE MODE: THE REAL DRIVE MOTORS ARE LIVE',
    '  The robot WILL MOVE when /cmd_vel is published. Keep clear of the robot and',
    '  keep the e-stop (joystick B button -> /e_stop) within reach.',
    '=' * 78,
)


class SwerveBridge(Node):
    """rclpy shell around CommandGovernor + Drivetrain.  Construct, then start().

    __init__ only reads parameters and resolves the target, so a refused hardware request ends
    the process before phoenix6 was ever imported.
    """

    def __init__(self) -> None:
        super().__init__('swerve_bridge')
        self._hardware = self._param(
            'hardware', False,
            'true = REAL motors on the CAN bus; false (default) = phoenix6 simulation target.')
        self._site_packages = resolve_site_packages(self._param(
            'phoenix6_site_packages',
            '/home/labx/nav_nav/Navigation/generated/venv/lib/python3.12/site-packages',
            'Directory providing phoenix6 + importlib_resources. The PHOENIX6_SITE_PACKAGES '
            'environment variable overrides this when set.'), os.environ)
        self._tuner_constants_dir = self._param(
            'tuner_constants_dir', '/home/labx/nav_nav/Navigation/generated',
            'Directory containing the Tuner X generated tuner_constants.py (loaded by path).')
        self._work_dir = self._param(
            'work_dir', '~/.ros/swerve_bridge',
            'Process cwd set before tuner_constants is imported (it opens ./logs/example.hoot and '
            'the simulation writes ./ctre_sim there).')
        self._odom_frame = self._param('odom_frame', 'odom', 'Odometry / TF parent frame.')
        self._base_frame = self._param(
            'base_frame', 'base_footprint', 'Odometry child frame / TF child frame.')
        self._publish_tf = self._param('publish_tf', True, 'Broadcast TF odom_frame->base_frame.')
        rate_hz = self._param(
            'rate_hz', 50.0, f'Control + odometry timer rate [{MIN_RATE_HZ}, {MAX_RATE_HZ}] Hz.')
        self._odometry_hz = self._param(
            'odometry_hz', 100.0, 'phoenix6 odometry thread frequency, Hz: 0.0 = CTRE default, '
            f'else [{MIN_ODOMETRY_HZ}, {MAX_ODOMETRY_HZ}] (it is also the control latency).')
        limits = GovernorLimits(
            cmd_timeout=self._param(
                'cmd_timeout', 0.3, 'Without a /cmd_vel newer than this the drivetrain is '
                f'zeroed and disabled, s (max {MAX_CMD_TIMEOUT_S}).'),
            max_linear_speed=self._param(
                'max_linear_speed', 1.0, 'Clamp on the norm of (vx, vy), m/s '
                f'(hard ceiling {HARD_MAX_LINEAR_SPEED}).'),
            max_angular_speed=self._param(
                'max_angular_speed', 1.5, f'Clamp on |wz|, rad/s (hard ceiling '
                f'{HARD_MAX_ANGULAR_SPEED}).'),
            max_linear_accel=self._param(
                'max_linear_accel', 2.0, 'Slew limit for speeding up, m/s^2. Slowing down and '
                'stops are never delayed.'),
            max_angular_accel=self._param(
                'max_angular_accel', 4.0, 'Slew limit for speeding up rotation, rad/s^2.'))
        self._deadband = self._param(
            'deadband', 0.02,
            'Commanded speeds below this [m/s] are treated as zero so the modules hold their '
            'angle instead of chasing the meaningless direction of a near-zero velocity vector. '
            '0.0 disables.')
        self._rotational_deadband = self._param(
            'rotational_deadband', 0.05,
            'Commanded rotation below this [rad/s] is treated as zero (see deadband). 0.0 disables.')
        steer = None
        if self._param('steer_filter', True,
                       'Stabilise the module angles against command jitter (SteerStabiliser).'):
            steer = SteerStabiliser(
                hold_angle=self._param(
                    'steer_hold_angle', 0.035,
                    'Direction changes of the commanded twist below this [rad] do not move the '
                    f'azimuths (max {MAX_STEER_HOLD_ANGLE}). 0.0 disables the hold band.'),
                max_rate=self._param(
                    'steer_max_rate', 2.5,
                    'Fastest change of the commanded twist direction while moving [rad/s]. '
                    'Stops, slowdowns and starts from rest are never delayed.'),
                radius=self._param(
                    'steer_radius', 0.359,
                    'Module distance from the base centre [m]; weighs wz against vx, vy.'))
        self._drive_request_type = self._param(
            'drive_request_type', 'velocity',
            '"velocity" (closed loop, default) or "open_loop" (voltage feed-forward only).')
        self._phoenix_diagnostics = self._param(
            'phoenix_diagnostics', False,
            'Start the phoenix6 diagnostics server (TCP 1250) so Tuner X can attach.  Off by '
            'default: on this robot its start is what precedes the control timer going silent.')
        self._pose_covariance = diagonal_covariance(self._param(
            'pose_covariance_diagonal', [0.01, 0.01, 1e-6, 1e-6, 1e-6, 0.02],
            'Diagonal of the /odom pose covariance (x, y, z, roll, pitch, yaw).'),
            'pose_covariance_diagonal')
        self._twist_covariance = diagonal_covariance(self._param(
            'twist_covariance_diagonal', [0.005, 0.005, 1e-6, 1e-6, 1e-6, 0.01],
            'Diagonal of the /odom twist covariance (vx, vy, vz, wx, wy, wz).'),
            'twist_covariance_diagonal')

        validate_rates(rate_hz, self._odometry_hz)
        drive_request_type_name(self._drive_request_type)   # fail before touching phoenix6
        self._period = 1.0 / rate_hz

        # Raises HardwareRefusedError.  phoenix6 has NOT been imported at this point.
        self._target = resolve_target(self._hardware, os.environ)

        self._governor = CommandGovernor(limits, steer, rest_linear=self._deadband,
                                         rest_angular=self._rotational_deadband)
        self._drivetrain: Optional[Drivetrain] = None
        self._control: Optional[ControlLoop] = None
        self._timer = None
        self._sample: Optional[OdomSample] = None
        self._odometry_watchdog = OdometryWatchdog(ODOMETRY_STALE_S)
        # Publication is deduplicated on the acquisition counter, not on the phoenix6 timestamp:
        # the counter is what says "this is a new pose", and it keeps counting even when the
        # CANivore-synced phoenix6 timebase stops advancing (which it does, see _publish_odometry).
        self._last_published_daqs: Optional[int] = None
        self._clock_timestamp: Optional[float] = None
        self._clock_progress = -math.inf
        self._last_tick = -math.inf
        self._last_heartbeat = -math.inf
        self._last_publish = -math.inf
        self._skip_reason: Optional[str] = None
        self._published = 0
        self._tared = False
        self._holdoff_until = 0.0
        self._last_status_time = -math.inf

    def _param(self, name: str, default, description: str):
        # read_only: every parameter is consumed once at start-up, so a later "ros2 param set"
        # must fail loudly instead of silently not changing a safety limit.
        descriptor = ParameterDescriptor(description=description, read_only=True)
        return self.declare_parameter(name, default, descriptor).value

    # ------------------------------------------------------------------------------------------
    def start(self) -> None:
        """Bring up phoenix6 and the ROS interfaces.  Only called after resolve_target()."""
        log = self.get_logger()
        if self._hardware:
            for line in HARDWARE_BANNER:
                log.warning(line)
        else:
            log.info('swerve_bridge mode: SIMULATION (phoenix6 Simulation target, no CAN traffic, '
                     'the real robot cannot move)')
        log.info(f'phoenix6 from {self._site_packages}; tuner_constants from '
                 f'{self._tuner_constants_dir}')

        group = MutuallyExclusiveCallbackGroup()
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._status_pub = self.create_publisher(DiagnosticArray, 'swerve/status', 1)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        # rclpy passes the MessageInfo (reception timestamps) only to a callback that REQUIRES a
        # second argument, and only from Iron on: Humble's rclpy has no MessageInfo and calls
        # every callback with the message alone.  Without it the queue delay counts as 0.
        on_cmd_vel = (self._on_cmd_vel if hasattr(Subscription, 'CallbackType')
                      else lambda msg: self._on_cmd_vel(msg, None))
        self.create_subscription(Twist, 'cmd_vel', on_cmd_vel, 1, callback_group=group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'e_stop', self._on_e_stop, latched, callback_group=group)
        # The latched subscription above does NOT match a volatile or best-effort publisher, and
        # "ros2 topic pub /e_stop ..." is volatile by default: an e-stop typed in a hurry would
        # be ignored without a trace.  This second subscription matches every publisher; getting
        # the same message twice is harmless because the e-stop is a state, not an event.
        any_publisher = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Bool, 'e_stop', self._on_e_stop, any_publisher,
                                 callback_group=group)
        self.create_service(Trigger, '~/reset_odometry', self._on_reset_odometry,
                            callback_group=group)

        self._drivetrain = Drivetrain(
            target=self._target, site_packages=self._site_packages,
            tuner_constants_dir=self._tuner_constants_dir, work_dir=self._work_dir,
            odometry_hz=self._odometry_hz, drive_request_type=self._drive_request_type,
            deadband=self._deadband, rotational_deadband=self._rotational_deadband,
            diagnostics=self._phoenix_diagnostics, logger=log)
        self._control = ControlLoop(
            self._governor, self._drivetrain,
            lambda message: log.error(message, throttle_duration_sec=1.0))
        try:
            # The single timer is the only caller of set_control()/feed_enable().  Steady clock:
            # a wall-clock step (NTP) must neither stall nor burst the control loop.
            self._timer = self.create_timer(self._period, self._on_timer, callback_group=group,
                                            clock=Clock(clock_type=ClockType.STEADY_TIME))
        except BaseException:
            self.shutdown_drivetrain()
            raise
        log.info(f'running at {1.0 / self._period:.0f} Hz; DISARMED until a fresh /cmd_vel '
                 'arrives and /e_stop is false')

    def shutdown_drivetrain(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
        if self._drivetrain is not None:
            self._drivetrain.shutdown(self.get_logger())

    # ------------------------------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist, message_info: Optional[Mapping[str, object]]) -> None:
        # time.monotonic(), not ROS time: a wall-clock step (NTP) must not defeat the watchdog.
        # Back-dated by the time the message sat in the queue, so that a command that went stale
        # while this executor was busy (or the drivetrain was starting) cannot count as fresh.
        delay = queue_delay_s(time.time_ns(), message_info)
        verdict = self._governor.on_command(msg.linear.x, msg.linear.y, msg.angular.z,
                                            time.monotonic() - delay)
        if verdict is CommandVerdict.REJECTED_NON_FINITE:
            self.get_logger().error('non-finite /cmd_vel rejected: drivetrain zeroed, disarmed',
                                    throttle_duration_sec=1.0)
        if delay > QUEUE_DELAY_WARN_S:
            self.get_logger().warning(
                f'/cmd_vel waited {delay:.3f} s in the queue (executor stall or clock step); '
                'it is aged accordingly', throttle_duration_sec=1.0)

    def _on_e_stop(self, msg: Bool) -> None:
        changed = msg.data != self._governor.e_stop
        self._governor.on_e_stop(msg.data)      # first: a logging problem must not prevent it
        if changed:
            self.get_logger().warning('E-STOP ACTIVE: drivetrain zeroed and disabled' if msg.data
                                      else 'e-stop released: a fresh /cmd_vel is needed to move')

    def _on_reset_odometry(self, _request: Trigger.Request,
                           response: Trigger.Response) -> Trigger.Response:
        now = time.monotonic()
        refusal = reset_odometry_refusal(
            self._governor.state,
            self._sample if self._odometry_watchdog.alive(now) else None,
            command_pending=self._governor.command_age(now) is not None)
        if refusal is None:
            try:
                self._drivetrain.tare()
                self._holdoff_until = time.monotonic() + TARE_SETTLE_S
            except Exception as exc:
                refusal = f'tare_everything failed: {exc!r}'
        response.success = refusal is None
        response.message = 'odometry reset to (0, 0, 0)' if refusal is None \
            else f'refused: {refusal}'
        self.get_logger().info(f'reset_odometry: {response.message}')
        return response

    # ------------------------------------------------------------------------------------------
    def _on_timer(self) -> None:
        now = time.monotonic()
        self._last_tick = now
        self._read_odometry(now)
        # No live odometry means a dead CAN bus / device, a hung odometry thread (which is also
        # what applies our requests) or a pending pose reset: whoever is commanding us (Nav2,
        # field-centric teleop) would be steering blind -> inhibit.
        ready = self._odometry_ready(now)
        self._control.step(now, inhibit=not ready)     # first: nothing may delay the safety path
        if not ready:
            self._guarded('odometry stall report', self._report_stalled_odometry, now)
        self._guarded('odometry publish', lambda at: self._publish_odometry(at, ready), now)
        self._guarded('publish stall check', self._check_publish_stall, now)
        if now - self._last_status_time >= STATUS_PERIOD_S:
            self._last_status_time = now
            self._guarded('status publish', self._publish_status, now)
        if now - self._last_heartbeat >= HEARTBEAT_PERIOD_S:
            self._last_heartbeat = now
            self._guarded('heartbeat', self._log_heartbeat, now)

    def _guarded(self, what: str, step, now: float) -> None:
        # Publishing problems must never take the control loop (and with it the watchdog) down.
        try:
            step(now)
        except Exception as exc:
            self.get_logger().error(f'{what} failed: {exc!r}', throttle_duration_sec=1.0)

    def _read_odometry(self, now: float) -> None:
        try:
            self._sample = self._drivetrain.read()
        except Exception as exc:
            self._sample = None
            self.get_logger().error(f'drivetrain state read failed: {exc!r}',
                                    throttle_duration_sec=1.0)
            return
        self._odometry_watchdog.observe(self._sample, now)
        self._note_clock_progress(self._sample, now)
        if not self._tared and self._odometry_watchdog.alive(now):
            # The one start-up tare: the governor is still inhibited, so nothing is driving.
            try:
                self._drivetrain.tare()
            except Exception as exc:
                self.get_logger().error(f'start-up tare failed: {exc!r}',
                                        throttle_duration_sec=1.0)
                return
            self._tared = True
            self._holdoff_until = now + TARE_SETTLE_S
            self.get_logger().info('odometry valid: pose tared to (0, 0, 0), publishing /odom')

    def _check_publish_stall(self, now: float) -> None:
        """Announce, loudly and with a reason, that /odom and TF have stopped.

        If the reason comes back as "publish call is being reached", the node believes it is
        publishing at 50 Hz and the messages are not arriving - which is a transport problem, not
        a drivetrain one, and needs looking at from the subscriber side.
        """
        if self._last_publish == -math.inf or now - self._last_publish < PUBLISH_STALL_S:
            return
        sample = self._sample
        detail = 'no sample' if sample is None else (
            f'daqs {sample.successful_daqs}/{sample.failed_daqs}, age {sample.age:.3f} s, '
            f'valid={sample.valid}')
        self.get_logger().error(
            f'/odom AND TF STOPPED {now - self._last_publish:.1f} s ago, after {self._published} '
            f'messages: {self._skip_reason or "publish call is being reached (no skip)"} '
            f'[{detail}, odom_subs={self._odom_pub.get_subscription_count()}]; localisation and '
            'Nav2 are now working from a frozen pose', throttle_duration_sec=2.0)

    def _log_heartbeat(self, now: float) -> None:
        """One line every few seconds, unconditionally.

        This exists because every failure so far has been a SILENCE: /odom and TF stop and not one
        node says anything.  The counters below separate the only remaining explanations - a loop
        that stopped running, a loop that runs but declines to publish, and a loop that publishes
        to nobody - which no amount of reading the other nodes' logs can do.
        """
        sample = self._sample
        state = 'no sample' if sample is None else (
            f'daqs {sample.successful_daqs}/{sample.failed_daqs} age {sample.age:.3f}s '
            f'valid={sample.valid} pose ({sample.x:.2f}, {sample.y:.2f}, {sample.yaw:.2f})')
        tf_pub = getattr(self._tf_broadcaster, 'pub_tf', None)
        tf_subs = tf_pub.get_subscription_count() if tf_pub is not None else '-'
        since = ('never' if self._last_publish == -math.inf
                 else f'{now - self._last_publish:.2f}s ago')
        self.get_logger().info(
            f'alive: ready={self._odometry_ready(now)} published={self._published} (last {since}) '
            f'odom_subs={self._odom_pub.get_subscription_count()} tf_subs={tf_subs} {state}'
            + (f' skip={self._skip_reason}' if self._skip_reason else ''))

    def check_control_loop_alive(self) -> None:
        """Report a control timer that has stopped firing.  Called from the spin loop, because a
        timer that is no longer firing cannot report its own absence - and that silence is exactly
        what made this failure so hard to see: /odom and TF just stopped, with nothing in the log.

        Nothing is driven from here: the drivetrain fails safe on its own, because feeding enable
        is what the timer does, and phoenix6 disables the motors once the feed stops arriving.
        """
        if self._timer is None or self._last_tick == -math.inf:
            return
        late = time.monotonic() - self._last_tick
        if late >= CONTROL_TICK_STALL_S:
            self.get_logger().error(
                f'CONTROL LOOP NOT TICKING: no timer callback for {late:.1f} s. /odom and TF have '
                'stopped, so localisation and Nav2 are working from a frozen pose; the drivetrain '
                'disables itself as the enable feed stops.', throttle_duration_sec=2.0)

    def _note_clock_progress(self, sample: Optional[OdomSample], now: float) -> None:
        """Say so when the phoenix6 timebase freezes, instead of going quietly blind.

        A frozen timebase is survivable - the acquisitions keep delivering fresh poses and the
        stamps come from the ROS clock - but it is not normal, and silence here once cost a whole
        run: /odom stopped and every Nav2 lookup failed with "extrapolation into the future".
        """
        if sample is None or not sample.valid:
            return
        if sample.timestamp != self._clock_timestamp:
            self._clock_timestamp = sample.timestamp
            self._clock_progress = now
            return
        if now - self._clock_progress >= CLOCK_STALL_WARN_S:
            self.get_logger().warning(
                f'phoenix6 timebase frozen at {sample.timestamp:.3f} for '
                f'{now - self._clock_progress:.1f} s while acquisitions keep succeeding: /odom '
                'and TF are being stamped from the ROS clock', throttle_duration_sec=10.0)

    def _odometry_ready(self, now: float) -> bool:
        return (self._tared and now >= self._holdoff_until
                and self._odometry_watchdog.alive(now))

    def _report_stalled_odometry(self, now: float) -> None:
        if not self._tared or now < self._holdoff_until:
            return      # start-up or a requested pose reset: expected, not a stall
        sample = self._sample
        detail = 'state read failed' if sample is None else (
            f'valid={sample.valid}, state age {sample.age:.3f} s, '
            f'daqs ok/failed {sample.successful_daqs}/{sample.failed_daqs}')
        self.get_logger().error(f'ODOMETRY STALLED ({detail}): driving is inhibited',
                                throttle_duration_sec=1.0)

    def _publish_odometry(self, now: float, ready: bool) -> None:
        sample = self._sample
        # Every skip below used to be a bare return, and that silence is what hid this failure for
        # four runs: /odom and TF stopped and no node, anywhere, said why.  Record the reason so
        # _check_publish_stall can name it.
        if sample is None:
            self._skip_reason = 'no drivetrain sample (state read failed)'
        elif not ready:
            # "not ready" is a pose that is not advancing (stalled acquisitions, dead CAN device)
            # or one that was just reset.  Publishing it anyway would stamp a frozen pose with the
            # current time, i.e. tell AMCL and Nav2 "the robot is standing still, as of now" - and
            # a controller that never sees the robot turn keeps asking it to turn, for ever.
            self._skip_reason = (
                f'odometry not ready (tared={self._tared}, '
                f'holdoff {max(0.0, self._holdoff_until - now):.2f} s, '
                f'watchdog_alive={self._odometry_watchdog.alive(now)})')
        elif sample.successful_daqs == self._last_published_daqs:
            # no new acquisition; a repeated stamp would make tf2 complain
            self._skip_reason = f'no new acquisition (daqs stuck at {sample.successful_daqs})'
        else:
            self._skip_reason = None
        if self._skip_reason is not None:
            return
        self._last_published_daqs = sample.successful_daqs
        self._published += 1
        self._last_publish = now
        # phoenix6 timestamps live in utils.get_current_time_seconds()'s timebase, not the ROS
        # clock, so the state is dated by its age.  The age is clamped because that timebase is
        # CANivore-synced and has been seen to stop advancing while acquisitions kept succeeding:
        # unclamped, every stamp would then freeze (or slide into the past) and tf2 would let the
        # whole odom subtree go stale, taking localisation and Nav2 down with it.
        age = min(max(0.0, sample.age), ODOMETRY_STALE_S)
        stamp = (self.get_clock().now() - Duration(nanoseconds=int(age * 1e9))).to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(sample.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = sample.x
        odom.pose.pose.position.y = sample.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self._pose_covariance
        odom.twist.twist.linear.x = sample.vx       # phoenix6 speeds are robot-centric already
        odom.twist.twist.linear.y = sample.vy
        odom.twist.twist.angular.z = sample.wz
        odom.twist.covariance = self._twist_covariance
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = sample.x
            transform.transform.translation.y = sample.y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)

    def _publish_status(self, now: float) -> None:
        level, message, values = summarise_status(BridgeStatus(
            hardware=self._hardware,
            state=self._governor.state,
            e_stop=self._governor.e_stop,
            enabled=self._drivetrain.enabled(),
            cmd_age=self._governor.command_age(now),
            odometry_ready=self._odometry_ready(now),
            sample=self._sample,
            fault_count=self._control.fault_count,
            last_fault=self._control.last_fault,
            rejected_commands=self._governor.rejected_commands,
            sim_alive=self._drivetrain.sim_alive(),
            drive_request_type=self._drive_request_type,
            sim_error=self._drivetrain.sim_error()))
        status = DiagnosticStatus()
        status.level = bytes([level])
        status.name = 'swerve_bridge: drivetrain'
        status.hardware_id = 'swerve_drivetrain'
        status.message = message
        status.values = [KeyValue(key=key, value=value) for key, value in values]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_pub.publish(array)


def main(args=None) -> None:
    # Own signal handling: rclpy's default handler tears the context down underneath us, but the
    # drivetrain must be zeroed/disabled/closed in order while everything is still alive.  Python's
    # default SIGTERM action would even skip phoenix6's atexit close().  SIGHUP: the terminal
    # (or ssh session) the bridge was started from went away.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: stop.set())

    exit_code = 0
    node = None
    try:
        node = SwerveBridge()      # parameters + resolve_target(); phoenix6 not imported yet
        node.start()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node.check_control_loop_alive()
        node.get_logger().info('stop requested: shutting the drivetrain down')
    except HardwareRefusedError as exc:
        get_logger('swerve_bridge').fatal(str(exc))
        exit_code = EXIT_HARDWARE_REFUSED
    except Exception:
        get_logger('swerve_bridge').fatal('swerve_bridge failed:\n' + traceback.format_exc())
        exit_code = EXIT_ERROR
    finally:
        if node is not None:
            node.shutdown_drivetrain()
            node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

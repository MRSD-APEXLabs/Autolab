"""Xbox-pad teleop for the swerve base: /joy -> /cmd_vel_teleop (+ /e_stop, /nav_cancel).

Same feel as the user's final.py (right stick translates field-centric, left stick X
rotates, A re-zeroes the field heading) with the safety behaviour a shared Nav2 stack needs:

* Motion is only ever commanded while the deadman (LT, held past deadman_axis_threshold) is
  held. On release exactly ONE zero Twist is sent and the node goes silent so that Nav2 can
  take over through cmd_vel_mux. The triggers read +1.0 RELEASED and -1.0 fully pressed, so
  they go through trigger_fraction() rather than being compared against zero.
* joy_node goes completely SILENT when the pad is unplugged, so a /joy watchdog treats a stale
  /joy as "deadman released".
* SDL also lists the 3Dconnexion SpaceMouse as a joystick (it is device index 0 on this
  machine). A Joy message with fewer than 8 axes / 11 buttons is therefore ignored completely.
* Field-centric driving uses the odometry yaw and a stored field_zero_yaw. Unlike final.py the
  gyro / odometry is NEVER reseeded: SLAM and AMCL need continuous odometry.
* B latches /e_stop (transient local); start releases it only with the deadman released and the
  sticks centred, so releasing the e-stop can never immediately produce motion.

All decision logic lives in the ROS-free TeleopLogic so that it is unit-testable.
"""

from dataclasses import dataclass
from enum import Enum
import math
import signal
import time
from typing import Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty

# joy_node (raw SDL layout) reports 8 axes / 11 buttons for the Xbox 360 pad. Anything smaller
# is another device (the SpaceMouse has 6 axes / 2 buttons) and must never drive the robot.
MIN_AXES = 8
MIN_BUTTONS = 11
# LT / RT rest at +1.0 (not 0.0): mapping a velocity to them commands full speed at rest.
TRIGGER_AXES = (2, 5)
MAX_WATCHDOG_TIMEOUT = 5.0  # [s] upper limit of joy_timeout / odom_timeout
PUBLISH_RATE_LIMITS = (5.0, 200.0)  # [Hz]

Command = Tuple[float, float, float]  # (vx, vy, wz): robot frame, REP-103
ZERO: Command = (0.0, 0.0, 0.0)


def apply_deadband(value: float, deadband: float) -> float:
    """Deadband with linear rescale, identical to final.py's apply_deadband inside [-1, 1].

    Hardened for a real robot: non-finite input gives 0.0 and the result is clamped to
    [-1, 1] (joy_node can overshoot 1.0 by a few 1e-5, a broken driver by anything).
    """
    if not math.isfinite(value) or abs(value) < deadband:
        return 0.0
    rescaled = (value - math.copysign(deadband, value)) / (1.0 - deadband)
    return max(-1.0, min(1.0, rescaled))


def trigger_fraction(value: float) -> float:
    """Convert an Xbox trigger axis to 0.0 (released) .. 1.0 (fully pressed).

    joy_node reports the triggers as +1.0 at rest and -1.0 fully pressed, so a naive
    "axis > 0" test would read a RELEASED trigger as fully engaged. Non-finite input gives
    0.0: a broken driver must never engage the deadman.
    """
    if not math.isfinite(value):
        return 0.0
    return max(0.0, min(1.0, (1.0 - value) / 2.0))


def checked_publish_rate(rate: float) -> float:
    """Return the rate if it is usable, raise ValueError otherwise."""
    # Too slow and the mux (teleop_timeout 0.5 s, teleop_hold 1.0 s) hands the robot to Nav2
    # BETWEEN two teleop messages while the deadman is held; inf would be a busy loop.
    low, high = PUBLISH_RATE_LIMITS
    if not low <= rate <= high:
        raise ValueError(f'publish_rate must be in [{low}, {high}] Hz (got {rate})')
    return rate


def field_to_robot(field_x: float, field_y: float,
                   robot_yaw_in_field: float) -> Tuple[float, float]:
    """Rotate a field-frame vector by -robot_yaw_in_field to express it in the robot frame."""
    c, s = math.cos(robot_yaw_in_field), math.sin(robot_yaw_in_field)
    return c * field_x + s * field_y, -s * field_x + c * field_y


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Yaw (rad) of a quaternion; None when it is non-finite or not close to unit length."""
    norm_sq = x * x + y * y + z * z + w * w
    if not math.isfinite(norm_sq) or abs(norm_sq - 1.0) > 0.1:
        return None
    # w^2 + x^2 - y^2 - z^2 instead of 1 - 2 (y^2 + z^2): exact for a slightly non-unit
    # quaternion too (the latter is 3 deg off at 45 deg of yaw for a norm^2 of 1.08).
    return math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)


@dataclass(frozen=True)
class TeleopConfig:
    """Parameters of TeleopLogic (defaults = interface contract)."""

    axis_vx: int = 4
    axis_vy: int = 3
    axis_wz: int = 0
    # The deadman can be an analogue trigger (deadman_axis, e.g. 2 = LT) and/or a plain button
    # (deadman_button, e.g. 4 = LB); either one engaged means "held". -1 disables that source.
    deadman_axis: int = 2
    deadman_axis_threshold: float = 0.6
    deadman_button: int = -1
    require_deadman: bool = True
    turbo_button: int = 5
    scale_linear: float = 0.30
    scale_angular: float = 0.50
    scale_linear_turbo: float = 0.80
    scale_angular_turbo: float = 1.20
    stick_deadband: float = 0.10
    field_centric: bool = True
    reseed_button: int = 0
    estop_button: int = 1
    estop_release_button: int = 7
    cancel_nav_button: int = 2
    estop_cancels_nav: bool = True
    joy_timeout: float = 0.5
    odom_timeout: float = 0.5

    def __post_init__(self):
        # A negative index would silently read from the END of the list in Python.
        for name in ('axis_vx', 'axis_vy', 'axis_wz', 'estop_button', 'estop_release_button'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} must be >= 0 (got {getattr(self, name)})')
        for name in ('turbo_button', 'reseed_button', 'cancel_nav_button',
                     'deadman_axis', 'deadman_button'):
            if getattr(self, name) < -1:
                raise ValueError(
                    f'{name} must be >= 0, or -1 to disable (got {getattr(self, name)})')
        # Without any deadman source, require_deadman would silently mean "can never drive".
        if self.require_deadman and self.deadman_axis < 0 and self.deadman_button < 0:
            raise ValueError(
                'require_deadman is true but both deadman_axis and deadman_button are -1: '
                'set at least one (deadman_axis 2 = LT, deadman_button 4 = LB)')
        # A trigger rests at +1.0 and reads -1.0 fully pressed, i.e. fraction 0.0 -> 1.0 (see
        # trigger_fraction). An axis stuck at exactly 0.0 maps to 0.5, so a threshold at or below
        # that midpoint could read "held" from a half-pressed or unread trigger: require > 0.5.
        if self.deadman_axis >= 0 and not 0.5 < self.deadman_axis_threshold <= 1.0:
            raise ValueError('deadman_axis_threshold must be in (0.5, 1.0] '
                             f'(got {self.deadman_axis_threshold})')
        if self.deadman_axis >= 0 and self.deadman_axis in (
                self.axis_vx, self.axis_vy, self.axis_wz):
            raise ValueError(f'deadman_axis {self.deadman_axis} is also a drive axis')
        for name in ('scale_linear', 'scale_angular', 'scale_linear_turbo', 'scale_angular_turbo'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and >= 0 (got {value})')
        if not 0.0 <= self.stick_deadband < 1.0:
            raise ValueError(f'stick_deadband must be in [0, 1) (got {self.stick_deadband})')
        # joy_node goes SILENT on unplug, so these two are the watchdogs: inf (valid YAML) or
        # a huge value would keep repeating the last stick position for ever.
        for name in ('joy_timeout', 'odom_timeout'):
            if not 0.0 < getattr(self, name) <= MAX_WATCHDOG_TIMEOUT:
                raise ValueError(f'{name} must be in (0, {MAX_WATCHDOG_TIMEOUT}] s '
                                 f'(got {getattr(self, name)})')
        # One button with two jobs is a trap, e.g. turbo == deadman means ALWAYS turbo.
        buttons = {name: getattr(self, name) for name in (
            'deadman_button', 'turbo_button', 'reseed_button', 'estop_button',
            'estop_release_button', 'cancel_nav_button') if getattr(self, name) >= 0}
        if len(set(buttons.values())) != len(buttons):
            raise ValueError(f'every button needs its own index (got {buttons})')

    @property
    def min_axes(self) -> int:
        return max(MIN_AXES, self.axis_vx + 1, self.axis_vy + 1, self.axis_wz + 1,
                   self.deadman_axis + 1)

    @property
    def min_buttons(self) -> int:
        return max(MIN_BUTTONS, 1 + max(
            self.deadman_button, self.turbo_button, self.reseed_button, self.estop_button,
            self.estop_release_button, self.cancel_nav_button))


class Status(Enum):
    SILENT = 'silent'          # not enabled: publish nothing
    RELEASED = 'released'      # just became not enabled: the single zero Twist
    DRIVING = 'driving'        # enabled, command from the sticks
    ESTOP = 'estop'            # enabled but e-stop latched: zero
    ODOM_STALE = 'odom_stale'  # enabled, field-centric without fresh odometry: zero


@dataclass(frozen=True)
class TickResult:
    command: Optional[Command]  # None = publish nothing
    status: Status
    detail: str = ''            # why teleop is not enabled (SILENT / RELEASED)


@dataclass(frozen=True)
class JoyEvents:
    """What one Joy message triggered; the node turns these into publishes / log lines."""

    layout_ok: bool = True
    estop_changed: bool = False
    release_refused: str = ''       # non-empty: start was pressed but the e-stop stays latched
    cancel_nav: bool = False
    reseeded: bool = False
    reseed_refused: bool = False    # A pressed without fresh odometry
    deadman_released: bool = False  # tick NOW so the zero Twist is not delayed by a timer period


@dataclass(frozen=True)
class _JoySnapshot:
    stamp: float
    layout_ok: bool
    deadman: bool = False
    turbo: bool = False
    # Deadbanded (forward, left, ccw) in [-1, 1]; None when any of the three axes is non-finite.
    sticks: Optional[Command] = None

    @property
    def deflected(self) -> bool:
        return self.sticks is not None and self.sticks != ZERO


class TeleopLogic:
    """ROS-free teleop state machine. All times are monotonic seconds supplied by the caller."""

    def __init__(self, config: TeleopConfig):
        self._cfg = config
        self._joy: Optional[_JoySnapshot] = None
        self._prev_buttons: Optional[Tuple[bool, ...]] = None
        self._yaw: Optional[float] = None
        self._yaw_stamp = 0.0
        self._field_zero_yaw: Optional[float] = None
        self._e_stop = False
        self._active = False

    @property
    def e_stop(self) -> bool:
        return self._e_stop

    @property
    def active(self) -> bool:
        """Return True between the first enabled tick and the zero Twist that ends it."""
        return self._active

    @property
    def field_zero_yaw(self) -> Optional[float]:
        return self._field_zero_yaw

    def on_odom(self, yaw: Optional[float], now: float) -> None:
        if yaw is None or not math.isfinite(yaw):
            return  # keep the previous yaw; it ages out through odom_timeout
        self._yaw, self._yaw_stamp = yaw, now
        if self._field_zero_yaw is None:
            self._field_zero_yaw = yaw

    def on_joy(self, axes: Sequence[float], buttons: Sequence[int], now: float) -> JoyEvents:
        cfg = self._cfg
        if len(axes) < cfg.min_axes or len(buttons) < cfg.min_buttons:
            # Wrong device: acts as "deadman released" and none of its buttons mean anything.
            self._joy = _JoySnapshot(stamp=now, layout_ok=False)
            return JoyEvents(layout_ok=False, deadman_released=self._active)

        pressed = tuple(bool(b) for b in buttons)
        # The first message only initialises edge detection: a button that is already down
        # when the node starts must be released and pressed again to count.
        previous = self._prev_buttons if self._prev_buttons is not None else pressed
        self._prev_buttons = pressed

        def rising(index: int) -> bool:
            return 0 <= index < len(previous) and pressed[index] and not previous[index]

        raw = (axes[cfg.axis_vx], axes[cfg.axis_vy], axes[cfg.axis_wz])
        sticks = None
        if all(math.isfinite(v) for v in raw):
            sticks = tuple(apply_deadband(v, cfg.stick_deadband) for v in raw)
        # Either deadman source engaged counts as held; a disabled source (-1) never engages.
        deadman = (
            (cfg.deadman_axis >= 0
             and trigger_fraction(axes[cfg.deadman_axis]) >= cfg.deadman_axis_threshold)
            or (cfg.deadman_button >= 0 and pressed[cfg.deadman_button]))
        self._joy = _JoySnapshot(
            stamp=now, layout_ok=True, deadman=deadman,
            turbo=cfg.turbo_button >= 0 and pressed[cfg.turbo_button], sticks=sticks)

        estop_changed = False
        release_refused = ''
        if pressed[cfg.estop_button] and not self._e_stop:
            # Level-triggered on purpose: latching twice is harmless, missing an edge is not.
            self._e_stop = True
            estop_changed = True
        elif self._e_stop and rising(cfg.estop_release_button):
            release_refused = self._release_blocker(pressed)
            if not release_refused:
                self._e_stop = False
                estop_changed = True

        reseeded = reseed_refused = False
        if cfg.field_centric and rising(cfg.reseed_button):
            if self._odom_fresh(now):
                self._field_zero_yaw = self._yaw
                reseeded = True
            else:
                reseed_refused = True

        # Latching the e-stop also cancels navigation: otherwise Nav2 keeps its goal and the
        # robot would drive off by itself the moment the e-stop is released.
        cancel_nav = rising(cfg.cancel_nav_button) or (
            estop_changed and self._e_stop and cfg.estop_cancels_nav)
        return JoyEvents(
            estop_changed=estop_changed, release_refused=release_refused, cancel_nav=cancel_nav,
            reseeded=reseeded, reseed_refused=reseed_refused,
            deadman_released=self._active and not self._enabled(now)[0])

    def tick(self, now: float) -> TickResult:
        enabled, why_not = self._enabled(now)
        if not enabled:
            if self._active:
                self._active = False
                return TickResult(ZERO, Status.RELEASED, why_not)
            return TickResult(None, Status.SILENT, why_not)
        self._active = True
        if self._e_stop:
            return TickResult(ZERO, Status.ESTOP)
        return self._drive_command(now)

    def _release_blocker(self, pressed: Sequence[bool]) -> str:
        """Why the e-stop may not be released right now ('' = it may)."""
        if pressed[self._cfg.estop_button]:
            return 'the e-stop button is still pressed'
        if self._joy.deadman:  # covers both deadman sources (trigger and/or button)
            return 'the deadman is held'
        if self._joy.sticks != ZERO:  # also refuses non-finite axes (sticks is None)
            return 'the sticks are not centred'
        return ''

    def _enabled(self, now: float) -> Tuple[bool, str]:
        joy = self._joy
        if joy is None:
            return False, 'no /joy received yet'
        if now - joy.stamp > self._cfg.joy_timeout:
            return False, (f'/joy is stale (> {self._cfg.joy_timeout:.2f} s): '
                           'pad unplugged or joy_node down?')
        if not joy.layout_ok:
            return False, 'the /joy publisher is not the Xbox pad'
        if self._cfg.require_deadman:
            return joy.deadman, 'deadman released'
        # Without a deadman a deflected stick is what enables teleop. Publishing zeros for
        # as long as a pad is plugged in would lock Nav2 out of the mux forever.
        return joy.deflected, 'sticks centred'

    def _odom_fresh(self, now: float) -> bool:
        return self._yaw is not None and now - self._yaw_stamp <= self._cfg.odom_timeout

    def _drive_command(self, now: float) -> TickResult:
        cfg, joy = self._cfg, self._joy
        forward, left, ccw = joy.sticks if joy.sticks is not None else ZERO
        # Limit the stick vector to the unit circle so that scale_linear really is the top
        # speed (per-axis deadbanding lets a diagonal reach a norm of 1.41).
        norm = math.hypot(forward, left)
        if norm > 1.0:
            forward, left = forward / norm, left / norm
        if cfg.field_centric:
            if not self._odom_fresh(now):
                return TickResult(ZERO, Status.ODOM_STALE)
            forward, left = field_to_robot(forward, left, self._yaw - self._field_zero_yaw)
        linear = cfg.scale_linear_turbo if joy.turbo else cfg.scale_linear
        angular = cfg.scale_angular_turbo if joy.turbo else cfg.scale_angular
        return TickResult((forward * linear, left * linear, ccw * angular), Status.DRIVING)


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        param = self._param
        self._cfg = TeleopConfig(
            axis_vx=param('axis_vx', 4, 'Joy axis of forward speed (4 = right stick Y, up = +1).'),
            axis_vy=param('axis_vy', 3, 'Joy axis of left speed (3 = right stick X, left = +1).'),
            axis_wz=param('axis_wz', 0, 'Joy axis of CCW rotation (0 = left stick X, left = +1).'),
            deadman_axis=param(
                'deadman_axis', 2,
                'Analogue trigger that must be held to drive (2 = LT, 5 = RT, -1 = off). '
                'joy_node reports triggers as +1.0 released and -1.0 fully pressed.'),
            deadman_axis_threshold=param(
                'deadman_axis_threshold', 0.6,
                'How far deadman_axis must be pressed, 0.0 = released .. 1.0 = fully. '
                'Must be > 0.5 so that an unread or half-pressed trigger cannot engage it.'),
            deadman_button=param(
                'deadman_button', -1,
                'Additional button that also acts as the deadman (4 = LB, -1 = off).'),
            require_deadman=param(
                'require_deadman', True,
                'true: drive only while the deadman is held. '
                'false: a deflected stick enables teleop.'),
            turbo_button=param(
                'turbo_button', 5, 'Button selecting the turbo scales (5 = RB, -1 = off).'),
            scale_linear=param('scale_linear', 0.30, 'Top linear speed [m/s].'),
            scale_angular=param('scale_angular', 0.50, 'Top angular speed [rad/s].'),
            scale_linear_turbo=param(
                'scale_linear_turbo', 0.80, 'Top linear speed with turbo held [m/s].'),
            scale_angular_turbo=param(
                'scale_angular_turbo', 1.20, 'Top angular speed with turbo held [rad/s].'),
            stick_deadband=param(
                'stick_deadband', 0.10,
                'Per-axis deadband, output rescaled to 0..1 as in final.py.'),
            field_centric=param(
                'field_centric', True,
                'true: stick directions are relative to field_zero_yaw (needs /odom). '
                'false: robot-centric.'),
            reseed_button=param(
                'reseed_button', 0,
                'Button storing the current odom yaw as field zero (0 = A, -1 = off).'),
            estop_button=param('estop_button', 1, 'Button latching /e_stop true (1 = B).'),
            estop_release_button=param(
                'estop_release_button', 7,
                'Button releasing /e_stop (7 = start); only with the deadman released and the '
                'sticks centred.'),
            cancel_nav_button=param(
                'cancel_nav_button', 2, 'Button publishing /nav_cancel (2 = X, -1 = off).'),
            estop_cancels_nav=param(
                'estop_cancels_nav', True,
                'Also publish /nav_cancel when the e-stop latches, so that releasing it cannot '
                'resume a goal.'),
            joy_timeout=param(
                'joy_timeout', 0.5,
                '/joy older than this [s] counts as deadman released (pad unplugged).'),
            odom_timeout=param(
                'odom_timeout', 0.5,
                'Field-centric only: /odom older than this [s] gives a zero command.'),
        )
        publish_rate = checked_publish_rate(
            param('publish_rate', 20.0, 'Rate of /cmd_vel_teleop while enabled [Hz].'))

        self._logic = TeleopLogic(self._cfg)
        self._was_enabled = False

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 1)
        self._estop_pub = self.create_publisher(Bool, 'e_stop', latched)
        self._cancel_pub = self.create_publisher(Empty, 'nav_cancel', 1)
        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        if self._cfg.field_centric:
            # Only the newest yaw matters, so best effort is enough, and it matches a
            # publisher of either reliability.
            self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        # This timer IS the /joy watchdog. Steady clock: the node clock stalls with use_sim_time
        # and follows wall-clock steps.
        self.create_timer(1.0 / publish_rate, self._on_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._estop_pub.publish(Bool(data=False))
        self._log_startup(publish_rate)

    def _param(self, name, default, description):
        # Read once at startup; read_only makes a later `ros2 param set` fail loudly instead of
        # appearing to work. Floats are dynamically typed so that `30` in a YAML file is
        # accepted for 30.0 instead of aborting the launch with a type error.
        is_float = isinstance(default, float)
        descriptor = ParameterDescriptor(
            description=description, read_only=True, dynamic_typing=is_float)
        value = self.declare_parameter(name, default, descriptor).value
        if not is_float:
            return value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'parameter {name} must be a number (got {value!r})')
        return float(value)

    def _log_startup(self, publish_rate: float) -> None:
        cfg = self._cfg
        frame = 'FIELD-centric (odom yaw)' if cfg.field_centric else 'ROBOT-centric'
        if cfg.require_deadman:
            sources = []
            if cfg.deadman_axis >= 0:
                sources.append(f'axis {cfg.deadman_axis} past '
                               f'{cfg.deadman_axis_threshold:.2f}')
            if cfg.deadman_button >= 0:
                sources.append(f'button {cfg.deadman_button}')
            enable = 'deadman = ' + ' or '.join(sources)
        else:
            enable = 'NO deadman (a deflected stick enables teleop)'
        self.get_logger().info(
            f'joy_teleop up: {frame}, {enable}, '
            f'{cfg.scale_linear:.2f} m/s {cfg.scale_angular:.2f} rad/s '
            f'(turbo {cfg.scale_linear_turbo:.2f} / {cfg.scale_angular_turbo:.2f}), '
            f'{publish_rate:.0f} Hz, joy_timeout {cfg.joy_timeout:.2f} s. '
            'Published /e_stop = false.')
        for name in ('axis_vx', 'axis_vy', 'axis_wz'):
            if getattr(cfg, name) in TRIGGER_AXES:
                self.get_logger().warn(
                    f'{name}={getattr(cfg, name)} is a trigger on the Xbox pad: it rests at +1.0, '
                    'which commands FULL speed as soon as teleop is enabled.')

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._logic.on_odom(yaw_from_quaternion(q.x, q.y, q.z, q.w), time.monotonic())

    def _on_joy(self, msg: Joy) -> None:
        log = self.get_logger()
        events = self._logic.on_joy(msg.axes, msg.buttons, time.monotonic())
        if not events.layout_ok:
            log.warn(
                f'Ignoring /joy with {len(msg.axes)} axes / {len(msg.buttons)} buttons (need >= '
                f'{self._cfg.min_axes} / {self._cfg.min_buttons}): not the Xbox pad. Start '
                'joy_node with device_name "Xbox 360 Controller" (SDL index 0 is the SpaceMouse).',
                throttle_duration_sec=5.0)
        if events.estop_changed:
            # Published from the callback, not the timer: an e-stop must not wait for a tick.
            self._estop_pub.publish(Bool(data=self._logic.e_stop))
            if self._logic.e_stop:
                log.error('E-STOP LATCHED. To release: let go of the deadman, centre the sticks, '
                          'press start.')
            else:
                log.warn('E-stop released.')
        if events.release_refused:
            log.warn(f'E-stop NOT released: {events.release_refused}.')
        if events.cancel_nav:
            self._cancel_pub.publish(Empty())
            log.info('Published /nav_cancel.')
        if events.reseeded:
            zero_deg = math.degrees(self._logic.field_zero_yaw)
            log.info(f'Field heading re-zeroed at odom yaw {zero_deg:.1f} deg.')
        if events.reseed_refused:
            log.warn('Field heading NOT re-zeroed: no fresh /odom.')
        if events.deadman_released:
            self._on_timer()

    def _on_timer(self) -> None:
        result = self._logic.tick(time.monotonic())
        if result.command is not None:
            self._publish_command(result.command)
        log = self.get_logger()
        if result.status is Status.ODOM_STALE:
            log.warn('Field-centric teleop without fresh /odom: commanding zero.',
                     throttle_duration_sec=2.0)
        enabled = result.status not in (Status.SILENT, Status.RELEASED)
        if enabled and not self._was_enabled:
            log.info('Teleop enabled.')
        if result.status is Status.RELEASED:
            log.info(f'Teleop released ({result.detail}): sent one zero Twist, now silent.')
        self._was_enabled = enabled

    def _publish_command(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command
        self._cmd_pub.publish(msg)

    def publish_final_zero(self) -> None:
        """On shutdown while driving: stop now instead of after the mux teleop_timeout."""
        if self._logic.active:
            self._publish_command(ZERO)


def main(args=None):
    # rclpy's own signal handler shuts the context down before `finally` runs, which would make
    # the final zero Twist impossible to publish. Take SIGINT/SIGTERM as KeyboardInterrupt.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, signal.default_int_handler)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second Ctrl-C must not abort the final stop.
        for signum in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signum, signal.SIG_IGN)
        node.publish_final_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

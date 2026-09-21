"""Priority mux: /cmd_vel_teleop + /cmd_vel_nav -> /cmd_vel (the only input of the base).

Every timer tick:  e-stop latched -> zero ; teleop fresh -> teleop ; nav fresh and teleop not
heard for teleop_hold -> nav ; otherwise zero.

Safety reasoning:
* A human on the pad always beats Nav2, and Nav2 stays locked out for teleop_hold after the
  last teleop message so that letting go of the deadman never hands the robot straight back to
  a planner that is still running.
* Twist carries no stamp, so freshness is judged by ARRIVAL time on a monotonic clock. A source
  that stops publishing (crashed node, unplugged pad) is dropped after its timeout.
* When nothing is active the mux publishes zeros for one more second and then goes SILENT, so
  that the bridge's own cmd_vel watchdog disarms the drive instead of being fed zeros forever.
* Non-finite input is replaced by a full stop instead of being forwarded.
* A latched /e_stop reaches a (re)started mux only after DDS discovery, LATER than the first
  /cmd_vel_nav messages (measured: 40 ms of nav commands forwarded under a latched e-stop).
  Nothing is forwarded until the e-stop state is known, or until estop_wait has passed without
  any /e_stop publisher (joy:=false).
* The timer runs on the steady clock: neither use_sim_time nor a wall-clock step can stall it.

All decision logic lives in the ROS-free MuxLogic so that it is unit-testable.
"""

from dataclasses import dataclass
import math
import signal
import time
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

Command = Tuple[float, float, float]  # (vx, vy, wz): robot frame, REP-103
ZERO: Command = (0.0, 0.0, 0.0)

ESTOP = 'estop'
TELEOP = 'teleop'
NAV = 'nav'
IDLE = 'idle'

MAX_SOURCE_TIMEOUT = 5.0  # [s] upper limit of teleop_timeout / nav_timeout
RATE_LIMITS = (5.0, 200.0)  # [Hz]
MODULE_RADIUS = 0.359       # [m] swerve module distance from the base centre
MAX_NAV_MIN_SPEED = 0.25    # [m/s] a larger floor would be a jump, not stiction compensation


@dataclass(frozen=True)
class MuxConfig:
    """Parameters of MuxLogic (defaults = interface contract). All in seconds."""

    teleop_timeout: float = 0.5
    nav_timeout: float = 0.5
    teleop_hold: float = 1.0
    # How long zeros are still published after the last active tick before going silent.
    idle_zero_period: float = 1.0
    # How long after the first tick an unknown e-stop state (no /e_stop message yet) blocks
    # every source. Covers DDS discovery of a latched /e_stop after a mux (re)start; longer
    # than the 3 s participant announcement period of Fast DDS in case the initial burst is
    # lost. Only costs anything when nobody publishes /e_stop at all (joy:=false).
    estop_wait: float = 5.0
    # Stiction compensation for NAV commands only (teleop keeps its fine control).  The real
    # drive does not move below some module speed (drive k_s = 0), and a controller tapers its
    # output near the goal: the robot stalled just short of the tolerance with the azimuths
    # chasing noise.  "Module speed" = |v| + MODULE_RADIUS * |wz|.  A nav command between
    # nav_zero_below and nav_min_speed is scaled UP to nav_min_speed (direction unchanged);
    # below nav_zero_below it is noise and becomes a true zero.  nav_min_speed 0.0 disables.
    nav_min_speed: float = 0.0
    nav_zero_below: float = 0.0

    def __post_init__(self):
        # The mux REPEATS the last command of a source until it times out, which hides a dead
        # source from the bridge watchdog: an unbounded (inf / huge) timeout is a runaway.
        for name in ('teleop_timeout', 'nav_timeout'):
            if not 0.0 < getattr(self, name) <= MAX_SOURCE_TIMEOUT:
                raise ValueError(f'{name} must be in (0, {MAX_SOURCE_TIMEOUT}] s '
                                 f'(got {getattr(self, name)})')
        if not 0.0 <= self.nav_zero_below <= self.nav_min_speed <= MAX_NAV_MIN_SPEED:
            raise ValueError(f'need 0 <= nav_zero_below <= nav_min_speed <= {MAX_NAV_MIN_SPEED} '
                             f'(got {self.nav_zero_below}, {self.nav_min_speed})')
        for name in ('teleop_hold', 'idle_zero_period', 'estop_wait'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f'{name} must be finite and >= 0 (got {value})')


@dataclass(frozen=True)
class MuxOutput:
    source: str                 # ESTOP | TELEOP | NAV | IDLE
    command: Optional[Command]  # None = publish nothing this tick


def checked_rate(rate: float) -> float:
    """Return the rate if it is usable, raise ValueError otherwise."""
    # Too slow and the bridge watchdog (cmd_timeout 0.3 s) disarms the drive between two
    # /cmd_vel messages; inf would be a busy loop.
    low, high = RATE_LIMITS
    if not low <= rate <= high:
        raise ValueError(f'rate_hz must be in [{low}, {high}] Hz (got {rate})')
    return rate


def sanitise(command: Command) -> Tuple[Command, bool]:
    """Return (command, True), or (ZERO, False) when any component is NaN / inf."""
    if all(math.isfinite(v) for v in command):
        return command, True
    return ZERO, False


def lift_to_min_speed(command: Command, min_speed: float, zero_below: float) -> Command:
    """Stiction compensation: see MuxConfig.nav_min_speed.  Never changes the direction."""
    speed = math.hypot(command[0], command[1]) + MODULE_RADIUS * abs(command[2])
    if min_speed <= 0.0 or speed >= min_speed:
        return command
    if speed <= zero_below or speed <= 0.0:
        return ZERO
    gain = min_speed / speed
    return (command[0] * gain, command[1] * gain, command[2] * gain)


class _Source:
    """Latest command of one input and when it arrived."""

    def __init__(self, timeout: float):
        self._timeout = timeout
        self.command: Command = ZERO
        self.stamp: Optional[float] = None

    def update(self, command: Command, now: float) -> bool:
        self.command, valid = sanitise(command)
        self.stamp = now
        return valid

    def age(self, now: float) -> float:
        return math.inf if self.stamp is None else now - self.stamp

    def fresh(self, now: float) -> bool:
        return self.age(now) <= self._timeout


class MuxLogic:
    """ROS-free mux state machine. All times are monotonic seconds supplied by the caller."""

    def __init__(self, config: MuxConfig):
        self._cfg = config
        self._teleop = _Source(config.teleop_timeout)
        self._nav = _Source(config.nav_timeout)
        self._e_stop: Optional[bool] = None  # None = no /e_stop message received yet
        self._first_tick: Optional[float] = None
        self._last_active: Optional[float] = None

    @property
    def e_stop_known(self) -> bool:
        return self._e_stop is not None

    def on_teleop(self, command: Command, now: float) -> bool:
        """Store a teleop command; False when it was non-finite (stored as a stop)."""
        return self._teleop.update(command, now)

    def on_nav(self, command: Command, now: float) -> bool:
        """Store a nav command; False when it was non-finite (stored as a stop)."""
        command, valid = sanitise(command)
        command = lift_to_min_speed(command, self._cfg.nav_min_speed, self._cfg.nav_zero_below)
        return self._nav.update(command, now) and valid

    def set_e_stop(self, stopped: bool) -> None:
        self._e_stop = stopped

    def tick(self, now: float) -> MuxOutput:
        if self._first_tick is None:
            self._first_tick = now
        source, command = self._select(now)
        if source != IDLE:
            self._last_active = now
            return MuxOutput(source, command)
        if self._last_active is not None and now - self._last_active <= self._cfg.idle_zero_period:
            return MuxOutput(IDLE, ZERO)
        return MuxOutput(IDLE, None)

    def _select(self, now: float) -> Tuple[str, Command]:
        if self._e_stop:
            return ESTOP, ZERO
        if self._e_stop is None and now - self._first_tick < self._cfg.estop_wait:
            # A latched e-stop may still be on its way (DDS discovery): forward nothing yet.
            return IDLE, ZERO
        if self._teleop.fresh(now):
            return TELEOP, self._teleop.command
        if self._nav.fresh(now) and self._teleop.age(now) >= self._cfg.teleop_hold:
            return NAV, self._nav.command
        return IDLE, ZERO


class CmdVelMuxNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')
        config = MuxConfig(
            teleop_timeout=self._param(
                'teleop_timeout', 0.5, '/cmd_vel_teleop older than this [s] is dropped.'),
            nav_timeout=self._param(
                'nav_timeout', 0.5, '/cmd_vel_nav older than this [s] is dropped.'),
            teleop_hold=self._param(
                'teleop_hold', 1.0,
                'Nav stays locked out this long [s] after the last teleop message.'),
            nav_min_speed=self._param(
                'nav_min_speed', 0.0,
                'Nav commands with a module speed (|v| + 0.359 |wz|) below this [m/s] are scaled '
                f'up to it: the drive does not move slower. 0.0 disables, max {MAX_NAV_MIN_SPEED}.'),
            nav_zero_below=self._param(
                'nav_zero_below', 0.0,
                'Nav commands with a module speed below this [m/s] are noise and become zero.'),
        )
        rate_hz = checked_rate(self._param(
            'rate_hz', 30.0, 'Output rate of /cmd_vel while a source is active [Hz].'))

        self._logic = MuxLogic(config)
        self._source = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        # Latched so that a late subscriber (rqt, a test) sees the current source right away.
        self._source_pub = self.create_publisher(String, '~/active_source', latched)
        self.create_subscription(Twist, 'cmd_vel_teleop', self._on_teleop, 1)
        self.create_subscription(Twist, 'cmd_vel_nav', self._on_nav, 1)
        # Transient local to receive an e-stop that was latched before this node started.
        self.create_subscription(Bool, 'e_stop', self._on_e_stop, latched)
        # Steady clock: the node clock stalls with use_sim_time and follows wall-clock steps.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate_hz, self._on_timer, clock=steady)
        self._estop_wait_timer = self.create_timer(
            config.estop_wait, self._on_estop_wait_over, clock=steady)

        self.get_logger().info(
            f'cmd_vel_mux up: estop > teleop (timeout {config.teleop_timeout:.2f} s) > nav '
            f'(timeout {config.nav_timeout:.2f} s, locked out {config.teleop_hold:.2f} s after '
            f'teleop), {rate_hz:.0f} Hz, silent {config.idle_zero_period:.1f} s after the last '
            f'active source. Forwarding nothing until /e_stop is known (max '
            f'{config.estop_wait:.1f} s).')

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

    def _on_teleop(self, msg: Twist) -> None:
        self._store(self._logic.on_teleop, msg, 'cmd_vel_teleop')

    def _on_nav(self, msg: Twist) -> None:
        self._store(self._logic.on_nav, msg, 'cmd_vel_nav')

    def _store(self, sink, msg: Twist, topic: str) -> None:
        # Planar base: only vx, vy and wz exist; everything else in the Twist is dropped.
        if not sink((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic()):
            self.get_logger().error(
                f'Non-finite Twist on {topic}: treating it as a STOP.', throttle_duration_sec=2.0)

    def _on_e_stop(self, msg: Bool) -> None:
        self._logic.set_e_stop(msg.data)
        self._on_timer()  # act on an e-stop now, not up to one timer period later

    def _on_estop_wait_over(self) -> None:
        self._estop_wait_timer.cancel()
        if not self._logic.e_stop_known:
            self.get_logger().warn(
                'No /e_stop publisher found: assuming NOT stopped. The pad e-stop does not exist '
                '(joy_teleop not running?).')

    def _on_timer(self) -> None:
        output = self._logic.tick(time.monotonic())
        if output.command is not None:
            self._publish_command(output.command)
        if output.source != self._source:
            self._source = output.source
            self._source_pub.publish(String(data=output.source))
            self.get_logger().info(f'Active source: {output.source}')

    def _publish_command(self, command: Command) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command
        self._cmd_pub.publish(msg)

    def publish_final_zero(self) -> None:
        """On shutdown: leave a stop as the last /cmd_vel instead of a stale velocity."""
        self._publish_command(ZERO)


def main(args=None):
    # rclpy's own signal handler shuts the context down before `finally` runs, which would make
    # the final zero Twist impossible to publish. Take SIGINT/SIGTERM as KeyboardInterrupt.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, signal.default_int_handler)
    node = CmdVelMuxNode()
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

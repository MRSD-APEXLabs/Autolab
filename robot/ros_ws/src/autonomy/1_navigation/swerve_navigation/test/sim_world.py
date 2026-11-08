"""sim_world: hardware-free 2D simulator for tests and demos (never used on the robot).

Stands in for the VLP-16 + cloud_to_scan (and optionally for swerve_bridge) so that slam_toolbox,
AMCL and Nav2 can be exercised without any hardware:

  * ray-casts an occupancy map (map_server yaml + PGM) and publishes /scan and /obstacle_points
    with exactly the frame, beam layout, range limits and QoS that cloud_to_scan produces;
  * simulate_base:=true  -> integrates /cmd_vel kinematically and publishes /odom + TF
                            odom->base_footprint (the role of swerve_bridge);
    simulate_base:=false -> takes the odom pose from /odom (swerve_bridge in phoenix6-sim mode);
  * publishes ~/ground_truth (the true pose in the map frame) to score localisation against.

World pose = start pose (in the map) composed with the odom pose. The node deliberately does NOT
publish map->odom: estimating that transform is the job of slam_toolbox / AMCL under test.

The simulation itself (map loading, ray-casting, base kinematics) is plain numpy code without any
ROS types, unit-tested in test/test_sim_world.py; the SimWorld node at the bottom only does I/O.
"""
from __future__ import annotations

import array
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
import yaml


# --------------------------------------------------------------------------------------------
# ROS-free core
# --------------------------------------------------------------------------------------------

def wrap_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def compose(self, child: 'Pose2D') -> 'Pose2D':
        """Return self * child: `child` is expressed in the frame of `self`."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * child.x - s * child.y,
                      self.y + s * child.x + c * child.y,
                      wrap_angle(self.yaw + child.yaw))


def read_pgm(path: Path) -> tuple[np.ndarray, int]:
    """Read a binary (P5) PGM. Returns (image[row 0 = top], maxval)."""
    data = Path(path).read_bytes()
    tokens: list[bytes] = []
    pos = 0
    try:
        while len(tokens) < 4:                      # magic, width, height, maxval
            while data[pos:pos + 1].isspace():
                pos += 1
            if data[pos:pos + 1] == b'#':           # comment runs to the end of the line
                pos = data.index(b'\n', pos) + 1
                continue
            end = pos
            while not data[end:end + 1].isspace():
                if end >= len(data):
                    raise ValueError('truncated header')
                end += 1
            tokens.append(data[pos:end])
            pos = end
        magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    except ValueError as exc:
        raise ValueError(f'{path}: not a valid PGM file ({exc})') from exc
    if magic != b'P5':
        raise ValueError(f'{path}: only binary PGM (P5) maps are supported, got {magic!r}')
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise ValueError(f'{path}: bad PGM header (size {width} x {height}, maxval {maxval})')
    pos += 1                                        # exactly one whitespace byte ends the header
    dtype = np.dtype('u1') if maxval < 256 else np.dtype('>u2')
    if len(data) - pos < width * height * dtype.itemsize:
        raise ValueError(f'{path}: pixel data is shorter than {width} x {height}')
    image = np.frombuffer(data, dtype=dtype, count=width * height, offset=pos)
    return image.reshape(height, width), maxval


class OccupancyMap:
    """Boolean occupancy grid in map coordinates. `occupied[row, col]`, row 0 = MIN y."""

    def __init__(self, occupied: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float) -> None:
        if occupied.ndim != 2 or resolution <= 0.0:
            raise ValueError('occupied must be a 2D array and resolution must be > 0')
        self.occupied = np.ascontiguousarray(occupied, dtype=bool)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)             # map coordinates of the lower-left corner
        self.origin_y = float(origin_y)

    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'OccupancyMap':
        """Load a map_server style yaml (image, resolution, origin, negate, thresholds, mode)."""
        yaml_file = Path(yaml_path).expanduser()
        with open(yaml_file, 'r') as f:
            try:
                meta = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                raise ValueError(f'{yaml_file}: not valid YAML ({exc})') from exc
        # Every problem with the file surfaces as a ValueError, which main() reports as one
        # clear FATAL line instead of a traceback.
        if not isinstance(meta, dict) or not {'image', 'resolution', 'origin'} <= set(meta):
            raise ValueError(
                f'{yaml_file}: not a map_server yaml (needs image, resolution, origin)')
        mode = str(meta.get('mode', 'trinary'))
        if mode not in ('trinary', 'scale'):
            raise ValueError(f"{yaml_file}: map mode '{mode}' is not supported (trinary | scale)")
        origin = meta['origin']
        if not isinstance(origin, (list, tuple)) or len(origin) != 3:
            raise ValueError(f'{yaml_file}: origin must be [x, y, yaw]')
        if abs(float(origin[2])) > 1e-9:
            raise ValueError(f'{yaml_file}: a rotated map origin (yaw != 0) is not supported')
        image_path = Path(meta['image'])
        if not image_path.is_absolute():
            image_path = yaml_file.parent / image_path
        image, maxval = read_pgm(image_path)
        # Same rule as nav2_map_server: dark = occupied unless negate. Only cells above
        # occupied_thresh stop a ray; free AND unknown cells are transparent.
        darkness = image.astype(np.float32) / maxval
        if not int(meta.get('negate', 0)):
            darkness = 1.0 - darkness
        occupied = darkness > float(meta.get('occupied_thresh', 0.65))
        # The image's first row is the TOP of the map (max y): flip so that row 0 = origin y.
        return cls(occupied[::-1], float(meta['resolution']), origin[0], origin[1])

    @property
    def width(self) -> int:
        return self.occupied.shape[1]

    @property
    def height(self) -> int:
        return self.occupied.shape[0]

    @property
    def diagonal(self) -> float:
        return math.hypot(self.width, self.height) * self.resolution

    def occupied_at(self, x, y) -> np.ndarray:
        """Vectorised lookup in map coordinates; anything outside the image is not occupied."""
        x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        col = np.floor((x - self.origin_x) / self.resolution).astype(int)
        row = np.floor((y - self.origin_y) / self.resolution).astype(int)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        result = np.zeros(col.shape, dtype=bool)
        result[inside] = self.occupied[row[inside], col[inside]]
        return result


@dataclass(frozen=True)
class ScanSpec:
    """Beam layout of /scan. Must stay identical to cloud_to_scan (contract section 4)."""

    angle_min_deg: float = -62.0
    angle_max_deg: float = 154.0
    angle_increment_deg: float = 0.5
    range_min: float = 0.55
    range_max: float = 25.0

    def __post_init__(self) -> None:
        if self.angle_increment_deg <= 0.0 or self.angle_max_deg <= self.angle_min_deg:
            raise ValueError('need angle_increment_deg > 0 and angle_max_deg > angle_min_deg')
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError('need 0 <= range_min < range_max')

    @property
    def num_beams(self) -> int:
        return round((self.angle_max_deg - self.angle_min_deg) / self.angle_increment_deg) + 1

    @property
    def angle_min(self) -> float:
        return math.radians(self.angle_min_deg)

    @property
    def angle_increment(self) -> float:
        return math.radians(self.angle_increment_deg)

    @property
    def angle_max(self) -> float:
        return self.angle_min + (self.num_beams - 1) * self.angle_increment

    def angles(self) -> np.ndarray:
        return self.angle_min + np.arange(self.num_beams) * self.angle_increment


class SimLidar:
    """Vectorised ray-march of all beams through an OccupancyMap at half-cell steps."""

    def __init__(self, grid: OccupancyMap, spec: ScanSpec, noise_std: float = 0.0,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.spec = spec
        self._noise_std = float(noise_std)
        self._rng = rng if rng is not None else np.random.default_rng()
        self._resolution = grid.resolution
        self._origin = (grid.origin_x, grid.origin_y)
        # A one-cell free border lets out-of-map samples be clamped onto it instead of being
        # masked: one gather per sample, no bounds test.
        self._padded = np.zeros((grid.height + 2, grid.width + 2), dtype=bool)
        self._padded[1:-1, 1:-1] = grid.occupied
        self._stride = grid.width + 2
        self._max_col = np.float32(grid.width + 1)
        self._max_row = np.float32(grid.height + 1)
        # Nothing can be hit further away than the map diagonal, so do not march beyond it.
        self._step = grid.resolution / 2.0
        reach = min(spec.range_max, grid.diagonal)
        steps = np.arange(1, int(math.ceil(reach / self._step)) + 1)
        self._t = steps * self._step                                        # [m]
        self._t_cells = (self._t / grid.resolution).astype(np.float32)[:, None]
        self._angles = spec.angles()

    def true_ranges(self, lidar: Pose2D) -> np.ndarray:
        """Noise-free distance to the first occupied cell per beam (+inf if none in reach)."""
        heading = self._angles + lidar.yaw
        # sample positions in (padded) cell units, shape (steps, beams)
        col0 = np.float32((lidar.x - self._origin[0]) / self._resolution + 1.0)
        row0 = np.float32((lidar.y - self._origin[1]) / self._resolution + 1.0)
        cols = col0 + self._t_cells * np.cos(heading).astype(np.float32)
        rows = row0 + self._t_cells * np.sin(heading).astype(np.float32)
        np.clip(cols, 0.0, self._max_col, out=cols)
        np.clip(rows, 0.0, self._max_row, out=rows)
        flat = rows.astype(np.int32) * self._stride + cols.astype(np.int32)
        hit = self._padded.ravel()[flat]
        first = hit.argmax(axis=0)
        # The surface lies between the last free and the first occupied sample: use the middle.
        ranges = self._t[first] - self._step / 2.0
        ranges[~hit.any(axis=0)] = np.inf
        return ranges

    def scan(self, lidar: Pose2D) -> np.ndarray:
        """float32 ranges as the real pipeline reports them: noisy, +inf outside the limits."""
        ranges = self.true_ranges(lidar)
        if self._noise_std > 0.0:
            ranges = ranges + self._rng.normal(0.0, self._noise_std, ranges.shape)
        # The VLP-16 gives NO return for anything closer than ~0.5 m (not a short one), and
        # cloud_to_scan drops everything outside [range_min, range_max]: both end up as +inf.
        ranges[(ranges < self.spec.range_min) | (ranges > self.spec.range_max)] = np.inf
        return ranges.astype(np.float32)


def scan_to_points(spec: ScanSpec, ranges: np.ndarray, max_range: float) -> np.ndarray:
    """Hit points (M, 3) float32 in the lidar frame (z = 0) for beams with range <= max_range."""
    valid = np.isfinite(ranges) & (ranges <= max_range)
    r = ranges[valid].astype(np.float64)
    a = spec.angles()[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r))).astype(np.float32)


def footprint_collides(grid: OccupancyMap, pose: Pose2D, half_width: float) -> bool:
    """Return True if the outline of the square footprint at `pose` touches an occupied cell."""
    n = max(2, int(math.ceil(2.0 * half_width / (grid.resolution / 2.0))) + 1)
    side = np.linspace(-half_width, half_width, n)
    edge = np.full(n, half_width)
    bx = np.concatenate((side, side, edge, -edge))
    by = np.concatenate((edge, -edge, side, side))
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return bool(grid.occupied_at(pose.x + c * bx - s * by, pose.y + s * bx + c * by).any())


def integrate_twist(pose: Pose2D, vx: float, vy: float, wz: float, dt: float) -> Pose2D:
    """Exact integration of a constant BODY-frame twist over dt (SE(2) exponential map)."""
    theta = wz * dt
    if abs(theta) < 1e-6:
        a, b = dt, 0.5 * dt * theta                 # limits of sin(th)/wz and (1-cos(th))/wz
    else:
        a, b = math.sin(theta) / wz, (1.0 - math.cos(theta)) / wz
    return pose.compose(Pose2D(vx * a - vy * b, vx * b + vy * a, theta))


@dataclass(frozen=True)
class BaseLimits:
    """Defaults mirror swerve_bridge so the simulated base behaves like the real one."""

    max_linear_speed: float = 1.0       # m/s, on the norm of (vx, vy)
    max_angular_speed: float = 1.5      # rad/s
    max_linear_accel: float = 2.0       # m/s^2
    max_angular_accel: float = 4.0      # rad/s^2
    cmd_timeout: float = 0.3            # s without /cmd_vel before the base stops
    max_dt: float = 0.1                 # s; a stalled process must not teleport the robot
    # Static friction of the real drivetrain (drive k_s = 0): a command whose module speed
    # |v| + 0.359 * |wz| is below this does not move the base at all.  0.0 = ideal base.
    dead_speed: float = 0.0


class KinematicBase:
    """Holonomic base: robot-frame (vx, vy, wz) commands -> pose in the odom frame.

    `is_blocked(pose)` is an optional collision predicate: a step that would move a free robot
    into an obstacle is refused and the base stops dead, so a planner bug shows up as a stuck
    robot instead of a robot silently driving through walls.
    """

    def __init__(self, limits: BaseLimits = BaseLimits(),
                 is_blocked: Optional[Callable[[Pose2D], bool]] = None) -> None:
        self.limits = limits
        self.pose = Pose2D()
        self.velocity = (0.0, 0.0, 0.0)             # body frame vx, vy, wz
        self._is_blocked = is_blocked
        self._command = (0.0, 0.0, 0.0)
        self._command_time = -math.inf
        self._last_step: Optional[float] = None

    def set_command(self, vx: float, vy: float, wz: float, now: float) -> None:
        lim = self.limits
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            # NaN passes every clamp below and would poison the pose for good: a garbage
            # command is a stop command.
            vx = vy = wz = 0.0
        speed = math.hypot(vx, vy)
        if speed > lim.max_linear_speed:
            vx, vy = vx * lim.max_linear_speed / speed, vy * lim.max_linear_speed / speed
        wz = min(max(wz, -lim.max_angular_speed), lim.max_angular_speed)
        self._command = (vx, vy, wz)
        self._command_time = now

    def step(self, now: float) -> None:
        """Advance the simulation to time `now` (seconds, any monotonic origin)."""
        if self._last_step is None:
            self._last_step = now
            return
        dt = min(now - self._last_step, self.limits.max_dt)
        self._last_step = now       # also when the clock stepped BACK: resync instead of freezing
        if dt <= 0.0:
            return
        lim = self.limits
        # a command stamped in the future (clock stepped back) is stale, not eternally fresh
        fresh = 0.0 <= (now - self._command_time) <= lim.cmd_timeout
        target = self._command if fresh else (0.0, 0.0, 0.0)
        if math.hypot(target[0], target[1]) + 0.359 * abs(target[2]) < lim.dead_speed:
            target = (0.0, 0.0, 0.0)

        # Unlike swerve_bridge (which slews its COMMAND on speed-up only and leaves braking to
        # the drivetrain) the limit applies in both directions here: this IS the drivetrain,
        # and a real base does not stop instantly either.
        vx, vy, wz = self.velocity
        dvx, dvy = target[0] - vx, target[1] - vy
        dv, dv_max = math.hypot(dvx, dvy), lim.max_linear_accel * dt
        if dv > dv_max:
            dvx, dvy = dvx * dv_max / dv, dvy * dv_max / dv
        dw_max = lim.max_angular_accel * dt
        dwz = min(max(target[2] - wz, -dw_max), dw_max)
        new_velocity = (vx + dvx, vy + dvy, wz + dwz)

        # The velocity ramps linearly during the step: integrate its mean.
        candidate = integrate_twist(self.pose, vx + dvx / 2.0, vy + dvy / 2.0, wz + dwz / 2.0, dt)
        if (self._is_blocked is not None and self._is_blocked(candidate)
                and not self._is_blocked(self.pose)):    # already stuck -> allow the escape
            self.velocity = (0.0, 0.0, 0.0)
            return
        self.pose = candidate
        self.velocity = new_velocity


# --------------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------------

def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimWorld(Node):
    STALE_POSE_S = 0.5          # external /odom older than this -> stop publishing scans

    def __init__(self) -> None:
        super().__init__('sim_world')

        def param(name, default, description):
            # Floats are dynamically typed: a launch argument like start_x:=1 arrives as an
            # integer and would otherwise abort the node with a parameter type error.
            is_float = isinstance(default, float)
            value = self.declare_parameter(name, default, ParameterDescriptor(
                description=description, dynamic_typing=is_float)).value
            return float(value) if is_float else value

        map_yaml = param('map_yaml', '', 'map_server yaml of the world to simulate (required)')
        self._start = Pose2D(
            param('start_x', 0.0, 'robot x in the map frame at t0 [m] (odom starts at 0,0,0)'),
            param('start_y', 0.0, 'robot y in the map frame at t0 [m]'),
            param('start_yaw', 0.0, 'robot yaw in the map frame at t0 [rad]'))
        simulate_base = param(
            'simulate_base', True,
            'true: integrate /cmd_vel and publish /odom + TF odom->base_frame; '
            'false: read the odom pose from /odom (swerve_bridge in phoenix6-sim mode)')
        self._map_frame = param('map_frame', 'map', 'frame of ~/ground_truth')
        self._odom_frame = param('odom_frame', 'odom', 'odometry frame (simulate_base)')
        self._base_frame = param('base_frame', 'base_footprint', 'robot base frame')
        scan_frame = param('scan_frame', 'velodyne', 'frame of /scan and /obstacle_points')
        self._lidar_mount = Pose2D(
            param('lidar_x', 0.25, 'lidar x in the base frame [m] (keep equal to the URDF)'),
            param('lidar_y', 0.25, 'lidar y in the base frame [m] (keep equal to the URDF)'),
            param('lidar_yaw', 0.0, 'lidar yaw in the base frame [rad] (keep equal to the URDF)'))
        spec = ScanSpec(
            param('angle_min_deg', -62.0, 'first beam azimuth in the lidar frame [deg]'),
            param('angle_max_deg', 154.0, 'last beam azimuth in the lidar frame [deg]'),
            param('angle_increment_deg', 0.5, 'beam spacing [deg]'),
            param('range_min', 0.55, 'closer hits give no return (+inf) [m]'),
            param('range_max', 25.0, 'farther hits give no return (+inf) [m]'))
        rate_hz = param('rate_hz', 10.0, 'scan rate [Hz]')
        noise_std = param('noise_std', 0.01, 'gaussian range noise, 1 sigma [m]')
        self._publish_obstacles = param('publish_obstacles', True, 'publish /obstacle_points')
        self._obstacle_max_range = param(
            'obstacle_max_range', 6.0, 'max range of the points in /obstacle_points [m]')
        odom_rate_hz = param('odom_rate_hz', 50.0, 'odom + TF rate when simulate_base [Hz]')
        publish_tf = param('publish_tf', True, 'broadcast odom->base_frame (simulate_base)')
        limits = BaseLimits(
            param('max_linear_speed', 1.0, 'clamp on the norm of (vx, vy) [m/s]'),
            param('max_angular_speed', 1.5, 'clamp on |wz| [rad/s]'),
            param('max_linear_accel', 2.0, 'simulated base acceleration limit [m/s^2]'),
            param('max_angular_accel', 4.0, 'simulated base acceleration limit [rad/s^2]'),
            param('cmd_timeout', 0.3, 'the base stops this long after the last /cmd_vel [s]'),
            dead_speed=param('drive_dead_speed', 0.0, 'commands with a module speed below this '
                             'do not move the base (static friction of the real drive) [m/s]'))
        self._half_width = param(
            'footprint_half_width', 0.34, 'half side of the square collision footprint [m]')
        block_on_collision = param(
            'block_on_collision', True,
            'simulate_base: refuse motion into occupied cells (the robot gets stuck instead)')

        if not map_yaml:
            raise ValueError("parameter 'map_yaml' is required")
        if rate_hz <= 0.0 or odom_rate_hz <= 0.0:
            raise ValueError('rate_hz and odom_rate_hz must be > 0')
        self._grid = OccupancyMap.from_yaml(map_yaml)
        self._lidar = SimLidar(self._grid, spec, noise_std)

        # Latest odom pose and the time it is valid at. Scans are stamped with THAT time, not
        # with the (up to one odom period later) publish time: a TF sample with exactly this
        # stamp exists, so consumers never have to extrapolate or blend two poses.
        self._odom_pose: Optional[Pose2D] = None
        self._pose_stamp = None

        # Same QoS as cloud_to_scan: RELIABLE depth 5 (also matches best-effort subscribers).
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', sensor_qos)
        self._cloud_pub = self.create_publisher(PointCloud2, 'obstacle_points', sensor_qos)
        self._truth_pub = self.create_publisher(PoseStamped, '~/ground_truth', 10)
        # Output topics that must not have a second publisher, see _is_muted().
        self._exclusive_topics = ['scan', 'obstacle_points'] + (['odom'] if simulate_base else [])
        self._muted = False

        self._scan_msg = LaserScan(
            angle_min=spec.angle_min, angle_max=spec.angle_max,
            angle_increment=spec.angle_increment, time_increment=0.0, scan_time=1.0 / rate_hz,
            range_min=spec.range_min, range_max=spec.range_max)
        self._scan_msg.header.frame_id = scan_frame
        self._cloud_msg = PointCloud2(
            height=1, is_bigendian=False, is_dense=True, point_step=12,
            fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')])
        self._cloud_msg.header.frame_id = scan_frame

        if simulate_base:
            self._base = KinematicBase(
                limits, self._odom_pose_collides if block_on_collision else None)
            self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
            self._tf_broadcaster = TransformBroadcaster(self) if publish_tf else None
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 1)
            self.create_timer(1.0 / odom_rate_hz, self._step_base)
            base_mode = (f'KINEMATIC base (/cmd_vel -> /odom + TF {self._odom_frame}->'
                         f'{self._base_frame} at {odom_rate_hz:g} Hz)')
        else:
            self.create_subscription(Odometry, 'odom', self._on_odom, 10)
            base_mode = 'EXTERNAL base (pose read from /odom, e.g. swerve_bridge phoenix6-sim)'
        self.create_timer(1.0 / rate_hz, self._publish_scan)

        self.get_logger().info(
            f'sim_world [SIMULATION ONLY, no hardware access], {base_mode}; map {map_yaml} '
            f'({self._grid.width} x {self._grid.height} cells @ {self._grid.resolution:g} m); '
            f'start ({self._start.x:.2f}, {self._start.y:.2f}, '
            f'{math.degrees(self._start.yaw):.1f} deg); /scan {spec.num_beams} beams '
            f'[{spec.angle_min_deg:g}, {spec.angle_max_deg:g}] deg @ {rate_hz:g} Hz in frame '
            f"'{scan_frame}', range noise {noise_std:g} m")
        if footprint_collides(self._grid, self._start, self._half_width):
            self.get_logger().warn('the start pose is IN COLLISION with the map')

    # ---- pose sources ----

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_pose_collides(self, odom_pose: Pose2D) -> bool:
        return footprint_collides(self._grid, self._start.compose(odom_pose), self._half_width)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._base.set_command(msg.linear.x, msg.linear.y, msg.angular.z, self._now_seconds())

    def _step_base(self) -> None:
        now = self.get_clock().now()
        self._base.step(now.nanoseconds * 1e-9)
        pose, (vx, vy, wz) = self._base.pose, self._base.velocity
        qz, qw = math.sin(pose.yaw / 2.0), math.cos(pose.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx              # twist is in the base frame (REP-105)
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        if not self._muted:                         # see _is_muted(): never a second odom source
            self._odom_pub.publish(odom)
            if self._tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header = odom.header
                tf.child_frame_id = self._base_frame
                tf.transform.translation.x = pose.x
                tf.transform.translation.y = pose.y
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self._tf_broadcaster.sendTransform(tf)
        self._set_pose(pose, odom.header.stamp)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._set_pose(Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation)),
                       msg.header.stamp)

    def _set_pose(self, odom_pose: Pose2D, stamp) -> None:
        self._odom_pose, self._pose_stamp = odom_pose, stamp
        world = self._start.compose(odom_pose)
        truth = PoseStamped()
        truth.header.stamp = stamp
        truth.header.frame_id = self._map_frame
        truth.pose.position.x = world.x
        truth.pose.position.y = world.y
        truth.pose.orientation.z = math.sin(world.yaw / 2.0)
        truth.pose.orientation.w = math.cos(world.yaw / 2.0)
        self._truth_pub.publish(truth)
        if footprint_collides(self._grid, world, self._half_width):
            self.get_logger().warn(
                f'COLLISION: the footprint touches an obstacle at ({world.x:.2f}, {world.y:.2f})',
                throttle_duration_sec=2.0)

    # ---- sensor ----

    def _is_muted(self) -> bool:
        """Return True (and stay silent) while another node publishes one of our topics.

        Started by hand next to the real robot stack, sim_world would interleave simulated scans
        (and a second odom->base TF) with the real ones and Nav2 would steer the REAL robot by
        them. The launch files never combine the two; this is the guard for everything else.
        """
        shared = [t for t in self._exclusive_topics if self.count_publishers(t) > 1]
        self._muted = bool(shared)
        if shared:
            self.get_logger().error(
                f'another node also publishes {shared} (real lidar pipeline / swerve_bridge?): '
                'sim_world publishes NOTHING on its sensor and odom topics until it is alone, '
                'so that simulated and real data never mix', throttle_duration_sec=5.0)
        return self._muted

    def _publish_scan(self) -> None:
        if self._is_muted():
            return
        if self._odom_pose is None:
            self.get_logger().warn('no odom pose yet: not publishing scans',
                                   throttle_duration_sec=5.0)
            return
        stamp = self._pose_stamp
        age = self._now_seconds() - (stamp.sec + stamp.nanosec * 1e-9)
        if age > self.STALE_POSE_S:
            self.get_logger().warn(f'/odom is stale ({age:.2f} s): not publishing scans',
                                   throttle_duration_sec=5.0)
            return
        lidar_pose = self._start.compose(self._odom_pose).compose(self._lidar_mount)
        ranges = self._lidar.scan(lidar_pose)

        self._scan_msg.header.stamp = stamp
        # array.array is the zero-check fast path of the generated message setters
        self._scan_msg.ranges = array.array('f', ranges.tobytes())
        self._scan_pub.publish(self._scan_msg)

        if self._publish_obstacles:
            points = scan_to_points(self._lidar.spec, ranges, self._obstacle_max_range)
            self._cloud_msg.header.stamp = stamp
            self._cloud_msg.width = points.shape[0]
            self._cloud_msg.row_step = self._cloud_msg.point_step * points.shape[0]
            self._cloud_msg.data = array.array('B', points.tobytes())
            self._cloud_pub.publish(self._cloud_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = SimWorld()
    except (OSError, ValueError, KeyError) as exc:      # unreadable map / bad parameters
        get_logger('sim_world').fatal(f'cannot start: {exc}')
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGINT/SIGTERM invalidate the context from rclpy's signal thread, possibly while a
        # callback is about to publish ("publisher's context is invalid"). That is a normal
        # shutdown, not a crash; with a live context it is a real error.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

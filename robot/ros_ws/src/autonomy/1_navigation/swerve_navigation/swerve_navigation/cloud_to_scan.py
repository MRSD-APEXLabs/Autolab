"""
VLP-16 /velodyne_points -> /scan (2D slice for SLAM/AMCL) + /obstacle_points (3D obstacles).

Everything happens in the lidar (cloud) frame, so no TF is needed. The lidar sits on the
front-left corner of the robot and ~136 deg of its view is the robot's own body; only the
azimuth window [angle_min, angle_max] is ever let through. A beam without a return is
published as +inf ("unknown"), never as free space: the VLP-16 returns NOTHING for objects
closer than 0.5 m, so silence does not mean the space is empty.

The numeric work lives in the ROS-graph-free `CloudToScanConverter` / `cloud_xyz`
(unit-tested with synthetic points and regression-tested against the user's bag).
"""

import array
from dataclasses import dataclass
import math
import sys
import time
from typing import NamedTuple, Sequence, Tuple

import numpy as np

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

# sensor_msgs/PointField datatype -> numpy type code. Literal values (they are part of the
# message definition and cannot change) so that `cloud_xyz` works on any PointField-like object.
_FLOAT_TYPES = {7: 'f4', 8: 'f8'}

# Tolerance on the azimuth window edges [rad] (0.00006 deg): float32 atan2 of a point lying
# exactly on an edge may land 1 ulp outside. Far smaller than the 2 deg safety margin that the
# default window already keeps from the robot's body.
_EDGE_EPS = 1e-6

# Contract: a cloud must be processed in less than this [s]; slower means /scan lags the robot.
_PROCESSING_BUDGET = 0.015


@dataclass(frozen=True)
class ScanConfig:
    """Parameters of the conversion (same names and defaults as the node parameters)."""

    lidar_height: float = 0.330
    angle_min_deg: float = -62.0
    angle_max_deg: float = 154.0
    angle_increment_deg: float = 0.5
    range_min: float = 0.55
    range_max: float = 25.0
    scan_min_height: float = 0.20
    scan_max_height: float = 0.60
    obstacle_min_height: float = 0.10
    obstacle_max_height: float = 1.20
    obstacle_max_range: float = 6.0
    obstacle_voxel: float = 0.05

    def validate(self) -> None:
        """Raise ValueError with a clear message if the parameter set is unusable."""
        problems = []
        # NaN slips through ordered comparisons only where no bound exists (lidar_height) and
        # inf through every open-ended one; either would silently blank or garble the outputs.
        not_finite = [name for name, value in vars(self).items() if not math.isfinite(value)]
        if not_finite:
            problems.append('not finite: ' + ', '.join(not_finite))
        if not 0.01 <= self.angle_increment_deg <= 10.0:
            problems.append('angle_increment_deg must be in [0.01, 10]')
        if not -180.0 <= self.angle_min_deg < self.angle_max_deg <= 180.0:
            problems.append('need -180 <= angle_min_deg < angle_max_deg <= 180')
        elif self.angle_max_deg - self.angle_min_deg < self.angle_increment_deg:
            problems.append('azimuth window is narrower than one angle_increment_deg')
        # range_min 0 would let (0, 0, 0) "no return" placeholders through as a 0 m beam and as an
        # obstacle at the lidar origin, i.e. inside the robot footprint.
        if not 0.0 < self.range_min < self.range_max:
            problems.append('need 0 < range_min < range_max')
        if not self.range_min < self.obstacle_max_range:
            problems.append('need range_min < obstacle_max_range')
        if not self.scan_min_height < self.scan_max_height:
            problems.append('need scan_min_height < scan_max_height')
        if not self.obstacle_min_height < self.obstacle_max_height:
            problems.append('need obstacle_min_height < obstacle_max_height')
        if not self.obstacle_voxel >= 0.005:
            problems.append('obstacle_voxel must be >= 0.005 m')
        if problems:
            raise ValueError('invalid cloud_to_scan parameters: ' + '; '.join(problems))


class ScanResult(NamedTuple):
    ranges: np.ndarray     # float32 [num_beams], +inf where no return
    obstacles: np.ndarray  # float32 [M, 3] xyz in the cloud frame


class CloudToScanConverter:
    """
    x/y/z arrays in the lidar frame -> LaserScan ranges + voxel-downsampled obstacle points.

    All ranges are HORIZONTAL (sqrt(x^2 + y^2)): that is what a LaserScan in the lidar plane
    means, and with the VLP-16's +-15 deg vertical field of view it differs from the 3D range by
    at most 3.5 %.
    """

    def __init__(self, config: ScanConfig = ScanConfig()):
        config.validate()
        self.config = config

        inc_deg = config.angle_increment_deg
        self.num_beams = int(round((config.angle_max_deg - config.angle_min_deg) / inc_deg)) + 1
        # Scan geometry exactly as consumers see it: LaserScan angles are float32, and slam_toolbox
        # rejects every scan unless, with those float32 values,
        #     round((angle_max - angle_min) / angle_increment) + 1 == len(ranges).
        # Binning uses the same rounded values so beam i really is the direction
        # angle_min + i * angle_increment.
        self.angle_min = float(np.float32(math.radians(config.angle_min_deg)))
        self.angle_increment = float(np.float32(math.radians(inc_deg)))
        self.angle_max = float(
            np.float32(self.angle_min + (self.num_beams - 1) * self.angle_increment))

        # Accepted azimuths. If (max - min) is not a multiple of the increment the last beam may
        # point beyond the requested window; never accept returns from there (robot body).
        self._az_lo = self.angle_min - _EDGE_EPS
        self._az_hi = min(math.radians(config.angle_max_deg), self.angle_max) + _EDGE_EPS

        self._scan_z = (config.scan_min_height - config.lidar_height,
                        config.scan_max_height - config.lidar_height)
        self._obstacle_z = (config.obstacle_min_height - config.lidar_height,
                            config.obstacle_max_height - config.lidar_height)
        self._z_lo = min(self._scan_z[0], self._obstacle_z[0])
        self._z_hi = max(self._scan_z[1], self._obstacle_z[1])
        self._r2_lo = config.range_min ** 2
        self._r2_hi = max(config.range_max, config.obstacle_max_range) ** 2

    def convert(self, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                with_obstacles: bool = True) -> ScanResult:
        """Convert one cloud. NaN/inf points fail every comparison below and drop out."""
        cfg = self.config
        # Garbage coordinates too large for float32 overflow to inf (in the cast or the square)
        # and then drop out like any other inf: intended, so no numpy overflow warning.
        with np.errstate(over='ignore'):
            x = np.ascontiguousarray(x, dtype=np.float32).reshape(-1)
            y = np.ascontiguousarray(y, dtype=np.float32).reshape(-1)
            z = np.ascontiguousarray(z, dtype=np.float32).reshape(-1)
            r2 = x * x + y * y

        # Cheap box/ring tests first so that atan2 only runs on candidate points.
        keep = (r2 >= self._r2_lo) & (r2 <= self._r2_hi) & (z >= self._z_lo) & (z <= self._z_hi)
        x, y, z, r2 = x[keep], y[keep], z[keep], r2[keep]

        azimuth = np.arctan2(y, x)
        keep = (azimuth >= self._az_lo) & (azimuth <= self._az_hi)
        x, y, z, r2, azimuth = x[keep], y[keep], z[keep], r2[keep], azimuth[keep]
        rng = np.sqrt(r2)

        ranges = np.full(self.num_beams, np.inf, dtype=np.float32)
        in_scan = (z >= self._scan_z[0]) & (z <= self._scan_z[1]) & (rng <= cfg.range_max)
        beam = np.rint((azimuth[in_scan] - self.angle_min) / self.angle_increment).astype(np.intp)
        np.minimum.at(ranges, beam, rng[in_scan])

        if with_obstacles:
            is_obstacle = ((z >= self._obstacle_z[0]) & (z <= self._obstacle_z[1])
                           & (rng <= cfg.obstacle_max_range))
            obstacles = voxel_downsample(
                np.column_stack((x[is_obstacle], y[is_obstacle], z[is_obstacle])),
                cfg.obstacle_voxel)
        else:
            obstacles = np.empty((0, 3), dtype=np.float32)
        return ScanResult(ranges, obstacles)


def voxel_downsample(xyz: np.ndarray, voxel: float) -> np.ndarray:
    """
    One point (the centroid) per occupied voxel floor(xyz / voxel). float32 [M, 3].

    The centroid of real returns stays on the measured surface; voxel centres would shift
    obstacles by up to half a costmap cell.
    """
    if xyz.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32)
    index = np.floor(xyz / voxel).astype(np.int64)
    index -= index.min(axis=0)
    span = index.max(axis=0) + 1
    # One scalar key per voxel: unique() on a 1-D int array is ~20x faster than on rows.
    key = (index[:, 0] * span[1] + index[:, 1]) * span[2] + index[:, 2]
    _, inverse, counts = np.unique(key, return_inverse=True, return_counts=True)
    centroids = np.empty((counts.size, 3), dtype=np.float32)
    for axis in range(3):
        centroids[:, axis] = np.bincount(inverse, weights=xyz[:, axis]) / counts
    return centroids


def cloud_xyz(data, fields: Sequence, point_step: int, width: int, height: int = 1,
              row_step: int = 0, is_bigendian: bool = False
              ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Zero-copy views of the x, y, z columns of a PointCloud2 data buffer.

    The dtype is built from the message's own field table with explicit offsets and
    itemsize = point_step, because the velodyne layout is PACKED (22-byte points, `time` f32 at
    offset 18): numpy's default aligned struct layout would read garbage. The returned arrays are
    strided, possibly unaligned views [height * width] into `data`; nothing is copied here.
    Organized clouds (height > 1, optional row padding) are handled through row_step.
    """
    by_name = {f.name: f for f in fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = by_name.get(name)
        if field is None:
            raise ValueError(f"PointCloud2 has no '{name}' field (fields: {sorted(by_name)})")
        if field.datatype not in _FLOAT_TYPES:
            raise ValueError(f"PointCloud2 field '{name}' has non-float datatype {field.datatype}")
        formats.append(('>' if is_bigendian else '<') + _FLOAT_TYPES[field.datatype])
        offsets.append(field.offset)
    try:
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats, 'offsets': offsets,
                          'itemsize': point_step})
    except ValueError as exc:  # a field does not fit inside point_step
        raise ValueError(f'inconsistent PointCloud2 layout: {exc}') from exc

    count = width * height
    if count == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty
    row_step = row_step or width * point_step
    if row_step < width * point_step:
        raise ValueError(f'inconsistent PointCloud2 layout: row_step {row_step} < width {width} '
                         f'* point_step {point_step}')
    needed = (height - 1) * row_step + width * point_step
    if len(data) < needed:
        raise ValueError(f'PointCloud2 data too short: {len(data)} bytes, layout needs {needed} '
                         f'(width {width}, height {height}, point_step {point_step}, '
                         f'row_step {row_step})')
    points = np.ndarray(shape=(height, width), dtype=dtype, buffer=data,
                        strides=(row_step, point_step))
    return points['x'].reshape(-1), points['y'].reshape(-1), points['z'].reshape(-1)


def make_scan_msg(header: Header, converter: CloudToScanConverter, ranges: np.ndarray,
                  scan_time: float) -> LaserScan:
    scan = LaserScan()
    scan.header = header
    scan.angle_min = converter.angle_min
    scan.angle_max = converter.angle_max
    scan.angle_increment = converter.angle_increment
    # The cloud carries no usable per-beam timing after min-per-bin; consumers must not de-skew.
    scan.time_increment = 0.0
    scan.scan_time = scan_time
    scan.range_min = converter.config.range_min
    scan.range_max = converter.config.range_max
    # array.array is the native storage of float32[]: handing one over skips the per-element
    # type check rclpy does for generic sequences.
    scan.ranges = array.array('f', ranges.tobytes())
    return scan


def make_obstacle_msg(header: Header, obstacles: np.ndarray) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = int(obstacles.shape[0])
    cloud.fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate('xyz')]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * cloud.width
    cloud.is_dense = True  # NaN/inf never pass the converter
    cloud.data = array.array('B', obstacles.astype('<f4', copy=False).tobytes())
    return cloud


class CloudToScanNode(Node):

    def __init__(self, **node_kwargs):
        super().__init__('cloud_to_scan', **node_kwargs)

        def declare(name, default, description):
            # Read-only: everything is baked into the converter at startup, so a later
            # `ros2 param set` must be refused rather than accepted and silently ignored
            # (e.g. someone "narrowing" the body window on a running robot).
            return self.declare_parameter(
                name, default,
                ParameterDescriptor(description=description, read_only=True)).value

        defaults = ScanConfig()
        config = ScanConfig(
            lidar_height=declare(
                'lidar_height', defaults.lidar_height,
                'Height of the lidar optical centre above the floor [m].'),
            angle_min_deg=declare(
                'angle_min_deg', defaults.angle_min_deg,
                'Start of the usable azimuth window in the cloud frame [deg]; everything outside '
                'the window is the robot body and is discarded.'),
            angle_max_deg=declare(
                'angle_max_deg', defaults.angle_max_deg,
                'End of the usable azimuth window in the cloud frame [deg].'),
            angle_increment_deg=declare(
                'angle_increment_deg', defaults.angle_increment_deg,
                'Angular resolution of /scan [deg].'),
            range_min=declare(
                'range_min', defaults.range_min,
                'Minimum horizontal range [m] for /scan and /obstacle_points (sensor floor is '
                '0.50 m; closer returns are the robot itself).'),
            range_max=declare(
                'range_max', defaults.range_max, 'Maximum horizontal range of /scan [m].'),
            scan_min_height=declare(
                'scan_min_height', defaults.scan_min_height,
                'Lower edge of the /scan height band, above the floor [m].'),
            scan_max_height=declare(
                'scan_max_height', defaults.scan_max_height,
                'Upper edge of the /scan height band, above the floor [m].'),
            obstacle_min_height=declare(
                'obstacle_min_height', defaults.obstacle_min_height,
                'Lower edge of the /obstacle_points height band, above the floor [m] '
                '(floor noise reaches +0.04 m while driving).'),
            obstacle_max_height=declare(
                'obstacle_max_height', defaults.obstacle_max_height,
                'Upper edge of the /obstacle_points height band, above the floor [m]; keep it '
                'above the robot height so overhangs are seen.'),
            obstacle_max_range=declare(
                'obstacle_max_range', defaults.obstacle_max_range,
                'Maximum horizontal range of /obstacle_points [m].'),
            obstacle_voxel=declare(
                'obstacle_voxel', defaults.obstacle_voxel,
                'Voxel edge used to downsample /obstacle_points [m].'),
        )
        self._scan_frame = declare(
            'scan_frame', '',
            'frame_id written into /scan; empty = copy the frame of the incoming cloud. The data '
            'is NOT transformed, so only name a frame that coincides with the cloud frame.')
        self._scan_time = declare(
            'scan_time', 0.1009, 'LaserScan.scan_time [s] (VLP-16 sweep period at 600 rpm).')
        self._publish_obstacles = declare(
            'publish_obstacles', True, 'Publish /obstacle_points in addition to /scan.')

        self._converter = CloudToScanConverter(config)  # raises ValueError on bad parameters

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', qos)
        self._obstacle_pub = (self.create_publisher(PointCloud2, 'obstacle_points', qos)
                              if self._publish_obstacles else None)
        self._layout_logged = False
        self._sub = self.create_subscription(PointCloud2, 'velodyne_points', self._on_cloud, qos)

        conv = self._converter
        self.get_logger().info(
            f'cloud_to_scan: velodyne_points -> scan ({conv.num_beams} beams, azimuth '
            f'[{config.angle_min_deg:.1f}, {math.degrees(conv.angle_max):.1f}] deg @ '
            f'{config.angle_increment_deg:.2f} deg, range [{config.range_min:.2f}, '
            f'{config.range_max:.1f}] m, height band [{config.scan_min_height:.2f}, '
            f'{config.scan_max_height:.2f}] m above floor, lidar at {config.lidar_height:.3f} m, '
            f"frame '{self._scan_frame or '<cloud frame>'}')")
        if self._publish_obstacles:
            self.get_logger().info(
                f'cloud_to_scan: obstacle_points ENABLED (height band '
                f'[{config.obstacle_min_height:.2f}, {config.obstacle_max_height:.2f}] m, '
                f'range <= {config.obstacle_max_range:.1f} m, '
                f'voxel {config.obstacle_voxel:.3f} m)')
        else:
            self.get_logger().info('cloud_to_scan: obstacle_points DISABLED')

    def _on_cloud(self, msg: PointCloud2) -> None:
        start = time.perf_counter()
        try:
            x, y, z = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.width, msg.height,
                                msg.row_step, msg.is_bigendian)
        except ValueError as exc:
            # Publishing nothing is the safe reaction: the consumers' source timeouts then stop
            # the robot instead of letting it drive on stale or garbled data.
            self.get_logger().error(f'dropping cloud: {exc}', throttle_duration_sec=5.0)
            return
        result = self._converter.convert(x, y, z, with_obstacles=self._publish_obstacles)

        # The cloud stamp is the END of the 0.1 s sweep; keep it (contract) so TF lookups match.
        scan_header = Header(stamp=msg.header.stamp,
                             frame_id=self._scan_frame or msg.header.frame_id)
        try:
            self._scan_pub.publish(
                make_scan_msg(scan_header, self._converter, result.ranges, self._scan_time))
            if self._obstacle_pub is not None:
                self._obstacle_pub.publish(make_obstacle_msg(msg.header, result.obstacles))
        except rclpy_implementation.RCLError:
            # SIGINT/SIGTERM invalidate the context from rclpy's signal thread, possibly while a
            # cloud is mid-callback; publish() then fails. That is a normal shutdown, not a crash
            # (unhandled it ends the node with a traceback and exit code 1 on Ctrl-C).
            if self.context.ok():
                raise
            return

        elapsed = time.perf_counter() - start
        summary = (f"frame '{msg.header.frame_id}', {msg.width} x {msg.height} points, "
                   f'point_step {msg.point_step} -> {int(np.isfinite(result.ranges).sum())} of '
                   f'{self._converter.num_beams} beams valid, {result.obstacles.shape[0]} '
                   f'obstacle points, {elapsed * 1e3:.1f} ms')
        if not self._layout_logged:
            self._layout_logged = True
            self.get_logger().info('first cloud: ' + summary)
        else:  # per-cloud figures: --ros-args --log-level cloud_to_scan:=debug
            self.get_logger().debug('cloud: ' + summary)
        if elapsed > _PROCESSING_BUDGET:
            self.get_logger().warning(
                f'cloud took {elapsed * 1e3:.1f} ms to convert (budget '
                f'{_PROCESSING_BUDGET * 1e3:.0f} ms): /scan is lagging',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CloudToScanNode()
    except (ValueError, InvalidParameterTypeException) as exc:
        # e.g. an inverted window, or an integer given for a float parameter (range_max:=25)
        print(f'[cloud_to_scan] FATAL: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # rclpy's executor loses the same race as _on_cloud now and then: the signal thread
        # invalidates the context between its ok() check and the next rcl call ("failed to
        # initialize wait set"). With a live context it is a real error.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

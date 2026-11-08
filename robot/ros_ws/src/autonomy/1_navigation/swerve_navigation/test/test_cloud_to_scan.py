"""Unit tests of the ROS-graph-free part of cloud_to_scan (synthetic points, no node/topics)."""

import array
import dataclasses
import math
from types import SimpleNamespace
import warnings

import numpy as np
import pytest
from std_msgs.msg import Header

from swerve_navigation.cloud_to_scan import (
    cloud_xyz,
    CloudToScanConverter,
    make_obstacle_msg,
    make_scan_msg,
    ScanConfig,
    voxel_downsample,
)

LIDAR_HEIGHT = 0.330
FLOAT32, FLOAT64, UINT16 = 7, 8, 4  # sensor_msgs/PointField datatype codes

# The packed 22-byte point of velodyne_pointcloud 2.5.1 (`time` f32 at the unaligned offset 18).
VELODYNE_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 18],
    'itemsize': 22})
VELODYNE_FIELDS = [
    SimpleNamespace(name='x', offset=0, datatype=FLOAT32, count=1),
    SimpleNamespace(name='y', offset=4, datatype=FLOAT32, count=1),
    SimpleNamespace(name='z', offset=8, datatype=FLOAT32, count=1),
    SimpleNamespace(name='intensity', offset=12, datatype=FLOAT32, count=1),
    SimpleNamespace(name='ring', offset=16, datatype=UINT16, count=1),
    SimpleNamespace(name='time', offset=18, datatype=FLOAT32, count=1),
]


def polar(azimuth_deg, rng, height=0.33, lidar_height=LIDAR_HEIGHT):
    """Points given by azimuth [deg], horizontal range [m] and height ABOVE THE FLOOR [m]."""
    az, rng, height = np.broadcast_arrays(
        np.radians(np.asarray(azimuth_deg, dtype=np.float64)),
        np.asarray(rng, dtype=np.float64), np.asarray(height, dtype=np.float64))
    return rng * np.cos(az), rng * np.sin(az), height - lidar_height


def scan_of(converter, azimuth_deg, rng, height=0.33):
    return converter.convert(*polar(azimuth_deg, rng, height, converter.config.lidar_height))


def valid_beams(ranges):
    return np.flatnonzero(np.isfinite(ranges)).tolist()


@pytest.fixture
def conv():
    return CloudToScanConverter()


# ---------------------------------------------------------------------------- scan geometry

def test_default_geometry_matches_contract(conv):
    assert conv.num_beams == 433  # round((154 - -62) / 0.5) + 1
    assert conv.angle_min == float(np.float32(math.radians(-62.0)))
    assert conv.angle_increment == float(np.float32(math.radians(0.5)))
    assert conv.angle_max == float(np.float32(conv.angle_min + 432 * conv.angle_increment))
    assert math.degrees(conv.angle_max) == pytest.approx(154.0, abs=1e-4)


def slam_toolbox_expected_readings(angle_min, angle_max, angle_increment, num_ranges):
    """
    Exact re-statement of slam_toolbox 2.8.5 (ref/nav2_jazzy_notes.md section 9).

    laser_utils.cpp:109-121 (float32 message fields):
        angular_range = fabs(angle_max - angle_min)
        is_360 = fabs(angular_range - 2*pi) < (angle_increment - FLT_EPSILON*2*pi)
        if (angular_range > 6.10865 && round(angular_range/angle_increment) + 1 == ranges.size())
            is_360 = false
    Karto.h LaserRangeFinder::Update() (doubles made from those float32 values):
        expected = round((angle_max - angle_min) / angle_increment) + (is_360 ? 0 : 1)
    Karto.cpp Validate() rejects the scan unless ranges.size() == expected.
    """
    f32 = np.float32

    def c_round(value):  # std::round / karto math::Round: halves away from zero, not to even
        return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)

    amin, amax, inc = f32(angle_min), f32(angle_max), f32(angle_increment)
    angular_range = f32(abs(amax - amin))
    is_360 = abs(float(angular_range) - 2.0 * math.pi) < float(inc) - float(
        f32(np.finfo(f32).eps) * f32(2.0 * math.pi))
    if angular_range > 6.10865 and c_round(float(angular_range / inc)) + 1 == num_ranges:
        is_360 = False
    return int(c_round((float(amax) - float(amin)) / float(inc))) + (0 if is_360 else 1)


@pytest.mark.parametrize('inc_deg', [0.1, 0.2, 0.25, 0.4, 0.5, 1.0, 2.0])
@pytest.mark.parametrize('window', [(-62.0, 154.0), (-180.0, 180.0), (-90.0, 90.0),
                                    (-62.0, 154.3), (-179.9, 179.9), (-135.0, 135.0)])
def test_beam_count_satisfies_slam_toolbox_formula(window, inc_deg):
    cfg = ScanConfig(angle_min_deg=window[0], angle_max_deg=window[1],
                     angle_increment_deg=inc_deg)
    conv = CloudToScanConverter(cfg)
    assert conv.num_beams == int(round((window[1] - window[0]) / inc_deg)) + 1

    # Judge what slam_toolbox RECEIVES: the float32 fields of the published message.
    msg = make_scan_msg(Header(frame_id='velodyne'), conv, conv.convert([], [], []).ranges, 0.1)
    wire = [float(np.float32(v)) for v in (msg.angle_min, msg.angle_max, msg.angle_increment)]
    assert wire == [msg.angle_min, msg.angle_max, msg.angle_increment]  # float32-exact already
    assert wire[1] == float(np.float32(wire[0] + (conv.num_beams - 1) * wire[2]))
    assert len(msg.ranges) == conv.num_beams
    assert slam_toolbox_expected_readings(*wire, len(msg.ranges)) == len(msg.ranges)


def test_beam_count_is_constant_whatever_the_input(conv):
    rng = np.random.default_rng(1)
    for n in (0, 1, 50, 5000):
        ranges = scan_of(conv, rng.uniform(-180, 180, n), rng.uniform(0.1, 30, n)).ranges
        assert ranges.shape == (433,)
        assert ranges.dtype == np.float32


# ---------------------------------------------------------------------------- binning

@pytest.mark.parametrize('azimuth_deg', [-62.0, -61.8, -61.74, -61.5, -30.2, 0.0, 0.2, 0.3, 46.0,
                                         90.0, 120.7, 153.76, 153.8, 154.0])
def test_point_lands_in_rounded_bin(conv, azimuth_deg):
    ranges = scan_of(conv, azimuth_deg, 2.0).ranges
    expected = round((math.radians(azimuth_deg) - conv.angle_min) / conv.angle_increment)
    assert valid_beams(ranges) == [expected]
    assert ranges[expected] == pytest.approx(2.0, abs=1e-5)


def test_beam_direction_matches_published_angles(conv):
    # A consumer reconstructs the hit at angle_min + i * angle_increment: at most half a bin off.
    azimuths = np.linspace(-61.9, 153.9, 1500)
    for az in azimuths:
        beam = valid_beams(scan_of(conv, az, 3.0).ranges)[0]
        assert abs(conv.angle_min + beam * conv.angle_increment - math.radians(az)) \
            <= conv.angle_increment / 2 + 1e-6


def test_minimum_range_wins_in_a_bin(conv):
    # three returns in the 10 deg bin (order must not matter), one in the 20 deg bin
    ranges = scan_of(conv, [10.1, 10.0, 9.9, 20.0], [3.0, 1.5, 2.0, 4.0]).ranges
    assert valid_beams(ranges) == [144, 164]
    assert ranges[144] == pytest.approx(1.5, abs=1e-5)
    assert ranges[164] == pytest.approx(4.0, abs=1e-5)


def test_empty_bins_are_plus_inf(conv):
    ranges = scan_of(conv, [0.0], [2.0]).ranges
    assert np.isposinf(np.delete(ranges, 124)).all()  # never 0 / NaN / range_max: unknown != free


def test_empty_cloud(conv):
    empty = np.empty(0, dtype=np.float32)
    result = conv.convert(empty, empty, empty)
    assert result.ranges.shape == (433,) and np.isposinf(result.ranges).all()
    assert result.obstacles.shape == (0, 3) and result.obstacles.dtype == np.float32


# ---------------------------------------------------------------------------- azimuth window

def test_window_edges(conv):
    assert valid_beams(scan_of(conv, -62.0, 2.0).ranges) == [0]
    assert valid_beams(scan_of(conv, 154.0, 2.0).ranges) == [432]
    # Outside the window but within half a bin of the edge beams: still robot body -> rejected.
    for outside in (-62.01, -62.2, 154.01, 154.2):
        result = scan_of(conv, outside, 2.0)
        assert valid_beams(result.ranges) == []
        assert result.obstacles.shape == (0, 3)


def test_last_beam_never_reaches_beyond_requested_window():
    # 216.3 / 0.5 rounds UP: the last beam points at 154.5 deg, beyond the requested 154.3 deg.
    conv = CloudToScanConverter(ScanConfig(angle_max_deg=154.3))
    assert conv.num_beams == 434
    assert valid_beams(scan_of(conv, 154.29, 2.0).ranges) == [433]
    assert valid_beams(scan_of(conv, 154.4, 2.0).ranges) == []


def test_body_wedge_is_rejected(conv):
    # Dense fan over the whole blocked wedge, at body-hit and environment ranges, all heights.
    blocked = np.concatenate([np.arange(-180.0, -62.0, 0.1) - 0.05, np.arange(154.1, 180.01, 0.1)])
    for rng in (0.52, 0.6, 0.68, 1.5, 3.0):
        for height in (0.15, 0.33, 0.5, 1.0):
            result = scan_of(conv, blocked, rng, height)
            assert valid_beams(result.ranges) == []
            assert result.obstacles.shape == (0, 3)


def test_measured_self_hits_are_rejected(conv):
    # ref/bag_analysis.md section 3, velodyne frame. Cluster A: vertical face of the robot,
    # cluster B: right edge of the robot.
    rng = np.random.default_rng(7)
    cluster_a = np.column_stack([rng.uniform(-0.62, -0.47, 200), rng.uniform(-0.25, -0.16, 200),
                                 rng.uniform(-0.15, 0.05, 200)])
    az_b = np.radians(rng.uniform(-124, -104, 50))
    r_b = rng.uniform(0.51, 0.56, 50)
    cluster_b = np.column_stack([r_b * np.cos(az_b), r_b * np.sin(az_b),
                                 rng.uniform(-0.02, 0.16, 50)])
    body = np.vstack([cluster_a, cluster_b])
    result = conv.convert(body[:, 0], body[:, 1], body[:, 2])
    assert valid_beams(result.ranges) == []
    assert result.obstacles.shape == (0, 3)


# ---------------------------------------------------------------------------- height / range

def test_scan_height_band_is_above_floor(conv):
    heights = [0.19, 0.21, 0.59, 0.61]          # band is [0.20, 0.60] above the floor
    ranges = scan_of(conv, [0.0, 10.0, 20.0, 30.0], 2.0, heights).ranges
    assert valid_beams(ranges) == [144, 164]    # only the 10 and 20 deg points


def test_height_band_follows_lidar_height():
    # Same z in the lidar frame, different mounting height -> different height above the floor.
    z = np.array([0.0], dtype=np.float32)
    x, y = np.array([2.0]), np.array([0.0])
    low = CloudToScanConverter(ScanConfig(lidar_height=0.33))
    high = CloudToScanConverter(ScanConfig(lidar_height=0.70))   # z = 0 is 0.70 m up: above band
    assert valid_beams(low.convert(x, y, z).ranges) == [124]
    assert valid_beams(high.convert(x, y, z).ranges) == []
    # ... and a point 0.30 m below the high lidar is back inside [0.20, 0.60].
    assert valid_beams(high.convert(x, y, z - 0.30).ranges) == [124]


def test_scan_range_limits(conv):
    result = scan_of(conv, [0.0, 10.0, 20.0, 30.0], [0.54, 0.56, 24.9, 25.1])
    assert valid_beams(result.ranges) == [144, 164]
    assert (result.ranges[[144, 164]] >= conv.config.range_min).all()
    assert (result.ranges[[144, 164]] <= conv.config.range_max).all()


def test_range_is_horizontal(conv):
    # 3D distance 0.64 m but only 0.50 m horizontally: closer than range_min in the scan plane.
    assert valid_beams(conv.convert([0.5], [0.0], [0.2]).ranges) == []
    assert conv.convert([3.0], [4.0], [0.2]).ranges[
        round((math.atan2(4, 3) - conv.angle_min) / conv.angle_increment)
    ] == pytest.approx(5.0, abs=1e-5)


def test_nan_and_inf_points_are_ignored_without_warnings(conv):
    x = np.array([np.nan, 2.0, np.inf, 2.0, 2.0, -np.inf], dtype=np.float32)
    y = np.array([0.0, np.nan, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    z = np.array([0.0, 0.0, 0.0, np.nan, 0.0, 0.0], dtype=np.float32)
    with warnings.catch_warnings(), np.errstate(all='raise'):
        warnings.simplefilter('error')
        result = conv.convert(x, y, z)
    assert valid_beams(result.ranges) == [124]
    assert result.obstacles.shape == (1, 3)
    assert np.isfinite(result.obstacles).all()


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_absurdly_large_coordinates_are_ignored_without_warnings(conv, dtype):
    # 1e30 squared overflows float32, 1e300 already overflows in the cast: both must drop out.
    huge = 1e30 if dtype is np.float32 else 1e300
    x = np.array([huge, 2.0, -huge, 0.0], dtype=dtype)
    y = np.array([0.0, 0.0, huge, huge], dtype=dtype)
    z = np.zeros(4, dtype=dtype)
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        result = conv.convert(x, y, z)
    assert valid_beams(result.ranges) == [124]
    np.testing.assert_allclose(result.obstacles, [[2.0, 0.0, 0.0]])


# ---------------------------------------------------------------------------- obstacle cloud

def test_obstacle_band_range_and_window(conv):
    azimuth = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, -100.0]
    rng = [2.0, 2.0, 2.0, 2.0, 5.9, 6.1, 2.0]
    height = [0.09, 0.11, 1.19, 1.21, 0.5, 0.5, 0.5]
    obstacles = scan_of(conv, azimuth, rng, height).obstacles
    got = sorted(round(math.degrees(math.atan2(p[1], p[0]))) for p in obstacles)
    assert got == [10, 20, 40]   # too low, too high, too far and body-wedge points are gone
    assert obstacles.dtype == np.float32


def test_obstacles_are_voxel_downsampled(conv):
    rng = np.random.default_rng(3)
    # 300 points inside the single 5 cm voxel [1.00,1.05) x [0.50,0.55) x [0.10,0.15)
    cell = np.array([1.0, 0.5, 0.1]) + rng.uniform(0.002, 0.048, (300, 3))
    other = np.array([[2.011, -0.489, 0.011]])
    pts = np.vstack([cell, other])
    obstacles = conv.convert(pts[:, 0], pts[:, 1], pts[:, 2]).obstacles
    assert obstacles.shape == (2, 3)
    near = obstacles[np.argmin(obstacles[:, 0])]
    np.testing.assert_allclose(near, cell.mean(axis=0), atol=1e-5)      # centroid of the voxel
    np.testing.assert_allclose(obstacles[np.argmax(obstacles[:, 0])], other[0], atol=1e-6)


def test_voxel_downsample_keeps_one_point_per_voxel():
    rng = np.random.default_rng(11)
    xyz = rng.uniform(-6, 6, (20000, 3)).astype(np.float32)
    xyz[:, 2] = rng.uniform(-0.23, 0.87, 20000)
    out = voxel_downsample(xyz, 0.05)
    voxels_in = np.unique(np.floor(xyz / np.float32(0.05)).astype(np.int64), axis=0)
    voxels_out = np.unique(np.floor(out / np.float32(0.05)).astype(np.int64), axis=0)
    assert out.shape[0] == voxels_in.shape[0] < xyz.shape[0]
    # a centroid can sit 1 ulp across its own voxel face, hence >= 99.9 % instead of equality
    assert voxels_out.shape[0] >= 0.999 * voxels_in.shape[0]
    assert voxel_downsample(np.empty((0, 3), np.float32), 0.05).shape == (0, 3)


def test_obstacles_can_be_disabled(conv):
    result = conv.convert(*polar([0.0, 10.0], 2.0), with_obstacles=False)
    assert result.obstacles.shape == (0, 3)
    assert valid_beams(result.ranges) == [124, 144]


# ---------------------------------------------------------------------------- cross-check

def test_matches_naive_implementation_on_random_cloud(conv):
    cfg = conv.config
    gen = np.random.default_rng(42)
    n = 20000
    az = gen.uniform(-math.pi, math.pi, n)
    # keep clear of bin / window boundaries, where float32 vs float64 rounding may legally differ
    frac = (az - conv.angle_min) / conv.angle_increment + 0.5
    az = az[np.abs(frac - np.round(frac)) > 1e-3]
    rng = gen.uniform(0.3, 30.0, az.size)
    z = gen.uniform(-0.4, 1.0, az.size)
    x, y = rng * np.cos(az), rng * np.sin(az)
    x, y, z = (v.astype(np.float32) for v in (x, y, z))

    expected = [math.inf] * conv.num_beams
    for xi, yi, zi in zip(x.tolist(), y.tolist(), z.tolist()):
        a, r = math.atan2(yi, xi), math.hypot(xi, yi)
        in_band = cfg.scan_min_height - cfg.lidar_height <= zi <= cfg.scan_max_height \
            - cfg.lidar_height
        if conv.angle_min <= a <= conv.angle_max and cfg.range_min <= r <= cfg.range_max \
                and in_band:
            k = round((a - conv.angle_min) / conv.angle_increment)
            expected[k] = min(expected[k], r)
    ranges = conv.convert(x, y, z).ranges
    assert np.isfinite(ranges).sum() > 300
    np.testing.assert_allclose(ranges, np.array(expected), rtol=0, atol=1e-5)


# ---------------------------------------------------------------------------- PointCloud2 decoding

def velodyne_cloud(n, seed=5):
    gen = np.random.default_rng(seed)
    pts = np.zeros(n, dtype=VELODYNE_DTYPE)
    for name in ('x', 'y', 'z', 'intensity', 'time'):
        pts[name] = gen.uniform(-5, 5, n)
    pts['ring'] = gen.integers(0, 16, n)
    return pts


@pytest.mark.parametrize('container', [bytes, bytearray, lambda b: array.array('B', b)])
def test_cloud_xyz_reads_packed_22_byte_points_without_copy(container):
    assert VELODYNE_DTYPE.itemsize == 22 and not VELODYNE_DTYPE.isalignedstruct
    pts = velodyne_cloud(1000)
    data = container(pts.tobytes())
    x, y, z = cloud_xyz(data, VELODYNE_FIELDS, 22, 1000)
    for name, column in zip('xyz', (x, y, z)):
        np.testing.assert_array_equal(column, pts[name])
        assert column.strides == (22,)
        assert np.shares_memory(column, np.frombuffer(data, dtype=np.uint8))  # a view, no copy


def test_aligned_struct_assumption_would_be_wrong():
    # Guards the reason for the explicit-offset dtype: numpy's aligned layout of the same fields
    # is 24 bytes with `time` at 20, which would shear every point after the first.
    aligned = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                        ('ring', '<u2'), ('time', '<f4')], align=True)
    assert aligned.itemsize == 24 and aligned.fields['time'][1] == 20


def test_cloud_xyz_organized_cloud_with_row_padding():
    width, height, pad = 8, 4, 10
    pts = velodyne_cloud(width * height).reshape(height, width)
    pts['x'][1, 3] = np.nan                       # organized clouds mark invalid returns as NaN
    data = b''.join(row.tobytes() + b'\xff' * pad for row in pts)
    x, y, z = cloud_xyz(data, VELODYNE_FIELDS, 22, width, height, row_step=width * 22 + pad)
    np.testing.assert_array_equal(x, pts['x'].reshape(-1))
    np.testing.assert_array_equal(y, pts['y'].reshape(-1))
    np.testing.assert_array_equal(z, pts['z'].reshape(-1))
    # unpadded organized cloud (what velodyne_transform_node publishes with organize_cloud true)
    x2, _, _ = cloud_xyz(pts.tobytes(), VELODYNE_FIELDS, 22, width, height, row_step=width * 22)
    np.testing.assert_array_equal(x2, pts['x'].reshape(-1))


def test_organized_cloud_with_nan_holes_equals_its_valid_points(conv):
    # velodyne_transform_node with organize_cloud true: 16 x 1824, is_dense false, NaN = no return
    width, height = 16, 1824
    gen = np.random.default_rng(9)
    pts = velodyne_cloud(width * height)
    pts['z'] = gen.uniform(-0.3, 0.9, pts.size)
    hole = gen.random(pts.size) < 0.4
    for name in 'xyz':
        pts[name][hole] = np.nan
    organized = conv.convert(*cloud_xyz(pts.tobytes(), VELODYNE_FIELDS, 22, width, height,
                                        row_step=width * 22))
    valid = pts[~hole]
    dense = conv.convert(*cloud_xyz(valid.tobytes(), VELODYNE_FIELDS, 22, valid.size))
    assert np.isfinite(organized.ranges).sum() > 100
    np.testing.assert_array_equal(organized.ranges, dense.ranges)
    np.testing.assert_array_equal(organized.obstacles, dense.obstacles)


@pytest.mark.parametrize('bigendian', [False, True])
@pytest.mark.parametrize('organized', [False, True])
def test_selected_ring_filters_both_outputs(bigendian, organized):
    dtype = VELODYNE_DTYPE.newbyteorder('>' if bigendian else '<')
    pts = np.zeros(4, dtype=dtype)
    pts['x'] = [2.0, 1.0, 3.0, 4.0]
    pts['z'] = pts['x'] * math.tan(math.radians(1.0))
    pts['ring'] = [8, 7, 8, 9]
    if organized:
        # Two rows with eight bytes of padding each.
        data = pts[:2].tobytes() + bytes(8) + pts[2:].tobytes() + bytes(8)
        width, height, row_step = 2, 2, 52
    else:
        data, width, height, row_step = pts.tobytes(), 4, 1, 88
    xyz = cloud_xyz(data, VELODYNE_FIELDS, 22, width, height, row_step,
                    bigendian, selected_ring=8)
    np.testing.assert_array_equal(xyz[0], [2.0, 3.0])
    result = CloudToScanConverter().convert(*xyz)
    assert result.ranges[124] == pytest.approx(2.0)  # rejected ring 7 was nearer
    np.testing.assert_array_equal(np.sort(result.obstacles[:, 0]), [2.0, 3.0])
    empty = cloud_xyz(data, VELODYNE_FIELDS, 22, width, height, row_step,
                      bigendian, selected_ring=15)
    assert all(column.size == 0 for column in empty)


def test_selected_ring_requires_valid_metadata():
    pts = np.zeros(1, dtype=VELODYNE_DTYPE)
    for fields in (
            [f for f in VELODYNE_FIELDS if f.name != 'ring'],
            [SimpleNamespace(name=f.name, offset=f.offset,
                             datatype=FLOAT32 if f.name == 'ring' else f.datatype, count=1)
             for f in VELODYNE_FIELDS]):
        with pytest.raises(ValueError, match='UINT16.*ring'):
            cloud_xyz(pts.tobytes(), fields, 22, 1, selected_ring=8)


def test_cloud_xyz_big_endian_and_float64():
    fields = [SimpleNamespace(name='x', offset=0, datatype=FLOAT64, count=1),
              SimpleNamespace(name='y', offset=8, datatype=FLOAT64, count=1),
              SimpleNamespace(name='z', offset=16, datatype=FLOAT32, count=1)]
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['>f8', '>f8', '>f4'],
                      'offsets': [0, 8, 16], 'itemsize': 21})
    pts = np.zeros(3, dtype=dtype)
    pts['x'], pts['y'], pts['z'] = [1.5, 2.5, 3.5], [-1.0, 0.0, 1.0], [0.1, 0.2, 0.3]
    x, y, z = cloud_xyz(pts.tobytes(), fields, 21, 3, is_bigendian=True)
    np.testing.assert_array_equal(x, [1.5, 2.5, 3.5])
    np.testing.assert_array_equal(y, [-1.0, 0.0, 1.0])
    np.testing.assert_allclose(z, [0.1, 0.2, 0.3], rtol=1e-6)
    # and the converter accepts those non-native views directly
    assert np.isfinite(CloudToScanConverter().convert(x, y, z).ranges).sum() == 2


def test_cloud_xyz_empty_cloud():
    x, y, z = cloud_xyz(b'', VELODYNE_FIELDS, 22, 0)
    assert x.size == y.size == z.size == 0


def test_cloud_xyz_rejects_bad_layouts():
    data = velodyne_cloud(10).tobytes()
    with pytest.raises(ValueError, match="no 'z' field"):
        cloud_xyz(data, VELODYNE_FIELDS[:2], 22, 10)
    ring_as_x = [namespace_replace(VELODYNE_FIELDS[0], datatype=UINT16)] + VELODYNE_FIELDS[1:]
    with pytest.raises(ValueError, match='non-float'):
        cloud_xyz(data, ring_as_x, 22, 10)
    with pytest.raises(ValueError, match='too short'):
        cloud_xyz(data, VELODYNE_FIELDS, 22, 11)
    with pytest.raises(ValueError, match='too short'):
        cloud_xyz(data, VELODYNE_FIELDS, 22, 5, height=3, row_step=110)   # 3 rows > data
    with pytest.raises(ValueError, match='inconsistent.*row_step'):
        cloud_xyz(data, VELODYNE_FIELDS, 22, 5, height=2, row_step=100)   # rows would overlap
    with pytest.raises(ValueError, match='inconsistent'):
        cloud_xyz(data, VELODYNE_FIELDS, 10, 10)   # z (offset 8, 4 bytes) does not fit in 10
    with pytest.raises(ValueError, match='inconsistent'):
        cloud_xyz(data, VELODYNE_FIELDS, 0, 10)    # point_step 0 (uninitialised message)


def namespace_replace(namespace, **changes):
    return SimpleNamespace(**{**vars(namespace), **changes})


def test_packed_cloud_end_to_end(conv):
    pts = np.zeros(3, dtype=VELODYNE_DTYPE)
    pts['x'], pts['y'], pts['z'] = [2.0, 0.0, -0.6], [0.0, 3.0, -0.2], [0.0, 0.1, 0.0]
    pts['time'] = [-0.09, -0.05, 0.0]      # bytes an aligned reader would mistake for xyz
    result = conv.convert(*cloud_xyz(pts.tobytes(), VELODYNE_FIELDS, 22, 3))
    assert valid_beams(result.ranges) == [124, 304]           # 0 deg and 90 deg; body hit dropped
    np.testing.assert_allclose(result.ranges[[124, 304]], [2.0, 3.0], atol=1e-6)
    assert result.obstacles.shape == (2, 3)


# ---------------------------------------------------------------------------- messages / params

def test_scan_message_fields(conv):
    header = Header(frame_id='velodyne')
    header.stamp.sec, header.stamp.nanosec = 1788027582, 678000000
    ranges = scan_of(conv, [0.0], [2.0]).ranges
    msg = make_scan_msg(header, conv, ranges, 0.1009)
    assert msg.header.frame_id == 'velodyne'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1788027582, 678000000)
    assert (msg.angle_min, msg.angle_max, msg.angle_increment) == (
        conv.angle_min, conv.angle_max, conv.angle_increment)
    assert msg.time_increment == 0.0 and msg.scan_time == pytest.approx(0.1009)
    assert (msg.range_min, msg.range_max) == (0.55, 25.0)
    assert len(msg.ranges) == 433 and msg.ranges[124] == pytest.approx(2.0)
    assert math.isinf(msg.ranges[0]) and msg.ranges[0] > 0
    assert len(msg.intensities) == 0


def test_obstacle_message_layout():
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    msg = make_obstacle_msg(Header(frame_id='velodyne'), xyz)
    assert (msg.height, msg.width, msg.point_step, msg.row_step) == (1, 2, 12, 24)
    assert [(f.name, f.offset, f.datatype, f.count) for f in msg.fields] == [
        ('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1)]
    assert not msg.is_bigendian and msg.is_dense
    np.testing.assert_array_equal(np.frombuffer(msg.data, dtype='<f4').reshape(-1, 3), xyz)
    # round trip through the decoder used for the input side
    x, y, z = cloud_xyz(msg.data, msg.fields, msg.point_step, msg.width, msg.height, msg.row_step)
    np.testing.assert_array_equal(np.column_stack([x, y, z]), xyz)

    empty = make_obstacle_msg(Header(), np.empty((0, 3), np.float32))
    assert (empty.width, empty.row_step, len(empty.data)) == (0, 0, 0)


@pytest.mark.parametrize('bad', [
    {'angle_increment_deg': 0.0}, {'angle_increment_deg': -0.5},
    {'angle_min_deg': 154.0, 'angle_max_deg': -62.0}, {'angle_max_deg': 200.0},
    {'angle_min_deg': 0.0, 'angle_max_deg': 0.2},
    {'range_min': -1.0}, {'range_min': 30.0}, {'obstacle_max_range': 0.5},
    {'scan_min_height': 0.7}, {'obstacle_max_height': 0.05}, {'obstacle_voxel': 0.0},
    # range_min 0 would publish (0, 0, 0) placeholders as a 0 m beam + an obstacle at the lidar
    {'range_min': 0.0},
    # NaN / inf would pass the ordered comparisons and silently blank or garble the output
    {'lidar_height': math.nan}, {'lidar_height': math.inf}, {'range_max': math.inf},
    {'obstacle_max_range': math.inf}, {'obstacle_voxel': math.inf}, {'obstacle_voxel': math.nan},
    {'scan_max_height': math.inf}, {'obstacle_min_height': -math.inf}, {'angle_min_deg': math.nan},
])
def test_invalid_parameters_are_refused(bad):
    with pytest.raises(ValueError, match='invalid cloud_to_scan parameters'):
        CloudToScanConverter(dataclasses.replace(ScanConfig(), **bad))


def test_defaults_are_the_contract_values():
    assert dataclasses.asdict(ScanConfig()) == {
        'lidar_height': 0.330, 'angle_min_deg': -62.0, 'angle_max_deg': 154.0,
        'angle_increment_deg': 0.5, 'range_min': 0.55, 'range_max': 25.0,
        'scan_min_height': 0.20, 'scan_max_height': 0.60, 'obstacle_min_height': 0.10,
        'obstacle_max_height': 1.20, 'obstacle_max_range': 6.0, 'obstacle_voxel': 0.05}

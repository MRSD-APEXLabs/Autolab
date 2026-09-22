"""Depth to point cloud, flying-pixel filter, voxel grid, PointCloud2 packing and the snapshot cache."""
import numpy as np
import pytest

from camera_perception.pointcloud import Snapshot, depth_to_points, voxel_downsample
from scene import INTRINSICS, Scene, rx

BOUNDS = ((-0.6, 0.6), (-0.6, 0.6), (0.2, 1.7))


def test_points_lie_on_the_plane_they_came_from():
    scene = Scene(rotation=rx(0.5), origin=(0.0, 0.0, 1.0))
    points, rgb = depth_to_points(scene.depth_mm(), INTRINSICS, stride=4)
    assert rgb is None and points.dtype == np.float32
    assert len(points) == 150 * 240
    normal = scene.rotation[:, 2]
    np.testing.assert_allclose((points - scene.origin) @ normal, 0.0, atol=0.002)


def test_bounds_crop_in_the_camera_frame():
    scene = Scene(rotation=rx(0.5), origin=(0.0, 0.0, 1.0))
    points, _ = depth_to_points(scene.depth_mm(), INTRINSICS, stride=4, bounds=((-0.2, 0.2), (-0.6, 0.6), (0.9, 1.2)))
    assert 0 < len(points) < 150 * 240
    assert points[:, 0].min() >= -0.2 and points[:, 0].max() <= 0.2
    assert points[:, 2].min() >= 0.9 and points[:, 2].max() <= 1.2


def test_colours_are_packed_rgb():
    depth = np.full((8, 8), 1000, np.uint16)
    bgr = np.zeros((8, 8, 3), np.uint8)
    bgr[..., 2] = 255                      # red
    bgr[0, 0] = (1, 2, 3)
    points, rgb = depth_to_points(depth, INTRINSICS, stride=2, bgr=bgr)
    assert len(points) == len(rgb) == 16
    assert rgb[0] == 0x030201 and set(rgb[1:].tolist()) == {0xFF0000}


def test_edge_filter_drops_points_on_depth_jumps_only():
    depth = np.full((600, 960), 800, np.uint16)
    depth[:, 480:] = 1500                  # a step
    depth[100:120, 100:120] = 0            # a hole: not an edge
    kept, _ = depth_to_points(depth, INTRINSICS, stride=4, edge_threshold=0.02)
    everything, _ = depth_to_points(depth, INTRINSICS, stride=4)
    assert len(everything) - len(kept) == 2 * 150                       # the two columns that face the step
    u = np.round(kept[:, 0] / kept[:, 2] * INTRINSICS.fx + INTRINSICS.cx).astype(int)
    assert not np.isin(u, [476, 480]).any()
    tilted = Scene(rotation=rx(1.2), origin=(0.0, 0.0, 1.0)).depth_mm()   # a steep but smooth surface stays
    assert len(depth_to_points(tilted, INTRINSICS, stride=4, bounds=BOUNDS, edge_threshold=0.02)[0]) == \
        len(depth_to_points(tilted, INTRINSICS, stride=4, bounds=BOUNDS)[0])


def test_voxel_downsample_merges_points_per_cube():
    points = np.array([[0.001, 0.001, 1.001], [0.009, 0.009, 1.009], [0.05, 0.0, 1.0]], np.float32)
    rgb = np.array([1, 2, 3], np.uint32)
    merged, colours = voxel_downsample(points, rgb, 0.01)
    assert len(merged) == 2
    order = np.argsort(merged[:, 0])
    np.testing.assert_allclose(merged[order[0]], [0.005, 0.005, 1.005], atol=1e-6)
    assert sorted(colours.tolist()) == [1, 3]
    same, none = voxel_downsample(points, None, 0.0)
    assert same is points and none is None


def test_pointcloud2_round_trip():
    pytest.importorskip('sensor_msgs')
    from std_msgs.msg import Header
    from camera_perception.pointcloud import pointcloud2_to_numpy, to_pointcloud2
    points = np.random.default_rng(1).uniform(-1, 1, (50, 3)).astype(np.float32)
    rgb = np.arange(50, dtype=np.uint32) * 0x010203
    msg = to_pointcloud2(Header(frame_id='top_camera'), points, rgb)
    assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'rgb'] and msg.point_step == 16
    assert (msg.width, msg.height, msg.row_step, msg.is_dense) == (50, 1, 800, True)
    back, back_rgb = pointcloud2_to_numpy(msg)
    np.testing.assert_array_equal(back, points)
    np.testing.assert_array_equal(back_rgb, rgb)
    plain = to_pointcloud2(Header(), points)
    assert plain.point_step == 12 and pointcloud2_to_numpy(plain)[1] is None


def test_snapshot_stacks_frames_and_caches_them(tmp_path):
    cache = tmp_path / 'sub' / 'cloud.npy'
    snapshot = Snapshot(frames=3, cache_path=str(cache))
    assert not snapshot.load()
    frame = np.array([[0.0, 0.0, 1.0], [0.1, 0.0, 1.0]], np.float32)
    rgb = np.array([0xFF0000, 0x00FF00], np.uint32)
    assert not snapshot.add(frame, rgb) and not snapshot.add(frame, rgb) and snapshot.progress == 2
    assert snapshot.add(frame, rgb) and snapshot.ready and len(snapshot.points) == 6
    assert np.load(cache).shape == (6, 4)
    loaded = Snapshot(frames=3, cache_path=str(cache))
    assert loaded.load()
    np.testing.assert_array_equal(loaded.points, snapshot.points)
    np.testing.assert_array_equal(loaded.rgb, snapshot.rgb)
    loaded.reset()
    assert not loaded.ready and loaded.progress == 0


def test_snapshot_skips_empty_frames_and_survives_a_failed_cache_write(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('')
    snapshot = Snapshot(frames=2, cache_path=str(blocker / 'cloud.npy'))   # parent is a file: mkdir fails
    frame = np.array([[0.0, 0.0, 1.0]], np.float32)
    assert not snapshot.add(np.zeros((0, 3), np.float32)) and snapshot.progress == 0
    assert not snapshot.add(frame) and snapshot.add(frame)
    assert snapshot.ready and len(snapshot.points) == 2 and isinstance(snapshot.save_error, OSError)


def test_snapshot_reads_the_pc3_cache_and_rejects_other_shapes(tmp_path):
    pc3 = tmp_path / 'pc3_accumulated.npy'
    np.save(pc3, np.ones((5, 3), np.float32))
    snapshot = Snapshot(cache_path=str(pc3))
    assert snapshot.load() and snapshot.points.shape == (5, 3) and snapshot.rgb is None
    np.save(pc3, np.ones((5, 2), np.float32))
    with pytest.raises(ValueError):
        Snapshot(cache_path=str(pc3)).load()


def test_snapshot_voxel_and_mixed_colours():
    snapshot = Snapshot(frames=2, voxel=0.01)
    frame = np.array([[0.0, 0.0, 1.0]], np.float32)
    snapshot.add(frame, np.array([1], np.uint32))
    assert snapshot.add(frame + 0.001, None)
    assert len(snapshot.points) == 1 and snapshot.rgb is None       # a frame without colour: no colours

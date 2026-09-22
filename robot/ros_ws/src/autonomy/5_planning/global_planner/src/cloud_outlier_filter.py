#!/usr/bin/env python3
"""Radius outlier removal for a PointCloud2.

Drops points sitting in sparse space: the cloud is binned into cubes of side
`search_radius` metres and every point in a cube holding fewer than
`min_neighbors` other points is removed (isolated / very sparse noise). Voxel
counting is O(n), which matters on the Jetson. Fields, frame and stamp are
passed through untouched.

  input_topic   /zed_pointcloud          cloud to clean
  output_topic  /zed_pointcloud_clean    cleaned cloud (MoveIt octomap reads this)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class CloudOutlierFilter(Node):
    def __init__(self):
        super().__init__('cloud_outlier_filter')
        self.declare_parameter('input_topic', '/zed_pointcloud')
        self.declare_parameter('output_topic', '/zed_pointcloud_clean')
        self.declare_parameter('search_radius', 0.06)
        self.declare_parameter('min_neighbors', 8)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, qos)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self.on_cloud, qos)
        self._last_log = 0.0

    def on_cloud(self, msg: PointCloud2):
        radius = self.get_parameter('search_radius').value
        min_nb = self.get_parameter('min_neighbors').value
        t0 = time.monotonic()

        n = msg.width * msg.height
        if n == 0:
            self.pub.publish(msg)
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        xyz = np.stack([rows[:, off[a]:off[a] + 4].copy().view(np.float32).ravel()
                        for a in ('x', 'y', 'z')], axis=1)

        keep = np.zeros(n, dtype=bool)
        idx = np.flatnonzero(np.isfinite(xyz).all(axis=1))
        if idx.size:
            cells = np.floor(xyz[idx] / radius).astype(np.int64)
            cells -= cells.min(axis=0)
            dims = cells.max(axis=0) + 1
            key = (cells[:, 0] * dims[1] + cells[:, 1]) * dims[2] + cells[:, 2]  # one int per cube
            _, inv, counts = np.unique(key, return_inverse=True, return_counts=True)
            counts = counts[inv]  # points in each point's own cube, incl. itself
            keep[idx[counts >= min_nb + 1]] = True

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = rows[keep].tobytes()
        self.pub.publish(out)

        now = time.monotonic()
        if now - self._last_log > 5.0:
            self._last_log = now
            self.get_logger().info(
                f'{n} -> {out.width} points ({n - out.width} removed) in {1000 * (now - t0):.0f} ms')


def main():
    rclpy.init()
    rclpy.spin(CloudOutlierFilter())


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import asyncio
import base64
import json
import zlib
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import websockets

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2


#WS_URI = "ws://192.168.10.7:8766"
WS_URI = "ws://192.168.1.101:8766"
FRAME_ID = "top_camera"
PC_COUNT = 10


class WSVisualizer(Node):
    def __init__(self):
        super().__init__("ws_visualizer")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()

        self._pc_buffer: List[np.ndarray] = []
        self._pcs: Optional[np.ndarray] = None

        # Servo topics
        self.servo_image_pub = self.create_publisher(Image, "/servo/image", qos)
        self.servo_det_pub = self.create_publisher(MarkerArray, "/servo/detections", qos)
        self.servo_target_pub = self.create_publisher(Marker, "/servo/target", qos)
        self.servo_annot_pub = self.create_publisher(Image, "/servo/annotated_image", qos)

        # Inspect topics
        self.inspect_image_pub = self.create_publisher(Image, "/inspect/image", qos)
        self.inspect_pc_pub = self.create_publisher(PointCloud2, "/zed_pointcloud", qos)
        self.inspect_wp_pub = self.create_publisher(MarkerArray, "/inspect/wellplates", qos)
        self.inspect_tag_pub = self.create_publisher(MarkerArray, "/inspect/apriltags", qos)
        self.inspect_annot_pub = self.create_publisher(Image, "/inspect/annotated_image", qos)

        self.get_logger().info("WS visualizer initialized")

    def header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = FRAME_ID
        return h

    def decode_image(self, b64: str) -> Optional[np.ndarray]:
        if not b64:
            return None
        try:
            img_bytes = base64.b64decode(b64)
            np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")
            return None

    def decode_pointcloud(self, b64: str, shape: List[int], dtype: str) -> Optional[np.ndarray]:
        if not b64 or not shape or not dtype:
            return None
        try:
            raw = zlib.decompress(base64.b64decode(b64))
            pc = np.frombuffer(raw, dtype=np.dtype(dtype)).reshape(shape)
            return pc[:, :3]
        except Exception as e:
            self.get_logger().error(f"Pointcloud decode error: {e}")
            return None

    def create_pc2(self, points: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(self.header(), fields, points.tolist())

    def _make_text_marker(
        self,
        ns: str,
        idx: int,
        x: float,
        y: float,
        z: float,
        text: str,
        r: float,
        g: float,
        b: float,
    ) -> Marker:
        m = Marker()
        m.header = self.header()
        m.ns = ns
        m.id = idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = 0.06
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.text = text
        return m

    def _pose_to_marker_pose(self, m: Marker, pose: Dict[str, Any]):
        pos = pose.get("position", [0.0, 0.0, 0.0])
        quat = pose.get("orientation", [0.0, 0.0, 0.0, 1.0])

        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])

        # Expected quaternion order from server: [x, y, z, w]
        m.pose.orientation.x = float(quat[0])
        m.pose.orientation.y = float(quat[1])
        m.pose.orientation.z = float(quat[2])
        m.pose.orientation.w = float(quat[3])

    def servo_detections_to_markers(self, dets: List[Dict[str, Any]]) -> MarkerArray:
        markers = MarkerArray()

        # Clear previous markers in RViz
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, d in enumerate(dets):
            # If a 3D pose is available, prefer it; otherwise fall back to image-space proxy.
            pose = d.get("pose")
            label = d.get("name", d.get("cls_id", -1))
            conf = float(d.get("conf", 0.0))
            theta = float(d.get("theta", 0.0))

            m = Marker()
            m.header = self.header()
            m.ns = "servo_detections"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.35

            if pose and isinstance(pose, dict):
                self._pose_to_marker_pose(m, pose)
                # Use the estimated 3D pose as-is
                if "size" in d:
                    size = d["size"]
                    if isinstance(size, (list, tuple)) and len(size) >= 2:
                        # Keep cube visible, but do not interpret pixel size as metric.
                        m.scale.x = max(float(size[0]) / 5000.0, 0.01)
                        m.scale.y = max(float(size[1]) / 5000.0, 0.01)
                        m.scale.z = 0.02
            else:
                cx, cy = d.get("center", [0.0, 0.0])
                w, h = d.get("size", [0.0, 0.0])
                m.pose.position.x = float(cx) / 500.0
                m.pose.position.y = float(cy) / 500.0
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = max(float(w) / 500.0, 0.001)
                m.scale.y = max(float(h) / 500.0, 0.001)
                m.scale.z = 0.01

            markers.markers.append(m)
            markers.markers.append(
                self._make_text_marker(
                    "servo_detections_text",
                    1000 + i,
                    m.pose.position.x,
                    m.pose.position.y,
                    m.pose.position.z + 0.04,
                    f"{label} {conf:.2f} th={theta:.2f}",
                    0.0,
                    1.0,
                    0.0,
                )
            )

        return markers

    def servo_target_to_marker(self, target_uv: List[float]) -> Marker:
        m = Marker()
        m.header = self.header()
        m.ns = "servo_target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(target_uv[0]) / 500.0
        m.pose.position.y = float(target_uv[1]) / 500.0
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.scale.y = 0.02
        m.scale.z = 0.02
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        return m

    def wellplates_to_markers(self, dets: List[Dict[str, Any]]) -> MarkerArray:
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, d in enumerate(dets):
            m = Marker()
            m.header = self.header()
            m.ns = "wellplates"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD

            pose = d.get("pose")
            if pose and isinstance(pose, dict):
                self._pose_to_marker_pose(m, pose)
            else:
                cx, cy = d.get("center", [0.0, 0.0])
                m.pose.position.x = float(cx) / 500.0
                m.pose.position.y = float(cy) / 500.0
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0

            # Prefer metric size if the server provides it; otherwise keep a visible placeholder.
            size = d.get("metric_size", d.get("size", [0.0, 0.0]))
            if isinstance(size, (list, tuple)) and len(size) >= 2:
                sx = float(size[0])
                sy = float(size[1])
                # If these are image-space pixels, the values will be huge; keep a fallback cap.
                if sx > 1.0 or sy > 1.0:
                    sx = max(sx / 5000.0, 0.05)
                    sy = max(sy / 5000.0, 0.05)
                m.scale.x = max(sx, 0.01)
                m.scale.y = max(sy, 0.01)
            else:
                m.scale.x = 0.1
                m.scale.y = 0.06
            m.scale.z = 0.01

            m.color.g = 1.0
            m.color.a = 0.8
            markers.markers.append(m)

            markers.markers.append(
                self._make_text_marker(
                    "wellplates_text",
                    1000 + i,
                    m.pose.position.x,
                    m.pose.position.y,
                    m.pose.position.z + 0.03,
                    f"{d.get('name', d.get('cls_id', -1))} {float(d.get('conf', 0.0)):.2f}",
                    0.0,
                    1.0,
                    0.0,
                )
            )

        return markers

    def apriltags_to_markers(self, tags: List[Dict[str, Any]]) -> MarkerArray:
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, t in enumerate(tags):
            m = Marker()
            m.header = self.header()
            m.ns = "apriltags"
            m.id = t.get("id")
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            pose = t.get("pose")
            if pose and isinstance(pose, dict) and "position" in pose and "orientation" in pose:
                self._pose_to_marker_pose(m, pose)
            else:
                cx, cy = t.get("center", [0.0, 0.0])
                m.pose.position.x = float(cx) / 500.0
                m.pose.position.y = float(cy) / 500.0
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0

            m.scale.x = 0.02
            m.scale.y = 0.02
            m.scale.z = 0.02
            m.color.r = 1.0
            m.color.a = 1.0
            markers.markers.append(m)

        return markers

    def publish_servo(self, data: Dict[str, Any]):
        img = self.decode_image(data.get("image_jpeg_b64", ""))
        if img is None:
            return

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_img.header = self.header()
        self.servo_image_pub.publish(ros_img)

        # Publish an annotated copy too so you can debug with image viewers.
        ann = img.copy()
        for det in data.get("detections", []):
            cx, cy = det.get("center", [0, 0])
            cv2.circle(ann, (int(cx), int(cy)), 6, (0, 255, 0), -1)
            cv2.putText(
                ann,
                f"cls={det.get('cls_id', -1)} {float(det.get('conf', 0.0)):.2f}",
                (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        target_uv = data.get("target_uv", [0.0, 0.0])
        cv2.drawMarker(
            ann,
            (int(target_uv[0]), int(target_uv[1])),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )
        ros_ann = self.bridge.cv2_to_imgmsg(ann, encoding="bgr8")
        ros_ann.header = self.header()
        self.servo_annot_pub.publish(ros_ann)

        self.servo_det_pub.publish(self.servo_detections_to_markers(data.get("detections", [])))
        self.servo_target_pub.publish(self.servo_target_to_marker(target_uv))

    def publish_inspect(self, data: Dict[str, Any]):
        img = self.decode_image(data.get("image_jpeg_b64", ""))
        if img is None:
            return

        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_img.header = self.header()
        self.inspect_image_pub.publish(ros_img)

        ann = img.copy()
        for det in data.get("wellplates", []):
            cx, cy = det.get("center", [0, 0])
            w, h = det.get("size", [0, 0])
            cv2.rectangle(
                ann,
                (int(cx - w / 2), int(cy - h / 2)),
                (int(cx + w / 2), int(cy + h / 2)),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                ann,
                f"{det.get('name', det.get('cls_id', -1))} {float(det.get('conf', 0.0)):.2f}",
                (int(cx) + 5, int(cy) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        for tag in data.get("apriltags", []):
            cx, cy = tag.get("center", [0, 0])
            cv2.circle(ann, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(
                ann,
                f"ID {tag.get('id', -1)}",
                (int(cx) + 5, int(cy) + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        ros_ann = self.bridge.cv2_to_imgmsg(ann, encoding="bgr8")
        ros_ann.header = self.header()
        self.inspect_annot_pub.publish(ros_ann)

        pc = self.decode_pointcloud(
            data.get("pointcloud_zlib_b64", ""),
            data.get("pointcloud_shape", [0, 0]),
            data.get("pointcloud_dtype", "float32"),
        )
        if pc is not None and self._pcs is None:
            self._pc_buffer.append(pc)
            self.get_logger().info(
                f"Buffering point clouds: {len(self._pc_buffer)}/{PC_COUNT}"
            )
            if len(self._pc_buffer) >= PC_COUNT:
                self._pcs = np.vstack(self._pc_buffer)
                self._pc_buffer.clear()
                self.get_logger().info(
                    f"Point cloud accumulated: {self._pcs.shape[0]} points."
                )

        if self._pcs is not None:
            self.inspect_pc_pub.publish(self.create_pc2(self._pcs))

        self.inspect_wp_pub.publish(self.wellplates_to_markers(data.get("wellplates", [])))
        self.inspect_tag_pub.publish(self.apriltags_to_markers(data.get("apriltags", [])))

    def _handle_message(self, raw: str):
        try:
            data = json.loads(raw)
            msg_type = data.get("type", "")
            if msg_type == "servo_frame":
                self.publish_servo(data)
            elif msg_type == "inspect_frame":
                self.publish_inspect(data)
            elif msg_type == "idle":
                pass
            else:
                self.get_logger().debug(f"Unknown message type: {msg_type}")
        except Exception as e:
            self.get_logger().error(f"Message handling error: {e}")

    async def run(self):
        loop = asyncio.get_running_loop()
        while rclpy.ok():
            try:
                async with websockets.connect(
                    WS_URI,
                    max_size=None,
                    ping_interval=None,
                ) as ws:
                    self.get_logger().info(f"Connected to {WS_URI}")
                    async for msg in ws:
                        loop.run_in_executor(None, self._handle_message, msg)

            except (websockets.ConnectionClosedError, websockets.InvalidStatusCode) as e:
                self.get_logger().warn(f"WebSocket connection error, retrying: {e}")
                await asyncio.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")
                await asyncio.sleep(0.1)


def main():
    rclpy.init()
    node = WSVisualizer()

    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

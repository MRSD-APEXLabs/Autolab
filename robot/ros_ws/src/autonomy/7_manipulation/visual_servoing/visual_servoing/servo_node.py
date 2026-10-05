"""ROS 2 node: wrist-camera visual servo grasp of a well plate on the xArm6 (the Camera-Edge "servo" mode).

Inputs, from the camera stack's zedx_nano_depth node (`camera_namespace`, /zedx_nano):

  left/image_rect_color   bgr8, paired with the depth by exact stamp
  depth/image_rect        16UC1 millimetres, 0 = invalid
  depth/camera_info       the rectified pinhole model of both

Interface:

  visual_servo/start      std_srvs/Trigger: start a grasp run (returns at once; follow visual_servo/status)
  visual_servo/stop       std_srvs/Trigger: abort the run; the arm stops where it is
  visual_servo/status     std_msgs/String JSON, 2 Hz and on every change of phase: phase, result, errors, plate
  visual_servo/annotated_image   the wrist image with the plate, the fingertip model and the state (while subscribed)
  ws://0.0.0.0:8765/      the Camera-Edge control protocol (edge_control.py), for Autolab's manipulation_executive

The detector (camera_perception's YoloDetector, best_wrist.pt) runs only during a run or with `preview`.
At the start of a run the camera hub is switched to the Nano (`select_service`) and, afterwards, back to
the camera it streamed before (`restore_camera`). With execute:=false (the default) the arm is only read.
"""
from __future__ import annotations

from functools import partial
import json
import logging
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from camera_perception.geometry import Intrinsics
from zedx_nano_depth.sync import PairBuffer

from .arm import XArm
from .control import camera_point
from .edge_control import EdgeControlServer, EdgeModes
from .params import build_config, build_mount
from .servo import ServoRunner

QOS = QoSProfile(depth=2)
STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class VisualServoNode(Node):
    def __init__(self, arm=None, detector=None, **kwargs):
        """`arm` and `detector` replace the xArm and YOLO (tests); kwargs go to rclpy Node."""
        super().__init__('visual_servo', **kwargs)
        param = lambda name, default: self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.camera_ns = param('camera_namespace', '/zedx_nano').rstrip('/')
        self.preview = bool(param('preview', False))
        self.depth_scale = float(param('depth_scale', 1.0))
        self.config = build_config(param)
        self.mount = build_mount(param)
        self.select_service = param('select_service', '/camera_hub/select_zedx_nano')
        self.select_timeout = float(param('select_timeout', 20.0))
        self.restore_camera = bool(param('restore_camera', True))
        self.hub_node = param('hub_node', '/camera_hub').rstrip('/')
        robot_ip = param('robot_ip', '192.168.1.236')
        control_host, control_port = param('control_host', '0.0.0.0'), int(param('control_port', 8765))

        self.detector = detector if detector is not None else self._make_detector(param)
        self.arm = arm if arm is not None else XArm(robot_ip, read_only=self.config.dry_run)
        self.runner = ServoRunner(self.arm, self.mount, self.config, prepare=self._prepare_camera)
        self.bridge = CvBridge()

        services = ReentrantCallbackGroup()
        self.pub_status = self.create_publisher(String, 'visual_servo/status', STATUS_QOS)
        self.pub_annotated = self.create_publisher(Image, 'visual_servo/annotated_image', QOS)
        self.create_service(Trigger, 'visual_servo/start', self._on_start, callback_group=services)
        self.create_service(Trigger, 'visual_servo/stop', self._on_stop, callback_group=services)
        self._select_client = (self.create_client(Trigger, self.select_service, callback_group=services)
                               if self.select_service else None)
        self._hub_active = None       # the camera the hub streamed before our switch, from its status topic
        self._restore_to = None
        self._restore_clients = {camera: self.create_client(Trigger, f'{self.hub_node}/select_{camera}',
                                                            callback_group=services)
                                 for camera in ('zedx', 'zedx_nano')} if self.restore_camera else {}
        self.create_subscription(String, f'{self.hub_node}/status', self._on_hub_status, STATUS_QOS,
                                 callback_group=MutuallyExclusiveCallbackGroup())

        self._info = None
        self._intrinsics_key = self._intrinsics = None
        self._pairs = PairBuffer()
        self._pending = None
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._last_detections = []
        self._last_phase = None
        inputs = MutuallyExclusiveCallbackGroup()
        self.create_subscription(CameraInfo, f'{self.camera_ns}/depth/camera_info', self._on_info, QOS,
                                 callback_group=inputs)
        for index, topic in enumerate(('left/image_rect_color', 'depth/image_rect')):
            self.create_subscription(Image, f'{self.camera_ns}/{topic}', partial(self._on_input, index), QOS,
                                     callback_group=inputs)
        self._worker = threading.Thread(target=self._work, name='visual_servo_detect', daemon=True)
        self._worker.start()
        self.create_timer(0.5, self._publish_status)

        self.modes = EdgeModes(self._start_run, self.runner.abort, self.runner.status)
        self.control = None
        if control_port >= 0:
            self.control = EdgeControlServer(self.modes, control_host, control_port)
            self.control.start()
        self.get_logger().info(
            f'Visual servo on {self.camera_ns}: {"EXECUTE (the arm moves)" if not self.config.dry_run else "dry run (arm read-only)"}, '
            f'xArm {robot_ip}, hover {self.config.hover_height * 1000:.0f} mm, '
            f'grasp {"on" if self.config.grasp else "off"}; {self.mount.describe()}'
            + (f'; control ws://{control_host}:{self.control.port}/' if self.control else ''))

    # -- setup ---------------------------------------------------------------------------------------
    def _make_detector(self, param):
        from camera_perception.yolo import YoloDetector
        model = os.path.expanduser(param('yolo.model', ''))
        if not model:
            raise ValueError('set yolo.model to the wrist YOLO weights (best_wrist.pt)')
        detector = YoloDetector(model, device=param('yolo.device', 'cuda:0'), conf=float(param('yolo.conf', 0.25)),
                                iou=float(param('yolo.iou', 0.7)), imgsz=int(param('yolo.imgsz', 0)),
                                classes=param('yolo.classes', ''), max_detections=1,
                                clahe_clip=float(param('yolo.clahe_clip', 0.0)))
        t0 = time.monotonic()
        detector.warmup()
        self.get_logger().info(f'YOLO {os.path.basename(model)} ready in {time.monotonic() - t0:.1f} s: '
                               f'{detector.describe()}')
        return detector

    # -- runs ----------------------------------------------------------------------------------------
    def _start_run(self, on_done):
        return self.runner.start(on_done=lambda status: self._run_done(status, on_done))

    def _run_done(self, status, on_done):
        self.get_logger().info(f'Visual servo run ended: {status.get("result")} {status.get("message", "")}')
        client = self._restore_clients.get(self._restore_to)
        if client is not None and client.service_is_ready():
            client.call_async(Trigger.Request())   # the next step (planning) needs it; don't wait here
            self.get_logger().info(f'Switching the camera hub back to {self._restore_to}')
        self._restore_to = None
        on_done(status)
        self._publish_status()

    def _on_start(self, request, response):
        try:
            response.success = self.modes.switch('servo') == 'servo'
            response.message = 'started' if response.success else 'not started'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_stop(self, request, response):
        self.modes.stop()
        response.success, response.message = True, 'stopping'
        return response

    def _on_hub_status(self, msg):
        try:
            self._hub_active = json.loads(msg.data).get('active')
        except (ValueError, AttributeError):
            pass

    def _prepare_camera(self, check_abort):
        """Runs on the run's thread: switch the hub to the Nano and wait until it streams."""
        if self._select_client is None:
            return
        before = self._hub_active
        if not self._select_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{self.select_service} is not available: is the camera stack (camera_hub_node) running?')
        done = threading.Event()
        future = self._select_client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + self.select_timeout
        while not done.wait(0.1):
            check_abort()
            if time.monotonic() > deadline:
                raise RuntimeError(f'{self.select_service} did not answer in {self.select_timeout:.0f} s')
        result = future.result()
        if not result.success:
            raise RuntimeError(f'{self.select_service}: {result.message}')
        target = self.select_service.rsplit('select_', 1)[-1]
        if self.restore_camera and before and before != target:
            self._restore_to = before

    # -- inputs --------------------------------------------------------------------------------------
    def _on_info(self, msg):
        self._info = msg

    def _on_input(self, index, msg):
        pair = self._pairs.add(index, (msg.header.stamp.sec, msg.header.stamp.nanosec), msg)
        if pair is not None:
            with self._cond:
                self._pending = pair
                self._cond.notify()

    def _intrinsics_for(self, info, shape):
        key = (info.width, info.height, tuple(info.k))
        if key != self._intrinsics_key:
            if (info.height, info.width) != tuple(shape[:2]):
                raise ValueError(f'image is {shape[1]}x{shape[0]} but camera_info is {info.width}x{info.height}')
            self._intrinsics, self._intrinsics_key = Intrinsics.from_camera_info(info), key
        return self._intrinsics

    def _work(self):
        while not self._stop.is_set():
            with self._cond:
                while self._pending is None and not self._stop.is_set():
                    self._cond.wait(0.5)
                pending, self._pending = self._pending, None
            if pending is None or not (self.runner.running or self.preview):
                continue
            try:
                self._process(*pending)
            except Exception as exc:
                if self._stop.is_set() or not self.context.ok():
                    return
                self.get_logger().error(f'Skipping frame: {exc!r}', throttle_duration_sec=10.0)

    def _process(self, image_msg, depth_msg):
        info = self._info
        if info is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if self.depth_scale != 1.0:
            depth = np.clip(depth.astype(np.float64) * self.depth_scale, 0, 65535).astype(np.uint16)
        intrinsics = self._intrinsics_for(info, bgr.shape)
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        detections = self.detector.detect(bgr, depth, intrinsics)
        self._last_detections = detections
        self.runner.observe(stamp, detections, intrinsics)
        if self.pub_annotated.get_subscription_count() > 0:
            out = self.bridge.cv2_to_imgmsg(self.annotate(bgr, detections, intrinsics), 'bgr8')
            out.header = image_msg.header
            self.pub_annotated.publish(out)

    # -- outputs -------------------------------------------------------------------------------------
    def annotate(self, bgr, detections, intrinsics):
        img = bgr.copy()
        status = self.runner.status()
        for det in detections[:1]:
            cv2.polylines(img, [np.round(det.polygon).astype(np.int32)], True, (0, 255, 0), 2)
            cv2.circle(img, tuple(int(round(c)) for c in det.center), 4, (0, 255, 0), -1)
        tip = self.mount.project(self.mount.tip_in_camera(), intrinsics)
        if tip is not None:   # where the mount model puts the fingertips: check it against the real ones
            cv2.drawMarker(img, (int(tip[0]), int(tip[1])), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        pose = self.runner.latest_pose()
        if pose is not None and self.runner.plate.center is not None:
            base_T_camera = pose @ self.mount.flange_T_camera
            for point, color in ((self.runner.plate.center, (255, 0, 255)),):
                px = self.mount.project(camera_point(base_T_camera, point), intrinsics)
                if px is not None:
                    cv2.drawMarker(img, (int(px[0]), int(px[1])), color, cv2.MARKER_TILTED_CROSS, 16, 2)
        lines = [f'{status.get("phase")}{" (dry run)" if self.config.dry_run else ""}']
        if 'error_mm' in status and status.get('running'):
            e = status['error_mm']
            lines.append(f'err {e[0]:+.0f} {e[1]:+.0f} {e[2]:+.0f} mm {status.get("error_yaw_deg", 0):+.1f} deg '
                         f'settled {status.get("settled", 0)}')
        if status.get('result'):
            lines.append(f'{status["result"]}: {status.get("message", "")}'[:70])
        for i, line in enumerate(lines):
            cv2.putText(img, line, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(img, line, (8, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def _publish_status(self):
        status = self.runner.status()
        status['mode'] = self.modes.mode
        status['detections'] = len(self._last_detections)
        self.pub_status.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        self._stop.set()
        self.runner.abort(wait=15.0)
        if self.control is not None:
            self.control.stop()
        with self._cond:
            self._cond.notify_all()
        self._worker.join(2.0)
        try:
            self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] [%(name)s] %(message)s')
    rclpy.init(args=args)
    node = None
    try:
        node = VisualServoNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

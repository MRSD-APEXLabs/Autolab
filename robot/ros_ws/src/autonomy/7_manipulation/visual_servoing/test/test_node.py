"""The ROS node with a simulated arm and detector: camera hub switching, a dry run, the outputs (needs ROS 2)."""
import inspect
import json
import os
from pathlib import Path
import time
import types

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
yaml = pytest.importorskip('yaml')
from cv_bridge import CvBridge  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from camera_perception.yolo import YoloDetector  # noqa: E402
from sim import INTRINSICS, FakeArm, detect_plate, tool_down  # noqa: E402
from visual_servoing.mount import Mount  # noqa: E402
from visual_servoing.servo import ServoConfig  # noqa: E402
from visual_servoing.servo_node import VisualServoNode, build_config, build_mount  # noqa: E402

YAML = Path(__file__).resolve().parents[1] / 'config' / 'visual_servo.yaml'
CENTER = np.array([0.42, 0.03, 0.02])
START = tool_down(0.40, 0.0, 0.35)


def flatten(params, prefix=''):
    flat = {}
    for key, value in params.items():
        if isinstance(value, dict):
            flat.update(flatten(value, f'{prefix}{key}.'))
        else:
            flat[prefix + key] = value
    return flat


def yaml_params():
    return flatten(yaml.safe_load(YAML.read_text())['/**']['ros__parameters'])


def test_yaml_matches_the_defaults():
    params = yaml_params()
    param = lambda name, default: params.get(name, default)
    assert build_config(param) == ServoConfig()          # execute: false -> dry run, like the dataclass
    mount, cad = build_mount(param), Mount()
    assert np.allclose(mount.flange_T_camera, cad.flange_T_camera)
    assert np.allclose(mount.tip_in_camera(), [0.0089, 0.0472, 0.1312])             # the measured fingertips...
    assert 0.002 < (mount.flange_T_tip - cad.flange_T_tip)[2, 3] < 0.005             # ...a little beyond the CAD tip
    yolo = {k.split('.', 1)[1] for k in params if k.startswith('yolo.')}
    assert yolo <= set(inspect.signature(YoloDetector).parameters) | {'model'}


def test_build_config_validates():
    with pytest.raises(ValueError, match='finger_axis'):
        build_config(lambda name, default: 'z' if name == 'servo.finger_axis' else default)
    with pytest.raises(ValueError, match='2 values'):
        build_config(lambda name, default: [0.1] if name == 'servo.grasp_offset' else default)
    config = build_config(lambda name, default: {'execute': True, 'gains.tol_yaw_deg': 3.0}.get(name, default))
    assert not config.dry_run and np.isclose(np.degrees(config.gains.tol_yaw), 3.0)
    mount = build_mount(lambda name, default: [10.0, 20.0, 130.0] if name == 'mount.tip_camera_mm' else default)
    assert np.allclose(mount.tip_in_camera(), [0.010, 0.020, 0.130])


class FakeDetector:
    """Sees the plate from wherever the (still) arm is, as YOLO + the depth would."""

    def __init__(self, arm, mount):
        self.arm, self.mount = arm, mount
        self.depths = []

    def detect(self, bgr, depth_mm, intrinsics):
        self.depths.append(int(depth_mm[0, 0]))
        det = detect_plate(self.arm.pose @ self.mount.flange_T_camera, CENTER, np.radians(30), intrinsics=intrinsics)
        return [det] if det is not None else []


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_dry_run_through_ros(ros):
    ns = f'/test_visual_servo_{os.getpid()}'
    overrides = {'camera_namespace': f'{ns}/zedx_nano', 'hub_node': f'{ns}/hub',
                 'select_service': f'{ns}/hub/select_zedx_nano', 'control_port': -1, 'servo.align_timeout': 1.5,
                 'depth_scale': 1.5}
    arm = FakeArm(types.SimpleNamespace(time=time.time), START, read_only=True)
    # the YAML through rcl's parser, which differs from PyYAML (an unquoted y is a bool there)
    node = VisualServoNode(arm=arm, detector=FakeDetector(arm, Mount()), namespace=ns,
                           cli_args=['--ros-args', '--params-file', str(YAML)],
                           parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
    assert node.config.finger_axis == 'y' and node.config.gripper.stop_pressure == (25, -1)
    unread = [k for k in yaml_params() if not k.startswith('yolo.') and not node.has_parameter(k)]
    assert unread == []                                   # every YAML entry is a parameter of the node

    io = rclpy.create_node('io', namespace=ns)
    calls = []
    for camera in ('zedx', 'zedx_nano'):
        io.create_service(Trigger, f'hub/select_{camera}',
                          lambda req, resp, camera=camera: (calls.append(camera), setattr(resp, 'success', True), resp)[2])
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    io.create_publisher(String, 'hub/status', latched).publish(String(data=json.dumps({'active': 'zedx'})))
    statuses, annotated = [], []
    io.create_subscription(String, 'visual_servo/status', lambda m: statuses.append(json.loads(m.data)), latched)
    io.create_subscription(Image, 'visual_servo/annotated_image', annotated.append, 2)
    pubs = {topic: io.create_publisher(kind, f'zedx_nano/{topic}', 2) for topic, kind in (
        ('left/image_rect_color', Image), ('depth/image_rect', Image), ('depth/camera_info', CameraInfo))}
    start = io.create_client(Trigger, 'visual_servo/start')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(io)
    bridge = CvBridge()
    image = np.full((INTRINSICS.height, INTRINSICS.width, 3), 90, np.uint8)
    depth = np.full((INTRINSICS.height, INTRINSICS.width), 200, np.uint16)
    last = [0.0]

    def spin_until(condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if time.monotonic() - last[0] > 1 / 15:     # the camera stack, 15 Hz
                last[0] = time.monotonic()
                header = Header(stamp=node.get_clock().now().to_msg(), frame_id='zedx_nano_left_camera_optical_frame')
                info = CameraInfo(header=header, width=INTRINSICS.width, height=INTRINSICS.height)
                info.k = [INTRINSICS.fx, 0.0, INTRINSICS.cx, 0.0, INTRINSICS.fy, INTRINSICS.cy, 0.0, 0.0, 1.0]
                pubs['depth/camera_info'].publish(info)
                for topic, msg in (('left/image_rect_color', bridge.cv2_to_imgmsg(image, 'bgr8')),
                                   ('depth/image_rect', bridge.cv2_to_imgmsg(depth, '16UC1'))):
                    msg.header = header
                    pubs[topic].publish(msg)
        return condition()

    try:
        assert spin_until(lambda: start.service_is_ready() and node._hub_active == 'zedx'
                          and io.count_subscribers(f'{ns}/zedx_nano/depth/image_rect'), 10.0)
        assert node.detector.depths == []                  # no run, no preview: the detector idles
        future = start.call_async(Trigger.Request())
        assert spin_until(future.done, 5.0) and future.result().success
        assert spin_until(lambda: statuses and statuses[-1].get('result') == 'stopped', 15.0), statuses[-1:]
        assert spin_until(lambda: calls == ['zedx_nano', 'zedx'], 3.0), calls   # selected, then restored
    finally:
        node.destroy_node()
        io.destroy_node()
        executor.shutdown()
    final = statuses[-1]
    assert final['dry_run'] and final['mode'] == 'idle' and final['detections'] == 1
    assert np.allclose(final['plate_mm'], CENTER * 1000, atol=0.5)
    assert set(node.detector.depths) == {300}              # depth_scale 1.5 applied to the 200 mm input
    assert annotated and (annotated[-1].height, annotated[-1].width) == (INTRINSICS.height, INTRINSICS.width)
    assert arm.moves == [] and arm.velocities == []

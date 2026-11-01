"""ROS 2 node: ACT placing rollouts on the xArm6, started by Autolab's manipulation_executive.

Interface (same contract as visual_servoing's visual_servo/*):

  act_inference/start     std_srvs/Trigger: start a rollout (returns at once; follow act_inference/status)
  act_inference/stop      std_srvs/Trigger: end the rollout; the arm stops where it is
  act_inference/status    std_msgs/String JSON, 2 Hz and at the end of a run (transient local):
                          running, result (done | stopped | failed), message, phase, started

The checkpoint is loaded and warmed up once, at startup. Each run switches the camera hub to the
Nano (`select_service`), reads its rectified image and depth from the zedx_nano_depth topics
(camera_source ros), connects to the arm, runs `duration_sec` and closes the arm again.
With execute:=false the arm is only read (prediction only); the run still ends `done`.
"""
from pathlib import Path
import json
import logging
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .config import InferenceConfig, MotionConfig, RolloutConfig
from .rollout import load_policy, run_act_rollout, warm_up
from .runner import ActRunner

STATUS_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DEFAULT_CKPT = Path(__file__).resolve().parents[1] / 'models' / 'xarm6_placing_act_ft_20261029'


class ActInferenceNode(Node):
    def __init__(self, policy=None, rollout_fn=run_act_rollout, **kwargs):
        """`policy` (a load_policy() result) and `rollout_fn` replace the checkpoint and the arm (tests)."""
        super().__init__('act_inference', **kwargs)
        param = lambda name, default: self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.ckpt_dir = Path(param('ckpt_dir', str(DEFAULT_CKPT))).expanduser()
        self.config = InferenceConfig(robot_ip=param('robot_ip', '192.168.1.236'),
                                      camera_source=param('camera_source', 'ros'),
                                      ros_namespace=param('ros_namespace', '/zedx_nano'),
                                      camera_host=param('camera_host', '192.168.1.101'),
                                      camera_port=int(param('camera_port', 8090)),
                                      record_image_width=int(param('record_image_width', 480)),
                                      depth_width=int(param('depth_width', 640)),
                                      depth_backend=param('depth_backend', 'neural'),
                                      depth_model=param('depth_model', 'raft-realtime'),
                                      depth_repo=param('depth_repo', ''),
                                      depth_models_dir=param('depth_models_dir', ''),
                                      max_frame_age=float(param('max_frame_age', 0.25)),
                                      preview=bool(param('preview', False)),
                                      motion=MotionConfig(loop_hz=float(param('control_hz', 50.0)),
                                                          translation_speed=float(param('translation_speed', 0.01)),
                                                          rotation_speed=float(param('rotation_speed', 0.15)),
                                                          max_tracking_error=float(param('max_tracking_error', 0.03))))
        self.rollout = RolloutConfig(ckpt_dir=self.ckpt_dir, execute=bool(param('execute', True)),
                                     duration_sec=float(param('duration_sec', 60.0)),
                                     enable_rotation=bool(param('enable_rotation', True)),
                                     query_interval=float(param('query_interval', 0.2)),
                                     prediction_timeout=float(param('prediction_timeout', 0.75)),
                                     workspace_radius=float(param('workspace_radius', 0.15)),
                                     contact_gap=float(param('contact_gap', 0.001)),
                                     contact_hold=float(param('contact_hold', 0.3)),
                                     contact_max_lag=float(param('contact_max_lag', 0.1)),
                                     settle_motion=float(param('settle_motion', 0.002)),
                                     settle_queries=int(param('settle_queries', 3)),
                                     approach_speed=float(param('approach_speed', 0.003)),
                                     approach_distance=float(param('approach_distance', 0.012)),
                                     ensemble=int(param('ensemble', 5)),
                                     final_offset=tuple(float(v) / 1000 for v in param('final_offset_mm', [0.0, 0.0, 0.0])))
        self.select_service = param('select_service', '/camera_hub/select_zedx_nano')
        self.select_timeout = float(param('select_timeout', 20.0))
        self._rollout_fn = rollout_fn

        if policy is None:
            t0 = time.monotonic()
            policy = load_policy(self.ckpt_dir)
            warm_up(policy, self.config.record_image_width)
            self.get_logger().info(f'ACT checkpoint {self.ckpt_dir.name} ready in {time.monotonic() - t0:.1f} s')
        self._policy = policy

        self.runner = ActRunner(self._run, prepare=self._prepare_camera, on_done=self._run_done)
        services = ReentrantCallbackGroup()
        self.pub_status = self.create_publisher(String, 'act_inference/status', STATUS_QOS)
        self.create_service(Trigger, 'act_inference/start', self._on_start, callback_group=services)
        self.create_service(Trigger, 'act_inference/stop', self._on_stop, callback_group=services)
        self._select_client = (self.create_client(Trigger, self.select_service, callback_group=services)
                               if self.select_service else None)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f'ACT inference: {"EXECUTE (the arm moves)" if self.rollout.execute else "prediction only (arm read-only)"}, '
            f'xArm {self.config.robot_ip}, {self.rollout.duration_sec:.1f} s per run, camera {self.config.camera_source}')

    # -- runs ----------------------------------------------------------------------------------------
    def _run(self, stop):
        return self._rollout_fn(self.config, self.rollout, policy=self._policy, stop=stop)

    def _run_done(self, status):
        log = self.get_logger().info if status['result'] == 'done' else self.get_logger().warn
        log(f'ACT run ended: {status["result"]} {status["message"]}')
        self._publish_status()

    def _on_start(self, request, response):
        response.success, response.message = self.runner.start()
        if response.success:
            self._publish_status()   # a run that fails within one timer period still shows running first
        return response

    def _on_stop(self, request, response):
        self.runner.abort()
        response.success, response.message = True, 'stopping'
        return response

    def _prepare_camera(self, check_abort):
        """Runs on the run's thread: switch the hub to the Nano and wait until it streams."""
        if self._select_client is None:
            return
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

    # -- outputs -------------------------------------------------------------------------------------
    def _publish_status(self):
        self.pub_status.publish(String(data=json.dumps(self.runner.status())))

    def destroy_node(self):
        self.runner.abort(wait=15.0)
        super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] [%(name)s] %(message)s')
    rclpy.init(args=args)
    node = None
    try:
        node = ActInferenceNode()
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

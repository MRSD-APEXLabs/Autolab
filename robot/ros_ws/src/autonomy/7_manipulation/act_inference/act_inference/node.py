"""ROS 2 node: ACT placing rollouts on the xArm6, started by Autolab's manipulation_executive.

Interface (same contract as visual_servoing's visual_servo/*):

  act_inference/start/{ot2,shaker,base} std_srvs/Trigger: start one placement (returns at once)
  act_inference/start     std_srvs/Trigger: legacy alias for start/ot2
  act_inference/stop      std_srvs/Trigger: end the rollout; the arm stops where it is
  act_inference/status    std_msgs/String JSON, 2 Hz and at the end of a run (transient local):
                          running, result (done | stopped | failed), message, phase, started, target

Each target has its own placing policy (`ckpt_dir` for OT-2, `shaker_ckpt_dir`, `base_ckpt_dir`); all are
loaded and warmed up once, at startup. Each run switches the camera hub to the Nano (`select_service`), reads
its rectified image and depth from the zedx_nano_depth topics (camera_source ros), connects to the arm, runs
up to `duration_sec` and closes the arm again. OT-2/shaker first align yaw to 90 degrees (their demos end
there), then open the gripper and lift 100 mm after a successful policy run. Base first moves straight to
`base_start_pose_mm_deg` (~80 mm above the bracket at yaw ~0, where its demos started) instead of aligning
yaw, then runs its policy and lifts 60 mm (`base_mode:=policy`). `base_mode:=scripted` (the default) places on
the base without camera or policy: move to a fixed XYZ/RPY probing start, descend until the
wrist F/T sensor feels the deck (Fz), slide along -X until it feels the ridge (Fx/Fy), then release and lift 60 mm;
`base_debug:=true` logs every tick of it. All targets share one runner, stop service, and status topic. With
execute:=false the arm is only read.
"""
from dataclasses import replace
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
from controller_manager_msgs.srv import ListControllers, SwitchController

from .config import InferenceConfig, MotionConfig, RolloutConfig
from .base_placement import (BASE_POLICY_START_MM_DEG, BasePlacementConfig, align_policy_yaw,
                             move_to_base_policy_start, run_base_placement)
from .post_placement import release_and_lift
from .rollout import load_policy, run_act_rollout, warm_up
from .runner import ActRunner
from .ownership import DriverHandoff
from .robot import XArm6Robot

STATUS_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MODELS = Path(__file__).resolve().parents[1] / 'models'
TARGETS = ('ot2', 'shaker', 'base')
DEFAULT_CKPTS = {'ot2': MODELS / 'xarm6_placing_act_ft_all_20261105b',
                 'shaker': MODELS / 'xarm6_shaker_act_ft_20261110',
                 'base': MODELS / 'xarm6_base_act_ft_20261108b'}
DEFAULT_CKPT = DEFAULT_CKPTS['ot2']
BASE_LIFT_MM = 60.0
# BasePlacementConfig F/T seating fields, exposed as base_<name> parameters.
BASE_SEATING_PARAMS = {'probe_speed_mm_s': float, 'probe_depth_mm': float, 'touch_force_n': float, 'slide_speed_mm_s': float,
                       'slide_max_mm': float, 'ridge_force_n': float, 'force_count': int, 'settle_sec': float,
                       'baseline_sec': float, 'debug': bool, 'debug_dir': str}


class ActInferenceNode(Node):
    def __init__(self, policy=None, rollout_fn=run_act_rollout, base_move_fn=run_base_placement,
                 yaw_align_fn=align_policy_yaw, post_placement_fn=release_and_lift,
                 base_start_fn=move_to_base_policy_start, **kwargs):
        """`policy` (a load_policy() result, or a dict of them per target) and `rollout_fn` replace the
        checkpoints and the arm (tests)."""
        super().__init__('act_inference', **kwargs)
        param = lambda name, default: self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        self.ckpt_dirs = {target: Path(param('ckpt_dir' if target == 'ot2' else f'{target}_ckpt_dir',
                                             str(DEFAULT_CKPTS[target]))).expanduser() for target in TARGETS}
        self.ckpt_dir = self.ckpt_dirs['ot2']
        self.base_mode = param('base_mode', 'scripted')
        if self.base_mode not in ('policy', 'scripted'):
            raise ValueError("base_mode must be 'policy' or 'scripted'")
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
        base_radius = float(param('base_workspace_radius', 0.15))   # the bracket is ~80 mm below the start pose
        self.rollouts = {target: replace(self.rollout, ckpt_dir=self.ckpt_dirs[target],
                                         **({'workspace_radius': base_radius} if target == 'base' else {}))
                         for target in TARGETS}
        self.select_service = param('select_service', '/camera_hub/select_zedx_nano')
        self.select_timeout = float(param('select_timeout', 20.0))
        driver_prefix = str(param('driver_control_prefix', '/xarm/sdk_control')).rstrip('/')
        arm_controllers = list(param('arm_controllers', ['xarm6_traj_controller']))
        self._rollout_fn = rollout_fn
        self.base_placement = BasePlacementConfig(
            pose_mm_deg=tuple(param('base_pose_mm_deg', list(BasePlacementConfig().pose_mm_deg))),
            speed_mm_s=float(param('base_speed_mm_s', 15.0)),
            rotation_speed_deg_s=float(param('base_rotation_speed_deg_s', 6.0)),
            timeout_sec=float(param('base_timeout_sec', 180.0)),
            **{name: kind(param(f'base_{name}', getattr(BasePlacementConfig, name)))
               for name, kind in BASE_SEATING_PARAMS.items()})
        self.base_start = replace(self.base_placement, pose_mm_deg=tuple(
            float(v) for v in param('base_start_pose_mm_deg', list(BASE_POLICY_START_MM_DEG))))
        self._base_move_fn = base_move_fn
        self._base_start_fn = base_start_fn
        self._yaw_align_fn = yaw_align_fn
        self._post_placement_fn = post_placement_fn
        self._handoff_robot = None

        needed = [t for t in TARGETS if t != 'base' or self.base_mode == 'policy']
        if policy is not None:
            self._policies = {t: policy[t] if isinstance(policy, dict) else policy for t in needed}
        else:
            loaded = {}   # one load per checkpoint folder, even if targets share it
            for target in needed:
                directory = self.ckpt_dirs[target].resolve()
                if directory not in loaded:
                    t0 = time.monotonic()
                    loaded[directory] = load_policy(directory)
                    warm_up(loaded[directory], self.config.record_image_width)
                    self.get_logger().info(f'ACT checkpoint {directory.name} ready in {time.monotonic() - t0:.1f} s')
            self._policies = {t: loaded[self.ckpt_dirs[t].resolve()] for t in needed}
        self.get_logger().info('Placing policies: ' + ', '.join(f'{t}={self.ckpt_dirs[t].name}' for t in needed)
                               + ('' if 'base' in needed else ', base=scripted pose'))

        self.runner = ActRunner(self._run, prepare=self._prepare_run, on_done=self._run_done)
        services = ReentrantCallbackGroup()
        self.pub_status = self.create_publisher(String, 'act_inference/status', STATUS_QOS)
        self.create_service(Trigger, 'act_inference/start', self._on_start, callback_group=services)
        for target in ('ot2', 'shaker', 'base'):
            self.create_service(Trigger, f'act_inference/start/{target}',
                                lambda request, response, target=target: self._on_start(request, response, target),
                                callback_group=services)
        self.create_service(Trigger, 'act_inference/stop', self._on_stop, callback_group=services)
        self._select_client = (self.create_client(Trigger, self.select_service, callback_group=services)
                               if self.select_service else None)
        self._driver_clients = {name: self.create_client(Trigger, f'{driver_prefix}/{name}', callback_group=services)
                                for name in ('status', 'acquire', 'release')}
        self._list_controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers', callback_group=services)
        self._switch_controllers = self.create_client(
            SwitchController, '/controller_manager/switch_controller', callback_group=services)
        self.handoff = DriverHandoff(
            status=self._driver_status,
            list_controllers=lambda check: self._call(self._list_controllers, ListControllers.Request(), check).controller,
            switch=self._switch_arm_controllers,
            acquire=lambda check: self._call(self._driver_clients['acquire'], Trigger.Request(), check, mutation=True).success,
            release=lambda check: self._call(self._driver_clients['release'], Trigger.Request(), check, mutation=True),
            ready=self._ready_for_ros, controllers=arm_controllers, robot_ip=self.config.robot_ip)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f'ACT inference: {"EXECUTE (the arm moves)" if self.rollout.execute else "prediction only (arm read-only)"}, '
            f'xArm {self.config.robot_ip}, {self.rollout.duration_sec:.1f} s per run, camera {self.config.camera_source}')

    # -- runs ----------------------------------------------------------------------------------------
    def _run(self, stop):
        target = self.runner.status()['target']
        if target == 'base' and self.base_mode == 'scripted':
            result = self._base_move_fn(self.config, self.base_placement, execute=self.rollout.execute, stop=stop)
            if result == 'done' and not stop.is_set():
                result = self._post_placement_fn(self.config, execute=self.rollout.execute, stop=stop,
                                                 lift_mm=BASE_LIFT_MM)
        else:
            result = self._run_policy(stop, target)
        if result == 'done' and not stop.is_set() and self.rollout.execute:
            self.handoff.finish(self.runner._check_abort)
        return 'stopped' if stop.is_set() else result

    def _run_policy(self, stop, target='ot2'):
        # OT-2 / shaker demos end at yaw ~90 degrees; base demos start ~80 mm above the bracket at yaw ~0.
        if target == 'base':
            prepare = lambda: self._base_start_fn(self.config, self.base_start, execute=self.rollout.execute, stop=stop)
        else:
            prepare = lambda: self._yaw_align_fn(self.config, execute=self.rollout.execute, stop=stop)
        if prepare() == 'stopped' or stop.is_set():
            return 'stopped'
        result = self._rollout_fn(self.config, self.rollouts[target], policy=self._policies[target], stop=stop)
        if result != 'done':
            return result
        if stop.is_set():
            return 'stopped'
        lift = {'lift_mm': BASE_LIFT_MM} if target == 'base' else {}
        return self._post_placement_fn(self.config, execute=self.rollout.execute, stop=stop, **lift)

    def _run_done(self, status):
        # one call site per severity: rclpy refuses a severity change at the same call site
        text = f'Placement {status["target"]} ended: {status["result"]} {status["message"]}'
        if status['result'] == 'done':
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self._publish_status()

    def _on_start(self, request, response, target='ot2'):
        response.success, response.message = self.runner.start(target=target)
        if response.success:
            self._publish_status()   # a run that fails within one timer period still shows running first
        return response

    def _on_stop(self, request, response):
        self.runner.abort()
        response.success, response.message = True, 'stopping'
        return response

    def _prepare_run(self, check_abort):
        self._verify_motion_ownership(check_abort)
        if self.runner.status()['target'] != 'base' or self.base_mode == 'policy':
            self._prepare_camera(check_abort)

    def _verify_motion_ownership(self, check_abort):
        check_abort()
        if not self.rollout.execute:
            return
        if self.get_node_names().count('act_inference') > 1:
            raise RuntimeError('Multiple act_inference nodes are running; stop duplicate nodes before execution')
        self.handoff.acquire(check_abort)

    def _call(self, client, request, check_abort, mutation=False):
        check_abort()
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{client.srv_name} unavailable; rebuild/restart the patched xarm_controller')
        check_abort()
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done():
            # Once a switch/acquire/release request is sent, retain its outcome even
            # if stop arrives: never lose track of an acquired driver lease.
            if not mutation:
                check_abort()
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{client.srv_name} timed out; ownership requires inspection')
            time.sleep(.02)
        return future.result()

    def _driver_status(self, check_abort):
        response = self._call(self._driver_clients['status'], Trigger.Request(), check_abort)
        if not response.success:
            raise RuntimeError(f'Driver ownership status failed: {response.message}')
        return json.loads(response.message)

    def _switch_arm_controllers(self, stopped, started, check_abort):
        request = SwitchController.Request()
        request.deactivate_controllers = list(stopped)
        request.activate_controllers = list(started)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        return self._call(self._switch_controllers, request, check_abort, mutation=True).ok

    def _ready_for_ros(self, check_abort):
        # This runs only after successful SDK motion. Never clear errors or enable
        # the robot after a failed/stopped run.
        if self._handoff_robot is None:
            self._handoff_robot = XArm6Robot(self.config.robot_ip, self.config.motion)
        robot = self._handoff_robot
        try:
            check_abort()
            robot.connect()
            robot.initialize_target_pose()
            check_abort()
            robot.enable()
            check_abort()
            robot.hand_back()
            # Match visual_servoing: keep the SDK connection alive in ready mode 1
            # through ROS handback, rather than disconnecting at the mode boundary.
        except Exception:
            robot.close()
            self._handoff_robot = None
            raise

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
        status = self.runner.status()
        status['sdk_control_held'] = self.handoff.held
        self.pub_status.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        self.runner.abort(wait=15.0)
        if self._handoff_robot is not None:
            self._handoff_robot.close()
            self._handoff_robot = None
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

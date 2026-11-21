"""ROS 2 node: ACT placing rollouts on the xArm6, started by Autolab's manipulation_executive.

Interface (same contract as visual_servoing's visual_servo/*):

  act_inference/start/{ot2,shaker,base} std_srvs/Trigger: start one placement (returns at once)
  act_inference/start     std_srvs/Trigger: legacy alias for start/ot2
  act_inference/stop      std_srvs/Trigger: end the rollout; the arm stops where it is
  act_inference/status    std_msgs/String JSON, 2 Hz and at the end of a run (transient local):
                          running, result (done | stopped | failed), message, phase, started, target
  act_inference/jog/{x,y,z}_{plus,minus} std_srvs/Trigger: move the TCP `jog_step_mm` (10) along that xArm
                          base-frame axis at `jog_speed_mm_s` (10), holding orientation, with the same
                          ROS-controller/SDK handoff as a placement; answers when the jog has ended (target
                          jog_<axis>_<direction> in the status). Refused while a run is active, the planner is
                          planning/executing (/planning_state) or the visual servo runs (/visual_servo/status).
  act_inference/jog/rz_{plus,minus} std_srvs/Trigger: the same, turning the TCP (and the wrist camera)
                          `jog_step_deg` (10) about the xArm base-frame z axis at `jog_rotation_speed_deg_s` (10),
                          holding XYZ.
  act_inference/jog/home  std_srvs/Trigger: the same handoff and refusals, moving the TCP straight to the
                          home_offset pose (the controller's forward kinematics of `home_joints`, the planner's
                          plan_home_offset joints) at `home_speed_mm_s` (40) and `home_rotation_speed_deg_s` (30),
                          the controller's IK following it. No planning: nothing on the way is avoided.
  act_inference/clear_errors std_srvs/Trigger: clear the xArm's error and warning and enable it again (motors on,
                          servo mode 1, state 0; no pose is commanded, the arm holds where it is), then give it
                          back to the planner: hand back the SDK control a failed jog/teleop/placement kept, or
                          re-activate the ROS arm controllers the driver stopped at the fault. Same refusals as a
                          jog; target clear_errors in the status.
  /joy                    sensor_msgs/Joy, the navigation Bluetooth gamepad: arm teleop. Pressing R2 (RT,
                          `teleop_enable_axis` 5) starts a run (target jog_teleop, same refusals and handoff as a
                          jog); while R2 stays held the sticks drive the TCP in the xArm base frame at up to
                          `teleop_speed_mm_s` (40) within `teleop_radius_mm` (150) of where it started, and turn it
                          about the base z axis at up to `teleop_rotation_speed_deg_s` (30): right stick up/down =
                          +x/-x, right stick left/right = +y/-y, left stick up/down = +z/-z, left stick left/right =
                          +rz/-rz (`teleop_axes`, `teleop_signs`). While Y (`teleop_home_button` 3) is held too,
                          the TCP drives straight to the home_offset pose at those speeds instead, and the sticks
                          then work around where it stopped. Releasing R2, or /joy going quiet for
                          `teleop_joy_timeout` (0.5 s), ends the run and hands the arm back to ROS. While L2 (the
                          base-drive deadman, `teleop_block_axis` 2) is held the arm holds still. act_inference/stop
                          also ends it. Pressing Back (`clear_errors_button` 6, no R2 needed) does
                          act_inference/clear_errors.

Each target has its own placing policy (`ckpt_dir` for OT-2, `shaker_ckpt_dir`, `base_ckpt_dir`); all are
loaded and warmed up once, at startup. Each run switches the camera hub to the Nano (`select_service`), reads
its rectified image and depth from the zedx_nano_depth topics (camera_source ros), connects to the arm, runs
up to `duration_sec` and closes the arm again. OT-2/shaker first align yaw to 90 degrees (their demos end
there), then open the gripper and lift 100 mm (25 mm/s) after a successful policy run. Shaker moves at
`shaker_approach_speed` (1.5 mm/s) below `shaker_slow_below_z_mm` (110 mm TCP height, ~47 mm above the clamp seat),
so the plate reaches the clamp's bounds slowly. Base first moves straight to
`base_start_pose_mm_deg` (~80 mm above the bracket at yaw ~0, where its demos started) instead of aligning
yaw, then runs its policy and lifts 60 mm (`base_mode:=policy`). `base_mode:=scripted` (the default) places on
the base without camera or policy: move to a fixed XYZ/RPY probing start (`base_approach_speed_mm_s` 150 to
`base_approach_clearance_mm` 20 above it, then `base_speed_mm_s` 40, both ramped at `base_accel_mm_s2` 500),
descend until the wrist F/T sensor feels the deck (Fz), slide along -X until it feels the ridge (Fx/Fy) (the
first `base_probe_fast_mm` 3.5 / `base_slide_fast_mm` 12 of them at `base_fast_speed_mm_s` 10, the rest at
`base_probe_speed_mm_s` 2 / `base_slide_speed_mm_s` 4), then release (`base_open_wait_sec` 1.5 for the jaws)
and lift 60 mm
(both modes at `base_lift_speed_mm_s`, 150, ramped at `base_accel_mm_s2`);
`base_debug:=true` logs every tick of it. All targets share one runner, stop service, and status topic. With
execute:=false the arm is only read.
"""
from dataclasses import replace
from pathlib import Path
import json
import logging
import math
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers, SwitchController

from .config import InferenceConfig, MotionConfig, RolloutConfig
from .base_placement import (BASE_ARRIVAL_SEC, BASE_POLICY_START_MM_DEG, HOME_OFFSET_JOINTS, TELEOP_HOME,
                             BasePlacementConfig,
                             align_policy_yaw, clear_arm_errors, jog_base_rz, jog_base_xyz, move_home,
                             move_to_base_policy_start, run_base_placement, teleop_base_xyz)
from .post_placement import release_and_lift
from .rollout import load_policy, run_act_rollout, warm_up
from .runner import ActRunner
from .ownership import DriverHandoff
from .robot import XArm6Robot

STATUS_QOS = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MODELS = Path(__file__).resolve().parents[1] / 'models'
TARGETS = ('ot2', 'shaker', 'base')
DEFAULT_CKPTS = {'ot2': MODELS / 'xarm6_placing_act_ft_all_20261105b',
                 'shaker': MODELS / 'xarm6_shaker_act_ft_vg_20261120',
                 'base': MODELS / 'xarm6_base_act_ft_20261108b'}
DEFAULT_CKPT = DEFAULT_CKPTS['ot2']
BASE_LIFT_MM = 60.0
ACT_LIFT_SPEED_MM_S = 25.0   # OT-2/shaker lift after the policy (2.5x the ACT translation_speed default, 10)
# BasePlacementConfig approach, lift and F/T seating fields, exposed as base_<name> parameters.
BASE_SEATING_PARAMS = {'approach_speed_mm_s': float, 'approach_clearance_mm': float, 'lift_speed_mm_s': float,
                       'accel_mm_s2': float, 'open_wait_sec': float,
                       'probe_speed_mm_s': float, 'probe_depth_mm': float, 'touch_force_n': float, 'slide_speed_mm_s': float,
                       'slide_max_mm': float, 'ridge_force_n': float, 'fast_speed_mm_s': float, 'probe_fast_mm': float,
                       'slide_fast_mm': float, 'force_count': int, 'settle_sec': float,
                       'baseline_sec': float, 'debug': bool, 'debug_dir': str}


def base_lift_args(placement):
    """release_and_lift arguments after a base placement (BasePlacementConfig `placement`)."""
    return {'lift_mm': BASE_LIFT_MM, 'speed_mm_s': placement.lift_speed_mm_s, 'accel_mm_s2': placement.accel_mm_s2,
            'open_wait_sec': placement.open_wait_sec, 'report_type': 'real', 'arrival_sec': BASE_ARRIVAL_SEC}


TELEOP_TARGET = 'jog_teleop'
CLEAR_ERRORS_TARGET = 'clear_errors'


def stick(axes, index, deadband):
    """Gamepad stick axis `index` without its deadband, rescaled to [-1, 1]; 0 if missing or non-finite."""
    value = float(axes[index]) if 0 <= index < len(axes) else 0.0
    if not math.isfinite(value) or abs(value) < deadband:
        return 0.0
    return max(-1.0, min(1.0, (value - math.copysign(deadband, value)) / (1.0 - deadband)))


def button_pressed(buttons, index):
    """Whether gamepad button `index` is held; False if missing."""
    return 0 <= index < len(buttons) and bool(buttons[index])


def trigger_pressed(axes, index, threshold):
    """Whether gamepad trigger axis `index` (+1 released .. -1 fully pressed) is pressed past `threshold` (0..1)."""
    value = float(axes[index]) if 0 <= index < len(axes) else 1.0
    return math.isfinite(value) and (1.0 - value) / 2.0 > threshold


class ActInferenceNode(Node):
    def __init__(self, policy=None, rollout_fn=run_act_rollout, base_move_fn=run_base_placement,
                 yaw_align_fn=align_policy_yaw, post_placement_fn=release_and_lift,
                 base_start_fn=move_to_base_policy_start, jog_fn=jog_base_xyz, jog_rz_fn=jog_base_rz,
                 home_fn=move_home, teleop_fn=teleop_base_xyz, clear_errors_fn=clear_arm_errors, **kwargs):
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
        # Shaker: the clamp's bounds stand ~8 mm above the seat (TCP z ~58-67 mm in every demo). Below
        # shaker_slow_below_z_mm the arm moves at shaker_approach_speed, so the plate crosses them on fresh images.
        shaker = {'approach_speed': float(param('shaker_approach_speed', 0.0015)),
                  'slow_below_z': float(param('shaker_slow_below_z_mm', 110.0)) / 1000}
        overrides = {'base': {'workspace_radius': base_radius}, 'shaker': shaker}
        self.rollouts = {target: replace(self.rollout, ckpt_dir=self.ckpt_dirs[target], **overrides.get(target, {}))
                         for target in TARGETS}
        self.select_service = param('select_service', '/camera_hub/select_zedx_nano')
        self.select_timeout = float(param('select_timeout', 20.0))
        driver_prefix = str(param('driver_control_prefix', '/xarm/sdk_control')).rstrip('/')
        arm_controllers = list(param('arm_controllers', ['xarm6_traj_controller']))
        self._rollout_fn = rollout_fn
        self.base_placement = BasePlacementConfig(
            pose_mm_deg=tuple(param('base_pose_mm_deg', list(BasePlacementConfig().pose_mm_deg))),
            speed_mm_s=float(param('base_speed_mm_s', 40.0)),
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
        self._jog_fn = jog_fn
        self._jog_rz_fn = jog_rz_fn
        self.jog_step_mm = float(param('jog_step_mm', 10.0))
        self.jog_speed_mm_s = float(param('jog_speed_mm_s', 10.0))
        self.jog_step_deg = float(param('jog_step_deg', 10.0))
        self.jog_rotation_speed_deg_s = float(param('jog_rotation_speed_deg_s', 10.0))
        self._home_fn = home_fn
        self.home_joints = [float(v) for v in param('home_joints', list(HOME_OFFSET_JOINTS))]   # rad
        self.home_speed_mm_s = float(param('home_speed_mm_s', 40.0))
        self.home_rotation_speed_deg_s = float(param('home_rotation_speed_deg_s', 30.0))
        self._teleop_fn = teleop_fn
        self.teleop_joy_topic = param('teleop_joy_topic', '/joy')
        self.teleop_enable_axis = int(param('teleop_enable_axis', 5))   # R2 / RT: hold to drive the arm
        self.teleop_block_axis = int(param('teleop_block_axis', 2))     # L2 / LT: base-drive deadman, arm holds still
        self.teleop_home_button = int(param('teleop_home_button', 3))   # Y, with R2: drive to home_offset
        self._clear_errors_fn = clear_errors_fn
        self.clear_errors_button = int(param('clear_errors_button', 6))   # Back: clear the arm's errors, re-enable it
        self.teleop_trigger_threshold = float(param('teleop_trigger_threshold', 0.6))
        self.teleop_axes = [int(v) for v in param('teleop_axes', [4, 3, 1, 0])]   # -> x, y, z, rz: RY, RX, LY, LX
        self.teleop_signs = [float(v) for v in param('teleop_signs', [1.0, 1.0, 1.0, 1.0])]
        self.teleop_deadband = float(param('teleop_deadband', 0.10))
        self.teleop_joy_timeout = float(param('teleop_joy_timeout', 0.5))
        self.teleop_speed_mm_s = float(param('teleop_speed_mm_s', 40.0))
        self.teleop_rotation_speed_deg_s = float(param('teleop_rotation_speed_deg_s', 30.0))
        self.teleop_radius_mm = float(param('teleop_radius_mm', 150.0))
        if len(self.teleop_axes) != 4 or len(self.teleop_signs) != 4:
            raise ValueError('teleop_axes and teleop_signs must have 4 entries (x, y, z, rz)')
        self._joy_lock = threading.Lock()
        self._joy = (-math.inf, [], [])   # (receipt time, axes, buttons)
        self._teleop_released = False   # a run starts only on a fresh R2 press seen after a release
        self._clear_errors_held = True   # likewise Back: one clear per press, seen released first
        self._planning_state = None
        self._servo_running = False
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
        jogs = {}   # service suffix -> (move(stop), what a completed move did)
        for index, axis in enumerate('xyz'):
            for sign, direction in ((1.0, 'plus'), (-1.0, 'minus')):
                offset = tuple(sign * self.jog_step_mm / 1000 if i == index else 0.0 for i in range(3))
                jogs[f'{axis}_{direction}'] = (
                    lambda stop, offset=offset: self._jog_fn(self.config, offset, self.jog_speed_mm_s,
                                                             execute=self.rollout.execute, stop=stop),
                    f'moved {self.jog_step_mm:g} mm')
        for sign, direction in ((1.0, 'plus'), (-1.0, 'minus')):
            jogs[f'rz_{direction}'] = (
                lambda stop, angle=sign * self.jog_step_deg: self._jog_rz_fn(
                    self.config, angle, self.jog_rotation_speed_deg_s, execute=self.rollout.execute, stop=stop),
                f'rotated {self.jog_step_deg:g} deg')
        jogs['home'] = (lambda stop: self._home_fn(self.config, self.home_joints, self.home_speed_mm_s,
                                                   self.home_rotation_speed_deg_s, execute=self.rollout.execute,
                                                   stop=stop),
                        'at home_offset')
        for suffix, (move, done) in jogs.items():
            self.create_service(Trigger, f'act_inference/jog/{suffix}',
                                lambda request, response, name=f'jog_{suffix}', move=move, done=done:
                                    self._on_jog(request, response, name, move, done),
                                callback_group=services)
        self.create_service(Trigger, 'act_inference/clear_errors',
                            lambda request, response: self._on_jog(
                                request, response, CLEAR_ERRORS_TARGET, None, 'errors cleared, arm enabled',
                                run=self._clear_errors, prepare=self._verify_single_node,
                                refusal='not clearing errors'),
                            callback_group=services)
        self.create_subscription(String, '/planning_state', self._on_planning_state, 10, callback_group=services)
        self.create_subscription(String, '/visual_servo/status', self._on_servo_status, STATUS_QOS,
                                 callback_group=services)
        # one /joy message at a time: a press must start exactly one run
        self.create_subscription(Joy, self.teleop_joy_topic, self._on_joy, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())
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
        if target == 'base' and self._handoff_robot is not None:
            # A base run's own placement, F/T and release connections use the 'real' report socket; with the
            # handback connection on it too, the controller dropped the release's report socket (2026-11-21).
            # The handback reconnects after the run.
            self._handoff_robot.close()
            self._handoff_robot = None
        if target == 'base' and self.base_mode == 'scripted':
            result = self._base_move_fn(self.config, self.base_placement, execute=self.rollout.execute, stop=stop)
            if result == 'done' and not stop.is_set():
                result = self._post_placement_fn(self.config, execute=self.rollout.execute, stop=stop,
                                                 **base_lift_args(self.base_placement))
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
        lift = base_lift_args(self.base_placement) if target == 'base' else {'speed_mm_s': ACT_LIFT_SPEED_MM_S}
        return self._post_placement_fn(self.config, execute=self.rollout.execute, stop=stop, **lift)

    def _jog(self, stop, move):
        result = move(stop)
        if result == 'done' and not stop.is_set() and self.rollout.execute:
            self.handoff.finish(self.runner._check_abort)
        return 'stopped' if stop.is_set() else result

    def _clear_errors(self, stop):
        result = self._clear_errors_fn(self.config, execute=self.rollout.execute, stop=stop)
        if result == 'done' and not stop.is_set() and self.rollout.execute:
            if self.handoff.held:
                self.handoff.finish(self.runner._check_abort)   # a failed SDK run kept the arm: hand it back
            else:
                self._activate_arm_controllers(self.runner._check_abort)
        return 'stopped' if stop.is_set() else result

    def _activate_arm_controllers(self, check_abort):
        """A fault under ROS control makes the driver stop the arm controllers, and nothing starts them again
        (the planner does, at its next command); start them now that the arm is ready. They hold where it is."""
        states = {controller.name: controller.state for controller in self.handoff.list_controllers(check_abort)}
        inactive = [name for name in self.handoff.controllers if states.get(name) != 'active']
        if inactive and not self._switch_arm_controllers([], inactive, check_abort):
            raise RuntimeError(f'xArm errors cleared and motors enabled, but {", ".join(inactive)} did not start '
                               f'(driver: {self.handoff.status(check_abort)})')

    def _on_jog(self, request, response, name, move, done, run=None, prepare=None, refusal='not jogging'):
        """Start `run` (default: `move` and the handback) as target `name`, answering when it has ended."""
        busy = self._arm_busy()
        if busy:
            response.success, response.message = False, f'{refusal}: {busy}'
            return response
        response.success, response.message = self.runner.start(
            target=name, run=run or (lambda stop: self._jog(stop, move)),
            prepare=prepare or self._verify_motion_ownership)
        if not response.success:
            return response
        self._publish_status()
        started = self.runner.status()['started']
        while (status := self.runner.status())['running'] and status['started'] == started:
            time.sleep(.05)
        response.success = status['result'] == 'done'
        response.message = (f'{name}: {done}' if response.success
                            else f'{name} {status["result"]}: {status["message"]}')
        return response

    # -- gamepad arm teleop --------------------------------------------------------------------------
    def _on_joy(self, msg):
        axes, buttons = list(msg.axes), list(msg.buttons)
        with self._joy_lock:
            self._joy = (time.monotonic(), axes, buttons)
        clear = button_pressed(buttons, self.clear_errors_button)
        if clear and not self._clear_errors_held:
            self._start_clear_errors()
        self._clear_errors_held = clear
        if not trigger_pressed(axes, self.teleop_enable_axis, self.teleop_trigger_threshold):
            self._teleop_released = True
            return
        if not self._teleop_released:
            return
        self._teleop_released = False   # one start attempt per R2 press
        busy = self._arm_busy()
        if busy:
            self.get_logger().warn(f'Arm teleop refused: {busy}')
            return
        ok, message = self.runner.start(target=TELEOP_TARGET, run=self._teleop, prepare=self._verify_motion_ownership)
        if not ok:
            self.get_logger().warn(f'Arm teleop refused: {message}')
            return
        self.get_logger().info('Arm teleop started: sticks drive the TCP while R2 is held')
        self._publish_status()

    def _start_clear_errors(self):
        busy = self._arm_busy()
        if busy:
            self.get_logger().warn(f'Clear errors refused: {busy}')
            return
        ok, message = self.runner.start(target=CLEAR_ERRORS_TARGET, run=self._clear_errors,
                                        prepare=self._verify_single_node)
        if not ok:
            self.get_logger().warn(f'Clear errors refused: {message}')
            return
        self.get_logger().info('Clearing the xArm errors (gamepad Back)')
        self._publish_status()

    def _teleop_command(self):
        """(x, y, z, rz) stick fractions for the teleop loop, TELEOP_HOME while Y is held; None once R2 is
        released or /joy has gone quiet."""
        with self._joy_lock:
            stamp, axes, buttons = self._joy
        if (time.monotonic() - stamp > self.teleop_joy_timeout
                or not trigger_pressed(axes, self.teleop_enable_axis, self.teleop_trigger_threshold)):
            return None
        if trigger_pressed(axes, self.teleop_block_axis, self.teleop_trigger_threshold):
            return (0.0, 0.0, 0.0, 0.0)   # L2 drives the base: never move the base and the arm together
        if button_pressed(buttons, self.teleop_home_button):
            return TELEOP_HOME
        return tuple(sign * stick(axes, index, self.teleop_deadband)
                     for index, sign in zip(self.teleop_axes, self.teleop_signs))

    def _teleop(self, stop):
        result = self._teleop_fn(self.config, self._teleop_command, self.teleop_speed_mm_s,
                                 self.teleop_radius_mm / 1000, execute=self.rollout.execute, stop=stop,
                                 rotation_speed_deg_s=self.teleop_rotation_speed_deg_s, home_joints=self.home_joints)
        if result == 'done' and not stop.is_set() and self.rollout.execute:
            self.handoff.finish(self.runner._check_abort)
        return 'stopped' if stop.is_set() else result

    def _arm_busy(self):
        """Why another node is driving the arm now, or '' (a node that has exited does not count)."""
        names = self.get_node_names()
        if self._planning_state in ('PLANNING', 'EXECUTING') and 'move_to_pose_node' in names:
            return f'the planner is {self._planning_state.lower()}'
        if self._servo_running and 'visual_servo' in names:
            return 'the visual servo is running'
        return ''

    def _on_planning_state(self, msg):
        self._planning_state = msg.data

    def _on_servo_status(self, msg):
        self._servo_running = bool(json.loads(msg.data).get('running'))

    def _run_done(self, status):
        # one call site per severity: rclpy refuses a severity change at the same call site
        kind = ('Jog' if str(status['target']).startswith('jog_')
                else 'Arm' if status['target'] == CLEAR_ERRORS_TARGET else 'Placement')
        text = f'{kind} {status["target"]} ended: {status["result"]} {status["message"]}'
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

    def _verify_single_node(self, check_abort):
        check_abort()
        if self.rollout.execute and self.get_node_names().count('act_inference') > 1:
            raise RuntimeError('Multiple act_inference nodes are running; stop duplicate nodes before execution')

    def _verify_motion_ownership(self, check_abort):
        self._verify_single_node(check_abort)
        if self.rollout.execute:
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
        # This runs only after successful SDK motion or an explicit clear_errors. Never clear
        # errors here, nor enable the robot after a failed/stopped run.
        if self._handoff_robot is None:
            # 'real' (100 Hz) reports: initialize_target_pose() checks the report against a fresh query, and a
            # 5 Hz 'rich' report can still show the end of a just-finished move (place_base's fast lift).
            self._handoff_robot = XArm6Robot(self.config.robot_ip, self.config.motion, report_type='real')
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

import json
import time

import rclpy
from rclpy.node import Node
from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from routine_executor.state_machine import RoutineStateMachine
from routine_executor import shaker_step
from routine_executor.shaker_step import ShakerPhase
from routine_executor.step_config import STEPS, KNOWN_STEPS, make_shaker_msg

STEP_MSG_CLASSES = {
    'manipulation': ManipulationCommand,
    'lab_machine': LabMachineCommand,
    'string': String,
    'pose': PoseStamped,
}

NAV_CANCEL_TOPIC = '/nav/cancel'

STATUS_NAMES = {Status.RUNNING: 'RUNNING', Status.SUCCESS: 'SUCCESS', Status.FAILURE: 'FAILURE'}

# Steps whose failure ends the run (never re-dispatched); operator retry needs force.
NO_RETRY_STEPS = frozenset(name for name, cfg in STEPS.items() if not cfg.get('retryable', True))

STATUS_QOS = rclpy.qos.QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RoutineExecutorNode(Node):
    def __init__(self):
        super().__init__('routine_executor')

        self._sm = RoutineStateMachine([])
        self._robot = 'robot_1'
        self._listening = False    # ignore status msgs until after command is published
        self._seen_running = False  # ignore FAILURE until action has reported RUNNING
        self._last_status_id: dict = {}  # step_name -> latest Status.id (BT activation id)
        self._dispatch_status_id = None  # that id when the current command was sent
        self._status_trace: set = set()  # (id, status, listening) already logged since dispatch
        self._last_status_time: dict = {}  # step_name -> monotonic time of the latest Status
        self._silence_warned = False
        self._listen_timer = None
        self._wait_timer = None     # a wait step's timer, or a timed shaker step's hold
        self._shaker = ShakerPhase()

        # Input topics
        self.create_subscription(String, '/routine_executor/start_routine_cmd',
                                 self._on_start_cmd, 10)
        self.create_subscription(String, '/routine_executor/cancel',
                                 self._on_cancel, 10)
        self.create_subscription(String, '/routine_executor/pause',
                                 self._on_pause_cmd, 10)
        self.create_subscription(String, '/routine_executor/resume',
                                 self._on_resume_cmd, 10)
        self.create_subscription(String, '/routine_executor/retry',
                                 self._on_retry_cmd, 10)
        self.create_subscription(String, '/routine_executor/skip',
                                 self._on_skip_cmd, 10)

        # Status output
        self._status_pub = self.create_publisher(String, '/routine_executor/status', 10)

        # Robot command publishers (keyed by (robot, topic_suffix))
        self._cmd_pubs: dict = {}
        # Cancel stops a navigation step on the robot (nav_api's topic, not under /<robot>/)
        self._nav_cancel_pub = self.create_publisher(Empty, NAV_CANCEL_TOPIC, 10)

        # Subscribe to all known action status topics for the default robot
        self._status_subs: dict = {}  # step_name -> subscription
        self._setup_status_subscriptions(self._robot)

        # Tip-check results published by lab_machine_executive after each aspirate, and its live
        # OT-2 progress (current command, tip-check pass)
        self._recording_sub = None
        self._progress_sub = None
        self._setup_recording_subscription(self._robot)

        # 1 Hz status publish + execution loop
        self.create_timer(1.0, self._tick)

        self.get_logger().info('RoutineExecutorNode started')

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _setup_status_subscriptions(self, robot: str) -> None:
        """Subscribe to all known action status topics for the given robot."""
        for step_name, config in STEPS.items():
            if config['watch_topic'] is None:
                continue
            topic = f'/{robot}/{config["watch_topic"]}'
            sub = self.create_subscription(
                Status, topic,
                lambda msg, s=step_name: self._on_action_status(msg, s),
                STATUS_QOS,
            )
            self._status_subs[step_name] = sub
        self.get_logger().info(f'Subscribed to action status topics for robot: {robot}')

    def _setup_recording_subscription(self, robot: str) -> None:
        if self._recording_sub is not None:
            self.destroy_subscription(self._recording_sub)
        self._recording_sub = self.create_subscription(
            String, f'/{robot}/behavior/tip_check_result', self._on_tip_check_result, 10)
        if self._progress_sub is not None:
            self.destroy_subscription(self._progress_sub)
        self._progress_sub = self.create_subscription(
            String, f'/{robot}/behavior/ot2_progress', self._on_ot2_progress, 10)

    def _on_ot2_progress(self, msg: String) -> None:
        try:
            progress = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Malformed ot2_progress message: {e}')
            return
        if not isinstance(progress, dict):
            self.get_logger().warn('Malformed ot2_progress message: not an object')
            return
        # Only the OT-2 step's own progress; a late message after the step ended is dropped.
        if self._sm.current_step_name() != 'ot2':
            return
        if self._sm.set_step_progress({**progress, 'machine': 'ot2'}):
            self._publish_status()  # don't wait for the 1 Hz tick: the UI follows each command

    def _on_tip_check_result(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Malformed tip_check_result message: {e}')
            return
        if not isinstance(result, dict):
            self.get_logger().warn('Malformed tip_check_result message: not an object')
            return
        if self._sm.add_recording(result):
            self.get_logger().info(f'Attached tip check {result.get("id")} to this run')

    def _get_cmd_pub(self, robot: str, topic_suffix: str, msg_type):
        key = (robot, topic_suffix)
        if key not in self._cmd_pubs:
            topic = f'/{robot}/{topic_suffix}'
            self._cmd_pubs[key] = self.create_publisher(msg_type, topic, 10)
        return self._cmd_pubs[key]

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_start_cmd(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in start_routine_cmd: {e}')
            return

        if self._sm.state in ('running', 'paused'):
            self.get_logger().warn(f'Routine already active (state={self._sm.state}) — ignoring start command')
            return

        steps = payload.get('steps', [])
        if not steps:
            self.get_logger().error('start_routine_cmd received empty steps list')
            return
        max_retries = int(payload.get('max_retries', 3))
        robot = payload.get('robot', 'robot_1')

        # Validate step names
        unknown = [s.get('name') for s in steps if s.get('name') not in KNOWN_STEPS]
        if unknown:
            self.get_logger().error(f'Unknown steps: {unknown}. Known: {KNOWN_STEPS}')
            return

        # Validate required params per step
        for step in steps:
            name = step['name']
            config = STEPS[name]
            missing = [p for p in config['required_params'] if p not in step]
            if missing:
                self.get_logger().error(
                    f'Step "{name}" missing required params: {missing}'
                )
                return
            if name == 'shaker':
                try:
                    shaker_step.validate(step)
                except ValueError as e:
                    self.get_logger().error(f'Step "shaker" rejected: {e}')
                    return

        # Re-subscribe if robot changed
        if robot != self._robot:
            for sub in self._status_subs.values():
                self.destroy_subscription(sub)
            self._status_subs = {}
            self._robot = robot
            self._setup_status_subscriptions(robot)
            self._setup_recording_subscription(robot)

        self._sm = RoutineStateMachine(steps, max_retries=max_retries, no_retry=NO_RETRY_STEPS)
        self._listening = False
        self._seen_running = False
        self._shaker.reset()
        self._sm.start()
        self.get_logger().info(f'Starting routine: {steps} (robot={robot}, retries={max_retries})')

    def _on_cancel(self, msg: String) -> None:
        if self._sm.state in ('running', 'paused'):
            if self._listen_timer is not None and not self._listen_timer.is_canceled():
                self._listen_timer.cancel()
            if self._wait_timer is not None and not self._wait_timer.is_canceled():
                self._wait_timer.cancel()
                self._wait_timer = None
            step_name = self._sm.current_step_name()
            self._sm.cancel()
            self.get_logger().info('Routine cancelled')
            self._stop_step_on_robot(step_name)
        else:
            self.get_logger().warn('Cancel received but no routine is running')

    def _stop_step_on_robot(self, step_name: str) -> None:
        """Stop the cancelled step's action on the robot too (a wait or OT-2 step has no stop)."""
        stop = STEPS.get(step_name, {}).get('stop')
        if stop is None:
            return
        topic, stop_msg = stop()
        if topic == NAV_CANCEL_TOPIC:
            pub = self._nav_cancel_pub
        else:
            pub = self._get_cmd_pub(self._robot, topic, type(stop_msg))
        pub.publish(stop_msg)
        self.get_logger().info(f'Stopping step "{step_name}" on the robot ({pub.topic_name})')

    def _on_pause_cmd(self, msg: String) -> None:
        if self._sm.state == 'running':
            self._sm.pause()
            self.get_logger().info('Routine paused')
        else:
            self.get_logger().warn(f'Pause received but state is {self._sm.state}')

    def _on_resume_cmd(self, msg: String) -> None:
        if self._sm.state == 'paused':
            self._sm.resume()
            self.get_logger().info('Routine resumed')
        else:
            self.get_logger().warn(f'Resume received but state is {self._sm.state}')

    def _on_retry_cmd(self, msg: String) -> None:
        # Payload '' or {"force": true}; force restarts a no-retry (liquid-handling) step.
        try:
            force = json.loads(msg.data).get('force') is True if msg.data else False
        except (json.JSONDecodeError, AttributeError):
            force = False
        if self._sm.retry(force=force):
            self._listening = False
            self._seen_running = False
            self.get_logger().info(
                f'Retrying routine from step {self._sm.current_step_idx + 1} '
                f'"{self._sm.current_step_name()}"'
            )
        elif self._sm.to_dict()['retry_needs_force']:
            self.get_logger().warn(
                f'Retry refused: step "{self._sm.current_step_name()}" handles liquid; '
                'send {"force": true} after inspecting the machine')
        else:
            self.get_logger().warn(f'Retry received but state is {self._sm.state}')

    def _on_skip_cmd(self, msg: String) -> None:
        # Only from a failed run: the skipped step is not running on the robot any more.
        step_idx, step_name = self._sm.current_step_idx, self._sm.current_step_name()
        if self._sm.skip():
            self._listening = False
            self._seen_running = False
            self.get_logger().warn(
                f'Skipped step {step_idx + 1} "{step_name}" at operator request '
                f'(state={self._sm.state})')
        else:
            self.get_logger().warn(f'Skip received but state is {self._sm.state}')

    def _on_action_status(self, msg: Status, step_name: str) -> None:
        # Tracked on every message: the id the BT gave the action's latest activation.
        self._last_status_id[step_name] = msg.id
        self._last_status_time[step_name] = time.monotonic()
        if self._sm.state not in ('running', 'paused'):
            return
        if step_name != self._sm.current_step_name():
            return

        # A terminal status belongs to this attempt once RUNNING was heard, or once its id differs
        # from the one before dispatch (the BT re-activated the action). The id check matters when
        # RUNNING is too brief to hear: the shaker stop is one quick HTTP call, so RUNNING could
        # come and go inside the 100 ms deaf window and the SUCCESS was then ignored forever.
        fresh = self._seen_running or (
            self._dispatch_status_id is not None and msg.id != self._dispatch_status_id)

        # Trace each distinct (id, status) once per dispatch, so a stuck step shows what it heard.
        key = (msg.id, msg.status, self._listening)
        if key not in self._status_trace:
            self._status_trace.add(key)
            if not self._listening:
                verdict = 'not listening yet'
            elif msg.status == Status.RUNNING:
                verdict = 'running'
            else:
                verdict = 'accepted' if fresh else 'IGNORED as stale'
            self.get_logger().info(
                f'"{step_name}" status {STATUS_NAMES.get(msg.status, msg.status)} '
                f'id={msg.id} (id at send={self._dispatch_status_id}, '
                f'RUNNING heard={self._seen_running}): {verdict}')

        if not self._listening:
            return  # ignore stale messages published before this step started

        if msg.status == Status.RUNNING:
            self._seen_running = True
        elif msg.status == Status.SUCCESS and fresh:
            if step_name == 'shaker' and self._shaker.on_success(self._sm.current_step()) == 'hold':
                self._listening = False
                self._seen_running = False
                self._start_shaker_hold()
                return
            self.get_logger().info(f'Step "{step_name}" succeeded')
            self._sm.on_success()
            self._listening = False
            self._seen_running = False
        elif msg.status == Status.FAILURE and fresh:
            # Status is broadcast repeatedly. Consume this attempt's terminal
            # result once, then wait for a new dispatch/RUNNING before counting
            # another failure. Otherwise one failed plan exhausts every retry.
            self._listening = False
            self._seen_running = False
            self._sm.on_failure()
            self.get_logger().warn(
                f'Step "{step_name}" failed '
                f'(retry {self._sm.retry_count}/{self._sm.max_retries}; '
                f'state={self._sm.state})'
            )

    # ------------------------------------------------------------------
    # Execution tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._publish_status()

        if self._sm.state != 'running':
            return

        # The robot re-publishes action status at 20 Hz; silence while waiting means the link
        # (or the robot's executive) dropped, not that the step is still running.
        if self._listening and not self._silence_warned:
            step_name = self._sm.current_step_name()
            last = self._last_status_time.get(step_name)
            if STEPS.get(step_name, {}).get('watch_topic') and (
                    last is None or time.monotonic() - last > 3.0):
                self._silence_warned = True
                age = 'never' if last is None else f'{time.monotonic() - last:.1f}s ago'
                self.get_logger().warn(
                    f'No "{step_name}" status from the robot (last one {age})')

        if self._sm.needs_dispatch:
            self._dispatch_current_step()

    def _dispatch_current_step(self) -> None:
        step = self._sm.current_step()
        step_name = step['name']
        config = STEPS[step_name]

        if config['msg_type'] == 'wait':
            time_s = float(step.get('time_s', 0))
            self._sm.acknowledge_dispatch()
            if self._wait_timer is not None and not self._wait_timer.is_canceled():
                self._wait_timer.cancel()
            self._wait_timer = self.create_timer(time_s, self._on_wait_complete)
            self.get_logger().info(f'Waiting {time_s}s for step "{step_name}"')
            return

        if step_name == 'shaker':
            msg = make_shaker_msg(self._shaker.dispatch_pwm(self._sm.current_step_idx, step))
        else:
            msg = config['make_msg'](step)
        self._sm.acknowledge_dispatch()
        self._publish_and_listen(config, msg)
        self.get_logger().info(
            f'Dispatched step "{step_name}" '
            f'(retry {self._sm.retry_count}/{self._sm.max_retries})'
        )

    def _publish_and_listen(self, config: dict, msg) -> None:
        """Send a step's command, then listen for this attempt's status (not a stale one)."""
        pub = self._get_cmd_pub(
            self._robot, config['publish_topic'], STEP_MSG_CLASSES[config['msg_type']])
        pub.publish(msg)

        self._listening = False
        self._seen_running = False
        step_name = self._sm.current_step_name()
        self._dispatch_status_id = self._last_status_id.get(step_name)
        self._status_trace = set()
        self._silence_warned = False
        self.get_logger().info(
            f'Sent "{step_name}" command on {pub.topic_name} '
            f'(status id before send={self._dispatch_status_id})')

        # One-shot timer: enable listening after 100ms
        if self._listen_timer is not None and not self._listen_timer.is_canceled():
            self._listen_timer.cancel()
        self._listen_timer = self.create_timer(0.1, self._enable_listening_once)

    def _start_shaker_hold(self) -> None:
        hold_s = shaker_step.hold_time_s(self._sm.current_step())
        if self._wait_timer is not None and not self._wait_timer.is_canceled():
            self._wait_timer.cancel()
        self._wait_timer = self.create_timer(hold_s, self._on_shaker_hold_complete)
        self.get_logger().info(f'Shaker running; stopping it in {hold_s}s')

    def _on_shaker_hold_complete(self) -> None:
        # Sent straight away, even while paused: the shaker stops on time regardless.
        self._wait_timer.cancel()
        self._wait_timer = None
        self._shaker.on_hold_done()
        self._publish_and_listen(STEPS['shaker'], make_shaker_msg(0))
        self.get_logger().info('Shaker hold complete; sent PWM 0')

    def _on_wait_complete(self) -> None:
        self._wait_timer.cancel()
        self._wait_timer = None
        self.get_logger().info('Wait step complete')
        self._sm.on_success()

    def _enable_listening_once(self) -> None:
        self._listening = True
        self._listen_timer.cancel()

    def _publish_status(self) -> None:
        payload = self._sm.to_dict()
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoutineExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

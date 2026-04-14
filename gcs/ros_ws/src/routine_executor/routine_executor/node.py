import json

import rclpy
from rclpy.node import Node
from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand, Status
from std_msgs.msg import String

from routine_executor.state_machine import RoutineStateMachine
from routine_executor.step_config import STEPS, KNOWN_STEPS

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
        self._listen_timer = None

        # Input topics
        self.create_subscription(String, '/routine_executor/start_routine_cmd',
                                 self._on_start_cmd, 10)
        self.create_subscription(String, '/routine_executor/cancel',
                                 self._on_cancel, 10)

        # Status output
        self._status_pub = self.create_publisher(String, '/routine_executor/status', 10)

        # Robot command publishers (keyed by (robot, topic_suffix))
        self._cmd_pubs: dict = {}

        # Subscribe to all known action status topics for the default robot
        self._status_subs: dict = {}  # step_name -> subscription
        self._setup_status_subscriptions(self._robot)

        # 1 Hz status publish + execution loop
        self.create_timer(1.0, self._tick)

        self.get_logger().info('RoutineExecutorNode started')

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _setup_status_subscriptions(self, robot: str) -> None:
        """Subscribe to all known action status topics for the given robot."""
        for step_name, config in STEPS.items():
            topic = f'/{robot}/{config["watch_topic"]}'
            sub = self.create_subscription(
                Status, topic,
                lambda msg, s=step_name: self._on_action_status(msg, s),
                STATUS_QOS,
            )
            self._status_subs[step_name] = sub
        self.get_logger().info(f'Subscribed to action status topics for robot: {robot}')

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

        if self._sm.state == 'running':
            self.get_logger().warn('Routine already running — ignoring start command')
            return

        steps = payload.get('steps', [])
        if not steps:
            self.get_logger().error('start_routine_cmd received empty steps list')
            return
        max_retries = int(payload.get('max_retries', 3))
        robot = payload.get('robot', 'robot_1')

        unknown = [s for s in steps if s not in KNOWN_STEPS]
        if unknown:
            self.get_logger().error(f'Unknown steps: {unknown}. Known: {KNOWN_STEPS}')
            return

        # Re-subscribe if robot changed
        if robot != self._robot:
            for sub in self._status_subs.values():
                self.destroy_subscription(sub)
            self._status_subs = {}
            self._robot = robot
            self._setup_status_subscriptions(robot)

        self._sm = RoutineStateMachine(steps, max_retries=max_retries)
        self._listening = False
        self._sm.start()
        self.get_logger().info(f'Starting routine: {steps} (robot={robot}, retries={max_retries})')

    def _on_cancel(self, msg: String) -> None:
        if self._sm.state == 'running':
            if self._listen_timer is not None and not self._listen_timer.is_canceled():
                self._listen_timer.cancel()
            self._sm.cancel()
            self.get_logger().info('Routine cancelled')
        else:
            self.get_logger().warn('Cancel received but no routine is running')

    def _on_action_status(self, msg: Status, step_name: str) -> None:
        if self._sm.state != 'running':
            return
        if step_name != self._sm.current_step_name():
            return
        if not self._listening:
            return  # ignore stale messages published before this step started

        if msg.status == Status.SUCCESS:
            self.get_logger().info(f'Step "{step_name}" succeeded')
            self._sm.on_success()
        elif msg.status == Status.RUNNING:
            self._seen_running = True
        elif msg.status == Status.FAILURE and self._seen_running:
            self._sm.on_failure()
            self.get_logger().warn(
                f'Step "{step_name}" failed '
                f'(attempt {self._sm.retry_count}/{self._sm.max_retries})'
            )

    # ------------------------------------------------------------------
    # Execution tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._publish_status()

        if self._sm.state != 'running':
            return

        if self._sm.needs_dispatch:
            self._dispatch_current_step()

    def _dispatch_current_step(self) -> None:
        step_name = self._sm.current_step_name()
        config = STEPS[step_name]

        msg = config['make_msg']()
        topic_suffix = config['publish_topic']
        msg_type = ManipulationCommand if config['msg_type'] == 'manipulation' else LabMachineCommand

        pub = self._get_cmd_pub(self._robot, topic_suffix, msg_type)
        pub.publish(msg)

        self._listening = False
        self._seen_running = False
        self._sm.acknowledge_dispatch()

        # One-shot timer: enable listening after 100ms
        if self._listen_timer is not None and not self._listen_timer.is_canceled():
            self._listen_timer.cancel()
        self._listen_timer = self.create_timer(0.1, self._enable_listening_once)

        self.get_logger().info(
            f'Dispatched step "{step_name}" '
            f'(retry {self._sm.retry_count}/{self._sm.max_retries})'
        )

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

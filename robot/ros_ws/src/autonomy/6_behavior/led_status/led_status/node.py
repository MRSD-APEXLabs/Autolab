"""ROS 2 wrapper: watches behavior tree actions and drives the underglow LEDs."""

import time

import rclpy
from rclpy.node import Node
from behavior_tree_msgs.msg import Active, Status

from led_status.logic import LedLogic, topic_base
from led_status.serial_link import DEFAULT_BY_ID, LedLink, SerialPort, find_port

# Ordered by priority: earlier groups win when several actions run at once.
GROUPS = [
    ('M', ['Go To Location', 'Navigate To Pose']),
    ('A', ['Pick Object', 'Place Object', 'Pick Base Object', 'Place Base Object']),
    ('W', ['Execute OT2 Protocol', 'Execute Shaker Protocol']),
]


class LedStatusNode(Node):
    def __init__(self):
        super().__init__('led_status')
        self._port_param = self.declare_parameter('port', '').value
        self._by_id = self.declare_parameter('by_id_path', DEFAULT_BY_ID).value
        tick_hz = self.declare_parameter('tick_hz', 10.0).value

        self._logic = LedLogic(
            GROUPS,
            success_hold_s=self.declare_parameter('success_hold_s', 5.0).value,
            stale_s=self.declare_parameter('stale_s', 1.0).value,
            grace_s=self.declare_parameter('grace_s', 0.5).value,
        )
        self._link = LedLink(
            self._open,
            resend_s=self.declare_parameter('resend_s', 3.0).value,
            retry_s=self.declare_parameter('retry_s', 5.0).value,
            settle_s=self.declare_parameter('settle_s', 2.0).value,
        )
        self._warned = False
        self._letter = None

        for _, labels in GROUPS:
            for label in labels:
                base = topic_base(label)
                self.create_subscription(
                    Active, f'{base}_active',
                    lambda msg, l=label: self._logic.on_active(l, msg.active, time.monotonic()),
                    10)
                self.create_subscription(
                    Status, f'{base}_status',
                    lambda msg, l=label: self._logic.on_status(l, msg.status, time.monotonic()),
                    10)

        self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info('led_status started')

    def _open(self):
        path = find_port(self._port_param, self._by_id)
        if path is None:
            raise OSError('LED Nano (CH340 1a86:7523) not found')
        port = SerialPort(path)
        self.get_logger().info(f'LED Nano opened on {path}')
        return port

    def _tick(self):
        now = time.monotonic()
        letter = self._logic.tick(now)
        if letter != self._letter:
            self.get_logger().info(f'LED status -> {letter}')
            self._letter = letter
        self._link.update(letter, now)

        if self._link.connected:
            self._warned = False
        elif self._link.last_error is not None and not self._warned:
            self.get_logger().warn(
                f'LED Nano unavailable ({self._link.last_error}); retrying quietly')
            self._warned = True

    def shutdown(self):
        self._link.close()


def main(args=None):
    rclpy.init(args=args)
    node = LedStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

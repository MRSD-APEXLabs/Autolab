"""ROS 2 wrapper: watches behavior tree actions and says each step on the speaker."""

import os
import time

import rclpy
from rclpy.node import Node
from behavior_tree_msgs.msg import Active, ManipulationCommand, Status
from std_msgs.msg import String

from voice_status.logic import WATCHED, VoiceLogic, topic_base
from voice_status.speech import AlsaStream, Announcer, PiperVoice


class VoiceStatusNode(Node):
    def __init__(self):
        super().__init__('voice_status')
        voice_dir = self.declare_parameter('voice_dir', '').value
        piper = self.declare_parameter('piper', os.path.join(voice_dir, 'piper', 'piper')).value
        voice = self.declare_parameter('voice', 'en_US-ljspeech-high').value
        model = self.declare_parameter('model', os.path.join(voice_dir, voice + '.onnx')).value
        length_scale = self.declare_parameter('length_scale', 1.3).value
        device = self.declare_parameter('device', 'plughw:CARD=HDA,DEV=3').value
        tick_hz = self.declare_parameter('tick_hz', 10.0).value
        for path in (piper, model):
            if not os.path.exists(path):
                self.get_logger().warn(f'{path} not found: run voice_status/fetch_voice.sh; phrases will fail')

        self._logic = VoiceLogic(stale_s=self.declare_parameter('stale_s', 1.0).value,
                                 grace_s=self.declare_parameter('grace_s', 0.5).value)
        self._stream = AlsaStream(device, self.get_logger().warn)
        self._announcer = Announcer(PiperVoice(piper, model, length_scale), self._stream,
                                    self.get_logger().info, self.get_logger().warn)

        for label in WATCHED:
            base = topic_base(label)
            self.create_subscription(
                Active, f'{base}_active',
                lambda msg, l=label: self._say(self._logic.on_active(l, msg.active, time.monotonic())), 10)
            self.create_subscription(
                Status, f'{base}_status',
                lambda msg, l=label: self._say(self._logic.on_status(l, msg.status, time.monotonic())), 10)
        self.create_subscription(String, 'go_to_location_command',
                                 lambda msg: self._logic.on_location_command(msg.data), 10)
        self.create_subscription(ManipulationCommand, 'manipulation_command',
                                 lambda msg: self._logic.on_manipulation_command(msg.target_machine, msg.object_type),
                                 10)
        self.create_timer(1.0 / tick_hz, lambda: self._say(self._logic.tick(time.monotonic())))
        self.get_logger().info(f'voice_status started: {model} (length_scale {length_scale}) on {device}')

    def _say(self, phrases):
        for text in phrases:
            self.get_logger().info(f'saying: "{text}"')
            self._announcer.say(text)

    def shutdown(self):
        self._announcer.close()
        self._stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceStatusNode()
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

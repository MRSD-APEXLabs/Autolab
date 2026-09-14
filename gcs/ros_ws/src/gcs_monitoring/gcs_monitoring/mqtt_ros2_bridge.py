#!/usr/bin/env python3
"""
MQTT → ROS2 bridge node.

Subscribes to UI command topics on the MQTT broker and republishes them
as ROS2 messages on the corresponding ROS2 topics, so web UIs (the main
APEX flow and the /debug console) can command the robot.

Two kinds of command are supported:
  - STRING_CMD_TOPIC_MAP:  raw std_msgs/String passthrough (payload bytes
    become msg.data verbatim).
  - TYPED_CMD_TOPIC_MAP:   JSON payload converted to a typed ROS2 message
    by a named builder function. Every command has its own builder — this
    stays an explicit, hardcoded mapping rather than a generic
    JSON-schema-driven converter.

Environment variables:
    MQTT_HOST   broker hostname  (default: localhost)
    MQTT_PORT   broker port      (default: 1883)
"""

import json
import os
import time

import paho.mqtt.client as mqtt
import rclpy
from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String

MQTT_HOST = os.environ.get('MQTT_HOST', 'localhost')
MQTT_PORT = int(os.environ.get('MQTT_PORT', '1883'))

# ── Plain string passthrough commands ───────────────────────────────────────
STRING_CMD_TOPIC_MAP = {
    'cmd/routine_executor/start_routine_cmd': '/routine_executor/start_routine_cmd',
    'cmd/routine_executor/cancel': '/routine_executor/cancel',
    'cmd/planning/command': '/planning_command',
    'cmd/perception/camera_mode_cmd': '/robot_1/behavior/perception/camera_mode_cmd',
}


# ── Typed commands — one named builder function per command ────────────────
def _build_goal_pose(payload: dict) -> Pose2D:
    msg = Pose2D()
    msg.x = float(payload['x'])
    msg.y = float(payload['y'])
    msg.theta = float(payload['theta'])
    return msg


def _build_manipulation_command(payload: dict) -> ManipulationCommand:
    msg = ManipulationCommand()
    msg.type = payload['type']
    msg.object_type = payload.get('object_type', '')
    msg.target_machine = payload.get('target_machine', '')
    return msg


def _build_ot2_command(payload: dict) -> LabMachineCommand:
    msg = LabMachineCommand()
    msg.device = 'ot2'
    msg.action = payload.get('action', 'protocol')
    parameters = payload['parameters_json']
    msg.parameters_json = (
        parameters if isinstance(parameters, str) else json.dumps(parameters)
    )
    return msg


def _build_shaker_command(payload: dict) -> LabMachineCommand:
    msg = LabMachineCommand()
    msg.device = 'shaker'
    msg.action = 'protocol'
    msg.parameters_json = json.dumps({
        'pwm': payload['pwm'],
        'wait_time_s': payload.get('wait_time_s', 0),
    })
    return msg


TYPED_CMD_TOPIC_MAP = {
    'cmd/navigation/goal_pose': (
        '/robot_1/swerve/goal_pose', Pose2D, _build_goal_pose,
    ),
    'cmd/manipulation/command': (
        '/robot_1/behavior/manipulation_command', ManipulationCommand,
        _build_manipulation_command,
    ),
    'cmd/lab_machine/ot2_command': (
        '/robot_1/behavior/lab_machine_command', LabMachineCommand,
        _build_ot2_command,
    ),
    'cmd/lab_machine/shaker_command': (
        '/robot_1/behavior/lab_machine_command', LabMachineCommand,
        _build_shaker_command,
    ),
}


class MqttRos2Bridge(Node):
    def __init__(self, mqtt_client: mqtt.Client) -> None:
        super().__init__('mqtt_ros2_bridge')
        self._mqtt = mqtt_client

        self._string_pubs: dict = {
            ros2_topic: self.create_publisher(String, ros2_topic, 10)
            for ros2_topic in STRING_CMD_TOPIC_MAP.values()
        }

        self._typed_pubs: dict = {}
        for ros2_topic, msg_type, _builder in TYPED_CMD_TOPIC_MAP.values():
            if ros2_topic not in self._typed_pubs:
                self._typed_pubs[ros2_topic] = self.create_publisher(
                    msg_type, ros2_topic, 10)

        self.get_logger().info(
            f'MQTT→ROS2 bridge started — broker: {MQTT_HOST}:{MQTT_PORT}'
        )

    def relay(self, mqtt_topic: str, payload: bytes) -> None:
        if mqtt_topic in STRING_CMD_TOPIC_MAP:
            self._relay_string(mqtt_topic, payload)
        elif mqtt_topic in TYPED_CMD_TOPIC_MAP:
            self._relay_typed(mqtt_topic, payload)

    def _relay_string(self, mqtt_topic: str, payload: bytes) -> None:
        ros2_topic = STRING_CMD_TOPIC_MAP[mqtt_topic]
        msg = String()
        msg.data = payload.decode('utf-8', errors='replace')
        self._string_pubs[ros2_topic].publish(msg)
        self.get_logger().info(f'Relayed {mqtt_topic} → {ros2_topic}')

    def _relay_typed(self, mqtt_topic: str, payload: bytes) -> None:
        ros2_topic, _msg_type, builder = TYPED_CMD_TOPIC_MAP[mqtt_topic]
        try:
            body = json.loads(payload.decode('utf-8'))
            msg = builder(body)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(
                f'Dropping malformed payload on {mqtt_topic}: {exc}')
            return
        self._typed_pubs[ros2_topic].publish(msg)
        self.get_logger().info(f'Relayed {mqtt_topic} → {ros2_topic}')


def main() -> None:
    rclpy.init()

    client = mqtt.Client(client_id='mqtt-ros2-bridge')

    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
            break
        except Exception as exc:
            print(
                f'[mqtt_ros2_bridge] Waiting for broker at '
                f'{MQTT_HOST}:{MQTT_PORT} ({exc}) — retrying in 2s',
                flush=True,
            )
            time.sleep(2)

    node = MqttRos2Bridge(client)
    all_cmd_topics = list(STRING_CMD_TOPIC_MAP) + list(TYPED_CMD_TOPIC_MAP)

    def on_connect(_client, _userdata, _flags, rc):
        if rc != 0:
            print(f'[mqtt_ros2_bridge] Connect failed rc={rc}', flush=True)
            return
        for mqtt_topic in all_cmd_topics:
            _client.subscribe(mqtt_topic)
            print(f'[mqtt_ros2_bridge] Subscribed: {mqtt_topic}', flush=True)

    def on_message(_client, _userdata, msg):
        node.relay(msg.topic, msg.payload)

    client.on_connect = on_connect
    client.on_message = on_message

    client.loop_start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        client.loop_stop()
        client.disconnect()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

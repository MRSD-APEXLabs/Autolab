#!/usr/bin/env python3
"""
MQTT → ROS2 bridge node.

Subscribes to UI command topics on the MQTT broker and republishes them
as std_msgs/String messages on the corresponding ROS2 topics so the GCS
web UI can trigger the routine executor.

MQTT topic → ROS2 topic mapping:
    cmd/routine_executor/start_routine_cmd  →  /routine_executor/start_routine_cmd
    cmd/routine_executor/cancel             →  /routine_executor/cancel

Environment variables:
    MQTT_HOST   broker hostname  (default: localhost)
    MQTT_PORT   broker port      (default: 1883)
"""

import os
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MQTT_HOST = os.environ.get('MQTT_HOST', 'localhost')
MQTT_PORT = int(os.environ.get('MQTT_PORT', '1883'))

CMD_TOPIC_MAP = {
    'cmd/routine_executor/start_routine_cmd': '/routine_executor/start_routine_cmd',
    'cmd/routine_executor/cancel': '/routine_executor/cancel',
}


class MqttRos2Bridge(Node):
    def __init__(self, mqtt_client: mqtt.Client) -> None:
        super().__init__('mqtt_ros2_bridge')
        self._mqtt = mqtt_client
        self._pubs: dict = {
            ros2_topic: self.create_publisher(String, ros2_topic, 10)
            for ros2_topic in CMD_TOPIC_MAP.values()
        }
        self.get_logger().info(
            f'MQTT→ROS2 bridge started — broker: {MQTT_HOST}:{MQTT_PORT}'
        )

    def relay(self, mqtt_topic: str, payload: bytes) -> None:
        ros2_topic = CMD_TOPIC_MAP.get(mqtt_topic)
        if ros2_topic is None:
            return
        msg = String()
        msg.data = payload.decode('utf-8', errors='replace')
        self._pubs[ros2_topic].publish(msg)
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

    def on_connect(_client, _userdata, _flags, rc):
        if rc != 0:
            print(f'[mqtt_ros2_bridge] Connect failed rc={rc}', flush=True)
            return
        for mqtt_topic in CMD_TOPIC_MAP:
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

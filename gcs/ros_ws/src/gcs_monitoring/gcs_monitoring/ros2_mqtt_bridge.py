#!/usr/bin/env python3
"""
ROS2 → MQTT bridge node.

Discovers active ROS2 topics dynamically, subscribes to them, and
publishes each message as a JSON payload to a Mosquitto broker.

MQTT topic mirrors the ROS2 topic path (e.g. /odom → ros2/odom).
Intended to run inside the GCS container where Mosquitto listens on
localhost:1883.

Usage:
    ros2 run gcs_bringup ros2_mqtt_bridge

Environment variables:
    MQTT_HOST                    broker hostname  (default: localhost)
    MQTT_PORT                    broker port      (default: 1883)
    ROS2MQTT_DISCOVERY_INTERVAL  seconds between topic re-discovery (default: 10)
"""

import json
import os
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
DISCOVERY_INTERVAL = float(os.environ.get("ROS2MQTT_DISCOVERY_INTERVAL", "10.0"))


class Ros2MqttBridge(Node):
    def __init__(self, mqtt_client: mqtt.Client) -> None:
        super().__init__("ros2_mqtt_bridge")
        self._mqtt = mqtt_client
        self._sub_map: dict = {}
        self._lock = threading.Lock()

        # Best-effort QoS for broad compatibility (sensor streams, etc.)
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.get_logger().info(
            f"ROS2→MQTT bridge started — broker: {MQTT_HOST}:{MQTT_PORT}"
        )
        self._discover_and_subscribe()
        self.create_timer(DISCOVERY_INTERVAL, self._discover_and_subscribe)

    def _discover_and_subscribe(self) -> None:
        for topic_name, type_list in self.get_topic_names_and_types():
            with self._lock:
                if topic_name in self._sub_map or "stereo" in topic_name:
                    continue

            if not type_list:
                continue

            msg_type_str = type_list[0]
            try:
                msg_type = get_message(msg_type_str)
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping {topic_name}: cannot load type "
                    f"{msg_type_str} — {exc}"
                )
                continue

            sub = self.create_subscription(
                msg_type,
                topic_name,
                self._make_callback(topic_name),
                self._qos,
            )
            with self._lock:
                self._sub_map[topic_name] = sub
            self.get_logger().info(f"Subscribed: {topic_name} [{msg_type_str}]")

    def _make_callback(self, topic_name: str):
        mqtt_topic = f"ros2{topic_name}"  # e.g. ros2/odom, ros2/scan

        def callback(msg) -> None:
            try:
                payload = json.dumps(message_to_ordereddict(msg), default=str)
            except Exception:
                payload = str(msg)
            self._mqtt.publish(mqtt_topic, payload, qos=0)

        return callback


def main() -> None:
    rclpy.init()

    client = mqtt.Client(client_id="ros2-mqtt-bridge")
    client.on_connect = lambda c, u, f, rc: print(
        f"[ros2_mqtt_bridge] MQTT connected (rc={rc})", flush=True
    )

    # Retry until Mosquitto is ready (it may still be starting up)
    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
            break
        except Exception as exc:
            print(
                f"[ros2_mqtt_bridge] Waiting for broker at "
                f"{MQTT_HOST}:{MQTT_PORT} ({exc}) — retrying in 2s",
                flush=True,
            )
            time.sleep(2)

    client.loop_start()

    node = Ros2MqttBridge(client)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        client.loop_stop()
        client.disconnect()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

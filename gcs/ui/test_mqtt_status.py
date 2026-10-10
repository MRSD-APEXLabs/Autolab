#!/usr/bin/env python3
"""
Test script for ros2/routine_executor/status MQTT topic.

Usage:
    # Publish a simulated sequence (watch the UI respond):
    python3 test_mqtt_status.py publish

    # Just subscribe and print what's arriving:
    python3 test_mqtt_status.py subscribe

    # Publish a single state directly:
    python3 test_mqtt_status.py publish --state running --step 2

Requires: pip install paho-mqtt
Broker:   localhost:9001 (MQTT over WebSocket)
"""

import argparse
import json
import time
import sys

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("Install paho-mqtt first:  pip install paho-mqtt")

BROKER_HOST = "gcs.autolab"
BROKER_PORT = 9001          # WebSocket port (matches the UI)
TOPIC = "ros2/routine_executor/status"

SAMPLE_STEPS = [
    "arm_robot",
    "take_off",
    "fly_to_waypoint_1",
    "fly_to_waypoint_2",
    "land",
    "disarm_robot",
]


# ---------------------------------------------------------------------------
# Publisher helpers
# ---------------------------------------------------------------------------

def make_client(client_id: str) -> mqtt.Client:
    client = mqtt.Client(
        client_id=client_id,
        transport="websockets",
    )
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    return client


def publish(client: mqtt.Client, payload: dict) -> None:
    msg = json.dumps(payload)
    result = client.publish(TOPIC, msg, qos=0, retain=True)
    result.wait_for_publish()
    print(f"  → published: {msg}")


def run_simulation(args) -> None:
    """Walk through idle → running (step-by-step) → success."""
    client = make_client("test-publisher")
    client.loop_start()

    steps = SAMPLE_STEPS
    delay = args.delay

    # 1. Idle
    print("\n[idle]")
    publish(client, {"state": "idle", "steps": steps, "current_step": 0,
                     "current_step_name": None, "retry_count": 0, "max_retries": 3})
    time.sleep(delay)

    # 2. Running — advance through each step
    for i, name in enumerate(steps):
        print(f"\n[running — step {i+1}/{len(steps)}: {name}]")
        publish(client, {
            "state": "idle" if args.state == "idle" else "running",
            "steps": steps,
            "current_step": i,
            "current_step_name": name,
            "retry_count": 0,
            "max_retries": 3,
        })
        time.sleep(delay)

    # 3. Terminal state
    terminal = args.terminal
    if terminal == "success":
        print("\n[success]")
        publish(client, {"state": "success", "steps": steps,
                         "current_step": len(steps), "current_step_name": None,
                         "retry_count": 0, "max_retries": 3})
    elif terminal == "failed":
        fail_step = min(args.fail_at, len(steps) - 1)
        print(f"\n[failed at step {fail_step+1}]")
        publish(client, {
            "state": "failed",
            "steps": steps,
            "current_step": fail_step,
            "current_step_name": steps[fail_step],
            "retry_count": args.max_retries,
            "max_retries": args.max_retries,
            "error": f"Step '{steps[fail_step]}' timed out after {args.max_retries} retries",
        })
    elif terminal == "retry":
        i = args.fail_at
        for retry in range(1, args.max_retries + 1):
            print(f"\n[running — retry {retry}/{args.max_retries} on step {i+1}]")
            publish(client, {
                "state": "running",
                "steps": steps,
                "current_step": i,
                "current_step_name": steps[i],
                "retry_count": retry,
                "max_retries": args.max_retries,
            })
            time.sleep(delay)

    time.sleep(1)
    client.loop_stop()
    client.disconnect()
    print("\nDone.")


def run_single(args) -> None:
    """Publish one hand-crafted payload."""
    client = make_client("test-publisher-single")
    client.loop_start()

    step_idx = args.step
    payload = {
        "state": args.state,
        "steps": SAMPLE_STEPS,
        "current_step": step_idx,
        "current_step_name": SAMPLE_STEPS[step_idx] if step_idx < len(SAMPLE_STEPS) else None,
        "retry_count": args.retry_count,
        "max_retries": args.max_retries,
    }
    if args.error:
        payload["error"] = args.error

    print(f"\nPublishing single payload to {TOPIC}:")
    publish(client, payload)

    time.sleep(1)
    client.loop_stop()
    client.disconnect()


# ---------------------------------------------------------------------------
# Subscriber
# ---------------------------------------------------------------------------

def run_subscriber(_args) -> None:
    """Subscribe and pretty-print every message."""

    def on_connect(client, _userdata, _flags, rc):
        if rc == 0:
            print(f"Connected to {BROKER_HOST}:{BROKER_PORT}")
            client.subscribe(TOPIC)
            print(f"Subscribed to '{TOPIC}' — waiting for messages (Ctrl-C to quit)…\n")
        else:
            print(f"Connection failed, rc={rc}")

    def on_message(_client, _userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            print(json.dumps(data, indent=2))
            print("-" * 40)
        except Exception as exc:
            print(f"Bad payload: {exc} — raw: {msg.payload!r}")

    client = mqtt.Client(client_id="test-subscriber", transport="websockets")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nDisconnecting.")
        client.disconnect()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    # publish (simulation)
    p = sub.add_parser("publish", help="Simulate a full routine run")
    p.add_argument("--delay", type=float, default=2.0,
                   help="Seconds between step transitions (default: 2)")
    p.add_argument("--terminal", choices=["success", "failed", "retry"], default="success",
                   help="How the routine ends (default: success)")
    p.add_argument("--fail-at", type=int, default=2, dest="fail_at",
                   help="Step index to fail/retry at (default: 2)")
    p.add_argument("--max-retries", type=int, default=3, dest="max_retries")
    p.add_argument("--state", default="running",
                   help="Override running state label (e.g. idle to freeze)")
    p.set_defaults(func=run_simulation)

    # single — publish one payload
    s = sub.add_parser("single", help="Publish a single payload")
    s.add_argument("--state", choices=["idle", "running", "success", "failed"],
                   default="running")
    s.add_argument("--step", type=int, default=0, help="current_step index")
    s.add_argument("--retry-count", type=int, default=0, dest="retry_count")
    s.add_argument("--max-retries", type=int, default=3, dest="max_retries")
    s.add_argument("--error", default=None)
    s.set_defaults(func=run_single)

    # subscribe
    sub.add_parser("subscribe", help="Subscribe and print incoming messages").set_defaults(
        func=run_subscriber)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

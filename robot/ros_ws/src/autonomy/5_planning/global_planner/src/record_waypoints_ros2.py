#!/usr/bin/env python3
"""
record_waypoints_ros2.py
Auto-records the first 6 joint states every 1 second, deduplicates continuously.
Saves to /home/robot/AutoLab/robot/manual_waypoints.json on exit (Ctrl+C).
"""
import json
import math
import time
import signal
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WAYPOINTS_FILE  = "/home/robot/AutoLab/robot/manual_waypoints_0.json"
RECORD_INTERVAL = 0.05 # seconds between auto-snapshots
DUPLICATE_TOL   = 0.02  # rad — increased slightly to avoid near-duplicates
NUM_JOINTS      = 6     # xArm6 has 6 joints


def joint_dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.latest_angles_rad = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.get_logger().info("Subscribed to /joint_states")

    def _cb(self, msg: JointState):
        with self.lock:
            # Take only the first NUM_JOINTS values
            self.latest_angles_rad = list(msg.position)[:NUM_JOINTS]

    def get_angles_rad(self):
        with self.lock:
            return list(self.latest_angles_rad) if self.latest_angles_rad else None


def main():
    rclpy.init()
    node = WaypointRecorder()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # Load existing waypoints
    try:
        with open(WAYPOINTS_FILE) as f:
            waypoints = json.load(f)
        print(f"Loaded {len(waypoints)} existing waypoints.")
    except FileNotFoundError:
        waypoints = []

    print(f"\n=== Auto Waypoint Recorder (6-DOF) ===")
    print(f"Recording every {RECORD_INTERVAL}s — duplicates within {DUPLICATE_TOL} rad ignored.")
    print("Move the arm to new poses. Press Ctrl+C to stop and save.\n")

    print("Waiting for /joint_states...", end="", flush=True)
    while node.get_angles_rad() is None:
        time.sleep(0.1)
        print(".", end="", flush=True)
    print(" ready!\n")

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)

    while running:
        time.sleep(RECORD_INTERVAL)

        angles_rad = node.get_angles_rad()
        if angles_rad is None:
            continue

        # Duplicate check against all existing waypoints
        is_dup = any(joint_dist(angles_rad, wp["angles_rad"]) < DUPLICATE_TOL
                     for wp in waypoints)
        if is_dup:
            continue

        angles_deg = [math.degrees(a) for a in angles_rad]
        entry = {
            "index":      len(waypoints),
            "angles_deg": angles_deg,
            "angles_rad": angles_rad,
            "tcp_mm":     [],
            "note":       ""
        }
        waypoints.append(entry)
        print(f"  ✓ #{entry['index']}  deg={[f'{a:.1f}' for a in angles_deg]}", flush=True)

    with open(WAYPOINTS_FILE, "w") as f:
        json.dump(waypoints, f, indent=2)
    print(f"\nSaved {len(waypoints)} waypoints to {WAYPOINTS_FILE}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
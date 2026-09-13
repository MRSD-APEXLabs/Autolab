#!/usr/bin/env python3
"""
record_waypoints.py
Jog the robot manually, press Enter to record each pose.
Live pose + joint angles displayed every 5 seconds.
Saves to /home/robot/AutoLab/robot/manual_waypoints.json
"""
import json
import math
import time
import threading
from xarm.wrapper import XArmAPI

WAYPOINTS_FILE = "/home/robot/AutoLab/robot/manual_waypoints.json"

arm = XArmAPI('192.168.1.236')
DISPLAY_INTERVAL = 5.0  # seconds between live pose prints
time.sleep(0.5)
arm.set_mode(2)   # manual/teach mode — lets you move the arm by hand
arm.set_state(0)

waypoints = []
stop_display = threading.Event()


def display_pose():
    """Background thread: print current pose + joint angles every 5 seconds."""
    while not stop_display.is_set():
        time.sleep(DISPLAY_INTERVAL)
        if stop_display.is_set():
            break
        try:
            angles_deg = arm.angles
            tcp = arm.position  # [x, y, z, roll, pitch, yaw] in mm/deg
            angles_rad = [math.radians(a) for a in angles_deg]

            print("\n" + "─" * 55)
            print(f"  LIVE POSE  ({time.strftime('%H:%M:%S')})")
            print(f"  TCP  (mm/°):  {[f'{v:.1f}' for v in tcp]}")
            print(f"  Joints (°):   {[f'{a:.1f}' for a in angles_deg]}")
            print(f"  Joints (rad): {[f'{a:.4f}' for a in angles_rad]}")
            print("─" * 55)
            # Reprint the prompt so the user sees where to type
            print(f"[{len(waypoints)} recorded] >> ", end="", flush=True)
        except Exception as e:
            print(f"\n  [live display error: {e}]")


# Start background display thread
display_thread = threading.Thread(target=display_pose, daemon=True)
display_thread.start()

print("=== Waypoint Recorder ===")
print(f"Live pose updates every {DISPLAY_INTERVAL:.0f}s in background.")
print("Move the arm to a pose, then press Enter to record.")
print("Commands: [Enter]=record, [n]=note last, [d]=delete last, [s]=show all, [q]=quit+save\n")

while True:
    cmd = input(f"[{len(waypoints)} recorded] >> ").strip().lower()

    if cmd == "":
        angles_deg = arm.angles
        angles_rad = [math.radians(a) for a in angles_deg]
        tcp = arm.position  # [x, y, z, roll, pitch, yaw] in mm/deg
        entry = {
            "index": len(waypoints),
            "angles_deg": angles_deg,
            "angles_rad": angles_rad,
            "tcp_mm": tcp,
            "note": ""
        }
        waypoints.append(entry)
        print(f"  ✓ Recorded #{len(waypoints) - 1}")
        print(f"    Degrees: {[f'{a:.1f}' for a in angles_deg]}")
        print(f"    Radians: {[f'{a:.4f}' for a in angles_rad]}")
        print(f"    TCP:     {[f'{v:.1f}' for v in tcp]}")

    elif cmd == "n":
        if waypoints:
            note = input("  Note for last waypoint: ")
            waypoints[-1]["note"] = note
            print(f"  Note saved.")
        else:
            print("  No waypoints to annotate.")

    elif cmd == "d":
        if waypoints:
            removed = waypoints.pop()
            print(f"  Removed #{removed['index']}")
        else:
            print("  Nothing to remove.")

    elif cmd == "s":
        if not waypoints:
            print("  No waypoints yet.")
        for wp in waypoints:
            print(f"  [{wp['index']}] deg={[f'{a:.1f}' for a in wp['angles_deg']]} note='{wp['note']}'")

    elif cmd == "q":
        break

# Shutdown
stop_display.set()
display_thread.join(timeout=DISPLAY_INTERVAL + 1)

arm.set_mode(0)
arm.set_state(0)
arm.disconnect()

with open(WAYPOINTS_FILE, "w") as f:
    json.dump(waypoints, f, indent=2)

print(f"\nSaved {len(waypoints)} waypoints to {WAYPOINTS_FILE}")
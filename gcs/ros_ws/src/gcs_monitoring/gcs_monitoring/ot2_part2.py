#!/usr/bin/env python3
"""Dispense part 1's liquid into empty_plate A1 (slot 2), discard tip, stay connected.

Run: python3 ot2_part2.py (after part 1 stops above source tube A1).
Uses the saved native session, not the wrapper's stale liquid-volume tracker.
Coordinates are specific to the current deck. No aspiration is performed here.
"""

import argparse
import json
from pathlib import Path

from gcs_monitoring.ot2_part1 import request, robot_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path,
                        default=Path(__file__).with_name("ot2_part1_state.json"))
    args = parser.parse_args()
    state = json.loads(args.state.read_text())
    if state["stage"] != "ready_for_part2":
        raise SystemExit(f"Cannot start part 2: stage={state['stage']!r}")
    host, robot_url = state["ssh_host"], state["robot_url"]
    wrapper = state["wrapper_url"]

    def save(stage):
        state["stage"] = stage
        args.state.write_text(json.dumps(state, indent=2) + "\n")

    def command(kind, params):
        return robot_command(host, robot_url, state["run_id"], kind, params)

    # Check that another client has not replaced the wrapper session.
    recent = request(host, wrapper, "/debug/ot2-commands?limit=1")
    if recent["run_id"] != state["run_id"]:
        raise SystemExit("Active run changed; inspect before continuing")
    save("part2_started_do_not_repeat")
    try:
        pipette = {"pipetteId": state["pipette_id"]}
        command("moveToCoordinates", {
            **pipette, "coordinates": {"x": 146.9, "y": 74.2, "z": 1.9},
            "minimumZHeight": 80, "forceDirect": False,
        })
        save("dispense_submitted_do_not_repeat")
        dispense = command("dispenseInPlace", {
            **pipette, "volume": state["volume_ul"], "flowRate": 92.86,
            "pushOut": False,
        })
        state["dispense_command_id"] = dispense["id"]
        state["destination"] = "empty_plate A1 (slot 2)"
        save("dispensed")
        command("home", {"axes": ["leftZ"]})
        # Use the wrapper to discard so its tip trackers match the hardware.
        print(request(host, wrapper, "/commands/drop-tips", {}), flush=True)
        save("tip_discarded")
        save("completed")
        print('{"status":"ok"}', flush=True)
        print("Dispensed into plate A1, discarded tip, ready for the next transfer.")
    except Exception as exc:
        state["error"] = str(exc)
        state["failed_stage"] = state["stage"]
        save("needs_inspection")
        raise SystemExit(f"Stopped: {exc}\nDo not repeat dispensing blindly.") from exc


if __name__ == "__main__":
    main()

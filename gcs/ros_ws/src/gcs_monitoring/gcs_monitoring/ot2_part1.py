#!/usr/bin/env python3
"""Pick up a fresh tip, aspirate, and hold fully raised above the source.

Run here: python3 ot2_part1.py --tip-index 1
Tip indices are zero-based: 0=A1, 1=B1, ... (choose an unused tip).
Defaults: 200 uL from tube A1 in slot 10; tip rack in slot 11.
Raises, then stops fully raised above source tube A1 (slot 10) for part 2.
Run Home & discard tip once to initialize; the wrapper stays connected between cycles.
No commands are executed merely by importing this module.
"""

import argparse
import json
from pathlib import Path
import time


def request(host, base, path, body=None):
    """Make one direct request from GCS. Never retry a hardware request."""
    import urllib.request
    import urllib.error
    req = urllib.request.Request(
        base + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={'Content-Type': 'application/json', 'Opentrons-Version': '3'})
    try:
        with urllib.request.urlopen(req, timeout=90) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f'HTTP {exc.code}: {exc.read().decode()}') from exc



def robot_command(host, robot_url, run_id, kind, params):
    """Submit once and let the robot wait for completion; no polling sleeps."""
    path = f"/runs/{run_id}/commands?waitUntilComplete=true&timeout=80000"
    command = request(host, robot_url, path, {
        "data": {"commandType": kind, "params": params, "intent": "setup"},
    })["data"]
    command_id = command["id"]
    status = command["status"]
    print(f"{kind}: {status} ({command_id})", flush=True)
    if status == "succeeded":
        return command
    if status in ("queued", "running"):
        raise TimeoutError(f"Command {command_id} is unresolved; inspect before retrying")
    raise RuntimeError(json.dumps(command.get("error") or command))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh-host", default="p5090")
    parser.add_argument("--tip-index", type=int, required=True,
                        help="Unused tip index, 0=A1, 1=B1, etc.")
    parser.add_argument("--source-tube", choices=("A1", "A2", "A3", "A4"), default="A1")
    parser.add_argument("--volume", type=float, default=200)
    parser.add_argument("--state", type=Path,
                        default=Path(__file__).with_name("ot2_part1_state.json"))
    args = parser.parse_args()
    if not 0 <= args.tip_index < 96 or not 0 < args.volume <= 200:
        parser.error("tip-index must be 0..95; volume must be >0 and <=200 uL")

    # Only a successfully completed part 2 allows a new cycle with this file.
    previous = None
    if args.state.exists():
        previous = json.loads(args.state.read_text())
        if previous.get("stage") in ("completed", "prepared"):
            args.state.rename(args.state.with_name(
                f"{args.state.stem}.completed-{time.time_ns()}.json"))
    # An unfinished state file prevents accidentally repeating an aspiration.
    with args.state.open("x") as file:
        json.dump({"stage": "started"}, file)
    state = {"stage": "started", "ssh_host": args.ssh_host}

    def save(stage):
        state["stage"] = stage
        args.state.write_text(json.dumps(state, indent=2) + "\n")

    wrapper = "http://127.0.0.1:8001"

    def api(path, body=None):
        return request(args.ssh_host, wrapper, path, body)

    try:
        status = api("/robot/status")
        if not status["connected"]:
            raise RuntimeError("Robot is disconnected. Use Home & discard tip before Part 1.")
        robot_url = f"http://{status['host']}:31950"
        state["robot_url"] = robot_url
        state["wrapper_url"] = wrapper
        tips = api("/debug/ot2-tip-state")
        if any(tip["has_tip"] for tip in tips.values()):
            raise RuntimeError("A tip is already mounted. Use Home & discard tip before Part 1.")
        print(f"Picking up tip index {args.tip_index}...", flush=True)
        print(api("/commands/pick-up-tips", {
            "resource_name": "tip_rack", "well_indices": [args.tip_index],
        }), flush=True)
        recent = api("/debug/ot2-commands?limit=50")
        pickups = [cmd for cmd in recent["commands"]
                   if cmd["type"] == "pickUpTip" and cmd["status"] == "succeeded"]
        pickup = max(pickups, key=lambda cmd: cmd["createdAt"])
        pipette_id = pickup["params"]["pipetteId"]
        state.update(run_id=recent["run_id"], pipette_id=pipette_id,
                     volume_ul=args.volume, tip_index=args.tip_index,
                     source=f"tube_rack {args.source_tube} (slot 10)",
                     source_tube=args.source_tube)
        save("tip_picked_up")

        def command(kind, **params):
            params["pipetteId"] = pipette_id
            return robot_command(args.ssh_host, robot_url, state["run_id"], kind, params)

        # Coordinates from this setup's successful tube-A1 aspiration.
        # Do not reuse these coordinates with a different deck/tube geometry.
        # Verified against the wrapper's loaded rack: A1/A2/A3/A4 are children 0/3/6/9.
        source_positions = {
            "A1": {"x": 13.88, "y": 339.24, "z": 7.85},
            "A2": {"x": 38.88, "y": 339.24, "z": 7.85},
            "A3": {"x": 63.88, "y": 339.24, "z": 7.85},
            "A4": {"x": 88.88, "y": 339.24, "z": 7.85},
        }
        command("moveToCoordinates", coordinates=source_positions[args.source_tube],
                minimumZHeight=80, forceDirect=False)
        save("aspiration_submitted_do_not_repeat")
        result = command("aspirateInPlace", volume=args.volume, flowRate=46.43)
        state["aspiration_command_id"] = result["id"]
        state["completed_at"] = result.get("completedAt")
        save("aspirated_needs_lift")
        # Vertical homing raises fully without targeting the unreachable Z=120.
        lift = robot_command(args.ssh_host, robot_url, state["run_id"],
                             "home", {"axes": ["leftZ"]})
        state["lift_command_id"] = lift["id"]
        state["position"] = f"fully_raised_above_tube_rack_{args.source_tube}"
        save("ready_for_part2")
        print('{"status":"ok"}', flush=True)
        print(f"Holding fully raised above tube {args.source_tube}. Part-2 state: {args.state}", flush=True)
        # No dispense, drop, or disconnect after reaching the hold position.
        # Part 2 must use this native run/pipette: the wrapper's PLR volume
        # tracker does not know about this native aspiration.
    except Exception as exc:
        state["error"] = str(exc)
        save("needs_inspection")
        raise SystemExit(f"Stopped: {exc}\nInspect robot/state before rerunning.") from exc


if __name__ == "__main__":
    main()

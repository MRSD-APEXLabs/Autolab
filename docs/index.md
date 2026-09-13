# Autolab Documentation

Autolab is a **self-driving laboratory robot** built by MRSD APEX Labs (CMU). A UFACTORY xArm6 arm with a custom parallel gripper picks up well plates, loads them into an **Opentrons OT-2** liquid handler, runs a pipetting protocol, transfers the plate to a **custom shaker module**, shakes it, and retrieves it — all as one scripted routine launched from a web browser.

!!! danger "Hardware safety"
    Most run instructions in these docs move a **real robot arm**. Before running anything:

    - keep the physical e-stop within reach,
    - clear people and glassware out of the arm's workspace,
    - never let two programs command the arm at once (e.g. [joystick teleop](subsystems/teleop.md) while the MoveIt driver is up).

## What a routine looks like

The default routine shipped in the UI (`gcs/ui/src/App.js`) tells the whole story:

1. `pick_base` — pick the well plate up off the robot's base
2. `place → ot2` — place it into the OT-2
3. `ot2` — run a pipetting protocol (pick up tips, aspirate 200 µL, dispense 5×40 µL, return tips)
4. `pick_up` — retrieve the plate from the OT-2
5. `place → shaker` — place it onto the shaker
6. `shaker pwm=150` — shake
7. `wait 20 s`
8. `shaker pwm=0` — stop
9. `pick_up` — retrieve the plate

Each manipulation step is vision-guided: the arm first moves to an inspection pose, an external perception box detects AprilTags and well plates, the planner computes a constrained collision-free path over a taught-waypoint roadmap, and MoveIt executes it on the arm.

## The stack, top to bottom

| Layer | Component | Where |
|---|---|---|
| Operator | React web UI / CLI / raw `ros2 topic pub` | [`gcs/ui/`](subsystems/ui.md) |
| Transport | Mosquitto MQTT (ws :9001, tcp :1883) + MQTT↔ROS 2 bridges | [`gcs_monitoring`](subsystems/gcs.md) |
| Mission | `routine_executor` — retrying step state machine | [`gcs/ros_ws`](subsystems/gcs.md) |
| Task | `manipulation_executive` + `lab_machine_executive` + behavior tree | [`6_behavior`](subsystems/behavior.md) |
| Motion | `global_planner` (PRM + constrained RRT) over MoveIt | [`5_planning`](subsystems/planning.md) |
| Driver | vendored `xarm_ros2` → xArm6; HTTP → OT-2/shaker | [`5_planning`](subsystems/planning.md), [`6_behavior`](subsystems/behavior.md) |
| Sensing | the Xavier (YOLO + AprilTags + servo grasp) + host bridge scripts + VLP-16 | [Camera-Edge (Xavier)](subsystems/xavier.md), [Perception](subsystems/perception.md) |

Everything runs in Docker, orchestrated by the [`autolab` CLI](setup.md#the-autolab-cli).

## Reading order

1. **New to ROS?** Start with the [ROS 2 primer](ros2-primer.md) — it explains nodes, topics, launch files, and workspaces using this repo's own files.
2. [Setup](setup.md) — clone, `autolab` CLI, `.env`, and the lab network (IPs, ports, ROS domain IDs).
3. [Running the System](running.md) — the end-to-end runbook, from powering the robot to clicking **Continue** in the UI. For planning alone, the copy-paste [Planning Quickstart](planning-quickstart.md).
4. [Subsystems](subsystems/robot-bringup.md) — one page per subsystem: purpose, nodes, topics, how to run it alone.
5. [Reference](reference/commands.md) — every command, topic, and data-file format in one place.
6. [Known Issues & Gotchas](known-issues.md) — **read before debugging**; several things in this repo look runnable but are known-broken or need manual steps.

## Status honesty

Not everything in the repo is live. The swerve base, RTAB-Map mapping, and several bringup stages are stubs or unwired; the UI's "Vision Understanding / Intelligent Planning" cards are static text; a few scripts are broken as checked in; and — most importantly — **no node in the autonomy stack commands the gripper**, so today's `place` step hovers above the target machine and returns home without releasing the plate ([#21](known-issues.md)). These docs mark each of those explicitly rather than describing an idealized system — see [Known Issues](known-issues.md).

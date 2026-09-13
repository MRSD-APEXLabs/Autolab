# Running the System

This is the end-to-end runbook: from a powered-off robot to a routine running from the web UI. Follow it in order; each step lists what you should see before moving on.

**Just want the arm planning?** Use the [Planning Quickstart](planning-quickstart.md) — copy-paste commands only, verified on hardware.

!!! danger "Before you start"
    The arm will move. E-stop in reach, workspace clear, and make sure nothing else can command the arm (no [joystick teleop](subsystems/teleop.md), no UFACTORY Studio session).

## 0. Power & network

Power on: the robot (Jetson + xArm6 controller), the **Xavier** camera box (`192.168.1.101` / `xavier.autolab`), the LiDAR, the OT-2, and the shaker. On the robot Jetson, the wired port `enP2p1s0` (192.168.1.100) carries the arm/Xavier/LiDAR subnet and WiFi `wlP1p1s0` (192.168.10.3) carries the OT-2/shaker subnet — **both must be up**:

```bash
ip -4 -br addr | grep -E "enP2p1s0|wlP1p1s0"          # both UP
for h in 192.168.1.236 192.168.1.101 192.168.1.202 192.168.10.9 192.168.10.14; do ping -c1 -W1 $h >/dev/null && echo "$h UP" || echo "$h down"; done
```

Two services that live **outside this repo** must also be running, or steps will fail later:

- **The Xavier's perception server** — `ssh autolab@192.168.1.101`, `cd ~/Autolab-Camera-Edge && python3 main.py` (system python, in tmux). Its dashboard (`example_client/test.html`, served with `python3 -m http.server 8080` in that folder → `http://192.168.1.101:8080/test.html`) shows the feeds, the `wellplates / apriltags` counts and the mode buttons. Full details: [Camera-Edge (Xavier)](subsystems/xavier.md).
- **The OT-2 HTTP wrapper** (FastAPI on port 8000; `lab_machine_executive` calls `http://192.168.10.4:8000`). Its source is `~/ot2-wrapper` on the robot Jetson (`docker compose -f ~/ot2-wrapper/docker/docker-compose.yml up -d`). If you run it on the Jetson instead of on `192.168.10.4`, change `ot2_base_url` in `robot/ros_ws/src/autonomy/6_behavior/lab_machine_executive/config/lab_machines.yaml` to `http://192.168.10.3:8000`.

## 1. Robot stack

```bash
docker start autolab-robot-1          # existing container, existing build — nothing rebuilt
autolab connect robot                 # shell in; you land in robot/ros_ws
tmux kill-session -t robot_bringup    # stop the auto `bws` + broken robot.launch.xml immediately
# (no container yet?  AUTOLAUNCH=false autolab up robot  creates one idle)
```

With `AUTOLAUNCH=true` the container runs `bws && sws && ros2 launch robot_bringup robot.launch.xml` (with `sim:=false` on the Jetson/L4T profile) inside tmux.

!!! warning "The full launch chain is currently broken"
    `robot.launch.xml` → `autonomy.launch.xml` includes a `global_bringup` package that **does not exist** ([known issue #2](known-issues.md)), and `colcon` cannot build `robot_bringup`/`rviz_behavior_tree_panel` (`README_planning.md` says to `--packages-ignore` them). The practical, known-working path is to launch the needed stages **individually**:

```bash
# inside the robot container, one terminal each (or tmux windows):
colcon build --packages-ignore robot_bringup rviz_behavior_tree_panel
source install/setup.bash

# 1. MoveIt + xArm6 driver (the arm will enable — expect it to hold position)
ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236

# 2. The Autolab planner is ALREADY started by step 1: the (team-modified) xarm launch it includes
#    spawns move_to_pose_node 15 s later and publishes the static TFs world→link_base and
#    Chassis_1→top_camera. Do NOT start a second planner. (`ros2 run` is not installed in this image.)

# 3. The behavior layer (manipulation + lab-machine executives, behavior tree)
ros2 launch behavior_bringup behavior.launch.xml
#    …or the two executives directly:
# ros2 launch manipulation_executive manipulation_executive.launch.xml
# ros2 launch lab_machine_executive lab_machine_executive.launch.xml
```

**You should see:** `move_group` up; after ~15 s `move_to_pose_node` printing `Received command: idle` once per second; the executives idle. Sanity check: `ros2 topic echo /planning_state` shows `IDLE` (1 Hz). Red `shape_mask: Missing transform for shape mesh` lines are cosmetic.

## 2. Perception

The detection models run on the external camera-edge box (not in this repo). Once it's up:

```bash
# on the robot host machine, FROM THE REPO ROOT (the script uses the relative
# path ./mobility/transform_annotated.npy for the LiDAR extrinsic):
cd ~/coding/Autolab
python3 pointcloud_lidar_accumulated.py
```

Use the **system** `python3` in a plain terminal — if the prompt shows a `(venv)`, `deactivate` first (ROS Jazzy is sourced by `~/.bashrc`; the repo `./venv` lacks scipy). The host's `ROS_DOMAIN_ID=1` matches the robot container's, so these topics reach the planner directly.

Start the Velodyne driver too — it lives in a separate host workspace:

```bash
cd ~/coding/vlp16 && source install/setup.bash
ros2 launch vlp16_rviz vlp16_rviz.launch.py start_rviz:=false    # LiDAR at 192.168.1.202 → /velodyne_points
```

**You should see:** `Connected to ws://192.168.1.101:8766` — and then **silence** while the Xavier is idle (it only streams in inspect/servo mode; `idle` messages are ignored). Put it in inspect mode with `test_server.py` and `/inspect/wellplates`, `/inspect/apriltags`, `/inspect/annotated_image`, `/zed_pointcloud` start publishing (`ros2 topic hz /inspect/wellplates`). This one script carries **both** the detection poses (live, every frame) and the point cloud (cached snapshot) — without it the planner has no targets and MoveIt has no obstacles.

!!! note "⚠ local file, not in git"
    `pointcloud_lidar_accumulated.py` and `mobility/` exist only on the robot host — see [Perception](subsystems/perception.md).

## 3. Ground Control Station

```bash
# one-time: in the ROOT docker-compose.yaml, un-comment the include line
#     # - gcs/docker/docker-compose.yaml
# (gcs/docker/docker-compose.yaml can't run standalone with -f: it uses the
#  autolab_network that only the root compose defines)
autolab up gcs-real             # host-networked GCS — REQUIRED with a real robot, see below
autolab connect gcs             # or: docker exec -it gcs-real bash
```

!!! warning "Use `gcs-real`, not `gcs`, on the robot machine"
    Both containers export `ROS_LOCALHOST_ONLY=1` (`robot/docker/.bashrc:13`, `gcs/docker/.bashrc:13`) and the robot container is `network_mode: host`. The default `gcs` service sits on the `autolab_network` bridge — a different network namespace — so its ROS 2 nodes **cannot discover the robot's** (localhost-only DDS never leaves a namespace). `gcs-real` (profiles `hitl`/`deploy`) is host-networked and shares localhost with the robot container, so run the GCS **on the same machine as the robot stack** with `autolab up gcs-real`. Sanity check from the GCS shell: `ros2 topic list` must show `/robot_1/behavior/...` topics. ([known issue #25](known-issues.md))

Once the include is enabled, note that plain `autolab up` (no service name) will also start `gcs`, `ui`, `elasticsearch`, `kibana`, and `filebeat` alongside the robot.

Mosquitto starts automatically inside the GCS container (ports 1883 + 9001). Then start the **three glue nodes that no launch file covers** ([known issue #8](known-issues.md)) — without them the UI is talking to nobody:

```bash
# inside the gcs container, after bws && sws — one terminal each:
ros2 run routine_executor routine_executor_node
ros2 run gcs_monitoring mqtt_ros2_bridge      # MQTT cmd/* → ROS
ros2 run gcs_monitoring ros2_mqtt_bridge      # ROS topics → MQTT ros2/*
```

Bridge env vars (defaults in parentheses): `MQTT_HOST` (`localhost`), `MQTT_PORT` (`1883`), `ROS2MQTT_DISCOVERY_INTERVAL` (`10.0`).

**You should see:** `routine_executor` publishing idle status — verify from any GCS shell with `ros2 topic echo /routine_executor/status`, or over MQTT: `mosquitto_sub -t 'ros2/#' -v`.

Optional operator console (RViz + rqt panels): `ros2 launch gcs_bringup gcs.launch.xml`.

## 4. Web UI

Pick one:

```bash
# (requires the gcs include enabled in the root compose — see step 3)
# dev server with hot reload → http://localhost:3000
COMPOSE_PROFILES=ui-dev autolab up ui-dev

# production build (nginx) → http://localhost:80
autolab up ui

# or straight on the host:
cd gcs/ui && npm install && npm start
```

The UI connects to MQTT at `ws://<hostname-you-browsed-to>:9001`, so opening it from another machine works **except** the camera panel, which hardcodes `localhost` ([known issue #15](known-issues.md)).

## 4b. Smoke tests before your first routine

Run these in order — each isolates one layer, and #2 decides whether the arm can be trusted to move toward a detection at all.

1. **Xavier → ROS:** `python3 ~/coding/percep/test_server.py` → `inspect`; on the host: `ros2 topic hz /inspect/wellplates` and `ros2 topic echo --once /inspect/wellplates | grep -c "ns: wellplates"` (= wellplates seen in that frame). Send `idle` afterwards.
2. **`top_camera` TF:** `ros2 run tf2_ros tf2_echo world top_camera` must print `Translation: [-0.013, 0.266, 1.477]` (published by `planning.launch.xml` via the modified xarm launch). If it errors, step 1 isn't fully up yet.
3. **First arm motion, known pose:** `ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home_offset'}"`; watch `ros2 topic echo /planning_state` go `PLANNING → EXECUTING → SUCCESS`.
4. **One step through the whole chain:** in the UI, run `{"steps":[{"name":"pick_base"}],"max_retries":1,"robot":"robot_1"}` — UI → MQTT → executor → executive → planner → Xavier servo.

## 5. Run a routine from the browser

1. **Begin →** takes you from the welcome page to the routine editor. (The "Objects detected: 3 items" banner and the three feature cards are static text, not live data.)
2. The textarea is pre-filled with the default routine. Edit it or paste your own. The schema:

```json
{
  "steps": [
    { "name": "pick_base" },
    { "name": "place", "target_machine": "ot2" },
    { "name": "ot2", "parameters_json": { "steps": [
        { "action": "pick_up_tips", "resource_name": "tip_rack", "well_indices": [0] },
        { "action": "aspirate", "resource_name": "tube_rack", "well_indices": [0], "volumes": [200] },
        { "action": "dispense", "resource_name": "empty_plate", "well_indices": [0, 5, 10, 15, 20], "volumes": [40, 40, 40, 40, 40] },
        { "action": "return_tips" }
    ]}},
    { "name": "pick_up" },
    { "name": "place", "target_machine": "shaker" },
    { "name": "shaker", "pwm": 150 },
    { "name": "wait", "time_s": 20 },
    { "name": "shaker", "pwm": 0 },
    { "name": "pick_up" }
  ],
  "max_retries": 3,
  "robot": "robot_1"
}
```

3. **Continue →** publishes the JSON to MQTT topic `cmd/routine_executor/start_routine_cmd` and shows a loader (a fixed 5-second animation, not real readiness), then the live progress view: one dot per step, retry counts, and error/success cards fed by `ros2/routine_executor/status`.
4. **Cancel** works. **Pause/Resume do nothing end-to-end** — the MQTT→ROS bridge drops them ([known issue #1](known-issues.md)). Pause/resume *do* work at the ROS level: `ros2 topic pub --once /routine_executor/pause std_msgs/String "{}"`.

### Step vocabulary

Defined in `gcs/ros_ws/src/routine_executor/routine_executor/step_config.py`:

| Step `name` | Required params | What it commands |
|---|---|---|
| `pick_base` | — | Pick well plate from the robot base — the grasp itself is the Xavier's servo mode ([Xavier](subsystems/xavier.md)); with its `DEBUG = True` nothing moves and the step still reports SUCCESS ([#32](known-issues.md)) |
| `pick_up` | — | Inspect → plan above the plate → Xavier servo grasp → home. Same `DEBUG` caveat |
| `place` | `target_machine` (`ot2` \| `shaker`) | Move to 25 cm above the machine's AprilTag, dwell 10 s, return home — ⚠ **does not lower or release the plate** ([#21](known-issues.md)) |
| `pick_wellplate` | `target_machine` | Pick from a specific machine — ⚠ **broken, never dispatches** ([#22](known-issues.md)) |
| `shaker` | `pwm` (int) | Set shaker PWM |
| `ot2` | `parameters_json` (object or string) | Run an OT-2 protocol (`pick_up_tips` / `aspirate` / `dispense` / `return_tips` steps) |
| `wait` | `time_s` | Executor-side dwell, no robot command |

## Alternatives to the UI

Same pipeline, different entry points — useful for debugging one layer at a time:

```bash
# CLI (inside gcs container; ROS directly, skips MQTT + UI):
python3 gcs/ros_ws/src/routine_executor/scripts/run_routine.py \
    pick_base "place:target_machine=ot2" ot2 pick_up "shaker:pwm=200"

# Shell end-to-end smoke test (skips the routine executor too; raw ros2 topic pub
# of ManipulationCommand/LabMachineCommand + polls the *_status topics).
# ⚠ its steps 4–5 (pick_up + place→shaker) are missing — known issue #23:
./test_full_routine.sh

# Single raw commands (see `test commands.txt` for more):
ros2 topic pub --once /behavior/manipulation_command behavior_tree_msgs/msg/ManipulationCommand \
  "{'type': 'place', 'object_type': 'well_plate', 'target_machine': 'ot2'}"
ros2 topic pub --once /behavior/lab_machine_command behavior_tree_msgs/msg/LabMachineCommand \
  "{device: 'shaker', action: 'protocol', parameters_json: '{\"pwm\": 100}'}"

# Drive just the planner:
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_wellplate'}"
ros2 topic echo /planning_state
```

## Watching it run

- `ros2 topic echo /behavior/manipulation_phase` — the manipulation executive's phase machine (`IDLE → ACTIVATING_INSPECT → PLANNING → …`)
- `ros2 topic echo /planning_state` — planner state
- `ros2 topic echo /routine_executor/status` — routine progress (what the UI shows)
- RViz (`ros2 launch gcs_bringup gcs.launch.xml` or the MoveIt RViz config) — planned trajectories on `/display_planned_path`, target markers, octomap
- Record a debugging bag: `bash record_behavior_bag.sh robot_1` (writes `~/behavior_debug_<timestamp>/`)

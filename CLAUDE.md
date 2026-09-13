# CLAUDE.md — launching the whole Autolab framework

Operating notes for this repo on the robot Jetson (`labx`). Everything below was
re-verified on hardware on **2026-09-12**. Longer prose lives in `docs/`; this
file is the short, copy-pasteable path plus the traps that cost real time.

---

## 1. Machine map

| Machine | Address | What it owns |
|---|---|---|
| Robot Jetson `labx` (this box) | `192.168.1.100` (wired), `192.168.10.3` (WiFi) | robot container, planner, MoveIt, `pc3.py` bridge |
| Camera-edge Xavier | `xavier.autolab` → `192.168.10.7`, also `192.168.1.101` | both ZED X cameras, YOLO, AprilTags, **visual servo / the grasp** |
| GCS laptop (Juan's) | `gcs.autolab` → `192.168.10.4` | React UI on `:3000`, Mosquitto WebSocket on `:9001` |
| xArm6 controller | `192.168.1.236` | the arm |
| VLP-16 LiDAR | `192.168.1.202` | optional |

SSH to the Xavier is **password-only** from this box (`autolab@`). Our key is
offered and rejected, so any step that runs a command there needs a human to
type the password. `ssh-copy-id autolab@192.168.1.101` once would fix that.

---

## 2. The trap that breaks everything silently

The robot container sets its real ROS environment in `~/.bashrc`, **not** in the
Docker env:

* `docker inspect` shows `ROS_DOMAIN_ID=0`; `.bashrc` overrides it to **1**.
* `.bashrc` also sets `ROS_LOCALHOST_ONLY=1`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  `FASTRTPS_DEFAULT_PROFILES_FILE`, and sources the built workspace (`sws`).

So every command you run inside the container must go through a **login shell**:

```bash
docker exec autolab-robot-1 bash -lc '<command>'
```

If you instead source `/opt/ros/humble/setup.bash` by hand, the stack comes up on
domain 0. It looks completely healthy — the launch log prints `Received command:
idle` once a second — but `/planning_command` and `/planning_state` never appear
to anyone else, and the host bridge cannot see it. Symptom: `ros2 topic list`
shows only the `/inspect/*` topics and `/ws_visualizer`.

After any domain mix-up, the CLI daemon caches the wrong graph and `ros2 topic
echo` dies with `ResponseError: unknown tag 'rclpy.type_hash.TypeHash'`. Fix:

```bash
docker exec autolab-robot-1 bash -lc 'ros2 daemon stop && ros2 daemon start'
```

---

## 3. Launch sequence

### 3.1 Camera-edge (Xavier) — needed for anything vision-guided

Password SSH, so a human does this:

```bash
ssh autolab@192.168.1.101
tmux new -s edge
cd ~/Autolab-Camera-Edge && python3 main.py     # SYSTEM python; its .venv lacks websockets
```

Ready when it prints control `:8765`, stream `:8766`, raw `:8767`.
Check from here without logging in:

```bash
for p in 8765 8766 8767; do timeout 3 bash -c "echo > /dev/tcp/xavier.autolab/$p" \
  && echo "$p OPEN" || echo "$p closed"; done
```

### 3.2 Mode control (inspect / servo / idle)

Three ways, in order of what actually works from this box:

1. **CLI, works right now, no SSH** — `/usr/bin/python3` is the only interpreter
   here with `websockets`:
   ```bash
   /usr/bin/python3 ~/coding/percep/test_server.py ws://192.168.1.101:8765
   # prompt: servo | inspect | idle | status | stop
   ```
2. **Xavier dashboard with mode buttons** — `http://192.168.1.101:8080/test.html`
   (host box `192.168.1.101`). Only exists while someone runs
   `cd ~/Autolab-Camera-Edge/example_client && python3 -m http.server 8080` on the
   Xavier. **Not running by default.**
3. **GCS UI** `http://gcs.autolab:3000` — shows the three camera tiles but has
   **no mode buttons**; it never opens port 8765. See §5.

One-shot status check:

```bash
/usr/bin/python3 - <<'EOF'
import asyncio, json, websockets
async def m():
    async with websockets.connect("ws://192.168.1.101:8765", open_timeout=8) as ws:
        print(await ws.recv())
        await ws.send(json.dumps({"cmd": "status"}))
        print(await ws.recv())
asyncio.run(m())
EOF
```

Healthy reply: `{"ok": true, "mode": "inspect", "worker_alive": true,
"wrist_camera_connected": true, "base_camera_connected": true}`.

### 3.3 Bridge the Xavier into ROS (host)

`pc3.py` turns the `:8766` stream into ROS topics. Use **system** python (the
repo `venv/` and `.venv_jog/` both lack `websockets`), in a ROS-sourced shell:

```bash
cd ~/coding/Autolab
setsid nohup env PYTHONUNBUFFERED=1 ROS_DOMAIN_ID=1 \
  bash -lc 'source /opt/ros/jazzy/setup.bash && exec /usr/bin/python3 pc3.py' \
  > /tmp/pc3.log 2>&1 &
```

Ready when the log says `Connected to ws://192.168.1.101:8766`. It retries once a
second, so starting it before the Xavier is fine. Publishes `/inspect/apriltags`,
`/inspect/wellplates`, `/inspect/image`, `/inspect/annotated_image`,
`/zed_pointcloud`, and in servo mode `/servo/detections`, `/servo/target`,
`/servo/image`, `/servo/annotated_image`.

Stop with `pkill -INT -f 'pc3[.]py'` (bracket so pkill misses its own shell).

### 3.4 Planning stack (robot container)

```bash
xhost +local:                                   # let the container open RViz
docker start autolab-robot-1
docker exec autolab-robot-1 tmux kill-session -t robot_bringup   # stop the auto-rebuild
docker exec autolab-robot-1 tmux new -d -s planning \
  'bash -lc "export DISPLAY=:1; ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236 2>&1 | tee /home/robot/planning.log"'
```

**The arm enables and holds position here.** One launch starts the xArm driver,
`move_group`, RViz, the `Chassis_1→top_camera` static TF, and — after a 15 s
`TimerAction` — `move_to_pose_node`. Never start the planner separately; you get
two planners answering `/planning_command`.

Ready checks:

```bash
docker exec autolab-robot-1 bash -c 'grep -c "Received command: idle" /home/robot/planning.log'
docker exec autolab-robot-1 bash -lc 'ros2 topic echo --once /planning_state'   # data: IDLE
docker exec autolab-robot-1 bash -lc 'ros2 topic hz /joint_states'              # ~10 Hz
docker exec autolab-robot-1 bash -lc 'ros2 topic hz /inspect/apriltags'         # ~5.5 Hz
```

Rebuild only if a source is newer than the binary:

```bash
ls -l --time-style=+%F_%T robot/ros_ws/src/autonomy/5_planning/global_planner/src/*.?pp
docker exec autolab-robot-1 ls -l --time-style=+%F_%T \
  /home/robot/AutoLab/robot/ros_ws/build/global_planner/move_to_pose_node
docker exec autolab-robot-1 bash -lc 'bws --packages-select global_planner && sws'
```

### 3.5 Commanding a plan

```bash
docker exec autolab-robot-1 bash -lc 'ros2 topic echo /planning_state'   # watch in one shell
docker exec autolab-robot-1 bash -lc \
  "ros2 topic pub --once /planning_command std_msgs/String \"{data: 'plan_home_offset'}\""
```

| Command | Target | Needs camera |
|---|---|---|
| `plan_home_offset` | fixed safe home | no |
| `plan_april_1` | 25 cm above AprilTag 1 (OT-2) | tag 1 visible |
| `plan_april_2` | 25 cm above AprilTag 2 (shaker) | tag 2 visible |
| `plan_wellplate` | 25 cm above the plate (+10 cm y, −10 cm x) | plate at conf ≥ 0.8 |
| `idle` | clears the last command | no |

State goes `PLANNING → EXECUTING → SUCCESS`. Check which tags are actually in
view before using `plan_april_*`:

```bash
docker exec autolab-robot-1 bash -lc 'ros2 topic echo --once /inspect/apriltags' | grep -E "^\s+id:" | sort -u
```

### 3.6 Visual servo mode — the grasp

Servo lives entirely on the Xavier. It takes the arm over the **xArm SDK
directly**, bypassing MoveIt. Before switching:

* e-stop in hand, table clear;
* no MoveIt trajectory executing (`/planning_state` must read `IDLE`);
* the gripper must be attached, or every Modbus call fails soft and the grasp
  "fails" 4 times while the executive still reports SUCCESS;
* the Xavier's `config.py` must have `DEBUG = False` for the arm to move at all.
  With `DEBUG = True` nothing moves and the run still reports success.

Then switch with the CLI client from §3.2 and type `servo`. Go back with
`inspect` (or `idle`) when done. The worker going `worker_alive: false` is what
`manipulation_executive` waits on.

### 3.7 Shutdown, in order

```bash
# park the arm first
docker exec autolab-robot-1 bash -lc \
  "ros2 topic pub --once /planning_command std_msgs/String \"{data: 'plan_home_offset'}\""
docker exec autolab-robot-1 tmux kill-session -t planning
docker stop autolab-robot-1
pkill -INT -f 'pc3[.]py'
# camera-edge: set mode idle, then Ctrl+C main.py in the Xavier's tmux
```

---

## 4. Ground control station

Mosquitto WebSocket on `gcs.autolab:9001` is up; `1883` is not exposed and the
OT-2 wrapper port `8000` is closed, so OT-2 steps will fail until someone starts
`~/ot2-wrapper` where `lab_machines.yaml` points. `routine_executor_node` and the
two MQTT bridges have no launch file and must be started by hand — `docs/running.md`.

---

## 5. The UI at `http://gcs.autolab:3000`

A Create-React-App **dev server** (`X-Powered-By: Express`, title "Apex") running
on Juan's laptop. Its camera tiles work because the copy deployed there is
**patched**: the served bundle has `HOST = 'xavier.autolab'`, while this repo's
`gcs/ui/src/CameraPanel.js` still has `HOST = 'localhost'`. That patch is not in
git. A UI built from this repo shows "No signal" unless you edit `HOST` or open
an SSH tunnel for 8766/8767.

The browser connects straight to `ws://xavier.autolab:8766` and `:8767`, so
whatever machine runs the browser must resolve `xavier.autolab` and reach those
ports. The panel never opens `:8765`, which is why it has no mode buttons.

---

## 6. Gotchas worth remembering

* `ros2 run` does not exist in the robot image — only `launch node param pkg
  service topic control`. Use `ros2 launch`, the binary under
  `install/global_planner/lib/global_planner/`, or run the tool on the host.
  This is why `ros2 run tf2_ros tf2_echo` silently prints nothing in there.
* `shape_mask: Missing transform for shape mesh` from `move_group` is cosmetic.
* `Could not find the planner configuration 'BITstar' / 'InformedRRTstar'` is
  harmless unless you request those planners.
* `Could not enable FIFO RT scheduling policy` is expected (unprivileged container).
* `sequence size exceeds remaining buffer` on `ros2 topic echo` is a CDR warning
  from the large point-cloud messages; the data still arrives.
* Killing the launch can leave `rviz2` / `move_group` as **zombies** (`STAT Z`).
  They hold no DDS participant — ignore them, `kill -9` will not clear them.
* `pkill` matches its own shell. Always bracket: `pkill -INT -f 'pc3[.]py'`.
* Confirm what `xavier.autolab` resolves to before debugging: `getent hosts
  xavier.autolab`. Router DNS points it at the WiFi address `192.168.10.7`,
  while the repo hardcodes the wired `192.168.1.101`. Both work.

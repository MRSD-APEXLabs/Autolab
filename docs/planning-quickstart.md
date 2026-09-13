# Planning Quickstart — from power-on (copy-paste)

This page assumes nothing. It starts with the machines switched off and ends with the arm moving to home and to an AprilTag. Every command is meant to be pasted as-is; the line after it says what it does and what you should see. Verified on hardware on 2026-09-01.

!!! danger "The arm moves from Part F onwards"
    Keep the xArm e-stop within reach, clear the table around the arm, and make sure nobody else is commanding it (no joystick script, no UFactory Studio browser tab).

## Part A — Power on (physical)

1. **xArm6 control box** — switch it on, release the e-stop (twist), wait until the controller is ready (about 30 s).
2. **Xavier** (the camera computer with the two ZED cameras) — power it on, wait about a minute.
3. **Robot Jetson** (this computer, `labx`) — power on, log in.
4. **Plug the Ethernet cable** into the Jetson. The arm, the Xavier and the LiDAR are on that cable's network (`192.168.1.x`); without it none of them is reachable.
5. **LiDAR** (VLP-16) — optional for planning; leave it off unless you want the LiDAR cloud.

## Part B — Check the network (Jetson)

Open a terminal (`Ctrl`+`Alt`+`T`). Call it **T1**.

```bash
ip -4 -br addr | grep enP2p1s0
```
Must say `UP` and `192.168.1.100`. If it says `DOWN`, the cable isn't in.

```bash
ping -c1 192.168.1.236 && echo ARM OK
```
The arm answers.

```bash
ping -c1 192.168.1.101 && echo XAVIER OK
```
The Xavier answers. (If not, wait another minute — it's still booting.)

## Part C — Start the camera + YOLO server on the Xavier

Still in **T1**:

```bash
ssh autolab@192.168.1.101
```
Type the Xavier password when asked. The prompt changes to `autolab@ubuntu`.

```bash
tmux new -s edge
```
Opens a session that survives if the ssh connection drops.

```bash
cd ~/Autolab-Camera-Edge && python3 main.py
```
Starts the server with the **system** Python (do **not** run `source .venv/bin/activate` — that venv is missing `websockets`). Wait for these three lines:

```
Control server:    ws://0.0.0.0:8765
Stream server:     ws://0.0.0.0:8766
Raw stream server: ws://0.0.0.0:8767
```

Leave it running. Press `Ctrl`+`b`, then `d` to detach from tmux (the server keeps running), then `exit` to leave the Xavier.

## Part D — Camera dashboard (see the feeds, the counts, and switch modes)

Open a second terminal **T2**:

```bash
ssh autolab@192.168.1.101
cd ~/Autolab-Camera-Edge/example_client && python3 -m http.server 8080
```
Serves the dashboard page. Leave this terminal alone.

In Firefox on the Jetson open **http://192.168.1.101:8080/test.html**:

1. In the **host** box type `192.168.1.101`.
2. Click **Connect** (green). The status badge turns green.
3. Click **Inspect** (amber). The **Active Stream** tile shows the top camera with chips `wellplates N · apriltags N · pts N · FPS`; **Wrist** and **Base** tiles show the two raw cameras.

Zero counts are normal until a well plate / AprilTag is in the top camera's view with the lights on.

## Part E — Bridge the Xavier into ROS (Jetson)

Open terminal **T3**. If its prompt starts with `(something)` in brackets, type `deactivate` first.

```bash
cd ~/coding/Autolab && python3 pc3.py
```
Turns the Xavier's stream into ROS topics (`/inspect/apriltags`, `/inspect/wellplates`, `/inspect/image`, `/zed_pointcloud`). You should see `Connected to ws://192.168.1.101:8766`. Leave it running.

Open terminal **T4** and check:

```bash
ros2 topic hz /inspect/apriltags
```
Prints `average rate: ~7` while the Xavier is in Inspect. `Ctrl`+`C` to stop the check.

## Part F — Start the robot software (Jetson) — the arm enables here

In **T4**:

```bash
xhost +local:
```
Allows the robot container to open its RViz window on your screen.

```bash
docker start autolab-robot-1
```
Starts the existing robot container (nothing is rebuilt).

```bash
autolab connect robot
```
Puts you inside the container. The prompt becomes `[robot_1]robot@labx:~/AutoLab/robot/ros_ws$`.

```bash
tmux kill-session -t robot_bringup
```
Do this immediately: it stops the container's automatic rebuild + old launch. (If it says `no server running`, that's fine.)

```bash
sws
```
Loads the already-built ROS workspace into this shell.

```bash
ros2 launch planning_bringup planning.launch.xml robot_ip:=192.168.1.236
```

!!! danger "Launch it from a login shell"
    If you script this with `docker exec` instead of `autolab connect robot`, use `docker exec autolab-robot-1 bash -lc '…'`. The container's ROS domain (**1**), `ROS_LOCALHOST_ONLY`, RMW and Fast DDS profile are set in `.bashrc`, not in the Docker env (which says domain 0). Sourcing `setup.bash` by hand brings the whole stack up on domain 0, where it looks healthy but `/planning_command` and `/planning_state` are invisible to everything else ([#37](known-issues.md)).
Starts everything for planning in one go: the xArm driver (**the arm enables and holds position**), MoveIt (`move_group`), an RViz window, the camera transform, and — 15 seconds later — the Autolab planner `move_to_pose_node`. Ready when you see `Received command: idle` repeating once per second. Red lines containing `shape_mask: Missing transform` are harmless.

Do **not** start `move_to_pose_node` yourself — it's already running (and `ros2 run` does not exist in this container).

## Part G — Move the arm

Open terminal **T5**:

```bash
ros2 topic echo /planning_state
```
Shows `data: IDLE` once a second. Leave it running; this is where you watch progress.

Open terminal **T6** and send commands one at a time, watching T5 go `PLANNING → EXECUTING → SUCCESS`:

```bash
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_home_offset'}"
```
Moves to the fixed safe "home" pose. Needs no camera. Always do this first.

```bash
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_2'}"
```
Moves to 25 cm above AprilTag **2** (the shaker). Tag 2 must show in the dashboard chips (`apriltags ≥ 1`).

```bash
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_april_1'}"
```
Same for tag **1** (the OT-2).

```bash
ros2 topic pub --once /planning_command std_msgs/String "{data: 'plan_wellplate'}"
```
Moves to 25 cm above the detected well plate (shifted +10 cm y, −10 cm x). The plate must be detected with ≥ 80 % confidence — lights on.

```bash
ros2 topic pub --once /planning_command std_msgs/String "{data: 'idle'}"
```
Does nothing (clears the last command).

## Part H — Seeing what the planner sees (optional)

```bash
ros2 run rqt_image_view rqt_image_view /inspect/annotated_image
```
The top-camera image with boxes and tag IDs drawn (run on the Jetson, any free terminal).

```bash
ros2 topic echo --once /inspect/apriltags | grep -E "^\s+id:"
```
Lists the tag IDs seen right now.

```bash
ros2 run tf2_ros tf2_echo world top_camera
```
Must print `Translation: [-0.013, 0.266, 1.477]` — the camera-to-world transform is up.

In the RViz window from Part F: *Add → By topic* → MarkerArray `/inspect/apriltags`, `/inspect/wellplates`, `/target_pose_marker`, PointCloud2 `/zed_pointcloud`; set each display's **Reliability** to **Best Effort**.

## Part I — Shut down (in this order)

1. T6: send `plan_home_offset` so the arm parks.
2. T4 (container): `Ctrl`+`C` the launch, wait for it to finish, then `exit`; on the Jetson: `docker stop autolab-robot-1`.
3. T3: `Ctrl`+`C` `pc3.py`.
4. Dashboard: click **Idle**. T2: `Ctrl`+`C` the web server, `exit`.
5. Xavier server: `ssh autolab@192.168.1.101`, `tmux attach -t edge`, `Ctrl`+`C`, `exit`, `exit` — only if you want the cameras off.
6. Power off the Xavier, then the arm control box (engage the e-stop first).

## Errors you will actually see

| What you see | What it means / what to do |
|---|---|
| `April tag ID 2 not found or stale after 10s — aborting` (launch log) | Tag 2 wasn't detected in the last second. Check the dashboard chips, the lights, and that the Xavier is in **Inspect**. |
| `Wellplate not found or stale after 10s` | Same for the plate; it must score ≥ 0.8. |
| `ros2: error: … invalid choice: 'run'` inside the container | Normal here — nothing needed; the planner is already running. |
| `ModuleNotFoundError: No module named 'websockets'` from `pc3.py` | A venv is active in that terminal — type `deactivate`, run again. |
| `ModuleNotFoundError: No module named 'websockets'` from `main.py` on the Xavier | You activated its `.venv` — `deactivate`, run again with plain `python3`. |
| Dashboard tiles say *No signal* | The Xavier server isn't running (Part C) or the host box isn't `192.168.1.101`. |
| `[moveit.ros.perception.shape_mask]: Missing transform for shape mesh` | Cosmetic. Ignore. |
| T5 shows `ERROR` | The reason is in the launch log (T4) — usually one of the two "not found or stale" messages above, or an IK/collision failure for that target. |

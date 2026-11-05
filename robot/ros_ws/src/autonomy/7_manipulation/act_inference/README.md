# F/T placing inference

This folder includes the runtime and checkpoint used by
`apple_server_manip/scripts/place_on_opentrons_ft.sh`. No teleop checkout is needed.
`models/xarm6_placing_act_ft_20261029/` contains `config.pkl`, `dataset_stats.pkl`, and
`policy_best.ckpt` (step 2000, 67 demonstrations). The older
`models/xarm_final_ckpt_placing/` checkpoint remains available by explicit path.

The new policy uses wrist RGB, metric depth and validity, and the compensated external
F/T wrench (N, Nm, SDK sensor coordinates). TCP pose is hidden from the model; measured
TCP pose anchors its tool-relative action chunks. The model's recorded image size and
capture-time anchoring are respected.

## ROS services

Execution requires the patched `xarm_controller` driver to acknowledge exclusive SDK
ownership before motion. ACT stops the ROS arm controller for the complete run and
restores it from measured joints after successful completion. Old/unavailable drivers
block execution. See [the mode 1 handoff and build instructions](patches/README.md).

With the ROS workspace sourced and dependencies installed in the Python environment:

```bash
cd src/autonomy/7_manipulation/act_inference
python3 run_act_node.py --ros-args -p execute:=false  # prediction only
# python3 run_act_node.py                          # executes after /start
ros2 service call /act_inference/start/ot2 std_srvs/srv/Trigger
# Or choose another target (one placement runs at a time):
ros2 service call /act_inference/start/shaker std_srvs/srv/Trigger
ros2 service call /act_inference/start/base std_srvs/srv/Trigger
ros2 topic echo /act_inference/status
ros2 service call /act_inference/stop std_srvs/srv/Trigger
```

OT-2 and shaker run the same ACT placing policy. `/act_inference/start` remains an alias
for `/act_inference/start/ot2`. Before either policy executes, the arm's base-frame Euler yaw
is aligned to **90 degrees** at **3 degrees/s**, holding its current XYZ, roll, and pitch.
Alignment is skipped when yaw is already within 0.5 degrees. Measured arrival is confirmed
before starting ACT; a stopped or failed alignment prevents the policy from starting.
During yaw, measured XYZ drift beyond 2 mm stops the run. Initial report feedback is
cross-checked against a direct axis-angle query before enabling any motion.
Prediction-only mode reads/logs the yaw adjustment without commanding it.
Start returns immediately; any start while another target
is running returns `already running`. `/act_inference/stop` cancels whichever target is
active. The shared `/act_inference/status` remains a `std_msgs/String` topic (not a service).
Status is JSON at 2 Hz and on
completion, transient local: `running`, `result` (`done`, `stopped`, `failed`), `message`,
`phase` (`idle`, `preparing`, `running`), `started` (monotonic time), and `target`
(`ot2`, `shaker`, `base`; null before the first run, retained after completion).
`sdk_control_held` shows whether this node still holds the driver's exclusive lease.
After a stopped/failed run, ROS remains paused; faults are never automatically cleared.
For OT-2/shaker, a successful policy result (settled or configured duration elapsed) is followed
by gripper release and a 70 mm vertical retreat. `done` is published only after that sequence
finishes; it is not an independent placement measurement. Contact detection or cancellation returns `stopped`;
sensor, inference, camera, workspace, and robot errors return `failed`.

The node defaults now match the F/T placing script: the bundled F/T checkpoint,
60 s maximum duration, 50 Hz control, translation 0.01 m/s, rotation enabled at
0.15 rad/s, workspace radius 0.15 m, and no final offset. It preloads and warms the
policy once, then creates a camera/robot session for each ACT run. It selects
`/camera_hub/select_zedx_nano` and reads `/zedx_nano` topics by default, as before.
`execute:=false` makes no robot or sensor configuration writes; F/T sampling must
already be on in xArm Studio. Execution enables sampling if needed and restores it
on exit; it never zeroes the sensor or selects force-control mode.

`manipulation_bringup` defaults to the script's 60 s ACT limit:

```bash
ros2 launch manipulation_bringup manipulation.launch.xml act_duration_sec:=60.0
```

ROS parameters also expose `ckpt_dir`, `robot_ip`, `camera_source`, `ros_namespace`,
`select_service` (empty skips switching), `select_timeout`, `control_hz`,
`translation_speed`, `rotation_speed`, `max_tracking_error`, `enable_rotation`,
`workspace_radius`, `duration_sec`, `query_interval`, `prediction_timeout`,
`contact_gap`, `contact_hold`, `contact_max_lag`, `settle_motion`, `settle_queries`,
`approach_speed`, `approach_distance`, `ensemble`, and `final_offset_mm` (XYZ).
Camera parameters are `camera_host`, `camera_port`, `record_image_width`, `depth_width`,
`depth_backend`, `depth_model`, `depth_repo`, `depth_models_dir`, `max_frame_age`, and `preview`.

Arrival holds the target after a chunk predicts under 2 mm of travel and ends after
three consecutive arrived chunks. Near the goal, translation slows to 3 mm/s and
averages up to five recent chunks. Execution stops if downward target lag indicates
sustained contact. `/stop` also interrupts initial inference and any final-offset move.

## OT-2/shaker release and retreat

After the ACT rollout returns `done`, the node opens the custom gripper (Modbus device
0x08, setpoint 0 at 57600 baud), waits **4 seconds** for jaw travel, then moves **70 mm up
along robot base-frame +Z**, holding XY and orientation. The lift uses the ACT translation
speed (`translation_speed`, default **10 mm/s**), independently of base-placement speed.
The gripper acknowledgement confirms command acceptance; this gripper has no measured
jaw-position feedback. A rejected command fails the run before lifting.

The shared status remains running and the start services stay locked through opening and
retreat. `/act_inference/stop` cancels the opening wait or lift. A stopped/failed policy
skips release and retreat. Prediction-only runs only log the sequence and read the TCP;
they send no gripper or motion commands. Base placement keeps its existing behavior.

## Base placement

`/act_inference/start/base` moves directly to the supplied `arm.get_position()` destination:

```text
XYZ (mm):       [32.074207, 257.162689, 40.484722]
roll/pitch/yaw (degrees): [-179.67395, -1.5738, -0.3]
```

The first `0` in the SDK result is its success code, not part of the pose. Euler angles
are converted to the rotation-vector representation used by the servo interface.
Translation is limited to **15 mm/s**, rotation to **6 degrees/s**, with 50 Hz updates.
It first moves to the target XY/orientation at **10 mm above the target Z** (50.484722 mm),
confirms measured arrival, then descends vertically to the target Z (40.484722 mm).
Both stages use the same slow motion limits and share the timeout. It does not open the
gripper. It skips camera switching, camera reads, F/T setup, and
policy inference. The shared node still preloads its ACT checkpoint at startup.

Execution reports `done` after the target has reached the destination and measured feedback
stays within 1 mm and 0.5 degrees for 0.3 s. Cancellation reports `stopped`; robot errors
or the 180 s timeout report `failed`. The timeout is independent of ACT's `duration_sec`.
With `execute:=false`, base placement reads the TCP, logs the destination, and returns
`done` without enabling or commanding the arm.

Node parameters: `base_pose_mm_deg` (six XYZ/RPY values), `base_speed_mm_s` (15.0),
`base_rotation_speed_deg_s` (6.0), `base_timeout_sec` (180.0).
Bringup exposes speeds and timeout:

```bash
ros2 launch manipulation_bringup manipulation.launch.xml \
  act_base_speed_mm_s:=15.0 act_base_rotation_speed_deg_s:=6.0 act_base_timeout_sec:=180.0
```

## Standalone script

```bash
./place_on_opentrons_ft.sh                          # prediction only, direct Nano stream
# Execution uses run_act_node.py and the target services above.
./place_on_opentrons_ft.sh --camera-source ros      # use running camera/depth nodes
python3 run_act_inference.py --help
```

Standalone `--execute` is blocked because it cannot acquire the ROS driver lease.
Use the target services for physical execution. The wrapper uses the current `python3`
environment, sets `OMP_NUM_THREADS=2`, and accepts
the same placing overrides as the original script. Generic `run_act_inference.py`
keeps its existing CLI defaults; use the wrapper for the F/T placing settings.

Install `requirements.txt` with a suitable CUDA PyTorch. Camera transport/depth libraries
come from this workspace's `3_perception/zedx_nano_camera` and `zedx_nano_depth` packages
(or installed equivalents); ROS mode also requires sourced `rclpy`, `sensor_msgs`,
`std_msgs`, and `std_srvs`. Direct streaming needs the existing stereo model weights.

Hardware-free checks:

```bash
ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OMP_NUM_THREADS=2 \
  python3 -m pytest tests -q
```

## Servo timing and diagnostics

The ROS camera now waits for at least two matched RGB/depth pairs, calibration, and a
usable latest frame before starting policy inference. This avoids applying a zero-cadence
startup threshold to the first delayed depth frame. Cadence uses capture stamps as well
as receipt spacing, so queued pairs arriving back-to-back do not look like an unrealistically
fast stream. The cadence allowance stays capped by `max_frame_age`; permanently old data,
stalled streams, and future timestamps still fail. Startup is bounded by 10 seconds and
logs the pair count, capture-to-receipt latency, and bounded frame interval when ready.

Only one publisher is permitted on each Nano RGB/depth input topic. Duplicate bringups
or orphaned depth nodes interleave timestamps and compete for GPU time; ACT now rejects
that condition explicitly. Stop an old launch cleanly and wait for its camera processes
to exit before starting another. Pairing preserves newer incomplete frames when messages
arrive out of order and drops duplicate/older messages without replacing the latest
accepted image. A sustained timestamp reset or stalled source still makes that image
expire through the age watchdog; old timestamps are never rebased to appear fresh.

If a run fails while holding SDK control, restart ACT and the controller manager together
when loading these changes; restarting only ACT loses the local record of the retained lease.

Active motion uses the SDK's report socket for measured pose, joints, controller state,
and errors. It converts reported Euler RPY radians to rotation vectors before using
the pose. Reports older than 250 ms, paused/stopped states, or controller faults stop
execution. Startup still checks the controller synchronously and waits for fresh,
ready feedback after enabling. These checks apply to yaw alignment, base placement,
ACT execution, and the post-placement lift.

Previously each 20 ms tick made six synchronous state/pose getter requests before the
servo command. Even 5 ms per request takes about 35 ms including the servo write, so
network jitter caused missed ticks. The active tick now needs only the servo command;
F/T polling runs in the policy worker under the inference deadline. Control pacing uses
monotonic deadlines and skips missed slots rather than sending catch-up bursts. The
existing speed, tracking, workspace, contact, and inference limits still apply.

If any command interval exceeds 30 ms at the default 50 Hz rate, the node logs
`Servo command gap ... (expected 20.0 ms)` (at most once every two seconds). Save these
warnings and the policy/camera timing lines if physical motion still pulses. The timing
fix has hardware-free regression coverage; smoothness still needs confirmation on the arm.

## Driver validation

The mode remains **1** for ACT. The compiled driver now suppresses joint writes when
unclaimed or SDK-owned. ACT checks `/xarm/sdk_control/status`, strictly stops configured
arm controllers, acquires exclusive SDK control, and retains it across yaw/policy/release
and retreat. Successful runs resynchronize measured joint state before restoring ROS.
Stopped/failed runs retain the SDK lease. Read [the handoff details](patches/README.md)
before rebuilding and restarting the stack.

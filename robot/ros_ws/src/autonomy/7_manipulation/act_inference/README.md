# F/T placing inference

This folder includes the runtime and checkpoints used by `apple_server_manip/scripts/place_on_{opentrons,shaker,base}_ft.sh`.
No teleop checkout is needed. One placing policy per target, each with `config.pkl`, `dataset_stats.pkl` and
`policy_best.ckpt` (all finished SpaceMouse demonstrations of that target, no held-out split; near-seat frames
weighted x4):

| target | checkpoint | demos | held-out check (80:20 run, median end error) |
|---|---|---|---|
| `ot2` | `models/xarm6_placing_act_ft_all_20261105b/` | 103, 11000 steps | 1.34 mm |
| `shaker` | `models/xarm6_shaker_act_ft_20261110/` | 66 (about 8 base parking spots + 17 at three newer spots), 9000 steps | 0.92 mm (x/y 0.19, height 0.66) for the 49-demo version |
| `base` | `models/xarm6_base_act_ft_20261108b/` | 43 (robot base deck bracket; 4 started at the base start pose), 4500 steps | 1.42 mm (x/y 0.74, height 0.84) for the 39-demo version |

The ROS node and the wrapper scripts default to these. The previous OT-2 checkpoints
`models/xarm6_placing_act_ft_all_20261105/` (8000 steps, 75 demonstrations),
`models/xarm6_placing_act_ft_20261029/` (step 2000, 67 demonstrations) and
`models/xarm_final_ckpt_placing/` remain available by explicit path.
The previous shaker checkpoints `models/xarm6_shaker_act_ft_20261108c/` (59 demonstrations, 8000 steps),
`models/xarm6_shaker_act_ft_20261108b/` (54, 7500 steps) and `models/xarm6_shaker_act_ft_20261108/` (49, 6500 steps) are kept too, and so is the previous base
checkpoint `models/xarm6_base_act_ft_20261108/` (39 demonstrations, 4000 steps).

The new policy uses wrist RGB, metric depth and validity, and the compensated external
F/T wrench (N, Nm, SDK sensor coordinates). TCP pose is hidden from the model; measured
TCP pose anchors its tool-relative action chunks. The model's recorded image size and
capture-time anchoring are respected.
The new checkpoint's bundled normalization statistics scale F/T in 1 N / 0.05 Nm
units; keep those statistics together with its config and weights.

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

Each target runs its own ACT placing policy (table above); all three are loaded and warmed up at
startup. `/act_inference/start` remains an alias for `/act_inference/start/ot2`. Before the OT-2 or
shaker policy executes, the arm's base-frame Euler yaw
is aligned to **90 degrees** at **9 degrees/s**, holding its current XYZ, roll, and pitch.
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
by gripper release and a 100 mm vertical retreat. `done` is published only after that sequence
finishes; it is not an independent placement measurement. Contact detection or cancellation returns `stopped`;
sensor, inference, camera, workspace, and robot errors return `failed`.

The node defaults now match the F/T placing scripts: the bundled F/T checkpoints,
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

ROS parameters also expose `ckpt_dir` (OT-2), `shaker_ckpt_dir`, `base_ckpt_dir`, `robot_ip`, `camera_source`, `ros_namespace`,
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
0x08, setpoint 0 at 57600 baud), waits **4 seconds** for jaw travel, then moves **100 mm up
along robot base-frame +Z**, holding XY and orientation. The lift uses the ACT translation
speed (`translation_speed`, default **10 mm/s**), independently of base-placement speed.
The gripper acknowledgement confirms command acceptance; this gripper has no measured
jaw-position feedback. A rejected command fails the run before lifting.

The shared status remains running and the start services stay locked through opening and
retreat. `/act_inference/stop` cancels the opening wait or lift. A stopped/failed policy
skips release and retreat. Prediction-only runs only log the sequence and read the TCP;
they send no gripper or motion commands. Base placement uses the same release sequence
with a **60 mm** lift.

## Base placement

`/act_inference/start/base` places on the base deck by touch (`base_mode:=scripted`, the default; bringup
`act_base_mode`), without camera or policy, from the executive's `plan_home_offset`. Probing starts at the
`arm.get_position()` result

```text
XYZ (mm):       [51.565662, 257.200928, 47.687004]
roll/pitch/yaw (degrees): [179.332982, -2.176667, 1.803213]   (base_pose_mm_deg)
```

(the first `0` in the SDK result is its success code; Euler angles become the servo interface's rotation vector).
Where the plate sits in the gripper varies along X, so a fixed pose only gets it close; the deck's ridge and the
wrist F/T sensor finish the job:

1. Move to the probing start at **15 mm/s**, rotation **6 degrees/s**, 50 Hz, with measured arrival.
2. Hold still 0.5 s, take the in-air F/T baseline (median over 0.3 s), then descend at **2 mm/s** until
   |Fz - baseline| > **1.0 N** for 2 consecutive reads. No touch within **11 mm** below the start fails the run.
3. Hold at the touch height, re-baseline, then slide along base **-X** at **2 mm/s** until the sideways force
   |(Fx, Fy) - baseline| > **1.0 N** for 2 reads: the plate edge is against the ridge. No ridge within **30 mm**
   fails the run. The arm then steps back to its measured X so it no longer pushes on the ridge.

The touch and ridge positions (and how far each is from the start) are printed every run. Then the node opens the
gripper, waits **4 seconds**, and lifts **60 mm along base +Z** at `translation_speed` (default **10 mm/s**).

F/T is `ft_ext_force` (SDK sensor frame, load-compensated) from a second, read-only SDK connection on the fast
`real` report socket, as visual_servoing reads it; the sensor is never zeroed (that drops the controller into
state 5), only baselined. Sampling is enabled before the arm is (and switched off again if this run turned it on);
force mode is refused. A report older than 100 ms, or force readings that never change while holding, fail the run.

`base_debug:=true` (bringup `act_base_debug:=true`) writes every control tick (phase, measured and target XYZ,
wrench, threshold metric, consecutive hits) to `~/.ros/log/act_inference_base_placement/base_placement_<time>.csv`
(`base_debug_dir`), prints a trace line every 0.1 s and the F/T baselines. Use it to tune the thresholds.

`base_mode:=policy` runs the base ACT policy instead: it first moves in one straight line to the base start pose
`[32.055321, 271.166351, 116.803917]` mm, RPY `[-179.675382, -1.57403, -0.737282]` degrees
(`base_start_pose_mm_deg`, about 80 mm above the bracket, where most base demos started; refused from more than
200 mm away), then runs the policy like OT-2/shaker (Nano camera, `base_workspace_radius` 0.15 m), followed by
the same release and 60 mm lift.

Measured arrival at each goal requires feedback within 1 mm and 0.5 degrees for 0.3 s.
Execution reports `done` after placing, releasing, lifting, and driver handback succeed.
Cancellation reports `stopped`; robot errors
or the 180 s timeout report `failed`. The timeout is independent of ACT's `duration_sec`.
With `execute:=false`, base placement reads the TCP, logs the destination, and returns
`done` without enabling or commanding the arm.

Node parameters: `base_mode` (`scripted` or `policy`); `base_pose_mm_deg` (six XYZ/RPY values), `base_speed_mm_s`
(15.0), `base_rotation_speed_deg_s` (6.0), `base_timeout_sec` (180.0, both modes' moves); scripted seating
`base_probe_speed_mm_s` (2.0), `base_probe_depth_mm` (11.0), `base_touch_force_n` (1.0), `base_slide_speed_mm_s` (2.0), `base_slide_max_mm`
(30.0), `base_ridge_force_n` (1.0), `base_force_count` (2), `base_settle_sec` (0.5), `base_baseline_sec` (0.3),
`base_debug` (false), `base_debug_dir`; policy mode `base_ckpt_dir`, `base_start_pose_mm_deg`,
`base_workspace_radius` (0.15 m). Bringup exposes the mode, debug, speeds and timeout:

```bash
ros2 launch manipulation_bringup manipulation.launch.xml act_base_mode:=scripted act_base_debug:=true \
  act_base_speed_mm_s:=15.0 act_base_rotation_speed_deg_s:=6.0 act_base_timeout_sec:=180.0
```

## Standalone script

```bash
./place_on_opentrons_ft.sh                          # prediction only, direct Nano stream
./place_on_shaker_ft.sh                             # shaker policy (start above the clamp, yaw ~90)
./place_on_base_ft.sh                               # base policy (start at the base start pose by hand)
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

The ROS camera waits for at least two matched RGB/depth pairs and calibration before
starting policy inference. There is no camera latency (frame age) limit: the newest frame
is used however old it is; only future timestamps (clock mismatch) fail. Startup is bounded
by 10 seconds and logs the pair count, capture-to-receipt latency, and frame interval when ready.

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

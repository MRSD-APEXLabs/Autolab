# 7_manipulation: visual servo grasp and live ACT inference

| folder | what it is | needs |
|---|---|---|
| `visual_servoing/` | ROS 2 package (`ament_python`). Wrist-camera (ZED X Nano) visual servo grasp of a well plate on the xArm6: YOLO oriented boxes located with the stereo depth, a position-based servo to a hover pose above the plate, then the descend / FSR close / lift sequence of the Xavier's Camera-Edge. Driven by `6_behavior/manipulation_executive` through `/visual_servo/{start,stop,status}` | `3_perception`'s `camera_perception` and `zedx_nano_depth` (as libraries), a running camera stack; pip: `xarm-python-sdk`, `websockets`, `ultralytics` (torch) |
| `act_inference/` | **Not** a colcon package (`COLCON_IGNORE`): plain Python. Live ACT rollout on the xArm6 from the wrist camera, with a trimmed copy of the ACT/DETR model code. `run_act_node.py` runs it as a ROS 2 node (`act_inference_node`), driven by `6_behavior/manipulation_executive` through `/act_inference/{start,stop,status}` for `place` and `place_base` | `3_perception`'s `zedx_nano_camera` and `zedx_nano_depth` (as libraries); torch, xArm SDK |

Both folders are shared with the `apple_server_manip` repo (`2_manipulation/`), where they are developed and
where the full documentation lives — `2_manipulation/README.md` has the mount model, the measurements behind it
and what has and has not been verified on the arm. This copy differs: it takes the YOLO weights from
`camera_perception`'s package share, as the rest of this workspace does (`yolo_models_dir` defaults to
`$(find-pkg-share camera_perception)/models/yolo`), and `act_inference` has the Autolab-only ROS node
(`node.py`, `runner.py`, `run_act_node.py`) plus a stoppable `run_act_rollout`.

## Where this fits in the stack

```
3_perception (camera hub, /zedx_nano image + depth)
        │
        ▼
visual_servoing  ◄──/visual_servo/{start,stop,status}──  6_behavior/manipulation_executive
   /visual_servo/annotated_image                                   (behavior tree)
        │                                                               │
        │      act_inference_node  ◄──/act_inference/{start,stop,status}─┘
        │             │
        ▼             ▼  xArm SDK, next to xarm_ros2
      xArm6
```

Placing runs the ACT placing policy where it used to wait at the placement pose:

| command | sequence |
|---|---|
| `place` (`target_machine`) | select base ZED X → `plan_april_<id>` → ACT run → `plan_home_offset` |
| `place_base` | `plan_home_offset` → ACT run → `plan_home_offset` |

The executive starts the ACT step with `/act_inference/start/<target>`: `target_machine` (`ot2`, `shaker`) for `place`,
`base` for `place_base`. It succeeds only for the results in the executive's `act_success_results` (`done` by default).

`manipulation_executive` calls `/visual_servo/start`, follows `/visual_servo/status` until that run ends and
counts it as a success only for the results in its `servo_success_results` parameter (`grasped`, `aligned`,
`stopped` by default; see `manipulation_executive/config/manipulation_executive.yaml`). There is no timeout: a BT
deactivation calls `/visual_servo/stop`, and the run fails if `/visual_servo/status` loses its publisher. The executive selects the base camera (`/camera_hub/select_zedx`)
itself before planning; the servo switches to the Nano and back on its own.

The Camera-Edge control WebSocket (`edge_control.py`) is kept for the `apple_server_manip` copy but is off by
default here (`control_port:=-1`); `control_port:=8765` turns it back on.

`manipulation_bringup` (`launch/manipulation.launch.xml`) starts the servo with the rest of autonomy on the real
robot (`autonomy_bringup`, skipped when `sim:=true`; `MANIPULATION_LAUNCH_PACKAGE` / `MANIPULATION_LAUNCH_FILE` in
`.env`) with `execute:=true`: the arm moves when the executive starts a run. `servo_execute:=false` on
`manipulation.launch.xml` makes it a dry run.
Without the node, the executive's servo step fails with "is visual_servo_node running?".

## Build

```bash
cd /home/robot/AutoLab/robot/ros_ws
colcon build --packages-select visual_servoing          # NOT `bws`: --symlink-install rewrites the node's
source install/setup.bash                               # shebang and loses the CUDA torch (see 3_perception)
pip install xarm-python-sdk websockets
pip install ultralytics --no-deps                       # as in 3_perception: plain install shadows the system cv2
```

`act_inference` is not built; it runs from this folder.

## Run the visual servo

The default is a **dry run**: the arm is only read, and the status shows the plate, the target and the velocity
the node would have commanded. The camera stack must be up and streaming the Nano (`3_perception`).

```bash
ros2 launch visual_servoing visual_servo.launch.xml                       # dry run
ros2 launch visual_servoing visual_servo.launch.xml execute:=true grasp:=false   # moves, stops at the hover pose
ros2 launch visual_servoing visual_servo.launch.xml execute:=true         # full grasp
ros2 service call /visual_servo/start std_srvs/srv/Trigger                # a run (or drive it over the WebSocket)
ros2 service call /visual_servo/stop std_srvs/srv/Trigger
```

| name | type | |
|---|---|---|
| `/visual_servo/start`, `/visual_servo/stop` | `std_srvs/Trigger` | start a run (returns at once) / abort it |
| `/visual_servo/status` | `std_msgs/String` | JSON at 2 Hz and at the end of a run (transient local): `phase`, `result`, `message`, `attempt`, `error_mm`, `error_yaw_deg`, `plate_mm`, `target_mm`, `start_tip_mm`, `command_mm_s`, `fsr`, `mode`; during a grasp also `ft`, `ft_lateral`, `ft_phase`, `ft_decision`, `ft_hold_delta_n`, `ft_hold`, `ft_contact` |
| `/visual_servo/annotated_image` | `Image` bgr8 | wrist image with the plate box, the fingertip model and the state; published only while subscribed |
| `ws://0.0.0.0:8765/` | WebSocket | the legacy Camera-Edge control protocol, only with `control_port:=8765` |

Launch arguments: `execute` (false), `grasp` (true), `preview` (false), `robot_ip` (192.168.1.236),
`yolo_models_dir` + `model` (best_wrist.pt), `device` (cuda:0), `control_port` (-1, off), `params_file`
(`config/visual_servo.yaml`, every other parameter with comments), and `with_cameras` (false) with
`depth_models_dir` / `depth_repo` to start the camera stack too.

### F/T sensor

The UFACTORY 6-axis F/T sensor between the flange and the gripper cross-checks the FSR close. It measures the
whole wrist, not each pad: one pad pushing the plate shows up as a sideways force along the finger axis, which a
balanced grip cancels; a held plate raises Fz by its weight after the lift; a fingertip on the tray spikes Fz.

Each grasp switches the sensor on and zeroes it at the hover with the gripper open, next to the FSR tare. It then
logs one `F/T <phase>: F=[..] N T=[..] N m lateral=.. N fsr_pressure=[..] -> <decision>` line at every decision:
`zero`, `grasp height` (or `descent contact`), every `balance` step, `after nudge`, `contact confirmed`,
`before squeeze`, `before lift`, `after lift`. The decision says whether F/T agrees with the FSRs
(`agree`, `disagree: opposite sides`, `disagree: no lateral force`, `disagree: F/T-only imbalance`) and, with
`gripper.use_ft: false`, what it `would` have done. The same values go into `/visual_servo/status`.

Roll it out in two stages:

1. **Watch** (`use_ft: false`, the default). Nothing moves differently. Grasp a few times and read the
   `F/T balance` lines when only one pad presses. They give `ft_finger_axis` (the component that grows) and
   `ft_finger_sign` (make `lateral` positive when the FSRs choose a `+` nudge, FSR1 with `fsr_nudge_sign: [1, -1]`).
   Choose `ft_lateral_deadband_n` above the free-closing noise, `ft_hold_min_n` from the `after lift` `|dFz|`,
   and `ft_contact_n` above the `grasp height` Fz. A failed sensor read only logs a warning here.
2. **Act** (`use_ft: true`). F/T wins where the two disagree about which pad is pushing (or that none is). Once
   one pad touches, the close is **tracked** instead of nudged: every gripper step also slides the arm
   `ft_track_mm_per_step` towards the other pad, more while the touching pad pushes harder than
   `ft_track_force_n`, so it rests on the plate while the other pad comes in (`F/T tracking ...` lines; the
   slide is limited to `ft_track_max_mm`). Without tracking, a plate held 6 mm off-centre made the touching pad
   load and unload at every step until the 5 mm nudge limit failed the run. F/T also stops the final descent when `|Fz| > ft_contact_n` and backs off `ft_contact_backoff`, and the
   `after lift` Fz change decides whether the grasp held. A failed sensor read fails the run.

### Probe contact

`grasp.probe_contact: true` finds the plate top by touch instead of trusting the camera's height (`grasp.depth`
is then unused). After the F/T zero and FSR tare at the hover:

1. close the gripper to `grasp.probe_close` (550) so the closed fingers form a probe;
2. move to `final_segment` above the camera's plate top, then descend at `final_speed` until Fz departs from its
   pre-probe median by more than `gripper.ft_probe_contact_n` for `gripper.ft_contact_count` reads in a row, and halt
   (final_speed down to `probe_slow_margin` above the camera's plate top, then `probe_touch_speed`).
   The plate-top height is the fingertip height at the *first* of those reads (the halt lands slightly deeper);
3. lift `grasp.probe_lift` (5 mm), open the gripper fully, wait `close_settle`, and re-zero the F/T sensor;
4. lower to `grasp.probe_grip_depth` (7.5 mm, tuned on the bench) below the touch and run the usual closing routine.

No touch within `grasp.probe_depth` below the camera's plate top (or `min_tip_z`) fails the attempt:
`max_retries` lifts, opens, re-aligns and probes again. The probe needs the F/T sensor whatever
`gripper.use_ft` says. Status fields: `probe_contact`, `probe_contact_tip_mm`, `probe_expected_top_mm`,
`probe_dfz_n`. The halt latency (`set_state(4)` + 0.1 s) at 8 mm/s pushes the fingers ~1 mm past the first
contact; slow `final_speed` if that shifts the plate.

### Probe tilt

The table leans towards the robot, so with the tool straight down one closed fingertip usually lands first (up to
~5 mm before the other). `grasp.probe_tilt: true` (with `probe_contact`) tilts the tool about the flange axis across
the fingers until both touch, and keeps that tilt for the grasp and the lift:

1. after each touch, settle `probe_tilt_settle` and read the torque change across the finger line,
   `t = gripper.ft_tilt_torque_sign * dT[gripper.ft_tilt_torque_axis]`. `|t| < gripper.ft_tilt_deadband_nm`: both
   fingertips touch, done. Otherwise its sign names the fingertip that touched alone;
2. lift to `probe_tilt_lift` above that touch, tilt `probe_tilt_step_deg` about the fingertip centre to lower the
   other fingertip, re-read the in-air F/T baseline (tilting puts the gripper's weight onto the torque, about as much
   as the contact) and touch again. Steps of `probe_tilt_step_deg` until the torque changes sign, then bisection of
   the two tilts either side; once they are `probe_tilt_min_step_deg` apart the seek ends at the tilt where
   torque / |dFz| crosses zero between them. Each touch goes at `final_speed` to `probe_slow_margin` above the expected
   contact, then at `probe_touch_speed`, and stops on `gripper.ft_probe_contact_n`;
3. the plate top is the fingertip-centre height of the last touch. Lift straight up `probe_lift`, turn to the grasp
   tilt, open, re-zero, lower `probe_grip_depth` below the touch and close as usual. FSR nudges and F/T tracking
   slide along the tilted finger axis.

More than `probe_tilt_max_deg`, `probe_tilt_max_touches` touches or a touch that finds nothing: the level grasp from
the first touch (as without probe_tilt). A retry re-levels the tool before aligning again. Only the tilt across the
fingers is measured; a lean along the pads stays. Status fields: `probe_tilt_result` (`balanced`, `converged`,
`cap`, `max_touches`, `lost`), `probe_tilt_deg`, `probe_tilt_torque_nm`, `probe_tilt_touches`,
`probe_first_contact_tip_mm`. Calibrate `ft_tilt_torque_sign` and the deadband first with
`tools/probe_tilt_check.py` (`watch`, then `seek`), which runs the same seek interactively.

### Safety

- `execute:=false` is the default; the dry-run arm object refuses every motion call.
- Velocity commands carry a 0.25 s deadman, so a stalled loop stops the arm. The fingertips move at most
  20 mm/s and 0.3 rad/s; the descent slows to 8 mm/s near the plate.
- A target more than 250 mm from where the run started, a plate whose measured length is off by more than 35 %,
  or 5 s without a detection fails the run. Without a fresh detection for 0.5 s the arm holds still.
- Nothing else may command the arm during a run — `xarm_ros2` trajectories, ACT, teleop. Every run ends in
  mode 1, state 0, so `xarm_ros2` can take over again.
- **Offsets since the F/T sensor (2026-10-26):** the sensor adds 55 mm between the flange and the gripper. The
  camera and the fingertips both hang below it, so the camera-to-fingertip model, and with it the servo and the
  grasp, is unchanged. But `mount.tip_mm` / `camera_offset_mm` were measured from the EE point before the sensor
  (2026-09-22) with `mount.ee_below_flange_mm: 0.0`; it should now be 55. Until it is, absolute heights are 55 mm
  off: `servo.min_tip_z` sits 55 mm lower than it reads, and `plate_mm`, `target_mm`, `start_tip_mm` in the status
  read 55 mm high.
- **Before the first grasp on this workspace**, check the fingertip height with a dry run and then a hover-only
  run (`execute:=true grasp:=false`), as `2_manipulation/README.md` describes. The mount model was measured on
  the arm on 2026-09-22; the grasp itself has only been run in simulation and against recorded episodes.

## Run ACT inference

```bash
cd src/autonomy/7_manipulation/act_inference
python run_act_inference.py --ckpt-dir models/xarm_final_ckpt_placing              # prediction only
python run_act_inference.py --ckpt-dir models/xarm_final_ckpt_placing --execute    # moves the arm
python run_act_inference.py --ckpt-dir models/xarm_final_ckpt_placing --camera-source ros --ros-namespace /zedx_nano
```

`models/xarm_final_ckpt_placing/` is the well-plate placing policy (RGB + depth, wrist camera), copied from
`apple_server_manip/data/act_ckpts/xarm_final_ckpt_placing` (`config.pkl`, `dataset_stats.pkl`, `policy_best.ckpt`).

`--camera-source nano-stream` (default) talks to the camera hub directly and needs no ROS; `--camera-source ros`
reads the camera stack's topics and needs a sourced workspace. `--help` lists the rest (rate, speed limits,
workspace radius, depth backend and model).

## Run the ACT node

`manipulation.launch.xml` starts it next to the servo (`with_act`, default true). Launch arguments: `act_execute`
(true: the arm moves; false: prediction only, the run still ends `done`), `act_duration_sec` (10.0), `act_dir`
(this folder inside the container, `/home/robot/AutoLab/robot/ros_ws/src/autonomy/7_manipulation/act_inference`).
By hand:

```bash
cd src/autonomy/7_manipulation/act_inference
python run_act_node.py --ros-args -p execute:=false                  # prediction only
ros2 service call /act_inference/start std_srvs/srv/Trigger
ros2 topic echo /act_inference/status
```

| name | type | |
|---|---|---|
| `/act_inference/start`, `/act_inference/stop` | `std_srvs/Trigger` | start a run (returns at once; `already running` while one is) / end it, the arm stops where it is |
| `/act_inference/status` | `std_msgs/String` | JSON at 2 Hz and at the end of a run (transient local): `running`, `result` (`done` \| `stopped` \| `failed`), `message`, `phase` (`idle` \| `preparing` \| `running`), `started` |

Parameters: `ckpt_dir` (`models/xarm_final_ckpt_placing`), `execute` (true), `duration_sec` (10.0),
`enable_rotation` (false), `camera_source` (`ros`), `ros_namespace` (`/zedx_nano`), `robot_ip` (192.168.1.236),
`select_service` (`/camera_hub/select_zedx_nano`, `''` skips the switch), `select_timeout` (20.0). The checkpoint
is loaded and warmed up once at startup; each run switches the hub to the Nano, connects to the arm, runs
`duration_sec` and closes the arm.

## Tests

No hardware: a synthetic camera, a stub robot and a stand-in for `ros2 launch`.

```bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep test traffic off the robot network
python -m pytest visual_servoing/test act_inference/tests -q
```

They need `rclpy`, `cv_bridge`, numpy, OpenCV, torch and the `3_perception` sources, which the tests find
either next to this folder or in the sibling group folder — no build required.

## Not done here yet

- These packages were written and tested on **Jazzy**. The source has no version-specific syntax and the tests
  pass on this machine; on Humble only the control WebSocket needed a fix so far (this container's
  `websockets` ≥ 14 wants `serve()` called inside the event loop), and nothing beyond node startup has
  been exercised here yet.
- The first on-arm grasp (fingertip check above).

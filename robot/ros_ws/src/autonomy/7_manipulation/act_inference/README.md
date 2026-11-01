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

With the ROS workspace sourced and dependencies installed in the Python environment:

```bash
cd src/autonomy/7_manipulation/act_inference
python3 run_act_node.py --ros-args -p execute:=false  # prediction only
# python3 run_act_node.py                          # executes after /start
ros2 service call /act_inference/start std_srvs/srv/Trigger
ros2 topic echo /act_inference/status
ros2 service call /act_inference/stop std_srvs/srv/Trigger
```

These are the same commands and interface as `../README.md`. Start returns immediately;
another start while running returns `already running`. Status is JSON at 2 Hz and on
completion, transient local: `running`, `result` (`done`, `stopped`, `failed`), `message`,
`phase` (`idle`, `preparing`, `running`), and `started` (monotonic time).
`done` means the policy settled or the configured duration elapsed; it is not an
independent placement measurement. Contact detection or cancellation returns `stopped`;
sensor, inference, camera, workspace, and robot errors return `failed`.

The node defaults now match the F/T placing script: the bundled F/T checkpoint,
60 s maximum duration, 50 Hz control, translation 0.01 m/s, rotation enabled at
0.15 rad/s, workspace radius 0.15 m, and no final offset. It preloads and warms the
policy once, then creates a camera/robot session for each run. It selects
`/camera_hub/select_zedx_nano` and reads `/zedx_nano` topics by default, as before.
`execute:=false` makes no robot or sensor configuration writes; F/T sampling must
already be on in xArm Studio. Execution enables sampling if needed and restores it
on exit; it never zeroes the sensor or selects force-control mode.

The existing `manipulation_bringup` launch file explicitly passes a **10 s** duration.
Override it to reproduce the script's 60 s limit:

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

## Standalone script

```bash
./place_on_opentrons_ft.sh                          # prediction only, direct Nano stream
./place_on_opentrons_ft.sh --execute                # commands the arm
./place_on_opentrons_ft.sh --camera-source ros      # use running camera/depth nodes
python3 run_act_inference.py --help
```

The wrapper uses the current `python3` environment, sets `OMP_NUM_THREADS=2`, and accepts
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

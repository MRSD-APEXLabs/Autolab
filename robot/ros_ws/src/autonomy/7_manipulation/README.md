# 7_manipulation: visual servo grasp and live ACT inference

| folder | what it is | needs |
|---|---|---|
| `visual_servoing/` | ROS 2 package (`ament_python`). Wrist-camera (ZED X Nano) visual servo grasp of a well plate on the xArm6: YOLO oriented boxes located with the stereo depth, a position-based servo to a hover pose above the plate, then the descend / FSR close / lift sequence of the Xavier's Camera-Edge. Serves the Camera-Edge control WebSocket that `6_behavior/manipulation_executive` drives | `3_perception`'s `camera_perception` and `zedx_nano_depth` (as libraries), a running camera stack; pip: `xarm-python-sdk`, `websockets`, `ultralytics` (torch) |
| `act_inference/` | **Not** a ROS package (`COLCON_IGNORE`): plain Python. Live ACT rollout on the xArm6 from the wrist camera, with a trimmed copy of the ACT/DETR model code | `3_perception`'s `zedx_nano_camera` and `zedx_nano_depth` (as libraries); torch, xArm SDK |

Both folders are shared with the `apple_server_manip` repo (`2_manipulation/`), where they are developed and
where the full documentation lives — `2_manipulation/README.md` has the mount model, the measurements behind it
and what has and has not been verified on the arm. The copies are identical except that this one
takes the YOLO weights from `camera_perception`'s package share, as the rest of this workspace does:
`yolo_models_dir` defaults to `$(find-pkg-share camera_perception)/models/yolo`. Port changes both ways.

## Where this fits in the stack

```
3_perception (camera hub, /zedx_nano image + depth)
        │
        ▼
visual_servoing  ──ws://<host>:8765──►  6_behavior/manipulation_executive
   /visual_servo/{start,stop,status,annotated_image}        (behavior tree)
        │
        ▼  xArm SDK (mode 5 velocity), next to xarm_ros2
      xArm6
```

`manipulation_executive` sends `{"cmd": "mode", "mode": "inspect"|"servo"}` and polls `{"cmd": "status"}` for
`{"mode": ..., "worker_alive": ...}`; `visual_servoing/edge_control.py` answers exactly that, so the executive
needs no change — **except its host**: `manipulation_executive.launch.xml` still has
`camera_edge_host = 192.168.1.101`, the Xavier's old Camera-Edge. Point it at whichever machine runs
`visual_servo_node` (the one with the GPU and the camera stack). The port stays 8765.

Nothing here is in `autonomy_bringup`, on purpose: this node can move the arm, so it is started by hand (or by
the behavior stack through the WebSocket) rather than with the rest of autonomy.

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
| `/visual_servo/status` | `std_msgs/String` | JSON at 2 Hz and at the end of a run (transient local): `phase`, `result`, `message`, `attempt`, `error_mm`, `error_yaw_deg`, `plate_mm`, `target_mm`, `start_tip_mm`, `command_mm_s`, `fsr`, `mode` |
| `/visual_servo/annotated_image` | `Image` bgr8 | wrist image with the plate box, the fingertip model and the state; published only while subscribed |
| `ws://0.0.0.0:8765/` | WebSocket | the Camera-Edge control protocol (`control_port:=-1` turns it off) |

Launch arguments: `execute` (false), `grasp` (true), `preview` (false), `robot_ip` (192.168.1.236),
`yolo_models_dir` + `model` (best_wrist.pt), `device` (cuda:0), `control_port` (8765), `params_file`
(`config/visual_servo.yaml`, every other parameter with comments), and `with_cameras` (false) with
`depth_models_dir` / `depth_repo` to start the camera stack too.

### Safety

- `execute:=false` is the default; the dry-run arm object refuses every motion call.
- Velocity commands carry a 0.25 s deadman, so a stalled loop stops the arm. The fingertips move at most
  20 mm/s and 0.3 rad/s; the descent slows to 8 mm/s near the plate.
- A target more than 250 mm from where the run started, a plate whose measured length is off by more than 35 %,
  or 5 s without a detection fails the run. Without a fresh detection for 0.5 s the arm holds still.
- Nothing else may command the arm during a run — `xarm_ros2` trajectories, ACT, teleop. Every run ends in
  mode 1, state 0, so `xarm_ros2` can take over again.
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

## Tests

No hardware: a synthetic camera, a stub robot and a stand-in for `ros2 launch`.

```bash
export ROS_DOMAIN_ID=73 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # keep test traffic off the robot network
python -m pytest visual_servoing/test act_inference/tests -q      # 85 tests
```

They need `rclpy`, `cv_bridge`, numpy, OpenCV, torch and the `3_perception` sources, which the tests find
either next to this folder or in the sibling group folder — no build required.

## Not done here yet

- These packages were written and tested on **Jazzy**. The source has no version-specific syntax and the tests
  pass on this machine; on Humble only the control WebSocket needed a fix so far (this container's
  `websockets` ≥ 14 wants `serve()` called inside the event loop), and nothing beyond node startup has
  been exercised here yet.
- The first on-arm grasp (fingertip check above).
- `manipulation_executive`'s `camera_edge_host` still points at the Xavier.

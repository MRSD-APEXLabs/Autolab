# Camera-Edge (the Xavier)

**Purpose:** the second Jetson that owns the cameras and does all the vision — YOLO well-plate detection, AprilTag detection, 3D lifting, and (in servo mode) the visual-servoed grasp itself. Its code lives **only on that machine**; this repo contains clients for it. Everything below was read from the Xavier on 2026-09-01.

| | |
|---|---|
| Hostname / addresses | `xavier.autolab` — **192.168.1.101** (wired, used by the repo) and 192.168.10.7 (WiFi) |
| Login | `ssh autolab@192.168.1.101` (password) |
| Code | `/home/autolab/Autolab-Camera-Edge/` — `main.py`, `config.py`, `cam_worker.py`, `image_worker.py`, `inspect_cam.py`, `visual_servo.py`, `streamhub.py`, `utils.py`, `example_client/test.html`, `models/`, `README.md`, `tests/` |
| Cameras | two ZED X: **cam1 = wrist** (`SERIAL_CAM1`, servo mode, no depth) and **cam2 = base/top** (`SERIAL_CAM2`, inspect mode, NEURAL depth 0.2–1.7 m) |
| Models | `models/best_top.pt` (inspect, top camera) and `models/best_wrist.pt` (servo, wrist camera) — YOLO OBB, run on CUDA |
| Arm | connects **directly** to the xArm (`ARM_IP = 192.168.1.236`, xArm SDK) — only when `DEBUG = False` |

## Starting it

```bash
ssh autolab@192.168.1.101
tmux new -s edge
cd ~/Autolab-Camera-Edge && python3 main.py      # SYSTEM python; the .venv there lacks websockets
```

Expected log: `Control server: ws://0.0.0.0:8765`, `Stream server: … 8766`, `Raw stream server: … 8767`. Project modules log at DEBUG (`main.py:363`), so per-frame `fps=… pts=… | grab=… yolo=… pose+apriltag=…` lines scroll in inspect mode.

## The dashboard (`example_client/test.html`)

A single static page — "Camera Edge Dashboard": host box, green **Connect** / red **Disconnect**, mode buttons **Servo** · **Inspect** · **Idle** · **Stop** · **Status**, three tiles (Active :8766, Wrist :8767, Base :8767), overlay chips `wellplates N · apriltags N · pts N · FPS`, and a log panel. Serve it from the Xavier and open it from any browser:

```bash
cd ~/Autolab-Camera-Edge/example_client && python3 -m http.server 8080
# → http://192.168.1.101:8080/test.html  → host = 192.168.1.101 → Connect
```

It does everything the React UI's camera panel does, with counts, and needs no build.

!!! warning "It is the only interface with mode buttons"
    The GCS UI at `http://gcs.autolab:3000` shows the same three camera tiles but opens only 8766/8767 — it never touches the control port 8765, so it **cannot switch inspect/servo/idle** ([#36](../known-issues.md)). This dashboard is served only while someone is running that `http.server 8080` on the Xavier; it was **not** running on 2026-09-12.

From the robot Jetson you can switch modes with no browser and no SSH, using the only interpreter there that has `websockets`:

```bash
/usr/bin/python3 ~/coding/percep/test_server.py ws://192.168.1.101:8765
# prompt: servo | inspect | idle | status | stop
```

A healthy `status` reply looks like
`{"ok": true, "mode": "inspect", "worker_alive": true, "wrist_camera_connected": true, "base_camera_connected": true}`
(verified 2026-09-12).

## Modes and protocol

Three WebSocket servers (from its README):

| Port | Role | Messages |
|---|---|---|
| 8765 | control | `{"cmd":"mode","mode":"servo"\|"inspect"\|"idle"}`, `{"cmd":"stop"}`, `{"cmd":"status"}` → `{"ok":true,"mode":…,"worker_alive":bool,"wrist_camera_connected":…,"base_camera_connected":…}`. Greets with `{"ok":true,"message":"connected"}` |
| 8766 | active feed | `inspect_frame` / `servo_frame` / `idle` JSON (see below) |
| 8767 | raw feed | send `{"camera":"wrist"|"base"}` → stream of `{"type":"raw_frame","camera":…,"image_jpeg_b64":…}` |

An **idle worker** (`ZEDImageWorker`) always streams both raw cameras on 8767. The **active worker** is replaced on every mode switch:

### Inspect mode (`inspect_cam.py`, top camera)

Per frame: grab → `self.model(frame)` (YOLO OBB, `best_top.pt`) → keep detections with **conf ≥ 0.8** → draw circle + label into the frame → lift each box to 3D with the ZED depth → `detect_apriltags()` (`apriltag` library, family `tag36h11`, tag side **6.25 cm**, pose from the tag plane) → subsample the point cloud → publish:

```json
{"type":"inspect_frame","image_jpeg_b64":"…",
 "wellplates":[{"pose":{"position":[x,y,z],"orientation":[qx,qy,qz,qw]},"metric_size":[w,h],"name":…,"conf":…,"theta":…,"center":[u,v],"size":[pw,ph]}],
 "apriltags":[{"id":2,"pose":{…},"center":[u,v],"corners":[…]}],
 "pointcloud_zlib_b64":"…","pointcloud_shape":[N,4],"pointcloud_dtype":"float32","pc_count":N,"fps":7.0,"frame_id":…,"timestamp":…}
```

Poses are in the **camera frame**; on the robot side `pc3.py` stamps them `top_camera`, and the static TF `Chassis_1 → top_camera` published by `planning.launch.xml` (via the modified vendored xarm launch) puts them in `world`.

### Servo mode (`visual_servo.py`, wrist camera) — the grasp

1. `move_to_start_pose()`: takes the arm over the xArm SDK, `gripper_open`, tool pointing straight down.
2. Cartesian-velocity visual servoing: YOLO OBB on the wrist camera, project the gripper center into the image (`target_uv`, the cyan cross), drive `vx, vy, vz` to center the plate while descending; streams `servo_frame {detections[], target_uv, fps…}`.
3. When the plate's box area exceeds `AREA_STOP_THRESHOLD = 50 000 px²`: `execute_threshold_motion()` — estimate remaining height from the box (pinhole with the 127.76 × 85.48 mm plate) minus `CAMERA_TO_GRASP_OFFSET_MM = 115`, align yaw to the plate's short side, lateral move, slow descent.
4. Tactile close: `gripper_set(50)` then step the custom Modbus gripper closed (step 20, max 350) while reading its two **FSR** finger sensors; nudge sideways if only one finger touches; stop when both exceed the stop pressure; lift.
5. Success = both FSRs loaded; otherwise lift 50 mm and retry (`GRASP_MAX_RETRIES = 3`). When the worker finishes it goes idle — that `worker_alive: false` is what `manipulation_executive` waits for.

**Nothing on the ROS side touches the gripper; nothing anywhere opens it at a machine** — the only `gripper_open` is at the start of the next servo run.

## `config.py` knobs that matter

| Key | Value seen | Meaning |
|---|---|---|
| `DEBUG` | **`True`** | Skip all robot motion and print commands instead. With `True`, servo mode moves nothing and still "succeeds". |
| `ARM_IP` | `192.168.1.236` | |
| `MODEL_INSPECT` / `MODEL_SERVO` | `models/best_top.pt` / `models/best_wrist.pt` | relative → run `main.py` from the project directory |
| `AREA_STOP_THRESHOLD` | `50000.0` | px² that triggers the grasp sequence |
| `GRASP_MAX_RETRIES` / `GRASP_RETRY_LIFT_MM` | `3` / `50` | |
| `GRASP_DESCENT_MM` / `CAMERA_TO_GRASP_OFFSET_MM` | `70` / `115` | fallback descent; camera-to-fingertip offset |
| `GRIPPER_CLOSE` / `GRIPPER_STEP` | `50` / `20` | Modbus gripper positions |
| `POINTCLOUD_Z_BOUNDS`, `POINTCLOUD_STRIDE`, `STREAM_HZ` | | cloud filtering / stream rate |

## Known problems (details in [Known Issues](../known-issues.md))

- **`DEBUG = True` also crashes servo mode**: `visual_servo.py:585-587` and `788-789` call `self.arm.connect()` / `set_mode(1)` outside the `if not self.debug` guard, and in DEBUG mode `self.arm` is `None` → the worker dies at start and the mode snaps back to `idle` (no wrist stream). Patch: wrap those lines in `if not self.debug:` (#32).
- Without the gripper connected, every Modbus call fails softly (`Error … clearing errors…`, FSR reads `None`) → grasp always "fails" after 4 attempts, and the executive still reports SUCCESS (#34).
- The servo controller and MoveIt's driver both talk to the arm; the executive serializes them, but never run servo while a MoveIt trajectory executes.

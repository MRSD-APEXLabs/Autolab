# 7_imitation_learning: ACT data collection + training for placing a wellplate

Not a ROS package (`COLCON_IGNORE` is present): plain Python with its own venv, so colcon skips it.

```
collect.sh            collect demonstrations (keyboard teleop, keep/delete each one)
inspect.sh            check an episode / list a dataset
view.sh               web viewer: both cameras + depth + joint angles for saved episodes (collect.sh starts it too)
configs/collect.yaml  IPs, stream size, speeds, workspace box, start pose
actlib/               collect.py keyboard_teleop.py arm_io.py cam_client.py episode_io.py inspect_episode.py
xavier/               record_worker.py + PATCH.md/PATCH.diff: what was added to the Xavier camera server
data/  runs/  .venv/  git-ignored: datasets, training runs, virtualenv
```

## Collect (arm side)

Before the first run:
1. **Check `workspace_mm` in `configs/collect.yaml`** for your deck. z min is the table clearance (now 70 mm): keep the fingertips above it.
2. The Xavier camera server must be the patched one (it has record mode; see `xavier/PATCH.md`).
3. Hand on the e-stop. Keys are read **globally** (any window) while jogging/recording.

**Planning stack:** it can keep running. The xArm ROS driver (`ros2 launch planning_bringup ...`) pauses its own controllers while the arm
is in this script's velocity mode and reactivates them when the arm is back in servo mode 1. This script owns the arm (and puts the
Xavier in record mode) **only while you jog or record**; after every jog and every episode it hands the arm back (mode 1 when the ROS
driver is running, else the mode it found) and returns the Xavier to its normal mode, so planning and perception work between episodes.
Do not send planning commands during a jog or recording: one arm, one commander. Only `servo.py` (leader teleop) and
`joy_cartesian_jog.py` block startup.

```bash
./collect.sh --dataset wellplate_place --target 40     # resumable: counts episodes already on disk
```

Per episode: `Enter` starts recording -> drive with the keys -> `Enter` finishes -> `k` keep / `d` delete.
A deleted episode never touches the disk. `j` at the first prompt jogs the arm without recording; `g` goes to `start_pose` (if set).

| key | action |
|---|---|
| W / S | +X / -X (base frame) |
| A / D | +Y / -Y |
| R / F | +Z / -Z |
| Q / E | rotate the last joint +/- (angular velocity about base +Z; tool stays vertical) |
| 1 / 2 / 3 | speed preset: 15 / 40 / 100 mm/s and 8 / 20 / 45 deg/s (starts on 2; edit `teleop.presets`) |
| Enter | finish (ramps to zero first) |
| Esc | stop immediately and discard the episode |

Hold to move, release to stop. Velocity is slewed (250 mm/s^2, 150 deg/s^2), the command loop runs at 100 Hz, and the TCP is kept
inside the workspace box. If the controller faults (e.g. near a singularity) it is re-armed automatically and the event is stored.

## What is recorded

| stream | rate | notes |
|---|---|---|
| wrist camera left + right | ~29 Hz | JPEG, 640x400 (default), stamped at capture on the Xavier. **Duplicate frames are removed at save time** (below) |
| depth (aligned to left) | ~29 Hz | PNG16, uint16 mm, 0 = invalid, lossless |
| joints, TCP pose, joint torque | ~99 Hz | controller real-time report (port 30003) |
| commanded velocity + raw key intent | 100 Hz | |
| gripper setpoint + 2 finger pressure sensors | ~5 Hz | state only |
| aligned/ | at each camera frame | robot state interpolated to the frame's capture time: what training reads |

```
data/<dataset>/dataset.yaml   conventions, units, camera calibration
data/<dataset>/index.csv      table of the episode files, REBUILT from them (never edit it: to delete an episode just delete its .hdf5)
data/<dataset>/episodes/episode_0001.hdf5 ...   (layout in actlib/episode_io.py)
data/<dataset>/sessions/*.log console log of every session, including discarded attempts
```
**Duplicate frames:** a camera frame is dropped when its joint angles are exactly the same as the previous frame's. Joint angles only:
no tolerance, and neither the picture nor the timestamps are compared. The first frame of each still stretch is kept. A waiting arm that
is exactly still reports bit-identical angles (in a first real episode 67% of consecutive frame pairs were), so those frames go; frames
where the angles differ even in the 4th decimal (sensor jitter, slow creep) are different frames and stay. Frames where only the scene or
the gripper changes while the arm is still are dropped too. The waiting at the END of a recording (after the last kept frame) is also cut
from the saved robot / control / gripper logs, so the saved episode ends where its frames end. The 100 Hz robot log is otherwise never
thinned. Kept frames are therefore not evenly spaced: `frames/t` has each one's true capture time and the
`stat_*` attributes / `index.csv` still describe the health of the full recorded stream. Disable in `recording.dedupe`.

Clock handling: the Xavier and this machine are NTP-synced yet differ by tens of ms and drift ~1 ms/min, so the offset is measured
(20 round trips, best RTT) before and after every episode and stored; `frames/t` is already corrected (verified to <1 ms).

**Deleting an episode:** delete its file `episodes/episode_000N.hdf5` and nothing else. `index.csv` is rebuilt from the files at every
start and save and re-checked every 2 s while `collect.sh` runs (`./inspect.sh <dataset>` always reads the files directly), so the row
disappears by itself and a re-recorded number never appears twice. Deleting in VS Code sends the file to the Trash (still on disk);
`rm` deletes it for good.

Watch saved episodes in a browser (left + right camera + colourised depth + joint angles / TCP / commanded velocity / gripper, all on
one timeline; play, step, scrub, click a plot to seek, 0.25x-2x):
```bash
./collect.sh ...        # starts the viewer automatically and prints http://localhost:8090/?episode=N after every save
./view.sh               # or run it on its own for any dataset: ./view.sh data/wellplate_place --port 8090
```
Keys in the page: space play/pause, left/right step one frame, shift+left/right +-1 s, [ ] previous/next episode.
It is read-only and runs as its own process (it cannot slow the control loop). `--host 0.0.0.0` opens it to other machines.

Inspect from the terminal:
```bash
./inspect.sh data/wellplate_place                       # list + totals
./inspect.sh data/wellplate_place/episodes/episode_0001.hdf5 --montage m.png --plot p.png --video v.mp4
```

## Known limits
* **The Xavier's Ethernet negotiated 100 Mb/s** (the Thor side is 1 Gb/s): probably a cable or switch port. That caps the wire at
  ~11.8 MB/s, hence 640x400. With a gigabit link set `xavier.scale: 1.0`, `jpeg_quality: 95` in `configs/collect.yaml`.
* Depth is NEURAL from a stereo pair: invalid on ~11% of pixels (very close / reflective / edges).
* `duration=0` velocity commands have no firmware watchdog: a `kill -9` of the collector while moving leaves the arm moving.
  Ctrl-C / Esc / exceptions all zero the velocity. Keep the e-stop in reach.

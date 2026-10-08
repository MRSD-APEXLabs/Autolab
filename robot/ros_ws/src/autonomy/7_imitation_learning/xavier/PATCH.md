# Xavier changes for record mode (2026-09-18)

Machine: the AGX Xavier camera box (`autolab@192.168.1.101`), project `~/Autolab-Camera-Edge/`.
The full edit is in [PATCH.diff](PATCH.diff) (applied to the files exactly as they were that day); the new file is
[record_worker.py](record_worker.py) (copy it to the Xavier next to `main.py`).

## What it adds
* **Record mode** (`{"cmd":"mode","mode":"record"}` on :8765): the wrist ZED X is grabbed with depth on and streamed as
  binary frames on **:8768**: `[u32 LE header_len][JSON header][left JPEG][right JPEG][depth PNG16, uint16 mm, 0 = invalid]`.
  Frames are stamped at capture. Images/depth are downscaled on the Xavier (`RECORD_SCALE`, default 0.6667 = 640x400)
  because the Xavier's Ethernet link negotiated only 100 Mb/s (`cat /sys/class/net/eth0/speed` -> 100).
* Control port commands: `time` (Xavier clock, for the clock probe), `record_config` (`scale`, `jpeg_quality`, applied per session
  by `collect.py`), and `status` now reports `record_available`.
* `config.py`: `RECORD_PORT`, `RECORD_SCALE`, `RECORD_JPEG_QUALITY`, `WRIST_DEPTH_MODE` (`NEURAL`), `WRIST_DEPTH_MIN_MM/MAX_MM` (-1 = SDK default).

## Behaviour changes to know about
* The wrist camera is now **opened with NEURAL depth**. Depth is only computed while record mode runs: `image_worker.py` and
  `visual_servo.py` set `enable_depth = False` on their grabs, so idle / servo behave as before.
* While record mode is active the idle worker does **not** grab the wrist camera (otherwise the two threads split the frames);
  the GCS `wrist_raw` tile is fed from the record worker at ~5 Hz. Inspect / servo modes are off while recording.
  `collect.py` restores the previous mode when it exits.

## How it is running now
Started by hand in tmux (not by systemd): `tmux attach -t camera_edge` (log: `~/camera_edge_20260918.log`).
The original service `autolab-camera-edge.service` was stopped with SIGINT (clean exit, systemd did not respawn it).

## Hand back to systemd (needs the Xavier sudo password)
```bash
tmux kill-session -t camera_edge        # or Ctrl-C in the pane
sudo systemctl start autolab-camera-edge
```
The service runs the same files, so it will come up with record mode included.

## Roll back
Originals were saved next to the files as `*.bak-20260918` (`main.py config.py image_worker.py visual_servo.py`):
```bash
cd ~/Autolab-Camera-Edge && for f in main.py config.py image_worker.py visual_servo.py; do cp -p $f.bak-20260918 $f; done
rm record_worker.py     # then restart as above
```

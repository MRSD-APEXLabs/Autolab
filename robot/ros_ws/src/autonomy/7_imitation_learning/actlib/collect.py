"""ACT demonstration collector: prompt -> keyboard teleop while recording -> keep / delete.

    ./collect.sh --dataset wellplate_place --target 40

Per episode: [Enter] starts recording (clock probe, then camera + 100 Hz robot state + control log), you drive the arm
with the keyboard (see keyboard_teleop.py), Enter stops it, then you are asked to keep or delete the episode. A deleted
episode never touches the disk. Everything is written under <project>/data/<dataset>/ (see episode_io.py).
"""

import argparse
import datetime
import logging
import os
import signal
import socket
import subprocess
import sys
import termios
import time
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import yaml

from . import episode_io as eio
from .keyboard_teleop import KEYS_DOC, KeyState, Teleop, quiet_terminal

PROJECT = Path(__file__).resolve().parents[1]
log = logging.getLogger("act.collect")

# --------------------------------------------------------------------------- small helpers
def say(msg: str = ""):
    print(msg, flush=True)
    log.info(msg)


def flush_stdin():
    if sys.stdin.isatty():
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)


def ask(prompt: str, valid: Sequence[str], default: Optional[str] = None) -> str:
    flush_stdin()
    while True:
        ans = input(prompt).strip().lower()
        if ans == "" and default is not None:
            return default
        if ans in valid:
            return ans
        print(f"  please answer one of: {', '.join(v or 'Enter' for v in valid)}")


def load_config(path: Path) -> dict:
    cfg = yaml.safe_load(Path(path).read_text())
    ws = cfg["workspace_mm"]
    for axis in "xyz":
        lo, hi = ws[axis]
        if not lo < hi:
            raise SystemExit(f"config: workspace_mm.{axis} must be [min, max] with min < max, got {ws[axis]}")
    t = cfg["teleop"]
    if not t["presets"] or not 1 <= t["start_preset"] <= len(t["presets"]):
        raise SystemExit("config: teleop.presets / start_preset invalid")
    sp = cfg.get("start_pose")
    if sp is not None:
        if len(sp) != 6:
            raise SystemExit("config: start_pose must be [x, y, z, roll, pitch, yaw] or null")
        for axis, v in zip("xyz", sp[:3]):
            if not ws[axis][0] <= v <= ws[axis][1]:
                raise SystemExit(f"config: start_pose {axis}={v} is outside workspace_mm.{axis} {ws[axis]}")
    return cfg


# Other teleops that actively drive the arm: they would fight this script, so it refuses to start next to them.
BLOCKING_SCRIPTS = {"servo.py", "joy_cartesian_jog.py"}
SHELLS = {"bash", "sh", "dash", "zsh", "fish"}


def _running_argvs():
    """argv of every process, read from /proc. Matching is done on the actual arguments, never on a substring of the
    whole command line: a shell whose text merely mentions 'servo.py' is not a process running it."""
    me = {os.getpid(), os.getppid()}
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as fh:
                argv = [a.decode(errors="replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        if argv and os.path.basename(argv[0]) not in SHELLS:
            yield int(d), argv


def arm_conflicts() -> List[str]:
    """Processes running one of the other arm teleops (leader-arm servo.py, joystick joy_cartesian_jog.py)."""
    return [f"{pid} {' '.join(argv)[:140]}" for pid, argv in _running_argvs()
            if any(os.path.basename(a) in BLOCKING_SCRIPTS for a in argv[:4])]


def ros_driver_running() -> bool:
    """The xArm ROS driver (planning stack) may keep running: it streams only a hold command once per second,
    deactivates its controllers when the arm leaves servo mode 1 and reactivates them when the arm is back in mode 1."""
    return any(os.path.basename(argv[0]) == "ros2_control_node" and any("xarm" in a for a in argv)
               for _, argv in _running_argvs())


# --------------------------------------------------------------------------- rig
class Rig:
    """Live components. Tests build one from fakes; build_rig() builds the real thing."""
    cam_ctl = cam = feed = arm = gripper = keys = teleop = None
    prev_xavier_mode = None
    viewer_url = None
    priority = ()   # cleanup steps other systems depend on, run first and in this order: Xavier mode, then the arm
    closers: list

    def close(self):
        """Must finish even if Ctrl-C is pressed again: a cleanup cut short once left the Xavier in record mode, which
        stops the perception stream (and the point cloud) the planning stack needs."""
        try:
            old = signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:  # not the main thread
            old = None
        try:
            for fn in list(self.priority) + list(reversed(self.closers)):
                try:
                    fn()
                except BaseException as e:
                    log.warning("cleanup step failed: %s", e)
        finally:
            if old is not None:
                signal.signal(signal.SIGINT, old)


def arm_target_mode(rig: Rig) -> int:
    """Mode to hand the arm back in: servo mode 1 while the xArm ROS driver runs (it reactivates its controllers only
    in mode 1, whichever was started first), otherwise the mode the arm was found in."""
    return 1 if ros_driver_running() else rig.arm.initial_mode


def give_back(rig: Rig):
    """Return the arm and the Xavier's normal (perception) mode, so the planning stack works between jogs / episodes."""
    try:
        rig.arm.hand_back(arm_target_mode(rig))
    except Exception as e:
        log.warning("handing the arm back failed: %s", e)
    try:
        rig.cam_ctl.set_mode(rig.prev_xavier_mode)
    except Exception as e:
        log.warning("restoring the Xavier mode failed: %s", e)


def build_rig(cfg: dict) -> Rig:
    from .arm_io import ArmLink, GripperLog, StateFeed
    from .cam_client import FrameStream, XavierControl
    rig = Rig()
    rig.closers = []
    rig.priority = []
    rec, xv = cfg["recording"], cfg["xavier"]
    try:
        say("preflight: checking that nothing else is driving the arm ...")
        found = arm_conflicts()
        if found:
            say("  these processes hold / command the arm and must be stopped first:")
            for ln in found:
                say("    " + ln)
            raise SystemExit("aborting: stop them, then run again (they command the arm and would fight this script)")
        if ros_driver_running():
            say("  note: the xArm ROS driver (planning stack) is running and is left alone. This script owns the arm only while you")
            say("        jog or record; after each jog / episode it hands the arm back (servo mode 1, the driver reactivates by itself)")
            say("        and puts the Xavier back in its normal mode, so planning + perception work between episodes.")
            say("        Do not send planning commands while jogging or recording.")

        say("preflight: Xavier camera server ...")
        rig.cam_ctl = XavierControl(xv["host"], xv["control_port"])
        st = rig.cam_ctl.status()
        if not st.get("record_available"):
            raise SystemExit("aborting: the Xavier server has no record mode (is the patched main.py running?)")
        if st["mode"] == "servo":
            raise SystemExit("aborting: the Xavier is in SERVO mode (it is driving the arm). Stop it first.")
        rig.prev_xavier_mode = "inspect" if st["mode"] == "record" else st["mode"]
        say(f"  applied stream settings: {rig.cam_ctl.configure_record(xv['scale'], xv['jpeg_quality'])}")
        rig.cam_ctl.set_mode("record")
        rig.priority.append(lambda: rig.cam_ctl.set_mode(rig.prev_xavier_mode))
        rig.cam = FrameStream(xv["host"], xv["record_port"])
        rig.cam.start()
        rig.closers.append(rig.cam.stop)
        time.sleep(2.5)
        off, rtt = rig.cam_ctl.measure_offset()
        rig.cam.begin_recording()
        time.sleep(1.5)
        fr = rig.cam.end_recording()
        if len(fr) < 10:
            raise SystemExit("aborting: the camera stream delivers almost no frames")
        lag_ms = float(np.median([f["t_recv"] - (f["t_image_ns"] / 1e9 - off) for f in fr]) * 1e3)
        hz = rig.cam.hz()
        say(f"  camera: {hz:.1f} Hz, capture->here lag {lag_ms:.0f} ms, image {fr[0]['image_size']}, clock offset {off*1e3:+.1f} ms")
        rig.cam_ctl.set_mode(rig.prev_xavier_mode)  # normal perception until a recording actually starts
        if hz < rec["min_cam_hz"] or lag_ms > rec["max_lag_ms"]:
            raise SystemExit(f"aborting: camera stream too slow (need >= {rec['min_cam_hz']} Hz and lag <= {rec['max_lag_ms']} ms). "
                             "The Xavier's Ethernet link may have negotiated 100 Mb/s: lower xavier.scale / jpeg_quality in the config.")

        say("preflight: xArm ...")
        rig.feed = StateFeed(cfg["arm_ip"])
        rig.closers.append(rig.feed.close)
        if not rig.feed.wait_ready(5.0):
            raise SystemExit("aborting: no state packets from the xArm")
        time.sleep(1.0)
        say(f"  robot state feed: {rig.feed.hz():.0f} Hz")
        rig.arm = ArmLink(cfg["arm_ip"])
        rig.arm.set_initial(rig.feed.latest["mode"])
        say(f"  arm found in mode {rig.arm.initial_mode}, state {rig.feed.latest['state']} (it is handed back in mode {rig.arm.initial_mode})")
        rig.priority.append(lambda: rig.arm.release(arm_target_mode(rig)))
        if cfg.get("gripper_log", True):
            rig.gripper = GripperLog(rig.arm.arm)
            rig.gripper.start()
            rig.closers.append(rig.gripper.stop)

        rig.keys = KeyState(cfg["teleop"]["release_debounce_ms"] / 1000.0)
        rig.keys.start_listener()
        rig.closers.append(rig.keys.stop_listener)
        tele_cfg = dict(cfg["teleop"], _workspace=cfg["workspace_mm"])
        rig.teleop = Teleop(tele_cfg, rig.arm, rig.feed, rig.keys)
        return rig
    except BaseException:
        rig.close()
        raise


# --------------------------------------------------------------------------- one episode
def _status_printer(rig: Rig, tag: str):
    def cb(s: dict):
        tcp = s["tcp"]
        pos = f"x={tcp[0]:7.1f} y={tcp[1]:7.1f} z={tcp[2]:7.1f}" if tcp else "tcp n/a"
        cam = f"cam {rig.cam.hz():4.1f} Hz, {rig.cam.recorded_count():4d} fr, lost {rig.cam.gaps}" if tag == "REC" else ""
        line = (f"\r{tag} {s['elapsed']:6.1f} s | {cam} | robot {rig.feed.hz():3.0f} Hz | preset {s['preset']} "
                f"({s['lin']} mm/s, {s['ang']} deg/s) | {pos} mm{' | FAULT' if s['faulted'] else ''}   ")
        sys.stdout.write(line)
        sys.stdout.flush()
    return cb


def jog(rig: Rig):
    """Move the arm with the keys without recording (Enter or Esc to return)."""
    say("JOG (not recording): move the arm to where the episode should start. Enter = done, Esc = stop.")
    rig.arm.prepare()
    rig.keys.activate()
    try:
        with quiet_terminal():
            rig.teleop.run(recording=False, on_status=_status_printer(rig, "JOG"))
    finally:
        rig.keys.deactivate()
        print()
        give_back(rig)


def _wait_for_frames(rig: Rig, n_new: int = 8, timeout: float = 4.0) -> bool:
    n0 = rig.cam.total
    end = time.time() + timeout
    while rig.cam.total - n0 < n_new and time.time() < end:
        time.sleep(0.05)
    return rig.cam.total - n0 >= n_new


def record_episode(rig: Rig, cfg: dict):
    """Returns a dict with everything collected, or None if the operator aborted with Esc."""
    rec = cfg["recording"]
    rig.cam_ctl.set_mode("record")  # the Xavier records only now; between episodes it serves perception as usual
    rig.arm.prepare()
    if not _wait_for_frames(rig):
        give_back(rig)
        say("The camera stream did not start: nothing recorded.")
        return {"empty": True}
    say("recording starts after a 0.3 s clock probe ...")
    t, off, rtt = time.time(), *rig.cam_ctl.measure_offset()
    off_pre = (t, off, rtt)
    rig.teleop.reset_log()
    rig.cam.begin_recording()
    rig.feed.start_recording()
    if rig.gripper:
        rig.gripper.start_recording()
    rig.keys.activate()
    say("REC  -  drive with the keys, Enter = finish, Esc = abort and discard")
    result = "esc"
    try:
        with quiet_terminal():
            result = rig.teleop.run(recording=True, on_status=_status_printer(rig, "REC"))
            if result == "enter":
                time.sleep(rec["tail_s"])  # camera frames arrive ~130 ms late: let the last ones in
    finally:
        rig.keys.deactivate()
        frames = rig.cam.end_recording()
        robot = rig.feed.stop_recording()
        grip = rig.gripper.stop_recording() if rig.gripper else {"t": np.zeros(0), "setpoint": np.zeros(0, np.int32), "fsr": np.zeros((0, 2), np.int32)}
        print()
        give_back(rig)
    control = rig.teleop.control_arrays()
    events = list(rig.teleop.events)
    faults = rig.teleop.faults
    t, off, rtt = time.time(), *rig.cam_ctl.measure_offset()
    off_post = (t, off, rtt)
    if result == "esc":
        return None
    if not len(frames) or len(robot["t"]) < 2:
        return {"empty": True}
    t_all, src = eio.frame_times(frames, off_pre[:2], off_post[:2])
    frames, t_cam = eio.trim_to_span(frames, t_all, robot["t"][0], robot["t"][-1])
    if not len(frames):
        return {"empty": True}
    stats = eio.episode_stats(t_cam, np.array([f["frame_id"] for f in frames]), robot["t"], faults)  # stream health, before dedupe
    stats["gripper_reads"] = int(len(grip["t"]))
    warnings = eio.quality_warnings(stats, rec)
    dd = rec.get("dedupe", {})
    if dd.get("enabled", True):
        keep = eio.dedupe_frames(frames, t_cam, robot["t"], robot["q"], dd["q_deg"], dd["img_mad"])
        frames, t_cam = [frames[i] for i in keep], t_cam[keep]
    stats["n_recorded"], stats["n_frames"] = stats["n_frames"], len(frames)
    return {"frames": frames, "t_cam": t_cam, "ts_source": src, "robot": robot, "control": control, "gripper": grip,
            "events": events, "off_pre": off_pre, "off_post": off_post, "stats": stats, "warnings": warnings}


def print_summary(res: dict):
    s = res["stats"]
    say(f"Stopped: {s['duration_s']:.1f} s | camera {s['n_recorded']} frames @ {s['cam_hz']:.1f} Hz "
        f"(max gap {s['max_frame_gap_ms']:.0f} ms, {s['frame_id_gaps']} lost) -> {s['n_frames']} kept, "
        f"{s['n_recorded'] - s['n_frames']} duplicate frames removed | robot {s['robot_rows']} rows @ {s['robot_hz']:.0f} Hz | "
        f"gripper {s['gripper_reads']} reads | faults {s['faults']}")
    for w in res["warnings"]:
        say(f"  WARNING: {w}")


def save_episode(rig: Rig, cfg: dict, dataset_dir: Path, ep_id: int, task: str, res: dict) -> Path:
    meta = {"created": datetime.datetime.now().isoformat(timespec="seconds"), "dataset": dataset_dir.name, "task": task,
            "git_hash": eio.git_hash(PROJECT), "collect_config": yaml.safe_dump(cfg, sort_keys=False),
            "warnings": "; ".join(res["warnings"])}
    path = eio.write_episode(dataset_dir, ep_id, res["frames"], res["t_cam"], res["ts_source"], res["robot"], res["control"],
                             res["gripper"], res["events"], res["off_pre"], res["off_post"], meta, res["stats"])
    s = res["stats"]
    eio.append_index(dataset_dir, {
        "episode_id": ep_id, "file": path.name, "created": meta["created"], "duration_s": round(s["duration_s"], 2),
        "n_frames": s["n_frames"], "n_recorded": s["n_recorded"], "cam_hz": round(s["cam_hz"], 2), "max_frame_gap_ms": round(s["max_frame_gap_ms"], 1),
        "frame_id_gaps": s["frame_id_gaps"], "robot_rows": s["robot_rows"], "robot_hz": round(s["robot_hz"], 1),
        "faults": s["faults"], "clock_offset_ms": round(res["off_post"][1] * 1e3, 2),
        "size_mb": round(path.stat().st_size / 1e6, 1), "notes": "; ".join(res["warnings"])})
    calib = rig.cam.calibration if rig.cam else None
    eio.ensure_dataset_yaml(dataset_dir, task, calib, cfg["xavier"]["scale"], cfg, KEYS_DOC)
    return path


# --------------------------------------------------------------------------- session
def run_session(rig: Rig, cfg: dict, dataset_dir: Path, target: int, task: str):
    for stale in (dataset_dir / "episodes").glob("*.hdf5.tmp"):
        stale.unlink()  # an interrupted save from an earlier session
    has_start = cfg.get("start_pose") is not None
    while True:
        kept = len(eio.episode_files(dataset_dir))
        if kept >= target:
            say(f"Target reached: {kept} / {target} episodes in {dataset_dir}")
            return
        ep_id = eio.next_episode_id(dataset_dir)
        say(f"\n=== Episode {ep_id:04d}   (kept {kept} / {target}) ===")
        opts = "Enter = start recording / j = jog first (not recorded) / " + ("g = go to start pose / " if has_start else "") + "q = quit"
        valid = ["", "j", "q"] + (["g"] if has_start else [])
        while True:
            choice = ask(f"Start recording episode {ep_id:04d}? [{opts}] ", valid, default="")
            if choice == "q":
                say(f"Quit. {len(eio.episode_files(dataset_dir))} episodes kept.")
                return
            if choice == "j":
                jog(rig)
            elif choice == "g":
                say(f"Moving to start pose {cfg['start_pose']} at {cfg['start_speed_mm_s']} mm/s (keep a hand on the e-stop) ...")
                code = rig.arm.move_to_pose(cfg["start_pose"], cfg["start_speed_mm_s"])
                say(f"  move finished (code {code})")
            else:
                break
        res = record_episode(rig, cfg)
        if res is None:
            say("Aborted with Esc: episode discarded (nothing written).")
            continue
        if res.get("empty"):
            say("Nothing usable was recorded (no camera frames inside the robot log): episode discarded.")
            continue
        print_summary(res)
        if ask("Keep this episode? [k = keep / d = delete] ", ["k", "d"]) == "k":
            path = save_episode(rig, cfg, dataset_dir, ep_id, task, res)
            say(f"saved {path.relative_to(dataset_dir.parent.parent)} ({path.stat().st_size / 1e6:.0f} MB)")
            if rig.viewer_url:
                say(f"  watch it (both cameras, depth, joint angles): {rig.viewer_url}/?episode={ep_id}")
        else:
            say(f"Episode {ep_id:04d} deleted (was {res['stats']['duration_s']:.1f} s; nothing written).")
        del res


def start_viewer(dataset_dir: Path, port: int):
    """Serve the dataset in a browser. A separate process, so it can never slow the 100 Hz control loop. Never raises:
    collecting must not depend on the viewer."""
    try:
        for p in range(port, port + 10):
            with socket.socket() as s:
                s.settimeout(0.3)
                if s.connect_ex(("127.0.0.1", p)) != 0:  # nothing listening there
                    break
        else:
            return None, None
        logf = open(dataset_dir / "sessions" / "viewer.log", "ab")
        proc = subprocess.Popen([sys.executable, "-m", "actlib.viewer", str(dataset_dir), "--port", str(p)],
                                cwd=str(PROJECT), stdout=logf, stderr=subprocess.STDOUT)
        for _ in range(40):
            with socket.socket() as s:
                s.settimeout(0.3)
                if s.connect_ex(("127.0.0.1", p)) == 0:
                    return proc, f"http://localhost:{p}"
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        proc.terminate()
    except Exception as e:
        log.warning("viewer not started: %s", e)
    return None, None


def main(argv=None, rig_factory=None):
    ap = argparse.ArgumentParser(description="Collect keyboard-teleop demonstrations for ACT training.")
    ap.add_argument("--dataset", default="wellplate_place", help="dataset folder name under data/")
    ap.add_argument("--target", type=int, default=40, help="stop after this many kept episodes (resumable)")
    ap.add_argument("--config", default=str(PROJECT / "configs" / "collect.yaml"))
    ap.add_argument("--task", default="Place the wellplate on the target (wrist camera, keyboard teleop)")
    ap.add_argument("--viewer-port", type=int, default=8090, help="web viewer for saved episodes (next free port is used)")
    args = ap.parse_args(argv)

    cfg = load_config(Path(args.config))
    dataset_dir = PROJECT / "data" / args.dataset
    (dataset_dir / "episodes").mkdir(parents=True, exist_ok=True)
    (dataset_dir / "sessions").mkdir(exist_ok=True)
    logfile = dataset_dir / "sessions" / datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.log")
    fh = logging.FileHandler(logfile)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s"))
    root = logging.getLogger("act")
    root.setLevel(logging.INFO)
    root.addHandler(fh)
    ch = logging.StreamHandler()
    ch.setLevel(logging.WARNING)
    root.addHandler(ch)

    say(f"dataset: {dataset_dir}   session log: {logfile.name}")
    ws = cfg["workspace_mm"]
    say(f"workspace box (mm): x {ws['x']}  y {ws['y']}  z {ws['z']}   <- check these for your deck (z min = table clearance)")
    say("keys: W/S +-X  A/D +-Y  R/F +-Z  Q/E last-joint rotation  1/2/3 speed  Enter finish  Esc stop+abort   (hold to move)")
    say("SAFETY: keep a hand on the e-stop. Keys are read globally: only type in this session while it is running.")

    def _terminate(signum, frame):  # closing the terminal (SIGHUP) or `kill` must also run the cleanup
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _terminate)

    rig = (rig_factory or build_rig)(cfg)
    viewer = None
    try:
        viewer, rig.viewer_url = start_viewer(dataset_dir, args.viewer_port)
        if rig.viewer_url:
            say(f"viewer: {rig.viewer_url}   (open it in a browser; every saved episode plays back there, and a link is printed after each save)")
        run_session(rig, cfg, dataset_dir, args.target, args.task)
    except KeyboardInterrupt:
        say("\nInterrupted (Ctrl-C): arm stopped, current episode discarded.")
    finally:
        say("cleaning up: handing the Xavier and the arm back (Ctrl-C is ignored for a moment) ...")
        if viewer is not None:
            viewer.terminate()
        rig.close()
        say("cleanup done (arm handed back, Xavier back to its normal mode)")


if __name__ == "__main__":
    main()

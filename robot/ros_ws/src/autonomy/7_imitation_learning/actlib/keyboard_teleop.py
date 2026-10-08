"""Keyboard Cartesian teleop for data collection.

  Hold to move, release to stop (nothing moves unless a key is held):
    W / S   +X / -X          A / D   +Y / -Y          R / F   +Z / -Z        (xArm base frame)
    Q / E   rotate the last joint +/-  (angular velocity about base +Z; the tool stays vertical)
    1 / 2 / 3   speed preset          Enter   finish          Esc   stop immediately and abort

The loop runs at `loop_hz` (100 Hz), slews the commanded velocity with acceleration limits (smooth start/stop),
keeps the TCP inside a soft workspace box, and re-arms the controller after a fault (see ArmLink.fault_reason).
Key events come from pynput (X11 global listener: it sees keys typed in ANY window while active).
"""

import contextlib
import logging
import sys
import termios
import threading
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

import numpy as np

log = logging.getLogger("act.teleop")

# key -> (axis, sign); axes: 0 x, 1 y, 2 z (mm/s), 3 wz (deg/s)
KEYMAP = {"w": (0, +1), "s": (0, -1), "a": (1, +1), "d": (1, -1), "r": (2, +1), "f": (2, -1), "q": (3, +1), "e": (3, -1)}
KEYS_DOC = {"w/s": "+X/-X", "a/d": "+Y/-Y", "r/f": "+Z/-Z", "q/e": "last-joint rotation +/-", "1/2/3": "speed preset",
            "enter": "finish", "esc": "stop immediately and abort"}


class KeyState:
    """Pressed-key tracker. `press` / `release` are called by the pynput listener (or by tests)."""

    def __init__(self, debounce_s: float = 0.03):
        self.debounce = debounce_s
        self._lock = threading.Lock()
        self._down: Set[str] = set()
        self._released: Dict[str, float] = {}
        self._enter = self._esc = False
        self._preset: Optional[int] = None
        self._active = False
        self._ignore_until = 0.0
        self._listener = None

    # ---- listener lifecycle
    def start_listener(self):
        from pynput import keyboard
        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()
        self._listener.wait()

    def stop_listener(self):
        if self._listener is not None:
            self._listener.stop()

    @staticmethod
    def _name(key) -> Optional[str]:
        from pynput import keyboard
        if key == keyboard.Key.enter:
            return "enter"
        if key == keyboard.Key.esc:
            return "esc"
        ch = getattr(key, "char", None)
        return ch.lower() if ch else None

    def _on_press(self, key):
        n = self._name(key)
        if n:
            self.press(n)

    def _on_release(self, key):
        n = self._name(key)
        if n:
            self.release(n)

    # ---- state
    def activate(self, ignore_enter_s: float = 0.4):
        """Start acting on keys. Enter is ignored briefly: the Enter that answered the previous prompt may still be
        in flight through the X server."""
        with self._lock:
            self._down.clear()
            self._released.clear()
            self._enter = self._esc = False
            self._preset = None
            self._ignore_until = time.monotonic() + ignore_enter_s
            self._active = True

    def deactivate(self):
        with self._lock:
            self._active = False
            self._down.clear()

    def press(self, name: str):
        with self._lock:
            if not self._active:
                return
            if name == "enter":
                if time.monotonic() >= self._ignore_until:
                    self._enter = True
            elif name == "esc":
                self._esc = True
            elif name in ("1", "2", "3"):
                self._preset = int(name)
            elif name in KEYMAP:
                self._down.add(name)
                self._released.pop(name, None)

    def release(self, name: str):
        with self._lock:
            if name in self._down:
                self._down.discard(name)
                self._released[name] = time.monotonic()

    def poll(self) -> dict:
        """Snapshot; one-shot events (enter / esc / preset) are consumed. A release counts only after `debounce`:
        X11 auto-repeat delivers release+press pairs while a key is held."""
        with self._lock:
            now = time.monotonic()
            pressed = set(self._down) | {k for k, t in self._released.items() if now - t < self.debounce}
            out = {"pressed": pressed, "enter": self._enter, "esc": self._esc, "preset": self._preset}
            self._enter = self._esc = False
            self._preset = None
            return out


def clamp_to_box(pos_mm, v_cmd_mm_s, target_mm_s, box_mm, accel_mm_s2: float, margin_mm: float = 2.0) -> np.ndarray:
    """Zero the target velocity on an axis that would carry the TCP through a wall. The TCP keeps moving for
    v*|v|/(2*accel) while the slew decelerates it, so the wall is applied to that predicted stopping point."""
    out = np.array(target_mm_s, dtype=float)
    for i in range(3):
        lo, hi = box_mm[i]
        p_stop = pos_mm[i] + v_cmd_mm_s[i] * abs(v_cmd_mm_s[i]) / (2.0 * accel_mm_s2)
        if out[i] > 0 and p_stop >= hi - margin_mm:
            out[i] = 0.0
        if out[i] < 0 and p_stop <= lo + margin_mm:
            out[i] = 0.0
    return out


@contextlib.contextmanager
def quiet_terminal():
    """No echo / line buffering while keys drive the arm, and typed keys are flushed before the next prompt."""
    if not sys.stdin.isatty():
        yield
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSADRAIN, new)
    try:
        yield
    finally:
        termios.tcflush(fd, termios.TCIFLUSH)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Teleop:
    """Runs the 100 Hz control loop; while `recording` every tick is appended to `rows`."""

    def __init__(self, cfg: dict, arm, feed, keys: KeyState):
        self.cfg = cfg
        self.arm, self.feed, self.keys = arm, feed, keys
        self.box = [cfg["_workspace"][a] for a in ("x", "y", "z")]
        self.preset = int(cfg["start_preset"]) - 1
        self.rows: List[tuple] = []
        self.events: List[Tuple[float, str]] = []
        self.faults = 0

    def reset_log(self):
        self.rows, self.events, self.faults = [], [], 0

    def _note(self, text: str, recording: bool):
        log.info(text)
        if recording:
            self.events.append((time.time(), text))

    def control_arrays(self) -> Dict[str, np.ndarray]:
        a = np.array(self.rows, dtype=np.float64).reshape(-1, 13)
        return {"t": a[:, 0], "cmd": a[:, 1:5], "key": a[:, 5:9], "state": a[:, 9].astype(np.int16),
                "mode": a[:, 10].astype(np.int16), "error": a[:, 11].astype(np.int32), "preset": a[:, 12].astype(np.int8) + 1}

    def run(self, recording: bool, on_status: Optional[Callable[[dict], None]] = None,
            max_seconds: Optional[float] = None) -> str:
        """Returns 'enter' (finished; velocity ramped to zero) or 'esc' (stopped at once)."""
        cfg = self.cfg
        period = 1.0 / cfg["loop_hz"]
        acc = np.array([cfg["accel_lin_mm_s2"]] * 3 + [cfg["accel_ang_deg_s2"]], dtype=float)
        presets = cfg["presets"]
        v = np.zeros(4)
        result, ending, t_end = None, None, 0.0
        was_faulted, last_rearm, last_status = False, 0.0, 0.0
        last = next_t = time.perf_counter()
        t_begin = last
        try:
            while True:
                now = time.perf_counter()
                dt = min(max(now - last, 0.5 * period), 5.0 * period)
                last = now
                k = self.keys.poll()
                if k["esc"]:
                    self.arm.stop()
                    v[:] = 0.0
                    self._note("Esc pressed: velocity zeroed, aborting", recording)
                    result = "esc"
                    break
                if k["preset"] is not None and 1 <= k["preset"] <= len(presets):
                    self.preset = k["preset"] - 1
                    self._note(f"speed preset {k['preset']}: {presets[self.preset]['lin_mm_s']} mm/s, "
                               f"{presets[self.preset]['ang_deg_s']} deg/s", recording)
                if k["enter"] and ending is None:
                    ending, t_end = "enter", now
                if max_seconds is not None and ending is None and now - t_begin > max_seconds:
                    ending, t_end = "enter", now
                lin, ang = presets[self.preset]["lin_mm_s"], presets[self.preset]["ang_deg_s"]
                target = np.zeros(4)
                for key in k["pressed"]:
                    axis, sign = KEYMAP[key]
                    target[axis] += sign
                target = np.clip(target, -1.0, 1.0) * np.array([lin, lin, lin, ang])
                if ending:
                    target[:] = 0.0
                key_vel = target.copy()

                latest = self.feed.latest
                if latest is not None:
                    target[:3] = clamp_to_box(latest["tcp"][:3], v[:3], target[:3], self.box, cfg["accel_lin_mm_s2"])
                step = acc * dt
                v += np.clip(target - v, -step, step)

                reason = self.arm.fault_reason(latest)
                if reason:
                    v[:] = 0.0
                    if not was_faulted:
                        self.faults += 1
                        self._note(f"FAULT ({reason}; error={self.arm.arm.error_code} warn={self.arm.arm.warn_code}): re-arming", recording)
                    was_faulted = True
                    if now - last_rearm > 0.25:
                        last_rearm = now
                        try:
                            self.arm.rearm()
                        except Exception as e:
                            log.warning("re-arm failed: %s", e)
                else:
                    if was_faulted:
                        self._note("controller re-armed", recording)
                        was_faulted = False
                    code = self.arm.velocity(v[0], v[1], v[2], v[3])
                    if code != 0 and abs(v).max() > 0:
                        log.debug("velocity command rejected, code=%s", code)  # usually STATE_NOT_READY between checks

                if recording:
                    st = latest["state"] if latest else -1
                    md = latest["mode"] if latest else -1
                    self.rows.append((time.time(), *v, *key_vel, st, md, self.arm.arm.error_code, self.preset))
                if ending and np.abs(v).max() < 1e-3:
                    result = ending
                    break
                if ending and now - t_end > 2.0:  # ramp should take < 0.5 s; never hang here
                    self.arm.stop()
                    v[:] = 0.0
                    result = ending
                    break
                if on_status is not None and now - last_status >= 0.2:
                    last_status = now
                    on_status({"elapsed": now - t_begin, "preset": self.preset + 1, "lin": lin, "ang": ang,
                               "tcp": latest["tcp"] if latest else None, "v": v.copy(), "faulted": was_faulted})
                next_t += period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -5 * period:
                    next_t = time.perf_counter()  # fell far behind: resync instead of bursting
        finally:
            self.arm.stop()
        return result or "esc"

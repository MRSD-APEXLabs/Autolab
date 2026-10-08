"""xArm I/O for data collection.

  StateFeed    robot state at the controller's real-time rate (~100 Hz, report port 30003): one row per packet.
  ArmLink      command connection: mode-5 Cartesian velocity streaming + fault detection / re-arm
               (the pattern proven in joy_cartesian_jog.py, git eb83bd6).
  GripperLog   gripper setpoint + both finger pressure sensors read at ~5 Hz. Read-only: nothing is ever written.

Why two SDK connections: the 'real' report (100 Hz) carries joints / pose / torque / state / mode but NOT the error
code, while the default 'rich' report has the error code but arrives at only 5 Hz.
"""

import collections
import logging
import math
import threading
import time
from typing import Dict, Optional

import numpy as np
from xarm.wrapper import XArmAPI

log = logging.getLogger("act.arm")

GRIP_READ_FRAME = [0x08, 0x03, 0x07, 0x00, 0x00, 0x04]  # regs 0x0700..0x0703, fn 0x03 only (fn 0x04 latches controller error 19)
GRIP_BAUD, GRIP_TIMEOUT_MS = 115200, 100


class StateFeed:
    def __init__(self, ip: str):
        self.arm = XArmAPI(ip, report_type="real")
        self._lock = threading.Lock()
        self._rows = []
        self._recording = False
        self._stamps = collections.deque(maxlen=400)
        self.latest: Optional[dict] = None
        self.arm.register_report_location_callback(self._on_packet, report_cartesian=True, report_joints=True)

    def _on_packet(self, data):  # runs in the SDK's report thread, once per packet, in order
        t = time.time()
        q, tcp = list(data["joints"][:6]), list(data["cartesian"][:6])
        tau = list(self.arm.joints_torque[:6])
        state, mode = self.arm.state, self.arm.mode
        with self._lock:
            self._stamps.append(t)
            self.latest = {"t": t, "q": q, "tcp": tcp, "tau": tau, "state": state, "mode": mode}
            if self._recording:
                self._rows.append((t, *q, *tcp, *tau, state, mode))

    def wait_ready(self, timeout: float = 5.0) -> bool:
        end = time.time() + timeout
        while self.latest is None and time.time() < end:
            time.sleep(0.02)
        return self.latest is not None

    def hz(self) -> float:
        with self._lock:
            now = time.time()
            n = sum(1 for s in self._stamps if now - s <= 1.0)
        return float(n)

    def start_recording(self):
        with self._lock:
            self._rows = []
            self._recording = True

    def stop_recording(self) -> Dict[str, np.ndarray]:
        with self._lock:
            self._recording = False
            rows, self._rows = self._rows, []
        a = np.array(rows, dtype=np.float64).reshape(-1, 21)
        return {"t": a[:, 0], "q": a[:, 1:7], "tcp": a[:, 7:13], "tau": a[:, 13:19],
                "state": a[:, 19].astype(np.int16), "mode": a[:, 20].astype(np.int16)}

    def close(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass


class ArmLink:
    """Mode-5 (Cartesian velocity) command link. `velocity()` is meant to be called at ~100 Hz."""

    def __init__(self, ip: str):
        self.arm = XArmAPI(ip)  # default 'rich' report: error_code / warn_code cached at 5 Hz
        self.initial_mode = 0   # mode the arm was in when this script found it; restored by release()

    def set_initial(self, mode: int):
        self.initial_mode = int(mode)

    def prepare(self):
        arm = self.arm
        if arm.warn_code != 0:
            arm.clean_warn()
        if arm.error_code != 0:
            arm.clean_error()
        arm.motion_enable(True)
        arm.set_mode(5)
        arm.set_state(0)
        time.sleep(0.3)

    def velocity(self, vx: float, vy: float, vz: float, wz_deg: float) -> int:
        """mm/s in the base frame; wz in deg/s about base +Z. duration=0 (continuous) is required: a non-zero duration
        makes the firmware re-plan on every command and the motion stutters."""
        return self.arm.vc_set_cartesian_velocity([vx, vy, vz, 0.0, 0.0, math.radians(wz_deg)],
                                                  is_radian=True, duration=0.0)

    def stop(self):
        try:
            self.arm.vc_set_cartesian_velocity([0.0] * 6, is_radian=True, duration=0.0)
        except Exception as e:
            log.warning("stop() failed: %s", e)

    def fault_reason(self, latest: Optional[dict]) -> Optional[str]:
        """None if healthy. Near a singularity / speed limit the controller drops to state 4 with NO error code and
        silently discards every velocity command until it is re-armed."""
        if latest is None:
            return "no state from controller"
        if self.arm.error_code != 0:
            return f"controller error {self.arm.error_code}"
        if latest["state"] not in (0, 1, 2):
            return f"controller state {latest['state']}"
        if latest["mode"] != 5:
            return f"controller mode {latest['mode']}"
        return None

    def rearm(self):
        arm = self.arm
        if arm.error_code != 0 or arm.warn_code != 0:
            arm.clean_error()
            arm.clean_warn()
            arm.motion_enable(True)
        if arm.mode != 5:
            arm.set_mode(5)
        arm.set_state(0)

    def move_to_pose(self, pose, speed_mm_s: float):
        """Blocking straight-line move in position mode, then back to velocity mode."""
        arm = self.arm
        arm.set_mode(0)
        arm.set_state(0)
        code = arm.set_position(*pose[:6], speed=speed_mm_s, mvacc=200, is_radian=False, wait=True)
        self.prepare()
        return code

    def hand_back(self, target_mode: Optional[int] = None):
        """Stop, then give the arm back in `target_mode` (default: the mode it was found in). With the xArm ROS driver
        running that is servo mode 1: the driver notices the arm left mode 1 while we were driving, deactivates its own
        controllers, and reactivates them by itself once the arm is back in mode 1 (uf_robot_system_hardware.cpp,
        _need_reset). The connection stays open so the next jog / episode can take the arm again with prepare()."""
        self.stop()
        time.sleep(0.3)  # let the arm finish decelerating before the mode changes
        arm = self.arm
        if arm.error_code != 0 or arm.warn_code != 0:
            arm.clean_error()
            arm.clean_warn()
        arm.motion_enable(True)
        arm.set_mode(self.initial_mode if target_mode is None else int(target_mode))
        arm.set_state(0)

    def release(self, target_mode: Optional[int] = None):
        try:
            self.hand_back(target_mode)
            self.arm.disconnect()
        except Exception as e:
            log.warning("release: %s", e)


class GripperLog(threading.Thread):
    """Logs gripper state while recording. Gives up after 5 consecutive failed reads (a missed Modbus reply can latch
    controller error 19), so a flaky tool bus can never keep interrupting the arm."""

    def __init__(self, arm: XArmAPI, hz: float = 5.0):
        super().__init__(name="gripper-log", daemon=True)
        self.arm = arm
        self.period = 1.0 / hz
        self._lock = threading.Lock()
        self._rows = []
        self._recording = False
        self._running = True
        self.failures = 0
        self.disabled = False
        arm.set_tgpio_modbus_baudrate(GRIP_BAUD)
        arm.set_tgpio_modbus_timeout(GRIP_TIMEOUT_MS)

    def start_recording(self):
        with self._lock:
            self._rows = []
            self._recording = True

    def stop_recording(self) -> Dict[str, np.ndarray]:
        with self._lock:
            self._recording = False
            rows, self._rows = self._rows, []
        a = np.array(rows, dtype=np.float64).reshape(-1, 4)
        return {"t": a[:, 0], "setpoint": a[:, 1].astype(np.int32), "fsr": a[:, 2:4].astype(np.int32)}

    def stop(self):
        self._running = False

    def run(self):
        consecutive = 0
        while self._running and not self.disabled:
            started = time.monotonic()
            with self._lock:
                recording = self._recording
            if recording:
                t = time.time()
                try:
                    code, ret = self.arm.getset_tgpio_modbus_data(GRIP_READ_FRAME)
                except Exception:
                    code, ret = -1, None
                if code == 0 and ret and len(ret) >= 11:
                    consecutive = 0
                    with self._lock:
                        self._rows.append((t, ret[5] << 8 | ret[6], ret[7] << 8 | ret[8], ret[9] << 8 | ret[10]))
                else:
                    consecutive += 1
                    self.failures += 1
                    if consecutive >= 5:
                        self.disabled = True
                        log.warning("gripper reads failing: gripper logging disabled for this session")
            time.sleep(max(0.0, self.period - (time.monotonic() - started)))

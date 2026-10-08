"""xArm6 and the FSR gripper over the xArm SDK (a second connection next to xarm_ros2, as Camera-Edge did).

Poses leave this module as base_T_flange 4x4 matrices in metres (mount.py). The controller's TCP offset is
read from the arm, so the reported TCP (currently 140 mm down the flange) can be anything.

The gripper is the Camera-Edge one: an Arduino on the tool RS-485 port, Modbus device 0x08. Register 0x0700
takes the target position (0 open .. 1000), 0x0702/0x0703 hold the two finger FSRs (0..1023). The Modbus
frames are copied from Camera-Edge utils.py. Unlike there, the gripper calls don't reset the arm's mode, so
they can run while the arm is in any mode.
"""
from __future__ import annotations

import logging
import time

import numpy as np

from .mount import matrix_xarm_pose, transform, xarm_pose_matrix, inverse, rpy_matrix

log = logging.getLogger(__name__)

MODE_POSITION, MODE_SERVO, MODE_VELOCITY = 0, 1, 5
GRIPPER_DEVICE = 0x08


class ArmError(RuntimeError):
    pass


class XArm:
    def __init__(self, ip, api_factory=None, read_only=False):
        self.ip = ip
        self.read_only = bool(read_only)
        self._factory = api_factory
        self._arm = None
        self.flange_T_tcp = np.eye(4)
        self.mode = None

    # -- connection and state ------------------------------------------------------------------
    @property
    def connected(self):
        return self._arm is not None and bool(self._arm.connected)

    def connect(self):
        if self.connected:
            return
        if self._factory is None:
            from xarm.wrapper import XArmAPI
            self._factory = XArmAPI
        try:
            self._arm = self._factory(self.ip, is_radian=False)
        except Exception as exc:   # the SDK raises a bare Exception when the socket doesn't connect
            raise ArmError(f'xArm connection to {self.ip} failed: {exc}') from exc
        if not self._arm.connected:
            self._arm = None
            raise ArmError(f'xArm connection to {self.ip} failed')
        self._wait_first_report()
        offset = [float(v) for v in self._arm.tcp_offset]   # [x, y, z mm, roll, pitch, yaw deg]
        self.flange_T_tcp = xarm_pose_matrix(offset)
        log.info('xArm %s connected, TCP offset %s', self.ip, offset)

    def _wait_first_report(self, timeout=2.0):
        """The SDK fills tcp_offset from its report socket, so right after connecting it reads all zeros (on the arm
        a run then took the TCP, 140 mm down, for the flange). Wait until the first report has been parsed."""
        sdk = getattr(self._arm, '_arm', None)
        deadline = time.monotonic() + timeout
        while sdk is not None and not getattr(sdk, '_first_report_over', True):
            if time.monotonic() > deadline:
                self.disconnect()
                raise ArmError(f'no state report from the xArm {self.ip} within {timeout:.0f} s')
            time.sleep(0.02)

    def disconnect(self):
        if self._arm is not None:
            try:
                self._arm.disconnect()
            finally:
                self._arm = None

    def _require(self):
        if not self.connected:
            raise ArmError('xArm not connected')
        return self._arm

    def _check(self, code, what):
        if code != 0:
            raise ArmError(f'xArm {what} failed: code {code}')

    def _motion(self):
        if self.read_only:
            raise ArmError('dry run: the arm is read-only')
        return self._require()

    def errors(self):
        code, (error, warn) = self._require().get_err_warn_code()
        self._check(code, 'get_err_warn_code')
        return int(error), int(warn)

    def flange_pose(self):
        """base_T_flange from the reported TCP pose."""
        code, pose = self._require().get_position(is_radian=False)
        self._check(code, 'get_position')
        pose = np.asarray(pose[:6], dtype=float)
        if not np.isfinite(pose).all():
            raise ArmError(f'invalid pose from the arm: {pose}')
        return xarm_pose_matrix(pose) @ inverse(self.flange_T_tcp)

    # -- modes -------------------------------------------------------------------------------------
    def _set_mode(self, mode):
        arm = self._motion()
        arm.clean_error()
        arm.clean_warn()
        self._check(arm.motion_enable(True), 'motion_enable')
        self._check(arm.set_mode(mode), f'set_mode({mode})')
        self._check(arm.set_state(0), 'set_state(0)')
        time.sleep(0.1)
        self.mode = mode

    def position_mode(self):
        self._set_mode(MODE_POSITION)

    def velocity_mode(self):
        self._set_mode(MODE_VELOCITY)

    def hand_back(self):
        """Mode 1, state 0: what xarm_ros2 needs to take the arm back (Camera-Edge cleanup). No disconnect,
        which can reset the mode."""
        if self.read_only or not self.connected:
            return
        self._set_mode(MODE_SERVO)

    # -- motion ------------------------------------------------------------------------------------
    def velocity(self, v_tcp, wz, duration):
        """Base-frame TCP velocity (m/s) and yaw rate (rad/s). The controller zeroes it after `duration` s
        unless another command arrives, so a stalled loop stops the arm."""
        arm = self._motion()
        if self.mode != MODE_VELOCITY:
            raise ArmError('velocity command outside velocity mode')
        speeds = [float(v_tcp[0]) * 1000.0, float(v_tcp[1]) * 1000.0, float(v_tcp[2]) * 1000.0, 0.0, 0.0, float(wz)]
        self._check(arm.vc_set_cartesian_velocity(speeds, is_radian=True, is_tool_coord=False, duration=float(duration)),
                    'vc_set_cartesian_velocity')

    def stop(self):
        if self.read_only or not self.connected or self.mode != MODE_VELOCITY:
            return
        try:
            self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian=True, is_tool_coord=False, duration=0)
        except Exception as exc:
            log.warning('zero velocity failed: %s', exc)

    def move_flange(self, base_T_flange, speed_mm_s, acc_mm_s2=500.0):
        """Blocking linear move of the flange to `base_T_flange` (position mode)."""
        arm = self._motion()
        if self.mode != MODE_POSITION:
            raise ArmError('position command outside position mode')
        x, y, z, roll, pitch, yaw = matrix_xarm_pose(base_T_flange @ self.flange_T_tcp)
        self._check(arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=float(speed_mm_s),
                                     mvacc=float(acc_mm_s2), is_radian=False, wait=True), 'set_position')

    # -- gripper -----------------------------------------------------------------------------------
    def _modbus(self, data):
        code, ret = self._motion().getset_tgpio_modbus_data(data, is_transparent_transmission=False)
        if code != 0:
            self._arm.clean_error()
        return code, ret

    def gripper_setup(self):
        arm = self._motion()
        arm.set_tgpio_modbus_timeout(50)
        arm.set_tgpio_modbus_baudrate(115200)

    def gripper_set(self, value, settle=0.5):
        """Target position 0 (open) .. 1000."""
        value = int(np.clip(int(value), 0, 1000))
        code, _ = self._modbus([GRIPPER_DEVICE, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00,
                                (value >> 8) & 0xFF, value & 0xFF])
        if code != 0:
            log.warning('gripper set %d failed: code %s', value, code)
        time.sleep(settle)
        return code == 0

    def gripper_open(self):
        return self.gripper_set(0, settle=0.0)

    def gripper_position(self):
        """The gripper's position register (0x0700-0x0701, 0..1000): the last target it was given; the Arduino
        reports no measured position. None if the read fails."""
        code, ret = self._modbus([GRIPPER_DEVICE, 0x03, 0x07, 0x00, 0x00, 0x02])
        if code != 0 or ret is None or len(ret) < 7:
            return None
        return (ret[5] << 8) | ret[6]

    def gripper_fsr(self):
        """(fsr1, fsr2) or (None, None)."""
        code, ret = self._modbus([GRIPPER_DEVICE, 0x03, 0x07, 0x02, 0x00, 0x02])
        if code != 0 or ret is None or len(ret) < 7:
            return None, None
        return (ret[3] << 8) | ret[4], (ret[5] << 8) | ret[6]


def level_pose(base_T_flange, yaw=None):
    """The same position with the tool pointing straight down (roll 180 deg, pitch 0) at `yaw` (the current
    yaw by default)."""
    if yaw is None:
        yaw = float(np.arctan2(base_T_flange[1, 0], base_T_flange[0, 0]))
    return transform(rpy_matrix(np.pi, 0.0, yaw), base_T_flange[:3, 3])


def flange_for_tip(tip_position, yaw, flange_T_tip):
    """base_T_flange that puts the fingertip at `tip_position` with the tool down at `yaw`."""
    rotation = rpy_matrix(np.pi, 0.0, yaw)
    return transform(rotation, np.asarray(tip_position, dtype=float) - rotation @ flange_T_tip[:3, 3])

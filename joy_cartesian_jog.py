#!/usr/bin/env python3
"""
Smooth joystick jogging for the xArm6 -- maps an Xbox360-style controller onto
the same jog controls UFACTORY Studio exposes (base-frame XYZ + RXYZ + gripper),
using the arm's native velocity-control mode (mode 5). The control box does the
smoothing on-board, so motion is smooth like Studio's jog (no MoveIt, no ROS).

Studio control            -> Joystick
--------------------------------------------------------------------------
XYZ  X+/X-  (translate)    -> Left stick  up/down
XYZ  Y+/Y-  (translate)    -> Left stick  left/right
XYZ  Z+/Z-  (translate)    -> RT / LT triggers   (RT = up, LT = down)
RXYZ RX+/RX- (rotate)      -> Right stick up/down
RXYZ RY+/RY- (rotate)      -> Right stick left/right
RXYZ RZ+/RZ- (rotate)      -> RB / LB bumpers
Gripper close / open       -> A / Y buttons
Gripper nudge -/+          -> D-pad left / right

Run (E-stop in hand):
    /home/labx/coding/Autolab/.venv_jog/bin/python joy_cartesian_jog.py
Stop: Ctrl+C

NOTE: nothing else may actively command the arm while this runs. Stop the MoveIt
servo launch / robot_bringup arm driver first. Studio can stay open, but don't
jog in Studio at the same time.

Any direction feels backwards? Flip the sign on that line in read_joystick().
"""
import math
import struct
import threading
import time

from xarm.wrapper import XArmAPI

# ---------------- config ----------------
ARM_IP = "192.168.1.236"
JS_DEV = "/dev/input/js0"

LIN_SPEED = 75.0    # mm/s  at full deflection   (~Studio 25%; raise to taste)
ANG_SPEED = 0.3      # rad/s at full deflection (~34 deg/s); needs is_radian=True in the call
DEADZONE = 0.15      # trigger deadzone (LT/RT rest at -1.0, so no drift issue)

# Stick deadzone is a Schmitt trigger, NOT a single threshold. Measured on this
# controller: the right stick's Y axis rests at -0.021 cold, but settles at
# -0.136..-0.143 after being pushed and released -- only 0.007 from the old
# single 0.15 threshold. Past it, FIXED_SPEED commands FULL speed, so the arm
# ran away in rotation with the stick released. A startup calibration would not
# help: it samples the -0.021 cold value, not the post-use settle point.
DZ_ON = 0.30         # must exceed this to START moving (>2x the measured settle)
DZ_OFF = 0.18        # keeps moving until it drops below this (> settle, so it releases)
FIXED_SPEED = True   # True: any push past the deadzone = full fixed speed (like
                     # Studio's buttons). False: speed proportional to deflection.
RATE_HZ = 80.0       # velocity command rate (denser stream = smoother)
CMD_DURATION = 0.0   # 0 = "always effective, never auto-stops". MUST be 0.
                     # A nonzero duration makes the firmware plan a *bounded*
                     # motion with a decel ramp toward that horizon; re-sending
                     # at RATE_HZ then re-plans constantly and the decel phases
                     # show up as visible stop-go stutter (measured: 0.5 gave an
                     # irregular 25 mm/s against a commanded 30; 0.0 held a flat
                     # 30 mm/s).
                     # TRADE-OFF: with 0 there is no firmware watchdog -- if this
                     # script is killed mid-jog the arm keeps moving. The finally
                     # block zeroes the velocity on a clean exit (incl. Ctrl+C);
                     # for anything else, that is what the E-stop is for.

ENABLE_GRIPPER = True
# The gripper is a CUSTOM Modbus RTU device on the xArm tool-GPIO RS485 bus.
# It is NOT a UFACTORY gripper -- set_gripper_position() & friends get no reply
# (probed: get_gripper_position -> code 1, get_gripper_err_code -> code 22).
# Frame layout, from the known-good teleop script:
#   [id, fn, addr_hi,addr_lo, nreg_hi,nreg_lo, nbytes, d0,d1, d2,d3]
#   id=0x08  fn=0x10 (write multiple registers)  addr=0x0700  nreg=2  nbytes=4
#   registers written: 0x0700 = 0x0000, 0x0701 = <position>
# The controller appends the Modbus CRC itself, so no CRC bytes here.
GRIP_ID, GRIP_FN, GRIP_ADDR = 0x08, 0x10, 0x0700
GRIP_OPEN = 0x6E     # 110 -- the two values the existing teleop script uses.
GRIP_CLOSE = 178    # 158
# Nudge bounds are deliberately clamped to the two proven values. Do NOT widen
# these without knowing the gripper's real mechanical range -- driving the
# position register past its travel can stall or damage the actuator.
GRIP_MIN, GRIP_MAX = min(GRIP_OPEN, GRIP_CLOSE), max(GRIP_OPEN, GRIP_CLOSE)
GRIP_STEP = 4        # units per D-pad left/right nudge
GRIP_BAUD, GRIP_TIMEOUT = 115200, 50

# Xbox360 wired indices (raw linux joystick interface)
AX_LX, AX_LY = 0, 1        # left stick
AX_LT, AX_RT = 2, 5        # triggers (rest ~= -1, full ~= +1)
AX_RX, AX_RY = 3, 4        # right stick
AX_DPAD_X = 6              # d-pad left/right -> gripper nudge
AX_DPAD_Y = 7              # d-pad up/down (unused)
BTN_LB, BTN_RB = 4, 5      # bumpers
BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
BTN_GRIP_CLOSE = BTN_A     # A = close
BTN_GRIP_OPEN = BTN_Y      # Y = open  (vertically opposed on the pad)
# ----------------------------------------

axes = {}
buttons = {}
_lock = threading.Lock()
_running = True


def js_reader():
    """Background thread: keep `axes` and `buttons` updated from /dev/input/js0."""
    fmt = "IhBB"                       # js_event: uint32 time, int16 value, uint8 type, uint8 number
    size = struct.calcsize(fmt)
    with open(JS_DEV, "rb") as f:
        while _running:
            data = f.read(size)
            if not data:
                break
            _t, value, typ, number = struct.unpack(fmt, data)
            if typ & 0x02:             # axis event
                with _lock:
                    axes[number] = max(-1.0, min(1.0, value / 32767.0))
            elif typ & 0x01:           # button event
                with _lock:
                    buttons[number] = value


_engaged = {}   # per-axis Schmitt state: is this axis currently commanding motion?


def shape(v, axis):
    """Hysteretic deadzone + speed mode. An axis must exceed DZ_ON to engage,
    then stays engaged until it falls below DZ_OFF. A stick that settles between
    the two after release therefore cannot re-trigger motion on its own."""
    if _engaged.get(axis, False):
        if abs(v) < DZ_OFF:
            _engaged[axis] = False
            return 0.0
    else:
        if abs(v) < DZ_ON:
            return 0.0
        _engaged[axis] = True
    return math.copysign(1.0, v) if FIXED_SPEED else v


def trig(v01):
    """Trigger magnitude 0..1 -> fixed (0/1) or proportional past deadzone."""
    if v01 <= DEADZONE:
        return 0.0
    return 1.0 if FIXED_SPEED else v01


def read_joystick():
    """Return (cartesian_velocity[6], gripper_dir) from the current stick state."""
    with _lock:
        lx = axes.get(AX_LX, 0.0); ly = axes.get(AX_LY, 0.0)
        rx = axes.get(AX_RX, 0.0); ry = axes.get(AX_RY, 0.0)
        lt = axes.get(AX_LT, -1.0); rt = axes.get(AX_RT, -1.0)
        lb = buttons.get(BTN_LB, 0);  rb = buttons.get(BTN_RB, 0)

    vx = -shape(ly, "ly") * LIN_SPEED        # left stick up    -> X+
    vy = -shape(lx, "lx") * LIN_SPEED        # left stick left  -> Y+
    lt01 = (lt + 1.0) / 2.0                  # triggers -1..1 -> 0..1
    rt01 = (rt + 1.0) / 2.0
    vz = (trig(rt01) - trig(lt01)) * LIN_SPEED   # RT -> Z+ , LT -> Z-

    # right stick: horizontal -> RX (roll), vertical -> RY (pitch)
    wx =  shape(rx, "rx") * ANG_SPEED        # right stick right -> RX+
    wy = -shape(ry, "ry") * ANG_SPEED        # right stick up    -> RY+
    wz = (rb - lb) * ANG_SPEED               # RB -> RZ+ , LB -> RZ-

    return [vx, vy, vz, wx, wy, wz]


_prev = {"close": 0, "open": 0, "dpad_x": 0.0}


def read_gripper_request():
    """Edge-triggered gripper intent, so one press = one Modbus frame.
    Returns ('abs', position) | ('nudge', +/-1) | None."""
    with _lock:
        close_btn = buttons.get(BTN_GRIP_CLOSE, 0)
        open_btn = buttons.get(BTN_GRIP_OPEN, 0)
        dx = axes.get(AX_DPAD_X, 0.0)

    req = None
    if close_btn and not _prev["close"]:
        req = ("abs", GRIP_CLOSE)
    elif open_btn and not _prev["open"]:
        req = ("abs", GRIP_OPEN)
    elif dx > 0.5 and not _prev["dpad_x"] > 0.5:
        req = ("nudge", +1)
    elif dx < -0.5 and not _prev["dpad_x"] < -0.5:
        req = ("nudge", -1)

    _prev["close"], _prev["open"], _prev["dpad_x"] = close_btn, open_btn, dx
    return req


_grip_lock = threading.Lock()
_grip_want = None       # pending absolute position, or None


def gripper_setup(arm):
    arm.set_tgpio_modbus_timeout(GRIP_TIMEOUT)
    arm.set_tgpio_modbus_baudrate(GRIP_BAUD)


def gripper_worker(arm):
    """Own thread. A Modbus round-trip blocks for up to GRIP_TIMEOUT ms (and the
    full timeout if the gripper does not answer). Doing that inline would stall
    the velocity stream for several cycles, so the jog loop only ever sets
    _grip_want and this thread does the talking."""
    global _grip_want
    while _running:
        with _grip_lock:
            want, _grip_want = _grip_want, None
        if want is None:
            time.sleep(0.01)
            continue
        frame = [GRIP_ID, GRIP_FN,
                 (GRIP_ADDR >> 8) & 0xFF, GRIP_ADDR & 0xFF,
                 0x00, 0x02,                     # write 2 registers
                 0x04,                           # 4 data bytes
                 0x00, 0x00,                     # reg 0x0700 = 0
                 (want >> 8) & 0xFF, want & 0xFF]  # reg 0x0701 = position
        try:
            code, ret = arm.getset_tgpio_modbus_data(frame)
            if code != 0:
                print("[gripper] pos=%d modbus code=%s ret=%s" % (want, code, ret))
        except Exception as e:
            print("[gripper] pos=%d error: %s" % (want, e))


def arm_setup(arm):
    """Clear any latched fault and put the arm in continuous velocity mode."""
    if arm.warn_code != 0:
        arm.clean_warn()
    if arm.error_code != 0:
        arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(5)                          # 5 = continuous velocity control
    arm.set_state(0)


def main():
    arm = XArmAPI(ARM_IP)
    arm_setup(arm)
    print("err/warn after setup:", arm.get_err_warn_code())

    grip_target = None
    if ENABLE_GRIPPER:
        try:
            gripper_setup(arm)
            # No position feedback is read back: the write-only frame from the
            # working teleop script is all we know is safe on this device. Start
            # the shadow at OPEN so the first nudge is from a known value.
            grip_target = GRIP_OPEN
            print("gripper: modbus id=%d @ %d baud, open=%d close=%d"
                  % (GRIP_ID, GRIP_BAUD, GRIP_OPEN, GRIP_CLOSE))
        except Exception as e:
            print("gripper init skipped:", e)
            grip_target = None

    time.sleep(0.3)
    print("Ready. Left stick = X/Y, triggers = Z, right stick = RX/RY, "
          "bumpers = RZ, A = close, Y = open, D-pad L/R = nudge. Ctrl+C to stop.")

    threading.Thread(target=js_reader, daemon=True).start()
    if grip_target is not None:
        threading.Thread(target=gripper_worker, args=(arm,), daemon=True).start()

    period = 1.0 / RATE_HZ
    last_err_note = 0.0
    try:
        while True:
            # The controller aborts Cartesian velocity motion near a singularity
            # or joint-velocity limit by dropping to state 4 (STOP) -- often with
            # NO error code. Once in state 4 every velocity command is discarded,
            # so we must watch state, not just error_code, or jogging dies for
            # good the first time you graze a singularity.
            if arm.error_code != 0 or arm.state not in (0, 1, 2):
                now = time.time()
                if now - last_err_note > 1.0:
                    print(f"[arm halted: err={arm.error_code} state={arm.state}] "
                          f"re-arming -- steer away from limits/singularities")
                    last_err_note = now
                if arm.error_code != 0:
                    arm.clean_error()
                    arm.clean_warn()
                    arm.motion_enable(True)
                if arm.mode != 5:
                    arm.set_mode(5)
                arm.set_state(0)
                time.sleep(0.05)
                continue

            vel = read_joystick()
            # is_radian=True: wx/wy/wz below are rad/s (SDK default treats them
            # as deg/s, which makes rotations ~57x too slow).
            code = arm.vc_set_cartesian_velocity(vel, is_radian=True, duration=CMD_DURATION)
            if code != 0:
                # Most often STATE_NOT_READY (9): the arm stopped underneath us
                # between the check above and this call. Loop back and re-arm.
                now = time.time()
                if now - last_err_note > 1.0:
                    print(f"[vc command rejected, code={code}] re-arming")
                    last_err_note = now
                continue

            if grip_target is not None:
                req = read_gripper_request()
                if req is not None:
                    kind, val = req
                    if kind == "abs":
                        grip_target = val
                    else:
                        grip_target = max(GRIP_MIN,
                                          min(GRIP_MAX, grip_target + val * GRIP_STEP))
                    # hand off to the worker; never block the velocity stream
                    global _grip_want
                    with _grip_lock:
                        _grip_want = grip_target

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        global _running
        _running = False
        arm.vc_set_cartesian_velocity([0.0] * 6)
        time.sleep(0.2)
        arm.disconnect()


if __name__ == "__main__":
    main()

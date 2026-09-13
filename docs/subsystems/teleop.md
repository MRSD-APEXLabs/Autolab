# Joystick Teleop

**Purpose:** direct Cartesian jogging of the xArm6 with an Xbox-style controller — a software replacement for UFACTORY Studio's jog panel. Deliberately **bypasses ROS and MoveIt entirely**: it talks straight to the arm over the xArm Python SDK in velocity mode (`set_mode(5)`).

!!! note "⚠ local file, not in git"
    `joy_cartesian_jog.py` and its venv `.venv_jog/` (whose only package is `xarm-python-sdk 1.18.4`) exist only on the robot host machine.

!!! danger "Read before running"
    - **Nothing else may command the arm concurrently.** Stop the MoveIt servo launch / `robot_bringup` arm driver / any UFACTORY Studio session first.
    - `CMD_DURATION` is `0.0` by design (non-zero causes stop-go stutter), which means **no firmware watchdog: if the script is killed mid-motion, the arm keeps moving**. Stop it with `Ctrl+C` (which zeroes velocity), not `kill -9`, and keep the e-stop in hand.

## Running

```bash
/home/labx/coding/Autolab/.venv_jog/bin/python /home/labx/coding/Autolab/joy_cartesian_jog.py
```

Requirements: arm reachable at `ARM_IP = "192.168.1.236"`, joystick at `JS_DEV = "/dev/input/js0"` (both constants at the top of the script). Exit with `Ctrl+C`.

## Controls

| Input | Action |
|---|---|
| Left stick | X / Y translation |
| RT / LT | Z down / up |
| Right stick | RX / RY rotation |
| RB / LB | RZ ± |
| A | Close gripper |
| Y | Open gripper |
| D-pad left/right | Nudge ±4 (gripper fine adjust) |

## The gripper

The gripper is a **custom Modbus RTU device** on the xArm tool-GPIO RS-485 bus (device id `0x08`, function `0x10`, register `0x0700`, 115200 baud) — *not* a UFACTORY gripper, so the SDK's `set_gripper_position()` does **not** work; the script writes Modbus frames directly.

## Engineering notes embedded in the script

Worth knowing before you edit it:

- **Schmitt-trigger deadzone** (`DZ_ON = 0.30`, `DZ_OFF = 0.18`): the right stick settles at about −0.14 after release; a single threshold caused runaway rotation.
- **`is_radian=True` is required** on SDK calls, or rotational jogs are ~57× too slow (degrees interpreted as radians).
- **Singularity watch:** near singularities the controller silently drops to state 4 without setting `error_code`, so the control loop monitors `arm.state`, not just errors.

## Relation to the rest of the stack

This is a maintenance/recovery tool (e.g. repositioning the arm for [waypoint teaching](planning.md#the-taught-waypoint-roadmap-toolchain), untangling after a failed grasp). For ROS-integrated servoing, the vendored `xarm_moveit_servo` package exists (`ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.236`) but is not used by the lab routine.

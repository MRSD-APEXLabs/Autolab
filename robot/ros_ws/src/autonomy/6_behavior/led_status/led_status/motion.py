"""Base velocity -> the LED firmware's navigation command. No ROS dependency.

The firmware (led/robot_status) takes "V<vx>,<vy>,<turn>" in mm/s: robot frame (+x forward, +y left), and turn is
the rotation's speed at the LED strip (+ counter-clockwise seen from above). Its packets flow at these speeds, so
the strip moves with the ground.
"""
import math

HALF_SIZE_M = 0.34   # half of the square base (nav2 footprint +-0.34 m): where the strip sits
LIMIT_MM_S = 3000    # the firmware clamps to this too


def velocity_command(vx, vy, wz, half_size_m=HALF_SIZE_M, step_mm_s=10, still_mm_s=15):
    """vx, vy [m/s] and wz [rad/s] -> "V<vx>,<vy>,<turn>". Values are rounded to `step_mm_s`; anything slower
    than `still_mm_s` is sent as 0, so odometry noise while stopped does not move the packets. Non-finite
    values count as 0."""
    def mm_s(value):
        value = value * 1000 if math.isfinite(value) else 0.0
        if abs(value) < still_mm_s:
            return 0
        value = max(-LIMIT_MM_S, min(LIMIT_MM_S, value))
        return int(round(value / step_mm_s)) * step_mm_s
    return f'V{mm_s(vx)},{mm_s(vy)},{mm_s(wz * half_size_m)}'


def pattern(letter, odom, now, stale_s=0.5):
    """(pattern letter, command to send). While navigating ('M') with odometry `odom` = (receive time, vx, vy, wz)
    no older than `stale_s`, the command is the velocity one and the letter 'V'; otherwise both are `letter`
    (no odometry: the fixed forward-flow 'M')."""
    if letter == 'M' and odom is not None and now - odom[0] <= stale_s:
        return 'V', velocity_command(*odom[1:])
    return letter, letter

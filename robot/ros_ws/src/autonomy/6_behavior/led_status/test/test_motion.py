import math

from led_status.motion import pattern, velocity_command


def test_forward_left_and_rotation_in_mm_s():
    assert velocity_command(0.3, 0.0, 0.0) == 'V300,0,0'
    assert velocity_command(-0.12, 0.154, 0.0) == 'V-120,150,0'
    # 1 rad/s at the strip, 0.34 m from the centre: 340 mm/s counter-clockwise
    assert velocity_command(0.0, 0.0, 1.0) == 'V0,0,340'
    assert velocity_command(0.0, 0.0, -0.5) == 'V0,0,-170'


def test_noise_while_stopped_is_zero_and_values_are_clamped():
    assert velocity_command(0.004, -0.012, 0.03) == 'V0,0,0'
    assert velocity_command(9.0, -9.0, 0.0) == 'V3000,-3000,0'
    assert velocity_command(math.nan, math.inf, 0.0) == 'V0,0,0'


def test_velocity_pattern_only_while_navigating_with_fresh_odometry():
    odom = (10.0, 0.25, 0.0, 0.0)
    assert pattern('M', odom, 10.4) == ('V', 'V250,0,0')
    assert pattern('M', odom, 10.6) == ('M', 'M')          # odometry stale: fixed forward flow
    assert pattern('M', None, 10.0) == ('M', 'M')          # no odometry (bridge not running)
    assert pattern('A', odom, 10.1) == ('A', 'A')          # arm working: odometry ignored
    assert pattern('I', odom, 10.1) == ('I', 'I')

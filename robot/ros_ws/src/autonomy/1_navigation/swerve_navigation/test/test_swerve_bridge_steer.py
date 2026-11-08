"""SteerStabiliser: the azimuths must stay still under command jitter, and a stop must stay instant."""

import math
import random

import pytest
from swerve_navigation.swerve_bridge import ArmState
from swerve_navigation.swerve_bridge import CommandGovernor
from swerve_navigation.swerve_bridge import GovernorLimits
from swerve_navigation.swerve_bridge import SteerStabiliser
from swerve_navigation.swerve_bridge import STOP_COMMAND

DT = 0.02
RADIUS = 0.359
HOLD = 0.035
RATE = 2.5
MODULES = [(0.254, 0.254), (0.254, -0.254), (-0.254, 0.254), (-0.254, -0.254)]
LIMITS = GovernorLimits(max_linear_speed=1.0, max_angular_speed=1.5, max_linear_accel=2.0,
                        max_angular_accel=4.0, cmd_timeout=0.25)


def make() -> SteerStabiliser:
    return SteerStabiliser(HOLD, RATE, RADIUS)


def scaled(twist):
    return (twist[0], twist[1], twist[2] * RADIUS)


def norm(twist) -> float:
    return math.sqrt(sum(c * c for c in scaled(twist)))


def direction_angle(a, b) -> float:
    """Angle between two twist directions, opposite twists counting as equal."""
    dot = sum(x * y for x, y in zip(scaled(a), scaled(b))) / (norm(a) * norm(b))
    return math.acos(min(1.0, abs(dot)))


def azimuths(twist):
    """Module steer angles modulo 180 deg (a reversed drive needs no steering)."""
    vx, vy, wz = twist
    return [math.atan2(vy + wz * x, vx - wz * y) % math.pi for x, y in MODULES]


def azimuth_travel(twists) -> float:
    """Total steer travel [rad], summed over the four modules."""
    total, prev = 0.0, None
    for twist in twists:
        if norm(twist) < 1e-6:
            continue
        now = azimuths(twist)
        if prev is not None:
            for a, b in zip(prev, now):
                d = abs(a - b)
                total += min(d, math.pi - d)
        prev = now
    return total


def test_first_command_passes_unchanged():
    assert make().apply(0.3, -0.1, 0.4, at_rest=True, dt=DT) == (0.3, -0.1, 0.4)


def test_jitter_inside_the_hold_band_never_moves_the_azimuths():
    steer = make()
    rng = random.Random(1)
    first = azimuths(steer.apply(0.3, 0.0, 0.0, at_rest=True, dt=DT))
    for _ in range(200):
        wz = rng.uniform(-0.025, 0.025)     # atan(0.359 * 0.025 / 0.3) = 1.7 deg < 2 deg
        out = steer.apply(0.3, 0.0, wz, at_rest=False, dt=DT)
        assert azimuths(out) == pytest.approx(first, abs=1e-12)
        assert norm(out) <= norm((0.3, 0.0, wz)) + 1e-12


def test_mppi_like_wz_noise_is_strongly_damped():
    rng = random.Random(7)
    raw, filtered, steer = [], [], make()
    wz = 0.0
    for tick in range(500):
        if tick % 3 == 0:                   # a new controller output every ~50 ms
            wz = rng.gauss(0.0, 0.3)
        raw.append((0.2, 0.0, wz))
        filtered.append(steer.apply(0.2, 0.0, wz, at_rest=tick == 0, dt=DT))
    assert azimuth_travel(filtered) < 0.35 * azimuth_travel(raw)


def test_zero_command_is_zero_at_once_and_keeps_the_direction():
    steer = make()
    steer.apply(0.4, 0.0, 0.0, at_rest=True, dt=DT)
    assert steer.apply(0.0, 0.0, 0.0, at_rest=False, dt=DT) == (0.0, 0.0, 0.0)
    # Same direction again after the stop: nothing to re-steer.
    assert steer.apply(0.2, 0.0, 0.0, at_rest=True, dt=DT) == pytest.approx((0.2, 0.0, 0.0))


def test_output_is_never_faster_than_the_command():
    rng = random.Random(3)
    steer = make()
    for _ in range(2000):
        cmd = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1.5, 1.5))
        out = steer.apply(*cmd, at_rest=rng.random() < 0.1, dt=DT)
        assert norm(out) <= norm(cmd) + 1e-12
        # ... and never against the command.
        assert sum(a * b for a, b in zip(scaled(out), scaled(cmd))) >= -1e-12


def test_direction_change_is_rate_limited_while_moving():
    steer = make()
    prev = steer.apply(0.3, 0.0, 0.0, at_rest=True, dt=DT)
    ticks = 0
    while True:
        out = steer.apply(0.0, 0.3, 0.0, at_rest=False, dt=DT)
        if norm(out) > 1e-9 and norm(prev) > 1e-9:
            assert direction_angle(prev, out) <= RATE * DT + 1e-9
        prev, ticks = out, ticks + 1
        if direction_angle(out, (0.0, 0.3, 0.0)) <= HOLD + 1e-9:
            break
        assert ticks < 100
    assert ticks * DT == pytest.approx((math.pi / 2 - HOLD) / RATE, abs=2 * DT)


def test_from_rest_the_new_direction_is_taken_at_once():
    steer = make()
    steer.apply(0.3, 0.0, 0.0, at_rest=True, dt=DT)
    out = steer.apply(0.0, 0.3, 0.0, at_rest=True, dt=DT)
    assert direction_angle(out, (0.0, 0.3, 0.0)) == pytest.approx(HOLD, abs=1e-9)
    assert norm(out) == pytest.approx(0.3)      # a small lag costs no drive


def test_noise_does_not_cost_drive_but_a_sharp_turn_slows_down():
    steer = make()
    steer.apply(0.3, 0.0, 0.0, at_rest=True, dt=DT)
    out = steer.apply(0.3, 0.0, 0.3, at_rest=False, dt=DT)       # 20 deg off: full speed
    assert norm(out) == pytest.approx(norm((0.3, 0.0, 0.3)))
    out = steer.apply(0.0, 0.3, 0.0, at_rest=False, dt=DT)       # 90 deg off: nearly stopped
    assert norm(out) < 0.05


def test_exact_reversal_needs_no_steering():
    steer = make()
    steer.apply(0.3, 0.1, 0.2, at_rest=True, dt=DT)
    assert steer.apply(-0.3, -0.1, -0.2, at_rest=False, dt=DT) == pytest.approx((-0.3, -0.1, -0.2))


def test_rotation_in_place_ignores_translation_noise():
    steer = make()
    first = azimuths(steer.apply(0.0, 0.0, 0.6, at_rest=True, dt=DT))
    for vx in (0.005, -0.004, 0.006, -0.006):       # 0.006 / (0.359 * 0.6) = 1.6 deg
        assert azimuths(steer.apply(vx, 0.0, 0.6, at_rest=False, dt=DT)) == pytest.approx(first)


@pytest.mark.parametrize('kwargs', [
    dict(hold_angle=-0.01, max_rate=RATE, radius=RADIUS),
    dict(hold_angle=0.5, max_rate=RATE, radius=RADIUS),
    dict(hold_angle=math.nan, max_rate=RATE, radius=RADIUS),
    dict(hold_angle=HOLD, max_rate=0.0, radius=RADIUS),
    dict(hold_angle=HOLD, max_rate=math.inf, radius=RADIUS),
    dict(hold_angle=HOLD, max_rate=RATE, radius=0.0),
])
def test_bad_parameters_are_refused(kwargs):
    with pytest.raises(ValueError):
        SteerStabiliser(**kwargs)


# ------------------------------------------------------------------------- inside the governor
def governor() -> CommandGovernor:
    return CommandGovernor(LIMITS, make(), rest_linear=0.02, rest_angular=0.05)


def drive(gov, cmd, t, ticks):
    out = STOP_COMMAND
    for _ in range(ticks):
        gov.on_command(*cmd, t)
        out = gov.update(t)
        t += DT
    return out, t


def test_governor_stop_is_still_immediate():
    gov = governor()
    out, t = drive(gov, (0.4, 0.0, 0.3), 0.0, 60)
    assert out.enable and out.vx == pytest.approx(0.4, abs=0.01)
    gov.on_command(0.0, 0.0, 0.0, t)
    out = gov.update(t)
    assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)


def test_governor_steady_command_is_reached_exactly():
    out, _ = drive(governor(), (0.3, -0.2, 0.5), 0.0, 100)
    assert (out.vx, out.vy, out.wz) == pytest.approx((0.3, -0.2, 0.5))


def test_governor_turn_is_followed_within_the_hold_band():
    gov = governor()
    _, t = drive(gov, (0.4, 0.0, 0.0), 0.0, 50)
    out, _ = drive(gov, (0.4, 0.0, 1.0), t, 100)
    assert direction_angle((out.vx, out.vy, out.wz), (0.4, 0.0, 1.0)) <= HOLD + 1e-6


def test_governor_disarm_forgets_the_direction():
    gov = governor()
    _, t = drive(gov, (0.4, 0.0, 0.0), 0.0, 20)
    gov.on_e_stop(True)
    assert gov.update(t) == STOP_COMMAND and gov.state is ArmState.DISARMED
    gov.on_e_stop(False)
    out, _ = drive(gov, (0.0, 0.3, 0.0), t + DT, 40)
    assert (out.vx, out.vy) == pytest.approx((0.0, 0.3))

"""Unit tests for the ROS-free, phoenix6-free CommandGovernor of swerve_bridge.

These tests never touch phoenix6 or a ROS graph: they exercise pure functions only.
"""

import math
import random
import sys

import pytest

from swerve_navigation.swerve_bridge import ArmState
from swerve_navigation.swerve_bridge import clamp_abs
from swerve_navigation.swerve_bridge import clamp_norm
from swerve_navigation.swerve_bridge import CommandGovernor
from swerve_navigation.swerve_bridge import CommandVerdict
from swerve_navigation.swerve_bridge import GovernorLimits
from swerve_navigation.swerve_bridge import HARD_MAX_ANGULAR_SPEED
from swerve_navigation.swerve_bridge import HARD_MAX_LINEAR_SPEED
from swerve_navigation.swerve_bridge import MAX_CMD_TIMEOUT_S
from swerve_navigation.swerve_bridge import slew_speed_up_only
from swerve_navigation.swerve_bridge import STOP_COMMAND

DT = 0.02   # 50 Hz control tick
DBL_MAX = sys.float_info.max
# Timeout/period values below are exactly representable in binary so edge tests are exact.
LIMITS = GovernorLimits(max_linear_speed=1.0, max_angular_speed=1.5, max_linear_accel=2.0,
                        max_angular_accel=4.0, cmd_timeout=0.25)


def make(limits: GovernorLimits = LIMITS) -> CommandGovernor:
    return CommandGovernor(limits)


def arm(gov: CommandGovernor, vx=0.0, vy=0.0, wz=0.0, t0=0.0) -> float:
    """Feed one command and tick through PRIMED; returns the time of the next tick."""
    assert gov.on_command(vx, vy, wz, t0) is CommandVerdict.ACCEPTED
    assert gov.update(t0) == STOP_COMMAND
    assert gov.state is ArmState.PRIMED
    return t0 + DT


def run(gov: CommandGovernor, cmd, t_start: float, duration: float):
    """Keep re-sending ``cmd`` every tick for ``duration``; returns (last output, next time)."""
    out, t = STOP_COMMAND, t_start
    for _ in range(round(duration / DT)):
        gov.on_command(*cmd, t)
        out = gov.update(t)
        t += DT
    return out, t


# ------------------------------------------------------------------------------------ clamping
class TestClampNorm:

    def test_inside_limit_is_untouched(self):
        assert clamp_norm(0.3, -0.4, 1.0) == (0.3, -0.4)
        assert clamp_norm(0.0, 0.0, 1.0) == (0.0, 0.0)

    def test_norm_exactly_at_limit_is_untouched(self):
        assert clamp_norm(0.6, 0.8, 1.0) == (0.6, 0.8)

    def test_clamps_the_norm_not_the_components(self):
        # (0.9, 0.9) has each component below 1.0 but a norm of 1.27: must be scaled.
        x, y = clamp_norm(0.9, 0.9, 1.0)
        assert math.hypot(x, y) == pytest.approx(1.0)
        assert x == pytest.approx(y)

    @pytest.mark.parametrize('x,y', [(3.0, 4.0), (-3.0, 4.0), (3.0, -4.0), (-3.0, -4.0),
                                     (5.0, 0.0), (0.0, -5.0)])
    def test_direction_is_preserved(self, x, y):
        cx, cy = clamp_norm(x, y, 1.0)
        assert math.hypot(cx, cy) == pytest.approx(1.0)
        assert math.atan2(cy, cx) == pytest.approx(math.atan2(y, x))

    @pytest.mark.parametrize('x,y', [(1e308, 1e308), (-1.7e308, 1.7e308), (1e308, 0.0),
                                     (1e300, -1e-300), (DBL_MAX, DBL_MAX), (-DBL_MAX, DBL_MAX)])
    def test_huge_values_do_not_overflow(self, x, y):
        cx, cy = clamp_norm(x, y, 1.0)
        assert math.isfinite(cx) and math.isfinite(cy)
        assert math.hypot(cx, cy) == pytest.approx(1.0)
        assert math.copysign(1.0, cx) == math.copysign(1.0, x)

    def test_clamp_abs(self):
        assert clamp_abs(0.4, 1.5) == 0.4
        assert clamp_abs(9.0, 1.5) == 1.5
        assert clamp_abs(-9.0, 1.5) == -1.5
        assert clamp_abs(1e308, 1.5) == 1.5


# ------------------------------------------------------------------------------------- slewing
class TestSlewSpeedUpOnly:

    def test_speed_up_from_rest_is_limited(self):
        assert slew_speed_up_only((0.0, 0.0), (1.0, 0.0), 0.04) == pytest.approx((0.04, 0.0))

    def test_target_within_step_is_reached(self):
        assert slew_speed_up_only((0.0, 0.0), (0.03, 0.0), 0.04) == pytest.approx((0.03, 0.0))

    def test_stop_is_immediate(self):
        assert slew_speed_up_only((1.0, 0.0), (0.0, 0.0), 0.04) == pytest.approx((0.0, 0.0))
        assert slew_speed_up_only((-0.7, 0.7), (0.0, 0.0), 0.0) == pytest.approx((0.0, 0.0))

    def test_slow_down_in_same_direction_is_immediate(self):
        assert slew_speed_up_only((1.0, 0.0), (0.2, 0.0), 0.04) == pytest.approx((0.2, 0.0))
        assert slew_speed_up_only((0.6, 0.8), (0.3, 0.4), 0.0) == pytest.approx((0.3, 0.4))

    def test_reversal_drops_to_zero_then_ramps_the_other_way(self):
        assert slew_speed_up_only((1.0, 0.0), (-1.0, 0.0), 0.04) == pytest.approx((-0.04, 0.0))

    def test_perpendicular_change_stops_old_motion_and_ramps_new(self):
        assert slew_speed_up_only((1.0, 0.0), (0.0, 1.0), 0.04) == pytest.approx((0.0, 0.04))

    def test_small_direction_change_at_speed_is_not_slowed(self):
        # Field-centric teleop while rotating: the robot-frame vector turns a little per tick.
        angle = 1.5 * DT
        target = (math.cos(angle), math.sin(angle))
        assert slew_speed_up_only((1.0, 0.0), target, 2.0 * DT) == pytest.approx(target)

    def test_zero_step_never_increases_speed(self):
        assert slew_speed_up_only((0.5, 0.0), (1.0, 0.0), 0.0) == pytest.approx((0.5, 0.0))

    def test_one_dimensional(self):
        assert slew_speed_up_only((0.0,), (1.5,), 0.08) == pytest.approx((0.08,))
        assert slew_speed_up_only((1.0,), (0.5,), 0.08) == pytest.approx((0.5,))
        assert slew_speed_up_only((1.0,), (-1.0,), 0.08) == pytest.approx((-0.08,))
        assert slew_speed_up_only((-1.0,), (0.0,), 0.08) == pytest.approx((0.0,))

    def test_denormal_previous_value_is_harmless(self):
        out = slew_speed_up_only((5e-324, 0.0), (1.0, 0.0), 0.04)
        assert all(math.isfinite(v) for v in out)
        assert out == pytest.approx((0.04, 0.0))

    def test_result_never_faster_than_both_endpoints(self):
        rng = random.Random(1)
        for _ in range(2000):
            prev = (rng.uniform(-1, 1), rng.uniform(-1, 1))
            target = (rng.uniform(-1, 1), rng.uniform(-1, 1))
            out = slew_speed_up_only(prev, target, rng.uniform(0.0, 0.5))
            assert math.hypot(*out) <= max(math.hypot(*prev), math.hypot(*target)) + 1e-12


# -------------------------------------------------------------------------------------- limits
class TestGovernorLimits:

    @pytest.mark.parametrize('field', ['max_linear_speed', 'max_angular_speed',
                                       'max_linear_accel', 'max_angular_accel', 'cmd_timeout',
                                       'max_dt'])
    @pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf, -math.inf, 1, '1.0', None])
    def test_rejects_bad_values(self, field, bad):
        kwargs = {'max_linear_speed': 1.0, 'max_angular_speed': 1.5, 'max_linear_accel': 2.0,
                  'max_angular_accel': 4.0, 'cmd_timeout': 0.3}
        kwargs[field] = bad
        with pytest.raises(ValueError):
            GovernorLimits(**kwargs)

    def test_hard_ceilings(self):
        ok = {'max_linear_speed': HARD_MAX_LINEAR_SPEED,
              'max_angular_speed': HARD_MAX_ANGULAR_SPEED, 'max_linear_accel': 2.0,
              'max_angular_accel': 4.0, 'cmd_timeout': MAX_CMD_TIMEOUT_S}
        GovernorLimits(**ok)
        for field, value in (('max_linear_speed', HARD_MAX_LINEAR_SPEED + 0.01),
                             ('max_angular_speed', HARD_MAX_ANGULAR_SPEED + 0.01),
                             ('cmd_timeout', MAX_CMD_TIMEOUT_S + 0.01),
                             ('max_linear_speed', 10.0)):
            with pytest.raises(ValueError):
                GovernorLimits(**{**ok, field: value})


# ------------------------------------------------------------------------------- state machine
class TestArming:

    def test_without_any_command_it_stays_disarmed(self):
        gov = make()
        for i in range(10):
            assert gov.update(i * DT) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED
        assert gov.command_age(1.0) is None

    def test_first_tick_after_a_command_is_zero_without_enable(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 0.0)
        assert gov.update(0.0) == STOP_COMMAND          # zero BEFORE enable is ever fed
        assert gov.state is ArmState.PRIMED
        out = gov.update(DT)
        assert gov.state is ArmState.ARMED
        assert out.enable
        assert out.vx == pytest.approx(2.0 * DT)        # ramp starts from rest
        assert (out.vy, out.wz) == (0.0, 0.0)

    def test_zero_command_arms_with_zero_velocity(self):
        gov = make()
        t = arm(gov)
        out = gov.update(t)
        assert out.enable and (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)

    def test_ramps_to_target_and_holds(self):
        gov = make()
        t = arm(gov, 0.3, 0.0, 0.0)
        outputs = []
        for _ in range(12):
            gov.on_command(0.3, 0.0, 0.0, t)
            outputs.append(gov.update(t).vx)
            t += DT
        assert outputs[:3] == pytest.approx([0.04, 0.08, 0.12])
        assert outputs[-1] == pytest.approx(0.3)
        assert all(b >= a for a, b in zip(outputs, outputs[1:]))

    def test_angular_ramp(self):
        gov = make()
        t = arm(gov, 0.0, 0.0, -1.0)
        assert gov.update(t).wz == pytest.approx(-4.0 * DT)

    def test_decrease_and_zero_command_are_immediate(self):
        gov = make()
        out, t = run(gov, (1.0, 0.0, 1.5), arm(gov, 1.0, 0.0, 1.5), 1.0)
        assert (out.vx, out.wz) == pytest.approx((1.0, 1.5))
        gov.on_command(0.2, 0.0, 0.5, t)
        out = gov.update(t)
        assert (out.vx, out.wz) == pytest.approx((0.2, 0.5))
        gov.on_command(0.0, 0.0, 0.0, t + DT)
        out = gov.update(t + DT)
        assert out.enable and (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)

    def test_clamps_norm_and_angular_while_armed(self):
        gov = make()
        out, _ = run(gov, (3.0, 4.0, -9.0), arm(gov, 3.0, 4.0, -9.0), 2.0)
        assert math.hypot(out.vx, out.vy) == pytest.approx(1.0)
        assert (out.vx, out.vy) == pytest.approx((0.6, 0.8))
        assert out.wz == pytest.approx(-1.5)

    def test_huge_finite_command_is_clamped_not_rejected(self):
        gov = make()
        out, _ = run(gov, (1e308, -1e308, 1e308), arm(gov, 1e308, -1e308, 1e308), 2.0)
        assert out.enable
        assert math.hypot(out.vx, out.vy) == pytest.approx(1.0)
        assert out.vx > 0 > out.vy
        assert out.wz == pytest.approx(1.5)

    def test_long_gap_between_ticks_is_one_bounded_step(self):
        gov = make()
        t = arm(gov, 1.0, 0.0, 0.0)
        gov.on_command(1.0, 0.0, 0.0, t + 5.0)
        out = gov.update(t + 5.0)     # 5 s stall, fresh command: step limited by max_dt (0.1 s)
        assert out.vx == pytest.approx(2.0 * LIMITS.max_dt)

    def test_inhibit_blocks_arming_and_disarms(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 0.0)
        assert gov.update(0.0, inhibit=True) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED
        assert gov.update(DT) == STOP_COMMAND and gov.state is ArmState.PRIMED
        assert gov.update(2 * DT).enable
        assert gov.update(3 * DT, inhibit=True) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED

    def test_fault_needs_a_new_command_and_a_new_zero(self):
        gov = make()
        out, t = run(gov, (0.5, 0.0, 0.0), arm(gov, 0.5), 0.2)
        assert out.enable
        gov.fault()
        assert gov.state is ArmState.DISARMED
        assert gov.update(t) == STOP_COMMAND                      # old command is gone
        assert gov.state is ArmState.DISARMED
        gov.on_command(0.5, 0.0, 0.0, t + DT)
        assert gov.update(t + DT) == STOP_COMMAND                 # zero before re-enable
        assert gov.update(t + 2 * DT).vx == pytest.approx(2.0 * DT)   # and the ramp restarts

    def test_non_finite_clock_disarms(self):
        gov = make()
        out, t = run(gov, (0.5, 0.0, 0.0), arm(gov, 0.5), 0.2)
        assert out.enable
        assert gov.update(math.nan) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED
        assert gov.update(t) == STOP_COMMAND


class TestCommandTimeout:

    def test_armed_until_just_before_the_timeout(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 0.0)
        gov.update(0.0)
        assert gov.update(0.125).enable
        assert gov.update(0.25 - 1e-9).enable

    def test_disarmed_exactly_at_the_timeout(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 0.0)
        gov.update(0.0)
        assert gov.update(0.125).enable
        assert gov.update(0.25) == STOP_COMMAND         # "newer than cmd_timeout" is strict
        assert gov.state is ArmState.DISARMED

    def test_silence_stops_within_the_timeout_plus_one_tick(self):
        gov = make()
        out, t = run(gov, (0.5, 0.0, 0.0), arm(gov, 0.5), 1.0)
        last_cmd = t - DT
        assert out.enable
        while gov.update(t).enable:
            t += DT
        assert LIMITS.cmd_timeout <= t - last_cmd <= LIMITS.cmd_timeout + DT + 1e-9

    def test_stale_command_never_revives(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 0.0)
        gov.update(0.0)
        assert gov.update(1.0) == STOP_COMMAND
        assert gov.command_age(1.0) is None
        # Even if a (non-monotonic) clock went back into the freshness window.
        assert gov.update(0.1) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED

    def test_command_from_the_future_is_not_fresh(self):
        gov = make()
        gov.on_command(0.5, 0.0, 0.0, 10.0)
        assert gov.update(9.0) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED

    def test_rearming_after_timeout_sends_zero_first_and_ramps_from_rest(self):
        gov = make()
        out, t = run(gov, (1.0, 0.0, 0.0), arm(gov, 1.0), 1.0)
        assert out.vx == pytest.approx(1.0)
        t += 1.0
        assert gov.update(t) == STOP_COMMAND
        gov.on_command(1.0, 0.0, 0.0, t + DT)
        assert gov.update(t + DT) == STOP_COMMAND
        assert gov.state is ArmState.PRIMED
        assert gov.update(t + 2 * DT).vx == pytest.approx(2.0 * DT)

    def test_command_age(self):
        gov = make()
        gov.on_command(0.0, 0.0, 0.0, 2.0)
        assert gov.command_age(2.1) == pytest.approx(0.1)


class TestEStop:

    def test_e_stop_stops_immediately(self):
        gov = make()
        out, t = run(gov, (1.0, 0.0, 1.0), arm(gov, 1.0, 0.0, 1.0), 1.0)
        assert out.enable
        gov.on_e_stop(True)
        assert gov.state is ArmState.DISARMED            # before the next tick even runs
        assert gov.update(t) == STOP_COMMAND

    def test_commands_during_e_stop_are_ignored_and_it_stays_stopped(self):
        gov = make()
        gov.on_e_stop(True)
        t = 0.0
        for _ in range(100):
            assert gov.on_command(1.0, 0.0, 0.0, t) is CommandVerdict.IGNORED_E_STOP
            assert gov.update(t) == STOP_COMMAND
            assert gov.state is ArmState.DISARMED
            t += DT
        assert gov.e_stop

    def test_e_stop_latches_until_released(self):
        gov = make()
        gov.on_e_stop(True)
        gov.on_e_stop(True)
        assert gov.e_stop
        gov.on_e_stop(False)
        assert not gov.e_stop

    def test_release_alone_does_not_move(self):
        gov = make()
        out, t = run(gov, (1.0, 0.0, 0.0), arm(gov, 1.0), 0.5)
        assert out.enable
        gov.on_e_stop(True)
        gov.on_command(1.0, 0.0, 0.0, t)      # sent while stopped
        gov.on_e_stop(False)                  # quick toggle, the old commands are < cmd_timeout
        for i in range(5):
            assert gov.update(t + i * DT) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED

    def test_release_plus_fresh_command_moves_again_from_rest(self):
        gov = make()
        _, t = run(gov, (1.0, 0.0, 0.0), arm(gov, 1.0), 0.5)
        gov.on_e_stop(True)
        gov.update(t)
        gov.on_e_stop(False)
        assert gov.on_command(1.0, 0.0, 0.0, t + DT) is CommandVerdict.ACCEPTED
        assert gov.update(t + DT) == STOP_COMMAND                    # zero before re-enable
        out = gov.update(t + 2 * DT)
        assert out.enable and out.vx == pytest.approx(2.0 * DT)


class TestNonFiniteCommands:

    BAD = [math.nan, math.inf, -math.inf]

    @pytest.mark.parametrize('bad', BAD)
    @pytest.mark.parametrize('index', [0, 1, 2])
    def test_rejected_and_disarmed(self, bad, index):
        gov = make()
        out, t = run(gov, (0.5, 0.0, 0.0), arm(gov, 0.5), 0.5)
        assert out.enable
        cmd = [0.5, 0.0, 0.0]
        cmd[index] = bad
        assert gov.on_command(*cmd, t) is CommandVerdict.REJECTED_NON_FINITE
        assert gov.state is ArmState.DISARMED
        assert gov.update(t) == STOP_COMMAND       # the previous good command is gone as well
        assert gov.rejected_commands == 1

    def test_non_finite_receive_time_is_rejected(self):
        gov = make()
        assert gov.on_command(0.5, 0.0, 0.0, math.nan) is CommandVerdict.REJECTED_NON_FINITE
        assert gov.update(0.0) == STOP_COMMAND

    def test_stream_of_nan_never_enables(self):
        gov = make()
        t = 0.0
        for _ in range(50):
            gov.on_command(math.nan, math.nan, math.nan, t)
            assert gov.update(t) == STOP_COMMAND
            t += DT
        assert gov.rejected_commands == 50

    def test_recovers_through_primed_after_a_good_command(self):
        gov = make()
        gov.on_command(math.inf, 0.0, 0.0, 0.0)
        gov.update(0.0)
        gov.on_command(0.5, 0.0, 0.0, DT)
        assert gov.update(DT) == STOP_COMMAND and gov.state is ArmState.PRIMED
        assert gov.update(2 * DT).enable

    def test_alternating_nan_and_good_commands_never_move(self):
        gov = make()
        t = 0.0
        for i in range(100):
            gov.on_command(math.nan if i % 2 else 0.5, 0.0, 0.0, t)
            out = gov.update(t)
            assert not out.enable      # never gets past PRIMED
            t += DT


class TestInvariants:

    def test_random_inputs_never_violate_the_limits(self):
        rng = random.Random(7)
        gov = make()
        t, prev = 0.0, STOP_COMMAND
        choices = [0.0, 0.3, -0.3, 5.0, -5.0, 1e308, -1e308, math.nan, math.inf]
        for _ in range(20000):
            t += rng.choice([0.0, DT, DT, DT, 0.2, 0.3, 2.0])
            action = rng.random()
            if action < 0.6:
                gov.on_command(rng.choice(choices), rng.choice(choices), rng.choice(choices), t)
            elif action < 0.65:
                gov.on_e_stop(rng.random() < 0.5)
            elif action < 0.67:
                gov.fault()
            was_disarmed = gov.state is ArmState.DISARMED
            out = gov.update(t, inhibit=rng.random() < 0.05)

            assert all(math.isfinite(v) for v in (out.vx, out.vy, out.wz))
            assert math.hypot(out.vx, out.vy) <= LIMITS.max_linear_speed + 1e-9
            assert abs(out.wz) <= LIMITS.max_angular_speed + 1e-9
            if not out.enable:
                assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)
            else:
                assert not gov.e_stop
                assert not was_disarmed, 'enabled without a zero-only tick in between'
                speed_up = math.hypot(out.vx, out.vy) - math.hypot(prev.vx, prev.vy)
                assert speed_up <= LIMITS.max_linear_accel * LIMITS.max_dt + 1e-9
                spin_up = abs(out.wz) - abs(prev.wz)
                assert spin_up <= LIMITS.max_angular_accel * LIMITS.max_dt + 1e-9
            prev = out

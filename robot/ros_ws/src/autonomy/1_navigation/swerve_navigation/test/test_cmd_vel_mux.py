"""Unit tests of the ROS-free cmd_vel mux logic (no node, no DDS traffic)."""

import math

import pytest

from swerve_navigation.cmd_vel_mux import (
    checked_rate,
    ESTOP,
    IDLE,
    MuxConfig,
    MuxLogic,
    MuxOutput,
    NAV,
    sanitise,
    TELEOP,
    ZERO,
)

NAN = float('nan')
INF = float('inf')
TELEOP_CMD = (0.3, -0.1, 0.5)
NAV_CMD = (0.4, 0.0, -1.0)


def make(**overrides):
    """Mux that has already received /e_stop = false (what joy_teleop publishes at startup)."""
    logic = MuxLogic(MuxConfig(**overrides))
    logic.set_e_stop(False)
    return logic


# ---------------------------------------------------------------- unknown e-stop state

def test_nothing_is_forwarded_while_the_estop_state_is_unknown():
    # A (re)started mux gets a latched /e_stop only after DDS discovery, later than the first
    # nav messages: until then neither source may reach /cmd_vel.
    logic = MuxLogic(MuxConfig())
    assert not logic.e_stop_known
    for step in range(150):  # 0 .. 4.97 s
        now = 100.0 + step / 30.0
        logic.on_nav(NAV_CMD, now)
        logic.on_teleop(TELEOP_CMD, now)
        assert logic.tick(now) == MuxOutput(IDLE, None)


def test_a_late_latched_estop_never_lets_a_command_through():
    logic = MuxLogic(MuxConfig())
    outputs = []
    for step in range(90):
        now = step / 30.0
        logic.on_nav(NAV_CMD, now)
        if step == 45:
            logic.set_e_stop(True)  # the latched sample arrives 1.5 s after the first tick
        outputs.append(logic.tick(now))
    assert outputs[:45] == [MuxOutput(IDLE, None)] * 45
    assert outputs[45:] == [MuxOutput(ESTOP, ZERO)] * 45


def test_estop_false_ends_the_wait_at_once():
    logic = MuxLogic(MuxConfig())
    logic.on_nav(NAV_CMD, 0.0)
    assert logic.tick(0.0) == MuxOutput(IDLE, None)
    logic.set_e_stop(False)
    assert logic.e_stop_known
    assert logic.tick(0.01) == MuxOutput(NAV, NAV_CMD)


def test_without_any_estop_publisher_the_wait_ends_after_estop_wait():
    # joy:=false launches have no /e_stop publisher at all.
    logic = MuxLogic(MuxConfig())
    assert logic.tick(50.0) == MuxOutput(IDLE, None)  # the first tick starts the wait
    logic.on_nav(NAV_CMD, 54.99)
    assert logic.tick(54.99) == MuxOutput(IDLE, None)
    logic.on_nav(NAV_CMD, 55.0)
    assert logic.tick(55.0) == MuxOutput(NAV, NAV_CMD)
    assert not logic.e_stop_known


def test_the_wait_is_measured_from_the_first_tick_not_from_time_zero():
    logic = MuxLogic(MuxConfig(estop_wait=0.5))
    logic.on_teleop(TELEOP_CMD, 1000.0)
    assert logic.tick(1000.0) == MuxOutput(IDLE, None)
    logic.on_teleop(TELEOP_CMD, 1000.5)
    assert logic.tick(1000.5) == MuxOutput(TELEOP, TELEOP_CMD)


# ---------------------------------------------------------------- selection

def test_silent_at_startup():
    logic = make()
    for step in range(100):
        assert logic.tick(step / 30.0) == MuxOutput(IDLE, None)


def test_teleop_is_forwarded_while_fresh():
    logic = make()
    logic.on_teleop(TELEOP_CMD, 10.0)
    assert logic.tick(10.0) == MuxOutput(TELEOP, TELEOP_CMD)
    assert logic.tick(10.5) == MuxOutput(TELEOP, TELEOP_CMD)  # exactly teleop_timeout
    assert logic.tick(10.51) == MuxOutput(IDLE, ZERO)         # crashed teleop node: stop


def test_nav_is_forwarded_while_fresh():
    logic = make()
    logic.on_nav(NAV_CMD, 10.0)
    assert logic.tick(10.0) == MuxOutput(NAV, NAV_CMD)
    assert logic.tick(10.5) == MuxOutput(NAV, NAV_CMD)
    assert logic.tick(10.51) == MuxOutput(IDLE, ZERO)


def test_timeouts_are_per_source():
    logic = make(teleop_timeout=0.2, nav_timeout=2.0, teleop_hold=0.0)
    logic.on_teleop(TELEOP_CMD, 0.0)
    logic.on_nav(NAV_CMD, 0.0)
    assert logic.tick(0.2).source == TELEOP
    assert logic.tick(0.21).source == NAV
    assert logic.tick(2.0).source == NAV
    assert logic.tick(2.01).source == IDLE


def test_teleop_beats_nav():
    logic = make()
    for step in range(30):
        now = step / 30.0
        logic.on_nav(NAV_CMD, now)
        logic.on_teleop(TELEOP_CMD, now)
        assert logic.tick(now) == MuxOutput(TELEOP, TELEOP_CMD)


def test_a_zero_from_teleop_still_beats_nav():
    logic = make()
    logic.on_nav(NAV_CMD, 0.0)
    logic.on_teleop(ZERO, 0.0)  # deadman held with the sticks centred = "hold still"
    assert logic.tick(0.0) == MuxOutput(TELEOP, ZERO)


def test_nav_is_locked_out_during_teleop_hold():
    logic = make()
    logic.on_teleop(ZERO, 0.0)  # the single zero joy_teleop sends on deadman release
    outputs = {}
    for step in range(0, 46):
        now = step / 30.0
        logic.on_nav(NAV_CMD, now)
        outputs[step] = logic.tick(now)
    assert all(outputs[step] == MuxOutput(TELEOP, ZERO) for step in range(0, 16))   # <= 0.5 s
    assert all(outputs[step] == MuxOutput(IDLE, ZERO) for step in range(16, 30))    # hold
    assert all(outputs[step] == MuxOutput(NAV, NAV_CMD) for step in range(30, 46))  # >= 1.0 s


def test_teleop_hold_restarts_with_every_teleop_message():
    logic = make()
    logic.on_teleop(TELEOP_CMD, 0.0)
    logic.on_teleop(ZERO, 3.0)
    assert logic.tick(3.0) == MuxOutput(TELEOP, ZERO)
    logic.on_nav(NAV_CMD, 3.9)
    assert logic.tick(3.9) == MuxOutput(IDLE, ZERO)  # 3.9 s after the first, 0.9 s after the last
    logic.on_nav(NAV_CMD, 4.0)
    assert logic.tick(4.0) == MuxOutput(NAV, NAV_CMD)


def test_long_teleop_hold_keeps_nav_out_even_when_silent():
    logic = make(teleop_hold=5.0)
    logic.on_teleop(ZERO, 0.0)
    logic.on_nav(NAV_CMD, 3.0)
    # teleop timed out at 0.5 s, the idle zeros ended at 1.5 s, nav is still locked out.
    assert logic.tick(3.0) == MuxOutput(IDLE, None)
    logic.on_nav(NAV_CMD, 5.0)
    assert logic.tick(5.0) == MuxOutput(NAV, NAV_CMD)


# ---------------------------------------------------------------- e-stop

def test_estop_forces_zero_over_everything():
    logic = make()
    logic.set_e_stop(True)
    for step in range(90):
        now = step / 30.0
        logic.on_teleop(TELEOP_CMD, now)
        logic.on_nav(NAV_CMD, now)
        assert logic.tick(now) == MuxOutput(ESTOP, ZERO)


def test_estop_keeps_publishing_zero_without_any_source():
    logic = make()
    logic.set_e_stop(True)
    assert logic.tick(0.0) == MuxOutput(ESTOP, ZERO)
    assert logic.tick(1000.0) == MuxOutput(ESTOP, ZERO)


def test_estop_release_returns_to_normal_selection():
    logic = make()
    logic.set_e_stop(True)
    logic.on_nav(NAV_CMD, 0.0)
    assert logic.tick(0.0).source == ESTOP
    logic.set_e_stop(False)
    assert logic.tick(0.1) == MuxOutput(NAV, NAV_CMD)
    assert logic.tick(0.6) == MuxOutput(IDLE, ZERO)  # 1 s of zeros after the last active tick
    assert logic.tick(1.2) == MuxOutput(IDLE, None)


# ---------------------------------------------------------------- going silent

def test_zeros_for_one_second_after_the_last_active_tick_then_silence():
    logic = make()
    logic.on_nav(NAV_CMD, 0.0)
    assert logic.tick(0.5) == MuxOutput(NAV, NAV_CMD)  # last active tick
    assert logic.tick(0.6) == MuxOutput(IDLE, ZERO)
    assert logic.tick(1.5) == MuxOutput(IDLE, ZERO)    # exactly 1.0 s later
    assert logic.tick(1.51) == MuxOutput(IDLE, None)
    assert logic.tick(100.0) == MuxOutput(IDLE, None)


def test_silence_ends_when_a_source_returns():
    logic = make()
    logic.on_nav(NAV_CMD, 0.0)
    logic.tick(0.0)
    assert logic.tick(50.0).command is None
    logic.on_nav(NAV_CMD, 60.0)
    assert logic.tick(60.0) == MuxOutput(NAV, NAV_CMD)


def test_30_hz_timeline_of_a_teleop_session():
    """Nav drives, the operator takes over and lets go, nav resumes, nav finishes, silence."""
    logic = make()
    timeline = []
    for step in range(0, 300):
        now = step / 30.0
        if now < 8.0:
            logic.on_nav(NAV_CMD, now)             # Nav2 publishes for the first 8 s
        if 2.0 <= now < 4.0:
            logic.on_teleop(TELEOP_CMD, now)       # deadman held from 2 s to 4 s
        if step == 120:
            logic.on_teleop(ZERO, now)             # the single zero on release (t = 4 s)
        timeline.append((now, logic.tick(now)))

    def at(t0, t1):
        return [out for now, out in timeline if t0 <= now < t1]

    assert all(out == MuxOutput(NAV, NAV_CMD) for out in at(0.0, 2.0))
    assert all(out == MuxOutput(TELEOP, TELEOP_CMD) for out in at(2.0, 4.0))
    assert all(out.command == ZERO for out in at(4.0, 5.0))        # zero + hold
    assert all(out == MuxOutput(NAV, NAV_CMD) for out in at(5.0, 8.0))
    assert all(out == MuxOutput(NAV, NAV_CMD) for out in at(8.0, 8.45))   # last msg still fresh
    assert all(out == MuxOutput(IDLE, ZERO) for out in at(8.55, 9.40))
    assert all(out == MuxOutput(IDLE, None) for out in at(9.55, 10.0))


# ---------------------------------------------------------------- non-finite input

@pytest.mark.parametrize('bad', [NAN, INF, -INF])
@pytest.mark.parametrize('index', [0, 1, 2])
def test_sanitise_rejects_non_finite(index, bad):
    command = [0.1, 0.2, 0.3]
    command[index] = bad
    assert sanitise(tuple(command)) == (ZERO, False)


def test_sanitise_passes_finite_commands_unchanged():
    assert sanitise(NAV_CMD) == (NAV_CMD, True)


def test_non_finite_nav_becomes_a_stop():
    logic = make()
    assert logic.on_nav(NAV_CMD, 0.0)
    assert not logic.on_nav((NAN, 0.0, 0.0), 0.1)
    assert logic.tick(0.1) == MuxOutput(NAV, ZERO)


def test_non_finite_teleop_becomes_a_stop_and_still_blocks_nav():
    logic = make()
    logic.on_nav(NAV_CMD, 0.0)
    assert not logic.on_teleop((0.1, INF, 0.0), 0.0)
    assert logic.tick(0.0) == MuxOutput(TELEOP, ZERO)


# ---------------------------------------------------------------- configuration

def test_defaults_match_the_contract():
    config = MuxConfig()
    assert (config.teleop_timeout, config.nav_timeout) == (0.5, 0.5)
    assert config.teleop_hold == 1.0
    assert config.idle_zero_period == 1.0
    assert config.estop_wait == 5.0


@pytest.mark.parametrize('overrides', [
    {'teleop_timeout': 0.0}, {'nav_timeout': -1.0}, {'teleop_timeout': NAN},
    {'teleop_hold': -0.1}, {'idle_zero_period': NAN}, {'estop_wait': -1.0},
    # `.inf` is valid YAML: the mux would repeat the last command of a dead source for ever.
    {'teleop_timeout': INF}, {'nav_timeout': INF}, {'nav_timeout': 5.01},
    {'teleop_hold': INF}, {'idle_zero_period': INF}, {'estop_wait': INF},
])
def test_invalid_config_is_rejected(overrides):
    with pytest.raises(ValueError):
        MuxConfig(**overrides)


def test_rate_limits():
    assert checked_rate(30.0) == 30.0
    assert checked_rate(5.0) == 5.0
    # 2 Hz: the bridge watchdog (0.3 s) would disarm the drive between two messages.
    for bad in (0.0, -30.0, 2.0, 200.1, INF, NAN):
        with pytest.raises(ValueError):
            checked_rate(bad)


# ------------------------------------------------------------- nav stiction compensation
class TestNavMinSpeed:

    def test_small_nav_command_is_lifted_without_changing_its_direction(self):
        from swerve_navigation.cmd_vel_mux import lift_to_min_speed
        out = lift_to_min_speed((0.03, -0.04, 0.0), 0.12, 0.03)
        assert math.hypot(out[0], out[1]) == pytest.approx(0.12)
        assert out[1] / out[0] == pytest.approx(-0.04 / 0.03)
        out = lift_to_min_speed((0.0, 0.0, 0.1), 0.12, 0.03)
        assert out == pytest.approx((0.0, 0.0, 0.12 / 0.359))

    def test_noise_becomes_zero_and_normal_commands_pass(self):
        from swerve_navigation.cmd_vel_mux import lift_to_min_speed
        assert lift_to_min_speed((0.01, 0.01, 0.02), 0.12, 0.03) == (0.0, 0.0, 0.0)
        assert lift_to_min_speed((0.0, 0.0, 0.0), 0.12, 0.03) == (0.0, 0.0, 0.0)
        assert lift_to_min_speed((0.3, 0.1, -0.5), 0.12, 0.03) == (0.3, 0.1, -0.5)
        assert lift_to_min_speed((0.01, 0.0, 0.0), 0.0, 0.0) == (0.01, 0.0, 0.0)   # disabled

    def test_only_nav_is_lifted_teleop_keeps_fine_control(self):
        from swerve_navigation.cmd_vel_mux import MuxConfig, MuxLogic
        logic = MuxLogic(MuxConfig(nav_min_speed=0.12, nav_zero_below=0.03, estop_wait=0.0))
        logic.set_e_stop(False)
        logic.on_nav((0.05, 0.0, 0.0), 0.0)
        assert logic.tick(0.0).command == pytest.approx((0.12, 0.0, 0.0))
        logic.on_teleop((0.05, 0.0, 0.0), 0.1)
        assert logic.tick(0.1).command == (0.05, 0.0, 0.0)

    def test_bad_floor_is_refused(self):
        from swerve_navigation.cmd_vel_mux import MuxConfig
        for kwargs in (dict(nav_min_speed=0.5), dict(nav_min_speed=0.1, nav_zero_below=0.2),
                       dict(nav_min_speed=-0.1)):
            with pytest.raises(ValueError):
                MuxConfig(**kwargs)

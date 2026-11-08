"""Unit tests for swerve_bridge.ControlLoop (fault policy) and Drivetrain (native call order).

Contract: on ANY exception in the control path -> zero request, stop feeding enable, log, and
keep trying to stay safe; shutdown = zero request -> feed_enable(0) -> stop sim -> close.
Everything runs against fakes: no phoenix6, no ROS graph.
"""

import math

import pytest

from swerve_navigation.swerve_bridge import ArmState
from swerve_navigation.swerve_bridge import CommandGovernor
from swerve_navigation.swerve_bridge import ControlLoop
from swerve_navigation.swerve_bridge import Drivetrain
from swerve_navigation.swerve_bridge import ENABLE_FEED_TIMEOUT_S
from swerve_navigation.swerve_bridge import GovernorLimits

DT = 0.02
LIMITS = GovernorLimits(max_linear_speed=1.0, max_angular_speed=1.5, max_linear_accel=2.0,
                        max_angular_accel=4.0, cmd_timeout=0.25)


class FakeDrivetrain:
    """Records calls.  drive() == request + enable feed, stop() == zero request, no feed."""

    def __init__(self) -> None:
        self.calls = []
        self.fail_drive = False
        self.fail_stop = False

    def drive(self, vx, vy, wz) -> None:
        if self.fail_drive:
            raise RuntimeError('drive failed')
        self.calls.append(('drive', vx, vy, wz))

    def stop(self) -> None:
        if self.fail_stop:
            raise RuntimeError('stop failed')
        self.calls.append(('stop',))

    def kinds(self):
        return [c[0] for c in self.calls]


@pytest.fixture
def rig():
    governor, drivetrain, faults = CommandGovernor(LIMITS), FakeDrivetrain(), []
    return governor, drivetrain, faults, ControlLoop(governor, drivetrain, faults.append)


def feed(governor, loop, ticks, t=0.0, cmd=(0.5, 0.0, 0.0), inhibit=False):
    for _ in range(ticks):
        governor.on_command(*cmd, t)
        loop.step(t, inhibit)
        t += DT
    return t


def test_idle_sends_a_zero_request_every_tick_and_never_enables(rig):
    governor, drivetrain, faults, loop = rig
    for i in range(5):
        loop.step(i * DT, False)
    assert drivetrain.kinds() == ['stop'] * 5
    assert not faults and loop.fault_count == 0


def test_zero_request_precedes_the_first_enabled_request(rig):
    governor, drivetrain, _, loop = rig
    feed(governor, loop, 3)
    assert drivetrain.kinds() == ['stop', 'drive', 'drive']
    assert drivetrain.calls[1][1] == pytest.approx(2.0 * DT)


def test_inhibit_keeps_sending_zero(rig):
    governor, drivetrain, _, loop = rig
    feed(governor, loop, 4, inhibit=True)
    assert drivetrain.kinds() == ['stop'] * 4


def test_drive_exception_zeroes_disarms_and_reports(rig):
    governor, drivetrain, faults, loop = rig
    t = feed(governor, loop, 3)
    drivetrain.fail_drive = True
    governor.on_command(0.5, 0.0, 0.0, t)
    loop.step(t, False)                                # must not raise
    assert drivetrain.kinds()[-1] == 'stop'            # zero request latched, enable not fed
    assert governor.state is ArmState.DISARMED
    assert loop.fault_count == 1 and 'drive failed' in loop.last_fault
    assert len(faults) == 1 and 'CONTROL FAULT #1' in faults[0]


def test_after_a_fault_the_old_command_is_gone(rig):
    governor, drivetrain, _, loop = rig
    t = feed(governor, loop, 3)
    drivetrain.fail_drive = True
    governor.on_command(0.5, 0.0, 0.0, t)
    loop.step(t, False)
    drivetrain.fail_drive = False
    before = len(drivetrain.calls)
    for i in range(1, 4):                              # command still "fresh" by age, but dropped
        loop.step(t + i * DT, False)
    assert drivetrain.kinds()[before:] == ['stop'] * 3


def test_recovery_needs_a_new_command_and_a_new_zero_only_tick(rig):
    governor, drivetrain, _, loop = rig
    t = feed(governor, loop, 3)
    drivetrain.fail_drive = True
    t = feed(governor, loop, 1, t)
    drivetrain.fail_drive = False
    before = len(drivetrain.calls)
    feed(governor, loop, 3, t)
    assert drivetrain.kinds()[before:] == ['stop', 'drive', 'drive']
    assert drivetrain.calls[before + 1][1] == pytest.approx(2.0 * DT)   # ramp restarts from rest


def test_persistent_drive_failure_never_gets_a_drive_call_through(rig):
    governor, drivetrain, faults, loop = rig
    drivetrain.fail_drive = True
    feed(governor, loop, 100)
    assert 'drive' not in drivetrain.kinds()
    assert loop.fault_count == len(faults) >= 30       # it kept trying and kept reporting


def test_zero_request_failing_too_does_not_raise_and_is_retried(rig):
    governor, drivetrain, faults, loop = rig
    t = feed(governor, loop, 3)
    drivetrain.fail_drive = drivetrain.fail_stop = True
    t = feed(governor, loop, 5, t)                     # must not raise
    assert 'zero request failed as well' in faults[-1]
    assert governor.state is ArmState.DISARMED
    calls_while_broken = len(drivetrain.calls)
    drivetrain.fail_drive = drivetrain.fail_stop = False
    loop.step(t, False)                                # no new command: zero is retried
    assert drivetrain.kinds()[calls_while_broken:] == ['stop']


def test_stop_failing_while_idle_is_a_fault_too(rig):
    governor, drivetrain, faults, loop = rig
    drivetrain.fail_stop = True
    for i in range(3):
        loop.step(i * DT, False)
    assert loop.fault_count == 3 and not drivetrain.calls


def test_governor_exception_is_contained(rig, monkeypatch):
    governor, drivetrain, faults, loop = rig
    feed(governor, loop, 3)

    def boom(now, inhibit=False):
        raise ZeroDivisionError('governor bug')

    monkeypatch.setattr(governor, 'update', boom)
    loop.step(1.0, False)
    assert drivetrain.kinds()[-1] == 'stop'
    assert 'governor bug' in loop.last_fault and governor.state is ArmState.DISARMED


def test_broken_fault_reporter_cannot_break_the_safety_path():
    governor, drivetrain = CommandGovernor(LIMITS), FakeDrivetrain()

    def reporter(_message):
        raise OSError('logger down')

    loop = ControlLoop(governor, drivetrain, reporter)
    t = feed(governor, loop, 3)
    drivetrain.fail_drive = True
    feed(governor, loop, 1, t)                         # must not raise
    assert drivetrain.kinds()[-1] == 'stop' and loop.fault_count == 1


# ------------------------------------------------------------------ Drivetrain call ordering
class FakeNative:
    """Stands in for the phoenix6 request, drivetrain, unmanaged module and sim stepper."""

    def __init__(self, log, fail=()):
        self.log, self.fail = log, set(fail)
        self.values = [0.0, 0.0, 0.0]

    def _record(self, name, *args):
        if name in self.fail:
            raise RuntimeError(f'{name} failed')
        self.log.append((name, *args))

    def with_velocity_x(self, value):
        self.values[0] = value
        return self

    def with_velocity_y(self, value):
        self.values[1] = value
        return self

    def with_rotational_rate(self, value):
        self.values[2] = value
        return self

    def set_control(self, request):
        self._record('set_control', *request.values)

    def feed_enable(self, timeout):
        self._record('feed_enable', timeout)

    def stop(self):
        self._record('sim_stop')

    def close(self):
        self._record('close')


class QuietLogger:

    def info(self, _message):
        pass

    def error(self, _message):
        pass


def make_drivetrain(fail=(), simulation=True):
    """Build a Drivetrain around fakes, without its phoenix6-importing constructor."""
    log = []
    native = FakeNative(log, fail)
    drivetrain = object.__new__(Drivetrain)
    drivetrain._simulation = simulation
    drivetrain._closed = False
    drivetrain._sim = native if simulation else None
    drivetrain._unmanaged = native
    drivetrain._request = native
    drivetrain._drivetrain = native
    return drivetrain, log


def test_drive_latches_the_request_before_it_feeds_enable():
    drivetrain, log = make_drivetrain()
    drivetrain.drive(0.3, -0.1, 0.5)
    assert log == [('set_control', 0.3, -0.1, 0.5), ('feed_enable', ENABLE_FEED_TIMEOUT_S)]
    assert all(type(v) is float for v in log[0][1:])


def test_drive_does_not_feed_enable_when_the_request_fails():
    drivetrain, log = make_drivetrain(fail={'set_control'})
    with pytest.raises(RuntimeError):
        drivetrain.drive(0.3, 0.0, 0.0)
    assert log == []


def test_stop_latches_zero_and_never_feeds_enable():
    drivetrain, log = make_drivetrain()
    drivetrain.drive(0.3, 0.0, 0.0)
    del log[:]
    drivetrain.stop()
    assert log == [('set_control', 0.0, 0.0, 0.0)]


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_request_never_reaches_the_native_drivetrain(bad):
    drivetrain, log = make_drivetrain()
    with pytest.raises(ValueError):
        drivetrain.drive(0.1, bad, 0.0)
    assert log == []


def test_integer_velocities_are_converted_to_python_floats():
    drivetrain, log = make_drivetrain()
    drivetrain.drive(1, 0, -1)
    assert log[0] == ('set_control', 1.0, 0.0, -1.0)
    assert all(type(v) is float for v in log[0][1:])


def test_shutdown_order_is_zero_disable_sim_stop_close():
    drivetrain, log = make_drivetrain()
    drivetrain.drive(0.3, 0.0, 0.0)
    del log[:]
    drivetrain.shutdown(QuietLogger())
    assert log == [('set_control', 0.0, 0.0, 0.0), ('feed_enable', 0), ('sim_stop',), ('close',)]


def test_shutdown_on_hardware_has_no_sim_step():
    drivetrain, log = make_drivetrain(simulation=False)
    drivetrain.shutdown(QuietLogger())
    assert [entry[0] for entry in log] == ['set_control', 'feed_enable', 'close']


@pytest.mark.parametrize('failing', ['set_control', 'feed_enable', 'sim_stop', 'close'])
def test_shutdown_runs_every_other_step_when_one_fails(failing):
    drivetrain, log = make_drivetrain(fail={failing})
    drivetrain.shutdown(QuietLogger())                     # must not raise
    expected = ['set_control', 'feed_enable', 'sim_stop', 'close']
    expected.remove(failing)
    assert [entry[0] for entry in log] == expected


def test_shutdown_is_idempotent_and_nothing_touches_a_closed_drivetrain():
    drivetrain, log = make_drivetrain()
    drivetrain.shutdown(QuietLogger())
    del log[:]
    drivetrain.shutdown(QuietLogger())
    for call in (lambda: drivetrain.drive(0.1, 0.0, 0.0), drivetrain.stop, drivetrain.read,
                 drivetrain.tare):
        with pytest.raises(RuntimeError, match='closed'):
            call()
    assert log == []

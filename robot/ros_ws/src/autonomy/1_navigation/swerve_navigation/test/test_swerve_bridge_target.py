"""Unit tests for swerve_bridge target resolution and the other pure helpers.

SAFETY: the refusal of hardware:=true is tested here as a PURE FUNCTION on a fake environment
mapping.  The node is never started with hardware:=true, not even to watch it refuse.
"""

import ast
import inspect
import math
import os
import subprocess
import sys

import pytest

from swerve_navigation import swerve_bridge
from swerve_navigation.swerve_bridge import ArmState
from swerve_navigation.swerve_bridge import BridgeStatus
from swerve_navigation.swerve_bridge import check_native_target
from swerve_navigation.swerve_bridge import CommandGovernor
from swerve_navigation.swerve_bridge import diagonal_covariance
from swerve_navigation.swerve_bridge import drive_request_type_name
from swerve_navigation.swerve_bridge import GovernorLimits
from swerve_navigation.swerve_bridge import HardwareRefusedError
from swerve_navigation.swerve_bridge import LEVEL_ERROR
from swerve_navigation.swerve_bridge import LEVEL_OK
from swerve_navigation.swerve_bridge import LEVEL_WARN
from swerve_navigation.swerve_bridge import mapped_ctre_libraries
from swerve_navigation.swerve_bridge import MIN_ODOMETRY_HZ
from swerve_navigation.swerve_bridge import odometry_usable
from swerve_navigation.swerve_bridge import OdometryWatchdog
from swerve_navigation.swerve_bridge import OdomSample
from swerve_navigation.swerve_bridge import queue_delay_s
from swerve_navigation.swerve_bridge import reset_odometry_refusal
from swerve_navigation.swerve_bridge import resolve_site_packages
from swerve_navigation.swerve_bridge import resolve_target
from swerve_navigation.swerve_bridge import STOP_COMMAND
from swerve_navigation.swerve_bridge import summarise_status
from swerve_navigation.swerve_bridge import TargetVerificationError
from swerve_navigation.swerve_bridge import validate_rates
from swerve_navigation.swerve_bridge import yaw_to_quaternion


# ----------------------------------------------------------------------------- resolve_target
class TestResolveTarget:

    @pytest.mark.parametrize('env', [
        {}, {'SWERVE_NAV_FORCE_SIM': '1'}, {'SWERVE_NAV_FORCE_SIM': '0'},
        {'CTR_TARGET': 'Hardware'}, {'CTR_TARGET': 'Simulation'},
        {'CTR_TARGET': 'Hardware', 'SWERVE_NAV_FORCE_SIM': '1'}])
    def test_hardware_false_is_always_simulation(self, env):
        assert resolve_target(False, env) == 'Simulation'

    @pytest.mark.parametrize('env', [{}, {'CTR_TARGET': 'Hardware'},
                                     {'SWERVE_NAV_FORCE_SIM': '0'}, {'SWERVE_NAV_FORCE_SIM': ''},
                                     {'SWERVE_NAV_FORCE_SIM': 'false'}])
    def test_hardware_true_without_force_sim(self, env):
        assert resolve_target(True, env) == 'Hardware'

    @pytest.mark.parametrize('value', ['1', ' 1 ', 'true', 'TRUE', 'yes', 'on', '2', 'garbage'])
    def test_force_sim_refuses_hardware(self, value):
        with pytest.raises(HardwareRefusedError) as info:
            resolve_target(True, {'SWERVE_NAV_FORCE_SIM': value})
        assert 'SWERVE_NAV_FORCE_SIM' in str(info.value)

    def test_force_sim_wins_over_ctr_target_hardware(self):
        with pytest.raises(HardwareRefusedError):
            resolve_target(True, {'SWERVE_NAV_FORCE_SIM': '1', 'CTR_TARGET': 'Hardware'})

    @pytest.mark.parametrize('ctr_target', ['Simulation', 'Replay', 'hardware', 'HARDWARE', ''])
    def test_contradicting_ctr_target_refuses_hardware(self, ctr_target):
        # Anything but the exact string would make phoenix6 pick a target on its own.
        with pytest.raises(HardwareRefusedError) as info:
            resolve_target(True, {'CTR_TARGET': ctr_target})
        assert 'CTR_TARGET' in str(info.value)

    @pytest.mark.parametrize('value', ['true', 'false', 'True', 1, 0, None, 1.0])
    def test_non_bool_hardware_param_is_a_type_error(self, value):
        # The string "false" is truthy: it must never be able to select the real robot.
        with pytest.raises(TypeError):
            resolve_target(value, {})

    def test_does_not_touch_the_environment(self):
        env = {'SWERVE_NAV_FORCE_SIM': '0'}
        resolve_target(True, env)
        assert env == {'SWERVE_NAV_FORCE_SIM': '0'}

    def test_refusal_is_not_a_generic_exception_type(self):
        assert issubclass(HardwareRefusedError, RuntimeError)
        assert not issubclass(TargetVerificationError, HardwareRefusedError)


class TestPhoenix6IsLazy:

    def test_importing_the_module_does_not_import_phoenix6(self):
        code = ('import sys; import swerve_navigation.swerve_bridge, swerve_navigation.phoenix_sim; '
                'sys.exit(1 if any(m.split(".")[0] == "phoenix6" for m in sys.modules) else 0)')
        result = subprocess.run([sys.executable, '-c', code], env=dict(os.environ), timeout=60)
        assert result.returncode == 0

    def test_phoenix6_is_only_imported_inside_the_drivetrain_class(self):
        with open(swerve_bridge.__file__) as source:
            tree = ast.parse(source.read())

        def phoenix_imports(node):
            for child in ast.walk(node):
                if isinstance(child, ast.Import):
                    if any(alias.name.split('.')[0] == 'phoenix6' for alias in child.names):
                        yield child
                elif isinstance(child, ast.ImportFrom):
                    if (child.module or '').split('.')[0] == 'phoenix6':
                        yield child

        everywhere = list(phoenix_imports(tree))
        drivetrain = next(n for n in tree.body
                          if isinstance(n, ast.ClassDef) and n.name == 'Drivetrain')
        inside = list(phoenix_imports(drivetrain))
        assert inside, 'the Drivetrain class is expected to import phoenix6 lazily'
        assert len(everywhere) == len(inside)

    def test_node_resolves_the_target_before_it_builds_the_drivetrain(self):
        with open(swerve_bridge.__file__) as source:
            tree = ast.parse(source.read())
        node_class = next(n for n in tree.body
                          if isinstance(n, ast.ClassDef) and n.name == 'SwerveBridge')
        methods = {m.name: m for m in node_class.body if isinstance(m, ast.FunctionDef)}

        def calls(func):
            return {c.func.id for c in ast.walk(func)
                    if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)}

        assert 'resolve_target' in calls(methods['__init__'])
        assert 'Drivetrain' not in calls(methods['__init__'])
        assert 'Drivetrain' in calls(methods['start'])


# ------------------------------------------------------------------------ native target checks
SIM_LIBS = ['/x/phoenix6/lib/libCTRE_PhoenixTools_Sim.so',
            '/x/phoenix6/lib/libCTRE_SimProTalonFX.so',
            '/x/phoenix6/lib/libCTRE_Phoenix6_Sim.so']


class TestCheckNativeTarget:

    def test_simulation_ok(self):
        check_native_target('Simulation', True, SIM_LIBS)

    def test_hardware_ok(self):
        check_native_target('Hardware', False, ['/x/lib/libCTRE_PhoenixTools.so'])

    def test_simulation_requested_but_hardware_loaded(self):
        with pytest.raises(TargetVerificationError):
            check_native_target('Simulation', False, SIM_LIBS)

    def test_hardware_requested_but_simulation_loaded(self):
        with pytest.raises(TargetVerificationError):
            check_native_target('Hardware', True, SIM_LIBS)

    @pytest.mark.parametrize('lib', ['/x/lib/libCTRE_PhoenixTools.so',
                                     '/x/lib/libCTRE_Phoenix6.so',
                                     '/Sim/lib/libCTRE_Phoenix6.so',
                                     '/x/lib/libCTRE_Phoenix6_Replay.so'])
    def test_any_non_sim_library_in_simulation_is_fatal(self, lib):
        # '/Sim/lib/...': only the file name counts, not a directory that happens to say Sim.
        with pytest.raises(TargetVerificationError) as info:
            check_native_target('Simulation', True, SIM_LIBS + [lib])
        assert os.path.basename(lib) in str(info.value)

    def test_mapped_ctre_libraries_parses_proc_maps(self, tmp_path):
        maps = tmp_path / 'maps'
        maps.write_text(
            '7f00-7f01 r-xp 00000000 08:01 1 /a/lib/libCTRE_SimProTalonFX.so\n'
            '7f01-7f02 rw-p 00001000 08:01 1 /a/lib/libCTRE_SimProTalonFX.so\n'
            '7f02-7f03 r-xp 00000000 08:01 2 /usr/lib/libc.so.6\n'
            '7f03-7f04 r-xp 00000000 08:01 3 /a/lib/libCTRE_PhoenixTools_Sim.so\n')
        assert mapped_ctre_libraries(str(maps)) == ['/a/lib/libCTRE_PhoenixTools_Sim.so',
                                                    '/a/lib/libCTRE_SimProTalonFX.so']

    def test_this_test_process_has_no_ctre_library_loaded(self):
        assert mapped_ctre_libraries() == []


class TestSmallHelpers:

    def test_site_packages_env_overrides_param(self):
        assert resolve_site_packages('/param/dir', {'PHOENIX6_SITE_PACKAGES': '/env/dir'}) \
            == '/env/dir'

    def test_site_packages_param_used_when_env_unset_or_empty(self):
        assert resolve_site_packages('/param/dir', {}) == '/param/dir'
        assert resolve_site_packages('/param/dir', {'PHOENIX6_SITE_PACKAGES': ''}) == '/param/dir'

    def test_site_packages_is_normalised(self):
        assert resolve_site_packages('~/p6/', {}) == os.path.expanduser('~/p6')

    def test_drive_request_type(self):
        assert drive_request_type_name('velocity') == 'VELOCITY'
        assert drive_request_type_name('open_loop') == 'OPEN_LOOP_VOLTAGE'
        for bad in ('', 'Velocity', 'voltage', 'VELOCITY'):
            with pytest.raises(ValueError):
                drive_request_type_name(bad)

    def test_yaw_to_quaternion(self):
        assert yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)
        _, _, z, w = yaw_to_quaternion(math.pi / 2)
        assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
        for yaw in (-3.0, -1.0, 0.3, 2.5, math.pi):
            x, y, z, w = yaw_to_quaternion(yaw)
            assert (x, y) == (0.0, 0.0)
            assert math.hypot(z, w) == pytest.approx(1.0)
            assert 2.0 * math.atan2(z, w) == pytest.approx(yaw)

    def test_diagonal_covariance(self):
        cov = diagonal_covariance([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 'p')
        assert len(cov) == 36
        assert [cov[i * 7] for i in range(6)] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert sum(cov) == 21.0

    @pytest.mark.parametrize('bad', [[1.0] * 5, [1.0] * 7, [], [1.0] * 5 + [math.nan],
                                     [1.0] * 5 + [-1.0], [1.0] * 5 + [math.inf]])
    def test_diagonal_covariance_rejects(self, bad):
        with pytest.raises(ValueError):
            diagonal_covariance(bad, 'p')

    def test_odometry_usable(self):
        good = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        assert odometry_usable(12.5, 10, True, good)
        assert not odometry_usable(0.0, 10, True, good)        # no odometry update yet
        assert not odometry_usable(12.5, 0, True, good)        # no successful acquisition yet
        assert not odometry_usable(12.5, 10, False, good)      # phoenix6 says invalid
        assert not odometry_usable(12.5, 10, True, (0.0, math.nan, 0.0, 0.0, 0.0, 0.0))
        assert not odometry_usable(12.5, 10, True, (math.inf, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert not odometry_usable(math.nan, 10, True, good)   # would give a state age of 0
        assert not odometry_usable(math.inf, 10, True, good)


def sample(valid=True, vx=0.0, vy=0.0, wz=0.0, age=0.005, daqs=500, failed=2) -> OdomSample:
    return OdomSample(valid=valid, timestamp=5.0, age=age, x=1.0, y=2.0, yaw=0.5,
                      vx=vx, vy=vy, wz=wz, successful_daqs=daqs, failed_daqs=failed,
                      odometry_period=0.01)


class TestQueueDelay:
    """A /cmd_vel is aged by the time it waited for its callback (executor stall, start-up)."""

    NOW = 1_700_000_000_000_000_000

    def test_delay_from_reception_time(self):
        info = {'source_timestamp': self.NOW - 900_000_000,
                'received_timestamp': self.NOW - 750_000_000}
        assert queue_delay_s(self.NOW, info) == pytest.approx(0.75)

    def test_falls_back_to_the_source_timestamp(self):
        # Some RMWs report received_timestamp 0 ("unsupported").
        info = {'source_timestamp': self.NOW - 400_000_000, 'received_timestamp': 0}
        assert queue_delay_s(self.NOW, info) == pytest.approx(0.4)
        assert queue_delay_s(self.NOW, {'source_timestamp': self.NOW - 400_000_000}) \
            == pytest.approx(0.4)

    def test_wall_clock_stepped_backwards_is_not_a_negative_age(self):
        assert queue_delay_s(self.NOW, {'received_timestamp': self.NOW + 5_000_000_000}) == 0.0

    @pytest.mark.parametrize('info', [None, {}, {'received_timestamp': 0, 'source_timestamp': 0},
                                      {'received_timestamp': None}, {'received_timestamp': -1},
                                      {'received_timestamp': 'x'}, {'received_timestamp': True},
                                      {'received_timestamp': 1.5e18}])
    def test_unknown_reception_time_means_no_back_dating(self, info):
        assert queue_delay_s(self.NOW, info) == 0.0

    def test_the_node_callback_takes_the_message_info(self):
        # rclpy passes the message info only to callbacks that accept a second argument.
        parameters = inspect.signature(swerve_bridge.SwerveBridge._on_cmd_vel).parameters
        assert list(parameters) == ['self', 'msg', 'message_info']

    def test_a_command_that_waited_longer_than_the_timeout_never_arms(self):
        gov = CommandGovernor(GovernorLimits(1.0, 1.5, 2.0, 4.0, cmd_timeout=0.3))
        now = 100.0
        delay = queue_delay_s(self.NOW, {'received_timestamp': self.NOW - 1_000_000_000})
        gov.on_command(0.5, 0.0, 0.0, now - delay)
        for i in range(5):
            assert gov.update(now + 0.02 * i) == STOP_COMMAND
        assert gov.state is ArmState.DISARMED and gov.command_age(now) is None


class TestOdometryWatchdog:
    """Frozen drivetrain state (hung odometry thread / every DAQ failing) must inhibit driving."""

    def test_not_alive_before_the_first_sample(self):
        assert not OdometryWatchdog(0.25).alive(0.0)

    def test_alive_while_the_daq_counter_advances(self):
        dog = OdometryWatchdog(0.25)
        for i in range(100):
            dog.observe(sample(daqs=500 + 2 * i), i * 0.02)
            assert dog.alive(i * 0.02)

    def test_frozen_state_goes_stale(self):
        # What get_state_copy() returns with the odometry thread stopped (verified in sim):
        # valid=True, same successful_daqs, growing age.
        dog = OdometryWatchdog(0.25)
        dog.observe(sample(daqs=128), 0.0)
        for i in range(1, 50):
            t = i * 0.02
            dog.observe(sample(daqs=128, age=t), t)
            assert dog.alive(t) == (t < 0.25)

    def test_failing_acquisitions_go_stale_even_with_a_recent_state(self):
        dog = OdometryWatchdog(0.25)
        dog.observe(sample(daqs=128, failed=0), 0.0)
        for i in range(1, 50):
            dog.observe(sample(daqs=128, failed=i, age=0.005), i * 0.02)
        assert not dog.alive(1.0)

    def test_old_state_is_not_progress_even_if_the_counter_moved(self):
        dog = OdometryWatchdog(0.25)
        dog.observe(sample(daqs=1, age=0.3), 0.0)
        assert not dog.alive(0.0)
        dog.observe(sample(daqs=2, age=math.nan), 0.02)
        assert not dog.alive(0.02)

    def test_invalid_or_missing_samples_are_not_progress(self):
        dog = OdometryWatchdog(0.25)
        dog.observe(None, 0.0)
        dog.observe(sample(valid=False, daqs=7), 0.0)
        assert not dog.alive(0.0)

    def test_recovers_and_counter_reset_counts_as_progress(self):
        dog = OdometryWatchdog(0.25)
        dog.observe(sample(daqs=128), 0.0)
        assert not dog.alive(1.0)
        dog.observe(sample(daqs=3), 1.0)
        assert dog.alive(1.0)

    def test_clock_going_backwards_is_not_alive(self):
        dog = OdometryWatchdog(0.25)
        dog.observe(sample(daqs=1), 10.0)
        assert not dog.alive(9.0)


class TestValidateRates:

    @pytest.mark.parametrize('rate_hz,odometry_hz', [(50.0, 100.0), (20.0, 0.0), (250.0, 1000.0),
                                                     (50.0, MIN_ODOMETRY_HZ), (50.0, 250.0)])
    def test_accepts(self, rate_hz, odometry_hz):
        validate_rates(rate_hz, odometry_hz)

    @pytest.mark.parametrize('rate_hz', [0.0, 5.0, 19.9, 250.1, -50.0, math.nan, math.inf])
    def test_rejects_rate(self, rate_hz):
        with pytest.raises(ValueError, match='rate_hz'):
            validate_rates(rate_hz, 100.0)

    @pytest.mark.parametrize('odometry_hz', [1.0, 10.0, 49.9, 1000.1, -100.0, math.nan, math.inf])
    def test_rejects_slow_or_absurd_odometry(self, odometry_hz):
        # The odometry thread applies the requests: 1 Hz would mean a 1 s late zero request.
        with pytest.raises(ValueError, match='odometry_hz'):
            validate_rates(50.0, odometry_hz)


class TestResetOdometryRefusal:

    def test_allowed_when_disarmed_and_stationary(self):
        assert reset_odometry_refusal(ArmState.DISARMED, sample()) is None
        assert reset_odometry_refusal(ArmState.DISARMED, sample(vx=0.01, wz=-0.01)) is None

    @pytest.mark.parametrize('state', [ArmState.PRIMED, ArmState.ARMED])
    def test_refused_unless_disarmed(self, state):
        assert 'cmd_vel' in reset_odometry_refusal(state, sample())

    def test_refused_while_a_command_is_about_to_arm(self):
        # Accepted command, next tick not run yet: state is still DISARMED.
        assert 'cmd_vel' in reset_odometry_refusal(ArmState.DISARMED, sample(),
                                                   command_pending=True)

    def test_refused_without_valid_odometry(self):
        assert 'odometry' in reset_odometry_refusal(ArmState.DISARMED, None)
        assert 'odometry' in reset_odometry_refusal(ArmState.DISARMED, sample(valid=False))

    @pytest.mark.parametrize('motion', [{'vx': 0.05}, {'vy': -0.05}, {'wz': 0.05},
                                        {'vx': 0.015, 'vy': 0.015}, {'wz': -0.3}])
    def test_refused_while_moving(self, motion):
        assert 'moving' in reset_odometry_refusal(ArmState.DISARMED, sample(**motion))


def status(**overrides) -> BridgeStatus:
    fields = {'hardware': False, 'state': ArmState.DISARMED, 'e_stop': False, 'enabled': False,
              'cmd_age': None, 'odometry_ready': True, 'sample': sample(), 'fault_count': 0,
              'last_fault': '', 'rejected_commands': 0, 'sim_alive': True,
              'drive_request_type': 'velocity'}
    fields.update(overrides)
    return BridgeStatus(**fields)


class TestSummariseStatus:

    def test_ok_contains_the_contract_keys(self):
        level, message, values = summarise_status(status(state=ArmState.ARMED, enabled=True,
                                                         cmd_age=0.0123))
        keys = dict(values)
        assert level == LEVEL_OK and message == 'armed'
        assert keys['mode'] == 'sim'
        assert keys['enabled'] == 'True'
        assert keys['armed'] == 'True'
        assert keys['cmd_age'] == '0.012'
        assert keys['successful_daqs'] == '500'
        assert keys['failed_daqs'] == '2'
        assert keys['sim_thread_alive'] == 'True'
        assert all(isinstance(k, str) and isinstance(v, str) for k, v in values)

    def test_hardware_mode_is_reported_and_has_no_sim_key(self):
        _, _, values = summarise_status(status(hardware=True, sim_alive=None))
        keys = dict(values)
        assert keys['mode'] == 'hardware'
        assert 'sim_thread_alive' not in keys

    def test_primed_is_not_reported_as_armed(self):
        _, _, values = summarise_status(status(state=ArmState.PRIMED))
        assert dict(values)['armed'] == 'False'

    def test_levels(self):
        assert summarise_status(status(odometry_ready=False, sample=None))[0] == LEVEL_ERROR
        level, message, _ = summarise_status(status(sim_alive=False, sim_error='Boom()'))
        assert level == LEVEL_ERROR and 'Boom()' in message
        assert summarise_status(status(e_stop=True))[0] == LEVEL_WARN
        level, message, _ = summarise_status(status(fault_count=3, last_fault='boom'))
        assert level == LEVEL_WARN and 'boom' in message and '3' in message

    def test_no_command_and_no_sample(self):
        _, _, values = summarise_status(status(sample=None, odometry_ready=False))
        keys = dict(values)
        assert keys['cmd_age'] == 'none'
        assert keys['successful_daqs'] == '0'
        assert keys['state_age'] == 'none'


class TestMainExitPaths:
    """main() wiring with rclpy, signal handlers and the node replaced: nothing real is touched.

    The node is NEVER started with hardware:=true; the refusal is injected as the exception that
    resolve_target() raises inside SwerveBridge.__init__().
    """

    @pytest.fixture
    def harmless(self, monkeypatch):
        monkeypatch.setattr(swerve_bridge.rclpy, 'init', lambda **_kwargs: None)
        monkeypatch.setattr(swerve_bridge.rclpy, 'try_shutdown', lambda: None)
        monkeypatch.setattr(swerve_bridge.signal, 'signal', lambda *_args: None)

    def test_refused_hardware_exits_with_code_2_before_phoenix6_import(
            self, harmless, monkeypatch):
        def refuse():
            return resolve_target(True, {'SWERVE_NAV_FORCE_SIM': '1'})

        monkeypatch.setattr(swerve_bridge, 'SwerveBridge', refuse)
        with pytest.raises(SystemExit) as info:
            swerve_bridge.main()
        assert info.value.code == swerve_bridge.EXIT_HARDWARE_REFUSED == 2
        assert not any(name.split('.')[0] == 'phoenix6' for name in sys.modules)

    def test_start_failure_shuts_the_drivetrain_down_and_exits_with_code_1(
            self, harmless, monkeypatch):
        calls = []

        class FailingNode:

            def start(self):
                raise RuntimeError('drivetrain did not come up')

            def shutdown_drivetrain(self):
                calls.append('shutdown_drivetrain')

            def destroy_node(self):
                calls.append('destroy_node')

        monkeypatch.setattr(swerve_bridge, 'SwerveBridge', FailingNode)
        with pytest.raises(SystemExit) as info:
            swerve_bridge.main()
        assert info.value.code == swerve_bridge.EXIT_ERROR == 1
        assert calls == ['shutdown_drivetrain', 'destroy_node']

    def test_sighup_is_handled_like_sigint_and_sigterm(self, monkeypatch):
        handled = []
        monkeypatch.setattr(swerve_bridge.rclpy, 'init', lambda **_kwargs: None)
        monkeypatch.setattr(swerve_bridge.rclpy, 'try_shutdown', lambda: None)
        monkeypatch.setattr(swerve_bridge.signal, 'signal',
                            lambda signum, _handler: handled.append(signum))

        def refuse():
            raise HardwareRefusedError('stop here')

        monkeypatch.setattr(swerve_bridge, 'SwerveBridge', refuse)
        with pytest.raises(SystemExit):
            swerve_bridge.main()
        assert set(handled) == {swerve_bridge.signal.SIGINT, swerve_bridge.signal.SIGTERM,
                                swerve_bridge.signal.SIGHUP}

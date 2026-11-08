"""Bounded correction, mode interlocks and fitting against independent synthetic data."""
import csv
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from swerve_navigation.swerve_bridge import (ArmState, CommandGovernor, ControlLoop, DriveCommand,
                                      GovernorLimits, STOP_COMMAND, SwerveBridge)
from swerve_navigation.yaw_bias import YawBiasConfig, YawBiasCompensator, StationHeadingHold

CONFIG = YawBiasConfig(vx_gain=0.10370857339120197, vy_gain=-0.20359885945361017)
LIMITS = GovernorLimits(max_linear_speed=1., max_angular_speed=1.5,
                        max_linear_accel=1.2, max_angular_accel=3., cmd_timeout=.3)


def run(comp, velocity=(0.3, 0., 0.), start=0., ticks=100, **kwargs):
    values = [comp.apply(*velocity, start + i * 0.02, **kwargs) for i in range(ticks)]
    return values[-1], start + ticks * 0.02


def test_forward_and_sideways_countersteering_have_correct_sign_and_size():
    for vx, vy in ((.3, 0.), (-.3, 0.), (0., .3), (0., -.3)):
        out, _ = run(YawBiasCompensator(CONFIG), (vx, vy, 0.))
        assert out == pytest.approx(-.75 * (CONFIG.vx_gain * vx + CONFIG.vy_gain * vy))
        assert abs(out) < .07


def test_settling_ramp_saturation_and_speed_envelope():
    comp = YawBiasCompensator(CONFIG)
    assert comp.apply(.3, 0., 0., 0.) == 0.
    assert comp.apply(.3, 0., 0., .1) == 0.
    outputs = [comp.apply(.3, 0., 0., i * .02) for i in range(6, 110)]
    assert max(abs(b-a) for a, b in zip(outputs, outputs[1:])) <= .2 * .02 + 1e-12
    out1, _ = run(YawBiasCompensator(CONFIG), (.3, 0., 0.))
    out2, _ = run(YawBiasCompensator(CONFIG), (.4, 0., 0.))
    assert out2 == pytest.approx(out1)  # no extrapolation
    for speed in (0., .05, .8):
        out, _ = run(YawBiasCompensator(CONFIG), (speed, 0., 0.))
        assert out == 0.


def test_stops_and_rotation_locks_remove_correction_immediately():
    comp = YawBiasCompensator(CONFIG)
    out, t = run(comp)
    assert out < 0
    assert comp.apply(.3, 0., .1, t, locked=True, force_zero=True) == 0.
    assert comp.apply(0., 0., 0., t + .02) == 0.
    assert run(YawBiasCompensator(CONFIG), locked=True)[0] == 0.
    assert run(YawBiasCompensator(CONFIG), source_known=False)[0] == 0.
    # An intentional operator turn still passes with compensation disabled.
    out, _ = run(YawBiasCompensator(CONFIG), (0., 0., .4), locked=True)
    assert out == pytest.approx(.4)


def test_raw_deadband_is_preserved_but_does_not_erase_feedforward():
    comp = YawBiasCompensator(CONFIG)
    out, _ = run(comp, (.3, 0., .02))
    assert out == pytest.approx(-.75 * CONFIG.vx_gain * .3)
    assert 0 < abs(out) < CONFIG.command_deadband
    out, _ = run(YawBiasCompensator(CONFIG), (.3, 0., .2))
    assert out == pytest.approx(.2)  # no fitted correction during turns


def test_direction_changes_clear_old_correction_and_bounds_are_enforced():
    comp = YawBiasCompensator(CONFIG)
    out, t = run(comp)
    assert out < 0
    assert comp.apply(-.3, 0., 0., t) == 0.
    out, _ = run(comp, (-.3, 0., 0.), start=t+.02)
    assert out > 0
    out, _ = run(YawBiasCompensator(YawBiasConfig(vx_gain=1., vy_gain=1.)), (.2, .2, 0.))
    assert out == pytest.approx(-.07)
    out, _ = run(YawBiasCompensator(YawBiasConfig(vx_gain=1., max_angular_speed=.03)))
    assert out == pytest.approx(-.03)


@pytest.mark.parametrize('phase,locked', [('dock', True), ('undock', True),
                                         ('stage_adjust', True), ('staging', False), ('align', False)])
def test_docking_phase_heartbeat_controls_correction(phase, locked):
    from swerve_navigation.station_docking import StationDocking
    messages = []
    fake = NS(state=phase, rotation_lock=NS(publish=messages.append))
    StationDocking._publish_rotation_lock(fake)
    assert messages[-1].data == locked


@pytest.mark.parametrize('velocity,bias', [((.12, 0.), .04), ((-.10, 0.), -.04), ((0., .06), -.04)])
def test_station_feedback_holds_physical_heading_on_biased_lagged_drive(velocity, bias):
    hold = StationHeadingHold(CONFIG)
    # Eight seconds of docking. A 0.1 s actuator lag and 0.8 angular response
    # represent a drive that tracks imperfectly; open loop would drift 18 degrees.
    yaw, rate, peak = 3.13, 0., 0.
    reference = yaw
    previous = 0.
    for i in range(400):
        correction = hold.apply(*velocity, yaw, rate, .1, i*.02)
        assert abs(correction) <= .07 + 1e-12
        assert abs(correction-previous) <= .004 + 1e-12
        rate += (.8*correction+bias-rate)*.02/.1
        yaw = math.atan2(math.sin(yaw+rate*.02), math.cos(yaw+rate*.02))
        error = abs(math.atan2(math.sin(yaw-reference), math.cos(yaw-reference)))
        peak = max(peak, error)
        previous = correction
    assert abs(bias * 8) > .08
    assert peak < .06
    assert hold.apply(0., 0., yaw, rate, 0., 8.) == 0.  # never correct by turning at rest
    assert hold.reference is None


def test_station_feedback_requires_commanded_and_measured_translation():
    hold = StationHeadingHold(CONFIG)
    assert hold.apply(.12, 0., 0., .05, 0., 0.) == 0.
    assert hold.apply(.12, 0., .04, .05, 0., .1) == 0.
    assert hold.apply(0., 0., .04, .05, .1, .2) == 0.
    assert hold.apply(.3, 0., .04, .05, .3, .3) == 0.  # not a slow docking maneuver


def test_station_feedback_trims_small_initial_error_to_requested_heading_while_moving():
    hold = StationHeadingHold(CONFIG)
    yaw, rate = .088, 0.
    for i in range(400):
        correction = hold.apply(.12, 0., yaw, rate, .12, i*.02, target_yaw=0.)
        rate += (.8*correction+.04-rate)*.02/.1
        yaw += rate*.02
    assert abs(yaw) < .08
    assert hold.apply(0., 0., yaw, rate, 0., 8., target_yaw=0.) == 0.
    assert hold.apply(.12, 0., .5, 0., .12, 8.1, target_yaw=0.) == 0.  # no large station turn


def test_station_feedback_remains_active_past_old_drift_abort_threshold():
    hold = StationHeadingHold(CONFIG)
    for i in range(20):
        correction = hold.apply(-.12, 0., .20, .01, .12, i*.02, target_yaw=0.)
    assert correction < 0.
    assert hold.apply(-.12, 0., .26, .01, .12, .4, target_yaw=0.) == 0.
    assert hold.apply(0., 0., .20, .01, 0., .5, target_yaw=0.) == 0.


def test_bridge_countersteers_while_docking_but_not_while_stationary_or_mode_stale():
    bridge = bridge_fake()
    bridge._station_heading_hold = StationHeadingHold(CONFIG)
    bridge._yaw_source = 'nav'
    bridge._yaw_lock_seen, bridge._yaw_lock = True, True
    bridge._station_heading_target = 0.
    bridge._sample = NS(yaw=0., wz=.04, vx=.12, vy=0.)
    command = DriveCommand(.12, 0., 0., True)
    for i in range(20):
        t = 4+i*.02
        bridge._yaw_lock_stamp = t
        bridge._station_heading_target_stamp = t
        bridge._sample.yaw = .04*i*.02
        corrected = SwerveBridge._correct_yaw(bridge, command, t)
    assert corrected.wz < 0 and corrected.vx == .12
    bridge._sample.vx = 0.
    assert SwerveBridge._correct_yaw(bridge, command, t+.02).wz == 0.
    bridge._sample.vx = .12
    assert SwerveBridge._correct_yaw(bridge, command, t+1.).wz == 0.


def test_heading_target_heartbeat_is_required_even_with_a_fresh_rotation_lock():
    bridge = bridge_fake()
    bridge._station_heading_hold = StationHeadingHold(CONFIG)
    bridge._yaw_source = 'nav'
    bridge._yaw_lock_seen, bridge._yaw_lock, bridge._yaw_lock_stamp = True, True, 5.
    bridge._station_heading_target, bridge._station_heading_target_stamp = 0., 4.
    bridge._sample = NS(yaw=.04, wz=.04, vx=.12, vy=0.)
    assert SwerveBridge._correct_yaw(bridge, DriveCommand(.12, 0., 0., True), 5.).wz == 0.


def bridge_fake():
    return NS(_yaw_bias=YawBiasCompensator(CONFIG), _yaw_source='teleop',
              _station_heading_hold=None,
              _station_heading_target=None, _station_heading_target_stamp=-math.inf,
              _yaw_lock_seen=False, _yaw_lock_stamp=-math.inf, _yaw_lock=True,
              _yaw_has_docking=False, _yaw_mode_started=0., _applied_command=(0., 0., 0.))


def test_bridge_discovery_stale_heartbeat_and_docking_force_zero():
    bridge = bridge_fake()
    cmd = DriveCommand(.3, 0., 0., True)
    assert SwerveBridge._correct_yaw(bridge, cmd, 1.).wz == 0.
    for i in range(150, 251):
        corrected = SwerveBridge._correct_yaw(bridge, cmd, i * .02)
    assert corrected.wz < 0
    bridge._yaw_lock_seen, bridge._yaw_lock_stamp = True, 5.
    bridge._yaw_lock, bridge._yaw_source = True, 'nav'
    assert SwerveBridge._correct_yaw(bridge, DriveCommand(.12, .02, .03, True), 5.02).wz == 0.
    bridge._yaw_lock = False
    # Once a docking mode publisher has been seen, losing it never enables correction.
    assert SwerveBridge._correct_yaw(bridge, cmd, 6.).wz == 0.
    assert SwerveBridge._correct_yaw(bridge, STOP_COMMAND, 6.02) == STOP_COMMAND
    assert bridge._applied_command == (0., 0., 0.)


def test_watchdog_and_estop_cannot_be_overridden_by_adapter():
    bridge = bridge_fake()
    governor = CommandGovernor(LIMITS)
    calls = []
    drive = NS(drive=lambda *v: calls.append(v), stop=lambda: calls.append('stop'))
    faults = []
    loop = ControlLoop(governor, drive, faults.append,
                       lambda command, now: SwerveBridge._correct_yaw(bridge, command, now))
    for i in range(200):
        t = 4 + i * .02
        governor.on_command(.3, 0., 0., t)
        loop.step(t, False)
    assert any(isinstance(c, tuple) and c[2] < 0 for c in calls)
    loop.step(t + 1., False)  # no new command -> watchdog
    assert calls[-1] == 'stop' and governor.state is ArmState.DISARMED
    governor.on_command(.3, 0., 0., t + 1.02)
    governor.on_e_stop(True)
    loop.step(t + 1.02, False)
    assert calls[-1] == 'stop' and not faults


def test_adapter_cannot_enable_drive_or_change_translation():
    governor = CommandGovernor(LIMITS)
    calls, faults = [], []
    loop = ControlLoop(governor, NS(drive=lambda *v: calls.append(v), stop=lambda: calls.append('stop')),
                       faults.append, lambda c, _: DriveCommand(1., 0., 0., True))
    loop.step(0., False)
    assert calls == ['stop'] and faults


def test_hardware_parameters_match_reviewed_fit():
    root = Path(__file__).resolve().parents[2]
    p = yaml.safe_load((root / 'navigation_bringup/config/swerve_bridge.yaml').read_text())['swerve_bridge']['ros__parameters']
    assert p['yaw_bias_enabled'] is True
    assert p['yaw_bias_vx_gain'] == CONFIG.vx_gain and p['yaw_bias_vy_gain'] == CONFIG.vy_gain
    assert p['rotational_deadband'] == CONFIG.command_deadband


@pytest.mark.parametrize('already_compensated', [False, True])
def test_fit_recovers_omni_bias_and_rejects_unidentifiable_data(tmp_path, already_compensated):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('fit_drive_drift', root / 'scripts/fit_drive_drift.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows = []
    for i, (vx, vy) in enumerate(((.15, 0.), (.3, 0.), (-.15, 0.), (-.3, 0.),
                                   (0., .15), (0., .3), (0., -.15), (0., -.3))):
        for j in range(200):
            rows.append(dict(t=(i*200+j)*.02, source='teleop', cmd_age=.01,
                             cmd_vx=vx, cmd_vy=vy, cmd_wz=0., odom_vx=vx,
                             odom_vy=vy, odom_wz=.12*vx-.2*vy))
            if already_compensated:
                correction = -.6 * rows[-1]['odom_wz']
                rows[-1].update(applied_age=.01, applied_wz=correction)
                rows[-1]['odom_wz'] += correction
    path = tmp_path / 'data.csv'
    with path.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    fit = module.fit_recording(path)
    assert fit['vx_gain_rad_m'] == pytest.approx(.12)
    assert fit['vy_gain_rad_m'] == pytest.approx(-.2)
    assert fit['removed_applied_rotation'] == already_compensated
    assert all(v['rms_reduction'] > .7 for v in fit['held_out'])
    with path.open('w') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows[:200])
    with pytest.raises(ValueError, match='independent forward and sideways'):
        module.fit_recording(path)


@pytest.mark.parametrize('kwargs', [{'gain': 2.}, {'vx_gain': math.nan},
                                    {'vx_gain': 5.}, {'max_correction': .3}, {'model_speed': .05}])
def test_unsafe_model_parameters_are_refused(kwargs):
    with pytest.raises(ValueError):
        YawBiasConfig(**kwargs)

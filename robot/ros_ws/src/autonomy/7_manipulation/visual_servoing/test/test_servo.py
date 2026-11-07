"""Whole grasp runs against the simulated arm and wrist camera (simulated time: each run takes milliseconds)."""
import numpy as np
import pytest

from sim import Camera, FakeArm, FakeClock, tool_down
from visual_servoing.mount import Mount, rpy_matrix, transform, wrap_pi
from visual_servoing.servo import GripperConfig, RunFailed, ServoConfig, ServoRunner

MOUNT = Mount()
START = tool_down(0.40, 0.0, 0.35)
CENTER = np.array([0.42, 0.03, 0.02])      # plate top-face centre, long side at 30 deg
LONG_DEG = 30.0
LEGACY_GRIPPER = dict(use_fsr=True, step=20, final_squeeze=30, nudge_mm=1.0, nudge_speed=30.0,
                      tare_samples=5, fsr_samples=1, target_pressure=(15, -1), stop_pressure=(25, -1))
TWO_FINGERS = dict(LEGACY_GRIPPER, target_pressure=(250, 120), stop_pressure=(370, 140))


def held_from(value):
    """Both fingers press once the gripper is closed to `value`."""
    return lambda v, arm: (400, 200) if v >= value else (0, 0)


def run(start=START, fsr=held_from(130), prepare=None, hook=None, camera=None, ft=None, **config):
    config.setdefault('dry_run', False)
    config.setdefault('return_to_start', True)  # exercise the optional return in the original simulations
    config.setdefault('gripper', GripperConfig(**LEGACY_GRIPPER))
    config = ServoConfig(**config)
    clock = FakeClock()
    arm = FakeArm(clock, start, read_only=config.dry_run, fsr=fsr, ft=ft)
    runner = ServoRunner(arm, MOUNT, config, clock=clock.time, sleep=clock.sleep, prepare=prepare)
    cam = Camera(runner, MOUNT, CENTER, np.radians(LONG_DEG), **(camera or {}))

    def on_pose(t, pose):
        cam(t, pose)
        if hook is not None:
            hook(t - 1000.0, runner)
    arm.on_pose = on_pose
    done = []
    assert runner.start(on_done=done.append)
    runner._thread.join(30.0)
    assert not runner.running
    status = runner.status()
    assert done == [status] and status['phase'] == 'idle' and status['running'] is False
    return status, arm


def tip(pose):
    return (pose @ MOUNT.flange_T_tip)[:3, 3]


def finger_angle_deg(pose):
    """Direction of the finger axis (flange y) in the base xy plane."""
    return np.degrees(np.arctan2(pose[1, 1], pose[0, 1]))


def axial_error_deg(a, b):
    return abs(np.degrees(wrap_pi(2 * np.radians(a - b)) / 2))


def test_execute_run_grasps_the_plate():
    status, arm = run()
    assert (status['result'], status['attempt']) == ('grasped', 1), status
    # position (level), velocity (servo), position (grasp), position (return), 1 (hand back to xarm_ros2)
    assert arm.modes == [0, 5, 0, 0, 1]
    level, above, grasp, lift, back = arm.moves
    assert np.allclose(level[0], START)
    assert np.allclose(tip(grasp[0]), CENTER + [0, 0, 0.002], atol=0.0025)       # 2 mm above the top face
    assert np.allclose(tip(above[0]), tip(grasp[0]) + [0, 0, 0.02])
    assert (above[1], grasp[1]) == (30.0, 8.0)
    assert axial_error_deg(finger_angle_deg(grasp[0]), LONG_DEG + 90) < 1.0      # fingers across the long side
    assert np.allclose(grasp[0][:3, 2], [0, 0, -1])                              # tool straight down
    assert np.allclose(lift[0][:3, 3], grasp[0][:3, 3] + [0, 0, 0.10])
    assert np.allclose(back[0], START)
    assert arm.gripper_values == [0, 50, 70, 90, 110, 130, 160]                    # open, close in steps, squeeze
    assert status['fsr'] == [400, 200] and status['plate_source'] == 'plane'
    assert np.allclose(status['plate_mm'], CENTER * 1000, atol=1.0)
    assert len(arm.velocities) > 100


def test_default_grasp_squeezes_forty_once_and_keeps_the_gripper_closed():
    status, arm = run(gripper=GripperConfig())
    assert (status['result'], status['attempt']) == ('grasped', 1), status
    assert arm.gripper_values[-2:] == [130, 170]
    assert status['gripper'] == 170
    assert status['gripper_before_squeeze'] == 130 and status['gripper_after_squeeze'] == 170
    assert status['gripper_squeeze_delta'] == 40 and status['squeeze_command_verified'] is True
    assert arm.gripper_values.count(0) == 1  # initial preparation only; no release on completion


def test_lost_grip_after_lift_does_not_retry_or_open_the_gripper():
    fsr = lambda v, arm: held_from(130)(v, arm) if len(arm.moves) < 4 else (0, 0)
    status, arm = run(fsr=fsr, gripper=GripperConfig())
    assert (status['result'], status['attempt']) == ('failed', 1), status
    assert status['message'] == 'no grasp after 1 attempt'
    assert arm.gripper_values[-2:] == [130, 170]
    assert arm.gripper_values.count(0) == 1
    assert arm.modes.count(5) == 1           # a single alignment phase


def test_contact_lost_during_squeeze_does_not_lift_or_reopen():
    fsr = lambda v, arm: (400, 200) if 130 <= v < 170 else (0, 0)
    status, arm = run(fsr=fsr, gripper=GripperConfig(), return_to_start=False)
    assert status['result'] == 'failed'
    assert 'grip lost before lift' in status['message']
    assert status['grip_contact_fsr'] == [400, 200]
    assert status['fsr_before_squeeze'] == [400, 200]
    assert status['fsr_before_lift'] == [0, 0]
    assert status['squeeze_command_verified']
    assert len(arm.moves) == 3  # level, approach, descend; no lift
    assert arm.gripper_values[-1] == 170
    assert arm.gripper_values.count(0) == 1


@pytest.mark.parametrize('held', [(400, 0), (0, 400)])
def test_lift_redistributes_pressure_to_one_pad_without_reporting_a_lost_grip(held):
    fsr = lambda v, arm: held_from(130)(v, arm) if len(arm.moves) < 4 else held
    status, arm = run(fsr=fsr, gripper=GripperConfig(), return_to_start=False)
    assert status['result'] == 'grasped' and status['attempt'] == 1
    assert status['fsr_contact_confirmed'] and status['hold_contact_confirmed']
    assert sum(status['hold_fsr_active']) == 1
    assert arm.gripper_values[-2:] == [130, 170]
    assert arm.gripper_values.count(0) == 1
    assert len(arm.moves) == 4
    assert np.allclose(arm.pose, arm.moves[-1][0])  # retain the lifted pose


def test_failed_final_squeeze_does_not_lift_or_open_the_gripper(monkeypatch):
    original = FakeArm.gripper_set

    def command(arm, value, settle=0.5):
        return False if value == 170 else original(arm, value, settle)

    monkeypatch.setattr(FakeArm, 'gripper_set', command)
    status, arm = run(gripper=GripperConfig())
    assert (status['result'], status['attempt']) == ('failed', 1), status
    assert status['message'] == 'final gripper squeeze command failed'
    assert arm.gripper_values[-1] == 130 and arm.gripper_values.count(0) == 1
    assert len(arm.moves) == 3              # level and descend; no lift after an unacknowledged squeeze


def test_final_squeeze_is_after_main_settling_and_immediately_before_lift(monkeypatch):
    events = []
    original_set, original_move = FakeArm.gripper_set, FakeArm.move_flange

    def command(arm, value, settle=0.5):
        events.append(('gripper', value, arm.clock.time()))
        return original_set(arm, value, settle)

    def move(arm, pose, speed_mm_s, acc_mm_s2=500.0):
        events.append(('move', pose[2, 3] - arm.pose[2, 3], arm.clock.time()))
        return original_move(arm, pose, speed_mm_s, acc_mm_s2)

    monkeypatch.setattr(FakeArm, 'gripper_set', command)
    monkeypatch.setattr(FakeArm, 'move_flange', move)
    grip = GripperConfig()
    status, arm = run(gripper=grip)
    assert status['result'] == 'grasped', status
    contact = next(e for e in events if e[:2] == ('gripper', 130))
    squeeze_index = next(i for i, e in enumerate(events) if e[:2] == ('gripper', 170))
    squeeze, lift = events[squeeze_index:squeeze_index + 2]
    assert squeeze[2] - contact[2] >= grip.close_settle
    assert lift[0] == 'move' and lift[1] == pytest.approx(.10)
    assert lift[2] - squeeze[2] >= grip.final_squeeze_settle


def test_unconfirmed_squeeze_readback_prevents_lifting(monkeypatch):
    monkeypatch.setattr(FakeArm, 'gripper_position', lambda arm: 130)
    status, arm = run(gripper=GripperConfig())
    assert status['result'] == 'failed' and 'readback mismatch' in status['message']
    assert status['squeeze_command_verified'] is False
    assert arm.gripper_values[-1] == 170 and arm.gripper_values.count(0) == 1
    assert len(arm.moves) == 3


def test_no_fsr_contact_does_not_squeeze_or_lift():
    status, arm = run(fsr=held_from(10_000), gripper=GripperConfig(samples=4), return_to_start=False)
    assert status['result'] == 'failed' and status['attempt'] == 1
    assert status['fsr_contact_confirmed'] is False
    assert 'gripper_before_squeeze' not in status
    assert arm.gripper_values == [0, 50, 60, 70, 80, 90]
    assert len(arm.moves) == 3


def test_switching_contact_takes_up_slack_before_another_nudge():
    def fsr(value, arm):
        if value < 130:
            return (0, 0)
        if value >= 132:
            return (400, 200)
        return (0, 200) if len(arm.moves) > 3 else (400, 0)

    status, arm = run(fsr=fsr, gripper=GripperConfig())
    assert status['result'] == 'grasped', status
    assert status['nudges'] == 1 and status['balance_close_increment'] == 2
    assert arm.gripper_values[-3:] == [130, 132, 172]
    assert status['gripper_before_squeeze'] == 132 and status['gripper_after_squeeze'] == 172


@pytest.mark.parametrize('finger', [0, 1])
def test_persistent_contact_takes_up_slack_only_after_measured_unloading(finger):
    def fsr(value, arm):
        if value < 130:
            return (0, 0)
        if value >= 132:
            return (400, 200)
        readings = [0, 0]
        readings[finger] = 350 if len(arm.moves) > 3 else 500
        return tuple(readings)

    status, arm = run(fsr=fsr, gripper=GripperConfig())
    assert status['result'] == 'grasped', status
    assert status['nudges'] == 1 and status['balance_close_increment'] == 2
    assert arm.gripper_values[-3:] == [130, 132, 172]
    assert status['nudge_pressure_before'][finger] == 500
    assert status['nudge_pressure_after'][finger] == 350
    assert status['gripper_squeeze_delta'] == 40


def test_switching_contact_above_firmware_cutoff_does_not_close_further():
    def fsr(value, arm):
        if value < 130:
            return (0, 0)
        if len(arm.moves) > 4:
            return (400, 200)
        return (0, 700) if len(arm.moves) > 3 else (400, 0)

    status, arm = run(fsr=fsr, gripper=GripperConfig())
    assert status['result'] == 'grasped', status
    assert status['nudges'] == 2 and 'balance_close_increment' not in status
    assert arm.gripper_values[-2:] == [130, 170]


def test_a_tilted_start_is_levelled_first():
    start = transform(rpy_matrix(np.radians(174), np.radians(4), np.radians(10)), [0.40, 0.0, 0.35])
    status, arm = run(start=start)
    assert status['result'] == 'grasped', status
    level = arm.moves[0][0]
    assert np.allclose(level[:3, 3], start[:3, 3]) and np.allclose(level[:3, 2], [0, 0, -1])
    assert np.allclose(arm.moves[-1][0], start)


def test_dry_run_only_reads_the_arm():
    status, arm = run(dry_run=True, align_timeout=3.0)      # FakeArm(read_only) fails the run on any motion call
    assert status['result'] == 'stopped' and status['dry_run'], status
    assert arm.modes == [] and arm.moves == [] and arm.velocities == [] and arm.gripper_values == []
    assert np.allclose(status['plate_mm'], CENTER * 1000, atol=0.5)
    expected = (CENTER + [0, 0, 0.08] - tip(START)) * 1000
    assert np.allclose(status['error_mm'], expected, atol=0.5)
    assert np.allclose(status['target_mm'], (CENTER + [0, 0, 0.08]) * 1000, atol=0.5)
    assert np.linalg.norm(status['command_mm_s']) > 10.0 and status['observations'] > 10


def test_grasp_disabled_stops_at_the_hover_pose():
    status, arm = run(grasp=False, return_to_start=False)
    assert status['result'] == 'aligned', status
    assert arm.modes == [0, 5, 1] and arm.gripper_values == [] and len(arm.moves) == 1
    error = tip(arm.pose) - (CENTER + [0, 0, 0.08])
    assert np.linalg.norm(error[:2]) < 0.0025 and abs(error[2]) < 0.0045
    assert axial_error_deg(finger_angle_deg(arm.pose), LONG_DEG + 90) < 2.0


@pytest.mark.parametrize('first, sign', [((300, 50), 1.0), ((100, 200), -1.0)])
def test_one_finger_contact_nudges_along_the_finger_axis(first, sign):
    # one finger presses from gripper value 90 on; both once the arm has moved after the grasp move (the nudge)
    fsr = lambda v, arm: (400, 200) if len(arm.moves) > 3 else (first if v >= 90 else (0, 0))
    status, arm = run(fsr=fsr, gripper=GripperConfig(**TWO_FINGERS))
    assert status['result'] == 'grasped', status
    grasp, nudged = arm.moves[2][0], arm.moves[3][0]
    # FSR1 alone: +1 mm along flange y (Camera-Edge's nudge, as it moved the arm); FSR2 alone: -1 mm
    assert np.allclose(nudged[:3, 3] - grasp[:3, 3], sign * 0.001 * grasp[:3, 1])
    assert np.allclose(nudged[:3, :3], grasp[:3, :3]) and arm.moves[3][1] == 30.0
    assert arm.gripper_values == [0, 50, 70, 90, 120]  # keep the opening fixed while unloading one pad


@pytest.mark.parametrize('tare', [5, 0])
def test_idle_fsr_offset_is_tared(tare):
    # FSR1 idles at ~360 on the arm, above its 250 touch level. Counted from the open-gripper baseline, the close
    # runs as without the offset; raw (tare 0), false contact is bounded instead of walking sideways indefinitely.
    fsr = lambda v, arm: tuple(np.add((360, 0), held_from(130)(v, arm)))
    status, arm = run(fsr=fsr, gripper=GripperConfig(**dict(TWO_FINGERS, tare_samples=tare)))
    if tare:
        assert status['result'] == 'grasped', status
        assert status['fsr_baseline'] == [360, 0]
        assert arm.gripper_values == [0, 50, 70, 90, 110, 130, 160] and len(arm.moves) == 5
    else:
        assert status['result'] == 'failed' and 'nudge limits' in status['message']
        assert max(arm.gripper_values) == 50


def test_both_touching_with_only_one_firm_contact_unloads_the_firm_finger():
    fsr = lambda v, arm: ((400, 200) if len(arm.moves) > 3 else (400, 130)) if v >= 90 else (0, 0)
    status, arm = run(fsr=fsr, gripper=GripperConfig(**TWO_FINGERS))
    assert status['result'] == 'grasped', status
    grasp, nudge = arm.moves[2][0], arm.moves[3][0]
    assert np.allclose(nudge[:3, 3] - grasp[:3, 3], .001 * grasp[:3, 1])
    assert arm.gripper_values == [0, 50, 70, 90, 120]  # do not squeeze further while centring


def test_unresponsive_finger_stops_balancing_without_further_squeezing():
    fsr = lambda v, arm: (400, 0) if v >= 90 else (0, 0)
    status, arm = run(fsr=fsr, gripper=GripperConfig(max_nudges=3, max_nudge_mm=10, **TWO_FINGERS))
    assert status['result'] == 'failed' and 'nudge limits' in status['message']
    assert status['nudges'] == 3 and max(arm.gripper_values) == 90
    assert len(arm.moves) == 6  # level, two descent moves, three nudges; no lift


def test_finger_mapping_can_be_reversed_for_rewired_pads():
    fsr = lambda v, arm: (400, 200) if len(arm.moves) > 3 else ((300, 50) if v >= 90 else (0, 0))
    status, arm = run(fsr=fsr, gripper=GripperConfig(fsr_nudge_sign=(-1, 1), **TWO_FINGERS))
    assert status['result'] == 'grasped', status
    grasp, nudge = arm.moves[2][0], arm.moves[3][0]
    assert np.allclose(nudge[:3, 3] - grasp[:3, 3], -.001 * grasp[:3, 1])


def test_a_dead_fsr_is_left_out():
    # on the arm FSR2 read 0 with the plate held and FSR1 rose ~40 over its ~354 idle: FSR1 alone decides, and
    # FSR1 pressing without FSR2 is no reason to nudge
    fsr = lambda v, arm: (354 + (40 if v >= 130 else 0), 0)
    status, arm = run(fsr=fsr)
    assert status['result'] == 'grasped', status
    assert arm.gripper_values == [0, 50, 70, 90, 110, 130, 160] and len(arm.moves) == 5
    status, arm = run(fsr=fsr, gripper=GripperConfig(use_fsr=True, target_pressure=(15, 15), stop_pressure=(25, 25), samples=12),
                      max_retries=0)
    assert status['result'] == 'failed', status


def test_a_falling_adc_sensor_can_be_configured_independently():
    # Supports an opposite divider orientation without changing the other finger's pressure calculation.
    fsr = lambda v, arm: (530, 40) if v >= 130 else (180, 95)
    grip = GripperConfig(use_fsr=True, fsr_polarity=(1, -1), target_pressure=(100, 15),
                         stop_pressure=(200, 30), fsr_samples=5, tare_samples=25)
    status, arm = run(fsr=fsr, gripper=grip)
    assert status['result'] == 'grasped', status
    assert status['fsr_baseline'] == [180, 95]
    assert status['fsr_pressure'] == [350, 55]
    assert len(arm.moves) == 5                 # balanced contact needs no nudge


def fsr_sampler(readings, count=5):
    clock = FakeClock()
    arm = FakeArm(clock, START)
    reads = iter(readings)
    arm.gripper_fsr = lambda: next(reads)
    runner = ServoRunner(arm, MOUNT, ServoConfig(gripper=GripperConfig(fsr_samples=count)),
                         clock=clock.time, sleep=clock.sleep)
    return runner


def test_fsr_median_rejects_an_isolated_false_contact():
    runner = fsr_sampler([(180, 95), (185, 94), (800, 1), (179, 96), (181, 95)])
    assert runner._sample_fsr(5) == (181, 95)


def test_a_missing_tare_cannot_be_treated_as_zero_pressure():
    runner = fsr_sampler([(None, None)] * 5)
    runner.config.gripper.tare_samples = 5
    with pytest.raises(RunFailed, match='0/5 valid replies'):
        runner._fsr_baseline()


def test_fsr_sampling_rejects_a_majority_of_bad_reads():
    runner = fsr_sampler([(180, 95), (None, None), (180, 95), (1024, 95), (180, None)])
    with pytest.raises(RunFailed, match='2/5 valid replies'):
        runner._sample_fsr(5)


def test_final_squeeze_respects_the_calibrated_close_limit():
    status, arm = run(fsr=held_from(130), gripper=GripperConfig(use_fsr=True, final_squeeze=30, max_close=140))
    assert status['result'] == 'grasped', status
    assert max(arm.gripper_values) == 140



def test_without_fsrs_the_gripper_closes_to_its_position_plus_the_offset():
    status, arm = run(fsr=lambda v, arm: (354, 0), gripper=GripperConfig(use_fsr=False))
    assert status['result'] == 'grasped', status
    assert arm.gripper_values == [0, 350] and status['gripper_before'] == 0
    grasp, lift = arm.moves[2][0], arm.moves[3][0]
    assert np.allclose(lift[:3, 3], grasp[:3, 3] + [0, 0, 0.10])


def test_retries_then_fails():
    status, arm = run(fsr=held_from(10_000), max_retries=1, gripper=GripperConfig(use_fsr=True, samples=4))
    assert (status['result'], status['attempt']) == ('failed', 2), status
    assert 'no grasp after 2 attempts' in status['message']
    assert arm.modes == [0, 5, 0, 0, 5, 0, 0, 1]            # ..., retry lift, servo again, grasp, return, hand back
    assert arm.gripper_values.count(0) == 2                  # opened at the start and before the retry
    assert np.allclose(arm.moves[-1][0], START)


def test_lost_plate_fails_without_moving():
    status, arm = run(camera={'visible': False})
    assert status['result'] == 'failed' and 'no plate detection for 5.0 s' in status['message'], status
    assert len(arm.moves) == 1 and arm.velocities == [] and arm.modes[-1] == 1     # level only; no return


def test_a_plate_of_the_wrong_size_is_rejected():
    status, arm = run(camera={'size': (0.20, 0.13)})
    assert status['result'] == 'failed' and status['rejected'].startswith('size 20'), status


@pytest.mark.parametrize('camera, result', [
    ({'conf': 0.03}, 'aligned'),                                  # weak, but the depth measures the plate
    ({'conf': 0.03, 'size': (0.15, 0.085)}, 'failed'),            # weak and 17 % long: rejected...
    ({'conf': 0.9, 'size': (0.15, 0.085)}, 'aligned'),            # ...a confident detection may be 35 % off
    ({'conf': 0.03, 'with_depth': False}, 'failed'),              # weak without depth: no size to check
])
def test_weak_detections_count_only_with_the_plates_size(camera, result):
    status, arm = run(camera=camera, grasp=False, return_to_start=False)
    assert status['result'] == result, status
    if result == 'failed':
        assert arm.velocities == [] and 'conf 0.03' in status['rejected']


def test_target_beyond_max_travel_fails_before_moving():
    status, arm = run(max_travel=0.05)
    assert status['result'] == 'failed' and 'max_travel' in status['message'], status
    assert arm.velocities == [] and len(arm.moves) == 1 and arm.modes[-1] == 1


def test_abort_stops_where_it_is():
    status, arm = run(hook=lambda t, runner: t > 2.0 and runner.abort())
    assert status['result'] == 'aborted', status
    assert len(arm.moves) == 1 and arm.modes == [0, 5, 1]     # no return to the start, handed back
    assert arm.velocities and not np.allclose(tip(arm.pose), tip(START))


def test_start_while_running_is_refused():
    answers = []
    status, arm = run(dry_run=True, align_timeout=1.0, hook=lambda t, runner: answers.append(runner.start()))
    assert answers and not any(answers)


def test_prepare_runs_first_and_its_failure_fails_the_run():
    calls = []
    status, arm = run(dry_run=True, align_timeout=1.0, prepare=lambda check_abort: calls.append(check_abort()))
    assert calls == [None] and status['result'] == 'stopped'

    def broken(check_abort):
        raise RuntimeError('camera hub down')
    status, arm = run(prepare=broken)
    assert status['result'] == 'failed' and status['message'] == 'RuntimeError: camera hub down'
    assert arm.connects == 0 and arm.modes == []


# -- the wrist F/T sensor ----------------------------------------------------------------------------
WITH_FT = dict(TWO_FINGERS, use_ft=True, ft_samples=1)


def wrench(lateral=0.0, fz=0.0, contact=lambda arm: True):
    """Fy `lateral` while `contact(arm)`, plus `fz` once the plate is lifted (the lift is the 50 mm/s move)."""
    def ft(arm):
        lifted = arm.moves and arm.moves[-1][1] == 50.0
        return [0.0, lateral if contact(arm) and not lifted else 0.0, fz if lifted else 0.0, 0.0, 0.0, 0.0]
    return ft


# FSR1 alone presses from gripper value 90 on (FSRs: nudge +1 mm), both once the arm has nudged
ONE_PAD = lambda v, arm: (400, 200) if len(arm.moves) > 3 else ((300, 50) if v >= 90 else (0, 0))
PUSHING = lambda arm: arm.gripper_values and arm.gripper_values[-1] >= 90 and len(arm.moves) <= 3


def nudge_mm(arm):
    grasp, nudged = arm.moves[2][0], arm.moves[3][0]
    return float(np.dot(nudged[:3, 3] - grasp[:3, 3], grasp[:3, 1]) * 1000)


def test_ft_off_logs_a_disagreement_but_nudges_as_the_fsrs_say():
    status, arm = run(fsr=ONE_PAD, ft=wrench(-3.0, contact=PUSHING), gripper=GripperConfig(**TWO_FINGERS))
    assert status['result'] == 'grasped', status
    assert nudge_mm(arm) == pytest.approx(1.0) and arm.halts == 0 and arm.ft_zeroes == 1
    assert arm.gripper_values == [0, 50, 70, 90, 120]
    assert 'ft' in status and 'ft_hold_delta_n' in status and status['ft_hold'] is False


def test_ft_off_balance_decision_is_logged_as_would(caplog):
    caplog.set_level('INFO', logger='visual_servoing.servo')
    run(fsr=ONE_PAD, ft=wrench(-3.0, contact=PUSHING), gripper=GripperConfig(**TWO_FINGERS))
    lines = [r.getMessage() for r in caplog.records if r.getMessage().startswith('F/T')]
    assert any('disagree: opposite sides, would: unload FSR2' in line for line in lines), lines
    for phase in ('F/T zero', 'F/T grasp height', 'F/T balance', 'F/T contact confirmed', 'F/T before squeeze',
                  'F/T before lift', 'F/T after lift'):
        assert any(line.startswith(phase) for line in lines), phase


def test_ft_overrides_the_fsr_side_when_tracking():
    status, arm = run(fsr=ONE_PAD, ft=wrench(-3.0, fz=-1.5, contact=PUSHING), gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert nudge_mm(arm) == pytest.approx(-1.0)          # towards FSR1: F/T says FSR2 pushes, at the 2-step cap
    assert status["track_fsr"] == 2 and arm.gripper_values == [0, 50, 70, 90, 110, 140]   # step 20
    assert status['ft_hold'] is True and status['ft_hold_delta_n'] == pytest.approx(1.5)


@pytest.mark.parametrize('lateral, slide', [(1.5, 0.65), (3.0, 1.0), (1.2, 0.56)])
def test_ft_tracking_slides_more_the_harder_the_pad_pushes(lateral, slide):
    status, arm = run(fsr=ONE_PAD, ft=wrench(lateral, fz=-1.5, contact=PUSHING), gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert nudge_mm(arm) == pytest.approx(slide) and status['track_fsr'] == 1


def test_ft_without_lateral_force_closes_instead_of_nudging():
    fsr = lambda v, arm: (400, 200) if v >= 110 else ((300, 50) if v >= 90 else (0, 0))
    status, arm = run(fsr=fsr, ft=wrench(fz=-1.5), gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert len(arm.moves) == 5 and arm.gripper_values[:5] == [0, 50, 70, 90, 110]   # no nudge move


def test_ft_only_imbalance_nudges_before_the_fsrs_see_it():
    fsr = lambda v, arm: (400, 200) if len(arm.moves) > 3 else (0, 0)
    pushing = lambda arm: arm.gripper_values and arm.gripper_values[-1] >= 70 and len(arm.moves) <= 3
    status, arm = run(fsr=fsr, ft=wrench(2.0, fz=-1.5, contact=pushing), gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert nudge_mm(arm) == pytest.approx(0.8) and arm.gripper_values == [0, 50, 70, 90, 120]   # tracked from 70, step 20


def test_ft_contact_stops_the_descent_and_backs_off():
    touching = lambda arm: tip(arm.pose)[2] < CENTER[2] + 0.006
    ft = lambda arm: [0.0, 0.0, 8.0 if touching(arm) else 0.0] + [0.0] * 3 if not (
        arm.moves and arm.moves[-1][1] == 50.0) else [0.0, 0.0, -1.5, 0.0, 0.0, 0.0]
    status, arm = run(ft=ft, gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert arm.halts == 1 and status['ft_contact'] is True
    assert status['ft_contact_tip_mm'][2] == pytest.approx((CENTER[2] + 0.006 + 0.002) * 1000, abs=0.6)


def test_ft_descent_without_contact_reaches_the_grasp_height():
    status, arm = run(ft=wrench(fz=-1.5), gripper=GripperConfig(**WITH_FT))
    assert status['result'] == 'grasped', status
    assert arm.halts == 0 and status['ft_contact'] is False
    assert arm.moves[2][1] == 8.0 and status['grasp_tip_mm'][2] == pytest.approx((CENTER[2] + 0.002) * 1000, abs=0.1)


@pytest.mark.parametrize('use_ft, result', [(True, 'failed'), (False, 'grasped')])
def test_ft_hold_check_decides_only_with_use_ft(use_ft, result):
    status, arm = run(ft=wrench(), gripper=GripperConfig(**dict(TWO_FINGERS, use_ft=use_ft, ft_samples=1)))
    assert status['result'] == result, status
    assert status['ft_hold'] is False and status['hold_contact_confirmed'] is True


@pytest.mark.parametrize('use_ft', [True, False])
def test_ft_read_failure_fails_only_with_use_ft(use_ft):
    status, arm = run(ft=lambda arm: None, gripper=GripperConfig(**dict(TWO_FINGERS, use_ft=use_ft, ft_samples=1)))
    if use_ft:
        assert status['result'] == 'failed' and 'F/T' in status['message'], status
    else:
        assert status['result'] == 'grasped', status
        assert 'F/T sampling failed' in status['ft_error'] and 'ft' not in status


def held_off_centre(offset_mm=6.0):
    """A plate held still `offset_mm` towards FSR1 (+ flange y of the grasp pose; a + slide unloads FSR1). One
    gripper unit moves each jaw 0.05 mm, a centred plate is touched at 180, a pad presses 3 N and 250 ADC per mm."""
    def penetrations(arm):
        grasp = arm.moves[2][0]
        slide = float(np.dot(arm.pose[:3, 3] - grasp[:3, 3], grasp[:3, 1]) * 1000)
        gap = (180 - (arm.gripper_values[-1] if arm.gripper_values else 0)) * 0.05   # each jaw outside the plate
        return max(0.0, offset_mm - slide - gap), max(0.0, slide - offset_mm - gap)

    def fsr(value, arm):
        if len(arm.moves) < 3:
            return (0, 0)
        return tuple(int(min(1023, 250 * p)) for p in penetrations(arm))

    def ft(arm):
        lifted = arm.moves and arm.moves[-1][1] == 50.0
        pad1, pad2 = penetrations(arm) if len(arm.moves) >= 3 else (0.0, 0.0)
        return [0.0, 0.0 if lifted else 3.0 * (pad1 - pad2), -1.5 if lifted else 0.0, 0.0, 0.0, 0.0]
    return fsr, ft


def test_a_plate_held_off_centre_runs_out_of_nudges_without_tracking():
    fsr, ft = held_off_centre()
    status, arm = run(fsr=fsr, ft=ft, gripper=GripperConfig(fsr_samples=1, ft_samples=1, nudge_speed=30.0))
    assert status['result'] == 'failed' and 'nudge limits' in status['message'], status


def test_tracking_closes_on_a_plate_held_off_centre():
    fsr, ft = held_off_centre()
    status, arm = run(fsr=fsr, ft=ft, gripper=GripperConfig(fsr_samples=1, ft_samples=1, nudge_speed=30.0,
                                                            use_ft=True, ft_lateral_deadband_n=0.5))
    assert status['result'] == 'grasped', status
    assert 5.0 < status['track_walk_mm'] < 7.5      # ~ the 6 mm offset
    assert max(arm.gripper_values[:-1]) < 220     # closed on the plate, not squeezing it


def test_tracking_stops_at_its_slide_limit():
    fsr, ft = held_off_centre(12.0)
    status, arm = run(fsr=fsr, ft=ft, gripper=GripperConfig(fsr_samples=1, ft_samples=1, nudge_speed=30.0,
                                                            use_ft=True, ft_lateral_deadband_n=0.5, max_close=500))
    assert status['result'] == 'failed' and 'ft_track_max_mm' in status['message'], status


# -- probe contact: find the plate top by touch ------------------------------------------------------
GRIP_DEPTH = 0.0075


def probe_ft(top=CENTER[2], fz=-1.5, touches=lambda arm: True, spike=None):
    """+8 N Fz while the closed (550) fingertips are below `top` and `touches(arm)`; `fz` once lifted.
    `spike(arm)` True: a one-read noise spike."""
    def ft(arm):
        if arm.moves and arm.moves[-1][1] == 50.0:
            return [0.0, 0.0, fz, 0.0, 0.0, 0.0]
        closed = arm.gripper_values and arm.gripper_values[-1] == 550
        if closed and spike is not None and spike(arm):
            return [0.0, 0.0, 8.0, 0.0, 0.0, 0.0]
        pressing = closed and touches(arm) and tip(arm.pose)[2] < top
        return [0.0, 0.0, 8.0 if pressing else 0.0, 0.0, 0.0, 0.0]
    return ft


def probe_run(ft, **config):
    gripper = config.pop('gripper', GripperConfig(**dict(TWO_FINGERS, ft_samples=1)))
    return run(fsr=held_from(130), ft=ft, gripper=gripper, probe_contact=True, **config)


@pytest.mark.parametrize('top', [CENTER[2], CENTER[2] - 0.004])
def test_probe_touches_the_plate_top_then_opens_and_lowers_a_quarter_plate(top):
    status, arm = probe_run(probe_ft(top=top))
    assert (status['result'], status['attempt']) == ('grasped', 1), status
    assert arm.gripper_values[:4] == [0, 550, 0, 50]                  # open, probe, reopen, close routine
    assert arm.halts == 1 and arm.ft_zeroes == 2                      # zero at the hover, again after reopening
    assert status['probe_contact'] is True
    contact = status['probe_contact_tip_mm'][2]
    assert contact == pytest.approx(top * 1000, abs=0.5)
    assert status['grasp_tip_mm'][2] == pytest.approx(contact - GRIP_DEPTH * 1000, abs=0.05)
    level, above, probe, up, grasp = arm.moves[:5]
    assert tip(above[0])[2] == pytest.approx(CENTER[2] + 0.02) and above[1] == 30.0
    assert probe[1] == 8.0 and tip(probe[0])[2] == pytest.approx(CENTER[2] - 0.010)
    assert tip(up[0])[2] * 1000 == pytest.approx(contact + 5.0, abs=0.05)
    assert np.allclose(tip(grasp[0])[:2], tip(above[0])[:2])           # straight down over the plate


def test_probe_ignores_a_single_fz_spike():
    reads = []
    spike = lambda arm: (reads.append(1) or len(reads)) == 3          # third read during the probe only
    status, arm = probe_run(probe_ft(spike=spike))
    assert status['result'] == 'grasped', status
    assert status['probe_contact_tip_mm'][2] == pytest.approx(CENTER[2] * 1000, abs=0.5)


def test_probe_without_contact_lifts_reopens_and_probes_again():
    touches = lambda arm: arm.gripper_values.count(550) >= 2          # nothing there on the first attempt
    status, arm = probe_run(probe_ft(touches=touches), max_retries=1)
    assert (status['result'], status['attempt']) == ('grasped', 2), status
    assert arm.gripper_values[:5] == [0, 550, 0, 550, 0]               # probe, retry open, probe, reopen
    assert arm.modes.count(5) == 2                                     # aligned twice


def test_probe_without_contact_on_every_attempt_fails_without_closing():
    status, arm = probe_run(probe_ft(touches=lambda arm: False), max_retries=1)
    assert (status['result'], status['attempt']) == ('failed', 2), status
    assert status['message'] == 'no grasp after 2 attempts' and status['probe_contact'] is False
    assert set(arm.gripper_values) == {0, 550} and arm.halts == 0


def test_probe_needs_the_ft_sensor():
    status, arm = probe_run(lambda arm: None)
    assert status['result'] == 'failed' and 'F/T' in status['message'], status
    assert arm.gripper_values == [0, 550]                             # never closed on the plate


def test_probe_waits_for_the_gripper_before_moving_the_arm():
    status, arm = probe_run(probe_ft())
    assert status['result'] == 'grasped', status
    events = arm.events
    close = next(i for i, e in enumerate(events) if e[1:] == ('gripper', 550))
    reopen = next(i for i, e in enumerate(events) if i > close and e[1:] == ('gripper', 0))
    after_close = next(e for e in events[close + 1:] if e[1] == 'move')
    after_open = next(e for e in events[reopen + 1:] if e[1] == 'move')
    assert after_close[0] - events[close][0] >= 550 / 80.0 + 1.0 - 1e-6        # 0 -> 550 at 80 units/s + 1 s
    assert after_open[0] - events[reopen][0] >= 550 / 80.0 + 1.0 - 1e-6
    assert status['probe_gripper_wait_s'] == pytest.approx(550 / 80.0 + 1.0, abs=0.01)


def test_probe_retry_waits_for_the_gripper_to_open():
    touches = lambda arm: arm.gripper_values.count(550) >= 2
    status, arm = probe_run(probe_ft(touches=touches), max_retries=1)
    assert (status['result'], status['attempt']) == ('grasped', 2), status
    closes = [i for i, e in enumerate(arm.events) if e[1:] == ('gripper', 550)]
    retry_open = next(e for e in arm.events[closes[0] + 1:] if e[1:] == ('gripper', 0))
    assert arm.events[closes[1]][0] - retry_open[0] >= 550 / 80.0 + 1.0 - 1e-6


# -- probe tilt: level the closed fingertips with a leaning plate by tilt-and-retouch --------------------------
FINGER_HALF = 0.012     # closed fingertips sit +-12 mm from the tip centre along flange y
STIFFNESS = 3000.0      # N/m of plate contact per fingertip


def tilted_plate_ft(lean_deg, top=CENTER[2], gravity_nm_per_rad=0.2, held_fz=-1.5):
    """F/T of the closed (550) fingertips on a plate whose top rises by tan(lean) per metre along the grasp's finger
    axis (flange y at the grasp yaw). Each fingertip below the plate pushes up with STIFFNESS * depth: Fz is the sum
    (sign as probe_ft: +), Tx = -y * F for the fingertip at flange y. Tilting adds a gravity torque the probe has to
    re-baseline away. Once lifted (50 mm/s), Fz reads `held_fz`."""
    slope = np.tan(np.radians(lean_deg))
    ref = {}

    def ft(arm):
        if arm.moves and arm.moves[-1][1] == 50.0:
            return [0.0, 0.0, held_fz, 0.0, 0.0, 0.0]
        pose = arm.pose
        level = rpy_matrix(np.pi, 0.0, np.arctan2(pose[1, 0], pose[0, 0]))
        tilt = np.arctan2(*(level.T @ pose[:3, :3])[[2, 1], [1, 1]])       # roll of the flange from tool-down
        gravity = gravity_nm_per_rad * tilt
        if not (arm.gripper_values and arm.gripper_values[-1] == 550):
            return [0.0, 0.0, 0.0, gravity, 0.0, 0.0]
        centre = tip(pose)
        axis = ref.setdefault('axis', pose[:3, 1].copy())                  # the finger axis at the first touch
        fz, tx = 0.0, 0.0
        for y in (FINGER_HALF, -FINGER_HALF):
            point = (pose @ np.append(MOUNT.flange_T_tip[:3, 3] + [0.0, y, 0.0], 1.0))[:3]
            surface = top + slope * float(np.dot(point[:2] - centre[:2], axis[:2]) + np.dot(centre[:2] - CENTER[:2],
                                                                                              axis[:2]))
            depth = surface - point[2]
            if depth > 0:
                fz += STIFFNESS * depth
                tx += -y * STIFFNESS * depth
        return [0.0, 0.0, fz, tx + gravity, 0.0, 0.0]
    return ft


def tilt_run(lean_deg, **config):
    gripper = config.pop('gripper', GripperConfig(**dict(TWO_FINGERS, ft_samples=1, ft_tilt_deadband_nm=0.005)))
    return run(fsr=held_from(130), ft=tilted_plate_ft(lean_deg), gripper=gripper, probe_contact=True,
               probe_tilt=True, **config)


def tool_tilt_deg(pose):
    """Roll of the flange about its own x axis from tool-down at the same yaw (+: lowers the flange +y finger)."""
    level = rpy_matrix(np.pi, 0.0, np.arctan2(pose[1, 0], pose[0, 0]))
    rel = level.T @ pose[:3, :3]
    return float(np.degrees(np.arctan2(rel[2, 1], rel[1, 1])))


@pytest.mark.parametrize('lean', [6.0, -9.0, 3.3])
def test_probe_tilt_levels_the_fingertips_and_grasps_at_that_tilt(lean):
    status, arm = tilt_run(lean)
    assert (status['result'], status['attempt']) == ('grasped', 1), status
    assert status['probe_tilt_result'] == 'balanced', status
    # the finger axis runs uphill by `lean`: the tool tilts by -lean about flange x so the fingertip line follows it
    assert abs(status['probe_tilt_deg'] + lean) <= 0.5, status
    assert status['probe_tilt_touches'] >= 2
    closing = next(i for i, v in enumerate(arm.gripper_values) if v == 50)
    grasp_move = [m for m in arm.moves if m[1] == 8.0][-1][0]               # the last final-speed move: the grasp
    assert tool_tilt_deg(grasp_move) == pytest.approx(status['probe_tilt_deg'], abs=0.05)
    lift = next(m for m in arm.moves if m[1] == 50.0)[0]
    assert tool_tilt_deg(lift) == pytest.approx(status['probe_tilt_deg'], abs=0.05)   # the lift keeps the tilt
    assert closing >= 3 and arm.ft_zeroes == 2


def test_probe_tilt_on_a_flat_plate_touches_once_and_stays_level():
    status, arm = tilt_run(0.0)
    assert status['result'] == 'grasped', status
    assert status['probe_tilt_result'] == 'balanced' and status['probe_tilt_deg'] == 0.0
    assert status['probe_tilt_touches'] == 1
    assert all(abs(tool_tilt_deg(m[0])) < 1e-6 for m in arm.moves)


def test_probe_tilt_beyond_the_cap_falls_back_to_the_level_grasp():
    status, arm = tilt_run(20.0, probe_tilt_max_deg=8.0)
    assert status['result'] == 'grasped', status
    assert status['probe_tilt_result'] == 'cap' and status['probe_tilt_deg'] == 0.0
    grasp_move = [m for m in arm.moves if m[1] == 8.0][-1][0]
    assert abs(tool_tilt_deg(grasp_move)) < 1e-6
    assert max(abs(tool_tilt_deg(m[0])) for m in arm.moves) <= 8.0 + 1e-6
    # the fallback grasp height is the first, level touch
    assert status['grasp_tip_mm'][2] == pytest.approx(status['probe_contact_tip_mm'][2] - GRIP_DEPTH * 1000, abs=0.05)


def test_probe_tilt_never_dips_the_tip_while_reorienting():
    """Every orientation change happens with the fingertip centre above where it touched."""
    status, arm = tilt_run(7.0)
    assert status['result'] == 'grasped', status
    first_touch = status['probe_first_contact_tip_mm'][2] / 1000.0
    prev = None
    for pose, speed in arm.moves:
        if prev is not None and abs(tool_tilt_deg(pose) - tool_tilt_deg(prev)) > 1e-6:
            assert tip(prev)[2] > first_touch - 0.003 and tip(pose)[2] > first_touch - 0.003
        prev = pose


def test_probe_tilt_off_keeps_the_level_probe():
    status, arm = run(fsr=held_from(130), ft=tilted_plate_ft(6.0),
                      gripper=GripperConfig(**dict(TWO_FINGERS, ft_samples=1)), probe_contact=True)
    assert status['result'] == 'grasped', status
    assert 'probe_tilt_deg' not in status
    assert all(abs(tool_tilt_deg(m[0])) < 1e-6 for m in arm.moves)


def test_retry_after_a_tilted_grasp_relevels_before_aligning():
    fsr_ok = held_from(130)
    servo_tilts = []

    def fsr(value, arm):                       # first close never reaches pressure: the attempt fails
        if arm.gripper_values.count(550) < 2:
            return (0, 0)
        return fsr_ok(value, arm)

    def hook(t, runner):
        status = runner.status()
        if status.get('phase') == 'servoing' and status.get('attempt') == 2:
            servo_tilts.append(tool_tilt_deg(runner.arm.pose))
    status, arm = run(fsr=fsr, ft=tilted_plate_ft(6.0), hook=hook,
                      gripper=GripperConfig(**dict(TWO_FINGERS, ft_samples=1, ft_tilt_deadband_nm=0.005,
                                                   samples=20)),
                      probe_contact=True, probe_tilt=True, max_retries=1)
    assert (status['result'], status['attempt']) == ('grasped', 2), status
    assert servo_tilts and max(abs(v) for v in servo_tilts) < 1e-6      # the second servo starts straight down
    assert abs(status['probe_tilt_deg'] + 6.0) <= 0.5                   # and the second probe tilts again


def test_nudge_slides_along_the_tilted_finger_axis():
    from visual_servoing.arm import flange_for_tip
    clock = FakeClock()
    pose = flange_for_tip(CENTER, np.radians(30.0), MOUNT.flange_T_tip, np.radians(-8.0))
    arm = FakeArm(clock, pose)
    arm.connect()
    arm.position_mode()
    runner = ServoRunner(arm, MOUNT, ServoConfig(dry_run=False), clock=clock.time, sleep=clock.sleep)
    runner._nudge(2.0)
    moved = arm.moves[-1][0]
    assert np.allclose(moved[:3, 3] - pose[:3, 3], 0.002 * pose[:3, 1])
    assert abs(moved[2, 3] - pose[2, 3]) > 0.0002                       # the tilted finger axis leaves the horizontal
    assert np.allclose(moved[:3, :3], pose[:3, :3])


def test_flange_for_tip_tilt_lowers_the_plus_side_finger_about_the_tip():
    from visual_servoing.arm import flange_for_tip
    level = flange_for_tip(CENTER, 0.4, MOUNT.flange_T_tip)
    tilted = flange_for_tip(CENTER, 0.4, MOUNT.flange_T_tip, np.radians(5.0))
    assert np.allclose(tip(tilted), CENTER)
    finger = MOUNT.flange_T_tip.copy()
    finger[1, 3] += 0.012
    assert (tilted @ finger)[2, 3] < (level @ finger)[2, 3] - 0.001
    assert tool_tilt_deg(tilted) == pytest.approx(5.0)

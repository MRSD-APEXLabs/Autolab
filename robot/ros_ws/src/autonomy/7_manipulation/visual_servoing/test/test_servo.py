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


def run(start=START, fsr=held_from(130), prepare=None, hook=None, camera=None, **config):
    config.setdefault('dry_run', False)
    config.setdefault('return_to_start', True)  # exercise the optional return in the original simulations
    config.setdefault('gripper', GripperConfig(**LEGACY_GRIPPER))
    config = ServoConfig(**config)
    clock = FakeClock()
    arm = FakeArm(clock, start, read_only=config.dry_run, fsr=fsr)
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

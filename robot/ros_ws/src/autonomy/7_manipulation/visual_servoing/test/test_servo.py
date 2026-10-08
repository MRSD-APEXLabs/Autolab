"""Whole grasp runs against the simulated arm and wrist camera (simulated time: each run takes milliseconds)."""
import numpy as np
import pytest

from sim import Camera, FakeArm, FakeClock, tool_down
from visual_servoing.mount import Mount, rpy_matrix, transform, wrap_pi
from visual_servoing.servo import GripperConfig, ServoConfig, ServoRunner

MOUNT = Mount()
START = tool_down(0.40, 0.0, 0.35)
CENTER = np.array([0.42, 0.03, 0.02])      # plate top-face centre, long side at 30 deg
LONG_DEG = 30.0
TWO_FINGERS = dict(use_fsr=True, target_pressure=(250, 120), stop_pressure=(370, 140))    # Camera-Edge's, both FSRs working


def held_from(value):
    """Both fingers press once the gripper is closed to `value`."""
    return lambda v, arm: (400, 200) if v >= value else (0, 0)


def run(start=START, fsr=held_from(130), prepare=None, hook=None, camera=None, **config):
    config.setdefault('dry_run', False)
    config.setdefault('gripper', GripperConfig(use_fsr=True))
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
    assert arm.gripper_values == [0, 50, 70, 90, 95, 125]


@pytest.mark.parametrize('tare', [5, 0])
def test_idle_fsr_offset_is_tared(tare):
    # FSR1 idles at ~360 on the arm, above its 250 touch level. Counted from the open-gripper baseline, the close
    # runs as without the offset; raw (tare 0), every step reads as finger 1 alone and nudges the arm sideways
    fsr = lambda v, arm: tuple(np.add((360, 0), held_from(130)(v, arm)))
    status, arm = run(fsr=fsr, gripper=GripperConfig(tare_samples=tare, **TWO_FINGERS))
    assert status['result'] == 'grasped', status
    if tare:
        assert status['fsr_baseline'] == [360, 0]
        assert arm.gripper_values == [0, 50, 70, 90, 110, 130, 160] and len(arm.moves) == 5
    else:
        assert len(arm.moves) > 10


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



def test_without_fsrs_the_gripper_closes_to_its_position_plus_the_offset():
    status, arm = run(fsr=lambda v, arm: (354, 0), gripper=GripperConfig())
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

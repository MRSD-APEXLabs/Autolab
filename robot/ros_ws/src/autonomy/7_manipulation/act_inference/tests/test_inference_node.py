import json
import os
import threading
import time
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('move_result', ['done', 'stopped', 'error'])
@pytest.mark.parametrize('retreat_result', ['done', 'stopped', 'error'])
def test_base_release_and_60_mm_retreat_before_handback(move_result, retreat_result):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    events = []
    def move(*args, **kwargs):
        events.append('place')
        if move_result == 'error': raise RuntimeError('move failed')
        return move_result
    def retreat(*args, **kwargs):
        assert kwargs['lift_mm'] == 60. and kwargs['speed_mm_s'] == 40. and kwargs['accel_mm_s2'] == 500.
        assert kwargs['open_wait_sec'] == 1.5 and kwargs['report_type'] == 'real' and kwargs['arrival_sec'] == .1
        events.append('release_and_lift')
        if retreat_result == 'error': raise RuntimeError('retreat failed')
        return retreat_result
    node = SimpleNamespace(config=None, base_placement=SimpleNamespace(lift_speed_mm_s=40., accel_mm_s2=500., open_wait_sec=1.5),
                           rollout=SimpleNamespace(execute=True), base_mode='scripted',
                           runner=SimpleNamespace(status=lambda: dict(target='base'), _check_abort=lambda: None),
                           _base_move_fn=move, _post_placement_fn=retreat, _handoff_robot=None,
                           handoff=SimpleNamespace(finish=lambda check: events.append('handback')))
    if move_result == 'error' or (move_result == 'done' and retreat_result == 'error'):
        with pytest.raises(RuntimeError):
            ActInferenceNode._run(node, threading.Event())
    else:
        expected = retreat_result if move_result == 'done' else move_result
        assert ActInferenceNode._run(node, threading.Event()) == expected
    assert events == (['place'] + (['release_and_lift'] if move_result == 'done' else [])
                      + (['handback'] if move_result == retreat_result == 'done' else []))


@pytest.mark.parametrize('target', ['base', 'ot2'])
def test_base_run_closes_the_handback_connection_first(target):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    events = []
    handback_robot = SimpleNamespace(close=lambda: events.append('close handback connection'))
    node = SimpleNamespace(config=None, base_placement=SimpleNamespace(lift_speed_mm_s=40., accel_mm_s2=500., open_wait_sec=1.5),
                           rollout=SimpleNamespace(execute=True), base_mode='scripted',
                           runner=SimpleNamespace(status=lambda: dict(target=target), _check_abort=lambda: None),
                           _base_move_fn=lambda *a, **kw: events.append('place') or 'done',
                           _post_placement_fn=lambda *a, **kw: events.append('release_and_lift') or 'done',
                           _run_policy=lambda stop, target: events.append('policy') or 'done',
                           _handoff_robot=handback_robot,
                           handoff=SimpleNamespace(finish=lambda check: events.append('handback')))
    assert ActInferenceNode._run(node, threading.Event()) == 'done'
    if target == 'base':
        assert events == ['close handback connection', 'place', 'release_and_lift', 'handback']
        assert node._handoff_robot is None
    else:
        assert events == ['policy', 'handback'] and node._handoff_robot is handback_robot


def test_cancelled_yaw_does_not_start_policy():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(_yaw_align_fn=lambda *args, **kwargs: 'stopped',
                           config=None, rollout=SimpleNamespace(execute=True))
    # No rollout callback is provided: cancellation must return before accessing it.
    assert ActInferenceNode._run_policy(node, threading.Event()) == 'stopped'


def test_cancelled_base_start_move_does_not_start_policy():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(_base_start_fn=lambda *args, **kwargs: 'stopped', base_start=None,
                           config=None, rollout=SimpleNamespace(execute=True))
    assert ActInferenceNode._run_policy(node, threading.Event(), 'base') == 'stopped'


@pytest.mark.parametrize('target', ['ot2', 'shaker', 'base'])
@pytest.mark.parametrize('outcome', ['done', 'stopped', 'error'])
def test_policy_then_release_only_on_success(outcome, target):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    stages = []

    def rollout(config, rollout_config, policy, stop):
        assert rollout_config == target + ' rollout' and policy == target + ' policy'
        stages.append('policy')
        if outcome == 'error':
            raise RuntimeError('policy failed')
        return outcome

    def release(*args, **kwargs):
        stages.append(f'release/lift {kwargs.get("lift_mm", 100.0):g}'
                      + (f' at {kwargs["speed_mm_s"]:g}' if 'speed_mm_s' in kwargs else '')
                      + (f' ramp {kwargs["accel_mm_s2"]:g}' if 'accel_mm_s2' in kwargs else '')
                      + (f' wait {kwargs["open_wait_sec"]:g} {kwargs["report_type"]}' if 'open_wait_sec' in kwargs else '')
                      + (f' shift x {kwargs["shift_x_mm"]:g}' if 'shift_x_mm' in kwargs else ''))
        return 'done'

    node = SimpleNamespace(config=None, rollout=SimpleNamespace(execute=True),
                           rollouts={t: t + ' rollout' for t in ('ot2', 'shaker', 'base')},
                           _policies={t: t + ' policy' for t in ('ot2', 'shaker', 'base')},
                           _yaw_align_fn=lambda *a, **kw: stages.append('yaw') or 'done',
                           base_start='start pose', base_placement=SimpleNamespace(lift_speed_mm_s=40., accel_mm_s2=500., open_wait_sec=1.5),
                           _base_start_fn=lambda config, pose, **kw: stages.append(pose) or 'done',
                           _rollout_fn=rollout, _post_placement_fn=release)
    if outcome == 'error':
        with pytest.raises(RuntimeError, match='policy failed'):
            ActInferenceNode._run_policy(node, threading.Event(), target)
    else:
        assert ActInferenceNode._run_policy(node, threading.Event(), target) == outcome
    # Base moves to its start pose (no 90 degree yaw alignment) and lifts 60 mm.
    expected = (['start pose'] if target == 'base' else ['yaw']) + ['policy']
    if outcome == 'done':
        expected.append({'base': 'release/lift 60 at 40 ramp 500 wait 1.5 real',
                         'ot2': 'release/lift 100 at 25 shift x -2',
                         'shaker': 'release/lift 100 at 25'}[target])
    assert stages == expected


@pytest.mark.parametrize('base_mode, switches_camera', [('policy', True), ('scripted', False)])
def test_base_camera_only_for_the_policy(base_mode, switches_camera):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    calls = []
    node = SimpleNamespace(base_mode=base_mode, runner=SimpleNamespace(status=lambda: dict(target='base')),
                           _verify_motion_ownership=lambda check: calls.append('ownership'),
                           _prepare_camera=lambda check: calls.append('camera'))
    ActInferenceNode._prepare_run(node, lambda: None)
    assert calls == ['ownership'] + (['camera'] if switches_camera else [])


@pytest.mark.parametrize('target', ['ot2', 'shaker', 'base', None])
def test_start_stop_and_status_over_ros(target):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    from act_inference.node import ActInferenceNode

    aligned = []

    def yaw_align(config, execute, stop):
        assert target != 'base' and not execute
        aligned.append('yaw')
        return 'done'

    def base_start(config, placement, execute, stop):
        assert target == 'base' and not execute
        assert placement.pose_mm_deg == (32.055321, 271.166351, 116.803917, -179.675382, -1.57403, -0.737282)
        assert placement.speed_mm_s == 40 and placement.rotation_speed_deg_s == 6
        aligned.append('start pose')
        return 'done'

    checkpoints = {'ot2': 'xarm6_placing_act_ft_vg_tilt_20261121', 'shaker': 'xarm6_shaker_act_ft_vg_tilt_strict_20261121',
                   'base': 'xarm6_base_act_ft_20261108b'}

    def rollout(config, rollout_config, policy, stop):
        assert aligned == (['start pose'] if target == 'base' else ['yaw'])
        assert policy == 'policy' and not rollout_config.execute
        assert rollout_config.ckpt_dir.name == checkpoints[target or 'ot2']
        assert rollout_config.workspace_radius == .15
        # Shaker only: 1.5 mm/s below 110 mm, so the plate reaches the clamp's bounds on fresh images.
        assert (rollout_config.approach_speed, rollout_config.slow_below_z) == (
            (.0015, .11) if target == 'shaker' else (.003, 0.))
        stop.wait(5)
        return 'stopped'

    def base_move(config, placement, execute, stop):
        pytest.fail('base_mode policy must not run the scripted base move')

    namespace = f'/test_act_node_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', rollout_fn=rollout, base_move_fn=base_move,
                            yaw_align_fn=yaw_align, base_start_fn=base_start, namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('base_mode', value='policy'),
                                                 rclpy.parameter.Parameter('select_service', value='')])
    client_node = rclpy.create_node('act_client', namespace=namespace)
    assert node.ckpt_dir.name == 'xarm6_placing_act_ft_vg_tilt_20261121'
    assert {t: d.name for t, d in node.ckpt_dirs.items()} == checkpoints and node.base_mode == 'policy'
    assert all(d.is_dir() for d in node.ckpt_dirs.values())
    assert node.rollout.duration_sec == 60 and node.rollout.enable_rotation
    assert node.rollout.workspace_radius == .15 and node.rollout.final_offset == (0., 0., 0.)
    assert node.config.motion.translation_speed == .01 and node.config.motion.rotation_speed == .15
    statuses = []
    client_node.create_subscription(String, 'act_inference/status', lambda m: statuses.append(json.loads(m.data)),
                                    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = client_node.create_client(Trigger, 'act_inference/start' + (f'/{target}' if target else ''))
    other = client_node.create_client(Trigger, 'act_inference/start/ot2' if target == 'base'
                                     else 'act_inference/start/base')
    stop = client_node.create_client(Trigger, 'act_inference/stop')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def call(client):
        assert client.wait_for_service(timeout_sec=5)
        return client.call(Trigger.Request())
    try:
        assert call(start).success
        deadline = time.monotonic() + 0.3   # well inside the 0.5 s status timer: published on start
        while not any(s['running'] for s in statuses):
            assert time.monotonic() < deadline, statuses
            time.sleep(0.01)
        second = call(start)
        assert not second.success and second.message == 'already running'
        assert not call(other).success  # all targets share one arm/run lock
        assert node.runner.status()['target'] == (target or 'ot2')
        assert call(stop).success
        deadline = time.monotonic() + 5
        while not (statuses and statuses[-1]['result'] == 'stopped'):
            assert time.monotonic() < deadline, statuses[-3:]
            time.sleep(0.05)
        assert statuses[-1]['running'] is False and statuses[-1]['started'] > 0
        assert statuses[-1]['target'] == (target or 'ot2')
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('result,handback', [('done', True), ('stopped', False)])
def test_jog_hands_back_only_after_a_completed_move(result, handback):
    from act_inference.node import ActInferenceNode
    events = []

    def move(stop):
        events.append('move')
        return result

    node = SimpleNamespace(rollout=SimpleNamespace(execute=True), runner=SimpleNamespace(_check_abort=lambda: None),
                           handoff=SimpleNamespace(finish=lambda check: events.append('handback')))
    assert ActInferenceNode._jog(node, threading.Event(), move) == result
    assert events == ['move'] + (['handback'] if handback else [])


@pytest.mark.parametrize('state,servo,alive,busy', [
    ('PLANNING', False, {'move_to_pose_node'}, 'the planner is planning'),
    ('EXECUTING', False, {'move_to_pose_node'}, 'the planner is executing'),
    ('EXECUTING', False, set(), ''),
    ('SUCCESS', False, {'move_to_pose_node'}, ''),
    (None, True, {'visual_servo'}, 'the visual servo is running'),
    (None, True, set(), ''),
])
def test_jog_refused_while_another_node_drives_the_arm(state, servo, alive, busy):
    from act_inference.node import ActInferenceNode
    starts = []
    node = SimpleNamespace(_planning_state=state, _servo_running=servo, get_node_names=lambda: list(alive),
                           _verify_motion_ownership=None,
                           runner=SimpleNamespace(start=lambda **kwargs: starts.append(kwargs) or (False, 'x')))
    node._arm_busy = lambda: ActInferenceNode._arm_busy(node)
    response = ActInferenceNode._on_jog(node, None, SimpleNamespace(), 'jog_z_plus', 'move', 'moved 10 mm')
    assert ActInferenceNode._arm_busy(node) == busy
    assert (response.success, response.message) == (False, f'not jogging: {busy}' if busy else 'x')
    assert len(starts) == (0 if busy else 1)


def test_jog_services_over_ros():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from std_srvs.srv import Trigger

    from act_inference.node import ActInferenceNode

    jogs = []

    def jog(config, offset, speed, execute, stop):
        assert not execute and speed == 10.
        jogs.append(offset)
        return 'done'

    def jog_rz(config, angle, speed, execute, stop):
        assert not execute and speed == 10.
        jogs.append(angle)
        return 'done'

    def home(config, joints, speed, rotation_speed, execute, stop):
        assert not execute and (speed, rotation_speed) == (40., 30.)
        jogs.append(tuple(joints))
        return 'done'

    def clear_errors(config, execute, stop):
        assert not execute
        jogs.append('clear')
        return 'done'

    namespace = f'/test_act_jog_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', jog_fn=jog, jog_rz_fn=jog_rz, home_fn=home, clear_errors_fn=clear_errors,
                            namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('select_service', value='')])
    client_node = rclpy.create_node('act_jog_client', namespace=namespace)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        for name, offset, done in (('x_plus', (.01, 0., 0.), 'moved 10 mm'), ('y_minus', (0., -.01, 0.), 'moved 10 mm'),
                                   ('z_plus', (0., 0., .01), 'moved 10 mm'), ('rz_plus', 10., 'rotated 10 deg'),
                                   ('rz_minus', -10., 'rotated 10 deg'),
                                   ('home', (1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284), 'at home_offset')):
            client = client_node.create_client(Trigger, f'act_inference/jog/{name}')
            assert client.wait_for_service(timeout_sec=5)
            response = client.call(Trigger.Request())
            assert response.success and response.message == f'jog_{name}: {done}', response.message
            assert jogs[-1] == offset
            status = node.runner.status()
            assert status['target'] == f'jog_{name}' and status['result'] == 'done' and not status['running']
        client = client_node.create_client(Trigger, 'act_inference/clear_errors')
        assert client.wait_for_service(timeout_sec=5)
        response = client.call(Trigger.Request())
        assert response.success and response.message == 'clear_errors: errors cleared, arm enabled', response.message
        assert jogs[-1] == 'clear'
        status = node.runner.status()
        assert status['target'] == 'clear_errors' and status['result'] == 'done' and not status['running']
    finally:
        executor.shutdown()
        node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def joy_axes(r2=False, l2=False, lx=0., ly=0., rx=0., ry=0.):
    """Normalised Xbox /joy axes: LX LY LT RX RY RT dpad-X dpad-Y; triggers +1 released, -1 fully pressed."""
    return [lx, ly, -1. if l2 else 1., rx, ry, -1. if r2 else 1., 0., 0.]


def teleop_node(**fields):
    return SimpleNamespace(teleop_enable_axis=5, teleop_block_axis=2, teleop_trigger_threshold=.6,
                           teleop_axes=[4, 3, 1, 0], teleop_signs=[1., 1., 1., 1.], teleop_deadband=.1,
                           teleop_joy_timeout=.5, teleop_home_button=3, clear_errors_button=6,
                           _joy_lock=threading.Lock(), _joy=(float('-inf'), [], []), _teleop_released=False,
                           _clear_errors_held=True, **fields)


Y_HELD = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]   # Xbox buttons: A B X Y LB RB back start guide LS RS
BACK_HELD = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]


@pytest.mark.parametrize('axes,buttons,age,expected', [
    (joy_axes(), [], 0., None),                                          # R2 released
    (joy_axes(r2=True, ry=1.), [], .6, None),                            # /joy went quiet
    (joy_axes(r2=True, ry=1.)[:5], [], 0., None),                        # no RT axis: never engaged
    (joy_axes(r2=True, l2=True, ry=1., lx=1.), Y_HELD, 0., (0., 0., 0., 0.)),     # L2 drives the base: the arm holds
    (joy_axes(r2=True, ry=1., rx=-1., ly=.55, lx=-1.), [], 0., (1., -1., .5, -1.)),  # RY -> x, RX -> y, LY -> z, LX -> rz
    (joy_axes(r2=True, ry=.05, rx=float('nan')), [], 0., (0., 0., 0., 0.)),         # deadband, non-finite
    (joy_axes(r2=True, ry=1.), Y_HELD, 0., 'home'),                      # R2 + Y: drive to home_offset
    (joy_axes(), Y_HELD, 0., None),                                      # Y alone does nothing
])
def test_teleop_command_from_the_gamepad(axes, buttons, age, expected):
    from act_inference.node import ActInferenceNode
    node = teleop_node()
    node._joy = (time.monotonic() - age, axes, buttons)
    command = ActInferenceNode._teleop_command(node)
    if expected is None or isinstance(expected, str):
        assert command == expected
    else:
        assert command == pytest.approx(expected)


def test_teleop_signs_flip_an_axis():
    from act_inference.node import ActInferenceNode
    node = teleop_node()
    node.teleop_signs = [-1., 1., -1., -1.]
    node._joy = (time.monotonic(), joy_axes(r2=True, ry=1., ly=1., lx=1.), [])
    assert ActInferenceNode._teleop_command(node) == pytest.approx((-1., 0., -1., -1.))


def joy_node(busy=''):
    from act_inference.node import ActInferenceNode
    starts, logs = [], []
    node = teleop_node(_arm_busy=lambda: busy, _verify_motion_ownership='verify', _teleop='teleop',
                       _verify_single_node='single', _clear_errors='clear',
                       _publish_status=lambda: None,
                       get_logger=lambda: SimpleNamespace(info=logs.append, warn=logs.append),
                       runner=SimpleNamespace(start=lambda **kwargs: starts.append(kwargs) or (True, '')))
    node._start_clear_errors = lambda: ActInferenceNode._start_clear_errors(node)
    return node, starts, logs


def test_back_press_clears_errors_once_per_press_without_r2():
    from act_inference.node import ActInferenceNode
    node, starts, _ = joy_node()
    press, release = SimpleNamespace(axes=joy_axes(), buttons=BACK_HELD), SimpleNamespace(axes=joy_axes(), buttons=[])
    ActInferenceNode._on_joy(node, press)   # held before the node saw it released: ignored
    assert not starts
    for msg in (release, press, press, release, press):
        ActInferenceNode._on_joy(node, msg)
    assert starts == [dict(target='clear_errors', run='clear', prepare='single')] * 2


def test_back_refused_while_another_node_drives_the_arm():
    from act_inference.node import ActInferenceNode
    node, starts, logs = joy_node(busy='the planner is executing')
    ActInferenceNode._on_joy(node, SimpleNamespace(axes=joy_axes(), buttons=[]))
    ActInferenceNode._on_joy(node, SimpleNamespace(axes=joy_axes(), buttons=BACK_HELD))
    assert not starts and logs == ['Clear errors refused: the planner is executing']


@pytest.mark.parametrize('held,result,after', [
    (True, 'done', ['handback']),    # a failed SDK run kept the arm: hand it back to ROS
    (False, 'done', ['activate']),   # faulted under ROS control: restart the controllers the driver stopped
    (True, 'stopped', []), (False, 'stopped', []),
])
def test_clear_errors_gives_the_arm_back_to_the_planner(held, result, after):
    from act_inference.node import ActInferenceNode
    events = []
    node = SimpleNamespace(config='config', rollout=SimpleNamespace(execute=True),
                           _clear_errors_fn=lambda config, execute, stop: events.append((config, execute)) or result,
                           runner=SimpleNamespace(_check_abort=lambda: None),
                           handoff=SimpleNamespace(held=held, finish=lambda check: events.append('handback')),
                           _activate_arm_controllers=lambda check: events.append('activate'))
    assert ActInferenceNode._clear_errors(node, threading.Event()) == result
    assert events == [('config', True)] + after


def controllers_node(state, switch_ok=True):
    switched = []
    node = SimpleNamespace(
        handoff=SimpleNamespace(controllers=('xarm6_traj_controller',), status=lambda check: {'sdk_owned': True},
                                list_controllers=lambda check: [SimpleNamespace(name='xarm6_traj_controller',
                                                                                state=state)]),
        _switch_arm_controllers=lambda stopped, started, check: switched.append((stopped, started)) or switch_ok)
    return node, switched


@pytest.mark.parametrize('state,expected', [('active', []), ('inactive', [([], ['xarm6_traj_controller'])])])
def test_clear_errors_starts_only_stopped_arm_controllers(state, expected):
    from act_inference.node import ActInferenceNode
    node, switched = controllers_node(state)
    ActInferenceNode._activate_arm_controllers(node, lambda: None)
    assert switched == expected


def test_clear_errors_reports_a_controller_that_will_not_start():
    from act_inference.node import ActInferenceNode
    node, _ = controllers_node('inactive', switch_ok=False)
    with pytest.raises(RuntimeError, match='xarm6_traj_controller did not start.*sdk_owned'):
        ActInferenceNode._activate_arm_controllers(node, lambda: None)


def test_r2_press_starts_one_teleop_run_and_needs_a_release_before_the_next():
    from act_inference.node import ActInferenceNode
    node, starts, _ = joy_node()
    press, release = SimpleNamespace(axes=joy_axes(r2=True), buttons=[]), SimpleNamespace(axes=joy_axes(), buttons=[])
    ActInferenceNode._on_joy(node, press)   # held before the node saw it released: ignored
    assert not starts
    for msg in (release, press, press, press):
        ActInferenceNode._on_joy(node, msg)
    assert starts == [dict(target='jog_teleop', run='teleop', prepare='verify')]
    assert node._joy[1] == joy_axes(r2=True)
    ActInferenceNode._on_joy(node, release)
    ActInferenceNode._on_joy(node, press)
    assert len(starts) == 2


def test_r2_refused_while_another_node_drives_the_arm():
    from act_inference.node import ActInferenceNode
    node, starts, logs = joy_node(busy='the planner is executing')
    ActInferenceNode._on_joy(node, SimpleNamespace(axes=joy_axes(), buttons=[]))
    ActInferenceNode._on_joy(node, SimpleNamespace(axes=joy_axes(r2=True), buttons=[]))
    assert not starts and logs == ['Arm teleop refused: the planner is executing']


@pytest.mark.parametrize('result,handback', [('done', True), ('stopped', False)])
def test_teleop_hands_back_only_after_a_completed_run(result, handback):
    from act_inference.node import ActInferenceNode
    events = []

    def teleop(config, command, speed, radius, execute, stop, rotation_speed_deg_s, home_joints):
        events.append(('teleop', command, speed, radius, execute, rotation_speed_deg_s, home_joints))
        return result

    node = SimpleNamespace(config=None, rollout=SimpleNamespace(execute=True), teleop_speed_mm_s=10.,
                           teleop_rotation_speed_deg_s=20., teleop_radius_mm=150., home_joints='joints',
                           _teleop_fn=teleop, _teleop_command='command',
                           runner=SimpleNamespace(_check_abort=lambda: None),
                           handoff=SimpleNamespace(finish=lambda check: events.append('handback')))
    assert ActInferenceNode._teleop(node, threading.Event()) == result
    assert events == [('teleop', 'command', 10., .15, True, 20., 'joints')] + (['handback'] if handback else [])


def test_gamepad_teleop_over_ros():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Joy

    from act_inference.node import ActInferenceNode

    seen = []

    def teleop(config, command, speed, radius, execute, stop, rotation_speed_deg_s, home_joints):
        assert not execute and speed == 40. and radius == .15 and rotation_speed_deg_s == 30.
        assert home_joints == pytest.approx([1.6284, -0.3927, -0.5812, 0.0, 0.9738, 1.6284])
        while (velocity := command()) is not None:
            seen.append(velocity)
            stop.wait(.02)
        return 'done'

    namespace = f'/test_act_teleop_{os.getpid()}'
    rclpy.init()
    node = ActInferenceNode(policy='policy', teleop_fn=teleop, namespace=namespace,
                            parameter_overrides=[rclpy.parameter.Parameter('execute', value=False),
                                                 rclpy.parameter.Parameter('select_service', value=''),
                                                 rclpy.parameter.Parameter('teleop_joy_topic', value='joy')])
    pad = rclpy.create_node('act_teleop_pad', namespace=namespace)
    publisher = pad.create_publisher(Joy, 'joy', 10)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(pad)
    threading.Thread(target=executor.spin, daemon=True).start()

    def publish_for(seconds, axes, buttons=()):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(Joy(axes=axes, buttons=list(buttons)))
            time.sleep(.05)

    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline
            time.sleep(.05)
        publish_for(.3, joy_axes())
        publish_for(1., joy_axes(r2=True, ry=1., ly=-1., lx=1.))
        publish_for(.5, joy_axes(r2=True), Y_HELD)
        status = node.runner.status()
        assert status['running'] and status['target'] == 'jog_teleop'
        publish_for(.3, joy_axes())
        deadline = time.monotonic() + 5
        while (status := node.runner.status())['running']:
            assert time.monotonic() < deadline
            time.sleep(.05)
        assert status['result'] == 'done', status
        assert (1., 0., -1., 1.) in seen and 'home' in seen
    finally:
        executor.shutdown()
        node.destroy_node()
        pad.destroy_node()
        rclpy.shutdown()

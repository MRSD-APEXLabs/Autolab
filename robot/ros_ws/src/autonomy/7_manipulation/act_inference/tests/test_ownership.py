from types import SimpleNamespace
import threading

import pytest

from act_inference.ownership import DriverHandoff
from act_inference.runner import RunAborted


def check():
    pass


class Driver:
    def __init__(self):
        self.calls = []
        self.state = dict(protocol=1, sdk_owned=False, claimed=1, resync_pending=False)
        self.fail = None

    def status(self, check):
        check()
        state = dict(self.state)
        self.state['resync_pending'] = False
        return state

    def controllers(self, check):
        return [SimpleNamespace(name='xarm6_traj_controller', state='active' if self.state['claimed'] else 'inactive')]

    def switch(self, stopped, started, check):
        self.calls.append(('switch', list(stopped), list(started)))
        assert not self.state['sdk_owned']
        if self.fail == 'switch': return False
        self.state['claimed'] = int(bool(started))
        return True

    def acquire(self, check):
        self.calls.append('acquire')
        assert not self.state['claimed']
        if self.fail == 'acquire': return False
        self.state['sdk_owned'] = True
        return True

    def release(self, check):
        self.calls.append('release')
        if self.fail == 'release': return False
        self.state.update(sdk_owned=False, resync_pending=True)
        return True

    def ready(self, check):
        self.calls.append('ready')
        check()
        if self.fail == 'ready': raise RuntimeError('C31')

    def handoff(self):
        return DriverHandoff(self.status, self.controllers, self.switch, self.acquire, self.release, self.ready)


def test_acquire_and_handback_order():
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    assert h.held and d.state['sdk_owned'] and not d.state['claimed']
    h.finish(check)
    assert d.calls == [('switch', ['xarm6_traj_controller'], []), 'acquire', 'ready', 'release',
                        ('switch', [], ['xarm6_traj_controller'])]
    assert not h.held and d.state['claimed']


@pytest.mark.parametrize('state', [dict(protocol=0), dict(sdk_owned=True)])
def test_unpatched_or_already_owned_driver_is_not_modified(state):
    d = Driver()
    d.state.update(state)
    with pytest.raises(RuntimeError): d.handoff().acquire(check)
    assert not d.calls


@pytest.mark.parametrize('failure', ['switch', 'acquire'])
def test_failed_acquisition_never_reaches_ready(failure):
    d = Driver()
    d.fail = failure
    h = d.handoff()
    with pytest.raises(RuntimeError): h.acquire(check)
    assert not h.held and 'ready' not in d.calls and 'release' not in d.calls


def test_local_owner_can_retry_after_stop_and_preserve_original_controllers():
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    h.acquire(check)
    assert d.calls.count('acquire') == 1 and h.original_active == ['xarm6_traj_controller']
    h.finish(check)
    assert d.state['claimed']


@pytest.mark.parametrize('failure', ['ready', 'release', 'stop'])
def test_failed_or_stopped_handback_keeps_ros_blocked(failure):
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    d.fail = failure
    def abort(): raise RunAborted()
    with pytest.raises((RuntimeError, RunAborted)):
        h.finish(abort if failure == 'stop' else check)
    assert h.held and d.state['sdk_owned'] and not d.state['claimed']


READINESS_REFUSAL = 'Arm must be fault-free in servo mode and ready before handback'


def handback_clock(monkeypatch):
    import act_inference.ownership as ownership
    clock = [0.]
    monkeypatch.setattr(ownership, 'time', SimpleNamespace(
        monotonic=lambda: clock[0], sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds)))


def test_delayed_driver_readiness_retries_release_without_reenabling(monkeypatch):
    handback_clock(monkeypatch)
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    attempts = []
    def release(check):
        attempts.append(True)
        if len(attempts) < 4:
            return SimpleNamespace(success=False, message=READINESS_REFUSAL)
        return SimpleNamespace(success=d.release(check), message='released')
    h.release_driver = release
    h.finish(check)
    assert len(attempts) == 4
    assert d.calls.count('ready') == 1
    assert not h.held and d.state['claimed']


@pytest.mark.parametrize('reason', [READINESS_REFUSAL, 'SDK control was not held exclusively'])
def test_driver_rejection_is_bounded_and_preserves_reason(monkeypatch, reason):
    handback_clock(monkeypatch)
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    attempts = []
    def release(check):
        attempts.append(True)
        return SimpleNamespace(success=False, message=reason)
    h.release_driver = release
    with pytest.raises(RuntimeError, match=reason):
        h.finish(check)
    assert 1 <= len(attempts) <= 42
    if reason != READINESS_REFUSAL:
        assert len(attempts) == 1
    assert h.held and d.state['sdk_owned'] and not d.state['claimed']
    assert d.calls.count('ready') == 1


def test_stop_during_driver_readiness_wait_does_not_restore_ros(monkeypatch):
    handback_clock(monkeypatch)
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    stopped = threading.Event()
    def release(check):
        stopped.set()
        return SimpleNamespace(success=False, message=READINESS_REFUSAL)
    def abort():
        if stopped.is_set(): raise RunAborted()
    h.release_driver = release
    with pytest.raises(RunAborted):
        h.finish(abort)
    assert h.held and d.state['sdk_owned'] and not d.state['claimed']


def test_initially_inactive_controller_is_not_activated():
    d = Driver()
    d.state['claimed'] = 0
    h = d.handoff()
    h.acquire(check)
    h.finish(check)
    assert not d.state['claimed'] and d.calls == ['acquire', 'ready', 'release']


def test_duplicate_nodes_are_blocked_before_driver_query():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(rollout=SimpleNamespace(execute=True),
                           get_node_names=lambda: ['act_inference', 'act_inference'])
    with pytest.raises(RuntimeError, match='Multiple act_inference'):
        ActInferenceNode._verify_motion_ownership(node, check)


def test_wrong_robot_is_rejected_before_controller_changes():
    d = Driver()
    h = d.handoff()
    h.robot_ip = 'different-arm'
    with pytest.raises(RuntimeError, match='robot_ip mismatch'):
        h.acquire(check)
    assert not d.calls


def test_standalone_execute_cannot_bypass_handoff():
    from act_inference.cli import main
    with pytest.raises(SystemExit, match='exclusive ROS driver ownership'):
        main(['--ckpt-dir', 'unused', '--execute'])


@pytest.mark.parametrize('result', ['done', 'stopped', 'error'])
@pytest.mark.parametrize('target, base_mode', [('ot2', 'policy'), ('base', 'policy'), ('base', 'scripted')])
def test_restore_only_after_all_motion_succeeds(result, target, base_mode):
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    calls = []
    def run(*args, **kwargs):
        if result == 'error': raise RuntimeError('motion failed')
        return result
    node = SimpleNamespace(rollout=SimpleNamespace(execute=True), config=None, base_placement=None, base_mode=base_mode,
                           runner=SimpleNamespace(status=lambda: dict(target=target), _check_abort=check),
                           _run_policy=run, _base_move_fn=run, _post_placement_fn=run,
                           handoff=SimpleNamespace(finish=lambda check: calls.append('finish')))
    if result == 'error':
        with pytest.raises(RuntimeError): ActInferenceNode._run(node, threading.Event())
    else:
        assert ActInferenceNode._run(node, threading.Event()) == result
    assert calls == (['finish'] if result == 'done' else [])


@pytest.mark.parametrize('target, base_mode', [('ot2', 'policy'), ('base', 'policy'), ('base', 'scripted')])
def test_ros_handoff_services_wrap_entire_run_without_robot(monkeypatch, target, base_mode):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import MultiThreadedExecutor
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from controller_manager_msgs.msg import ControllerState
    from std_srvs.srv import Trigger
    from act_inference.node import ActInferenceNode
    import json
    import time
    events = []
    state = dict(active=True, owned=False, ready=False)
    prefix = '/test_sdk_ownership'
    rclpy.init()
    server = rclpy.create_node('fake_driver_handoff')

    def status(request, response):
        response.success = True
        response.message = json.dumps(dict(protocol=1, robot_ip='192.168.1.236', sdk_owned=state['owned'],
                                           claimed=int(state['active']), resync_pending=False))
        return response

    def controllers(request, response):
        controller = ControllerState()
        controller.name = 'test_arm'
        controller.state = 'active' if state['active'] else 'inactive'
        response.controller = [controller]
        return response

    def switch(request, response):
        assert not state['owned']
        if request.deactivate_controllers:
            events.append('stop_ros')
            state['active'] = False
        if request.activate_controllers:
            assert state['ready']
            events.append('start_ros')
            state['active'] = True
        response.ok = True
        return response

    def acquire(request, response):
        assert not state['active']
        events.append('acquire')
        state['owned'] = True
        response.success = True
        return response

    def release(request, response):
        assert state['ready']
        events.append('release')
        state['owned'] = False
        response.success = True
        return response

    def motion(name):
        def run(*args, **kwargs):
            assert state['owned'] and not state['active']
            if name == 'open_lift':
                assert kwargs.get('lift_mm', 100.) == (60. if target == 'base' else 100.)
            events.append(name)
            return 'done'
        return run

    def ready(check):
        assert state['owned'] and not state['active']
        state['ready'] = True
        events.append('ready')

    server.create_service(Trigger, prefix + '/status', status)
    server.create_service(Trigger, prefix + '/acquire', acquire)
    server.create_service(Trigger, prefix + '/release', release)
    server.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
    server.create_service(SwitchController, '/controller_manager/switch_controller', switch)
    node = ActInferenceNode(policy='test', rollout_fn=motion('policy'), yaw_align_fn=motion('yaw'),
                            post_placement_fn=motion('open_lift'), base_move_fn=motion('base'),
                            base_start_fn=motion('start_pose'), namespace='/test_handoff', parameter_overrides=[
                                rclpy.parameter.Parameter('execute', value=True),
                                rclpy.parameter.Parameter('select_service', value=''),
                                rclpy.parameter.Parameter('base_mode', value=base_mode),
                                rclpy.parameter.Parameter('driver_control_prefix', value=prefix),
                                rclpy.parameter.Parameter('arm_controllers', value=['test_arm'])])
    node.handoff.ready = ready
    monkeypatch.setattr(node, 'get_node_names', lambda: ['act_inference'])
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        assert node.runner.start(target=target)[0]
        deadline = time.monotonic() + 6
        while node.runner.running:
            assert time.monotonic() < deadline
            time.sleep(.01)
        assert node.runner.status()['result'] == 'done', node.runner.status()
        expected_motion = {'ot2': ['yaw', 'policy', 'open_lift'],   # base policy: start pose instead of yaw
                           'base': ['base', 'open_lift'] if base_mode == 'scripted'
                           else ['start_pose', 'policy', 'open_lift']}[target]
        assert events == ['stop_ros', 'acquire'] + expected_motion + ['ready', 'release', 'start_ros']
    finally:
        executor.shutdown()
        node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_resync_timeout_does_not_restore_controllers(monkeypatch):
    import act_inference.ownership as ownership
    d = Driver()
    h = d.handoff()
    h.acquire(check)
    h.status = lambda check: dict(d.state)
    clock = [0.]
    def now():
        clock[0] += 1.
        return clock[0]
    monkeypatch.setattr(ownership, 'time', SimpleNamespace(monotonic=now, sleep=lambda seconds: None))
    with pytest.raises(RuntimeError, match='resynchronization timed out'):
        h.finish(check)
    assert not d.state['claimed']
    assert ('switch', [], ['xarm6_traj_controller']) not in d.calls


def test_ready_handoff_keeps_connection_like_visual_servoing(monkeypatch):
    pytest.importorskip('rclpy')
    import act_inference.node as module
    events = []
    class Robot:
        def __init__(self, *args): events.append('create')
        def connect(self): events.append('connect')
        def initialize_target_pose(self): events.append('pose')
        def enable(self): events.append('enable')
        def hand_back(self): events.append('hand_back')
        def close(self): events.append('close')
    monkeypatch.setattr(module, 'XArm6Robot', Robot)
    node = SimpleNamespace(config=SimpleNamespace(robot_ip='unused', motion=None), _handoff_robot=None)
    module.ActInferenceNode._ready_for_ros(node, check)
    module.ActInferenceNode._ready_for_ros(node, check)
    assert events == ['create', 'connect', 'pose', 'enable', 'hand_back',
                      'connect', 'pose', 'enable', 'hand_back']
    assert node._handoff_robot is not None and 'close' not in events

"""Node wiring for no-retry steps and tip-check results, without publishing to hardware."""
import json
from types import SimpleNamespace

from std_msgs.msg import String
from routine_executor.node import NO_RETRY_STEPS, RoutineExecutorNode
from routine_executor.state_machine import RoutineStateMachine


def make_node(steps):
    sm = RoutineStateMachine(steps, max_retries=3, no_retry=NO_RETRY_STEPS)
    sm.start()
    sm.acknowledge_dispatch()
    logs = []
    logger = SimpleNamespace(info=logs.append, warn=logs.append)
    return SimpleNamespace(_sm=sm, _listening=True, _seen_running=False,
                           get_logger=lambda: logger, logs=logs)


def test_ot2_is_in_the_no_retry_set():
    assert 'ot2' in NO_RETRY_STEPS
    assert 'go_to' not in NO_RETRY_STEPS


def test_tip_check_result_is_attached_to_the_run():
    node = make_node([{'name': 'ot2', 'parameters_json': {}}])
    result = {'type': 'tip_check', 'id': 'chk-1', 'status': 'done', 'anomaly_detected': True}
    RoutineExecutorNode._on_tip_check_result(node, String(data=json.dumps(result)))
    assert node._sm.recordings == [{**result, 'step': 0}]


def test_malformed_tip_check_result_is_ignored():
    node = make_node([{'name': 'ot2', 'parameters_json': {}}])
    RoutineExecutorNode._on_tip_check_result(node, String(data='not json'))
    RoutineExecutorNode._on_tip_check_result(node, String(data='["a list"]'))
    assert node._sm.recordings == []


def _failed_at_ot2():
    node = make_node([{'name': 'ot2', 'parameters_json': {}}])
    node._sm.on_failure()
    assert node._sm.state == 'failed'
    return node


def test_plain_retry_of_failed_ot2_step_is_refused():
    node = _failed_at_ot2()
    RoutineExecutorNode._on_retry_cmd(node, String(data=''))
    assert node._sm.state == 'failed'
    RoutineExecutorNode._on_retry_cmd(node, String(data='{"force": false}'))
    assert node._sm.state == 'failed'


def test_forced_retry_of_failed_ot2_step_restarts_it():
    node = _failed_at_ot2()
    RoutineExecutorNode._on_retry_cmd(node, String(data='{"force": true}'))
    assert node._sm.state == 'running'
    assert node._sm.needs_dispatch is True


def test_ot2_progress_is_attached_to_the_ot2_step_and_published_at_once():
    node = make_node([{'name': 'ot2', 'parameters_json': {}}])
    published = []
    node._publish_status = lambda: published.append(node._sm.to_dict())
    progress = {'phase': 'tip_check', 'command': 1, 'commands': 4, 'tip_check': {'pass': 2, 'passes': 5}}
    RoutineExecutorNode._on_ot2_progress(node, String(data=json.dumps(progress)))
    assert node._sm.step_progress == {**progress, 'machine': 'ot2', 'step': 0}
    assert published and published[-1]['step_progress']['phase'] == 'tip_check'


def test_ot2_progress_outside_an_ot2_step_or_malformed_is_ignored():
    node = make_node([{'name': 'go_to', 'target_machine': 'ot2'}])
    node._publish_status = lambda: None
    RoutineExecutorNode._on_ot2_progress(node, String(data='{"phase": "homing"}'))
    RoutineExecutorNode._on_ot2_progress(node, String(data='not json'))
    RoutineExecutorNode._on_ot2_progress(node, String(data='[1]'))
    assert node._sm.step_progress is None

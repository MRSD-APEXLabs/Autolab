"""Exercise repeated robot feedback without publishing commands to hardware."""
from types import SimpleNamespace

from behavior_tree_msgs.msg import Status
from routine_executor.node import RoutineExecutorNode
from routine_executor.state_machine import RoutineStateMachine


def make_node():
    sm = RoutineStateMachine([{'name': 'pick', 'target_machine': 'ot2'}])
    sm.start()
    sm.acknowledge_dispatch()
    logger = SimpleNamespace(info=lambda message: None, warn=lambda message: None)
    return SimpleNamespace(_sm=sm, _listening=True, _seen_running=False,
                           _last_status_id={}, _dispatch_status_id=None,
                           _status_trace=set(), _last_status_time={},
                           get_logger=lambda: logger)


def feedback(node, status):
    RoutineExecutorNode._on_action_status(node, Status(status=status), 'pick')


def test_repeated_failure_does_not_consume_retry_budget():
    node = make_node()
    feedback(node, Status.RUNNING)
    for _ in range(100):
        feedback(node, Status.FAILURE)
    assert node._sm.retry_count == 1
    assert node._sm.state == 'running'
    assert node._sm.needs_dispatch
    # Even after listening starts, stale FAILURE needs a fresh RUNNING first.
    node._listening = True
    feedback(node, Status.FAILURE)
    assert node._sm.retry_count == 1


def test_initial_attempt_plus_three_retries_before_failure():
    node = make_node()
    for attempt in range(4):
        node._listening = True
        node._sm.acknowledge_dispatch()
        feedback(node, Status.RUNNING)
        feedback(node, Status.FAILURE)
        assert node._sm.state == ('failed' if attempt == 3 else 'running')
        assert node._sm.needs_dispatch == (attempt < 3)
    assert node._sm.retry_count == 3


def test_success_on_retry_finishes_without_more_retries():
    node = make_node()
    feedback(node, Status.RUNNING)
    feedback(node, Status.FAILURE)
    node._listening = True
    node._sm.acknowledge_dispatch()
    feedback(node, Status.RUNNING)
    feedback(node, Status.SUCCESS)
    feedback(node, Status.FAILURE)
    assert node._sm.state == 'success'
    assert node._sm.retry_count == 0
    assert not node._sm.needs_dispatch


def test_zero_retries_fails_on_first_attempt():
    node = make_node()
    node._sm.max_retries = 0
    feedback(node, Status.RUNNING)
    feedback(node, Status.FAILURE)
    assert node._sm.state == 'failed'
    assert node._sm.retry_count == 0


def status_with_id(node, status, status_id):
    RoutineExecutorNode._on_action_status(node, Status(status=status, id=status_id), 'pick')


def test_success_from_a_new_activation_counts_even_if_running_was_missed():
    # A quick command (the shaker stop) can go RUNNING -> SUCCESS inside the 100 ms deaf
    # window after dispatch; its SUCCESS carries the BT's new activation id.
    node = make_node()
    node._dispatch_status_id = 4
    status_with_id(node, Status.SUCCESS, 5)
    assert node._sm.state == 'success'


def test_stale_success_with_the_dispatch_id_is_still_ignored():
    node = make_node()
    node._dispatch_status_id = 4
    for _ in range(20):
        status_with_id(node, Status.SUCCESS, 4)  # previous command's result, re-broadcast
    assert node._sm.state == 'running'
    status_with_id(node, Status.RUNNING, 5)
    status_with_id(node, Status.SUCCESS, 5)
    assert node._sm.state == 'success'


def test_status_id_is_tracked_even_while_not_listening():
    node = make_node()
    node._listening = False
    status_with_id(node, Status.SUCCESS, 7)
    assert node._last_status_id == {'pick': 7}
    assert node._sm.state == 'running'


def test_each_distinct_status_is_traced_once_with_its_verdict():
    # A stuck step must show what it heard: the stale re-broadcast is logged as ignored (once,
    # not 20 times a second), and the new activation's SUCCESS as accepted.
    node = make_node()
    logs = []
    node.get_logger = lambda: SimpleNamespace(info=logs.append, warn=logs.append)
    node._dispatch_status_id = 4
    node._listening = False
    status_with_id(node, Status.RUNNING, 5)
    node._listening = True
    for _ in range(20):
        status_with_id(node, Status.SUCCESS, 4)
    status_with_id(node, Status.SUCCESS, 5)
    trace = [line for line in logs if 'status' in line and 'id=' in line]
    assert len(trace) == 3
    assert 'not listening yet' in trace[0]
    assert 'id=4' in trace[1] and 'IGNORED as stale' in trace[1]
    assert 'id=5' in trace[2] and 'accepted' in trace[2]
    assert node._sm.state == 'success'

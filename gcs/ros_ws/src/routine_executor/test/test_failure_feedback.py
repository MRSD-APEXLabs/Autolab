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

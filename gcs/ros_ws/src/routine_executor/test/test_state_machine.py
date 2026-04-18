import pytest
from routine_executor.state_machine import RoutineStateMachine


def make_sm(steps=None, max_retries=2):
    steps = steps or [
        {'name': 'pick_base'},
        {'name': 'place', 'target_machine': 'ot2'},
    ]
    return RoutineStateMachine(steps, max_retries=max_retries)


def test_initial_state_is_idle():
    sm = RoutineStateMachine([], max_retries=3)
    assert sm.state == 'idle'


def test_start_sets_state_to_running():
    sm = make_sm()
    sm.start()
    assert sm.state == 'running'


def test_start_sets_current_step_to_zero():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick_up'}])
    sm.start()
    assert sm.current_step_idx == 0


def test_current_step_name_returns_correct_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}])
    sm.start()
    assert sm.current_step_name() == 'pick_base'


def test_current_step_returns_full_dict():
    step = {'name': 'place', 'target_machine': 'ot2'}
    sm = RoutineStateMachine([step], max_retries=2)
    sm.start()
    assert sm.current_step() == step


def test_on_success_advances_to_next_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}])
    sm.start()
    sm.on_success()
    assert sm.current_step_idx == 1
    assert sm.current_step_name() == 'place'


def test_on_success_last_step_sets_success_state():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()
    assert sm.state == 'success'


def test_on_failure_increments_retry_count():
    sm = make_sm(max_retries=3)
    sm.start()
    sm.on_failure()
    assert sm.retry_count == 1


def test_on_failure_within_retries_stays_running():
    sm = make_sm(max_retries=3)
    sm.start()
    sm.on_failure()
    assert sm.state == 'running'
    assert sm.current_step_idx == 0


def test_on_failure_exceeds_retries_sets_failed():
    sm = make_sm(max_retries=2)
    sm.start()
    sm.on_failure()  # retry 1
    sm.on_failure()  # retry 2 — exhausted
    assert sm.state == 'failed'
    assert sm.error is not None


def test_retry_count_resets_on_new_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}], max_retries=3)
    sm.start()
    sm.on_failure()  # pick_base retry 1
    sm.on_success()  # pick_base done
    assert sm.retry_count == 0


def test_cancel_sets_failed_state():
    sm = make_sm()
    sm.start()
    sm.cancel()
    assert sm.state == 'failed'
    assert sm.error == 'cancelled'


def test_start_while_running_is_rejected():
    sm = make_sm()
    sm.start()
    with pytest.raises(RuntimeError):
        sm.start()


def test_to_dict_contains_required_keys():
    steps = [{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}]
    sm = RoutineStateMachine(steps, max_retries=3)
    sm.start()
    d = sm.to_dict()
    assert d['state'] == 'running'
    assert d['steps'] == steps
    assert d['current_step'] == 0
    assert d['current_step_name'] == 'pick_base'
    assert d['retry_count'] == 0
    assert d['max_retries'] == 3
    assert d['error'] is None


def test_needs_dispatch_true_when_newly_started():
    sm = make_sm()
    sm.start()
    assert sm.needs_dispatch is True


def test_needs_dispatch_cleared_after_acknowledge():
    sm = make_sm()
    sm.start()
    sm.acknowledge_dispatch()
    assert sm.needs_dispatch is False


def test_needs_dispatch_true_after_retry():
    sm = make_sm(max_retries=3)
    sm.start()
    sm.acknowledge_dispatch()
    sm.on_failure()
    assert sm.needs_dispatch is True


def test_on_success_in_terminal_state_is_noop():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()  # transitions to 'success'
    sm.on_success()  # should be a no-op, not raise
    assert sm.state == 'success'


def test_on_failure_in_terminal_state_is_noop():
    sm = make_sm([{'name': 'pick_base'}], max_retries=1)
    sm.start()
    sm.on_failure()  # transitions to 'failed'
    sm.on_failure()  # should be a no-op, not raise
    assert sm.state == 'failed'


def test_cancel_in_terminal_state_is_noop():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()  # transitions to 'success'
    sm.cancel()      # should be a no-op
    assert sm.state == 'success'


def test_current_step_name_safe_after_completion():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()
    assert sm.current_step_name() == ''  # no IndexError


def test_current_step_safe_after_completion():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()
    assert sm.current_step() == {}  # no IndexError

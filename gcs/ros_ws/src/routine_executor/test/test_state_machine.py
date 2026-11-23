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
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick', 'target_machine': 'ot2'}])
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
    sm.on_failure()  # schedule retry 2
    sm.on_failure()  # retry 2 failed — exhausted
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
    sm.on_failure()  # schedule retry 1
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


# --- pause / resume ---

def test_pause_sets_state_to_paused():
    sm = make_sm()
    sm.start()
    sm.pause()
    assert sm.state == 'paused'


def test_pause_ignored_when_not_running():
    sm = make_sm()
    sm.pause()  # state is idle
    assert sm.state == 'idle'


def test_resume_sets_state_to_running():
    sm = make_sm()
    sm.start()
    sm.pause()
    sm.resume()
    assert sm.state == 'running'


def test_resume_ignored_when_not_paused():
    sm = make_sm()
    sm.start()
    sm.resume()  # state is running, not paused
    assert sm.state == 'running'


def test_cancel_while_paused_sets_failed():
    sm = make_sm()
    sm.start()
    sm.pause()
    sm.cancel()
    assert sm.state == 'failed'
    assert sm.error == 'cancelled'


def test_on_success_while_paused_stays_paused():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}])
    sm.start()
    sm.pause()
    sm.on_success()
    assert sm.state == 'paused'
    assert sm.current_step_idx == 1


def test_on_success_last_step_while_paused_sets_success():
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.pause()
    sm.on_success()
    assert sm.state == 'success'


def test_needs_dispatch_true_after_success_while_paused():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}])
    sm.start()
    sm.acknowledge_dispatch()
    sm.pause()
    sm.on_success()
    assert sm.needs_dispatch is True


def test_on_failure_while_paused_within_retries_stays_paused():
    sm = make_sm(max_retries=3)
    sm.start()
    sm.pause()
    sm.on_failure()
    assert sm.state == 'paused'
    assert sm.retry_count == 1


def test_on_failure_while_paused_exceeds_retries_sets_failed():
    sm = make_sm(max_retries=2)
    sm.start()
    sm.pause()
    sm.on_failure()  # retry 1
    sm.on_failure()  # schedule retry 2
    sm.on_failure()  # retry 2 failed — exhausted
    assert sm.state == 'failed'
    assert sm.error is not None


def test_resume_then_dispatch_on_pending_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}])
    sm.start()
    sm.acknowledge_dispatch()
    sm.pause()
    sm.on_success()       # step 0 done while paused, needs_dispatch = True, state stays paused
    sm.resume()           # back to running
    assert sm.state == 'running'
    assert sm.needs_dispatch is True
    assert sm.current_step_idx == 1


def test_to_dict_state_reflects_paused():
    sm = make_sm()
    sm.start()
    sm.pause()
    d = sm.to_dict()
    assert d['state'] == 'paused'
    assert d['current_step_name'] == 'pick_base'


def test_start_while_paused_is_rejected():
    sm = make_sm()
    sm.start()
    sm.pause()
    with pytest.raises(RuntimeError):
        sm.start()


# ── anomaly recordings ───────────────────────────────────────────────────────

def _running_sm():
    sm = RoutineStateMachine([{'name': 'a'}, {'name': 'b'}])
    sm.start()
    return sm


def tip(check_id, **fields):
    return {'type': 'tip_check', 'id': check_id, **fields}


def test_recordings_empty_on_new():
    assert RoutineStateMachine([{'name': 'a'}]).recordings == []


def test_add_recording_while_running_tags_the_routine_step():
    sm = _running_sm()
    assert sm.add_recording(tip('c1')) is True
    sm.on_success()
    sm.pause()
    assert sm.add_recording(tip('c2', anomaly_detected=True)) is True
    assert sm.recordings == [
        {'type': 'tip_check', 'id': 'c1', 'step': 0},
        {'type': 'tip_check', 'id': 'c2', 'anomaly_detected': True, 'step': 1},
    ]


def test_recording_ignored_when_inactive():
    idle = RoutineStateMachine([{'name': 'a'}])
    assert idle.add_recording(tip('c1')) is False
    assert idle.recordings == []
    done = RoutineStateMachine([{'name': 'a'}])
    done.start()
    done.on_success()
    assert done.state == 'success'
    assert done.add_recording(tip('c2')) is False
    assert done.recordings == []


def test_recording_without_id_or_type_is_ignored():
    sm = _running_sm()
    assert sm.add_recording({'type': 'tip_check'}) is False
    assert sm.add_recording({'id': 'c1'}) is False
    assert sm.add_recording('c1') is False
    assert sm.recordings == []


def test_recording_dedup_by_id():
    sm = _running_sm()
    sm.add_recording(tip('c1'))
    assert sm.add_recording(tip('c1', status='error')) is False
    assert [r['id'] for r in sm.recordings] == ['c1']


def test_recordings_kept_after_success():
    sm = _running_sm()
    sm.add_recording(tip('c1'))
    sm.on_success()
    sm.on_success()
    assert sm.state == 'success'
    assert [r['id'] for r in sm.recordings] == ['c1']


def test_to_dict_has_recordings():
    sm = _running_sm()
    sm.add_recording(tip('c1'))
    assert sm.to_dict()['recordings'] == [{'type': 'tip_check', 'id': 'c1', 'step': 0}]


def _fail_second_step(sm):
    sm.start()
    sm.on_success()
    for _ in range(sm.max_retries + 1):
        sm.on_failure()


def test_retry_resumes_failed_routine_at_failed_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick', 'target_machine': 'ot2'}])
    _fail_second_step(sm)
    assert sm.state == 'failed'
    assert sm.retry() is True
    assert sm.state == 'running'
    assert sm.current_step_idx == 1
    assert sm.retry_count == 0
    assert sm.error is None
    assert sm.needs_dispatch is True


def test_retry_after_cancel_redispatches_cancelled_step():
    sm = make_sm()
    sm.start()
    sm.on_success()
    sm.cancel()
    assert sm.retry() is True
    assert sm.current_step_name() == 'place'


def test_retry_keeps_recordings_of_the_run():
    sm = make_sm()
    sm.start()
    sm.add_recording(tip('clip_1'))
    sm.cancel()
    sm.retry()
    assert [r['id'] for r in sm.recordings] == ['clip_1']


def test_retry_only_from_failed():
    for setup in (lambda sm: None, lambda sm: sm.start(), lambda sm: (sm.start(), sm.pause())):
        sm = make_sm()
        setup(sm)
        before = sm.state
        assert sm.retry() is False
        assert sm.state == before
    sm = make_sm([{'name': 'pick_base'}])
    sm.start()
    sm.on_success()
    assert sm.retry() is False
    assert sm.state == 'success'


# ── Steps that must never be re-sent (liquid handling) ─────────────────────────

def _ot2_routine(max_retries=3):
    return RoutineStateMachine(
        [{'name': 'go_to', 'target_machine': 'ot2'}, {'name': 'ot2'}, {'name': 'home'}],
        max_retries=max_retries, no_retry={'ot2'})


def test_failed_no_retry_step_fails_the_run_without_redispatch():
    sm = _ot2_routine()
    sm.start()
    sm.acknowledge_dispatch()
    sm.on_success()
    sm.acknowledge_dispatch()
    sm.on_failure()
    assert sm.state == 'failed'
    assert sm.needs_dispatch is False
    assert sm.retry_count == 0
    assert sm.current_step_name() == 'ot2'
    assert 'not retried' in sm.error
    assert 'liquid' in sm.error


def test_other_steps_still_retry_when_some_steps_are_no_retry():
    sm = _ot2_routine()
    sm.start()
    sm.acknowledge_dispatch()
    sm.on_failure()
    assert sm.state == 'running'
    assert sm.retry_count == 1
    assert sm.needs_dispatch is True


def test_operator_retry_of_no_retry_step_needs_force():
    sm = _ot2_routine()
    sm.start()
    sm.on_success()
    sm.on_failure()
    assert sm.to_dict()['retry_needs_force'] is True
    assert sm.retry() is False
    assert sm.state == 'failed'
    assert sm.retry(force=True) is True
    assert sm.state == 'running'
    assert sm.current_step_name() == 'ot2'
    assert sm.to_dict()['retry_needs_force'] is False


def test_operator_retry_after_cancel_during_no_retry_step_needs_force():
    sm = _ot2_routine()
    sm.start()
    sm.on_success()
    sm.cancel()
    assert sm.retry() is False
    assert sm.retry(force=True) is True


def test_retry_needs_force_false_for_ordinary_failed_step():
    sm = _ot2_routine(max_retries=0)
    sm.start()
    sm.on_failure()
    assert sm.state == 'failed'
    assert sm.to_dict()['retry_needs_force'] is False
    assert sm.retry() is True


# ── Live step progress (e.g. OT-2 command and tip-check pass) ──────────────────

def _running(steps=None):
    sm = RoutineStateMachine(steps or [{'name': 'go_to'}, {'name': 'ot2'}], max_retries=1)
    sm.start()
    return sm


def test_step_progress_none_on_new():
    sm = RoutineStateMachine([{'name': 'ot2'}])
    assert sm.to_dict()['step_progress'] is None


def test_set_step_progress_tags_the_routine_step():
    sm = _running()
    sm.on_success()
    assert sm.set_step_progress({'phase': 'command', 'command': 1}) is True
    assert sm.to_dict()['step_progress'] == {'phase': 'command', 'command': 1, 'step': 1}


def test_step_progress_ignored_when_inactive_or_not_a_dict():
    sm = RoutineStateMachine([{'name': 'ot2'}])
    assert sm.set_step_progress({'phase': 'command'}) is False
    sm.start()
    assert sm.set_step_progress(['not', 'a', 'dict']) is False
    assert sm.step_progress is None


def test_step_progress_cleared_when_the_step_succeeds():
    sm = _running()
    sm.set_step_progress({'phase': 'homing'})
    sm.on_success()
    assert sm.step_progress is None


def test_step_progress_cleared_on_auto_retry_but_kept_when_the_run_fails():
    sm = _running([{'name': 'go_to'}])
    sm.set_step_progress({'phase': 'command', 'command': 2})
    sm.on_failure()  # retry 1 of 1
    assert sm.step_progress is None
    sm.set_step_progress({'phase': 'command', 'command': 3})
    sm.on_failure()  # retries exhausted
    assert sm.state == 'failed'
    assert sm.step_progress['command'] == 3


def test_step_progress_cleared_on_operator_retry_and_new_start():
    sm = _running([{'name': 'go_to'}])
    sm.set_step_progress({'phase': 'command'})
    sm.cancel()
    assert sm.step_progress is not None
    sm.retry()
    assert sm.step_progress is None
    sm.set_step_progress({'phase': 'command'})
    sm.cancel()
    sm.start()
    assert sm.step_progress is None


# ── Operator skip of a failed step ─────────────────────────────────────────────

def test_skip_moves_a_failed_routine_on_to_the_next_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick', 'target_machine': 'ot2'}])
    _fail_second_step(sm)
    assert sm.skip() is True
    assert sm.state == 'running'
    assert sm.current_step_idx == 2
    assert sm.retry_count == 0
    assert sm.error is None
    assert sm.needs_dispatch is True
    assert sm.to_dict()['skipped_steps'] == [1]


def test_skipping_the_last_step_completes_the_routine():
    sm = make_sm()
    _fail_second_step(sm)
    assert sm.skip() is True
    assert sm.state == 'success'
    assert sm.needs_dispatch is False
    assert sm.to_dict()['skipped_steps'] == [1]


def test_skip_after_cancel_skips_the_cancelled_step():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick', 'target_machine': 'ot2'}])
    sm.start()
    sm.cancel()
    assert sm.skip() is True
    assert sm.current_step_name() == 'place'


def test_skip_of_a_no_retry_step_needs_no_force():
    # Skipping never re-sends the step, so it cannot aspirate twice.
    sm = _ot2_routine()
    sm.start()
    sm.on_success()
    sm.on_failure()
    assert sm.skip() is True
    assert sm.current_step_name() == 'home'


def test_skip_only_from_failed():
    for setup in (lambda sm: None, lambda sm: sm.start(), lambda sm: (sm.start(), sm.pause())):
        sm = make_sm()
        setup(sm)
        before = (sm.state, sm.current_step_idx)
        assert sm.skip() is False
        assert (sm.state, sm.current_step_idx) == before


def test_skip_clears_step_progress_and_new_start_clears_skipped():
    sm = make_sm([{'name': 'pick_base'}, {'name': 'place', 'target_machine': 'ot2'}, {'name': 'pick', 'target_machine': 'ot2'}])
    sm.start()
    sm.set_step_progress({'phase': 'x'})
    sm.cancel()
    sm.skip()
    assert sm.step_progress is None
    sm.cancel()
    sm.start()
    assert sm.to_dict()['skipped_steps'] == []


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def test_elapsed_none_before_start():
    assert RoutineStateMachine([{'name': 'a'}], clock=FakeClock()).to_dict()['elapsed_s'] is None


def test_elapsed_counts_while_running_and_freezes_at_end():
    clock = FakeClock()
    sm = RoutineStateMachine([{'name': 'a'}], clock=clock)
    sm.start()
    clock.t += 5
    assert sm.to_dict()['elapsed_s'] == 5
    sm.on_success()
    clock.t += 30
    assert sm.to_dict()['elapsed_s'] == 5


def test_elapsed_resumes_on_retry_and_resets_on_start():
    clock = FakeClock()
    sm = RoutineStateMachine([{'name': 'a'}], max_retries=0, clock=clock)
    sm.start()
    clock.t += 2
    sm.on_failure()
    clock.t += 3
    assert sm.to_dict()['elapsed_s'] == 2
    assert sm.retry()
    clock.t += 1
    assert sm.to_dict()['elapsed_s'] == 6
    sm.cancel()
    sm.start()
    clock.t += 1
    assert sm.to_dict()['elapsed_s'] == 1


def test_timed_phase_reports_elapsed_capped_at_duration():
    clock = FakeClock()
    sm = RoutineStateMachine([{'name': 'shaker', 'pwm': 150, 'wait_time_s': 20}], clock=clock)
    sm.start()
    assert sm.start_timed_phase('shaker', 'hold', 20)
    clock.t += 7.5
    p = sm.to_dict()['step_progress']
    assert p == {'machine': 'shaker', 'phase': 'hold', 'duration_s': 20.0, 'step': 0, 'elapsed_s': 7.5}
    clock.t += 60
    assert sm.to_dict()['step_progress']['elapsed_s'] == 20.0
    sm.set_step_progress({'machine': 'shaker', 'phase': 'stopping'})
    assert 'elapsed_s' not in sm.to_dict()['step_progress']


def test_timed_phase_only_while_active():
    sm = RoutineStateMachine([{'name': 'shaker'}], clock=FakeClock())
    assert not sm.start_timed_phase('shaker', 'hold', 20)
    assert sm.to_dict()['step_progress'] is None

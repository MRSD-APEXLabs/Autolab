"""Tests for mock_executor.py — run with:  cd gcs/ui/mock && python3 -m pytest -q"""

import json
import urllib.error
import urllib.request

import pytest

from mock_executor import (
    FakeTipCheck,
    MockExecutor,
    STATUS_TOPIC,
    load_step_specs,
    start_tip_check_server,
)

STEP_TIME = 2.0


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def routine(*steps, max_retries=3, robot='robot_1'):
    return json.dumps({'steps': list(steps), 'max_retries': max_retries, 'robot': robot})


def status(ex):
    topic, payload = ex.status_message()
    assert topic == STATUS_TOPIC
    outer = json.loads(payload)
    return json.loads(outer['data'])  # real bridge wraps std_msgs/String as {"data": "<json>"}


def advance(ex, clock, seconds, dt=0.1):
    """Move the clock forward in small ticks; return every message published on the way."""
    out = []
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        out.extend(ex.step())
    return out


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def tips():
    return FakeTipCheck()


@pytest.fixture
def ex(clock, tips):
    return MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock, tip_check=tips)


def start(ex, *steps, **kw):
    ex.handle('cmd/routine_executor/start_routine_cmd', routine(*steps, **kw))
    ex.step()


GO = {'name': 'go_to', 'target_machine': 'ot2'}
PICK = {'name': 'pick', 'target_machine': 'ot2'}
HOME = {'name': 'home'}


# --- step specs are read from the real step_config.py ---------------------------------------

def test_step_specs_come_from_real_step_config():
    specs = load_step_specs()
    assert set(specs) == {'pick_base', 'place', 'place_base', 'pick', 'shaker', 'ot2',
                          'wait', 'go_to', 'home', 'go_to_pose'}
    assert specs['place']['required_params'] == ['target_machine']
    assert specs['go_to_pose']['required_params'] == ['x', 'y', 'yaw']
    assert specs['wait']['msg_type'] == 'wait'
    assert specs['ot2']['retryable'] is False
    assert specs['go_to']['retryable'] is True


# --- start / validation -----------------------------------------------------------------------

def test_before_any_routine_status_is_idle_with_no_steps(ex):
    s = status(ex)
    assert s['state'] == 'idle'
    assert s['steps'] == []


def test_valid_start_runs_first_step(ex):
    start(ex, GO, HOME)
    s = status(ex)
    assert s['state'] == 'running'
    assert s['current_step'] == 0
    assert s['current_step_name'] == 'go_to'


def test_unknown_step_is_rejected_like_real_executor(ex):
    start(ex, {'name': 'take_off'})
    assert status(ex)['state'] == 'idle'


def test_missing_required_param_is_rejected(ex):
    start(ex, {'name': 'place'})
    assert status(ex)['state'] == 'idle'


def test_start_ignored_while_running(ex):
    start(ex, GO, HOME)
    start(ex, HOME)
    assert len(status(ex)['steps']) == 2


# --- step progression -------------------------------------------------------------------------

def test_step_completes_after_step_time(ex, clock):
    start(ex, GO, HOME)
    advance(ex, clock, STEP_TIME - 0.3)
    assert status(ex)['current_step'] == 0
    advance(ex, clock, 0.5)
    assert status(ex)['current_step'] == 1


def test_wait_step_uses_its_time_s(ex, clock):
    start(ex, {'name': 'wait', 'time_s': 5}, HOME)
    advance(ex, clock, 4.5)
    assert status(ex)['current_step'] == 0
    advance(ex, clock, 1.0)
    assert status(ex)['current_step'] == 1


def test_full_routine_ends_in_success(ex, clock):
    start(ex, GO, HOME)
    advance(ex, clock, 2 * STEP_TIME + 1)
    s = status(ex)
    assert s['state'] == 'success'
    assert s['current_step_name'] is None


# --- pause / resume / cancel / retry ----------------------------------------------------------

def test_pause_lets_current_step_finish_but_holds_next(ex, clock):
    # Real executor: the robot keeps executing the dispatched step; pause only blocks the next dispatch.
    start(ex, GO, PICK, HOME)
    ex.handle('cmd/routine_executor/pause', '')
    advance(ex, clock, STEP_TIME + 0.5)
    s = status(ex)
    assert (s['state'], s['current_step']) == ('paused', 1)
    advance(ex, clock, 3 * STEP_TIME)
    assert status(ex)['current_step'] == 1
    ex.handle('cmd/routine_executor/resume', '')
    advance(ex, clock, STEP_TIME + 0.5)
    s = status(ex)
    assert (s['state'], s['current_step']) == ('running', 2)


def test_cancel_then_retry_resumes_same_step(ex, clock):
    start(ex, GO, PICK, HOME)
    advance(ex, clock, STEP_TIME + 0.5)
    ex.handle('cmd/routine_executor/cancel', '')
    ex.step()
    s = status(ex)
    assert (s['state'], s['error'], s['current_step']) == ('failed', 'cancelled', 1)
    advance(ex, clock, 3 * STEP_TIME)
    assert status(ex)['current_step'] == 1  # cancelled step does not keep running
    ex.handle('cmd/routine_executor/retry', '')
    ex.step()
    assert status(ex)['state'] == 'running'
    advance(ex, clock, STEP_TIME + 0.5)
    assert status(ex)['current_step'] == 2


def test_injected_failure_exhausts_retries_then_ui_retry_succeeds(clock, tips):
    # max_retries counts re-dispatches after the first attempt: 1 + 3 = 4 failures to exhaust.
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      tip_check=tips, fail_steps={'pick': 4})
    start(ex, GO, PICK, HOME)
    advance(ex, clock, STEP_TIME + 3 * STEP_TIME * 0.5)
    s = status(ex)
    assert (s['state'], s['retry_count']) == ('running', 3)  # 3 failed, last retry in flight
    advance(ex, clock, STEP_TIME)
    s = status(ex)
    assert s['state'] == 'failed'
    assert s['current_step'] == 1
    assert s['error'] == "Step 'pick' failed after 3 retries"
    ex.handle('cmd/routine_executor/retry', '')
    advance(ex, clock, 2 * STEP_TIME + 1)
    assert status(ex)['state'] == 'success'


def test_single_failure_shows_retry_count_then_recovers(clock, tips):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      tip_check=tips, fail_steps={'go_to': 1})
    start(ex, GO, HOME)
    advance(ex, clock, STEP_TIME + 0.2)
    s = status(ex)
    assert (s['state'], s['current_step'], s['retry_count']) == ('running', 0, 1)
    advance(ex, clock, STEP_TIME)
    assert status(ex)['current_step'] == 1


def test_force_fail_and_force_success_act_on_current_step(ex, clock):
    start(ex, GO, PICK, HOME)
    ex.force_success()
    ex.step()
    assert status(ex)['current_step'] == 1
    ex.force_fail()
    ex.step()
    assert status(ex)['retry_count'] == 1


# --- robot-side fakes -------------------------------------------------------------------------

ASPIRATE_A1 = {'action': 'aspirate', 'resource_name': 'tube_rack', 'well_indices': [0], 'volumes': [200]}
ASPIRATE_A2 = {'action': 'aspirate', 'resource_name': 'tube_rack', 'well_indices': [3], 'volumes': [200]}


def ot2(*aspirates):
    return {'name': 'ot2', 'parameters_json': {'steps': [
        {'action': 'pick_up_tips', 'resource_name': 'tip_rack', 'well_indices': [0]},
        *aspirates,
        {'action': 'drop_tips'},
    ]}}


def test_ot2_step_attaches_one_tip_check_per_aspirate(ex, clock):
    start(ex, ot2(ASPIRATE_A1, ASPIRATE_A2), HOME)
    advance(ex, clock, STEP_TIME + 0.5)
    recs = status(ex)['recordings']
    assert [r['type'] for r in recs] == ['tip_check', 'tip_check']
    assert len({r['id'] for r in recs}) == 2
    assert [r['aspirate'] for r in recs] == [ASPIRATE_A1, ASPIRATE_A2]
    assert [r['protocol_step'] for r in recs] == [1, 2]
    assert all(r['step'] == 0 for r in recs)
    first = recs[0]
    assert first['status'] == 'done' and first['anomaly_detected'] is False
    assert first['sample_count'] == 5 and len(first['samples']) == 5
    assert first['p_tip_empty'] < first['threshold']
    assert first['files']['crop'].startswith('/runs/')


def test_aspirate_with_tip_check_false_is_not_checked(ex, clock):
    skipped = {**ASPIRATE_A1, 'tip_check': False}  # e.g. clear liquid
    start(ex, ot2(skipped, ASPIRATE_A2), HOME)
    seen = _progress_trace(ex, clock, STEP_TIME + 0.5)
    recs = status(ex)['recordings']
    assert [r['aspirate'] for r in recs] == [ASPIRATE_A2]
    assert [r['protocol_step'] for r in recs] == [2]
    assert {p['command'] for p in seen if p['phase'] == 'tip_check'} == {2}


def test_clog_injection_flags_the_nth_tip_check_and_the_protocol_goes_on(clock):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      tip_check=FakeTipCheck(clog_checks={2}))
    start(ex, ot2(ASPIRATE_A1), ot2(ASPIRATE_A2), HOME)
    advance(ex, clock, 3 * STEP_TIME + 1)
    s = status(ex)
    assert s['state'] == 'success'
    assert [r['anomaly_detected'] for r in s['recordings']] == [False, True]
    assert s['recordings'][1]['p_tip_empty'] >= s['recordings'][1]['threshold']


def test_tip_check_service_down_is_reported_and_the_step_still_succeeds(clock):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      tip_check=FakeTipCheck(error_checks={1}))
    start(ex, ot2(ASPIRATE_A1, ASPIRATE_A2), HOME)
    advance(ex, clock, STEP_TIME + 0.5)
    s = status(ex)
    assert s['current_step'] == 1
    first, second = s['recordings']
    assert first['status'] == 'error' and 'unreachable' in first['error']
    assert second['status'] == 'done'


def test_tip_check_down_for_every_check(clock):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      tip_check=FakeTipCheck(down=True))
    start(ex, ot2(ASPIRATE_A1))
    advance(ex, clock, STEP_TIME + 0.5)
    assert [r['status'] for r in status(ex)['recordings']] == ['error']


def test_ot2_failure_after_the_aspirate_is_not_retried_and_needs_a_forced_retry(clock, tips):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock, tip_check=tips,
                      fail_steps={'ot2': 1})
    start(ex, ot2(ASPIRATE_A1), HOME, max_retries=3)
    advance(ex, clock, STEP_TIME)
    s = status(ex)
    assert s['state'] == 'failed'
    assert s['retry_count'] == 0
    assert 'not retried' in s['error']
    assert s['retry_needs_force'] is True
    assert len(s['recordings']) == 1  # the aspirate happened and was checked before the failure
    ex.handle('cmd/routine_executor/retry', '')
    ex.step()
    assert status(ex)['state'] == 'failed'
    ex.handle('cmd/routine_executor/retry', '{"force": true}')
    advance(ex, clock, 2 * STEP_TIME + 1)
    s = status(ex)
    assert s['state'] == 'success'
    assert len(s['recordings']) == 2


def test_manipulation_steps_publish_phases_under_robot_namespace(ex, clock):
    start(ex, GO, PICK, HOME)
    during_go = advance(ex, clock, STEP_TIME - 0.2)
    during_pick = advance(ex, clock, STEP_TIME)  # last 0.2 s of go_to, then most of pick
    phase_topic = 'ros2/robot_1/behavior/manipulation_phase'
    go_phases = {json.loads(p)['data'] for t, p in during_go if t == phase_topic}
    pick_phases = {json.loads(p)['data'] for t, p in during_pick if t == phase_topic}
    assert go_phases <= {'IDLE'}
    assert {'PLANNING', 'RUNNING_ACT'} <= pick_phases
    planning = [json.loads(p)['data'] for t, p in during_pick if t == 'ros2/planning_state']
    assert 'SUCCESS' in planning


# --- fake tip-check service (debug console's standalone check + the card's images) ----------

def _get(url, body=None):
    req = urllib.request.Request(url, data=json.dumps(body).encode() if body is not None else None,
                                 headers={'Content-Type': 'application/json'})
    return urllib.request.urlopen(req)


def test_tip_check_http_server_answers_check_jobs_health_and_images():
    server = start_tip_check_server(FakeTipCheck(), host='127.0.0.1', port=0)
    try:
        base = f'http://127.0.0.1:{server.server_address[1]}'
        with _get(f'{base}/check', {'step': 'debug tip check'}) as r:
            assert r.headers['Access-Control-Allow-Origin'] == '*'
            job = json.loads(r.read())
        assert job['status'] == 'done'
        assert 0 <= job['p_tip_empty'] <= 1
        assert job['files']['crop'].startswith('/runs/')
        with _get(f'{base}/jobs') as r:
            assert any(j['clip_id'] == job['clip_id'] for j in json.loads(r.read()))
        with _get(f'{base}/health') as r:
            assert json.loads(r.read())['camera_present'] is True
        with _get(base + job['files']['crop']) as r:
            assert r.headers['Content-Type'] == 'image/png'
            assert r.read().startswith(b'\x89PNG')
        with pytest.raises(urllib.error.HTTPError) as e:
            _get(f'{base}/nope')
        assert e.value.code == 404
    finally:
        server.shutdown()


def test_down_tip_check_service_fails_its_http_checks():
    server = start_tip_check_server(FakeTipCheck(down=True), host='127.0.0.1', port=0)
    try:
        with pytest.raises(urllib.error.HTTPError) as e:
            _get(f'http://127.0.0.1:{server.server_address[1]}/check', {})
        assert e.value.code == 503
    finally:
        server.shutdown()


# --- live OT-2 progress (status.step_progress, like lab_machine_executive's ot2_progress) -----

def _progress_trace(ex, clock, seconds, dt=0.01):
    seen = []
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        ex.step()
        p = status(ex)['step_progress']
        if p and (not seen or p != seen[-1]):
            seen.append(p)
    return seen


def test_ot2_step_reports_each_command_then_homing(ex, clock):
    start(ex, ot2(ASPIRATE_A1), HOME)
    seen = _progress_trace(ex, clock, STEP_TIME - 0.1)
    phases = [p['phase'] for p in seen]
    assert phases[0] == 'connecting'
    assert [p['command'] for p in seen if p['phase'] == 'command'] == [0, 1, 2]
    assert [p['action'] for p in seen if p['phase'] == 'command'] == ['pick_up_tips', 'aspirate', 'drop_tips']
    assert phases.index('homing') > max(i for i, ph in enumerate(phases) if ph == 'command')
    assert all(p['machine'] == 'ot2' and p['step'] == 0 and p['commands'] == 3 for p in seen)


def test_tip_check_progress_goes_pass_by_pass_before_the_result(ex, clock):
    start(ex, ot2(ASPIRATE_A1), HOME)
    seen = _progress_trace(ex, clock, STEP_TIME - 0.1)
    checks = [p['tip_check'] for p in seen if p['phase'] == 'tip_check']
    assert all(p['command'] == 1 for p in seen if p['phase'] == 'tip_check')
    assert sorted({c['pass'] for c in checks}) == [1, 2, 3, 4, 5]
    assert {'submitting', 'running', 'done'} <= {c['status'] for c in checks}
    last = checks[-1]
    assert last['pass'] == 5 and last['status'] == 'done' and len(last['samples']) == 5
    assert status(ex)['recordings'][0]['samples'] == last['samples']


def test_step_progress_is_cleared_when_the_ot2_step_succeeds(ex, clock):
    start(ex, ot2(ASPIRATE_A1), HOME)
    advance(ex, clock, STEP_TIME + 0.2)
    s = status(ex)
    assert s['current_step'] == 1 and s['step_progress'] is None


def test_failed_ot2_step_keeps_its_progress_marked_failed(clock, tips):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock, tip_check=tips,
                      fail_steps={'ot2': 1})
    start(ex, ot2(ASPIRATE_A1), HOME)
    advance(ex, clock, STEP_TIME)
    p = status(ex)['step_progress']
    assert status(ex)['state'] == 'failed'
    assert p['phase'] == 'failed' and p['step'] == 0


def test_ot2_time_sets_the_ot2_step_length_only(clock, tips):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, ot2_time=10.0, clock=clock, tip_check=tips)
    start(ex, ot2(ASPIRATE_A1), HOME)
    advance(ex, clock, STEP_TIME + 0.5)
    assert status(ex)['current_step'] == 0
    advance(ex, clock, 10.0 - STEP_TIME)
    assert status(ex)['current_step'] == 1

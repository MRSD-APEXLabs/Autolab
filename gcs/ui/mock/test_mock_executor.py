"""Tests for mock_executor.py — run with:  cd gcs/ui/mock && python3 -m pytest -q"""

import json
import urllib.error
import urllib.request

import pytest

from mock_executor import (
    FakeAnomalyStore,
    MockExecutor,
    STATUS_TOPIC,
    load_step_specs,
    start_anomaly_server,
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
def store(clock):
    return FakeAnomalyStore(clock=clock)


@pytest.fixture
def ex(clock, store):
    return MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock, anomaly_store=store)


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


def test_injected_failure_exhausts_retries_then_ui_retry_succeeds(clock, store):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      anomaly_store=store, fail_steps={'pick': 3})
    start(ex, GO, PICK, HOME)
    advance(ex, clock, STEP_TIME + 3 * STEP_TIME + 1)
    s = status(ex)
    assert s['state'] == 'failed'
    assert s['current_step'] == 1
    assert s['error'] == "Step 'pick' failed after 3 retries"
    ex.handle('cmd/routine_executor/retry', '')
    advance(ex, clock, 2 * STEP_TIME + 1)
    assert status(ex)['state'] == 'success'


def test_single_failure_shows_retry_count_then_recovers(clock, store):
    ex = MockExecutor(load_step_specs(), step_time=STEP_TIME, clock=clock,
                      anomaly_store=store, fail_steps={'go_to': 1})
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

def test_ot2_step_attaches_one_recording_per_aspirate(ex, clock, store):
    ot2 = {'name': 'ot2', 'parameters_json': {'steps': [
        {'action': 'pick_up_tips', 'resource_name': 'tip_rack', 'well_indices': [0]},
        {'action': 'aspirate', 'resource_name': 'tube_rack', 'well_indices': [0], 'volumes': [200]},
        {'action': 'aspirate', 'resource_name': 'tube_rack', 'well_indices': [1], 'volumes': [200]},
    ]}}
    start(ex, ot2, HOME)
    advance(ex, clock, STEP_TIME + 0.5)
    recs = status(ex)['recordings']
    assert len(recs) == 2
    assert all(store.result(r) is not None for r in recs)
    assert json.loads(store.result(recs[1])['step'])['well_indices'] == [1]


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


# --- fake clog-detection service --------------------------------------------------------------

def test_anomaly_job_progresses_to_done(clock):
    store = FakeAnomalyStore(clock=clock)
    store.add('c1', {'action': 'aspirate', 'resource_name': 'tube_rack', 'well_indices': [0]})
    seen = []
    for _ in range(40):
        seen.append(store.result('c1')['status'])
        clock.t += 0.5
    assert seen[0] == 'recording'
    assert 'queued' in seen and 'running' in seen
    final = store.result('c1')
    assert final['status'] == 'done' and final['anomaly_detected'] is False


def test_anomaly_clog_injection_marks_nth_clip(clock):
    store = FakeAnomalyStore(clock=clock, clog_clips={2})
    store.add('a', {})
    store.add('b', {})
    clock.t += 60
    assert store.result('a')['anomaly_detected'] is False
    assert store.result('b')['anomaly_detected'] is True


def test_anomaly_http_server_serves_results_with_cors(clock):
    store = FakeAnomalyStore(clock=clock)
    store.add('c1', {})
    server = start_anomaly_server(store, host='127.0.0.1', port=0)
    try:
        base = f'http://127.0.0.1:{server.server_address[1]}'
        with urllib.request.urlopen(f'{base}/result/c1') as r:
            assert r.headers['Access-Control-Allow-Origin'] == '*'
            assert json.loads(r.read())['clip_id'] == 'c1'
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(f'{base}/result/nope')
        assert e.value.code == 404
    finally:
        server.shutdown()

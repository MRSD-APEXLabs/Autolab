import threading
import time

from act_inference.runner import ActRunner


def wait_idle(runner, timeout=2.0):
    deadline = time.monotonic() + timeout
    while runner.running:
        assert time.monotonic() < deadline, 'run did not end'
        time.sleep(0.005)


def test_done_run_reports_done_and_started():
    runner = ActRunner(lambda stop: 'done')
    assert runner.status()['started'] == -1.0 and runner.status()['result'] is None
    assert runner.start() == (True, 'started')
    wait_idle(runner)
    s = runner.status()
    assert s['running'] is False and s['result'] == 'done' and s['started'] > 0 and s['phase'] == 'idle'


def test_status_while_running():
    gate = threading.Event()
    runner = ActRunner(lambda stop: (gate.wait(2), 'done')[1])
    runner.start()
    s = runner.status()
    assert s['running'] is True and s['result'] is None and s['phase'] in ('preparing', 'running')
    gate.set()
    wait_idle(runner)


def test_second_start_while_running_is_refused():
    gate = threading.Event()
    runs = []
    runner = ActRunner(lambda stop: (runs.append(1), gate.wait(2), 'done')[2])
    runner.start()
    assert runner.start() == (False, 'already running')
    gate.set()
    wait_idle(runner)
    assert len(runs) == 1


def test_abort_ends_run_as_stopped():
    runner = ActRunner(lambda stop: (stop.wait(2), 'stopped')[1])
    runner.start()
    runner.abort(wait=1.0)
    assert runner.status()['result'] == 'stopped' and not runner.running


def test_abort_during_prepare_never_runs():
    ran = []

    def prepare(check_abort):
        while True:
            check_abort()
            time.sleep(.01)
    runner = ActRunner(lambda stop: ran.append(1) or 'done', prepare=prepare)
    runner.start()
    time.sleep(.05)
    runner.abort(wait=1.0)
    assert runner.status()['result'] == 'stopped' and not ran


def test_exception_fails_and_runner_restarts():
    runs = iter([RuntimeError('No fresh policy action chunk'), 'done'])

    def run(stop):
        r = next(runs)
        if isinstance(r, Exception):
            raise r
        return r
    runner = ActRunner(run)
    runner.start()
    wait_idle(runner)
    assert runner.status()['result'] == 'failed' and 'No fresh policy' in runner.status()['message']
    first = runner.status()['started']
    runner.start()
    wait_idle(runner)
    assert runner.status()['result'] == 'done' and runner.status()['started'] > first


def test_prepare_failure_fails_the_run():
    def prepare(check_abort):
        raise RuntimeError('/camera_hub/select_zedx_nano is not available')
    ran = []
    runner = ActRunner(lambda stop: ran.append(1) or 'done', prepare=prepare)
    runner.start()
    wait_idle(runner)
    assert runner.status()['result'] == 'failed' and 'select_zedx_nano' in runner.status()['message'] and not ran


def test_on_done_gets_final_status():
    got = []
    runner = ActRunner(lambda stop: 'done', on_done=got.append)
    runner.start()
    wait_idle(runner)
    assert got and got[0]['result'] == 'done' and got[0]['running'] is False


def test_base_target_skips_policy_prepare_and_does_not_change_next_run():
    prepared, ran = [], []
    runner = ActRunner(lambda stop: ran.append('policy') or 'done',
                        prepare=lambda check: prepared.append('camera'))
    runner.start(target='base', run=lambda stop: ran.append('base') or 'done', prepare=lambda check: check())
    wait_idle(runner)
    assert runner.status()['target'] == 'base' and runner.status()['result'] == 'done'
    assert not prepared and ran == ['base']
    runner.start(target='shaker')
    wait_idle(runner)
    assert runner.status()['target'] == 'shaker' and runner.status()['result'] == 'done'
    assert prepared == ['camera'] and ran == ['base', 'policy']

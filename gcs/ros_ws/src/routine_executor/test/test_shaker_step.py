import pytest

from routine_executor.shaker_step import ShakerPhase, hold_time_s, is_timed, validate

TIMED = {'name': 'shaker', 'pwm': 150, 'wait_time_s': 20}


@pytest.mark.parametrize('step', [
    TIMED,
    {'name': 'shaker', 'pwm': 0},
    {'name': 'shaker', 'pwm': 255, 'wait_time_s': 0},
    {'name': 'shaker', 'pwm': 100, 'wait_time_s': 2.5},
])
def test_valid_steps_pass(step):
    validate(step)


@pytest.mark.parametrize('step', [
    {'name': 'shaker', 'pwm': 256},
    {'name': 'shaker', 'pwm': -1},
    {'name': 'shaker', 'pwm': 1.5},
    {'name': 'shaker', 'pwm': True},
    {'name': 'shaker', 'pwm': '150'},
    {'name': 'shaker', 'pwm': 150, 'wait_time_s': -1},
    {'name': 'shaker', 'pwm': 150, 'wait_time_s': float('inf')},
    {'name': 'shaker', 'pwm': 150, 'wait_time_s': '20'},
])
def test_invalid_steps_raise(step):
    with pytest.raises(ValueError):
        validate(step)


def test_hold_and_timed():
    assert hold_time_s(TIMED) == 20.0
    assert is_timed(TIMED)
    assert hold_time_s({'pwm': 150}) == 0.0
    assert not is_timed({'pwm': 150})
    assert not is_timed({'pwm': 0, 'wait_time_s': 20})  # a stop never holds


def test_timed_step_runs_start_hold_stop():
    p = ShakerPhase()
    assert p.dispatch_pwm(4, TIMED) == 150
    assert p.on_success(TIMED) == 'hold'
    p.on_hold_done()
    assert p.phase == 'stop'
    assert p.on_success(TIMED) == 'done'
    assert p.phase is None


def test_untimed_step_is_done_after_the_pwm_is_set():
    p = ShakerPhase()
    step = {'name': 'shaker', 'pwm': 150}
    assert p.dispatch_pwm(0, step) == 150
    assert p.on_success(step) == 'done'


def test_retry_after_failed_start_starts_again():
    p = ShakerPhase()
    p.dispatch_pwm(2, TIMED)
    assert p.dispatch_pwm(2, TIMED) == 150


def test_retry_after_failed_stop_only_resends_the_stop():
    p = ShakerPhase()
    p.dispatch_pwm(2, TIMED)
    p.on_success(TIMED)
    p.on_hold_done()
    assert p.dispatch_pwm(2, TIMED) == 0
    assert p.on_success(TIMED) == 'done'


def test_a_later_shaker_step_starts_fresh():
    p = ShakerPhase()
    p.dispatch_pwm(2, TIMED)
    p.on_success(TIMED)
    p.on_hold_done()
    assert p.dispatch_pwm(5, TIMED) == 150

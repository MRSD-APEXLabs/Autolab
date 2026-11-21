"""One routine shaker step: {'name': 'shaker', 'pwm': 150, 'wait_time_s': 20}.

A timed step (pwm > 0 and wait_time_s > 0) runs in three phases inside the same routine step:
start (PWM sent) -> hold (wait_time_s) -> stop (PWM 0 sent); the step succeeds once the stop is
acknowledged. Without wait_time_s (or with 0) the PWM is just set and left, so PWM 0 stops it.
No ROS imports: the node drives this, the tests drive it directly.
"""
import math


def _is_number(value) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def validate(step: dict) -> None:
    """Raise ValueError when the step's pwm / wait_time_s would be rejected."""
    pwm = step.get('pwm')
    if not _is_number(pwm) or pwm != int(pwm) or not 0 <= pwm <= 255:
        raise ValueError(f'shaker pwm must be an integer from 0 to 255, got {pwm!r}')
    wait = step.get('wait_time_s', 0)
    if not _is_number(wait) or wait < 0:
        raise ValueError(f'shaker wait_time_s must be a nonnegative number, got {wait!r}')


def hold_time_s(step: dict) -> float:
    return float(step.get('wait_time_s', 0) or 0)


def is_timed(step: dict) -> bool:
    return int(step.get('pwm', 0)) > 0 and hold_time_s(step) > 0


class ShakerPhase:
    """Which phase the current shaker step is in; keyed by step index so a retry knows."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.step_idx = None
        self.phase = None  # None | 'start' | 'hold' | 'stop'

    def dispatch_pwm(self, step_idx: int, step: dict) -> int:
        """PWM to send when the step is (re)dispatched.

        A retry after the stop was sent re-sends the stop: never shake again for another hold.
        """
        if self.step_idx == step_idx and self.phase == 'stop':
            return 0
        self.step_idx = step_idx
        self.phase = 'start'
        return int(step['pwm'])

    def on_success(self, step: dict) -> str:
        """The robot acknowledged the PWM just sent: 'hold' (start the timer) or 'done'."""
        if self.phase == 'start' and is_timed(step):
            self.phase = 'hold'
            return 'hold'
        self.reset()
        return 'done'

    def on_hold_done(self) -> None:
        self.phase = 'stop'

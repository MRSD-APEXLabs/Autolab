"""Schedule PWM 0 in GCS, independently of the browser's lifetime."""
import math
import threading


def validate(pwm, wait_time_s):
    if isinstance(pwm, bool) or not isinstance(pwm, (int, float)) or not math.isfinite(pwm):
        raise ValueError('PWM must be an integer from 0 to 255')
    if pwm != int(pwm) or not 0 <= pwm <= 255:
        raise ValueError('PWM must be an integer from 0 to 255')
    if (isinstance(wait_time_s, bool) or not isinstance(wait_time_s, (int, float))
            or not math.isfinite(wait_time_s) or wait_time_s < 0):
        raise ValueError('Wait time must be a finite, nonnegative number')
    return int(pwm), float(wait_time_s)


class ShakerTimeout:
    def __init__(self, send, timer_factory=threading.Timer):
        self.send = send
        self.timer_factory = timer_factory
        self.lock = threading.Lock()
        self.timer = None
        self.generation = 0

    def start(self, pwm, wait_time_s):
        pwm, wait_time_s = validate(pwm, wait_time_s)
        with self.lock:
            self.generation += 1
            generation = self.generation
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.send(pwm)
            # A zero duration preserves manual operation. PWM 0 is a stop now.
            if pwm > 0 and wait_time_s > 0:
                self.timer = self.timer_factory(wait_time_s, lambda: self._stop(generation))
                self.timer.daemon = True
                self.timer.start()

    def _stop(self, generation):
        with self.lock:
            if generation != self.generation:
                return
            self.timer = None
            self.send(0)

    def cancel(self):
        with self.lock:
            self.generation += 1
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None

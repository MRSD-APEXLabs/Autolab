"""Lifecycle of one ACT run at a time on a background thread, without ROS (node.py wraps it).

status() is what act_inference_node publishes on /act_inference/status: `running`, `result`
(None until a run ends, then done | stopped | failed), `message`, `phase` (idle | preparing |
running) and `started`, the run's time.monotonic() start (-1.0 before the first run) that
tells one run's status from the previous one's.
"""
import threading
import time


class RunAborted(Exception):
    """Raised by check_abort() inside prepare() once the run has been stopped."""


class ActRunner:
    def __init__(self, run, prepare=None, on_done=None):
        """`run(stop)` does the rollout and returns 'done' or 'stopped'; `prepare(check_abort)`
        runs first on the same thread; `on_done(status)` gets the final status."""
        self._run, self._prepare, self._on_done = run, prepare, on_done
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._status = {'running': False, 'result': None, 'message': '', 'phase': 'idle', 'started': -1.0}

    @property
    def running(self):
        with self._lock:
            return self._status['running']

    def status(self):
        with self._lock:
            return dict(self._status)

    def start(self):
        with self._lock:
            if self._status['running']:
                return False, 'already running'
            self._stop.clear()
            self._status = {'running': True, 'result': None, 'message': '', 'phase': 'preparing',
                            'started': time.monotonic()}
            self._thread = threading.Thread(target=self._work, name='act-run', daemon=True)
            self._thread.start()
        return True, 'started'

    def abort(self, wait=0.0):
        self._stop.set()
        thread = self._thread
        if wait > 0 and thread is not None and thread is not threading.current_thread():
            thread.join(wait)

    def _check_abort(self):
        if self._stop.is_set():
            raise RunAborted('stopped')

    def _set(self, **fields):
        with self._lock:
            self._status.update(fields)

    def _work(self):
        try:
            if self._prepare is not None:
                self._prepare(self._check_abort)
            self._check_abort()
            self._set(phase='running')
            outcome = self._run(self._stop)
            if outcome not in ('done', 'stopped'):
                raise RuntimeError(f'Unexpected rollout result: {outcome!r}')
            result, message = (('stopped', 'stopped') if self._stop.is_set() or outcome == 'stopped'
                               else ('done', 'rollout completed'))
        except RunAborted:
            result, message = 'stopped', 'stopped'
        except Exception as exc:
            result, message = 'failed', f'{type(exc).__name__}: {exc}'
        self._set(running=False, result=result, message=message, phase='idle')
        if self._on_done is not None:
            self._on_done(self.status())

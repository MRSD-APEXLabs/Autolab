class RoutineStateMachine:
    """Pure state machine — no ROS2 dependency. Tracks routine execution progress."""

    def __init__(self, steps: list, max_retries: int = 3):
        self.steps = list(steps)  # list[dict], each dict has at least {'name': str}
        self.max_retries = max_retries
        self.state = 'idle'          # idle | running | success | failed
        self.current_step_idx = 0
        self.retry_count = 0
        self.error = None
        self.needs_dispatch = False  # True when a command needs to be published

    def start(self):
        if self.state == 'running':
            raise RuntimeError('Routine already running')
        self.state = 'running'
        self.current_step_idx = 0
        self.retry_count = 0
        self.error = None
        self.needs_dispatch = True

    def current_step(self) -> dict:
        if self.current_step_idx >= len(self.steps):
            return {}
        return self.steps[self.current_step_idx]

    def current_step_name(self) -> str:
        return self.current_step().get('name', '')

    def on_success(self):
        if self.state != 'running':
            return
        self.retry_count = 0
        self.current_step_idx += 1
        if self.current_step_idx >= len(self.steps):
            self.state = 'success'
            self.needs_dispatch = False
        else:
            self.needs_dispatch = True

    def on_failure(self):
        if self.state != 'running':
            return
        self.retry_count += 1
        if self.retry_count >= self.max_retries:
            self.state = 'failed'
            self.error = (
                f"Step '{self.current_step_name()}' failed after {self.max_retries} retries"
            )
            self.needs_dispatch = False
        else:
            self.needs_dispatch = True  # retry: re-dispatch same step

    def cancel(self):
        if self.state != 'running':
            return
        self.state = 'failed'
        self.error = 'cancelled'
        self.needs_dispatch = False

    def acknowledge_dispatch(self):
        self.needs_dispatch = False

    def to_dict(self) -> dict:
        return {
            'state': self.state,
            'steps': self.steps,
            'current_step': self.current_step_idx,
            'current_step_name': (
                self.current_step_name()
                if self.state == 'running' and self.current_step_idx < len(self.steps)
                else None
            ),
            'retry_count': self.retry_count,
            'max_retries': self.max_retries,
            'error': self.error,
        }

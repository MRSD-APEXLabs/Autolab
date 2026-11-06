"""Pure LED pattern selection from behavior tree action state. No ROS dependency.

Each watched BT action has an ``<label>_active`` flag (published by the BT engine every tick)
and an ``<label>_status`` (published by its executive at 20 Hz, FAILURE by default even when
idle). Only status reported while the action is active counts.
"""

# behavior_tree_msgs/Status values
FAILURE = 0
RUNNING = 1
SUCCESS = 2

def topic_base(label):
    """BT label -> topic stem, as bt::Action::get_{published,subscribed}_topic_name do."""
    return label.lower().replace(' ', '_')


IDLE = 'I'
READY = 'R'
FAULT = 'E'


class _Action:
    def __init__(self, letter):
        self.letter = letter
        self.active = False
        self.active_t = None      # time of last _active message
        self.active_since = None  # start of current activation
        self.status = None        # last status seen during current activation
        self.seen_running = False
        self.reported = False     # terminal status of this run already latched


class LedLogic:
    """Turns action activity into one Nano status letter.

    ``groups`` is an ordered list of ``(letter, [action labels])``; earlier groups win when
    several actions are running at once.
    """

    def __init__(self, groups, success_hold_s=5.0, stale_s=1.0, grace_s=0.5):
        self._success_hold_s = success_hold_s
        self._stale_s = stale_s
        self._grace_s = grace_s
        self._order = [letter for letter, _ in groups]
        self._actions = {}
        for letter, labels in groups:
            for label in labels:
                self._actions[label] = _Action(letter)
        self._latch = None    # FAULT or READY
        self._latch_t = None

    def _is_active(self, a, now):
        return a.active and a.active_t is not None and now - a.active_t <= self._stale_s

    def _started(self):
        self._latch = None

    def _check_terminal(self, a, now):
        if a.reported or a.status not in (FAILURE, SUCCESS):
            return
        # Executives report their previous/default status for a moment after activation,
        # so a terminal status only counts once this run was seen RUNNING or after a grace period.
        if not a.seen_running and now - a.active_since < self._grace_s:
            return
        a.reported = True
        self._latch = FAULT if a.status == FAILURE else READY
        self._latch_t = now

    def on_active(self, label, active, now):
        a = self._actions.get(label)
        if a is None:
            return
        was_active = self._is_active(a, now)
        if active and not was_active:
            a.active_since = now
            a.status = None
            a.seen_running = False
            a.reported = False
            self._started()
        elif not active and was_active:
            self._check_terminal(a, now)
        a.active = active
        a.active_t = now

    def on_status(self, label, status, now):
        a = self._actions.get(label)
        if a is None or not self._is_active(a, now):
            return
        if status == RUNNING:
            if a.status != RUNNING:
                a.reported = False
                self._started()
            a.seen_running = True
        a.status = status
        self._check_terminal(a, now)

    def tick(self, now):
        running = set()
        for a in self._actions.values():
            if not self._is_active(a, now):
                continue
            self._check_terminal(a, now)
            if not a.reported:
                running.add(a.letter)
        for letter in self._order:
            if letter in running:
                return letter
        if self._latch == READY and now - self._latch_t >= self._success_hold_s:
            self._latch = None
        return self._latch or IDLE

"""Pure phrase selection from behavior tree action state. No ROS dependency.

As in led_status: each watched BT action has an ``<label>_active`` flag (published by the BT engine every tick)
and an ``<label>_status`` (published by its executive at 20 Hz, FAILURE by default even when idle). A run's start
is announced when the BT activates the action, its end when the run reports SUCCESS or FAILURE. A terminal
status only counts once the run has reported RUNNING, or after ``grace_s`` of activity, because an executive
can still report the previous run's status for a tick after activation.

The phrases name the destination or machine from the last command (go_to_location_command,
manipulation_command), taken when the action activates, so a command the executive ignores mid-run does not
change what the end of the run says.
"""

# behavior_tree_msgs/Status values
FAILURE = 0
RUNNING = 1
SUCCESS = 2

WATCHED = ['Go To Location', 'Navigate To Pose', 'Pick Object', 'Place Object', 'Pick Base Object',
           'Place Base Object', 'Execute OT2 Protocol', 'Execute Shaker Protocol']

# Command names (lower case) -> what to say; anything else is said as written, "_"/"-" as spaces.
SPOKEN_NAMES = {'ot2': 'the Opentrons', 'ot-2': 'the Opentrons', 'opentrons': 'the Opentrons',
                'shaker': 'the shaker', 'home': 'home'}


def topic_base(label):
    """BT label -> topic stem, as bt::Action::get_{published,subscribed}_topic_name do."""
    return label.lower().replace(' ', '_')


def spoken_name(name, fallback):
    key = (name or '').strip().lower()
    if key in SPOKEN_NAMES:
        return SPOKEN_NAMES[key]
    words = ' '.join(key.replace('_', ' ').replace('-', ' ').split())
    return f'the {words}' if words else fallback


def phrases(label, context):
    """(start, success, failure) for one run of the BT action `label`."""
    place = spoken_name(context.get('location'), 'the goal')
    to_place = place if place == 'home' else f'to {place}'
    at_place = place if place == 'home' else f'at {place}'
    machine = spoken_name(context.get('machine'), 'the machine')
    plate = ' '.join((context.get('object') or 'well plate').replace('_', ' ').split())
    table = {
        'Go To Location': (f'Moving {to_place}.', f'Arrived {at_place}.', f'Could not reach {place}.'),
        'Navigate To Pose': ('Moving to the goal pose.', 'Arrived at the goal pose.', 'Could not reach the goal pose.'),
        'Pick Object': (f'Picking up the {plate} from {machine}.', f'Picked up the {plate}.',
                        f'Could not pick up the {plate}.'),
        'Place Object': (f'Placing the {plate} on {machine}.', f'Placed the {plate} on {machine}.',
                         f'Could not place the {plate} on {machine}.'),
        'Pick Base Object': (f'Picking up the {plate} from the robot.', f'Picked up the {plate} from the robot.',
                             f'Could not pick up the {plate} from the robot.'),
        'Place Base Object': (f'Placing the {plate} on the robot.', f'Placed the {plate} on the robot.',
                              f'Could not place the {plate} on the robot.'),
        'Execute OT2 Protocol': ('Starting the Opentrons protocol.', 'The Opentrons protocol is done.',
                                 'The Opentrons protocol failed.'),
        'Execute Shaker Protocol': ('Starting the shaker.', 'The shaker is done.', 'The shaker failed.'),
    }
    return table[label]


class _Action:
    def __init__(self):
        self.active = False
        self.active_t = None      # time of last _active message
        self.active_since = None  # start of current activation
        self.status = None        # last status seen during current activation
        self.seen_running = False
        self.reported = False     # this run's end already announced
        self.context = {}


class VoiceLogic:
    """Turns action activity into phrases to say. Every method returns the (possibly empty) list of new phrases."""

    def __init__(self, labels=WATCHED, stale_s=1.0, grace_s=0.5):
        self._stale_s = stale_s
        self._grace_s = grace_s
        self._actions = {label: _Action() for label in labels}
        self._commanded = {}

    def on_location_command(self, name):
        self._commanded['location'] = name

    def on_manipulation_command(self, target_machine, object_type):
        self._commanded['machine'] = target_machine
        self._commanded['object'] = object_type

    def _is_active(self, a, now):
        return a.active and a.active_t is not None and now - a.active_t <= self._stale_s

    def _start(self, label, a):
        a.reported = False
        return [phrases(label, a.context)[0]]

    def _check_terminal(self, label, a, now):
        if a.reported or a.status not in (FAILURE, SUCCESS):
            return []
        if not a.seen_running and now - a.active_since < self._grace_s:
            return []
        a.reported = True
        start, success, failure = phrases(label, a.context)
        return [success if a.status == SUCCESS else failure]

    def on_active(self, label, active, now):
        a = self._actions.get(label)
        if a is None:
            return []
        said = []
        was_active = self._is_active(a, now)
        if active and not was_active:
            a.active_since = now
            a.status = None
            a.seen_running = False
            a.context = dict(self._commanded)
            said = self._start(label, a)
        elif not active and was_active:
            said = self._check_terminal(label, a, now)
        a.active = active
        a.active_t = now
        return said

    def on_status(self, label, status, now):
        a = self._actions.get(label)
        if a is None or not self._is_active(a, now):
            return []
        said = []
        if status == RUNNING:
            if a.reported:          # the same activation runs again (a retry)
                said = self._start(label, a)
            a.seen_running = True
        a.status = status
        return said + self._check_terminal(label, a, now)

    def tick(self, now):
        said = []
        for label, a in self._actions.items():
            if self._is_active(a, now):
                said += self._check_terminal(label, a, now)
        return said

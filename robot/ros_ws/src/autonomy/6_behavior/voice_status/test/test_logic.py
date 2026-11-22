from voice_status.logic import FAILURE, RUNNING, SUCCESS, VoiceLogic, phrases, spoken_name, topic_base


class Bt:
    """Drives VoiceLogic like the real BT + executives: `_active` is republished every 0.1 s tick while an
    action is active, and each status message arrives on a tick."""

    def __init__(self, **kwargs):
        self.logic = VoiceLogic(**kwargs)
        self.now = 0.0
        self.active = set()
        self.said = []

    def _tick(self):
        for label in sorted(self.active):
            self.said += self.logic.on_active(label, True, self.now)
        self.said += self.logic.tick(self.now)

    def wait(self, seconds):
        end = self.now + seconds
        while self.now + 0.1 <= end + 1e-9:
            self.now += 0.1
            self._tick()

    def start(self, label):
        self.active.add(label)
        self._tick()

    def stop(self, label):
        self.active.discard(label)
        self.said += self.logic.on_active(label, False, self.now)

    def status(self, label, value):
        self.said += self.logic.on_status(label, value, self.now)

    def take(self):
        said, self.said = self.said, []
        return said


def test_go_to_location_says_moving_then_arrived():
    bt = Bt()
    bt.logic.on_location_command('OT-2')
    bt.start('Go To Location')
    bt.status('Go To Location', RUNNING)
    assert bt.take() == ['Moving to the Opentrons.']
    bt.wait(5)
    bt.status('Go To Location', RUNNING)
    assert bt.take() == []
    bt.status('Go To Location', SUCCESS)
    assert bt.take() == ['Arrived at the Opentrons.']
    bt.wait(0.1)
    bt.status('Go To Location', SUCCESS)
    bt.stop('Go To Location')
    assert bt.take() == []


def test_home_has_no_preposition():
    bt = Bt()
    bt.logic.on_location_command('home')
    bt.start('Go To Location')
    bt.status('Go To Location', RUNNING)
    bt.wait(2)
    bt.status('Go To Location', SUCCESS)
    assert bt.take() == ['Moving home.', 'Arrived home.']


def test_failure_phrase():
    bt = Bt()
    bt.logic.on_location_command('shaker')
    bt.start('Go To Location')
    bt.status('Go To Location', RUNNING)
    bt.wait(1)
    bt.status('Go To Location', FAILURE)
    assert bt.take() == ['Moving to the shaker.', 'Could not reach the shaker.']


def test_idle_default_failure_is_silent():
    # Idle executives publish FAILURE at 20 Hz; nothing is said unless the BT has the action active.
    logic = VoiceLogic()
    assert logic.on_active('Pick Object', False, 0.0) == []
    assert logic.on_status('Pick Object', FAILURE, 0.0) == []
    assert logic.tick(2.0) == []


def test_previous_run_status_is_ignored_within_grace():
    bt = Bt(grace_s=0.5)
    bt.start('Place Object')
    bt.status('Place Object', SUCCESS)       # left over from the last run
    bt.wait(0.1)
    bt.status('Place Object', RUNNING)
    assert bt.take() == ['Placing the well plate on the machine.']
    bt.wait(3)
    bt.status('Place Object', SUCCESS)
    assert bt.take() == ['Placed the well plate on the machine.']


def test_terminal_without_running_counts_after_grace():
    bt = Bt(grace_s=0.5)
    bt.start('Execute Shaker Protocol')
    bt.status('Execute Shaker Protocol', FAILURE)
    bt.wait(0.3)
    assert bt.take() == ['Starting the shaker.']
    bt.wait(0.3)
    assert bt.take() == ['The shaker failed.']


def test_manipulation_context_is_bound_at_activation():
    bt = Bt()
    bt.logic.on_manipulation_command('shaker', 'well_plate')
    bt.start('Place Object')
    bt.status('Place Object', RUNNING)
    bt.logic.on_manipulation_command('ot2', 'well_plate')   # ignored by the busy executive
    bt.wait(4)
    bt.status('Place Object', SUCCESS)
    bt.stop('Place Object')
    assert bt.take() == ['Placing the well plate on the shaker.', 'Placed the well plate on the shaker.']
    bt.start('Pick Object')
    assert bt.take() == ['Picking up the well plate from the Opentrons.']


def test_retry_in_same_activation_announces_again():
    bt = Bt()
    bt.start('Pick Base Object')
    bt.status('Pick Base Object', RUNNING)
    bt.wait(2)
    bt.status('Pick Base Object', FAILURE)
    bt.wait(0.1)
    bt.status('Pick Base Object', RUNNING)
    bt.wait(2)
    bt.status('Pick Base Object', SUCCESS)
    assert bt.take() == ['Picking up the well plate from the robot.',
                         'Could not pick up the well plate from the robot.',
                         'Picking up the well plate from the robot.',
                         'Picked up the well plate from the robot.']


def test_new_activation_after_stale_bt():
    logic = VoiceLogic(stale_s=1.0)
    assert logic.on_active('Execute OT2 Protocol', True, 0.0) == ['Starting the Opentrons protocol.']
    assert logic.on_active('Execute OT2 Protocol', True, 0.5) == []
    assert logic.on_active('Execute OT2 Protocol', True, 2.0) == ['Starting the Opentrons protocol.']


def test_deactivation_reports_terminal_seen_last():
    logic = VoiceLogic()
    logic.on_active('Navigate To Pose', True, 0.0)
    logic.on_status('Navigate To Pose', RUNNING, 0.05)
    logic.on_active('Navigate To Pose', True, 0.1)
    logic._actions['Navigate To Pose'].status = SUCCESS   # recorded, not yet announced
    assert logic.on_active('Navigate To Pose', False, 0.2) == ['Arrived at the goal pose.']


def test_unknown_names_and_labels():
    assert spoken_name('Bio_Shaker-2', 'x') == 'the bio shaker 2'
    assert spoken_name('', 'the goal') == 'the goal'
    assert VoiceLogic().on_active('Some Other Action', True, 0.0) == []
    assert topic_base('Execute OT2 Protocol') == 'execute_ot2_protocol'
    assert phrases('Execute Shaker Protocol', {}) == ('Starting the shaker.', 'The shaker is done.',
                                                       'The shaker failed.')

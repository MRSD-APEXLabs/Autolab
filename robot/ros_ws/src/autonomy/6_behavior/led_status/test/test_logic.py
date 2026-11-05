from led_status.logic import FAILURE, RUNNING, SUCCESS, LedLogic

GROUPS = [
    ('M', ['Go To Location', 'Navigate To Pose']),
    ('A', ['Pick Object', 'Place Object']),
    ('W', ['Execute OT2 Protocol']),
]


def make():
    return LedLogic(GROUPS, success_hold_s=5.0, stale_s=1.0, grace_s=0.5)


def run(logic, label, now):
    """Activate an action and report RUNNING, as an executive would."""
    logic.on_active(label, True, now)
    logic.on_status(label, RUNNING, now)


def test_idle_when_nothing_active():
    assert make().tick(0.0) == 'I'


def test_running_nav_action_shows_moving():
    logic = make()
    run(logic, 'Go To Location', 0.0)
    assert logic.tick(0.1) == 'M'


def test_running_arm_action_shows_arm():
    logic = make()
    run(logic, 'Place Object', 0.0)
    assert logic.tick(0.1) == 'A'


def test_running_lab_machine_action_shows_waiting():
    logic = make()
    run(logic, 'Execute OT2 Protocol', 0.0)
    assert logic.tick(0.1) == 'W'


def test_status_of_inactive_action_is_ignored():
    # Idle executives publish FAILURE by default at 20 Hz.
    logic = make()
    logic.on_active('Pick Object', False, 0.0)
    logic.on_status('Pick Object', FAILURE, 0.0)
    assert logic.tick(1.0) == 'I'


def test_nav_has_priority_over_arm():
    logic = make()
    run(logic, 'Pick Object', 0.0)
    run(logic, 'Go To Location', 0.0)
    assert logic.tick(0.1) == 'M'


def test_failure_after_running_shows_fault():
    logic = make()
    run(logic, 'Pick Object', 0.0)
    logic.tick(0.1)
    logic.on_status('Pick Object', FAILURE, 0.2)
    assert logic.tick(0.2) == 'E'


def test_fault_is_held_after_action_goes_inactive():
    logic = make()
    run(logic, 'Pick Object', 0.0)
    logic.on_status('Pick Object', FAILURE, 0.2)
    logic.tick(0.2)
    logic.on_active('Pick Object', False, 0.3)
    assert logic.tick(60.0) == 'E'


def test_fault_cleared_when_next_action_starts():
    logic = make()
    run(logic, 'Pick Object', 0.0)
    logic.on_status('Pick Object', FAILURE, 0.2)
    logic.tick(0.2)
    logic.on_active('Pick Object', False, 0.3)
    logic.tick(0.3)
    run(logic, 'Go To Location', 10.0)
    assert logic.tick(10.1) == 'M'


def test_fault_cleared_when_failed_action_runs_again():
    logic = make()
    run(logic, 'Pick Object', 0.0)
    logic.on_status('Pick Object', FAILURE, 0.2)
    logic.tick(0.2)
    logic.on_status('Pick Object', RUNNING, 1.0)
    assert logic.tick(1.0) == 'A'


def test_stale_failure_at_activation_is_not_a_fault():
    # Executive still reports last/default FAILURE for a tick before set_running().
    logic = make()
    logic.on_active('Pick Object', True, 0.0)
    logic.on_status('Pick Object', FAILURE, 0.0)
    assert logic.tick(0.05) == 'A'


def test_failure_without_running_counts_after_grace():
    logic = make()
    logic.on_active('Pick Object', True, 0.0)
    logic.on_status('Pick Object', FAILURE, 0.0)
    logic.tick(0.1)
    logic.on_status('Pick Object', FAILURE, 0.6)
    assert logic.tick(0.6) == 'E'


def test_success_shows_ready_then_idle():
    logic = make()
    run(logic, 'Go To Location', 1.5)
    logic.on_active('Go To Location', True, 2.0)  # BT re-publishes active every tick
    logic.on_status('Go To Location', SUCCESS, 2.0)
    assert logic.tick(2.0) == 'R'
    logic.on_active('Go To Location', False, 2.1)
    assert logic.tick(6.9) == 'R'
    assert logic.tick(7.1) == 'I'


def test_stale_success_at_activation_is_not_success():
    logic = make()
    logic.on_active('Go To Location', True, 0.0)
    logic.on_status('Go To Location', SUCCESS, 0.0)
    assert logic.tick(0.05) == 'M'


def test_action_inactive_when_active_msgs_stop():
    # BT engine died: no more _active messages.
    logic = make()
    run(logic, 'Go To Location', 0.0)
    assert logic.tick(0.5) == 'M'
    assert logic.tick(1.5) == 'I'


def test_unknown_labels_are_ignored():
    logic = make()
    logic.on_active('Arm', True, 0.0)
    logic.on_status('Arm', RUNNING, 0.0)
    assert logic.tick(0.1) == 'I'


def test_topic_base_matches_bt_action_naming():
    from led_status.logic import topic_base
    assert topic_base('Execute OT2 Protocol') == 'execute_ot2_protocol'

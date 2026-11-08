"""Full station outcomes must own the public API across child Nav2 actions."""
import json
from types import SimpleNamespace as NS
from action_msgs.msg import GoalStatus, GoalStatusArray, GoalStatus as Status
from std_msgs.msg import String
from swerve_navigation.nav_api import NavApi, NAVIGATING, SUCCEEDED, FAILED, SENDING, IDLE, match_place


def api():
    node = NavApi.__new__(NavApi)
    node._station_mode = True
    node._station_active = node._station_awaiting = False
    node._station_child_ids = set()
    node._state = IDLE
    node._sent_at = node._pending = node._idle_since = node._goal = None
    node.states, node.results = [], []
    node._publish_state = lambda s: (setattr(node, '_state', s), node.states.append(s))
    node._publish_result = node.results.append
    node._time_pub = node._distance_pub = NS(publish=lambda _: None)
    return node


def report(node, state, success=False, message='moving'):
    node._on_station_result(String(data=json.dumps(dict(
        state=state, done=state in ('arrived', 'stopped', 'undocked'), success=success, message=message))))


def test_staging_child_success_cannot_finish_station_mission():
    node = api()
    report(node, 'preparing')
    child = Status(status=GoalStatus.STATUS_SUCCEEDED)
    child.goal_info.goal_id.uuid = [1]*16
    node._on_status(GoalStatusArray(status_list=[child]))
    assert node._state == NAVIGATING and not node.results
    report(node, 'dock')
    report(node, 'arrived', success=True)
    assert node._state == SUCCEEDED and node.results == [SUCCEEDED]
    node._on_status(GoalStatusArray(status_list=[child]))
    assert node.results == [SUCCEEDED]


def test_terminal_replay_is_ignored_until_a_live_station_phase():
    node = api()
    node._state, node._station_awaiting = SENDING, True
    report(node, 'arrived', success=True)
    assert not node.results
    report(node, 'preparing')
    report(node, 'stopped', message='final station heading outside tolerance')
    assert node._state == FAILED and node.results == [FAILED]


def test_refused_request_does_not_wait_for_child_nav2_timeout():
    node = api()
    node._state, node._station_awaiting = SENDING, True
    report(node, 'idle', message='request refused: need fresh TF')
    assert node._state == FAILED and not node._station_awaiting


def test_legacy_ot2_name_selects_canonical_station_without_ambiguous_aliases():
    assert match_place({'OT-2': (0., 0., 0.)}, 'ot2') == 'OT-2'
    assert match_place({'OT-2': (0., 0., 0.), 'OT_2': (1., 0., 0.)}, 'ot2') is None


def test_failed_action_is_failed_even_if_diagnostic_mentions_cancellation():
    node = api()
    report(node, 'undock')
    report(node, 'stopped', message='stopped: undock failed/cancelled (status 6, error 0)')
    assert node._state == FAILED


def test_explicit_undock_publishes_whole_mission_success():
    node = api()
    report(node, 'undock')
    report(node, 'undocked', success=True)
    assert node._state == SUCCEEDED

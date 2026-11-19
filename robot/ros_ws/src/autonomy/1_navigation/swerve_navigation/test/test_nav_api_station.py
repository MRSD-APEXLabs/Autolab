"""Full station outcomes must own the public API across child Nav2 actions."""
import json
import pytest
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


def test_recoveries_keep_public_request_active_until_verified_arrival():
    node = api()
    report(node, 'preparing')
    for state in ('staging', 'stage_adjust', 'dock', 'settling', 'undock',
                  'staging', 'align', 'stage_adjust', 'dock'):
        report(node, state, message='recovering 1/2' if state == 'settling' else 'moving')
        assert node._state == NAVIGATING and not node.results
    report(node, 'arrived', success=True)
    assert node.results == [SUCCEEDED] and FAILED not in node.states


def test_pose_is_saved_at_most_once_a_second_and_only_when_it_changed(tmp_path, monkeypatch):
    import math
    import yaml
    from geometry_msgs.msg import Quaternion, Transform, Vector3
    from swerve_navigation import nav_api
    clock = NS(now=100.0)
    monkeypatch.setattr(nav_api.time, 'monotonic', lambda: clock.now)
    node = NavApi.__new__(NavApi)
    node._pose_file = str(tmp_path / 'nav_state/last_pose.yaml')
    node._map_yaml = str(tmp_path / 'best.yaml')
    node._saved_pose, node._saved_at = None, -math.inf

    def remember(x, y, yaw, at):
        clock.now = at
        node._remember_pose(Transform(translation=Vector3(x=x, y=y),
                                      rotation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))))
        return yaml.safe_load(open(node._pose_file))

    saved = remember(1.0, 2.0, -2.5, 100.0)
    assert saved['map'] == node._map_yaml and (saved['x'], saved['y']) == (1.0, 2.0)
    assert saved['yaw'] == pytest.approx(-2.5)
    assert remember(1.5, 2.0, -2.5, 100.5)['x'] == 1.0         # within a second of the last save
    assert remember(1.5, 2.0, -2.5, 101.1)['x'] == 1.5         # the change is written afterwards
    assert remember(1.505, 2.0, -2.5, 103.0)['x'] == 1.5       # < 1 cm and < 0.5 deg: not rewritten
    assert remember(1.5, 2.0, -2.5 + math.radians(1), 104.0)['yaw'] == pytest.approx(-2.5 + math.radians(1))
    node._pose_file = ''
    clock.now = 110.0
    node._remember_pose(None)                                 # off: nothing read or written


def test_unwritable_pose_file_only_warns(tmp_path):
    import math
    from geometry_msgs.msg import Transform
    blocker = tmp_path / 'file'
    blocker.write_text('')
    warnings = []
    node = NavApi.__new__(NavApi)
    node._pose_file, node._map_yaml = str(blocker / 'last_pose.yaml'), 'best.yaml'
    node._saved_pose, node._saved_at = None, -math.inf
    node.get_logger = lambda: NS(warning=lambda text, **kwargs: warnings.append(text))
    node._remember_pose(Transform())
    assert len(warnings) == 1 and node._saved_pose is None

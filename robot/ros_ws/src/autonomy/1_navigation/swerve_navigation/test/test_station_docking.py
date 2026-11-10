"""Station maneuver geometry and asynchronous action ordering; no hardware/ROS graph."""

from concurrent.futures import Future
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
import pytest
import yaml

from swerve_navigation import station_docking as docking
from swerve_navigation.location_markers import place_of


def free_grid():
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = grid.info.height = 100
    grid.info.origin.position.x = grid.info.origin.position.y = -5.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 10000
    return grid


def obstacle(grid, x, y, cost=100):
    col = math.floor((x + 5) / 0.1)
    row = math.floor((y + 5) / 0.1)
    grid.data[row * 100 + col] = cost


@pytest.mark.parametrize('heading', [0.0, math.pi / 2, -2.9042])
def test_departure_remembers_partial_withdrawal(heading):
    station = (1.0, 2.0, heading)
    robot = docking.staging_pose(station, 0.4)
    assert docking.departure_station({'s': station}, robot, docking.DockConfig()) == station
    along, lateral, yaw = docking.line_errors(station, robot)
    assert along == pytest.approx(-0.4)
    assert lateral == pytest.approx(0)
    assert yaw == pytest.approx(0)


def test_observed_offset_pose_recognizes_ot2_departure():
    station = (-.535, .46, -2.9042)
    robot = (-.114, .706, -2.899)
    config = docking.DockConfig()
    assert abs(docking.line_errors(station, robot)[1]) > config.lateral_tolerance
    assert docking.departure_station({'OT-2': station}, robot, config) == station


def test_full_footprint_and_turn_sweep_are_checked():
    grid = free_grid()
    assert docking.corridor_clear(grid, (0, 0, 0), (0.8, 0, 0), 0.35)
    obstacle(grid, 0.7, 0.3)  # misses the centre line, intersects the footprint
    assert not docking.corridor_clear(grid, (0, 0, 0), (0.8, 0, 0), 0.35)
    grid = free_grid()
    obstacle(grid, 0.45, 0.05)
    assert docking.footprint_clear(grid, (0, 0, 0), 0.35)
    assert not docking.footprint_clear(grid, (0, 0, 0), 0.35, turning=True)
    grid = free_grid()
    obstacle(grid, 0, 0, -1)
    assert not docking.footprint_clear(grid, (0, 0, 0), 0.35)
    assert not docking.footprint_clear(None, (0, 0, 0), 0.35)
    assert not docking.footprint_clear(free_grid(), (-4.9, 0, 0), 0.35)


class Client:
    def __init__(self):
        self.requests = []

    def server_is_ready(self):
        return True

    service_is_ready = server_is_ready

    def call_async(self, request):
        future = Future()
        future.set_result(NS(return_code=0))
        return future

    def send_goal_async(self, goal):
        future = Future()
        self.requests.append((goal, future))
        return future


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancelled = False

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancelled = True
        future = Future()
        future.set_result(NS(return_code=0))
        return future


@pytest.fixture
def mission(monkeypatch):
    clock = NS(now=100.0)
    monkeypatch.setattr(docking, 'time', NS(monotonic=lambda: clock.now))
    station_a, station_b = (0., 0., 0.), (0., 2., math.pi / 2)
    node = NS(robot=station_a, _places={'OT-2': station_a, 'shaker': station_b}, _frame='map')
    node._robot_place = lambda frame=None: node.robot
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
    m = docking.StationDocking.__new__(docking.StationDocking)
    # Preserve the original strict geometry as regression coverage, while
    # dedicated tests below exercise the shorter production approach/defaults.
    m.node, m.config, m.tree = node, docking.DockConfig(
        approach_distance=.8, position_tolerance=.08, yaw_tolerance=.08,
        approach_yaw_tolerance=.10), '/staging.xml'
    m.keep_transit_heading = False
    m.clients = {leg: Client() for leg in ('dock', 'undock', 'staging', 'align', 'stage_adjust')}
    m.cancel_clients = [Client(), Client(), Client()]
    m.seq, m.state, m.handle, m.estop = 0, 'idle', None, False
    m.pending = []
    m.parked_station = m.parked_odom = None
    m.goal = m.dock_goal_odom = m.odom_goal_yaw = None
    m.local_grid, m.global_grid = free_grid(), free_grid()
    m.local_time = m.global_time = m.odom_time = clock.now
    m.stopped, m.still_since, m.after_settle = True, None, None
    m.settle_wait_actions, m.close_since = False, None
    m.reports = []
    m._report = m.reports.append

    def tick(seconds=0):
        clock.now += seconds
        m.local_time = m.global_time = m.odom_time = clock.now
        m._tick()

    def settle():
        tick()
        tick(0.6)

    m.test_tick, m.test_settle = tick, settle
    return m


def finish(m, leg, pose, status=GoalStatus.STATUS_SUCCEEDED, error=0):
    _, pending = m.clients[leg].requests[-1]
    handle = Handle()
    pending.set_result(handle)
    m.node.robot = pose
    handle.result.set_result(NS(status=status, result=NS(error_code=error)))
    if status == GoalStatus.STATUS_SUCCEEDED:
        m.test_settle()
    return handle


def test_station_to_perpendicular_station_withdraws_aligns_then_drives_straight(mission):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'undock'
    goal = m.clients['undock'].requests[-1][0]
    assert goal.target.x == pytest.approx(-0.8)
    assert goal.speed == pytest.approx(0.1)
    finish(m, 'undock', (-0.8, 0., 0.))
    assert m.state == 'staging'
    goal = m.clients['staging'].requests[-1][0]
    assert goal.behavior_tree == '/staging.xml'
    assert goal.pose.pose.position.y == pytest.approx(1.2)
    finish(m, 'staging', (0., 1.2, math.pi / 2))
    assert m.state == 'dock'
    goal = m.clients['dock'].requests[-1][0]
    assert goal.controller_id == 'DockPath' and goal.goal_checker_id == 'dock_position_checker'
    assert goal.path.poses[-1].pose.position.y == pytest.approx(2.0)
    assert goal.path.header.stamp.sec == 0
    from rclpy.serialization import serialize_message
    assert serialize_message(goal)  # every path pose must have a real zero Time message
    finish(m, 'dock', m.node._places['shaker'])
    assert m.state == 'arrived'
    assert len(m.clients['staging'].requests) == 1  # parked pose NEVER sent to NavigateToPose


def test_swerve_transit_keeps_heading_then_aligns_once_for_the_station(mission):
    m = mission
    m.keep_transit_heading = True
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'undock', (-.8, 0., 0.))
    goal = m.clients['staging'].requests[-1][0]
    assert place_of(goal.pose)[2] == pytest.approx(0.)
    assert m.goal[2] == pytest.approx(math.pi/2)
    finish(m, 'staging', (0., 1.2, 0.))
    assert m.state == 'align' and len(m.clients['align'].requests) == 1
    finish(m, 'align', (0., 1.2, math.pi/2))
    finish(m, 'dock', m.goal)
    assert m.state == 'arrived' and len(m.clients['align'].requests) == 1


def test_reported_29mm_0088rad_stage_error_trims_during_approach_without_spin(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (.08, 1.2, math.pi/2+.088))
    assert m.state == 'stage_adjust'
    finish(m, 'stage_adjust', (.029, 1.2, math.pi/2+.088))
    assert m.state == 'dock' and not m.clients['align'].requests
    # Correcting the initial 0.088 rad offset is intentional, not new yaw drift.
    m.node.robot = (0., 1.6, math.pi/2)
    m.test_tick()
    assert m.state == 'dock'
    finish(m, 'dock', m.goal)
    assert m.state == 'arrived'


def test_heading_outside_configured_final_tolerance_recovers_without_table_side_spin(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (0., 1.2, math.pi/2+.088))
    assert m.state == 'dock' and not m.clients['align'].requests
    finish(m, 'dock', (0., 2., math.pi/2+.088))
    assert m.state == 'settling'
    assert m.recovery_attempts == 1
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'undock'
    assert not m.clients['align'].requests


@pytest.mark.parametrize('yaw', [.0801, .12, .20])
def test_moving_withdrawal_allows_heading_feedback_to_correct_drift(mission, yaw):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    handle = Handle()
    m.clients['undock'].requests[-1][1].set_result(handle)
    m.node.robot = (-.3, .06, yaw)
    m.test_tick()
    assert m.state == 'undock' and not handle.cancelled


def test_completed_withdrawal_does_not_reapply_final_parking_tolerances(mission):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'undock', (-.75, .09, .12))
    assert m.state == 'staging'


@pytest.mark.parametrize('destination', ['shaker', 'OT-2', None])
def test_offset_departure_backs_out_without_rotation_before_navigation(mission, destination):
    m = mission
    m.node.robot = (-.47, -.14, .005)
    m.start(m.node._places[destination] if destination else None)
    m.test_settle()
    assert m.state == 'undock'
    request = m.clients['undock'].requests[-1][0]
    assert request.target.x == pytest.approx(-.33 / math.cos(.005))
    exit_pose = m.endpoint
    # Reach the staging plane without a lateral correction or a chassis turn.
    assert exit_pose[0] == pytest.approx(-.8)
    assert exit_pose[1] == pytest.approx(-.14 - .33 * math.tan(.005))
    assert exit_pose[2] == .005
    assert not m.clients['align'].requests and not m.clients['dock'].requests
    finish(m, 'undock', exit_pose)
    assert m.state == ('staging' if destination else 'undocked')


def test_offset_withdrawal_checks_actual_footprint_corridor(mission):
    m = mission
    m.node.robot = (-.47, .14, 0.)
    obstacle(m.local_grid, -.7, .45)
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'stopped'
    assert not m.clients['undock'].requests and not m.clients['staging'].requests


@pytest.mark.parametrize('pose', [(-.3, 0., .25), (-.2, .36, 0.)])
@pytest.mark.parametrize('clear', [True, False])
def test_unaligned_near_station_uses_turning_clearance_instead_of_distance_guard(mission, pose, clear):
    m = mission
    m.node.robot = pose
    if not clear:
        obstacle(m.global_grid, pose[0]+.45, pose[1]+.05)
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == ('staging' if clear else 'stopped')
    if not clear:
        assert 'turning clearance' in m.reports[-1]
        assert all(not c.requests for c in m.clients.values())


def test_explicit_undock_stops_at_exit(mission):
    m = mission
    m.node.robot = (-0.3, 0., 0.)
    m.start(None)
    m.test_settle()
    assert m.clients['undock'].requests[-1][0].target.x == pytest.approx(-0.5)
    finish(m, 'undock', (-0.8, 0., 0.))
    assert m.state == 'undocked'
    assert not m.clients['staging'].requests and not m.clients['dock'].requests


def test_saved_heading_quaternion_roundtrip_still_uses_docking(mission):
    m = mission
    saved = m.node._places['shaker']
    m.start((saved[0], saved[1], saved[2] + 1e-12))
    assert m.is_station and m.goal == saved


def test_new_mission_waits_for_late_old_action_to_finish(mission):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    pending = m.clients['undock'].requests[-1][1]
    m.cancel('cancelled')
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'preparing'
    handle = Handle()
    pending.set_result(handle)
    m.test_settle()
    assert m.state == 'preparing' and handle.cancelled
    handle.result.set_result(NS(status=GoalStatus.STATUS_CANCELED, result=NS(error_code=0)))
    m.test_settle()
    assert m.state == 'undock'


def test_local_costmap_queries_transform_map_poses_to_odom(mission):
    m = mission
    m.local_grid.header.frame_id = 'odom'
    yaw = math.pi / 2
    transform = NS(translation=NS(x=2., y=-1.), rotation=NS(
        x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    m.node._tf_buffer = NS(lookup_transform=lambda *args: NS(transform=transform))
    assert m._in_grid(m.local_grid, (1., 0., 0.)) == pytest.approx((2., 0., yaw))
    obstacle(m.local_grid, 2., 0.)
    assert not m._clear_line((1., 0., 0.), (1.5, 0., 0.))


@pytest.mark.parametrize('reason', ['cancelled', 'e-stop', 'manual takeover'])
def test_late_goal_acceptance_is_cancelled_without_advancing(mission, reason):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    pending = m.clients['undock'].requests[-1][1]
    m.cancel(reason)
    handle = Handle()
    pending.set_result(handle)
    assert handle.cancelled
    assert m.state == 'stopped'
    assert not m.clients['staging'].requests


def test_staging_failure_replans_to_staging_without_parked_pose_fallback(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', m.node.robot, status=GoalStatus.STATUS_ABORTED, error=1)
    assert m.state == 'settling'
    assert len(m.clients['staging'].requests) == 1
    assert not m.clients['dock'].requests
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'staging'
    assert len(m.clients['staging'].requests) == 2
    assert place_of(m.clients['staging'].requests[-1][0].pose)[:2] == pytest.approx((0., 1.2))


@pytest.mark.parametrize('pose', [(0.1, 1.2, math.pi / 2), (0., 1.2, 1.8)])
def test_alignment_corrections_happen_at_staging_before_docking(mission, pose):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', pose)
    assert not m.clients['dock'].requests
    if abs(pose[2] - math.pi / 2) > 0.08:
        assert m.state == 'align'
        goal = m.clients['align'].requests[-1][0]
        assert goal.target_yaw == pytest.approx(math.pi / 2 - pose[2])
        finish(m, 'align', (0., 1.2, math.pi / 2))
    else:
        assert m.state == 'stage_adjust'
        finish(m, 'stage_adjust', (0., 1.2, math.pi / 2))
    assert m.state == 'dock'


def test_heading_drift_cancels_straight_leg(mission):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    handle = Handle()
    m.clients['undock'].requests[-1][1].set_result(handle)
    m.node.robot = (-0.1, 0., 0.3)
    m.test_tick()
    assert handle.cancelled and m.state == 'stopped'
    handle.result.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED, result=NS(error_code=0)))
    assert not m.clients['staging'].requests


def test_localization_update_is_not_mistaken_for_physical_drift(mission):
    m = mission
    m.start(m.node._places['shaker'])
    m.test_settle()
    odom = (-0.2, 0., 0.)
    m.node.robot = (-0.2, 0.06, 0.1)  # AMCL adjusts map->odom
    m.node._robot_place = lambda frame=None: odom if frame == 'odom' else m.node.robot
    m.test_tick()
    assert m.state == 'undock'


def test_localization_yaw_update_at_arrival_preserves_physical_departure(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (0., 1.2, math.pi / 2))
    goal = m.node._places['shaker']
    odom = NS(pose=goal)
    m.node._robot_place = lambda frame=None: odom.pose if frame == 'odom' else m.node.robot
    map_pose = (goal[0], goal[1], goal[2] + 0.12)
    finish(m, 'dock', map_pose)
    assert m.state == 'arrived'
    assert m.parked_station == goal
    m.start(m.node._places['OT-2'])
    m.test_settle()
    assert m.state == 'undock'  # no table-side heading correction after AMCL shifts
    odom.pose = docking.staging_pose(goal, 0.8)
    exit_pose = docking.staging_pose(map_pose, 0.8)
    finish(m, 'undock', exit_pose)
    assert m.state == 'staging'


def test_stopped_parking_inside_existing_tolerance_waits_for_controller_to_finish(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (0., 1.2, math.pi / 2))
    handle = Handle()
    m.clients['dock'].requests[-1][1].set_result(handle)
    m.node.robot = (0., 1.91, math.pi / 2)  # 9 cm is outside the acceptance limit
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'dock' and not handle.cancelled
    m.node.robot = (0., 1.925, math.pi / 2)  # stopped at 7.5 cm, inside 8 cm
    m.test_tick()
    m.test_tick(1.1)
    assert handle.cancelled and m.state == 'settling'
    m.test_settle()
    assert m.state == 'settling'  # a cancellation request alone is insufficient
    handle.result.set_result(NS(status=GoalStatus.STATUS_CANCELED, result=NS(error_code=0)))
    m.test_settle()
    assert m.state == 'arrived'


def test_obstacle_blocks_departure_and_stale_map_aborts_motion(mission):
    m = mission
    obstacle(m.local_grid, -0.6, 0.1)
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'stopped' and not m.clients['undock'].requests
    m.local_grid = free_grid()
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'undock'
    m.local_time -= 2.0
    m._tick()
    assert m.state == 'stopped'


def test_already_parked_does_not_turn_or_move(mission):
    m = mission
    m.start(m.node._places['OT-2'])
    m.test_settle()
    assert m.state == 'arrived'
    assert all(not c.requests for c in m.clients.values())


def test_staging_success_cannot_be_reported_as_parked(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    stage = (0., 1.2, math.pi/2)
    m.node.robot = stage
    # Even if a stale stage endpoint were accidentally treated as the dock leg,
    # _parked verifies the actual station independently and rejects it.
    m.endpoint, m.odom_goal_yaw = stage, stage[2]
    m._after_leg('dock')
    assert m.state == 'stopped' and m.parked_station is None
    assert '0.800 m remaining' in m.reports[-1]


def test_localization_jump_to_goal_does_not_fake_physical_arrival(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    stage = (0., 1.2, math.pi/2)
    finish(m, 'staging', stage)
    m.node.robot = m.goal
    m.node._robot_place = lambda frame=None: stage if frame == 'odom' else m.node.robot
    m._parked()
    assert m.state == 'stopped'
    assert 'map/odometry disagree' in m.reports[-1]


@pytest.mark.parametrize('state', ['staging', 'align', 'stage_adjust', 'dock', 'arrived'])
def test_rviz_never_marks_a_staging_pose_docked(state):
    from swerve_navigation.station_docking import docking_marker
    from rclpy.serialization import serialize_message
    marker = docking_marker(state, 'shaker', (0., 2., 0.), (0., 1.2, 0.), .08,
                            True, 'test', 'map')
    assert 'NOT DOCKED' in marker.text and '0.80 m' in marker.text
    assert marker.color.r > marker.color.g
    assert serialize_message(marker)


def test_rviz_only_shows_docked_for_a_verified_current_pose():
    from swerve_navigation.station_docking import docking_marker
    marker = docking_marker('arrived', 'shaker', (0., 2., 0.), (0., 1.98, 0.), .08,
                            True, 'parked', 'map')
    assert 'DOCKED (verified)' in marker.text and marker.color.g > marker.color.r
    marker = docking_marker('arrived', 'shaker', (0., 2., 0.), (0., 1.98, 0.), .08,
                            False, 'moved', 'map')
    assert 'NOT DOCKED' in marker.text


def test_full_mission_result_does_not_reuse_nav2_staging_success(mission):
    import json
    m = mission
    m.goal, m.is_station = m.node._places['shaker'], True
    m.node.robot = (0., 1.2, math.pi/2)
    m.state = 'staging'
    output = []
    m.result = NS(publish=output.append)
    m.status = m.status_marker = m.rotation_lock = NS(publish=lambda _: None)
    m.node.get_logger = lambda: NS(info=lambda _: None)
    docking.StationDocking._report(m, 'staging reached')
    result = json.loads(output[-1].data)
    assert not result['done'] and not result['docked']
    assert result['remaining_m'] == pytest.approx(.8)
    m.state = 'arrived'
    m.parked_odom = m.node.robot
    docking.StationDocking._report(m, 'incorrect upstream success')
    assert not json.loads(output[-1].data)['docked']
    m.node.robot = m.goal
    m.parked_odom = m.goal
    docking.StationDocking._report(m, 'parked')
    assert json.loads(output[-1].data)['docked'] is True


def test_wrong_heading_at_table_never_triggers_alignment_spin(mission):
    m = mission
    m.node.robot = (0., 0., 0.3)
    obstacle(m.global_grid, .45, .05)  # a real table blocks the turning sweep
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'stopped'
    assert not m.clients['align'].requests and not m.clients['staging'].requests


def test_alignment_retries_are_bounded_and_stay_in_open_staging(mission):
    m = mission
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    stage = (0., 1.2, 1.8)
    finish(m, 'staging', stage)
    for _ in range(3):
        assert m.state == 'align'
        finish(m, 'align', stage)  # deliberately fail to change the measured heading
    assert m.state == 'stopped'
    assert len(m.clients['align'].requests) == 3
    assert not m.clients['dock'].requests


def test_location_save_preserves_docking_settings(tmp_path):
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('locations_cli', root / 'scripts/locations.py')
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    path = tmp_path / 'best.locations.yaml'
    path.write_text('map: best\ndocking: {enabled: true, approach_distance: 0.8}\nlocations: {}\n')
    cli.store(str(path), 'best', {'home': {'x': 0., 'y': 0., 'yaw': 0.}})
    data = yaml.safe_load(path.read_text())
    assert data['docking'] == {'enabled': True, 'approach_distance': 0.8}
    assert 'home' in data['locations']


def test_best_map_enabled_and_staging_tree_has_no_terminal_station_goal_or_recovery():
    root = Path(__file__).resolve().parents[2]
    config = docking.load_docking_config(root / 'navigation_bringup/maps/best.locations.yaml')
    assert config.approach_distance == 0.3
    assert config.staging_distance == 0.8
    bringup = root / 'navigation_bringup'
    tree = ET.parse(bringup / 'behavior_trees/navigate_to_station_staging.xml')
    assert not any(tree.iter('Spin')) and not any(tree.iter('BackUp'))
    follow = next(tree.iter('FollowPath'))
    params = yaml.safe_load((bringup / 'config/nav2_params.yaml').read_text())
    checker = params['controller_server']['ros__parameters'][follow.attrib['goal_checker_id']]
    assert checker['xy_goal_tolerance'] <= 0.10
    assert checker['yaw_goal_tolerance'] == math.pi  # rotation occurs in an explicit checked phase
    server = params['controller_server']['ros__parameters']
    assert server['dock_alignment_checker']['xy_goal_tolerance'] < config.lateral_tolerance
    assert server['dock_position_checker']['xy_goal_tolerance'] < config.position_tolerance


def test_docking_controller_cannot_command_rotation():
    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root / 'navigation_bringup/config/nav2_params.yaml').read_text())
    server = params['controller_server']['ros__parameters']
    dock = server['DockPath']
    assert dock['plugin'] == 'nav2_mppi_controller::MPPIController'  # no rotation shim
    assert dock['wz_max'] == 0.0 and dock['wz_std'] > 0.0  # MPPI divides by noise variance
    assert dock['motion_model'] == 'Omni'
    assert dock['CostCritic']['consider_footprint'] is True
    smoother = params['velocity_smoother']['ros__parameters']
    assert dock['vy_max'] <= smoother['max_velocity'][1]
    assert -dock['vy_max'] >= smoother['min_velocity'][1]


@pytest.mark.parametrize('distance', [.091, .104])
def test_observed_stalls_are_accepted_only_after_stopped_verification(mission, distance):
    m = mission
    m.config = docking.DockConfig()
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (0., 1.2, math.pi/2))
    assert m.state == 'stage_adjust'
    finish(m, 'stage_adjust', (0., 1.7, math.pi/2))
    assert m.state == 'dock'
    handle = Handle()
    m.clients['dock'].requests[-1][1].set_result(handle)
    m.node.robot = (0., 2.-distance, math.pi/2+.11)
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'settling' and handle.cancelled
    m.test_settle()
    assert m.state == 'settling'  # the old controller is still active
    handle.result.set_result(NS(status=GoalStatus.STATUS_CANCELED, result=NS()))
    m.test_settle()
    assert m.state == 'arrived' and m.parked_station == m.goal


def start_default_docking(m):
    m.config = docking.DockConfig()
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    finish(m, 'staging', (0., 1.2, math.pi/2))
    finish(m, 'stage_adjust', (0., 1.7, math.pi/2))
    assert m.state == 'dock'


def test_short_dock_keeps_alignment_and_withdrawal_farther_from_table(mission):
    m = mission
    m.config = docking.DockConfig()
    m.keep_transit_heading = True
    m.node.robot = (-2., -1., 0.)
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert place_of(m.clients['staging'].requests[-1][0].pose)[:2] == pytest.approx((0., 1.2))
    finish(m, 'staging', (0., 1.2, 0.))
    assert m.state == 'align'
    finish(m, 'align', (0., 1.2, math.pi/2))
    assert m.state == 'stage_adjust'
    goal = m.clients['stage_adjust'].requests[-1][0]
    assert goal.path.poses[-1].pose.position.y == pytest.approx(1.7)
    finish(m, 'stage_adjust', (0., 1.7, math.pi/2))
    goal = m.clients['dock'].requests[-1][0]
    assert goal.path.poses[0].pose.position.y == pytest.approx(1.7)
    assert goal.path.poses[-1].pose.position.y == pytest.approx(2.)
    finish(m, 'dock', m.goal)
    m.start(None)
    m.test_settle()
    assert m.clients['undock'].requests[-1][0].target.x == pytest.approx(-.8)


def test_translation_stall_recovers_before_nav2_deadline_and_waits_for_cancel(mission):
    m = mission
    start_default_docking(m)
    handle = Handle()
    m.clients['dock'].requests[-1][1].set_result(handle)
    m.test_tick(8.1)
    assert m.state == 'settling' and handle.cancelled and m.recovery_attempts == 1
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'settling'
    handle.result.set_result(NS(status=GoalStatus.STATUS_CANCELED, result=NS()))
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'dock' and len(m.clients['dock'].requests) == 2
    finish(m, 'dock', m.goal)
    assert m.state == 'arrived'


def test_final_heading_failure_withdraws_realigns_and_completes_same_request(mission):
    m = mission
    start_default_docking(m)
    finish(m, 'dock', (0., 2., math.pi/2+.16))
    assert m.state == 'settling' and m.recovery_attempts == 1
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'undock'
    assert not m.clients['align'].requests
    finish(m, 'undock', m.endpoint)
    assert m.state == 'staging'
    finish(m, 'staging', (0., 1.2, math.pi/2+.16))
    assert m.state == 'align'
    finish(m, 'align', (0., 1.2, math.pi/2))
    finish(m, 'stage_adjust', (0., 1.7, math.pi/2))
    finish(m, 'dock', m.goal)
    assert m.state == 'arrived' and m.recovery_attempts == 1


def test_heading_recovery_cannot_back_into_an_obstacle(mission):
    m = mission
    start_default_docking(m)
    finish(m, 'dock', (0., 2., math.pi/2+.16))
    obstacle(m.local_grid, 0., 1.4)
    m.test_tick()
    m.test_tick(1.1)
    assert m.state == 'stopped'
    assert not m.clients['undock'].requests and not m.clients['align'].requests


@pytest.mark.parametrize('reason', ['cancelled', 'manual takeover', 'e-stop'])
def test_operator_stop_during_recovery_never_resumes(mission, reason):
    m = mission
    start_default_docking(m)
    m._recover('test abort', m._dock)
    m.cancel(reason)
    m.test_tick(2.)
    assert m.state == 'stopped' and len(m.clients['dock'].requests) == 1


def test_aborted_near_goal_must_verify_heading_instead_of_claiming_success(mission):
    m = mission
    start_default_docking(m)
    finish(m, 'dock', (0., 1.91, math.pi/2+.20), status=GoalStatus.STATUS_ABORTED)
    m.test_settle()
    assert m.state == 'settling' and m.recovery_attempts == 1
    assert m.parked_station is None


def test_externally_canceled_controller_is_never_retried(mission):
    m = mission
    start_default_docking(m)
    finish(m, 'dock', m.node.robot, status=GoalStatus.STATUS_CANCELED)
    assert m.state == 'stopped' and m.recovery_attempts == 0


def test_recovery_budget_is_shared_between_legs_and_does_not_reset_on_replan(mission):
    m = mission
    start_default_docking(m)
    for attempt in (1, 2):
        finish(m, 'dock', m.node.robot, status=GoalStatus.STATUS_ABORTED)
        assert m.state == 'settling' and m.recovery_attempts == attempt
        m.test_tick()
        m.test_tick(1.1)
        assert m.state == 'dock'
    finish(m, 'dock', m.node.robot, status=GoalStatus.STATUS_ABORTED)
    assert m.state == 'stopped' and m.recovery_attempts == 2
    assert 'exhausted' in m.reports[-1]


def test_recovery_never_reanchors_a_large_map_odom_disagreement(mission):
    m = mission
    start_default_docking(m)
    m.node.robot = (0., 1.87, math.pi/2)
    m.node._robot_place = lambda frame=None: (0., 1.7, math.pi/2) if frame == 'odom' else m.node.robot
    m._parked()
    assert m.state == 'stopped' and m.recovery_attempts == 0
    assert 'map/odometry disagree' in m.reports[-1]


def test_docking_config_rejects_turning_too_near_table_and_excessive_tolerance():
    for settings in ({'approach_distance': .2}, {'staging_distance': .4},
                     {'position_tolerance': .3}, {'yaw_tolerance': .2}):
        with pytest.raises(ValueError):
            docking.DockConfig(**settings)


def test_centimetre_corrections_count_as_progress_on_humble():
    root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((root / 'navigation_bringup/config/nav2_params.yaml').read_text())
    checker = params['controller_server']['ros__parameters']['progress_checker']
    assert checker['required_movement_radius'] == .03
    assert checker['movement_time_allowance'] > 8.


def test_stale_perception_during_recovery_stops_without_retry(mission):
    m = mission
    start_default_docking(m)
    m._recover('test abort', m._dock)
    m.local_time -= 2.
    m._tick()
    assert m.state == 'stopped' and len(m.clients['dock'].requests) == 1


def test_small_real_movements_reset_stall_timer(mission):
    m = mission
    start_default_docking(m)
    handle = Handle()
    m.clients['dock'].requests[-1][1].set_result(handle)
    m.node.robot = (0., 1.725, math.pi/2)
    m.test_tick(7.)
    m.test_tick(2.)
    assert m.state == 'dock' and not handle.cancelled and m.recovery_attempts == 0


def test_same_station_request_from_farther_out_still_uses_short_final_dock(mission):
    m = mission
    m.config = docking.DockConfig()
    m.node.robot = (0., 1.2, math.pi/2)
    m.start(m.node._places['shaker'])
    m.test_settle()
    assert m.state == 'stage_adjust'
    finish(m, 'stage_adjust', (0., 1.7, math.pi/2))
    assert m.state == 'dock'


def test_mission_deadline_is_not_extended_by_recovery(mission):
    m = mission
    start_default_docking(m)
    deadline = m.mission_deadline
    m._recover('test abort', m._travel)
    m.test_tick()
    m.test_tick(1.1)
    assert m.mission_deadline == deadline
    m.mission_deadline = 0.
    m.test_tick()
    assert m.state == 'stopped' and 'time limit' in m.reports[-1]

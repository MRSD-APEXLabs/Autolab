"""Cross-component navigation contracts; no running graph or hardware needed."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[2] / 'navigation_bringup'


def merged(base, overlay):
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merged(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


@pytest.fixture(params=[('forward', 'normal'), ('omni', 'normal'),
                        ('forward', 'fast'), ('omni', 'fast')])
def params(request):
    config = yaml.safe_load((BRINGUP / 'config/nav2_params.yaml').read_text())
    motion, speed = request.param
    if motion == 'omni':
        config = merged(config, yaml.safe_load(
            (BRINGUP / 'config/nav2_omni_overlay.yaml').read_text()))
    if speed == 'fast':
        config = merged(config, yaml.safe_load((BRINGUP / 'config/nav2_fast_overlay.yaml').read_text()))
        if motion == 'omni':
            config = merged(config, yaml.safe_load(
                (BRINGUP / 'config/nav2_fast_omni_overlay.yaml').read_text()))
    return config


def test_predicted_velocities_are_executable_by_smoother(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    assert controller['vx_max'] <= smoother['max_velocity'][0]
    assert controller['vx_min'] >= smoother['min_velocity'][0]
    assert controller['wz_max'] <= min(smoother['max_velocity'][2],
                                        -smoother['min_velocity'][2])
    if controller['motion_model'] == 'Omni':
        assert controller['vy_max'] <= min(smoother['max_velocity'][1],
                                            -smoother['min_velocity'][1])


def test_controller_can_leave_the_drive_dead_zone_from_rest(params):
    # MPPI 1.3.13 clamps every sampled control to MEASURED speed +/- a * model_dt. With the
    # smoother's 1.0 m/s^2 the robot at rest was never sent more than ~0.1 m/s, which the real
    # drive does not execute: it never started (hardware logs 2026-09-21). One step from rest must
    # clear the speed floor of cmd_vel_mux (nav_min_speed); velocity_smoother sets the real ramp.
    controller = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    axes = [(0, 'x')]
    if controller['motion_model'] == 'Omni':
        axes.append((1, 'y'))
    for index, axis in axes:
        assert controller[f'a{axis}_max'] * controller['model_dt'] >= 0.12
        assert controller[f'a{axis}_min'] * controller['model_dt'] <= -0.12
        assert controller[f'a{axis}_max'] >= smoother['max_accel'][index]
    assert controller['az_max'] >= smoother['max_accel'][2]


def test_controller_timing_and_final_approach_contract(params):
    server = params['controller_server']['ros__parameters']
    controller = server['FollowPath']
    assert controller['model_dt'] == pytest.approx(1.0 / server['controller_frequency'])
    yaw_distance = controller['GoalAngleCritic']['threshold_to_consider']
    assert yaw_distance >= server['general_goal_checker']['xy_goal_tolerance']
    # Final-yaw scoring must not compete with heading preferences: both critics that want the robot
    # to face its direction of travel are switched off wherever the goal heading counts.
    # (PathAlignCritic scores positions only - use_path_orientations false - so it may overlap.)
    assert yaw_distance <= controller['PathAngleCritic']['threshold_to_consider']
    assert yaw_distance <= controller['PreferForwardCritic']['threshold_to_consider']
    assert not controller['PathAlignCritic']['use_path_orientations']
    if controller['motion_model'] == 'Omni':
        # Turn to the goal heading during the approach, not on the spot next to the goal.
        assert yaw_distance >= 1.0
    handoff = controller['GoalCritic']['threshold_to_consider']
    assert handoff == controller['PathFollowCritic']['threshold_to_consider']
    assert handoff <= controller['vx_max'] * controller['time_steps'] * controller['model_dt']


def test_heading_owner_and_rotation_limits(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    if controller['motion_model'] == 'DiffDrive':
        assert controller['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
        assert controller['primary_controller'] == 'nav2_mppi_controller::MPPIController'
        assert controller['rotate_to_goal_heading']
        assert not controller['GoalAngleCritic']['enabled']
        assert 0 < controller['angular_disengage_threshold'] < controller['angular_dist_threshold']
        assert controller['rotate_to_heading_angular_vel'] <= smoother['max_velocity'][2]
        assert controller['max_angular_accel'] <= smoother['max_accel'][2]
    else:
        assert controller['plugin'] == 'nav2_mppi_controller::MPPIController'
        assert controller['GoalAngleCritic']['enabled']


def test_transit_horizon_and_speeds_fit_local_window_and_bridge(params):
    controller = params['controller_server']['ros__parameters']['FollowPath']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    bridge = yaml.safe_load((BRINGUP / 'config/swerve_bridge.yaml').read_text())['swerve_bridge']['ros__parameters']
    import math
    linear = math.hypot(controller['vx_max'], controller['vy_max'])
    assert linear <= bridge['max_linear_speed']
    assert controller['wz_max'] <= bridge['max_angular_speed']
    assert params['behavior_server']['ros__parameters']['max_rotational_vel'] <= bridge['max_angular_speed']
    assert linear * controller['model_dt'] * controller['time_steps'] + .5 < min(local['width'], local['height']) / 2


def test_fast_mode_preserves_docking_steering_and_ramps(params):
    base = yaml.safe_load((BRINGUP / 'config/nav2_params.yaml').read_text())
    server = params['controller_server']['ros__parameters']
    assert server['DockPath'] == base['controller_server']['ros__parameters']['DockPath']
    assert params['velocity_smoother']['ros__parameters']['max_accel'][0] == .6
    assert params['velocity_smoother']['ros__parameters']['max_accel'][2] == 1.
    assert params['collision_monitor']['ros__parameters']['PolygonStop'] == base['collision_monitor']['ros__parameters']['PolygonStop']
    slow = params['collision_monitor']['ros__parameters']['PolygonSlow']
    if slow['type'] == 'velocity_polygon':
        assert slow['holonomic']
        assert slow['normal']['points'] == base['collision_monitor']['ros__parameters']['PolygonSlow']['points']
        assert slow['fast_transit']['linear_min'] > server['DockPath']['vx_max']


@pytest.mark.parametrize('motion,selection,expected', [
    ('forward', 'auto', 'travel'), ('omni', 'auto', 'keep'),
    ('forward', 'keep', 'keep'), ('omni', 'travel', 'travel'),
])
def test_launch_resolves_click_heading_without_overriding_explicit_choice(motion, selection, expected):
    spec = importlib.util.spec_from_file_location('navigation_launch',
                                                   BRINGUP / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.resolve_goal_heading(motion, selection) == expected


def test_best_profile_selects_swerve_without_changing_unprofiled_maps(tmp_path):
    spec = importlib.util.spec_from_file_location('navigation_launch', BRINGUP / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.resolve_nav_motion('auto', BRINGUP / 'maps/best.yaml') == 'omni'
    assert module.resolve_nav_motion('forward', BRINGUP / 'maps/best.yaml') == 'forward'
    assert module.resolve_nav_motion('auto', tmp_path / 'other.yaml') == 'forward'


def test_omni_position_corrections_are_not_penalized_for_strafing():
    config = merged(yaml.safe_load((BRINGUP / 'config/nav2_params.yaml').read_text()),
                    yaml.safe_load((BRINGUP / 'config/nav2_omni_overlay.yaml').read_text()))
    controller = config['controller_server']['ros__parameters']['FollowPath']
    assert controller['motion_model'] == 'Omni'
    assert controller['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert controller['PathAngleCritic']['enabled'] is False
    assert controller['PreferForwardCritic']['enabled'] is False
    assert controller['vy_max'] > 0.

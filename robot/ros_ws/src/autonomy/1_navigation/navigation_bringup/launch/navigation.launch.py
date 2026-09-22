"""
Navigation: everything the MK5 swerve base needs to localise in a saved map and drive to goals.

One launch file, one mode (there is no mapping and no simulated world here - those live in the
full workspace, nav_ws).  It starts:

  lidar        velodyne_driver + velodyne_transform -> /velodyne_points -> cloud_to_scan
               -> /scan (2-D, for AMCL and the costmaps) + /obstacle_points (3-D obstacle band)
  base         swerve_bridge: /cmd_vel -> phoenix6 swerve modules, drivetrain odometry -> /odom
               + TF odom -> base_footprint.  hardware:=false (default) drives the phoenix6
               SIMULATOR instead: the whole stack runs, the robot cannot move.
  operator     joy_node + joy_teleop (LT = drive, B = e-stop, X = cancel) and cmd_vel_mux, which
               gives the pad priority over Nav2 at all times
  localisation map_server + AMCL
  navigation   Nav2 planner / MPPI controller / behaviors / smoother / velocity smoother /
               collision monitor / BT navigator / waypoint follower
  goals        point_to_goal (RViz "Publish Point"), location_markers (saved places, staged
               approach) and nav_api (the /nav topic interface for the rest of the system)

Velocity chain -- NOTHING in Nav2 publishes /cmd_vel:
  controller_server, behavior_server -> /cmd_vel_nav_raw -> velocity_smoother
  -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_nav -> cmd_vel_mux -> /cmd_vel
No docking_server / route_server: the stock docking_server publishes straight to /cmd_vel.

    ros2 launch navigation_bringup navigation.launch.py            dry run (phoenix6 simulator)
    ros2 launch navigation_bringup navigation.launch.py hardware:=true    REAL ROBOT - it moves

navigation.launch.xml wraps this file for Autolab's autonomy bringup (NAVIGATION_LAUNCH_* in
.env). It runs on ROS 2 Humble (the Autolab robot container) and on Jazzy (the host); on Humble
it adds config/nav2_humble_overlay.yaml and the BehaviorTree.CPP v3 copy of the tree. Besides
Nav2 it needs the velodyne driver and the phoenix6 PYTHON package (the container image ships only
the C++ one): ../scripts/install_deps.sh installs both. Where they are missing this launch says so
and starts nothing, instead of failing the whole autonomy bringup. See ../README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'navigation_bringup'

# Default map and saved places: maps/ of this package. Override with map:=/path/to/other.yaml.
DEFAULT_MAP_NAME = 'map.yaml'

RUNTIME_PACKAGES = ('swerve_navigation', 'velodyne_driver', 'velodyne_pointcloud', 'joy',
                    'robot_state_publisher', 'topic_tools')

# swerve_bridge imports the phoenix6 PYTHON package from the first of these that has it (the
# environment variable PHOENIX6_SITE_PACKAGES, read by the node itself, wins over all of them):
# the robot container (scripts/install_deps.sh), then the host venv the stack was developed in.
PHOENIX6_SITE_PACKAGES = (
    '/opt/phoenix6_python',
    '/home/labx/nav_nav/Navigation/generated/venv/lib/python3.12/site-packages',
)

# Humble's Nav2 spells a few parameters differently and its BehaviorTree.CPP is v3.
HUMBLE = os.environ.get('ROS_DISTRO') == 'humble'

NAV2_PACKAGES = (
    'nav2_map_server', 'nav2_amcl', 'nav2_lifecycle_manager', 'nav2_controller',
    'nav2_mppi_controller', 'nav2_rotation_shim_controller', 'nav2_planner', 'nav2_smac_planner',
    'nav2_smoother', 'nav2_behaviors', 'nav2_bt_navigator', 'nav2_waypoint_follower',
    'nav2_velocity_smoother', 'nav2_collision_monitor', 'nav2_costmap_2d')

# Brought up in this order, shut down in reverse. bt_navigator loads its tree on activation and
# needs the action servers of the nodes listed before it.
LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                    'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']

# Every node runs on wall time.
WALL_TIME = {'use_sim_time': False}


def _share(*parts):
    return PathJoinSubstitution([FindPackageShare(PKG), *parts])


def parse_bool(name, text):
    """Parse a boolean launch argument strictly (same spellings as launch's IfCondition)."""
    value = text.strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(f'launch argument {name}:={text!r} is not a boolean (use true or false)')


def parse_float(name, text):
    try:
        return float(text)
    except ValueError:
        raise RuntimeError(f'launch argument {name}:={text!r} is not a number') from None


def missing_packages(names):
    missing = []
    for name in names:
        try:
            get_package_share_directory(name)
        except PackageNotFoundError:
            missing.append(name)
    return missing


def find_phoenix6():
    """Directory holding the phoenix6 Python package, or None."""
    candidates = [os.environ.get('PHOENIX6_SITE_PACKAGES', '')] + list(PHOENIX6_SITE_PACKAGES)
    for directory in candidates:
        if directory and os.path.isfile(os.path.join(directory, 'phoenix6', '__init__.py')):
            return directory
    return None


def resolve_goal_heading(nav_motion, goal_heading):
    """A point click in forward mode should not request a return to the starting yaw."""
    if goal_heading == 'auto':
        return 'travel' if nav_motion == 'forward' else 'keep'
    return goal_heading


def _lifecycle_manager(name, node_names):
    return Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name,
        output='both',
        parameters=[WALL_TIME,
                    {'autostart': True, 'node_names': node_names, 'bond_timeout': 4.0}])


def _nodes(context):
    """
    Validate the setup, then build every node.

    Everything that can be refused (Nav2 missing, no map, a malformed argument) is refused here,
    before a single process is started.
    """
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory(PKG)

    # Missing dependencies (a container built before they were added to the image): say so and
    # start nothing - raising here would take the whole Autolab autonomy bringup down with us.
    missing = missing_packages(NAV2_PACKAGES + RUNTIME_PACKAGES)
    phoenix6_dir = find_phoenix6()
    if phoenix6_dir is None:
        missing.append('phoenix6 (Python)')
    if missing:
        return [LogInfo(msg='[navigation] NOT STARTED, missing here: '
                            f"{', '.join(missing)}. Install them with "
                            'autonomy/1_navigation/scripts/install_deps.sh. Nothing was started.')]

    hardware = parse_bool('hardware', arg('hardware'))
    map_yaml = os.path.abspath(os.path.expanduser(
        arg('map') or os.path.join(share, 'maps', DEFAULT_MAP_NAME)))
    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'map file not found: {map_yaml}. Pass map:=/path/to/map.yaml, or build one with '
            'the mapping mode in nav_ws and save it next to this one   -- nothing was started.')

    params = [os.path.join(share, 'config', 'nav2_params.yaml')]
    # overlays come after the base file, so their values win
    if HUMBLE:
        params.append(os.path.join(share, 'config', 'nav2_humble_overlay.yaml'))
    if arg('nav_motion') == 'omni':
        params.append(os.path.join(share, 'config', 'nav2_omni_overlay.yaml'))
    params.append(WALL_TIME)

    x, y, yaw = (parse_float(name, arg(name)) for name in ('x', 'y', 'yaw'))
    goal_heading = resolve_goal_heading(arg('nav_motion'), arg('goal_heading'))
    approach = parse_float('approach_distance', arg('approach_distance'))
    bt_xml = os.path.join(share, 'behavior_trees', 'navigate_to_pose_cautious_humble.xml'
                          if HUMBLE else 'navigate_to_pose_cautious.xml')
    locations_file = os.path.splitext(map_yaml)[0] + '.locations.yaml'

    def nav2_node(package, executable, *, name=None, overrides=None, remappings=None):
        # No respawn: a crashed Nav2 server takes its lifecycle manager down (bond) and the
        # robot stops, which is what is wanted on a real robot.
        return Node(package=package, executable=executable, name=name, output='both',
                    parameters=params + ([overrides] if overrides else []),
                    remappings=remappings)

    # Every node that commands velocity is remapped away from /cmd_vel (see module docstring).
    to_nav_raw = [('cmd_vel', 'cmd_vel_nav_raw')]
    # Humble's controller_server TF buffer repeatedly stopped accepting map->odom from AMCL while
    # continuing to receive the other /tf writers. Feed that process a single DDS writer instead;
    # the serialized TFMessage is unchanged and every other consumer stays on the original /tf.
    controller_remaps = to_nav_raw + [('/tf', '/tf_controller')]

    actions = [
        LogInfo(msg=f"[navigation] map={map_yaml} nav_motion={arg('nav_motion')} "
                    f'goal_heading={goal_heading} approach={approach} '
                    f'AMCL initial pose x={x} y={y} yaw={yaw}'),
        LogInfo(msg='[navigation] *** HARDWARE MODE: swerve_bridge drives the REAL robot. Keep the '
                    'gamepad in hand: B = e-stop, releasing LT stops teleop. ***'
                    if hardware else
                    '[navigation] dry run: swerve_bridge is on the phoenix6 SIMULATOR, the robot '
                    'cannot move (hardware:=true drives it for real).'),

        # ---- lidar ----
        Node(package='velodyne_driver', executable='velodyne_driver_node',
             name='velodyne_driver_node', output='both',
             parameters=[_share('config', 'velodyne.yaml'), WALL_TIME]),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
             name='velodyne_transform_node', output='both',
             parameters=[_share('config', 'velodyne.yaml'), WALL_TIME, {
                 # must be an ABSOLUTE path: the node does not resolve a bare file name
                 'calibration': PathJoinSubstitution(
                     [FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml']),
             }]),
        Node(package='swerve_navigation', executable='cloud_to_scan', name='cloud_to_scan', output='both',
             parameters=[_share('config', 'cloud_to_scan.yaml'), WALL_TIME]),

        # ---- base ----
        # NEVER auto-restart the hardware-facing node: if it died, a human finds out why before
        # the drivetrain is enabled again. (While it is down nothing feeds the enable signal, so
        # the Talons go neutral.)
        Node(package='swerve_navigation', executable='swerve_bridge', name='swerve_bridge', output='both',
             # The launch argument always overrides the yaml: "hardware" has exactly one switch.
             parameters=[_share('config', 'swerve_bridge.yaml'), WALL_TIME,
                         {'hardware': hardware,
                          'phoenix6_site_packages': phoenix6_dir,
                          # generated by Tuner X for this robot; re-copy it after re-tuning
                          'tuner_constants_dir': os.path.join(share, 'phoenix6')}],
             respawn=False),

        # ---- localisation ----
        nav2_node('nav2_map_server', 'map_server', name='map_server',
                  overrides={'yaml_filename': map_yaml}),
        nav2_node('nav2_amcl', 'amcl', name='amcl', overrides={
            'set_initial_pose': True,
            'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}}),
        _lifecycle_manager('lifecycle_manager_localization', LOCALIZATION_NODES),

        # ---- Nav2 ----
        Node(package='topic_tools', executable='relay', name='controller_tf_relay', output='both',
             arguments=['/tf', '/tf_controller']),
        # controller_server is left unnamed as in nav2_bringup: its process also hosts the
        # local_costmap node.
        nav2_node('nav2_controller', 'controller_server', remappings=controller_remaps),
        nav2_node('nav2_smoother', 'smoother_server', name='smoother_server'),
        nav2_node('nav2_planner', 'planner_server', name='planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', name='behavior_server',
                  remappings=to_nav_raw),
        # cmd_vel is the smoother's INPUT; its output topic cmd_vel_smoothed is fixed in the node.
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', name='velocity_smoother',
                  remappings=to_nav_raw),
        # cmd_vel_smoothed -> cmd_vel_nav (cmd_vel_in_topic / cmd_vel_out_topic in the yaml)
        nav2_node('nav2_collision_monitor', 'collision_monitor', name='collision_monitor'),
        # The tree path cannot live in the params file (no substitutions there): set it here.
        nav2_node('nav2_bt_navigator', 'bt_navigator', name='bt_navigator',
                  overrides={'default_nav_to_pose_bt_xml': bt_xml}),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', name='waypoint_follower'),
        _lifecycle_manager('lifecycle_manager_navigation', NAVIGATION_NODES),

        # ---- goals ----
        Node(package='swerve_navigation', executable='point_to_goal', name='point_to_goal', output='both',
             parameters=[WALL_TIME, {'global_frame': 'map', 'base_frame': 'base_footprint',
                                     'goal_heading': goal_heading}]),
        Node(package='swerve_navigation', executable='location_markers', name='location_markers',
             output='both',
             parameters=[WALL_TIME, {'locations_file': locations_file, 'global_frame': 'map',
                                     'base_frame': 'base_footprint',
                                     'approach_distance': approach}]),
    ]

    # ---- description, operator, visualisation ----
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', _share('urdf', 'swerve_base.urdf.xacro'),
                 ' base_centre_height:=', LaunchConfiguration('base_centre_height'),
                 ' lidar_yaw:=', LaunchConfiguration('lidar_yaw')]),
        value_type=str)
    with_joy = IfCondition(LaunchConfiguration('joy'))
    actions += [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='both',
             parameters=[WALL_TIME, {'robot_description': robot_description}]),

        Node(package='swerve_navigation', executable='cmd_vel_mux', name='cmd_vel_mux', output='both',
             parameters=[_share('config', 'cmd_vel_mux.yaml'), WALL_TIME]),

        # If the pad is not found after a reboot with nobody logged in on the local display (no
        # uaccess ACL on /dev/input/event*), SDL can fall back to the world-readable legacy
        # /dev/input/js* devices:  additional_env={'SDL_LINUX_JOYSTICK_CLASSIC': '1'}
        Node(package='joy', executable='joy_node', name='joy_node', output='both',
             parameters=[_share('config', 'joy.yaml'), WALL_TIME], condition=with_joy),
        Node(package='swerve_navigation', executable='joy_teleop', name='joy_teleop', output='both',
             parameters=[_share('config', 'joy.yaml'), WALL_TIME], condition=with_joy),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', LaunchConfiguration('rviz_config'),
                        '-f', LaunchConfiguration('rviz_fixed_frame')],
             parameters=[WALL_TIME], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]

    if parse_bool('nav_api', arg('nav_api')):
        # The API topic names are relative ('nav/state', ...), so api_namespace decides where they
        # appear: '/' -> /nav/state (default), '/robot_1' -> /robot_1/nav/state for a system that
        # keeps its robot under a namespace. The stack behind the API stays where it is.
        actions.append(Node(
            package='swerve_navigation', executable='nav_api', name='nav_api',
            namespace=arg('api_namespace'), output='both',
            parameters=[WALL_TIME, {
                'global_frame': 'map', 'base_frame': 'base_footprint',
                'locations_file': locations_file,
                # Goals take the same path as a goal clicked in RViz: staged approach included.
                'goal_topic': '/goal_pose_staged' if approach > 0.0 else '/goal_pose',
                'cancel_topic': '/nav_cancel'}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware', default_value='false',
            description='SAFETY. false (default): swerve_bridge runs the phoenix6 SIMULATOR, the '
                        'robot cannot move. true: swerve_bridge enables and drives the REAL '
                        'swerve modules over CAN.'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map yaml for map_server / AMCL. Empty (default) = '
                        f'<navigation_bringup share>/maps/{DEFAULT_MAP_NAME}. The saved places '
                        'are read from <map>.locations.yaml next to it.'),
        DeclareLaunchArgument(
            'nav_motion', default_value='forward', choices=['forward', 'omni'],
            description='forward: Nav2 drives like a diff-drive, always into the lidar field of '
                        'view (the robot is blind behind / right). omni: also strafe (adds '
                        'config/nav2_omni_overlay.yaml).'),
        DeclareLaunchArgument(
            'goal_heading', default_value='auto', choices=['auto', 'travel', 'keep'],
            description='Publish Point final heading only. auto: travel in forward mode, keep '
                        'in omni mode. travel: face the bearing from the robot to the clicked '
                        'point. keep: restore the heading at click time. 2D Goal Pose, saved '
                        'places and /nav/goal_pose always use the heading they carry.'),
        DeclareLaunchArgument(
            'approach_distance', default_value='0.8',
            description='Saved places, 2D Goal Pose and /nav goals: first drive to a staging '
                        'pose this far [m] behind the goal, facing the goal heading, then '
                        'straight in (no turning next to the goal). 0 = go to the goal '
                        'directly.'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='AMCL initial pose x in the map [m]. 0, 0, 0 = where mapping started.'),
        DeclareLaunchArgument(
            'y', default_value='0.0', description='AMCL initial pose y in the map [m].'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='AMCL initial pose yaw in the map [rad].'),
        DeclareLaunchArgument(
            'api_namespace', default_value='/',
            description="Namespace the /nav API is published in. '/' (default) = /nav/state, "
                        "/nav/goal_pose, ...; '/robot_1' = /robot_1/nav/... for a larger system "
                        'that namespaces its robot.'),
        DeclareLaunchArgument(
            'nav_api', default_value='true',
            description='Start nav_api: the /nav topic interface (goals, state, pose) that the '
                        'rest of the system talks to.'),
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='Start joy_node + joy_teleop (Xbox pad: hold LT to drive, RB = turbo, '
                        'B = e-stop, start = release e-stop, X = cancel navigation, A = re-zero '
                        'the field-centric heading). The pad is the only e-stop: never run the '
                        'real robot without it.'),
        DeclareLaunchArgument('rviz', default_value='true',
                             description='Start RViz. rviz:=false for a headless robot.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=_share('rviz', 'navigation.rviz'),
            description='RViz config file.'),
        DeclareLaunchArgument(
            'rviz_fixed_frame', default_value='map', description='RViz fixed frame.'),
        DeclareLaunchArgument(
            'lidar_yaw', default_value='0.0',
            description='Yaw of the velodyne frame in base_link [rad]; velodyne +X points away '
                        'from the VLP-16 cable exit. 0 = cable to the rear, 1.5708 = cable to '
                        'the right, 3.14159 = front, -1.5708 = left.'),
        DeclareLaunchArgument(
            'base_centre_height', default_value='0.152',
            description='Height of base_link (CAD datum of the lidar offsets) above the floor '
                        '[m]. With the lidar z offset 0.178 it puts the optical centre at the '
                        'measured 0.330 m.'),

        # The only action: validates everything and raises before ANY process is started.
        OpaqueFunction(function=_nodes),
    ])

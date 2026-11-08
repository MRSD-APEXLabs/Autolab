"""Software-only port test: real Autolab launch with simulated lidar and base.

No hardware driver or joystick process is created. Use an isolated container and
ROS domain, with SWERVE_NAV_FORCE_SIM=1. Production launch behavior is unchanged.
"""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('navigation_bringup'))
    spec = importlib.util.spec_from_file_location('port_launch', share / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.RUNTIME_PACKAGES = ()  # the simulated sensors do not need velodyne or Phoenix
    module.find_phoenix6 = lambda: '/unused-in-simulation'
    original = module._nodes

    def simulated(context):
        assert context.launch_configurations['hardware'] == 'false'
        actions = []
        omitted = {'velodyne_driver_node', 'velodyne_transform_node', 'cloud_to_scan',
                   'swerve_bridge', 'joy_node', 'joy_auto', 'joy_teleop', 'rviz2'}
        for action in original(context):
            if isinstance(action, Node):
                executable = perform_substitutions(
                    context, normalize_to_list_of_substitutions(action.node_executable))
                if executable in omitted:
                    continue
            actions.append(action)
        args = ['python3', str(Path(__file__).with_name('sim_world.py')), '--ros-args',
                '-r', '__node:=sim_world', '-p', f'map_yaml:={share / "maps/best.yaml"}']
        for name, key in [('start_x', 'x'), ('start_y', 'y'), ('start_yaw', 'yaw')]:
            args += ['-p', f'{name}:={context.launch_configurations[key]}']
        actions.append(ExecuteProcess(cmd=args, output='both'))
        return actions

    module._nodes = simulated
    return module.generate_launch_description()

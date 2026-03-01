import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("demo_bot", package_name="base_moveit_config_2")
        .to_moveit_configs()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("base_moveit_config_2"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    move_to_pose_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="global_planner",
                executable="move_to_pose",
                name="move_to_pose_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            )
        ],
    )

    return LaunchDescription([
        moveit_launch,
        move_to_pose_node,
    ])
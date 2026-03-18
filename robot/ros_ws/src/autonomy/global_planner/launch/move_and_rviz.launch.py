import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder

# Import the specific pieces we need, ignoring the fake hardware
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
)

def generate_launch_description():

    # 1. Load the config (unchanged)
    moveit_config = (
        MoveItConfigsBuilder("demo_bot", package_name="base_moveit_config_2")
        .to_moveit_configs()
    )
    rviz_config = os.path.join(
        get_package_share_directory("base_moveit_config_2"),
        "config", "moveit.rviz"  
    )



    # 2. Your custom node (unchanged)
    move_to_pose_node = TimerAction(
        period=15.0,
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
                    {"use_sim_time": True},  
                ],
            )
        ],
    )
    

    # 3. The Controller Manager (The missing piece)
    control_node = TimerAction(
        period=3.0,  # Give Isaac time to start publishing
        actions=[Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(
                    get_package_share_directory("base_moveit_config_2"),
                    "config", "ros2_controllers.yaml"
                ),
                {"use_sim_time": True},
            ],
            output="both",
            # Add this to see ALL output including hardware errors:
            emulate_tty=True,
        )]
    )

    # 4. The Broadcaster (The bridge to /joint_states)


    # Delay spawners to give controller_manager time to start
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
        )]
    )

    arm_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["demo_arm_bot_controller", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["demo_gripper_bot_controller", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
        )]
    )
    

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"trajectory_execution.allowed_start_tolerance": 0.05},
            {"trajectory_execution.wait_for_trajectory_completion": True},
            {"current_state_monitor_wait_time": 10.0},
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'moveit_ros.planning_scene_monitor.planning_scene_monitor:=ERROR',
            '--log-level', 'moveit_robot_model.robot_model:=ERROR',  # silence SRDF virtual joint
        ],
    )

    rviz_node = TimerAction(
        period=5.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", rviz_config,
                    "--ros-args", "--log-level", "ERROR"],  # suppress RViz noise
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": True},
            ],
        )]
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )
    return LaunchDescription([
        rsp_node,                           # 1. robot state publisher first
        move_group_node,                    # 2. move_group
        rviz_node,                          # 3. rviz (delayed 5s internally)
        control_node,                       # 4. ros2_control (delayed 3s)
        joint_state_broadcaster_spawner,    # 5. broadcaster (delayed 8s)
        arm_controller_spawner,             # 6. arm controller (delayed 10s)
        gripper_controller_spawner,         # 7. gripper controller (delayed 10s)
        move_to_pose_node,                  # 8. your node (delayed 15s)
    ])
#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_display_robot_description(urdf_path):
    """Load the mechanical URDF without its duplicate camera TF links."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for element in list(root):
        if element.tag in ("link", "joint") and element.get("name", "").startswith("zed_"):
            root.remove(element)
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    base_urdf_share = get_package_share_directory("base_urdf")
    bringup_share = get_package_share_directory("robot_bringup")

    urdf_path = os.path.join(base_urdf_share, "robot_isaac.urdf")
    rviz_path = os.path.join(bringup_share, "rviz", "isaac_sim.rviz")
    robot_description = load_display_robot_description(urdf_path)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the /clock published by Isaac Sim",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the Isaac mobile-base configuration",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="autolab_robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                        "ignore_timestamp": True,
                        "publish_frequency": 30.0,
                    }
                ],
                # Isaac supplies joint states; this node publishes the connected
                # mechanical TF tree used by the display URDF.
                remappings=[("joint_states", "/autolab/joint_states")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="autolab_rviz",
                output="screen",
                arguments=["-d", rviz_path],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
            # The collected USD and display URDF use incompatible chassis
            # origins/directions.  Publish an explicit display calibration for
            # the base-mounted ZED X: chassis center, facing -X, 30 deg down.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="zed_x_rviz_calibration",
                arguments=[
                    "--x", "0.17517949",
                    "--y", "-0.20472002",
                    "--z", "1.50901977",
                    "--qx", "0.61237244",
                    "--qy", "0.61237244",
                    "--qz", "-0.35355339",
                    "--qw", "-0.35355339",
                    "--frame-id", "base_footprint",
                    "--child-frame-id", "zed_x_left_camera",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Calibrated wrist-camera transform.  Keeping this on the URDF
            # gripper frame avoids the imported USD link-origin mismatch.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="zed_x_nano_rviz_calibration",
                arguments=[
                    "--x", "0.05207864",
                    "--y", "0.03019945",
                    "--z", "-0.04880624",
                    "--qx", "0.56086248",
                    "--qy", "0.46390741",
                    "--qz", "0.50499490",
                    "--qw", "-0.46390015",
                    "--frame-id", "gripper_v43_1",
                    "--child-frame-id", "zed_x_nano_left_camera",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

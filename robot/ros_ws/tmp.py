#!/usr/bin/env python3
import rclpy
from moveit_ros_planning_interface import PlanningSceneInterface

rclpy.init()
scene = PlanningSceneInterface()
objects = scene.get_objects()
for obj in objects:
    print(f"ID: {obj.id}, Frame: {obj.header.frame_id}")
rclpy.shutdown()

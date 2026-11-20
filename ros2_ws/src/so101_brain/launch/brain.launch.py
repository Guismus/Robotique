#!/usr/bin/env python3
"""brain.launch.py — launches the brain of the arm (IK + pick & place)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stack", default_value="0", description="0 = phase 1 (bac), N = empiler N cubes"),
        Node(
            package="so101_brain",
            executable="brain",
            name="so101_brain",
            output="screen",
            parameters=[{"stack": ParameterValue(LaunchConfiguration("stack"), value_type=int)}],
        ),
    ])

#!/usr/bin/env python3
"""perception.launch.py - launches the SO-ARM101 perception node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="so101_perception",
            executable="perception",
            name="so101_perception",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
        ),
    ])

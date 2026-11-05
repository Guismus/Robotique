#!/usr/bin/env python3
"""perception.launch.py - launches the SO-ARM101 perception node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="so101_perception",
            executable="perception",
            name="so101_perception",
            output="screen",
        ),
    ])

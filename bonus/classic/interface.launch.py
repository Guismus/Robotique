"""Affichage uniquement : modele du bras, image annotee, cubes vus et reperes TF."""

from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path("/project/ros2_ws/src/so101_description")
    urdf = (
        (root / "urdf/so101.urdf")
        .read_text()
        .replace("package://so101_description/meshes/", f"file://{root}/meshes/")
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": urdf}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", "/project/bonus/classic/bonus.rviz"],
            ),
        ]
    )

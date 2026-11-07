#!/usr/bin/env python3
"""driver.launch.py - launches the SO-ARM101 driver node

    ros2 launch so101_driver driver.launch.py                # simulation
    ros2 launch so101_driver driver.launch.py use_sim:=false # real robot
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="Simulated backend or real robot"),
        DeclareLaunchArgument("sim_seed", default_value="-1", description="Seed for repeatable simulation (-1: random)"),
        DeclareLaunchArgument("use_sim_time", default_value=LaunchConfiguration("use_sim"),
                              description="Publish/use the MuJoCo simulation clock"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",  #  Use the following command to find the robot port:
                              # sudo udevadm monitor --subsystem-match=tty --property | grep --line-buffered -E "(DEVNAME=|ACTION=)"
                              description="Serial port of the real SO-ARM101"),
        Node(
            package="so101_driver",
            executable="driver",
            name="so101_driver",
            output="screen",
            parameters=[{
                "use_sim": ParameterValue(LaunchConfiguration("use_sim"), value_type=bool),
                "port": LaunchConfiguration("port"),
                "sim_seed": ParameterValue(LaunchConfiguration("sim_seed"), value_type=int),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
    ])

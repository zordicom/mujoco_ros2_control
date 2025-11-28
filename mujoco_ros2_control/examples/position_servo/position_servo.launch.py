"""
Copyright 2025 Zordi, Inc. All rights reserved.

Position Servo Example Launch File
Demonstrates position-only control (like Dynamixel servos)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_ros2_control")

    urdf_path = os.path.join(pkg_share, "examples/position_servo/position_servo.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    config_path = os.path.join(pkg_share, "examples/position_servo/position_servo.yaml")

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            config_path,
        ],
        output="screen",
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawn trajectory controller (after joint state broadcaster)
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Unpause simulation
    unpause_sim = Node(
        package="mujoco_ros2_control",
        executable="unpause_simulation.py",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[trajectory_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=trajectory_controller_spawner,
                on_exit=[unpause_sim],
            )
        ),
    ])

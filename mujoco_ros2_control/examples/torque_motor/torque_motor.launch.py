"""
Copyright 2025 Zordi, Inc. All rights reserved.

Torque Motor Example Launch File
Demonstrates effort-only control (like Kuka iiwa)
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_ros2_control")

    urdf_path = os.path.join(pkg_share, "examples/torque_motor/torque_motor.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    config_path = os.path.join(pkg_share, "examples/torque_motor/torque_motor.yaml")

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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn effort controller (after joint state broadcaster)
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_effort_controller", "--controller-manager", "/controller_manager"],
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
                on_exit=[effort_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=effort_controller_spawner,
                on_exit=[unpause_sim],
            )
        ),
    ])


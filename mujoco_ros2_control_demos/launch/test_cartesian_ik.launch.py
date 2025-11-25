"""
Copyright 2025 Zordi, Inc. All rights reserved.

Minimal launch file for debugging Cartesian IK controller.

This test loads only the essential controllers:
  - joint_state_broadcaster
  - zordi_joint_effort_controller (target JTC)
  - zordi_cartesian_ik_controller (forwards to JTC)

For debugging IK controller integration with a joint trajectory controller.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    """Generate minimal launch description for IK controller testing."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "planar_2dof.xacro.urdf"
    controller_config = pkg_share / "config" / "test_planar_2dof.yaml"

    # Read URDF
    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(controller_config),
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Minimal set of controllers for IK testing
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    load_zordi_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_joint_effort_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_cartesian_ik_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_cartesian_ik_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    # Reset to home keyframe after controllers load
    reset_to_home = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/reset_to_keyframe",
            "mujoco_ros2_control_msgs/srv/ResetToKeyframe",
            "{keyframe: 'home'}",
        ],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        # Load only essential controllers for IK testing
        load_joint_state_broadcaster,
        load_zordi_joint_effort_controller,
        load_zordi_cartesian_ik_controller,
        # Reset to home after controllers load
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=load_joint_state_broadcaster,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[reset_to_home],
                    )
                ],
            )
        ),
    ])

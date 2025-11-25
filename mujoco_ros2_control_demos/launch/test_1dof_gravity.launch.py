"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for 1-DOF gravity compensation and MIT mode demonstrations.

This test uses a VERTICAL pendulum with two controllers:
  1. zordi_grav_comp_controller: Pure gravity compensation (no PD control - fully backdrivable)
  2. zordi_ros_controllers: MIT mode with trajectory tracking and gravity compensation

Architecture:
  - MujocoSystem plugin loaded by controller_manager (lifecycle mode)
  - Viewer is integrated into plugin (enable via URDF parameter 'mujoco_viewer')
  - Plugin handles simulation stepping, services, clock publishing

Test objectives:
  Example 1 (zordi_grav_comp_controller):
    - Verify pure gravity compensation without trajectory tracking
    - Pendulum holds upright and is fully backdrivable

  Example 2 (zordi_ros_controllers):
    - Verify MIT mode with trajectory tracking
    - Send trajectories while maintaining gravity compensation
    - Smooth tracking with automatic hold after trajectory completion

Expected behavior:
  - Without gravity comp: pendulum falls to q=-π/2 (hanging down)
  - With gravity comp only: pendulum holds at q=0 (upright) and is backdrivable
  - With MIT controller: pendulum tracks commanded trajectories with gravity comp
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 1-DOF gravity test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "test_1dof_gravity.xacro.urdf"
    controller_config = pkg_share / "config" / "test_1dof_gravity.yaml"

    # Read URDF
    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Controller manager node (replaces old mujoco_ros2_control_node)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(controller_config),
        ],
        output="screen",
    )

    # Robot state publisher (needed for controller to fetch robot_description)
    # Note: Viewer is now integrated into MujocoSystem plugin - enable via URDF parameter
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Load controllers using spawner (more robust than ros2 control CLI)
    # Spawner automatically waits for controller_manager services to be ready
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    load_zordi_grav_comp_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_grav_comp_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_ros_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_ros_controllers",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    # Reset to test_pose keyframe after controllers load
    reset_to_test_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/reset_to_keyframe",
            "mujoco_ros2_control_msgs/srv/ResetToKeyframe",
            "{keyframe: 'test_pose'}",
        ],
        output="screen",
    )

    return LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        # Load controllers using spawner nodes
        # Spawner automatically waits for controller_manager to be ready
        # Both controllers are loaded in inactive state - activate manually:
        #   - zordi_grav_comp_controller: For pure gravity compensation (Example 1)
        #   - zordi_ros_controllers: For trajectory tracking with gravity comp (Example 2)
        load_joint_state_broadcaster,
        load_zordi_grav_comp_controller,
        load_zordi_ros_controllers,
        # Reset to test_pose after controllers load (with small delay)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=load_joint_state_broadcaster,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[reset_to_test_pose],
                    )
                ],
            )
        ),
    ])

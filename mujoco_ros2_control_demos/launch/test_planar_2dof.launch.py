"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for 2-DOF planar arm controller demonstrations.

This test uses a HORIZONTAL 2-link planar arm with five Zordi controllers:
  1. zordi_mit_controller: Gravity compensation and trajectory tracking (joint space)
  2. zordi_mit_rnea_controller: Joint space control with full inverse dynamics (RNEA)
  3. zordi_mit_gravity_controller: Pure gravity compensation, no trajectory tracking (fully backdrivable)
  4. zordi_cartesian_controller: Cartesian impedance control
  5. zordi_cartesian_rnea_controller: Cartesian control with full inverse dynamics (RNEA)

Architecture:
  - MujocoSystem plugin loaded by controller_manager (lifecycle mode)
  - Viewer is integrated into plugin (enable via URDF parameter 'mujoco_viewer')
  - Plugin handles simulation stepping, services, clock publishing

Expected behavior:
  - Zero gravity environment (no falling)
  - Robot starts at bent configuration q=[0.3, -0.2] (home keyframe)
  - Controllers provide damping and tracking forces
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 2-DOF planar arm test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "planar_2dof.xacro.urdf"
    controller_config = pkg_share / "config" / "test_planar_2dof.yaml"

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

    load_zordi_mit_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_mit_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_cartesian_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_cartesian_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_mit_rnea_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_mit_rnea_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_cartesian_rnea_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_cartesian_rnea_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_mit_gravity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_mit_gravity_controller",
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
        # Load controllers using spawner nodes
        # Spawner automatically waits for controller_manager to be ready
        # All five controllers are loaded in inactive state - activate manually:
        #   - zordi_mit_controller: Joint space gravity comp and trajectory tracking
        #   - zordi_mit_rnea_controller: Joint space with full inverse dynamics (RNEA)
        #   - zordi_mit_gravity_controller: Pure gravity comp, no trajectory tracking
        #   - zordi_cartesian_controller: Cartesian impedance control
        #   - zordi_cartesian_rnea_controller: Cartesian control with full inverse dynamics
        load_joint_state_broadcaster,
        load_zordi_mit_controller,
        load_zordi_mit_rnea_controller,
        load_zordi_mit_gravity_controller,
        load_zordi_cartesian_controller,
        load_zordi_cartesian_rnea_controller,
        # Reset to home after controllers load (with small delay)
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

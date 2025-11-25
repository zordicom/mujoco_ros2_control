"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for 2-DOF vertical double pendulum gravity compensation demonstrations.

This test uses a VERTICAL double pendulum with 5 Zordi controllers + 1 ROS-native:
  1. zordi_grav_comp_controller: Pure gravity compensation (backdrivable)
  2. zordi_joint_trajectory_controller: Joint space with gravity comp
  3. zordi_joint_rnea_controller: Joint space with full inverse dynamics
  4. zordi_cartesian_controller: Cartesian impedance control
  5. zordi_cartesian_rnea_controller: Cartesian with inverse dynamics
  6. joint_trajectory_controller: ROS-native (for comparison)

Test objectives:
  Example 1 (zordi_grav_comp_controller):
    - Pure gravity compensation (no trajectory tracking)
    - Fully backdrivable, holds position via gravity comp only

  Example 2 (zordi_joint_trajectory_controller):
    - Joint space trajectory tracking with gravity compensation
    - MIT mode with position/velocity/effort control

  Example 3 (zordi_joint_rnea_controller):
    - Joint space with full inverse dynamics (RNEA)
    - Improved tracking accuracy with feedforward dynamics

  Example 4 (zordi_cartesian_controller):
    - Cartesian impedance control with gravity compensation
    - End-effector pose tracking in vertical plane

  Example 5 (zordi_cartesian_rnea_controller):
    - Cartesian control with full inverse dynamics
    - Highest accuracy Cartesian tracking

  Example 6 (joint_trajectory_controller):
    - ROS-native controller for comparison
    - No gravity compensation (will struggle)

Expected behavior:
  - Gravity environment (g = 9.81 m/s²)
  - Robot starts at q=[0.3, -0.2] (near hanging, with gravity torques)
  - q=[0, 0] is the natural hanging configuration (stable equilibrium)
  - With gravity comp: robot holds any position and tracks trajectories
  - Controllers provide damping, tracking forces, and gravity compensation
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 2-DOF vertical double pendulum gravity test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "test_2dof_gravity.xacro.urdf"
    mujoco_model = pkg_share / "mujoco_models" / "test_2dof_gravity.xml"
    controller_config = pkg_share / "config" / "test_2dof_gravity.yaml"

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

    load_zordi_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    load_zordi_joint_rnea_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_joint_rnea_controller",
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

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
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
        # Launch argument for viewer (integrated into MujocoSystem plugin via URDF parameter)
        # Note: Viewer is now enabled/disabled via URDF <param name="mujoco_viewer">true/false</param>
        # This launch argument is kept for backward compatibility but has no effect
        DeclareLaunchArgument(
            "show_viewer",
            default_value="false",
            description="[DEPRECATED] Viewer is now controlled via URDF parameter 'mujoco_viewer'",
        ),
        controller_manager_node,
        robot_state_pub_node,
        # Load controllers when controller_manager starts
        # All 6 controllers loaded in inactive state - activate manually:
        #   - zordi_grav_comp_controller: Pure gravity compensation
        #   - zordi_joint_trajectory_controller: Joint space with gravity
        #   - zordi_joint_rnea_controller: Joint space with RNEA
        #   - zordi_cartesian_controller: Cartesian with gravity comp
        #   - zordi_cartesian_rnea_controller: Cartesian with RNEA
        #   - joint_trajectory_controller: ROS-native (no gravity comp)
        # Using spawner nodes - they automatically wait for controller_manager to be ready
        load_joint_state_broadcaster,
        load_zordi_grav_comp_controller,
        load_zordi_joint_trajectory_controller,
        load_zordi_joint_rnea_controller,
        load_zordi_cartesian_controller,
        load_zordi_cartesian_rnea_controller,
        load_joint_trajectory_controller,
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

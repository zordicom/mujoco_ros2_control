"""
Launch file for 1-DOF gravity compensation and MIT mode demonstrations.

This test uses a VERTICAL pendulum with two controllers:
  1. zordi_grav_comp_controller: Pure gravity compensation (no PD control - fully backdrivable)
  2. zordi_mit_controller: MIT mode with trajectory tracking and gravity compensation

Test objectives:
  Example 1 (zordi_grav_comp_controller):
    - Verify pure gravity compensation without trajectory tracking
    - Pendulum holds upright and is fully backdrivable

  Example 2 (zordi_mit_controller):
    - Verify MIT mode with trajectory tracking
    - Send trajectories while maintaining gravity compensation
    - Smooth tracking with automatic hold after trajectory completion

Expected behavior:
  - Without gravity comp: pendulum falls to q=-π/2 (hanging down)
  - With gravity comp only: pendulum holds at q=0 (upright) and is backdrivable
  - With MIT controller: pendulum tracks commanded trajectories with gravity comp
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 1-DOF gravity test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "test_1dof_gravity.xacro.urdf"
    mujoco_model = pkg_share / "mujoco_models" / "test_1dof_gravity.xml"
    controller_config = pkg_share / "config" / "test_1dof_gravity.yaml"

    # Read URDF
    robot_description = Path(urdf_file).read_text()

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

    # Optional MuJoCo viewer (separate process)
    viewer_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_viewer",
        name="mujoco_viewer",
        parameters=[
            {
                "mujoco_model_path": str(mujoco_model),
                "service_namespace": "/mujoco_system",
            }
        ],
        condition=IfCondition(LaunchConfiguration("show_viewer")),
        output="screen",
    )

    # Robot state publisher (needed for controller to fetch robot_description)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Load controllers using ros2 control CLI (standard mujoco_ros2_control pattern)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_zordi_grav_comp_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_grav_comp_controller",
        ],
        output="screen",
    )

    load_zordi_mit_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_mit_controller",
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
        # Launch argument for viewer
        DeclareLaunchArgument(
            "show_viewer",
            default_value="true",
            description="Launch MuJoCo interactive viewer (default: true)",
        ),
        controller_manager_node,
        robot_state_pub_node,
        viewer_node,
        # Load controllers when controller_manager starts
        # Both controllers are loaded in inactive state - activate manually as needed:
        #   - zordi_grav_comp_controller: For pure gravity compensation (Example 1)
        #   - zordi_mit_controller: For trajectory tracking with gravity comp (Example 2)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    load_joint_state_broadcaster,
                    load_zordi_grav_comp_controller,
                    load_zordi_mit_controller,
                    reset_to_test_pose,
                ],
            )
        ),
    ])

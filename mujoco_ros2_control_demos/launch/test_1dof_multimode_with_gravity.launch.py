"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for 1-DOF multimode control test with gravity compensation.

This test uses a VERTICAL pendulum (test_1dof_gravity.xml) with gravity enabled.
Tests two control modes:
  - joint_trajectory_controller (position+velocity)
  - zordi_mit_controller (position+velocity+effort with gravity compensation)

Test objective:
  - Verify both controllers can move the pendulum
  - Verify zordi_mit_controller holds position with gravity compensation
  - Compare controller switching behavior

Expected behavior:
  - JTC alone: pendulum will drift under gravity after trajectory completes
  - zordi_mit: pendulum holds position with gravity compensation (< 0.001 rad drift)

Usage:
  ros2 launch mujoco_ros2_control_demos test_1dof_multimode_with_gravity.launch.py

  Then use:
    - Automated tests: scripts/test_1dof_multimode_scenario_a.py or scenario_b.py
    - Manual tests: See docs/MANUAL_1DOF_MULTIMODE_TEST.md
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Load URDF (vertical pendulum with gravity)
    urdf_file = pkg_share / "urdf" / "test_1dof_gravity.xacro.urdf"
    robot_description = Path(urdf_file).read_text()

    # MuJoCo model path (vertical pendulum with gravity)
    mujoco_model = str(pkg_share / "mujoco_models" / "test_1dof_gravity.xml")

    # Controller config (both JTC and zordi_mit)
    controller_config = str(
        pkg_share / "config" / "test_1dof_multimode_with_gravity.yaml"
    )

    # Initial pose config
    initial_pose_config = str(pkg_share / "config" / "initial_pose_test.yaml")

    # MuJoCo ROS2 Control node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_description},
            {"mujoco_model_path": mujoco_model},
            {"headless": False},  # Show MuJoCo viewer for visualization
            {"unpause": True},
            {"initial_pose": "home"},  # Start at upright position
            {"initial_pose_config": initial_pose_config},
            controller_config,
        ],
        output="screen",
    )

    # Joint state broadcaster (start immediately)
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Load joint_trajectory_controller (inactive)
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_mit_controller (inactive)
    load_zordi = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_mit_controller", "--inactive"],
        output="screen",
    )

    return LaunchDescription([
        mujoco_node,
        # Start joint_state_broadcaster when mujoco node starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[spawn_joint_broadcaster, load_jtc, load_zordi],
            )
        ),
    ])

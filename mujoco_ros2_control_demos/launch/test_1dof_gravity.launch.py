"""
Launch file for 1-DOF gravity compensation test with zordi_mit_controller.

This test uses a VERTICAL pendulum (unlike test_1dof_multimode which is horizontal).
The pendulum will fall without gravity compensation.

Test objective:
  - Verify zordi_mit_controller loads with gravity compensation enabled
  - Verify Pinocchio integration works
  - Verify pendulum holds upright position (q=0) without falling

Expected behavior:
  - Without gravity comp: pendulum falls to q=-π/2 (hanging down)
  - With gravity comp: pendulum holds at q=0 (upright)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 1-DOF gravity test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "test_1dof_gravity.xacro.urdf"
    mujoco_model = pkg_share / "mujoco_models" / "test_1dof_gravity.xml"
    controller_config = pkg_share / "config" / "test_1dof_gravity.yaml"
    initial_pose_config = pkg_share / "config" / "initial_pose_test.yaml"

    # Read URDF
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # MuJoCo node with ros2_control
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {
                "robot_description": robot_description,
                "mujoco_model_path": str(mujoco_model),
                "headless": True,  # No viewer
                "unpause": True,  # Start simulation immediately
                "initial_pose": "test_pose",  # Name of pose in config
                "initial_pose_config": str(
                    initial_pose_config
                ),  # Path to YAML with poses
            },
            controller_config,
        ],
        output="screen",
    )

    # Spawn joint state broadcaster
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Spawn zordi_mit_controller
    spawn_zordi_mit = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_mit_controller"],
        output="screen",
    )

    return LaunchDescription([
        mujoco_node,
        # Start broadcaster when mujoco starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[spawn_joint_broadcaster],
            )
        ),
        # Start zordi_mit_controller after broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_broadcaster,
                on_exit=[spawn_zordi_mit],
            )
        ),
    ])

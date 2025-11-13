"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for 1-DoF multi-interface controller test.
Tests joint_trajectory_controller (position+velocity) and zordi_mit_controller (position+velocity+effort).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Load URDF
    urdf_file = pkg_share / "urdf" / "test_1dof_multimode.xacro.urdf"
    robot_description = Path(urdf_file).read_text()

    # MuJoCo model path
    mujoco_model = str(pkg_share / "mujoco_models" / "test_1dof_multimode.xml")

    # Controller config (multi-interface controllers)
    controller_config = str(pkg_share / "config" / "test_1dof_mit.yaml")

    # MuJoCo ROS2 Control node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_description},
            {"mujoco_model_path": mujoco_model},
            {"headless": True},
            {"unpause": True},
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

    # Note: Controllers are loaded but NOT activated by default
    # This allows manual switching for testing
    # Use: ros2 control load_controller <controller_name>
    #      ros2 control set_controller_state <controller_name> active

    return LaunchDescription([
        mujoco_node,
        # Start joint_state_broadcaster when mujoco node starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[spawn_joint_broadcaster],
            )
        ),
        # Controllers loaded but inactive - manually activate for testing
    ])

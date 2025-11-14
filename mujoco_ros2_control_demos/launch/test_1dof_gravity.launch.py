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
from launch.event_handlers import OnProcessStart
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

    # MuJoCo node with ros2_control
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            str(controller_config),
            {
                "robot_description": robot_description,
                "mujoco_model_path": str(mujoco_model),
                "headless": False,  # No viewer window
                "initial_keyframe": "pos_large",  # Name of keyframe in XML
                "use_sim_time": True,  # Use MuJoCo simulation clock
            },
        ],
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

    load_zordi_mit_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "zordi_mit_controller",
        ],
        output="screen",
    )

    return LaunchDescription([
        mujoco_node,
        robot_state_pub_node,
        # Load controllers when mujoco node starts (standard mujoco_ros2_control pattern)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[load_joint_state_broadcaster, load_zordi_mit_controller],
            )
        ),
    ])

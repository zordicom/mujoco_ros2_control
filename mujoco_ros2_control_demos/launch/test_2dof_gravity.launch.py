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
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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

    # MuJoCo node with ros2_control
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            str(controller_config),
            {
                "robot_description": robot_description,
                "mujoco_model_path": str(mujoco_model),
                "headless": False,  # Show viewer
                "initial_keyframe": "test_pose",  # Start at q=[0.3, -0.2]
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

    load_zordi_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_zordi_joint_rnea_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_joint_rnea_controller",
        ],
        output="screen",
    )

    load_zordi_cartesian_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_cartesian_controller",
        ],
        output="screen",
    )

    load_zordi_cartesian_rnea_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_cartesian_rnea_controller",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    return LaunchDescription([
        mujoco_node,
        robot_state_pub_node,
        # Load controllers when mujoco node starts
        # All 6 controllers loaded in inactive state - activate manually:
        #   - zordi_grav_comp_controller: Pure gravity compensation
        #   - zordi_joint_trajectory_controller: Joint space with gravity
        #   - zordi_joint_rnea_controller: Joint space with RNEA
        #   - zordi_cartesian_controller: Cartesian with gravity comp
        #   - zordi_cartesian_rnea_controller: Cartesian with RNEA
        #   - joint_trajectory_controller: ROS-native (no gravity comp)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[
                    load_joint_state_broadcaster,
                    load_zordi_grav_comp_controller,
                    load_zordi_joint_trajectory_controller,
                    load_zordi_joint_rnea_controller,
                    load_zordi_cartesian_controller,
                    load_zordi_cartesian_rnea_controller,
                    load_joint_trajectory_controller,
                ],
            )
        ),
    ])

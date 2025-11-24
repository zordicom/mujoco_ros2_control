"""
Launch file for 2-DOF planar arm controller demonstrations.

This test uses a HORIZONTAL 2-link planar arm with five Zordi controllers:
  1. zordi_mit_controller: Gravity compensation and trajectory tracking (joint space)
  2. zordi_mit_rnea_controller: Joint space control with full inverse dynamics (RNEA)
  3. zordi_mit_gravity_controller: Pure gravity compensation, no trajectory tracking (fully backdrivable)
  4. zordi_cartesian_controller: Cartesian impedance control
  5. zordi_cartesian_rnea_controller: Cartesian control with full inverse dynamics (RNEA)

Architecture:
  - MujocoSystem plugin loaded by controller_manager (lifecycle mode)
  - Optional MuJoCo viewer (separate process, visualization only)
  - Plugin handles simulation stepping, services, clock publishing

Launch arguments:
  show_viewer: true/false (default: true) - Launch interactive MuJoCo viewer

Usage:
  # With viewer (default)
  ros2 launch mujoco_ros2_control_demos test_planar_2dof.launch.py

  # Without viewer (use RViz instead)
  ros2 launch mujoco_ros2_control_demos test_planar_2dof.launch.py show_viewer:=false

Expected behavior:
  - Zero gravity environment (no falling)
  - Robot starts at bent configuration q=[0.3, -0.2] (home keyframe)
  - Controllers provide damping and tracking forces
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for 2-DOF planar arm test."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Paths
    urdf_file = pkg_share / "urdf" / "planar_2dof.xacro.urdf"
    mujoco_model = pkg_share / "mujoco_models" / "planar_2dof.xml"
    controller_config = pkg_share / "config" / "test_planar_2dof.yaml"

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

    load_zordi_mit_rnea_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_mit_rnea_controller",
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

    load_zordi_mit_gravity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_mit_gravity_controller",
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
        # All five controllers are loaded in inactive state - activate manually as needed:
        #   - zordi_mit_controller: Joint space gravity comp and trajectory tracking
        #   - zordi_mit_rnea_controller: Joint space with full inverse dynamics (RNEA)
        #   - zordi_mit_gravity_controller: Pure gravity comp, no trajectory tracking
        #   - zordi_cartesian_controller: Cartesian impedance control
        #   - zordi_cartesian_rnea_controller: Cartesian control with full inverse dynamics
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    load_joint_state_broadcaster,
                    load_zordi_mit_controller,
                    load_zordi_mit_rnea_controller,
                    load_zordi_mit_gravity_controller,
                    load_zordi_cartesian_controller,
                    load_zordi_cartesian_rnea_controller,
                    reset_to_home,
                ],
            )
        ),
    ])

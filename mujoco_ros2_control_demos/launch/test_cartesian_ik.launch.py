"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for Cartesian IK controller with GRAVITY compensation.

This test uses the 2-DOF vertical double pendulum (with gravity) and
zordi_joint_mit_controller (which has gravity compensation to hold position).

Vertical pendulum kinematics (Y-axis rotation):
  - Joint 1 at (0, 0, 1.5) in world frame
  - Link 1: 0.5m, hangs downward (-Z)
  - Link 2: 0.5m, hangs from end of link1
  - q=[0, 0] = natural hanging configuration

Test waypoints (FK-computed from joint configurations):
  NOTE: Positive Y rotation swings pendulum in -X direction!
  Starting from keyframe q=[0.30, -0.20], moving +0.1 rad per joint per waypoint:
  q=[0.40, -0.10] -> EE at (-0.3425, 0, 0.5618), quat(w=0.9888, y=0.1494)
  q=[0.50,  0.00] -> EE at (-0.4794, 0, 0.6224), quat(w=0.9689, y=0.2474)
  q=[0.60,  0.10] -> EE at (-0.6044, 0, 0.7049), quat(w=0.9394, y=0.3429)
  q=[0.70,  0.20] -> EE at (-0.7138, 0, 0.8068), quat(w=0.9004, y=0.4350)
  q=[0.80,  0.30] -> EE at (-0.8043, 0, 0.9248), quat(w=0.8525, y=0.5227)

Launch sequence (all while simulation is PAUSED until step 4):
  1. Start controller manager with 2dof_gravity robot
  2. Load and activate controllers: joint_state_broadcaster, joint_mit, IK
  3. Reset robot to test_pose keyframe (works while paused!)
  4. Unpause simulation (controllers active, robot at correct position)
  5. Send test Cartesian trajectory (5 waypoints in XZ plane)

Usage:
  ros2 launch mujoco_ros2_control_demos test_cartesian_ik.launch.py

Monitor:
  ros2 topic echo /joint_states --once
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
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """Generate automated launch description for IK controller testing with gravity."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Use 2-DOF vertical double pendulum with gravity
    urdf_file = pkg_share / "urdf" / "test_2dof_gravity.xacro.urdf"
    controller_config = pkg_share / "config" / "test_2dof_gravity.yaml"

    # Read URDF
    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(controller_config),
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # === Step 1: Load controllers ===
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Load and activate Zordi Joint MIT controller (has gravity compensation)
    # Using spawner without --inactive to activate immediately
    load_zordi_joint_mit_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_joint_mit_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # Load and activate IK controller (forwards to zordi_joint_mit_controller)
    # Using spawner without --inactive to activate immediately
    load_zordi_cartesian_ik_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_cartesian_ik_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # === Step 2: Reset to test_pose keyframe (q=[0.3, -0.2]) ===
    # (Controllers are activated automatically by spawners)
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

    # === Step 3: Unpause simulation ===
    unpause_simulation = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/simulation_control",
            "mujoco_ros2_control_msgs/srv/SimulationControl",
            "{command: 'unpause'}",
        ],
        output="screen",
    )

    # === Step 4: Send test Cartesian trajectory ===
    # FK-computed waypoints for vertical pendulum (XZ plane, Y-axis rotation):
    # NOTE: Positive Y rotation swings pendulum in -X direction!
    # Starting from keyframe q=[0.30, -0.20], moving +0.1 rad per joint per waypoint:
    #   q=[0.40, -0.10] -> (-0.3425, 0, 0.5618), quat(w=0.9888, y=0.1494)
    #   q=[0.50,  0.00] -> (-0.4794, 0, 0.6224), quat(w=0.9689, y=0.2474)
    #   q=[0.60,  0.10] -> (-0.6044, 0, 0.7049), quat(w=0.9394, y=0.3429)
    #   q=[0.70,  0.20] -> (-0.7138, 0, 0.8068), quat(w=0.9004, y=0.4350)
    #   q=[0.80,  0.30] -> (-0.8043, 0, 0.9248), quat(w=0.8525, y=0.5227)
    # Using 2s per waypoint (10s total) for smoother tracking with PD controller
    send_test_trajectory = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/cartesian_trajectory",
            "moveit_msgs/msg/CartesianTrajectory",
            """{
                header: {frame_id: 'world'},
                tracked_frame: 'ee_link',
                points: [
                    {
                        point: {
                            pose: {
                                position: {x: -0.3425, y: 0.0, z: 0.5618},
                                orientation: {w: 0.9888, x: 0.0, y: 0.1494, z: 0.0}
                            }
                        },
                        time_from_start: {sec: 2}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: -0.4794, y: 0.0, z: 0.6224},
                                orientation: {w: 0.9689, x: 0.0, y: 0.2474, z: 0.0}
                            }
                        },
                        time_from_start: {sec: 4}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: -0.6044, y: 0.0, z: 0.7049},
                                orientation: {w: 0.9394, x: 0.0, y: 0.3429, z: 0.0}
                            }
                        },
                        time_from_start: {sec: 6}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: -0.7138, y: 0.0, z: 0.8068},
                                orientation: {w: 0.9004, x: 0.0, y: 0.4350, z: 0.0}
                            }
                        },
                        time_from_start: {sec: 8}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: -0.8043, y: 0.0, z: 0.9248},
                                orientation: {w: 0.8525, x: 0.0, y: 0.5227, z: 0.0}
                            }
                        },
                        time_from_start: {sec: 10}
                    }
                ]
            }""",
        ],
        output="screen",
    )

    return LaunchDescription([
        # Launch argument
        DeclareLaunchArgument(
            "auto_send_trajectory",
            default_value="true",
            description="Automatically send test trajectory after setup",
        ),
        # Core nodes
        controller_manager_node,
        robot_state_pub_node,
        # Load controllers
        load_joint_state_broadcaster,
        load_zordi_joint_mit_controller,
        load_zordi_cartesian_ik_controller,
        # Chain of events after controllers load and activate:
        # Simulation is PAUSED. Reset works while paused (processed before pause check).
        # Spawners activate controllers automatically (no --inactive flag).
        #
        # 1. After IK spawner exits -> reset to test_pose (works while paused!)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_zordi_cartesian_ik_controller,
                on_exit=[
                    TimerAction(
                        period=1.0,  # Wait for all controllers to be fully active
                        actions=[reset_to_test_pose],
                    )
                ],
            )
        ),
        # 2. After reset -> unpause (controllers active, robot at correct position)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_to_test_pose,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[unpause_simulation],
                    )
                ],
            )
        ),
        # 3. After unpause -> send test trajectory
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_simulation,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[send_test_trajectory],
                    )
                ],
            )
        ),
    ])

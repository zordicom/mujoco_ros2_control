"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for debugging Cartesian IK controller.

This test uses the simpler planar_2dof (no gravity) with ROS native JTC.

Test waypoints (FK-computed from joint configurations):
  q=[0.30, -0.20] -> EE at (0.9752, 0.1977, 0), quat(w=0.9988, z=0.0500)
  q=[0.35, -0.25] -> EE at (0.9672, 0.2214, 0), quat(w=0.9988, z=0.0500)
  q=[0.40, -0.30] -> EE at (0.9581, 0.2446, 0), quat(w=0.9988, z=0.0500)

Launch sequence:
  1. Start controller manager with planar_2dof robot
  2. Load controllers: joint_state_broadcaster, joint_trajectory_controller, IK
  3. Activate target JTC (joint_trajectory_controller)
  4. Activate IK controller (zordi_cartesian_ik_controller)
  5. Reset robot to home keyframe
  6. Unpause simulation
  7. Send test Cartesian trajectory (3 waypoints)

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
    """Generate automated launch description for IK controller testing."""
    pkg_share = Path(get_package_share_directory("mujoco_ros2_control_demos"))

    # Use planar_2dof (no gravity, simpler debugging)
    urdf_file = pkg_share / "urdf" / "planar_2dof.xacro.urdf"
    controller_config = pkg_share / "config" / "test_planar_2dof.yaml"

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

    # Load ROS native JTC (position interface)
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

    # Load IK controller (forwards to joint_trajectory_controller)
    load_zordi_cartesian_ik_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_cartesian_ik_controller",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    # === Step 2: Activate controllers (after they're loaded) ===
    activate_jtc = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "joint_trajectory_controller",
            "active",
        ],
        output="screen",
    )

    activate_ik_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "zordi_cartesian_ik_controller",
            "active",
        ],
        output="screen",
    )

    # === Step 3: Reset to home keyframe ===
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

    # === Step 4: Unpause simulation ===
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

    # === Step 5: Send test Cartesian trajectory ===
    # FK-computed waypoints from q=[0.30,-0.20], [0.35,-0.25], [0.40,-0.30]
    # All have same EE orientation (q1+q2 = 0.1 rad about Z)
    send_test_trajectory = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/cartesian_trajectory",
            "moveit_msgs/msg/CartesianTrajectory",
            """{
                header: {frame_id: 'base_link'},
                tracked_frame: 'ee_link',
                points: [
                    {
                        point: {
                            pose: {
                                position: {x: 0.9752, y: 0.1977, z: 0.0},
                                orientation: {w: 0.9988, x: 0.0, y: 0.0, z: 0.0500}
                            }
                        },
                        time_from_start: {sec: 2}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: 0.9672, y: 0.2214, z: 0.0},
                                orientation: {w: 0.9988, x: 0.0, y: 0.0, z: 0.0500}
                            }
                        },
                        time_from_start: {sec: 4}
                    },
                    {
                        point: {
                            pose: {
                                position: {x: 0.9581, y: 0.2446, z: 0.0},
                                orientation: {w: 0.9988, x: 0.0, y: 0.0, z: 0.0500}
                            }
                        },
                        time_from_start: {sec: 6}
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
        load_joint_trajectory_controller,
        load_zordi_cartesian_ik_controller,
        # Chain of events after controllers load:
        # 1. After IK controller loads -> activate target JTC
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_zordi_cartesian_ik_controller,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[activate_jtc],
                    )
                ],
            )
        ),
        # 2. After target JTC activates -> activate IK controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=activate_jtc,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[activate_ik_controller],
                    )
                ],
            )
        ),
        # 3. After IK controller activates -> reset to home
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=activate_ik_controller,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[reset_to_home],
                    )
                ],
            )
        ),
        # 4. After reset -> unpause simulation
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_to_home,
                on_exit=[
                    TimerAction(
                        period=0.5,
                        actions=[unpause_simulation],
                    )
                ],
            )
        ),
        # 5. After unpause -> send test trajectory
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_simulation,
                on_exit=[
                    TimerAction(
                        period=1.0,  # Wait for simulation to stabilize
                        actions=[send_test_trajectory],
                    )
                ],
            )
        ),
    ])

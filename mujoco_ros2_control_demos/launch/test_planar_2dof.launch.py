"""
Launch file for 2-DOF planar arm Cartesian controller demonstrations.

This test uses a HORIZONTAL 2-link planar arm (not a cartpole!) with two controllers:
  1. zordi_mit_controller: Gravity compensation and trajectory tracking
  2. zordi_cartesian_controller: Cartesian impedance control

Test objectives:
  Example 1 (zordi_mit_controller):
    - Verify gravity compensation in zero-gravity environment
    - Trajectory tracking in joint space
    - Stable operation without pause/unpause issues

  Example 2 (zordi_cartesian_controller):
    - Verify Cartesian impedance control
    - End-effector pose tracking
    - Note: May have initial transient due to MuJoCo pause/unpause bug

Expected behavior:
  - Zero gravity environment (no falling)
  - Robot starts at bent configuration q=[0.3, -0.2] (avoids singularity)
  - Controllers provide damping and tracking forces
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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
                "initial_keyframe": "home",  # Start at bent configuration q=[0.3, -0.2]
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

    return LaunchDescription(
        [
            mujoco_node,
            robot_state_pub_node,
            # Load controllers when mujoco node starts (standard mujoco_ros2_control pattern)
            # Both controllers are loaded in inactive state - activate manually as needed:
            #   - zordi_mit_controller: For gravity comp / trajectory tracking (Example 1)
            #   - zordi_cartesian_controller: For Cartesian impedance control (Example 2)
            # NOTE: MuJoCo pause/unpause transition may cause initial velocity perturbations
            # This is a known mujoco_ros2_control limitation, not a controller issue.
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=mujoco_node,
                    on_start=[
                        load_joint_state_broadcaster,
                        load_zordi_mit_controller,
                        load_zordi_cartesian_controller,
                    ],
                )
            ),
        ]
    )

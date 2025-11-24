"""Launch MuJoCo interactive viewer (visualization only)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mujoco_model_path", description="Full path to MuJoCo XML model file"
        ),
        DeclareLaunchArgument(
            "service_namespace",
            default_value="/mujoco_system",
            description="Namespace for simulation control services",
        ),
        # MuJoCo viewer node
        Node(
            package="mujoco_ros2_control",
            executable="mujoco_viewer",
            name="mujoco_viewer",
            parameters=[
                {
                    "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
                    "service_namespace": LaunchConfiguration("service_namespace"),
                }
            ],
            output="screen",
        ),
    ])

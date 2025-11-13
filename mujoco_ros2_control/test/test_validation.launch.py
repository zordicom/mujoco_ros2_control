import os
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = "/home/gilwoo/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test/test_robot_mismatch.urdf"
    mujoco_model_path = "/home/gilwoo/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test/test_robot_mismatch.xml"

    # Read URDF
    robot_description = pathlib.Path(urdf_path).read_text()

    return LaunchDescription([
        Node(
            package="mujoco_ros2_control",
            executable="mujoco_ros2_control",
            name="mujoco_ros2_control",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"mujoco_model_path": mujoco_model_path},
                {"headless": True},
                {"use_sim_time": True},
                {"update_rate": 1000},
            ],
        )
    ])

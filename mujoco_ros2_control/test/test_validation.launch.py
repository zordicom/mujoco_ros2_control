
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/gilwoo/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test/test_robot_mismatch.urdf'
    mujoco_model_path = '/home/gilwoo/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test/test_robot_mismatch.xml'

    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            name='mujoco_ros2_control',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'mujoco_model_path': mujoco_model_path},
                {'headless': True},
                # use_sim_time defaults to True (set in node)
                # update_rate auto-computed from MuJoCo timestep
            ],
        )
    ])

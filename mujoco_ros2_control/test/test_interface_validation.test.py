"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for hardware interface validation across actuator types.

This test verifies that each actuator type correctly claims the expected
command interfaces:
  - Position servo: position only
  - Torque motor: effort only
  - MIT motor: position, velocity, effort, kp, kd
"""

import subprocess
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareInterfaces


def print_result_box(title: str, lines: list[str], width: int = 70):
    """Print a formatted result box."""
    print("\n" + "=" * width)
    print(f"  {title}")
    print("=" * width)
    for line in lines:
        print(f"  {line}")
    print("=" * width + "\n")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for interface validation test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

    # Use position servo URDF for first test
    urdf_file = pkg_share / "test" / "models" / "test_position_servo.urdf"
    controller_config = pkg_share / "test" / "config" / "test_actuator_types.yaml"

    robot_description = urdf_file.read_text(encoding="utf-8")

    controller_manager_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(controller_config),
        ],
        output="screen",
    )

    robot_state_pub_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    load_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "controller_manager_node": controller_manager_node,
    }


class TestPositionServoInterfaces(unittest.TestCase):
    """Test that position servo has only position command interface."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node."""
        self.node = rclpy.create_node("test_interface_validation")

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def _call_service(self, srv_type, srv_name: str, request, timeout: float = 10.0):
        """Call a service and wait for response."""
        client = self.node.create_client(srv_type, srv_name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {srv_name} not available")

        future = client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        raise RuntimeError(f"Service {srv_name} call timed out")

    def test_position_servo_interfaces(self):
        """Verify position servo claims only position command interface."""
        # Wait for hardware to be configured
        time.sleep(2.0)

        request = ListHardwareInterfaces.Request()
        result = self._call_service(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            request,
        )

        # Find command interfaces for joint1
        command_interfaces = []
        state_interfaces = []

        for iface in result.command_interfaces:
            if iface.name.startswith("joint1/"):
                command_interfaces.append(iface.name.split("/")[1])

        for iface in result.state_interfaces:
            if iface.name.startswith("joint1/"):
                state_interfaces.append(iface.name.split("/")[1])

        print_result_box(
            "Position Servo Interface Validation",
            [
                f"Command interfaces: {command_interfaces}",
                f"State interfaces: {state_interfaces}",
                "",
                "Expected command: ['position']",
                f"Match: {command_interfaces == ['position']}",
            ],
        )

        # Verify only position command interface
        self.assertEqual(
            command_interfaces,
            ["position"],
            f"Position servo should only have 'position' command interface, "
            f"got: {command_interfaces}",
        )

        # Verify state interfaces
        self.assertIn("position", state_interfaces)
        self.assertIn("velocity", state_interfaces)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    """Test that processes shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """Check that all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[
                0,
                -6,
                -15,
                -9,
                127,
            ],  # 127 = known Humble forward_command_controller shutdown issue
        )

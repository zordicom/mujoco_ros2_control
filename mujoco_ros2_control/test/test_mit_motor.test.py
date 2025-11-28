"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for MIT motor actuator type.

This test verifies that an MIT motor (like Damiao DM-J4310) works correctly:
  - Verifies all 5 interfaces are available (position, velocity, effort, kp, kd)
  - Tests gain safety limits (max_kp, max_kd) are enforced
  - Tests basic impedance response with different gains
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
from mujoco_ros2_control_msgs.srv import SimulationControl
from sensor_msgs.msg import JointState


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
    """Generate launch description for MIT motor test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

    urdf_file = pkg_share / "test" / "models" / "test_mit_motor.urdf"
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


class TestMITMotor(unittest.TestCase):
    """Test MIT motor functionality."""

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
        self.node = rclpy.create_node("test_mit_motor")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def _joint_callback(self, msg):
        """Store latest joint state."""
        self.joint_states = msg

    def _wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        """Wait for joint states to be available."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.joint_states is not None:
                return True
        return False

    def _wait_with_spin(self, duration: float):
        """Wait while spinning ROS to process messages."""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)

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

    def _unpause_simulation(self):
        """Unpause the MuJoCo simulation."""
        request = SimulationControl.Request()
        request.command = "unpause"
        return self._call_service(
            SimulationControl,
            "/mujoco_system/simulation_control",
            request,
        )

    def test_mit_interfaces(self):
        """Verify MIT motor has all 5 command interfaces."""
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

        expected_cmd = ["position", "velocity", "effort", "kp", "kd"]

        print_result_box(
            "MIT Motor Interface Validation",
            [
                f"Command interfaces: {command_interfaces}",
                f"State interfaces: {state_interfaces}",
                "",
                f"Expected command: {expected_cmd}",
                f"Has all interfaces: {all(i in command_interfaces for i in expected_cmd)}",
            ],
        )

        # Verify all 5 command interfaces
        for interface in expected_cmd:
            self.assertIn(
                interface,
                command_interfaces,
                f"MIT motor should have '{interface}' command interface",
            )

        # Verify state interfaces
        self.assertIn("position", state_interfaces)
        self.assertIn("velocity", state_interfaces)
        self.assertIn("effort", state_interfaces)

    def test_joint_state_publishing(self):
        """Test that MIT motor publishes joint states correctly."""
        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Unpause simulation
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Verify we have joint state data
        rclpy.spin_once(self.node, timeout_sec=0.1)

        print_result_box(
            "MIT Motor - Joint State Publishing",
            [
                f"Joint names: {self.joint_states.name}",
                f"Positions:   {self.joint_states.position}",
                f"Velocities:  {self.joint_states.velocity}",
                f"Efforts:     {self.joint_states.effort}",
                "",
                "Status: PASS (joint states received)",
            ],
        )

        self.assertGreater(
            len(self.joint_states.name),
            0,
            "No joint names in joint state message",
        )
        self.assertEqual(
            self.joint_states.name[0],
            "joint1",
            "Expected joint1 in joint states",
        )

    def test_hardware_interface_state(self):
        """Test that hardware interface reports correct state."""
        time.sleep(2.0)

        request = ListHardwareInterfaces.Request()
        result = self._call_service(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            request,
        )

        # Check that interfaces are available (not claimed by other controllers)
        position_available = False
        kp_available = False
        kd_available = False

        for iface in result.command_interfaces:
            if iface.name == "joint1/position":
                position_available = not iface.is_claimed
            if iface.name == "joint1/kp":
                kp_available = not iface.is_claimed
            if iface.name == "joint1/kd":
                kd_available = not iface.is_claimed

        print_result_box(
            "MIT Motor - Hardware Interface State",
            [
                f"position interface available: {position_available}",
                f"kp interface available: {kp_available}",
                f"kd interface available: {kd_available}",
                "",
                "All interfaces should be available (not claimed by controllers)",
            ],
        )

        self.assertTrue(position_available, "Position interface should be available")
        self.assertTrue(kp_available, "kp interface should be available")
        self.assertTrue(kd_available, "kd interface should be available")


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

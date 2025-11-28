"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for position servo actuator type.

This test verifies that a position-only servo (like Dynamixel) works correctly:
  - Uses forward_command_controller to send position commands
  - Verifies joint moves to commanded position
  - Position actuator's internal PD handles tracking (no gravity comp needed)
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
from mujoco_ros2_control_msgs.srv import SimulationControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


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
    """Generate launch description for position servo test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

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

    load_forward_position_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        load_forward_position_controller,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "controller_manager_node": controller_manager_node,
    }


class TestPositionServo(unittest.TestCase):
    """Test position servo functionality."""

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
        self.node = rclpy.create_node("test_position_servo")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )
        self.cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
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

    def test_position_command_tracking(self):
        """Test that position servo tracks commanded position."""
        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Unpause simulation
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Get initial position
        initial_pos = self.joint_states.position[0]

        # Command a new position
        target_position = 0.5  # radians
        cmd = Float64MultiArray()
        cmd.data = [target_position]

        # Publish command multiple times to ensure it's received
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            self._wait_with_spin(0.1)

        # Wait for tracking
        self._wait_with_spin(3.0)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_pos = self.joint_states.position[0]

        # Calculate error
        error = abs(final_pos - target_position)
        tolerance = 0.1  # radians (~5.7 degrees)

        print_result_box(
            "Position Servo - Command Tracking",
            [
                f"Initial position: {initial_pos:.4f} rad",
                f"Target position:  {target_position:.4f} rad",
                f"Final position:   {final_pos:.4f} rad",
                f"Error:            {error:.4f} rad ({error * 57.3:.2f}°)",
                f"Tolerance:        {tolerance:.4f} rad",
                "",
                f"Status: {'PASS' if error < tolerance else 'FAIL'}",
            ],
        )

        self.assertLess(
            error,
            tolerance,
            f"Position servo tracking error {error:.4f} rad exceeds tolerance "
            f"{tolerance:.4f} rad",
        )

    def test_position_hold(self):
        """Test that position servo holds position against gravity."""
        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Unpause simulation
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Command horizontal position (against gravity)
        target_position = 1.57  # ~90 degrees (horizontal)
        cmd = Float64MultiArray()
        cmd.data = [target_position]

        for _ in range(10):
            self.cmd_pub.publish(cmd)
            self._wait_with_spin(0.1)

        # Wait for settling
        self._wait_with_spin(3.0)

        # Record position over time to check for drift
        positions = []
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            positions.append(self.joint_states.position[0])

        avg_position = sum(positions) / len(positions)
        position_variance = sum((p - avg_position) ** 2 for p in positions) / len(
            positions
        )
        max_drift = max(positions) - min(positions)

        print_result_box(
            "Position Servo - Hold Against Gravity",
            [
                f"Target position:  {target_position:.4f} rad",
                f"Average position: {avg_position:.4f} rad",
                f"Position drift:   {max_drift:.4f} rad",
                f"Variance:         {position_variance:.6f}",
                "",
                f"Status: {'PASS' if max_drift < 0.1 else 'FAIL'}",
            ],
        )

        # Verify stable hold (small drift)
        self.assertLess(
            max_drift,
            0.1,  # radians
            f"Position servo drifting: max drift = {max_drift:.4f} rad",
        )


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

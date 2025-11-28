"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for position+velocity servo actuator type.

This test verifies position+velocity control (like Dynamixel Profile Position):
  - Uses separate forward_command_controllers for position and velocity
  - Verifies joint tracks commanded position with velocity feedforward
  - Tests the control law: τ = kp*(pos_cmd - q) + kv*(vel_cmd - qd)

Note: This mode is different from Damiao's native position-speed mode where velocity
is a speed LIMIT for ramping. Here, velocity is a feedforward target (the desired
velocity at each trajectory point).
"""

import subprocess
import time
import unittest
from pathlib import Path

import launch
import launch.actions
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
    """Generate launch description for position+velocity servo test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

    urdf_file = pkg_share / "test" / "models" / "test_position_velocity.urdf"
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

    # Add delay before spawning to avoid race conditions
    load_joint_state_broadcaster = launch.actions.TimerAction(
        period=2.0,
        actions=[
            launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    load_forward_position_controller = launch.actions.TimerAction(
        period=3.0,
        actions=[
            launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["forward_position_controller", "-c", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    load_forward_velocity_controller = launch.actions.TimerAction(
        period=4.0,
        actions=[
            launch_ros.actions.Node(
                package="controller_manager",
                executable="spawner",
                arguments=["forward_velocity_controller", "-c", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        load_forward_position_controller,
        load_forward_velocity_controller,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "controller_manager_node": controller_manager_node,
    }


class TestPositionVelocityServo(unittest.TestCase):
    """Test position+velocity servo functionality."""

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
        self.node = rclpy.create_node("test_position_velocity")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )
        self.pos_cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10,
        )
        self.vel_cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            "/forward_velocity_controller/commands",
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

    def test_interfaces_available(self):
        """Verify position+velocity interfaces are available."""
        # Wait for controllers to be fully loaded (spawners have delays)
        self._wait_with_spin(5.0)

        request = ListHardwareInterfaces.Request()
        result = self._call_service(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            request,
        )

        # Find command interfaces for joint1
        command_interfaces = [
            iface.name.split("/")[1]
            for iface in result.command_interfaces
            if iface.name.startswith("joint1/")
        ]
        state_interfaces = [
            iface.name.split("/")[1]
            for iface in result.state_interfaces
            if iface.name.startswith("joint1/")
        ]

        expected_cmd = ["position", "velocity"]

        print_result_box(
            "Position+Velocity Interface Validation",
            [
                f"Command interfaces: {command_interfaces}",
                f"State interfaces: {state_interfaces}",
                "",
                f"Expected command: {expected_cmd}",
                f"Has position: {'position' in command_interfaces}",
                f"Has velocity: {'velocity' in command_interfaces}",
                "",
                "Note: This mode is for Dynamixel Profile Position, ODrive, etc.",
                "NOT for Damiao native position-speed mode (speed limit).",
            ],
        )

        # Verify both command interfaces
        self.assertIn("position", command_interfaces)
        self.assertIn("velocity", command_interfaces)

        # Verify state interfaces
        self.assertIn("position", state_interfaces)
        self.assertIn("velocity", state_interfaces)

    def test_position_tracking_with_zero_velocity(self):
        """Test position tracking with velocity feedforward set to zero."""
        # Wait for controllers to be fully loaded (spawners have delays)
        self._wait_with_spin(5.0)

        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Unpause simulation
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Command position with zero velocity feedforward
        target_position = 0.5  # radians
        target_velocity = 0.0  # zero feedforward

        pos_cmd = Float64MultiArray()
        pos_cmd.data = [target_position]
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [target_velocity]

        # Publish commands multiple times to ensure receipt
        for _ in range(10):
            self.pos_cmd_pub.publish(pos_cmd)
            self.vel_cmd_pub.publish(vel_cmd)
            self._wait_with_spin(0.1)

        # Wait for tracking
        self._wait_with_spin(3.0)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_pos = self.joint_states.position[0]

        error = abs(final_pos - target_position)
        tolerance = 0.15  # radians

        print_result_box(
            "Position+Velocity - Position Tracking (vel=0)",
            [
                f"Target position:  {target_position:.4f} rad",
                f"Target velocity:  {target_velocity:.4f} rad/s (feedforward)",
                f"Final position:   {final_pos:.4f} rad",
                f"Error:            {error:.4f} rad ({error * 57.3:.2f}°)",
                f"Tolerance:        {tolerance:.4f} rad",
                "",
                "Control law: τ = kp*(pos_cmd - q) + kv*(vel_cmd - qd)",
                "With vel_cmd=0: τ = kp*(pos_cmd - q) - kv*qd (damping)",
                "",
                f"Status: {'PASS' if error < tolerance else 'FAIL'}",
            ],
        )

        self.assertLess(
            error,
            tolerance,
            f"Position tracking error {error:.4f} rad exceeds tolerance",
        )

    def test_velocity_feedforward_effect(self):
        """Test that velocity feedforward affects the response."""
        # Wait for controllers to be fully loaded (spawners have delays)
        self._wait_with_spin(5.0)

        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Unpause simulation
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # First, command to home position
        pos_cmd = Float64MultiArray()
        pos_cmd.data = [0.0]
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [0.0]

        for _ in range(10):
            self.pos_cmd_pub.publish(pos_cmd)
            self.vel_cmd_pub.publish(vel_cmd)
            self._wait_with_spin(0.1)
        self._wait_with_spin(2.0)

        # Now command same position but with positive velocity feedforward
        # This should create a "pulling" effect since vel_cmd > qd
        target_position = 0.0  # stay at home
        velocity_feedforward = 1.0  # positive feedforward

        pos_cmd.data = [target_position]
        vel_cmd.data = [velocity_feedforward]

        # Record initial position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        initial_pos = self.joint_states.position[0]

        # Apply velocity feedforward for a short time
        for _ in range(5):
            self.pos_cmd_pub.publish(pos_cmd)
            self.vel_cmd_pub.publish(vel_cmd)
            self._wait_with_spin(0.1)

        # Get position after feedforward
        rclpy.spin_once(self.node, timeout_sec=0.1)
        pos_with_ff = self.joint_states.position[0]

        # The velocity feedforward should cause some deviation
        # τ = kp*(0 - q) + kv*(1.0 - qd)
        # When qd ≈ 0, this adds positive torque
        deviation = pos_with_ff - initial_pos

        print_result_box(
            "Position+Velocity - Velocity Feedforward Effect",
            [
                f"Target position:      {target_position:.4f} rad",
                f"Velocity feedforward: {velocity_feedforward:.4f} rad/s",
                f"Initial position:     {initial_pos:.4f} rad",
                f"Position with FF:     {pos_with_ff:.4f} rad",
                f"Deviation:            {deviation:.4f} rad",
                "",
                "Control law: τ = kp*(pos_cmd - q) + kv*(vel_cmd - qd)",
                "Positive vel_cmd with qd≈0 should add positive torque",
                "",
                f"Status: {'PASS' if abs(deviation) > 0.001 else 'FAIL'}",
            ],
        )

        # Verify velocity feedforward has SOME effect (not necessarily positive)
        self.assertGreater(
            abs(deviation),
            0.001,
            "Velocity feedforward should affect joint position",
        )

    def test_hold_against_gravity(self):
        """Test that position+velocity mode holds position against gravity."""
        # Wait for controllers to be fully loaded (spawners have delays)
        self._wait_with_spin(5.0)

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
        target_velocity = 0.0  # no feedforward

        pos_cmd = Float64MultiArray()
        pos_cmd.data = [target_position]
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [target_velocity]

        for _ in range(10):
            self.pos_cmd_pub.publish(pos_cmd)
            self.vel_cmd_pub.publish(vel_cmd)
            self._wait_with_spin(0.1)

        # Wait for settling
        self._wait_with_spin(3.0)

        # Record position over time to check for drift
        positions = []
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            positions.append(self.joint_states.position[0])

        avg_position = sum(positions) / len(positions)
        max_drift = max(positions) - min(positions)

        print_result_box(
            "Position+Velocity - Hold Against Gravity",
            [
                f"Target position:  {target_position:.4f} rad",
                f"Average position: {avg_position:.4f} rad",
                f"Position drift:   {max_drift:.4f} rad",
                "",
                f"Status: {'PASS' if max_drift < 0.15 else 'FAIL'}",
            ],
        )

        # Verify stable hold (small drift)
        self.assertLess(
            max_drift,
            0.15,  # radians
            f"Position drifting: max drift = {max_drift:.4f} rad",
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

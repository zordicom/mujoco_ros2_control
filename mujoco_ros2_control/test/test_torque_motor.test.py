"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for torque motor actuator type.

This test verifies that a pure torque-controlled motor (like Dynamixel Current Mode)
works correctly:
  - Uses forward_command_controller to send effort commands
  - Verifies gravity causes pendulum to fall with zero torque
  - Verifies applied torque can move the joint
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
from mujoco_ros2_control_msgs.srv import ResetToKeyframe, SimulationControl
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
    """Generate launch description for torque motor test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

    urdf_file = pkg_share / "test" / "models" / "test_torque_motor.urdf"
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

    load_forward_effort_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        load_forward_effort_controller,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "controller_manager_node": controller_manager_node,
    }


class TestTorqueMotor(unittest.TestCase):
    """Test torque motor functionality."""

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
        self.node = rclpy.create_node("test_torque_motor")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )
        self.cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            "/forward_effort_controller/commands",
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

    def _pause_simulation(self):
        """Pause the MuJoCo simulation."""
        request = SimulationControl.Request()
        request.command = "pause"
        return self._call_service(
            SimulationControl,
            "/mujoco_system/simulation_control",
            request,
        )

    def _reset_to_keyframe(self, keyframe: str):
        """Reset the MuJoCo simulation to a specific keyframe."""
        request = ResetToKeyframe.Request()
        request.keyframe = keyframe
        return self._call_service(
            ResetToKeyframe,
            "/mujoco_system/reset_to_keyframe",
            request,
        )

    def test_gravity_response(self):
        """Test that pendulum falls with zero torque (no internal PD)."""
        # Wait for joint states
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Pause simulation first
        self._pause_simulation()
        self._wait_with_spin(0.5)

        # Reset to tilted keyframe (qpos=0.5 rad = ~29 degrees)
        # This puts the pendulum at a non-equilibrium position so gravity will act
        self._reset_to_keyframe("tilted")
        self._wait_with_spin(0.5)

        # Get initial position (should be ~1.57 rad)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        initial_pos = self.joint_states.position[0]

        # Send zero torque command
        cmd = Float64MultiArray()
        cmd.data = [0.0]

        for _ in range(10):
            self.cmd_pub.publish(cmd)
            self._wait_with_spin(0.1)

        # Unpause simulation - pendulum should fall under gravity
        self._unpause_simulation()

        # Wait for gravity to act
        self._wait_with_spin(2.0)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_pos = self.joint_states.position[0]

        # Pendulum should have moved due to gravity (no internal PD to hold it)
        movement = abs(final_pos - initial_pos)

        print_result_box(
            "Torque Motor - Gravity Response",
            [
                f"Initial position: {initial_pos:.4f} rad (~{initial_pos * 57.3:.1f}°)",
                f"Final position:   {final_pos:.4f} rad (~{final_pos * 57.3:.1f}°)",
                f"Movement:         {movement:.4f} rad ({movement * 57.3:.2f}°)",
                "",
                "Expected: Pendulum should fall (no internal PD)",
                f"Status: {'PASS' if movement > 0.1 else 'FAIL'}",
            ],
        )

        self.assertGreater(
            movement,
            0.1,  # Should move at least 0.1 rad (~5.7 degrees)
            "Torque motor should allow gravity to move pendulum "
            f"(movement was only {movement:.4f} rad)",
        )

    def test_torque_response(self):
        """Test that applied torque moves the joint."""
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

        # Apply positive torque to lift against gravity
        torque = 5.0  # Nm (enough to overcome gravity)
        cmd = Float64MultiArray()
        cmd.data = [torque]

        for _ in range(20):
            self.cmd_pub.publish(cmd)
            self._wait_with_spin(0.1)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_pos = self.joint_states.position[0]

        # Calculate movement
        movement = final_pos - initial_pos

        print_result_box(
            "Torque Motor - Torque Response",
            [
                f"Applied torque:   {torque:.2f} Nm",
                f"Initial position: {initial_pos:.4f} rad",
                f"Final position:   {final_pos:.4f} rad",
                f"Movement:         {movement:.4f} rad ({movement * 57.3:.2f}°)",
                "",
                "Expected: Positive torque should cause positive movement",
                f"Status: {'PASS' if movement > 0.1 else 'FAIL'}",
            ],
        )

        self.assertGreater(
            movement,
            0.1,  # Should move at least 0.1 rad in positive direction
            f"Applied torque should move joint (movement was {movement:.4f} rad)",
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

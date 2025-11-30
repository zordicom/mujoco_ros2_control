"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for MIT motor actuator type.

This test verifies that an MIT motor (like Damiao DM-J4310) works correctly:
  - Verifies all 5 interfaces are available (position, velocity, effort, kp, kd)
  - Tests MIT mode with kp=kd=0 (torque passthrough - pendulum falls)
  - Tests MIT mode with non-zero kp/kd (impedance control - pendulum holds)
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

    # Spawn all 5 MIT mode controllers (order matters for interface claiming)
    # kp and kd must be claimed before effort+position/velocity
    load_kp_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_kp_controller", "-c", "/controller_manager"],
        output="screen",
    )

    load_kd_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_kd_controller", "-c", "/controller_manager"],
        output="screen",
    )

    load_position_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    load_velocity_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )

    load_effort_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        load_kp_controller,
        load_kd_controller,
        load_position_controller,
        load_velocity_controller,
        load_effort_controller,
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
        """Create test node and publishers."""
        self.node = rclpy.create_node("test_mit_motor")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )

        # Create publishers for all 5 MIT interfaces
        self.pos_pub = self.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        self.vel_pub = self.node.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10
        )
        self.effort_pub = self.node.create_publisher(
            Float64MultiArray, "/forward_effort_controller/commands", 10
        )
        self.kp_pub = self.node.create_publisher(
            Float64MultiArray, "/forward_kp_controller/commands", 10
        )
        self.kd_pub = self.node.create_publisher(
            Float64MultiArray, "/forward_kd_controller/commands", 10
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

    def _reset_simulation(self):
        """Reset the MuJoCo simulation to initial state."""
        request = SimulationControl.Request()
        request.command = "reset"
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

    def _pause_simulation(self):
        """Pause the MuJoCo simulation."""
        request = SimulationControl.Request()
        request.command = "pause"
        return self._call_service(
            SimulationControl,
            "/mujoco_system/simulation_control",
            request,
        )

    def _send_mit_command(
        self, pos: float, vel: float, effort: float, kp: float, kd: float
    ):
        """Send commands to all 5 MIT interfaces."""
        pos_msg = Float64MultiArray(data=[pos])
        vel_msg = Float64MultiArray(data=[vel])
        effort_msg = Float64MultiArray(data=[effort])
        kp_msg = Float64MultiArray(data=[kp])
        kd_msg = Float64MultiArray(data=[kd])

        self.pos_pub.publish(pos_msg)
        self.vel_pub.publish(vel_msg)
        self.effort_pub.publish(effort_msg)
        self.kp_pub.publish(kp_msg)
        self.kd_pub.publish(kd_msg)

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

    def test_mit_torque_passthrough(self):
        """Test MIT mode with kp=kd=0 (torque passthrough - pendulum falls)."""
        # Wait for controllers to be ready
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
        initial_position = self.joint_states.position[0]

        # Send MIT command with kp=kd=0 (pure torque passthrough, no effort)
        # With zero gains and zero feedforward, pendulum should fall freely
        self._send_mit_command(pos=0.0, vel=0.0, effort=0.0, kp=0.0, kd=0.0)
        self._wait_with_spin(0.2)

        # Unpause simulation - pendulum should fall under gravity
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_position = self.joint_states.position[0]
        position_change = abs(final_position - initial_position)

        print_result_box(
            "MIT Motor - Torque Passthrough (kp=kd=0)",
            [
                f"Initial position: {initial_position:.4f} rad (~{initial_position * 57.3:.1f}°)",
                f"Final position:   {final_position:.4f} rad (~{final_position * 57.3:.1f}°)",
                f"Position change:  {position_change:.4f} rad (~{position_change * 57.3:.1f}°)",
                "",
                "Expected: Pendulum falls under gravity (>0.1 rad change)",
                f"Status: {'PASS' if position_change > 0.1 else 'FAIL'}",
            ],
        )

        # Pendulum should have moved significantly under gravity
        self.assertGreater(
            position_change,
            0.1,
            "Pendulum should fall under gravity with kp=kd=0",
        )

    def test_mit_impedance_control(self):
        """Test MIT mode with non-zero kp/kd (impedance control - holds position)."""
        # Wait for controllers to be ready
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Pause simulation first
        self._pause_simulation()
        self._wait_with_spin(0.5)

        # Reset to tilted keyframe (0.5 rad = ~29 degrees)
        # This is a non-equilibrium position - gravity wants to pull it down
        self._reset_to_keyframe("tilted")
        self._wait_with_spin(0.5)

        # Get initial position (should be ~0.5 rad)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        initial_position = self.joint_states.position[0]
        target_position = 0.5  # Hold at tilted position

        # Send MIT command with kp=100, kd=10 (impedance control at tilted position)
        # This should hold the pendulum against gravity
        self._send_mit_command(
            pos=target_position, vel=0.0, effort=0.0, kp=100.0, kd=10.0
        )
        self._wait_with_spin(0.2)

        # Unpause simulation - pendulum should be held by impedance control
        self._unpause_simulation()
        self._wait_with_spin(1.0)

        # Get final position
        rclpy.spin_once(self.node, timeout_sec=0.1)
        final_position = self.joint_states.position[0]
        position_error = abs(final_position - target_position)

        print_result_box(
            "MIT Motor - Impedance Control (kp=100, kd=10)",
            [
                f"Target position:  {target_position:.4f} rad (~{target_position * 57.3:.1f}°)",
                f"Initial position: {initial_position:.4f} rad (~{initial_position * 57.3:.1f}°)",
                f"Final position:   {final_position:.4f} rad (~{final_position * 57.3:.1f}°)",
                f"Position error:   {position_error:.4f} rad (~{position_error * 57.3:.1f}°)",
                "",
                "Expected: Pendulum holds position against gravity (<0.1 rad error)",
                f"Status: {'PASS' if position_error < 0.1 else 'FAIL'}",
            ],
        )

        # Pendulum should stay near target with impedance control
        self.assertLess(
            position_error,
            0.1,
            "Pendulum should hold position with non-zero kp/kd",
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

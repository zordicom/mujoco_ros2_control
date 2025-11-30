"""
Copyright 2025 Zordi, Inc. All rights reserved.

Integration test for dual pendulum (bimanual-like) control.

This test verifies that two separate hardware interfaces can share a single
MuJoCo simulation and be controlled independently:
  - Left and right pendulums in the same physics world
  - Separate MIT mode controllers for each
  - Tests independent control and shared simulation state
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
    """Generate launch description for dual pendulum test."""
    subprocess.run(
        ["ros2", "daemon", "stop"], capture_output=True, timeout=5, check=False
    )

    pkg_share = Path(get_package_share_directory("mujoco_ros2_control"))

    urdf_file = pkg_share / "test" / "models" / "test_dual_pendulum.urdf"
    controller_config = pkg_share / "test" / "config" / "test_dual_pendulum.yaml"

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

    # Left pendulum controllers (all 5 MIT interfaces)
    load_left_kp = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_kp_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_left_kd = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_kd_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_left_position = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_position_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_left_velocity = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_left_effort = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_effort_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Right pendulum controllers (all 5 MIT interfaces)
    load_right_kp = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_kp_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_right_kd = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_kd_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_right_position = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_position_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_right_velocity = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_velocity_controller", "-c", "/controller_manager"],
        output="screen",
    )
    load_right_effort = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_effort_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return launch.LaunchDescription([
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        # Left controllers
        load_left_kp,
        load_left_kd,
        load_left_position,
        load_left_velocity,
        load_left_effort,
        # Right controllers
        load_right_kp,
        load_right_kd,
        load_right_position,
        load_right_velocity,
        load_right_effort,
        launch_testing.actions.ReadyToTest(),
    ]), {
        "controller_manager_node": controller_manager_node,
    }


class TestDualPendulum(unittest.TestCase):
    """Test dual pendulum (bimanual-like) control."""

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
        self.node = rclpy.create_node("test_dual_pendulum")
        self.joint_states = None
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_callback,
            10,
        )

        # Left pendulum publishers
        self.left_pos_pub = self.node.create_publisher(
            Float64MultiArray, "/left_position_controller/commands", 10
        )
        self.left_vel_pub = self.node.create_publisher(
            Float64MultiArray, "/left_velocity_controller/commands", 10
        )
        self.left_effort_pub = self.node.create_publisher(
            Float64MultiArray, "/left_effort_controller/commands", 10
        )
        self.left_kp_pub = self.node.create_publisher(
            Float64MultiArray, "/left_kp_controller/commands", 10
        )
        self.left_kd_pub = self.node.create_publisher(
            Float64MultiArray, "/left_kd_controller/commands", 10
        )

        # Right pendulum publishers
        self.right_pos_pub = self.node.create_publisher(
            Float64MultiArray, "/right_position_controller/commands", 10
        )
        self.right_vel_pub = self.node.create_publisher(
            Float64MultiArray, "/right_velocity_controller/commands", 10
        )
        self.right_effort_pub = self.node.create_publisher(
            Float64MultiArray, "/right_effort_controller/commands", 10
        )
        self.right_kp_pub = self.node.create_publisher(
            Float64MultiArray, "/right_kp_controller/commands", 10
        )
        self.right_kd_pub = self.node.create_publisher(
            Float64MultiArray, "/right_kd_controller/commands", 10
        )

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def _joint_callback(self, msg):
        """Store latest joint state."""
        self.joint_states = msg

    def _get_joint_position(self, joint_name: str) -> float:
        """Get position of a specific joint."""
        if self.joint_states is None:
            return 0.0
        try:
            idx = self.joint_states.name.index(joint_name)
            return self.joint_states.position[idx]
        except ValueError:
            return 0.0

    def _wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        """Wait for joint states to be available."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.joint_states is not None and len(self.joint_states.name) >= 2:
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

    def _send_left_mit_command(
        self, pos: float, vel: float, effort: float, kp: float, kd: float
    ):
        """Send MIT command to left pendulum."""
        self.left_pos_pub.publish(Float64MultiArray(data=[pos]))
        self.left_vel_pub.publish(Float64MultiArray(data=[vel]))
        self.left_effort_pub.publish(Float64MultiArray(data=[effort]))
        self.left_kp_pub.publish(Float64MultiArray(data=[kp]))
        self.left_kd_pub.publish(Float64MultiArray(data=[kd]))

    def _send_right_mit_command(
        self, pos: float, vel: float, effort: float, kp: float, kd: float
    ):
        """Send MIT command to right pendulum."""
        self.right_pos_pub.publish(Float64MultiArray(data=[pos]))
        self.right_vel_pub.publish(Float64MultiArray(data=[vel]))
        self.right_effort_pub.publish(Float64MultiArray(data=[effort]))
        self.right_kp_pub.publish(Float64MultiArray(data=[kp]))
        self.right_kd_pub.publish(Float64MultiArray(data=[kd]))

    def test_both_pendulums_visible(self):
        """Verify both pendulums are visible in joint states."""
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        joint_names = self.joint_states.name
        has_left = "left_joint1" in joint_names
        has_right = "right_joint1" in joint_names

        print_result_box(
            "Dual Pendulum - Joint Visibility",
            [
                f"Joint names: {list(joint_names)}",
                f"Has left_joint1: {has_left}",
                f"Has right_joint1: {has_right}",
                "",
                f"Status: {'PASS' if has_left and has_right else 'FAIL'}",
            ],
        )

        self.assertTrue(has_left, "left_joint1 not found in joint states")
        self.assertTrue(has_right, "right_joint1 not found in joint states")

    def test_independent_control(self):
        """Test that left and right pendulums can be controlled independently."""
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Pause and reset to asymmetric keyframe
        self._pause_simulation()
        self._wait_with_spin(0.5)
        self._reset_to_keyframe("asymmetric")
        self._wait_with_spin(0.5)

        # Get initial positions
        rclpy.spin_once(self.node, timeout_sec=0.1)
        left_initial = self._get_joint_position("left_joint1")
        right_initial = self._get_joint_position("right_joint1")

        # Command left to hold at 0.5, right to hold at -0.5
        left_target = 0.5
        right_target = -0.5

        self._send_left_mit_command(
            pos=left_target, vel=0.0, effort=0.0, kp=100.0, kd=10.0
        )
        self._send_right_mit_command(
            pos=right_target, vel=0.0, effort=0.0, kp=100.0, kd=10.0
        )
        self._wait_with_spin(0.2)

        # Unpause and let controllers work
        self._unpause_simulation()
        self._wait_with_spin(1.5)

        # Get final positions
        rclpy.spin_once(self.node, timeout_sec=0.1)
        left_final = self._get_joint_position("left_joint1")
        right_final = self._get_joint_position("right_joint1")

        left_error = abs(left_final - left_target)
        right_error = abs(right_final - right_target)

        print_result_box(
            "Dual Pendulum - Independent Control",
            [
                f"Left:  initial={left_initial:.3f}, target={left_target:.3f}, "
                f"final={left_final:.3f}, error={left_error:.3f}",
                f"Right: initial={right_initial:.3f}, target={right_target:.3f}, "
                f"final={right_final:.3f}, error={right_error:.3f}",
                "",
                "Expected: Both pendulums reach their targets (<0.1 rad error)",
                f"Status: {'PASS' if left_error < 0.1 and right_error < 0.1 else 'FAIL'}",
            ],
        )

        self.assertLess(left_error, 0.1, f"Left pendulum error too large: {left_error}")
        self.assertLess(
            right_error, 0.1, f"Right pendulum error too large: {right_error}"
        )

    def test_asymmetric_gains(self):
        """Test different gains on left vs right (stiff left, compliant right)."""
        self.assertTrue(
            self._wait_for_joint_states(timeout=30.0),
            "Joint states not received",
        )

        # Pause and reset to tilted keyframe
        self._pause_simulation()
        self._wait_with_spin(0.5)
        self._reset_to_keyframe("tilted")
        self._wait_with_spin(0.5)

        # Get initial positions (both at 0.5)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        left_initial = self._get_joint_position("left_joint1")
        right_initial = self._get_joint_position("right_joint1")

        # Left: high stiffness (should hold position)
        # Right: zero stiffness (should fall under gravity)
        # Send commands multiple times to ensure they're received
        for _ in range(5):
            self._send_left_mit_command(pos=0.5, vel=0.0, effort=0.0, kp=200.0, kd=20.0)
            self._send_right_mit_command(pos=0.5, vel=0.0, effort=0.0, kp=0.0, kd=0.0)
            self._wait_with_spin(0.1)

        # Unpause and let physics work
        self._unpause_simulation()

        # Keep sending commands while simulation runs
        for _ in range(15):
            self._send_left_mit_command(pos=0.5, vel=0.0, effort=0.0, kp=200.0, kd=20.0)
            self._send_right_mit_command(pos=0.5, vel=0.0, effort=0.0, kp=0.0, kd=0.0)
            self._wait_with_spin(0.1)

        # Get final positions
        rclpy.spin_once(self.node, timeout_sec=0.1)
        left_final = self._get_joint_position("left_joint1")
        right_final = self._get_joint_position("right_joint1")

        left_drift = abs(left_final - left_initial)
        right_drift = abs(right_final - right_initial)

        print_result_box(
            "Dual Pendulum - Asymmetric Gains",
            [
                f"Left (kp=200):  initial={left_initial:.3f}, final={left_final:.3f}, "
                f"drift={left_drift:.3f}",
                f"Right (kp=0):   initial={right_initial:.3f}, final={right_final:.3f}, "
                f"drift={right_drift:.3f}",
                "",
                "Expected: Left holds (<0.1), Right falls (>0.1)",
                f"Status: {'PASS' if left_drift < 0.1 and right_drift > 0.1 else 'FAIL'}",
            ],
        )

        self.assertLess(left_drift, 0.1, "Stiff left pendulum should hold position")
        self.assertGreater(right_drift, 0.1, "Compliant right pendulum should fall")


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

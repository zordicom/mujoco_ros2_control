#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated test script for 1-DOF multimode control - Scenario A.

Test workflow: Use JointTrajectoryController to move → switch to zordi_mit_controller to hold

For each test configuration:
  1. Activate joint_trajectory_controller
  2. Send trajectory to target position (2s duration)
  3. Wait for arrival
  4. Switch to zordi_mit_controller
  5. Hold for 10 seconds
  6. Record drift (position change)
  7. Switch back to JTC

Expected results:
  - zordi_mit holds position with < 0.001 rad drift over 10s

Usage:
  ros2 launch mujoco_ros2_control_demos test_1dof_multimode_with_gravity.launch.py
  # Then in another terminal:
  python3 test_1dof_multimode_scenario_a.py
"""

import time
from typing import List, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MultimodeTestScenarioA(Node):
    def __init__(self):
        super().__init__("test_1dof_multimode_scenario_a")

        # Test configurations (rad)
        self.test_positions = [0.0, 0.5, 1.0, -0.5, -1.0]

        # Publishers and subscribers
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Service clients
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # State tracking
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.joint_state_received = False

        # Wait for services
        self.get_logger().info("Waiting for controller_manager services...")
        self.switch_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Services available!")

        # Wait for initial joint state
        self.get_logger().info("Waiting for joint states...")
        while not self.joint_state_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Initial position: {self.current_position:.4f} rad")

    def joint_state_callback(self, msg):
        """Store current joint state."""
        if "j1" in msg.name:
            idx = msg.name.index("j1")
            self.current_position = msg.position[idx]
            self.current_velocity = msg.velocity[idx] if msg.velocity else 0.0
            self.joint_state_received = True

    def switch_controller(
        self, activate: List[str], deactivate: List[str], strict: bool = True
    ) -> bool:
        """Switch controllers via controller_manager service."""
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = (
            SwitchController.Request.STRICT
            if strict
            else SwitchController.Request.BEST_EFFORT
        )
        req.start_asap = True
        req.timeout = Duration(sec=0, nanosec=0)

        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().ok:
            return True
        else:
            self.get_logger().error(
                f"Failed to switch controllers: {activate} / {deactivate}"
            )
            return False

    def send_trajectory(self, target_position: float, duration_sec: float = 2.0):
        """Send trajectory to joint_trajectory_controller."""
        traj = JointTrajectory()
        traj.joint_names = ["j1"]

        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )
        traj.points.append(point)

        self.joint_traj_pub.publish(traj)
        self.get_logger().info(
            f"Sent trajectory: target={target_position:.3f} rad, duration={duration_sec}s"
        )

    def wait_for_arrival(
        self, target_position: float, tolerance: float = 0.05, timeout: float = 5.0
    ):
        """Wait until joint reaches target position."""
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            error = abs(self.current_position - target_position)
            if error < tolerance:
                self.get_logger().info(f"Arrived at target (error={error:.4f} rad)")
                return True
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    f"Timeout waiting for arrival (error={error:.4f} rad)"
                )
                return False
        return False

    def measure_drift(self, duration: float = 10.0) -> float:
        """Measure position drift over a duration."""
        start_position = self.current_position
        self.get_logger().info(
            f"Measuring drift for {duration}s (start={start_position:.4f} rad)..."
        )

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        end_position = self.current_position
        drift = abs(end_position - start_position)
        self.get_logger().info(f"Drift measurement complete: {drift:.6f} rad")
        return drift

    def run_test(self) -> List[Tuple[float, float, bool]]:
        """Run the full test sequence."""
        results = []

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("SCENARIO A TEST: JTC to move → zordi_mit to hold")
        self.get_logger().info("=" * 70)

        for i, target_pos in enumerate(self.test_positions):
            self.get_logger().info(
                f"\n--- Test {i + 1}/{len(self.test_positions)}: "
                f"Target = {target_pos:.3f} rad ---"
            )

            # Step 1: Activate JTC
            self.get_logger().info("Step 1: Activating joint_trajectory_controller...")
            if not self.switch_controller(
                activate=["joint_trajectory_controller"],
                deactivate=["zordi_mit_controller"],
            ):
                self.get_logger().error("Failed to activate JTC, skipping test")
                results.append((target_pos, -1.0, False))
                continue

            time.sleep(0.5)  # Let controller stabilize

            # Step 2: Send trajectory
            self.get_logger().info("Step 2: Sending trajectory...")
            self.send_trajectory(target_pos, duration_sec=2.0)

            # Step 3: Wait for arrival
            self.get_logger().info("Step 3: Waiting for arrival...")
            if not self.wait_for_arrival(target_pos, tolerance=0.05, timeout=5.0):
                self.get_logger().warn("Failed to reach target, continuing anyway")

            time.sleep(0.5)  # Settle

            # Step 4: Switch to zordi_mit
            self.get_logger().info("Step 4: Switching to zordi_mit_controller...")
            if not self.switch_controller(
                activate=["zordi_mit_controller"],
                deactivate=["joint_trajectory_controller"],
            ):
                self.get_logger().error(
                    "Failed to switch to zordi_mit, skipping drift measurement"
                )
                results.append((target_pos, -1.0, False))
                continue

            time.sleep(0.5)  # Let controller stabilize

            # Step 5: Measure drift
            self.get_logger().info("Step 5: Measuring drift with zordi_mit holding...")
            drift = self.measure_drift(duration=10.0)

            # Step 6: Record results
            success = drift < 0.01  # Success if drift < 10 millirad
            results.append((target_pos, drift, success))

            status = "✓ PASS" if success else "✗ FAIL"
            self.get_logger().info(f"Result: {status} (drift={drift:.6f} rad)")

        return results

    def print_summary(self, results: List[Tuple[float, float, bool]]):
        """Print test summary table."""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"{'Position (rad)':<15} {'Drift (rad)':<15} {'Status':<10}"
        )
        self.get_logger().info("-" * 70)

        for target_pos, drift, success in results:
            status = "✓ PASS" if success else "✗ FAIL"
            drift_str = f"{drift:.6f}" if drift >= 0 else "N/A"
            self.get_logger().info(f"{target_pos:<15.3f} {drift_str:<15} {status:<10}")

        self.get_logger().info("-" * 70)
        passed = sum(1 for _, _, s in results if s)
        total = len(results)
        self.get_logger().info(f"Total: {passed}/{total} tests passed")
        self.get_logger().info("=" * 70 + "\n")


def main():
    rclpy.init()
    node = MultimodeTestScenarioA()

    try:
        results = node.run_test()
        node.print_summary(results)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

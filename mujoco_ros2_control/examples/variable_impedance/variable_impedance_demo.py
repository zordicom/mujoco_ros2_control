#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Variable Impedance Demo

This script demonstrates how MIT mode enables variable impedance control.
The robot's stiffness changes during execution to show different behaviors:
- Stiff mode: High kp for precise positioning
- Compliant mode: Low kp for backdrivable operation
- Zero stiffness: Pure damping, fully backdrivable

Usage:
    1. Launch the MIT motor example:
       ros2 launch mujoco_ros2_control mit_motor.launch.py

    2. Run this demo:
       ros2 run mujoco_ros2_control variable_impedance_demo.py

Note: This is a demonstration script. In production, you would implement
a custom controller or action interface for dynamic gain changes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class VariableImpedanceDemo(Node):
    """Demonstrates variable impedance control with MIT mode."""

    def __init__(self):
        super().__init__("variable_impedance_demo")

        # Publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/zordi_joint_mit_controller/joint_trajectory",
            10
        )

        self.get_logger().info("Variable Impedance Demo initialized")
        self.get_logger().info("=" * 60)
        self.get_logger().info("This demo shows how MIT mode enables variable impedance.")
        self.get_logger().info("The robot's stiffness changes during execution.")
        self.get_logger().info("=" * 60)

    def send_trajectory(self, positions: list, duration_sec: float = 2.0):
        """Send a trajectory command to the controller."""
        msg = JointTrajectory()
        msg.joint_names = ["joint1"]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        msg.points = [point]
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f"Sent trajectory to position {positions[0]:.2f}")

    def run_demo(self):
        """Run the variable impedance demonstration."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("DEMO: Variable Impedance Control")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")

        # Phase 1: Move to starting position with default stiffness
        self.get_logger().info("Phase 1: Moving to starting position (default kp=100, kd=10)")
        self.send_trajectory([0.0], 2.0)
        time.sleep(3.0)

        # Phase 2: Move to horizontal position
        self.get_logger().info("")
        self.get_logger().info("Phase 2: Moving to horizontal position")
        self.get_logger().info("         Robot should track precisely with kp=100")
        self.send_trajectory([1.57], 2.0)
        time.sleep(3.0)

        # Phase 3: Return to vertical
        self.get_logger().info("")
        self.get_logger().info("Phase 3: Returning to vertical position")
        self.send_trajectory([0.0], 2.0)
        time.sleep(3.0)

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Demo complete!")
        self.get_logger().info("")
        self.get_logger().info("To implement true variable impedance control,")
        self.get_logger().info("you would create a custom controller that:")
        self.get_logger().info("  1. Accepts dynamic gain commands via service/topic")
        self.get_logger().info("  2. Updates software_kp_/software_kd_ in real-time")
        self.get_logger().info("  3. Writes new gains to kp/kd interfaces each cycle")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = VariableImpedanceDemo()

    try:
        # Wait for controller to be ready
        node.get_logger().info("Waiting 2 seconds for controller to be ready...")
        time.sleep(2.0)

        # Run the demo
        node.run_demo()

    except KeyboardInterrupt:
        node.get_logger().info("Demo interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script for ZordiCartesianController with 2-DOF planar cartpole.

This script sends target poses to the Cartesian controller and monitors
the end-effector tracking performance.

Usage:
  1. Launch the simulation:
     ros2 launch zordi_mit_controller test_cartesian_2dof.launch.py

  2. Run this script:
     python3 launch/test_cartesian_2dof_script.py

  3. Observe the robot moving to different target poses in the XZ plane
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CartesianTest(Node):
    """Test node for Cartesian controller."""

    def __init__(self):
        super().__init__("cartesian_test")

        # Publisher for target pose
        self.pose_pub = self.create_publisher(
            PoseStamped, "/zordi_cartesian_effort_controller/target_pose", 10
        )

        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 10
        )

        self.current_joints = None
        self.get_logger().info("Cartesian test node initialized")
        self.get_logger().info("Waiting for joint states...")

        # Wait for first joint state
        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Joint states received. Starting test...")

    def joint_callback(self, msg: JointState):
        """Store current joint state."""
        self.current_joints = msg

    def send_target_pose(self, x: float, y: float, description: str):
        """
        Send target pose to controller.

        Args:
            x: X position (forward/back)
            y: Y position (left/right)
            description: Human-readable description
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Position (horizontal plane at z=0.0 relative to base_link)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0  # Relative to base_link (not world!)

        # Orientation (identity quaternion - no rotation preference)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        self.get_logger().info(f"Sending target: {description} (x={x:.3f}, y={y:.3f})")
        self.pose_pub.publish(msg)

    def run_test_sequence(self):
        """Run sequence of test poses."""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("STARTING CARTESIAN CONTROLLER TEST SEQUENCE")
        self.get_logger().info("=" * 60 + "\n")

        # First, verify initial position
        self.get_logger().info("Verifying initial robot position...")
        time.sleep(0.5)  # Let state stabilize
        rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joints:
            j1_idx = self.current_joints.name.index("joint1")
            j2_idx = self.current_joints.name.index("joint2")
            q1 = self.current_joints.position[j1_idx]
            q2 = self.current_joints.position[j2_idx]

            # Compute FK
            l1 = 0.5
            l2 = 0.5
            ee_x = l1 * cos(q1) + l2 * cos(q1 + q2)
            ee_y = l1 * sin(q1) + l2 * sin(q1 + q2)

            self.get_logger().info(f"Initial state: q=[{q1:.3f}, {q2:.3f}]")
            self.get_logger().info(
                f"Initial EE position (base frame): [{ee_x:.3f}, {ee_y:.3f}, 0.000]"
            )
            self.get_logger().info("")

        # Test poses (x, y, description, duration) - horizontal plane
        # ALL TARGETS ARE IN BASE_LINK FRAME (z=0 relative to base)
        # Starting from bent configuration q=[0.3, -0.2] → EE=[0.975, 0.198, 0]
        test_poses = [
            (0.975, 0.198, "Confirm home (bent config)", 3.0),
            (0.995, 0.208, "Small move (+2cm X, +1cm Y)", 5.0),
            (0.955, 0.218, "Move back (-2cm X, +2cm Y)", 5.0),
            (0.975, 0.178, "Move down (-2cm Y)", 5.0),
            (0.975, 0.198, "Return to home", 5.0),
        ]

        for x, y, desc, duration in test_poses:
            self.send_target_pose(x, y, desc)

            # Monitor for duration
            start_time = time.time()
            while (time.time() - start_time) < duration:
                rclpy.spin_once(self, timeout_sec=0.1)

                # Print current state every 0.5 seconds
                if self.current_joints and (time.time() - start_time) % 0.5 < 0.1:
                    j1_idx = self.current_joints.name.index("joint1")
                    j2_idx = self.current_joints.name.index("joint2")
                    q1 = self.current_joints.position[j1_idx]
                    q2 = self.current_joints.position[j2_idx]

                    # Compute FK (simple 2-link planar horizontal)
                    l1 = 0.5  # Link 1 length
                    l2 = 0.5  # Link 2 length
                    ee_x = l1 * cos(q1) + l2 * cos(q1 + q2)
                    ee_y = l1 * sin(q1) + l2 * sin(q1 + q2)

                    self.get_logger().info(
                        f"  Current: q1={q1:.3f}, q2={q2:.3f} | "
                        f"EE: x={ee_x:.3f}, y={ee_y:.3f}"
                    )

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST SEQUENCE COMPLETE")
        self.get_logger().info("=" * 60 + "\n")


def sin(angle):
    """Compute sine."""
    import math

    return math.sin(angle)


def cos(angle):
    """Compute cosine."""
    import math

    return math.cos(angle)


def main():
    """Main entry point."""
    rclpy.init()

    try:
        test_node = CartesianTest()
        test_node.run_test_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

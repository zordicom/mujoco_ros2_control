#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script to verify URDF/MuJoCo actuator validation.

This test intentionally creates a mismatch between URDF command interfaces
and MuJoCo actuators to verify that the validation catches the error.
"""

import subprocess
import sys
import tempfile
from pathlib import Path


def create_test_launch_file(test_dir: Path) -> Path:
    """Create a minimal launch file for testing."""
    launch_content = f"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '{test_dir / "test_robot_mismatch.urdf"}'
    mujoco_model_path = '{test_dir / "test_robot_mismatch.xml"}'

    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            name='mujoco_ros2_control',
            output='screen',
            parameters=[
                {{'robot_description': robot_description}},
                {{'mujoco_model_path': mujoco_model_path}},
                {{'headless': True}},
                # use_sim_time defaults to True (set in node)
                # update_rate auto-computed from MuJoCo timestep
            ],
        )
    ])
"""

    launch_file = test_dir / "test_validation.launch.py"
    launch_file.write_text(launch_content)
    return launch_file


def run_validation_test(test_dir: Path) -> bool:
    """
    Run the validation test.

    Returns:
        True if validation error was correctly detected, False otherwise.
    """
    print("=" * 80)
    print("MuJoCo ROS2 Control - Interface Validation Test")
    print("=" * 80)
    print()
    print("Test Setup:")
    print(f"  URDF: {test_dir / 'test_robot_mismatch.urdf'}")
    print(f"  MuJoCo XML: {test_dir / 'test_robot_mismatch.xml'}")
    print()
    print("Expected Behavior:")
    print("  - URDF declares: position, velocity, effort interfaces")
    print("  - MuJoCo XML defines: act_pos_*, act_vel_* actuators only")
    print("  - Missing: act_tau_* (torque actuator)")
    print("  - Expected result: Initialization should FAIL with validation error")
    print()
    print("-" * 80)
    print()

    # Create launch file
    launch_file = create_test_launch_file(test_dir)

    # Run the launch file and capture output
    cmd = ["ros2", "launch", str(launch_file)]

    print(f"Running: {' '.join(cmd)}")
    print()

    try:
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,  # Should fail quickly
        )

        output = result.stdout + result.stderr

        # Check if validation error was raised
        validation_error_found = (
            "declares effort interface in URDF but no 'act_tau_" in output
            or "URDF/MuJoCo mismatch: effort interface without actuator" in output
        )

        print("Process output:")
        print("-" * 80)
        print(output)
        print("-" * 80)
        print()

        if validation_error_found:
            print("✅ TEST PASSED: Validation error correctly detected!")
            print()
            print("The system correctly identified that:")
            print("  - URDF declares 'effort' interface for test_joint1")
            print("  - MuJoCo XML is missing 'act_tau_test_joint1' actuator")
            print("  - Initialization failed with explicit error message")
            return True
        else:
            print("❌ TEST FAILED: Validation error not detected!")
            print()
            print("Expected to see error message about missing effort actuator,")
            print("but it was not found in the output.")
            return False

    except subprocess.TimeoutExpired:
        print("❌ TEST FAILED: Process timed out (should fail quickly)")
        return False
    except FileNotFoundError:
        print("❌ TEST FAILED: 'ros2' command not found. Is ROS2 sourced?")
        print("   Run: source /opt/ros/humble/setup.bash")
        return False


def main():
    """Run the validation test."""
    # Get test directory (where this script is located)
    test_dir = Path(__file__).parent.absolute()

    # Verify test files exist
    urdf_file = test_dir / "test_robot_mismatch.urdf"
    xml_file = test_dir / "test_robot_mismatch.xml"

    if not urdf_file.exists():
        print(f"❌ ERROR: URDF file not found: {urdf_file}")
        return 1

    if not xml_file.exists():
        print(f"❌ ERROR: MuJoCo XML file not found: {xml_file}")
        return 1

    # Run test
    success = run_validation_test(test_dir)

    print()
    print("=" * 80)
    if success:
        print("VALIDATION TEST RESULT: PASSED ✅")
        return 0
    else:
        print("VALIDATION TEST RESULT: FAILED ❌")
        return 1


if __name__ == "__main__":
    sys.exit(main())

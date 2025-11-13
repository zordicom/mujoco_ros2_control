#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script to verify non-zero kv warning in position actuators.

This test verifies that the system warns when:
1. Position actuator has kv != 0 in MuJoCo model
2. Position interface is exposed in URDF
3. Position interface is not actively used (startup or MIT mode)
"""

import subprocess
import sys
import time
from pathlib import Path


def create_test_launch_file(test_dir: Path) -> Path:
    """Create a minimal launch file for testing."""
    launch_content = f"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '{test_dir / "test_robot_nonzero_kv.urdf"}'
    mujoco_model_path = '{test_dir / "test_robot_nonzero_kv.xml"}'

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
                {{'use_sim_time': True}},
                {{'update_rate': 1000}},
            ],
        )
    ])
"""

    launch_file = test_dir / "test_kv_warning.launch.py"
    launch_file.write_text(launch_content)
    return launch_file


def run_kv_warning_test(test_dir: Path) -> bool:
    """
    Run the kv warning test.

    Returns:
        True if warning was correctly emitted, False otherwise.
    """
    print("=" * 80)
    print("MuJoCo ROS2 Control - Non-Zero kv Warning Test")
    print("=" * 80)
    print()
    print("Test Setup:")
    print(f"  URDF: {test_dir / 'test_robot_nonzero_kv.urdf'}")
    print(f"  MuJoCo XML: {test_dir / 'test_robot_nonzero_kv.xml'}")
    print()
    print("Configuration:")
    print("  - URDF exposes: position, velocity, effort interfaces")
    print("  - MuJoCo position actuator: kp=20.0, kv=5.0 (NON-ZERO)")
    print("  - No controller activated (position interface exposed but not active)")
    print()
    print("Expected Behavior:")
    print("  - System should initialize successfully")
    print("  - Warning should be emitted about non-zero kv:")
    print("    'Position actuator has kv=5.0 (non-zero damping)'")
    print()
    print("-" * 80)
    print()

    # Create launch file
    launch_file = create_test_launch_file(test_dir)

    # Run the launch file and capture output
    cmd = ["ros2", "launch", str(launch_file)]

    print(f"Running: {' '.join(cmd)}")
    print("(Will run for 5 seconds to capture initialization logs)")
    print()

    try:
        # Run for a few seconds to capture initialization
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # Wait for initialization
        time.sleep(5)

        # Terminate process
        process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()

        output = stdout + stderr

        # Check for the warning message
        warning_found = (
            "Position actuator has kv=" in output and
            "non-zero damping" in output and
            "test_joint1" in output
        )

        # Also check that it initialized successfully (no fatal error)
        initialization_failed = (
            "FATAL" in output or
            "Failed to initialize" in output or
            "Could not initialize" in output
        )

        print("Process output:")
        print("-" * 80)
        print(output)
        print("-" * 80)
        print()

        if initialization_failed:
            print("❌ TEST FAILED: System failed to initialize!")
            print()
            print("The system should initialize successfully and only emit a warning,")
            print("not fail initialization.")
            return False

        if warning_found:
            print("✅ TEST PASSED: Non-zero kv warning correctly emitted!")
            print()
            print("The system correctly:")
            print("  - Initialized successfully (no fatal errors)")
            print("  - Detected position actuator with kv=5.0")
            print("  - Emitted warning about potential unwanted damping")
            return True
        else:
            print("❌ TEST FAILED: Warning not detected!")
            print()
            print("Expected to see warning message about non-zero kv,")
            print("but it was not found in the output.")
            return False

    except FileNotFoundError:
        print("❌ TEST FAILED: 'ros2' command not found. Is ROS2 sourced?")
        print("   Run: source /opt/ros/humble/setup.bash")
        return False
    except Exception as e:
        print(f"❌ TEST FAILED: Unexpected error: {e}")
        return False


def main():
    """Run the kv warning test."""
    # Get test directory (where this script is located)
    test_dir = Path(__file__).parent.absolute()

    # Verify test files exist
    urdf_file = test_dir / "test_robot_nonzero_kv.urdf"
    xml_file = test_dir / "test_robot_nonzero_kv.xml"

    if not urdf_file.exists():
        print(f"❌ ERROR: URDF file not found: {urdf_file}")
        return 1

    if not xml_file.exists():
        print(f"❌ ERROR: MuJoCo XML file not found: {xml_file}")
        return 1

    # Run test
    success = run_kv_warning_test(test_dir)

    print()
    print("=" * 80)
    if success:
        print("KV WARNING TEST RESULT: PASSED ✅")
        return 0
    else:
        print("KV WARNING TEST RESULT: FAILED ❌")
        return 1


if __name__ == "__main__":
    sys.exit(main())


#!/bin/bash
# Simple manual test to demonstrate validation

echo "========================================================================"
echo "Manual Test: URDF/MuJoCo Actuator Validation"
echo "========================================================================"
echo ""
echo "This test demonstrates the validation by showing the actuator mismatch."
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Test Files:"
echo "  URDF: $SCRIPT_DIR/test_robot_mismatch.urdf"
echo "  XML:  $SCRIPT_DIR/test_robot_mismatch.xml"
echo ""

echo "------------------------------------------------------------------------"
echo "1. URDF Command Interfaces (what controllers can claim):"
echo "------------------------------------------------------------------------"
grep -A 3 "command_interface" "$SCRIPT_DIR/test_robot_mismatch.urdf" | grep "name=" | sed 's/.*name="/  - /' | sed 's/".*//'
echo ""

echo "------------------------------------------------------------------------"
echo "2. MuJoCo Actuators (what actually exists in simulation):"
echo "------------------------------------------------------------------------"
grep "<position\|<velocity\|<motor" "$SCRIPT_DIR/test_robot_mismatch.xml" | grep -v "<!--" | sed 's/.*name="/  - /' | sed 's/".*//'
echo ""

echo "------------------------------------------------------------------------"
echo "3. Expected Validation Error:"
echo "------------------------------------------------------------------------"
echo "  ❌ URDF declares: effort interface"
echo "  ❌ MuJoCo missing: act_tau_test_joint1 actuator"
echo ""
echo "  When you try to launch with these files, initialization should fail with:"
echo ""
echo "  [ERROR] Joint 'test_joint1' declares effort interface in URDF but"
echo "          no 'act_tau_test_joint1' actuator found in MuJoCo model."
echo ""

echo "------------------------------------------------------------------------"
echo "4. How to Test:"
echo "------------------------------------------------------------------------"
echo ""
echo "Option A - Using the automated test script:"
echo "  ./test_validation.py"
echo ""
echo "Option B - Manual launch (will fail with validation error):"
echo "  # Create a launch file or parameter file with these paths"
echo "  # Then run mujoco_ros2_control and observe the error"
echo ""

echo "------------------------------------------------------------------------"
echo "5. How to Fix:"
echo "------------------------------------------------------------------------"
echo ""
echo "Add the missing actuator to test_robot_mismatch.xml:"
echo ""
echo '  <motor name="act_tau_test_joint1" joint="test_joint1"'
echo '         ctrlrange="-10 10" forcerange="-10 10"/>'
echo ""
echo "Or remove the effort interface from test_robot_mismatch.urdf:"
echo ""
echo '  <!-- Remove this line: -->'
echo '  <command_interface name="effort"/>'
echo ""

echo "========================================================================"

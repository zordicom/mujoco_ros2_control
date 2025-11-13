#!/bin/bash
# Test runner for MuJoCo ROS2 Control validation tests

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo ""
echo "=============================================="
echo "MuJoCo ROS2 Control - Validation Test Runner"
echo "=============================================="
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS2 not sourced!${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is built
if [ ! -f "$HOME/ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}WARNING: Workspace not built or install/setup.bash not found${NC}"
    echo "Building workspace..."
    cd "$HOME/ros2_ws"
    colcon build --packages-select mujoco_ros2_control
fi

# Source workspace
echo "Sourcing workspace..."
source "$HOME/ros2_ws/install/setup.bash"

# Run the Python test
echo ""
echo "Running validation test..."
echo ""

cd "$SCRIPT_DIR"
python3 test_validation.py

exit_code=$?

echo ""
if [ $exit_code -eq 0 ]; then
    echo -e "${GREEN}✅ All tests passed!${NC}"
else
    echo -e "${RED}❌ Tests failed!${NC}"
fi

exit $exit_code

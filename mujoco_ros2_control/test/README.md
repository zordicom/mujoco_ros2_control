# MuJoCo ROS2 Control - Validation Tests

This directory contains tests for the URDF/MuJoCo actuator validation system.

## Test: Interface Validation

### Purpose

Verifies that the system correctly detects mismatches between URDF command interfaces and MuJoCo actuators, preventing silent failures.

### Test Files

- `test_robot_mismatch.urdf` - URDF declaring position, velocity, and effort interfaces
- `test_robot_mismatch.xml` - MuJoCo model with **missing torque actuator** (intentional mismatch)
- `test_validation.py` - Test script that verifies the error is caught

### Running the Test

```bash
# From this directory
cd /home/gilwoo/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test

# Make script executable
chmod +x test_validation.py

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run the test
./test_validation.py
```

### Expected Behavior

The test should **PASS** by verifying that:

1. System attempts to initialize with mismatched files
2. Validation detects missing `act_tau_test_joint1` actuator
3. System fails initialization with clear error message:

   ```
   [ERROR] Joint 'test_joint1' declares effort interface in URDF but no 'act_tau_test_joint1'
   actuator found in MuJoCo model. Please add the actuator or remove the interface.
   MIT mode requires all three actuators (position, velocity, effort).
   ```

### Test Scenarios

#### Scenario 1: Missing Torque Actuator (Current Test)

- **URDF**: Declares position, velocity, effort
- **MuJoCo**: Defines act_pos_*, act_vel_* only
- **Result**: ERROR - missing act_tau_*

#### Scenario 2: Missing Velocity Actuator

- **URDF**: Declares position, velocity
- **MuJoCo**: Defines act_pos_* only
- **Result**: ERROR - missing act_vel_*

#### Scenario 3: Complete Match

- **URDF**: Declares position, velocity, effort
- **MuJoCo**: Defines all three actuators
- **Result**: SUCCESS - system initializes

## Adding More Tests

To test other mismatch scenarios:

1. Copy `test_robot_mismatch.xml` to a new file
2. Remove or add actuators as needed
3. Create corresponding URDF if needed
4. Update `test_validation.py` to test the new scenario

# MuJoCo ROS2 Control Tests

This directory contains integration tests for the `mujoco_ros2_control` hardware interface,
validating the actuator types supported by MujocoSystem.

## Actuator Types Tested

| Test | Actuator Type | Command Interfaces | Controller |
|------|---------------|-------------------|------------|
| `test_interface_validation` | Position Servo | position | N/A (interface check) |
| `test_position_servo` | Position Servo | position | forward_command_controller |
| `test_position_velocity` | Position+Velocity | position, velocity | forward_command_controller |
| `test_torque_motor` | Torque Motor | effort | forward_command_controller |
| `test_mit_motor` | MIT Motor | position, velocity, effort, kp, kd | N/A (interface check) |

## Quick Start

```bash
# Build the package with tests
colcon build --packages-select mujoco_ros2_control

# Run all tests
colcon test --packages-select mujoco_ros2_control

# View test results
colcon test-result --verbose --test-result-base build/mujoco_ros2_control
```

## Running Individual Tests

```bash
# Source workspace first
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run a single test
python3 -m launch_testing.launch_test \
    src/mujoco_ros2_control/mujoco_ros2_control/test/test_position_servo.test.py
```

## Test Descriptions

### test_interface_validation.test.py

Validates that the hardware interface correctly claims the expected command interfaces
for each actuator type:
- Position servo: only `position` command interface
- Position+velocity: `position` and `velocity` interfaces
- MIT motor: all 5 interfaces (`position`, `velocity`, `effort`, `kp`, `kd`)

### test_position_servo.test.py

Tests position servo actuator using `forward_command_controller`:
- Command tracking: Send position, verify joint reaches target
- Hold against gravity: Verify servo holds position with internal PD

### test_position_velocity.test.py

Tests position+velocity actuator using `forward_command_controller`:
- Velocity feedforward: Both position and velocity commands applied
- Trajectory tracking with velocity targets

### test_torque_motor.test.py

Tests torque motor actuator using `forward_command_controller`:
- Gravity response: With zero torque, pendulum should fall (no internal PD)
- Torque response: Applied torque should cause movement

### test_mit_motor.test.py

Tests MIT motor actuator interface validation:
- All 5 interfaces available (position, velocity, effort, kp, kd)
- Hardware interface state reporting
- Joint state publishing

## Test Models

Located in `test/models/`:

| File | Description |
|------|-------------|
| `test_position_servo.urdf` | Position interface only, MuJoCo viewer disabled |
| `test_position_servo.xml` | Position actuator with kp=100, kv=10 |
| `test_position_velocity.urdf` | Position + velocity interfaces |
| `test_position_velocity.xml` | Position + velocity actuators |
| `test_torque_motor.urdf` | Effort interface only (Dynamixel Current Mode) |
| `test_torque_motor.xml` | Motor actuator (direct torque passthrough) |
| `test_mit_motor.urdf` | All 5 MIT interfaces with gain limits |
| `test_mit_motor.xml` | Motor actuator for MIT mode |

## Dependencies

- `launch_testing_ament_cmake`
- `launch_testing_ros`
- `forward_command_controller`
- `robot_state_publisher`
- `joint_state_broadcaster`


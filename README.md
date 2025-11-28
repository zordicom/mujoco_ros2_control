# mujoco_ros2_control

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains a ROS2 control package for MuJoCo simulation, offering the `MujocoSystem` plugin to integrate `ros2_control` with MuJoCo. It provides a realistic physics simulation backend for robot control development and testing.

## Key Features

### Actuator-Centric Control Architecture

- **Multi-interface control:** Each joint supports independent position, velocity, and torque actuators
- **Dynamic interface activation:** Runtime switching between control modes without reconfiguration
- **True MIT mode support:** Controller sends (position, velocity, effort, kp, kd) each cycle for variable impedance
- **Gain safety limits:** max_kp/max_kd from URDF prevent runaway gains
- **Flexible control strategies:** Position servo, torque motor, or MIT motor modes

See [Actuator Types Guide](mujoco_ros2_control/docs/ACTUATOR_TYPES.md) for detailed documentation.

### Simulation Control

- **Pause/unpause/reset:** Runtime simulation state management via ROS2 services (see [doc/updates.md](doc/updates.md#ros-services))
- **Initial keyframe support:** Start simulations from predefined configurations using MuJoCo XML keyframes (see [doc/updates.md](doc/updates.md#mujoco-model-requirements))
- **Reset to keyframe:** Return to specific configurations during runtime for repeated testing
- **Paused startup:** Simulation starts frozen, allowing controller setup before physics execution

### Physics Integration

- **Zero-lag control:** Commands applied within the same physics step for accurate response (see [doc/updates.md](doc/updates.md#technical-details))
- **External wrench service:** Apply forces/torques to bodies for disturbance testing (see [doc/updates.md](doc/updates.md#ros-services))
- **Gravity compensation validation:** Publishes MuJoCo's `qfrc_bias` for controller verification (see [doc/updates.md](doc/updates.md#topics-published))
- **Real-time synchronization:** Proper ordering of read/write/step operations

### Sensor Support

- **IMU sensors:** Orientation (quaternion), angular velocity, linear acceleration
- **Force-torque sensors:** 6-axis wrench measurements
- **Thread-safe publishing:** Monotonic clock guarantees for multi-threaded environments

### Developer Features

- **Auto-computed update rate:** Automatically derived from MuJoCo timestep
- **URDF/MuJoCo validation:** Validates actuator/interface matching at startup
- **KV warning system:** Detects potential damping issues for MIT mode compatibility
- **Comprehensive documentation:** Detailed technical references and migration guides

## Installation

### Prerequisites

- [ROS 2](https://docs.ros.org/) (Humble or later)
- [MuJoCo 3.3](https://mujoco.org/)

### Build Instructions

Configure the MuJoCo directory environment variable:

```bash
export MUJOCO_DIR=/PATH/TO/MUJOCO/mujoco-3.x.x
```

Build the package:

```bash
cd <your_ros2_workspace>
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select mujoco_ros2_control mujoco_ros2_control_msgs mujoco_ros2_control_demos
```

## Testing

### Demo Launch

The package includes a demo configuration in `mujoco_ros2_control_demos`:

**One-DOF Gravity Compensation Test:**

```bash
ros2 launch mujoco_ros2_control_demos test_1dof_gravity.launch.py
```

This demo tests:

- Torque control with gravity compensation
- Initial keyframe configuration
- Simulation control (pause/unpause/reset)
- External wrench application
- qfrc_bias publishing for validation

See `mujoco_ros2_control_demos/README.md` for detailed usage examples and additional demos.

### Feature Usage Examples

#### Simulation Control (§13)

Control simulation state at runtime:

```bash
# Query current state
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'status'}"

# Start simulation
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"

# Pause for inspection
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'pause'}"

# Reset to initial state
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'reset'}"
```

#### Keyframe Configuration (§7)

Reset to specific configurations during runtime:

```bash
# Reset to a named keyframe
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'home'}"

# Reset by keyframe index (as string)
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: '0'}"
```

**Define keyframes in your MuJoCo XML:**

```xml
<keyframe>
  <key name="home" qpos="0 0 0 0 0 0 0"/>
  <key name="ready" qpos="0 -0.5 0 -1.5 0 1.0 0"/>
</keyframe>
```

**Specify initial keyframe in launch file:**

```python
parameters=[{
    "initial_keyframe": "home",  # Start at home position
}]
```

#### External Wrench Application (§9)

Apply disturbances for testing:

```bash
# Apply 5 Nm torque about Y-axis for 2 seconds
ros2 service call /mujoco_ros2_control/apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{body_name: 'link_name', wrench: {force: {x: 0, y: 0, z: 0}, torque: {x: 0, y: 5.0, z: 0}}, duration: 2.0}"

# Apply 10 N force in X direction for 1 second
ros2 service call /mujoco_ros2_control/apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{body_name: 'link_name', wrench: {force: {x: 10.0, y: 0, z: 0}, torque: {x: 0, y: 0, z: 0}}, duration: 1.0}"
```

**Note:** Wrenches must be expressed in world frame.

#### Gravity Compensation Validation (§11)

Compare controller gravity compensation with MuJoCo ground truth:

```bash
# Terminal 1: Monitor MuJoCo's computed gravity/bias forces
ros2 topic echo /mujoco/qfrc_bias

# Terminal 2: Monitor controller's effort commands
ros2 topic echo /joint_states

# Compare the effort values with qfrc_bias for validation
```

## Documentation

- [Complete Documentation](doc/updates.md) - Architecture, features, migration guides
- [Simulation Control Guide](doc/SIMULATION_CONTROL.md) - Pause/unpause/reset functionality
- [Real-Time Synchronization](doc/REAL_TIME_SYNC_FIX.md) - Control loop timing analysis

## MuJoCo Model Requirements

MuJoCo XML models must include actuators that match the command interfaces declared in your URDF `<ros2_control>` section. Follow the naming convention: `act_pos_*`, `act_vel_*`, `act_tau_*`.

**Example: All three actuators (for MIT mode or multi-interface control):**

```xml
<actuator>
  <position name="act_pos_joint_name" joint="joint_name" kp="5000" kv="0"/>
  <velocity name="act_vel_joint_name" joint="joint_name" kv="100"/>
  <motor name="act_tau_joint_name" joint="joint_name"/>
</actuator>
```

**Example: Position-only control:**

```xml
<actuator>
  <position name="act_pos_joint_name" joint="joint_name" kp="5000" kv="50"/>
</actuator>
```

**Example: Effort-only control:**

```xml
<actuator>
  <motor name="act_tau_joint_name" joint="joint_name"/>
</actuator>
```

### MIT Mode Configuration

MIT mode enables variable impedance control where the controller sends all 5 values per cycle:
- `position_desired`, `velocity_desired`, `effort_feedforward`, `kp`, `kd`

**URDF Configuration:**

```xml
<joint name="joint1">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <command_interface name="kp"/>
  <command_interface name="kd"/>
  <!-- Safety limits (not default gains) -->
  <param name="max_kp">500</param>
  <param name="max_kd">50</param>
</joint>
```

**Controller Configuration (YAML):**

```yaml
zordi_joint_mit_controller:
  ros__parameters:
    command_interfaces: [position, velocity, effort]
    compute_pd_internally: false  # MIT mode
    default_kp: [100.0, 80.0]     # Gains per joint
    default_kd: [10.0, 8.0]
```

**Validation:** If a controller claims `[position, velocity, effort]` without `[kp, kd]`, the hardware interface will **error out** to prevent misconfiguration.

**MuJoCo Model:** Set `kv="0"` on position actuators to prevent interference when neutralized.

See [examples/mit_motor](mujoco_ros2_control/examples/mit_motor/) for a complete working example.

## Usage

See the [documentation](doc/index.rst) for detailed usage instructions and examples.

<!--
## Docker

Docker support is currently disabled pending testing updates.
A basic containerized workflow was previously provided to test this package in isolation.
For more information refer to the [docker documentation](docker/RUNNING_IN_DOCKER.md).
-->

## Contributing

Contributions are welcome! Please ensure:

- Code follows the existing style
- New features include tests
- Documentation is updated

## Future Work

1. **Body-frame wrenches:** Add support for body-frame wrench application
2. **Dynamic actuator tuning:** Runtime adjustment of kp/kv parameters
3. **Enhanced sensor support:** Additional sensor types and configurations
4. **URDF loading:** Direct URDF to MuJoCo conversion
5. **Simulation speed control:** Adjust real-time factor for faster/slower execution

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

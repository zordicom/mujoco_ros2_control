# MuJoCo ROS2 Control - Complete Documentation

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Overview

This document describes the complete architecture of `mujoco_ros2_control`, a ROS 2 Control hardware interface for MuJoCo simulation. The package provides:

- **Drop-in replacement** for `picknik_mujoco_ros/MujocoSystem` in MoveIt Pro
- **Actuator-centric control** with per-joint, per-actuator command routing
- **Full MIT mode support** with position/velocity/effort interfaces
- **Runtime simulation control** (pause/unpause/reset services)
- **Integrated interactive viewer** (optional, via URDF parameter)
- **Camera publishing** (RGB + depth images)
- **Clock publishing** for sim time synchronization

---

## Quick Start

### For MoveIt Pro Users

Change one line in your URDF:

```xml
<!-- Before (picknik_mujoco_ros) -->
<plugin>picknik_mujoco_ros/MujocoSystem</plugin>

<!-- After (mujoco_ros2_control) -->
<plugin>mujoco_ros2_control/MujocoSystem</plugin>
```

### For Development/Testing

```bash
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control mujoco_ros2_control_demos
source install/setup.bash
ros2 launch mujoco_ros2_control_demos test_2dof_gravity.launch.py
```

---

## Architecture

### Unified Lifecycle Plugin

The `MujocoSystem` plugin is always lifecycle-managed and always owns the MuJoCo model:

```
controller_manager
  └─> MujocoSystem plugin (lifecycle-managed)
        ├─> on_init(): Parse URDF parameters, load MuJoCo model, register joints
        ├─> on_configure(): Create services, publishers, optional viewer
        ├─> on_activate(): Reset simulation, set PAUSED state
        ├─> read(): Copy joint states FROM mj_data
        └─> write(): Copy commands TO mj_data->ctrl + mj_step() + publish clock
```

### Actuator-Centric Control

Each joint can have up to three independent actuators in the MuJoCo model:

| Actuator Type | Naming Convention | Purpose |
|---------------|-------------------|---------|
| Position | `act_pos_{joint_name}` | Position servo control |
| Velocity | `act_vel_{joint_name}` | Velocity control |
| Torque | `act_tau_{joint_name}` | Direct torque/MIT mode |

**Control modes are determined automatically** based on which interfaces a controller claims:

| Active Interfaces | Mode | Behavior |
|-------------------|------|----------|
| Position only | Position Mode | Position actuator driven, others neutralized |
| Velocity only | Velocity Mode | Velocity actuator driven, others neutralized |
| Effort only | Torque Mode | Torque actuator driven, others neutralized |
| Effort + Position/Velocity | MIT Mode | τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff |

---

## Visualization Options

### Option 1: RViz (Recommended for MoveIt Pro)

- No additional configuration needed
- Visualize via `/joint_states` topic
- Works with any robot model

### Option 2: Integrated MuJoCo Viewer (Optional)

Enable in URDF hardware parameters:

```xml
<hardware>
  <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  <param name="mujoco_model">models/robot.xml</param>
  <param name="mujoco_model_package">my_robot_description</param>
  <param name="enable_viewer">true</param>  <!-- Enable interactive viewer -->
</hardware>
```

**Viewer features:**

- Standard MuJoCo interactive viewer with mouse camera controls
- Runs in background thread at 60 Hz
- No impact on simulation when disabled (default)
- Close window to stop viewer

---

## URDF Configuration

### Basic Configuration

```xml
<ros2_control name="my_robot" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <param name="mujoco_model">models/robot.xml</param>
    <param name="mujoco_model_package">my_robot_description</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### Optional Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mujoco_model` | string | required | Relative path to MuJoCo XML model |
| `mujoco_model_package` | string | required | ROS package containing model |
| `enable_viewer` | bool | false | Enable interactive MuJoCo viewer |
| `enable_cameras` | bool | false | Enable camera image publishing |
| `camera_publish_rate` | double | 6.0 | Camera update rate (Hz) |

---

## MuJoCo Model Requirements

### Actuator Naming Convention

For each joint interface declared in URDF, a corresponding actuator must exist in the MuJoCo XML:

```xml
<mujoco>
  <actuator>
    <!-- Position actuator for joint1 -->
    <position name="act_pos_joint1" joint="joint1" kp="100" kv="0"/>

    <!-- Velocity actuator for joint1 -->
    <velocity name="act_vel_joint1" joint="joint1" kv="10"/>

    <!-- Torque actuator for joint1 -->
    <motor name="act_tau_joint1" joint="joint1" gear="1"/>
  </actuator>
</mujoco>
```

**Important for MIT Mode:** Position actuators should have `kv="0"` to prevent unwanted velocity damping when neutralized.

### Keyframes (Initial Poses)

Define keyframes in MuJoCo XML for initial configurations:

```xml
<mujoco>
  <keyframe>
    <key name="home" qpos="0 0 0 0 0 0"/>
    <key name="test_pose" qpos="0.3 -0.2 0 0 0 0"/>
  </keyframe>
</mujoco>
```

### Camera Definitions

```xml
<mujoco>
  <worldbody>
    <camera name="wrist_camera" pos="0 0 0.1" fovy="60" resolution="640 480"/>
    <camera name="overhead" pos="0 -1 1" fovy="45" resolution="1280 720"/>
  </worldbody>
</mujoco>
```

---

## ROS Services

All services are available at `/mujoco_system/` namespace.

### Reset to Keyframe

Reset simulation to a named keyframe:

```bash
ros2 service call /mujoco_system/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'home'}"
```

### Simulation Control

Control simulation execution state:

```bash
# Check status
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'status'}"

# Unpause (required after launch - simulation starts PAUSED)
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"

# Pause
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'pause'}"

# Reset to initial keyframe
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'reset'}"
```

**Note:** Simulation starts PAUSED by default. You must call `unpause` after activating controllers.

### Apply External Wrench

Apply forces/torques to bodies (world frame):

```bash
ros2 service call /mujoco_system/apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{body_name: 'link7', wrench: {force: {x: 10.0, y: 0, z: 0}, torque: {x: 0, y: 0, z: 0}}, duration: 2.0}"
```

---

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Simulation time for `use_sim_time:=true` |
| `/mujoco_system/qfrc_bias` | `std_msgs/Float64MultiArray` | Gravity/bias forces for debugging |
| `/<camera>/color` | `sensor_msgs/Image` | RGB images (if cameras enabled) |
| `/<camera>/depth` | `sensor_msgs/Image` | Depth images (if cameras enabled) |
| `/<camera>/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration |

---

## Technical Details

### Simulation Stepping

The plugin steps simulation in the `write()` method:

```cpp
mj_step1(mj_model_, mj_data_);  // First half-step (kinematics, collision)
// Commands applied here via mj_data->ctrl
mj_step2(mj_model_, mj_data_);  // Second half-step (physics integration)
publish_clock();                 // Publish /clock
```

### Control Flow

1. **read()**: Copy joint states from `mj_data->qpos`, `qvel`, `qfrc_actuator`
2. **Controller update**: ros2_control updates controllers
3. **write()**:
   - Check for pending resets
   - Handle pause state
   - Apply commands to actuators via `mj_data->ctrl`
   - Step simulation
   - Apply external wrenches
   - Publish clock and diagnostics

### Thread Safety

All shared state is protected by mutexes:

- `sim_state_mutex_` - Pause/unpause state
- `wrench_mutex_` - External wrench data
- `reset_mutex_` - Pending keyframe reset

The executor runs in a separate thread for service callbacks.

### Paused State Behavior

When PAUSED:

- Physics steps skipped (`mj_step1/step2` not called)
- Simulation time frozen
- Controllers remain active with `period=0`
- State interfaces unchanged

---

## Migration Guide

### From picknik_mujoco_ros

**Before:**

```xml
<plugin>picknik_mujoco_ros/MujocoSystem</plugin>
<param name="mujoco_model">${mujoco_model}</param>
<param name="mujoco_model_package">${mujoco_model_package}</param>
<param name="mujoco_viewer">false</param>
```

**After:**

```xml
<plugin>mujoco_ros2_control/MujocoSystem</plugin>
<param name="mujoco_model">${mujoco_model}</param>
<param name="mujoco_model_package">${mujoco_model_package}</param>
<param name="enable_viewer">true</param>  <!-- Optional -->
```

### From Old mujoco_ros2_control (Mode 2)

**Old workflow (DEPRECATED):**

```bash
ros2 run mujoco_ros2_control mujoco_ros2_control_node \
  --ros-args -p mujoco_model_path:=/path/to/model.xml
```

**New workflow:**

```bash
ros2 run controller_manager ros2_control_node \
  --ros-args -p robot_description:="$(cat robot.urdf)"
```

### MuJoCo Model Updates Required

**Old naming (no longer supported):**

```xml
<actuator name="actuator_joint1" joint="joint1"/>
```

**New naming (required):**

```xml
<position name="act_pos_joint1" joint="joint1" kp="100" kv="0"/>
<velocity name="act_vel_joint1" joint="joint1" kv="10"/>
<motor name="act_tau_joint1" joint="joint1" gear="1"/>
```

---

## Breaking Changes

| Change | Impact | Migration |
|--------|--------|-----------|
| Mode 2 removed | `mujoco_ros2_control_node` deleted | Use `controller_manager` directly |
| Actuator naming | Must use `act_pos_*`, `act_vel_*`, `act_tau_*` | Update MuJoCo XML |
| Separate viewer removed | `mujoco_viewer` executable deleted | Use integrated viewer via URDF param |
| Simulation starts PAUSED | Must call unpause service | Add unpause to launch/startup |

---

## Demo Package

The `mujoco_ros2_control_demos` package provides examples:

### Available Demos

| Launch File | Description |
|-------------|-------------|
| `test_2dof_gravity.launch.py` | 2-DOF vertical pendulum with gravity compensation |
| `test_1dof_gravity.launch.py` | 1-DOF gravity compensation demo |
| `test_planar_2dof.launch.py` | 2-DOF planar robot (no gravity) |

### Running a Demo

```bash
ros2 launch mujoco_ros2_control_demos test_2dof_gravity.launch.py
```

Controllers are loaded automatically via spawner. Simulation starts PAUSED.

---

## Feature Matrix

| Feature | MujocoSystem Plugin |
|---------|---------------------|
| MoveIt Pro integration | ✅ |
| Simulation stepping | ✅ |
| Services (reset/pause/wrench) | ✅ |
| Clock publishing | ✅ |
| Camera publishing | ✅ (optional) |
| Interactive viewer | ✅ (optional) |
| MIT mode support | ✅ |
| Actuator-centric control | ✅ |
| Lifecycle management | ✅ |
| Drop-in for picknik_mujoco_ros | ✅ |

---

## Changelog

### 2025-11-24: Unified Architecture

**Major Changes:**

- Removed Mode 2 (standalone node) - plugin-only architecture
- Integrated viewer into plugin (via URDF parameter)
- Fixed controller loading timing with spawner
- Fixed viewer pitch-black issue (GLFW thread context)
- Removed separate `mujoco_viewer` executable

**Files Changed:**

- `mujoco_system.cpp` - Added integrated viewer support
- `CMakeLists.txt` - Linked GLFW to plugin, removed viewer executable
- Launch files - Updated to use spawner for controller loading
- URDFs - Added `enable_viewer` parameter

### 2025-11: Actuator-Centric Control

**Features Added:**

- Per-joint, per-actuator control routing
- Dynamic interface activation tracking
- MIT mode with automatic detection
- URDF/MuJoCo validation system
- KV warning for MIT mode compatibility
- Real-time synchronization fix
- External wrench service
- Simulation control service (pause/unpause/reset)
- Gravity compensation validation (qfrc_bias)
- Auto-compute update rate from MuJoCo timestep

---

## References

- Original mujoco_ros2_control: <https://github.com/sangteak601/mujoco_ros2_control>
- MoveIt Pro documentation: <https://moveit.picknik.ai/>
- ros2_control documentation: <https://control.ros.org/>
- MuJoCo documentation: <https://mujoco.readthedocs.io/>

---

**Copyright 2025 Zordi, Inc. All rights reserved.**

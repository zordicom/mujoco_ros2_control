# MuJoCo ROS2 Control - Complete Documentation

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Overview

This document describes the complete architecture of `mujoco_ros2_control`, a ROS 2 Control hardware interface for MuJoCo simulation. The package provides:

- **Drop-in replacement** for `picknik_mujoco_ros/MujocoSystem` in MoveIt Pro
- **Shared simulation singleton** for bimanual/multi-arm robots
- **Actuator-centric control** with per-joint, per-actuator command routing
- **Full MIT mode support** with dynamic kp/kd gains
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

### Shared Simulation (Singleton Pattern)

To support complex systems like bimanual robots where multiple `hardware_interface` instances (e.g., left arm, right arm) need to interact within the **same** physics world, `mujoco_ros2_control` uses a Singleton pattern for the MuJoCo model and data.

#### Mechanism

- **Shared State:** `mjModel*`, `mjData*`, and the ROS node are static members shared across all instances of `MujocoSystem`.
- **Primary/Secondary Logic:**
  - The **first** initialized interface becomes the **PRIMARY** instance.
  - Subsequent interfaces become **SECONDARY** instances.
  - Only the **PRIMARY** instance is responsible for:
    - Loading the XML model.
    - Creating the ROS node (`mujoco_system`).
    - Creating global services (`~/simulation_control`, `~/reset_to_keyframe`, `~/apply_external_wrench`).
    - Stepping the simulation (`mj_step1`, `mj_step2`).
    - Publishing `/clock` and `~/qfrc_bias`.
    - Managing the interactive viewer.

#### Static Members

```cpp
static std::mutex static_mutex_;
static mjModel* shared_model_;
static mjData* shared_data_;
static int instance_count_;
static rclcpp::Node::SharedPtr shared_node_;
static std::string shared_model_path_;
```

#### Lifecycle

| Phase | Primary Instance | Secondary Instance |
|-------|-----------------|-------------------|
| `on_init()` | Loads model, creates `mjData`, creates ROS node | Reuses shared model/data/node |
| `on_configure()` | Creates services, publishers, starts viewer | No-op for services/viewer |
| `read()` | Steps simulation, publishes clock | Only reads joint states |
| `write()` | Writes to `mj_data_->ctrl` | Writes to `mj_data_->ctrl` |
| Destructor | Frees model/data when last instance | Clears local pointers only |

#### Model Path Validation

- All instances MUST specify the same `mujoco_model` file.
- If a secondary instance requests a different model path, initialization **fails** with an error.

### Unified Lifecycle Plugin

The `MujocoSystem` plugin is always lifecycle-managed:

```
controller_manager
  └─> MujocoSystem plugin (lifecycle-managed)
        ├─> on_init(): Parse URDF parameters, load MuJoCo model, register joints
        ├─> on_configure(): Create services, publishers, optional viewer
        ├─> on_activate(): Reset simulation, set PAUSED state
        ├─> read(): Copy joint states FROM mj_data (PRIMARY steps simulation)
        └─> write(): Copy commands TO mj_data->ctrl
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
| Position + Velocity | Position+Velocity Mode | Both actuators active for trajectory tracking |
| Effort only | Torque Mode | Direct torque passthrough (Dynamixel Current Mode) |
| Effort + pos/vel + kp + kd | MIT Mode | τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff |

**Note:** MIT mode (effort + position/velocity) requires kp/kd interfaces. Pure effort-only mode is allowed for Dynamixel Current Mode simulation.

---

## Control Loop Timing

### Why Step in `read()`?

In ros2_control, the control loop is:

1. `read()` for ALL hardware interfaces
2. `update()` for ALL controllers
3. `write()` for ALL hardware interfaces

By stepping the simulation in the PRIMARY's `read()`:

- Commands from ALL interfaces (written in the previous cycle) are applied **before** the step.
- All interfaces read **consistent** post-step state.

This introduces a 1-cycle latency for secondary interfaces, which is standard for distributed hardware.

---

## MIT Control Mode Support

Full support for MIT-style control (Position + Velocity + Feedforward Torque + Kp + Kd).

### Control Law

```
τ = τ_ff + kp * (q_cmd - q) + kd * (qd_cmd - qd)
```

Where:

- `τ_ff` = feedforward torque (effort command)
- `kp`, `kd` = dynamic gains from controller (via kp/kd command interfaces)
- `q_cmd`, `qd_cmd` = position/velocity setpoints

### Interface Requirements

Controllers MUST claim all 5 interfaces for MIT mode:

- `position`, `velocity`, `effort`, `kp`, `kd`

### Safety

- **Hard Error:** If a controller claims `effort` + (`position` OR `velocity`) but NOT `kp`/`kd`, `perform_command_mode_switch()` logs a warning.
- **Gain Clamping:** `kp` and `kd` are clamped to `max_kp`/`max_kd` (configurable via URDF parameters).

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
  <param name="mujoco_viewer">true</param>  <!-- Enable interactive viewer -->
</hardware>
```

**Viewer features:**

- Standard MuJoCo interactive viewer with mouse camera controls
- Runs in background thread at 60 Hz (PRIMARY instance only)
- No impact on simulation when disabled (default)
- Close window to stop viewer

**For bimanual setups:** Enable viewer only on ONE hardware interface (e.g., left arm). The right arm should have `mujoco_viewer` set to `false` (GLFW limitation).

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

### Bimanual Robot Configuration

Define two `<ros2_control>` tags in your URDF (one per arm). Both MUST point to the **same** `mujoco_model` XML file.

```xml
<!-- Left Arm (PRIMARY, with viewer) -->
<ros2_control name="left_arm" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <param name="mujoco_model">robot_bimanual.xml</param>
    <param name="mujoco_model_package">my_robot_description</param>
    <param name="mujoco_viewer">true</param>
  </hardware>
  <joint name="left_joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="max_kp">500</param>
    <param name="max_kd">50</param>
  </joint>
  <!-- more joints... -->
</ros2_control>

<!-- Right Arm (SECONDARY, headless) -->
<ros2_control name="right_arm" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <param name="mujoco_model">robot_bimanual.xml</param>
    <param name="mujoco_model_package">my_robot_description</param>
    <param name="mujoco_viewer">false</param>
  </hardware>
  <joint name="right_joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="max_kp">500</param>
    <param name="max_kd">50</param>
  </joint>
  <!-- more joints... -->
</ros2_control>
```

### Optional Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mujoco_model` | string | required | Relative path to MuJoCo XML model |
| `mujoco_model_package` | string | required | ROS package containing model |
| `mujoco_viewer` | bool | false | Enable interactive MuJoCo viewer |
| `enable_cameras` | bool | false | Enable camera image publishing |
| `camera_publish_rate` | double | 6.0 | Camera update rate (Hz) |

### Joint Parameters (for MIT mode)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_kp` | double | 1000.0 | Maximum allowed kp gain |
| `max_kd` | double | 100.0 | Maximum allowed kd gain |

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

All services are available at `/mujoco_system/` namespace (PRIMARY instance only).

| Service | Description |
|---------|-------------|
| `~/simulation_control` | Pause/unpause/reset/status |
| `~/reset_to_keyframe` | Reset to named or indexed keyframe |
| `~/apply_external_wrench` | Apply force/torque to a body |

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

The PRIMARY instance steps simulation in the `read()` method:

```cpp
mj_step1(mj_model_, mj_data_);  // First half-step (kinematics, collision)
// Commands applied here via mj_data->ctrl
mj_step2(mj_model_, mj_data_);  // Second half-step (physics integration)
publish_clock();                 // Publish /clock
```

### Control Flow

1. **read()** (PRIMARY): Step simulation, copy joint states from `mj_data->qpos`, `qvel`, `qfrc_actuator`
2. **read()** (SECONDARY): Copy joint states only (no stepping)
3. **Controller update**: ros2_control updates controllers
4. **write()** (ALL):
   - Check for pending resets
   - Handle pause state
   - Apply commands to actuators via `mj_data->ctrl`
   - Apply external wrenches (PRIMARY only)

### Thread Safety

All shared state is protected by mutexes:

- `static_mutex_` - Shared model/data initialization
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

## Known Limitations

1. **Single Model Only:** All `MujocoSystem` instances must use the same XML model file. Loading different models is not supported.
2. **GLFW Single-Init:** Only one viewer can exist per process (GLFW constraint).
3. **1-Cycle Latency:** Secondary interfaces have a 1-cycle delay between command and state update.

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
<param name="mujoco_viewer">true</param>  <!-- Optional -->
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
| Shared simulation (bimanual) | ✅ |
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

### 2025-11-30: Shared Simulation Singleton + MIT Mode Fixes

**Major Changes:**

- Implemented singleton pattern for shared `mjModel`/`mjData` across multiple hardware interfaces
- PRIMARY instance handles simulation stepping, services, viewer
- SECONDARY instances read/write to shared state
- Fixed MIT mode to compute PD in software with dynamic `kp`/`kd` gains
- Added dual pendulum test for bimanual-like control validation
- All 6 actuator type tests passing

**Files Changed:**

- `mujoco_system.hpp` - Added static members for shared state
- `mujoco_system.cpp` - Implemented PRIMARY/SECONDARY logic, fixed MIT mode
- `test/test_dual_pendulum.test.py` - New test for shared simulation
- `test/models/test_dual_pendulum.xml` - New dual pendulum MuJoCo model
- `doc/status.md` - Architecture documentation (merged into this file)

### 2025-11-28: Support Both MIT Mode and Pure Torque Mode

**Supported Motor Types:**

- **Pure Torque Mode** (Dynamixel Current Mode, Kuka iiwa): Claim `[effort]` only
- **MIT Mode** (Damiao, Unitree): Claim `[position, velocity, effort, kp, kd]`

**Validation:**

MIT mode (effort + position/velocity) requires kp/kd interfaces. Pure effort-only is allowed for motors that support direct torque control.

**ZordiJointController Changes:**

- `compute_pd_internally: true` now uses MIT protocol with kp=kd=0 (software PD)
- `compute_pd_internally: false` sends config gains to hardware (hardware PD)
- Both modes claim all 5 MIT interfaces for Damiao compatibility

---

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
- URDFs - Added `mujoco_viewer` parameter

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

# MuJoCo ROS2 Control Updates

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Summary (Current Branch vs Main)

**Branch:** `2025-11-control-interface`
**Files Changed:** 42 files
**Changes:** +4316 insertions, -72 deletions

---

## Overview

This branch represents a complete architectural redesign of `mujoco_ros2_control` from global mode switching to **actuator-centric control**. The new design enables flexible multi-interface control, proper MIT mode support, and better alignment with real hardware behavior.

---

## Architecture Changes

### 1. Actuator-Centric Control Design

**Previous:** Global control mode (position/velocity/effort) determined behavior for all joints

**Current:** Per-joint, per-actuator control with dynamic interface activation

#### Three Actuators Per Joint

Each joint can have up to three independent actuators:

```xml
<!-- MuJoCo model -->
<actuator>
  <position name="act_pos_joint1" joint="joint1" kp="5000" kv="0"/>
  <velocity name="act_vel_joint1" joint="joint1" kv="500"/>
  <motor name="act_tau_joint1" joint="joint1" gear="1"/>
</actuator>
```

**Naming convention:**

- Position actuator: `act_pos_{joint_name}`
- Velocity actuator: `act_vel_{joint_name}`
- Torque actuator: `act_tau_{joint_name}`

#### JointState Structure

```cpp
struct JointState {
  // ... existing fields ...

  // MuJoCo actuator IDs
  int mj_pos_actuator_id{-1};
  int mj_vel_actuator_id{-1};
  int mj_tau_actuator_id{-1};

  // Dynamic interface activation tracking
  bool position_command_active{false};
  bool velocity_command_active{false};
  bool effort_command_active{false};

  // KV warning flag
  bool warned_about_position_kv{false};
};
```

---

### 2. Dynamic Interface Activation Tracking

**New callbacks implemented:**

```cpp
hardware_interface::return_type prepare_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces) override;

hardware_interface::return_type perform_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces) override;
```

**Behavior:**

- `prepare_command_mode_switch()`: Validates requested interface combination (always accepts in actuator-centric design)
- `perform_command_mode_switch()`: Updates `*_command_active` flags to track which interfaces are claimed by controllers

**Purpose:**

Distinguishes between:

- **Exposed interfaces** (always available): `is_*_control_enabled = true`
- **Active interfaces** (claimed by controller): `*_command_active = true/false`

This enables proper MIT mode detection and actuator neutralization.

---

### 3. Write Loop: Actuator Control Logic

**Core principle:** Drive or neutralize each actuator independently based on active interfaces

```cpp
hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time &time, const rclcpp::Duration &period)
{
  for (auto &joint_state : joint_states_)
  {
    const double q = mj_data_->qpos[joint_state.mj_pos_adr];
    const double qd = mj_data_->qvel[joint_state.mj_vel_adr];

    // MIT mode detection
    bool mit_mode = joint_state.effort_command_active &&
                    (joint_state.position_command_active ||
                     joint_state.velocity_command_active);

    // Position actuator: command or neutralize
    if (joint_state.mj_pos_actuator_id >= 0)
    {
      if (joint_state.position_command_active && !mit_mode)
      {
        // Pure position mode: drive position actuator
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = position_cmd;
      }
      else
      {
        // Neutralize: ctrl = q (requires kv=0!)
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = q;
      }
    }

    // Velocity actuator: command or neutralize
    if (joint_state.mj_vel_actuator_id >= 0)
    {
      if (joint_state.velocity_command_active && !mit_mode)
      {
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = vel_cmd;
      }
      else
      {
        // Neutralize: ctrl = qd
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = qd;
      }
    }

    // Torque actuator: MIT-style composition
    if (joint_state.mj_tau_actuator_id >= 0)
    {
      if (joint_state.effort_command_active)
      {
        double tau_total = joint_state.effort_command;

        // Add PD terms if position/velocity interfaces also active
        if (joint_state.position_command_active ||
            joint_state.velocity_command_active)
        {
          double tau_pd = kp * (pos_cmd - q) + kd * (vel_cmd - qd);
          tau_total += tau_pd;
        }

        mj_data_->ctrl[joint_state.mj_tau_actuator_id] =
          clamp(tau_total, -limit, limit);
      }
      else
      {
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] = 0.0;
      }
    }
  }
}
```

---

### 4. Control Modes

The actuator-centric design naturally supports multiple control modes without explicit mode switching:

#### Pure Position Mode

- **Active interfaces:** Position only
- **Behavior:** Position actuator driven with `ctrl = pos_cmd`, others neutralized
- **Use case:** `joint_trajectory_controller`, position servoing

#### Pure Velocity Mode

- **Active interfaces:** Velocity only
- **Behavior:** Velocity actuator driven with `ctrl = vel_cmd`, others neutralized
- **Use case:** Velocity control applications

#### Pure Torque Mode

- **Active interfaces:** Effort only
- **Behavior:** Torque actuator driven with `ctrl = effort_cmd`, others neutralized
- **Use case:** Force control, gravity compensation

#### MIT Mode (Full Multi-Interface)

- **Active interfaces:** Effort + (Position and/or Velocity)
- **Behavior:**
  - Position and velocity actuators neutralized
  - Torque actuator driven with `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff`
- **Use case:** `zordi_mit_controller`, advanced control with feedforward

---

### 5. URDF/MuJoCo Validation System

**Purpose:** Prevent silent failures from mismatched configurations

**Implementation:** During `register_joints()`, validate that each command interface in URDF has a corresponding actuator in MuJoCo model

```cpp
// Check for mismatches and fail initialization
if (has_position_interface && joint_state.mj_pos_actuator_id < 0) {
  RCLCPP_ERROR(
    logger_,
    "Joint '%s' declares position interface in URDF but no 'act_pos_%s' "
    "actuator found in MuJoCo model. Please add the actuator or remove the interface.",
    joint.name.c_str(), joint.name.c_str());
  throw std::runtime_error(
    "URDF/MuJoCo mismatch: position interface without actuator for joint " + joint.name);
}

// Similar checks for velocity and effort interfaces
```

**Error messages:**

- Clear indication of which joint has the mismatch
- Specific actuator name expected
- Guidance on how to fix (add actuator or remove interface)

**Testing:** See `mujoco_ros2_control/test/README.md` for validation test suite

---

### 6. KV Warning System for MIT Mode Compatibility

**Problem:**

When a position actuator is neutralized (ctrl = q), the MuJoCo control law becomes:

```
τ = kp*(q - q) - kv*qd = -kv*qd
```

If `kv ≠ 0`, this produces unwanted velocity damping that interferes with MIT mode torque control.

**Solution:**

Warn during initialization if position actuator has non-zero kv and effort interface is exposed:

```cpp
if (joint_state.mj_pos_actuator_id >= 0 && has_effort_interface)
{
  const double kv = /* read from actuator parameters */;

  if (std::abs(kv) > 1e-6)
  {
    RCLCPP_WARN(
      logger_,
      "Joint '%s': Position actuator has kv=%.3f but effort interface is also exposed. "
      "During MIT mode (when position actuator is neutralized), this will cause "
      "unwanted damping (τ = -%.3f * qd). For MIT mode compatibility, set kv=0.0.",
      joint.name.c_str(), kv, kv);
  }
}
```

**Recommended configuration for MIT mode:**

```xml
<position name="act_pos_joint1" joint="joint1" kp="5000" kv="0"/>
```

**Testing:** See `mujoco_ros2_control/test/test_kv_warning.py`

---

### 7. Initial Pose Configuration

**Added:** Support for defining initial joint configurations in YAML

**Purpose:** Start simulations from specific configurations (e.g., testing gravity compensation, collision avoidance, specific scenarios)

#### YAML Configuration Format

Create a YAML file defining one or more named poses:

```yaml
poses:
  test_pose:
    j1: 0.5
    description: "Pendulum at 0.5 rad for gravity comp testing"

  home_pose:
    joint1: 0.0
    joint2: -1.57
    joint3: 1.57
    description: "Robot home position"
```

**Joint naming:** Use the short joint name (e.g., `j1`, `joint2`). For joints named like `openarm_joint2`, you can use either `joint2` or the full name.

**Units:** Joint positions are in radians for revolute joints, meters for prismatic joints.

**Description:** Optional field for documentation purposes.

#### Launch File Configuration

Pass the pose name and config file path as node parameters:

```python
initial_pose_config = pkg_share / "config" / "initial_poses.yaml"

mujoco_node = Node(
    package="mujoco_ros2_control",
    executable="mujoco_ros2_control",
    parameters=[
        {
            "robot_description": robot_description,
            "mujoco_model_path": str(mujoco_model),
            "initial_pose": "test_pose",  # Name of pose in YAML
            "initial_pose_config": str(initial_pose_config),  # Path to YAML file
        },
        controller_config,
    ],
    output="screen",
)
```

#### Behavior

- Both `initial_pose` and `initial_pose_config` parameters must be provided to override URDF defaults
- Joint velocities are set to zero
- Joints not listed in the pose configuration retain their URDF default values
- MuJoCo forward dynamics (`mj_forward`) is called to update derived quantities (body positions, sensor data, etc.)
- Position commands are initialized from actual qpos (after override)
- If either parameter is missing, the system uses URDF default positions

#### Example

See `mujoco_ros2_control_demos` for a complete example:

- Config: `mujoco_ros2_control_demos/config/initial_pose_test.yaml`
- Launch: `mujoco_ros2_control_demos/launch/test_1dof_gravity.launch.py`

---

### 8. Real-Time Synchronization Fix

**Problem (old):** Read/write called after stepping simulation, causing 1-step lag

**Solution (new):** Reordered update loop to apply controls before physics step

#### New Update Loop

```cpp
void MujocoRos2Control::update()
{
  // 1. Compute current sim time BEFORE stepping
  auto pre_time = mj_data_->time;
  rclcpp::Time pre_time_ros = to_ros_time(pre_time);
  rclcpp::Duration pre_period = pre_time_ros - last_update_sim_time_ros_;

  // 2. Read state and update controllers BEFORE stepping
  controller_manager_->read(pre_time_ros, pre_period);
  controller_manager_->update(pre_time_ros, pre_period);

  // 3. First half-step (kinematics, collision detection)
  mj_step1(mj_model_, mj_data_);

  // 4. Apply commands BETWEEN step1 and step2
  controller_manager_->write(pre_time_ros, pre_period);

  // 5. Apply external wrenches (if any)
  apply_active_external_wrench();

  // 6. Second half-step (physics integration)
  mj_step2(mj_model_, mj_data_);

  // 7. Publish clock AFTER stepping
  publish_sim_time(to_ros_time(mj_data_->time));

  // 8. Update last control time
  if (pre_period >= control_period_) {
    last_update_sim_time_ros_ = to_ros_time(mj_data_->time);
  }
}
```

**Benefits:**

- Zero-lag control: commands applied to the step being computed
- Proper causality: controller decisions based on state at time t affect physics at time t
- Matches real-time control loop behavior

**Documentation:** See `doc/REAL_TIME_SYNC_FIX.md` for detailed analysis

---

### 9. External Wrench Service

**Added:** New `mujoco_ros2_control_msgs` package with `ApplyExternalWrench.srv`

#### Service Definition

```
# Apply external wrench (force + torque) to a MuJoCo body for testing
string body_name              # Name of the body to apply wrench to
geometry_msgs/Wrench wrench   # Wrench to apply (force + torque)
bool in_world_frame           # If true, wrench is in world frame; if false, in body frame
float64 duration              # Duration to apply the wrench (seconds)
---
bool accepted                 # True if the wrench was accepted
string message                # Status message
```

#### Implementation

- Uses MuJoCo's `xfrc_applied` (external force/torque vector)
- Thread-safe with mutex protection
- Automatic expiration after duration
- Applied between `mj_step1` and `mj_step2` in update loop

#### Usage Example

```python
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench
from geometry_msgs.msg import Wrench

client = node.create_client(ApplyExternalWrench, '/apply_external_wrench')

request = ApplyExternalWrench.Request()
request.body_name = 'link4'
request.wrench.force.x = 10.0  # 10N in x direction
request.duration = 2.0  # Apply for 2 seconds

future = client.call_async(request)
```

**Use cases:**

- Disturbance rejection testing
- External force simulation
- Debugging and validation

---

### 10. Python Viewer Support

**Added:** Embedded Python MuJoCo viewer using `mujoco.viewer`

**Files:**

- `mujoco_ros2_control/include/mujoco_ros2_control/python_viewer.hpp`
- `mujoco_ros2_control/src/python_viewer.cpp`

**Dependencies:** Python 3.10+, `mujoco` Python package

**Purpose:** Alternative to GLFW viewer with better Python integration

---

### 11. Auto-Compute Update Rate

**Previous:** Required manual `update_rate` parameter in controller manager config

**Current:** Automatically computed from MuJoCo timestep if not explicitly set

```cpp
if (!controller_manager_->has_parameter("update_rate"))
{
  // Derive update rate from MuJoCo model timestep
  // MuJoCo timestep (e.g., 0.001s) → update_rate (e.g., 1000 Hz)
  int auto_update_rate = static_cast<int>(1.0 / mj_model_->opt.timestep);
  controller_manager_->declare_parameter("update_rate", auto_update_rate);
  RCLCPP_INFO(
    logger_,
    "Auto-set controller update_rate=%d Hz from MuJoCo timestep=%.6f s",
    auto_update_rate, mj_model_->opt.timestep);
}
```

**Benefits:**

- Reduces configuration boilerplate
- Ensures synchronization between MuJoCo and ros2_control
- Can still be overridden explicitly if needed

---

### 12. Gravity Compensation Validation

**Added:** Publisher for `qfrc_bias` (MuJoCo's gravity/Coriolis/centrifugal term)

```cpp
// In update() loop, after mj_step2
std_msgs::msg::Float64MultiArray qfrc_bias_msg;
qfrc_bias_msg.data.resize(mj_model_->nv);
for (int i = 0; i < mj_model_->nv; i++)
{
  qfrc_bias_msg.data[i] = mj_data_->qfrc_bias[i];
}
qfrc_bias_publisher_->publish(qfrc_bias_msg);
```

**Purpose:**

- Validate gravity compensation implementations
- Compare controller-computed gravity terms with MuJoCo ground truth
- Debug Pinocchio integration in `zordi_mit_controller`

**Topic:** `/mujoco/qfrc_bias`

---

### 13. Thread-Safe Clock Publishing

**Problem:** Multi-threaded architecture (controller manager in separate thread) could cause out-of-order clock messages

**Solution:** Mutex-protected monotonic clock guarantee

```cpp
void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  static rclcpp::Time last_published_time(0, 0, RCL_ROS_TIME);
  static std::mutex clock_mutex;

  std::lock_guard<std::mutex> lock(clock_mutex);
  if (sim_time <= last_published_time)
  {
    return;  // Skip if time hasn't advanced
  }

  last_published_time = sim_time;
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}
```

**Benefits:**

- Prevents RViz and other nodes from resetting due to backward time jumps
- Standard practice for multi-threaded ROS2 simulations

---

## Testing Infrastructure

### New Test Suite

**Location:** `mujoco_ros2_control/test/`

**Tests:**

1. **URDF/MuJoCo Validation Test**
   - File: `test_validation.py`
   - Purpose: Verify mismatch detection between URDF and MuJoCo actuators
   - Expected: Throws error with clear message

2. **KV Warning Test**
   - File: `test_kv_warning.py`
   - Purpose: Verify warning for non-zero kv in position actuators
   - Expected: Emits warning during initialization

3. **Manual Integration Tests**
   - Scripts: `manual_test.sh`, `run_test.sh`
   - Purpose: End-to-end testing with real controllers

**Documentation:** See `test/README.md` and `test/TEST_SUMMARY.md`

---

## New Demo Packages

### Test Demos

**Location:** `mujoco_ros2_control_demos/`

**New demos:**

1. **1-DOF Gravity Test** (`test_1dof_gravity.launch.py`)
   - Vertical pendulum with gravity compensation
   - Uses initial pose configuration
   - Validates gravity compensation with `qfrc_bias` topic

2. **1-DOF MIT Mode Test** (`test_1dof_mit.launch.py`)
   - Full MIT mode with `zordi_mit_controller`
   - Tests multi-interface control

3. **1-DOF Multi-Mode Test** (`test_1dof_multimode.launch.py`)
   - Systematic testing of all interface combinations
   - See `mujoco_ros2_control_demos/docs/MULTI_INTERFACE_TESTING.md`

---

## Migration Guide

### For Existing Users

#### MuJoCo Model Changes

**Required naming:**

```xml
<actuator>
  <position name="act_pos_joint1" joint="joint1" kp="5000" kv="0"/>
  <velocity name="act_vel_joint1" joint="joint1" kv="500"/>
  <motor name="act_tau_joint1" joint="joint1" gear="1"/>
</actuator>
```

**Notes:**

- Must use exact naming convention: `act_pos_*`, `act_vel_*`, `act_tau_*`
- For MIT mode, must add all three actuators
- Set `kv="0"` on position actuators for clean neutralization
- Old naming (e.g., `actuator_joint1`) is no longer supported

#### URDF Changes

**Old:** Conditional interfaces based on control mode

**New:** Always expose all interfaces you want to use

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  </hardware>
  <joint name="joint1">
    <!-- Always expose all interfaces -->
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

#### Controller Configuration

No changes needed - controllers work as before. Mode switching happens automatically based on which interfaces the controller claims.

---

## Backward Compatibility

### Breaking Changes

- **Removed:** Global `control_mode` parameter (was: "position", "velocity", "effort", "all")
- **Removed:** Legacy actuator naming (`actuator_*`) - must use `act_pos_*`, `act_vel_*`, `act_tau_*`
- **Deprecated:** Mode switching logic (replaced by actuator-centric design)
- **Required:** Matching actuators in MuJoCo for each URDF command interface

### Preserved Compatibility

- Existing controllers work without modification (no changes needed)
- URDF with single interface (position-only, velocity-only, effort-only) works as before
- Only MuJoCo model actuator names need to be updated

---

## Related Changes

### Required: Updated Packages

- **openarm_description**: Standard interface names, three actuators per joint
- **openarm_ros2**: Launch file updates for initial pose support

### Enabled: New Packages

- **zordi_mit_controller**: Full MIT mode with gravity compensation
- **mujoco_ros2_control_msgs**: Service definitions for simulation utilities

---

## Documentation

### New Documentation Files

- `doc/DAMIAO_POSITION_MODE.md` - Modeling DAMIAO position servo mode
- `doc/MODE_COMPARISON.md` - Comparison of control modes
- `doc/POSITION_SERVO_MODE_FINAL.md` - Position servo implementation
- `doc/REAL_TIME_SYNC_FIX.md` - Real-time synchronization analysis
- `doc/mujoco_ros2_control_updates.md` - This file

### Test Documentation

- `test/README.md` - Test suite overview
- `test/TEST_SUMMARY.md` - Detailed test results
- `test/KV_WARNING_SUMMARY.md` - KV warning system documentation

---

## Performance Notes

- **Zero-lag control:** Commands applied immediately in the same simulation step
- **Overhead:** Minimal (actuator selection is simple boolean logic)
- **Thread safety:** Mutex-protected only for external wrench and clock publishing
- **Update rate:** Automatically matched to MuJoCo timestep (typically 1000 Hz)

---

## Future Work

1. **Body-frame wrenches:** Currently external wrenches are world-frame only
2. **Dynamic actuator gain tuning:** Runtime adjustment of kp/kv parameters
3. **Sensor support:** Expand IMU and force-torque sensor capabilities
4. **URDF loading:** Direct URDF to MuJoCo conversion (eliminate XML step)

---

**Copyright 2025 Zordi, Inc. All rights reserved.**

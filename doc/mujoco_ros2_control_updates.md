# MuJoCo ROS2 Control Updates

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Summary (Current Branch vs Main)

**Branch:** `2025-11-control-interface`
**Files Changed:** 47+ files
**Changes:** +4480+ insertions, -73 deletions

---

## Overview

This branch represents a complete architectural redesign of `mujoco_ros2_control` from global mode switching to **actuator-centric control**. The new design enables flexible multi-interface control, proper MIT mode support, better alignment with real hardware behavior, and improved simulation control capabilities.

**Key additions:**

- Actuator-centric control architecture (no global mode switching)
- Runtime simulation control (pause/unpause/reset)
- Initial keyframe configuration support
- External wrench application service
- Gravity compensation validation
- Real-time synchronization fixes

---

## Architecture Changes

### 1. Actuator-Centric Control Design

**Previous:** Global control mode (position/velocity/effort) determined behavior for all joints

**Current:** Per-joint, per-actuator control with dynamic interface activation

#### Three Actuators Per Joint

Each joint can have up to three independent actuators in the MuJoCo model.

**Naming convention:**

- Position actuator: `act_pos_{joint_name}`
- Velocity actuator: `act_vel_{joint_name}`
- Torque actuator: `act_tau_{joint_name}`

#### JointState Structure

The `JointState` struct tracks:

- MuJoCo actuator IDs for each control type (position/velocity/torque)
- Dynamic interface activation flags (`position_command_active`, `velocity_command_active`, `effort_command_active`)
- KV warning status to avoid repeated warnings

---

### 2. Dynamic Interface Activation Tracking

**New callbacks implemented:**

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

**MIT mode detection:** Effort interface active + (position OR velocity interface active)

**Position actuator logic:**

- If position active and not MIT mode: Drive with position command
- Otherwise: Neutralize by setting ctrl = current_position (requires kv=0!)

**Velocity actuator logic:**

- If velocity active and not MIT mode: Drive with velocity command
- Otherwise: Neutralize by setting ctrl = current_velocity

**Torque actuator logic:**

- If effort active: Use effort command
- If MIT mode: Add PD terms: τ_total = τ_ff + Kp*(q_cmd - q) + Kd*(qd_cmd - qd)
- Apply torque limits
- Otherwise: Set to 0.0

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

**Error behavior:**

- If position interface declared but no `act_pos_*` actuator found: Throw error
- If velocity interface declared but no `act_vel_*` actuator found: Throw error
- If effort interface declared but no `act_tau_*` actuator found: Throw error

**Error messages:**

- Clear indication of which joint has the mismatch
- Specific actuator name expected
- Guidance on how to fix (add actuator or remove interface)

**Testing:** See `mujoco_ros2_control/test/README.md` for validation test suite

---

### 6. KV Warning System for MIT Mode Compatibility

**Problem:**

When a position actuator is neutralized (ctrl = q), the MuJoCo control law becomes: τ = kp*(q - q) - kv*qd = -kv*qd

If `kv ≠ 0`, this produces unwanted velocity damping that interferes with MIT mode torque control.

**Solution:**

Warn during initialization if position actuator has non-zero kv and effort interface is exposed.

**Warning message includes:**

- Current kv value
- Expected damping force magnitude
- Recommendation to set kv=0.0 for MIT mode compatibility

**Recommended configuration for MIT mode:**
Position actuators should have `kp="5000" kv="0"` (or appropriate kp value with kv=0)

**Testing:** See `mujoco_ros2_control/test/test_kv_warning.py`

---

### 7. Initial Pose Configuration (XML Keyframes)

**Added:** Support for loading initial joint configurations using native MuJoCo XML keyframes

**Purpose:** Start simulations from specific configurations (e.g., testing gravity compensation, collision avoidance, specific scenarios) using MuJoCo's built-in keyframe mechanism.

#### XML Keyframe Format

Define keyframes in your MuJoCo XML model file within a `<keyframe>` tag.

**Attributes:**

- `name` - Identifier for the keyframe (required)
- `qpos` - Space-separated joint positions in radians/meters (required)
- `qvel` - Space-separated joint velocities (optional, defaults to 0)

**Units:** Joint positions are in radians for revolute joints, meters for prismatic joints.

#### Launch File Configuration

Pass the keyframe name or index as a node parameter using `initial_keyframe` parameter. Can specify by name (e.g., "test_pose") or numeric index (e.g., "0").

#### Behavior

- If `initial_keyframe` parameter is provided, system loads that keyframe by name or index
- If parameter is not provided but keyframes exist in XML, loads first keyframe (index 0)
- If no keyframes defined or parameter missing, uses URDF default positions
- MuJoCo `mj_resetDataKeyframe()` is called to load keyframe state
- `mj_forward()` is called to propagate kinematics and update derived quantities
- Position commands are initialized from loaded qpos values

#### Runtime Reset Service

Reset to any keyframe during runtime using `/mujoco_ros2_control/reset_to_keyframe` service. Accepts keyframe name (string) or numeric index.

#### Example

See `mujoco_ros2_control_demos` for complete examples:

- Model: `mujoco_ros2_control_demos/mujoco_models/test_1dof_gravity.xml`
- Launch: `mujoco_ros2_control_demos/launch/test_1dof_gravity.launch.py`

---

### 8. Real-Time Synchronization Fix

**Problem (old):** Read/write called after stepping simulation, causing 1-step lag

**Solution (new):** Reordered update loop to apply controls before physics step

#### New Update Loop Order

1. Compute current sim time BEFORE stepping
2. Read state and update controllers BEFORE stepping
3. First half-step (kinematics, collision detection) - `mj_step1`
4. Apply commands BETWEEN step1 and step2 - `controller_manager->write`
5. Apply external wrenches (if any)
6. Second half-step (physics integration) - `mj_step2`
7. Publish clock AFTER stepping
8. Update last control time

**Benefits:**

- Zero-lag control: commands applied to the step being computed
- Proper causality: controller decisions based on state at time t affect physics at time t
- Matches real-time control loop behavior

**Documentation:** See `doc/REAL_TIME_SYNC_FIX.md` for detailed analysis

---

### 9. External Wrench Service

**Added:** New `mujoco_ros2_control_msgs` package with `ApplyExternalWrench.srv`

#### Service Definition

Service: `ApplyExternalWrench.srv`

**Request fields:**

- `body_name`: Name of the body to apply wrench to
- `wrench`: Force + torque (geometry_msgs/Wrench) in world frame
- `duration`: Duration to apply the wrench (seconds)

**Response fields:**

- `accepted`: True if the wrench was accepted
- `message`: Status message

#### Implementation

- Uses MuJoCo's `xfrc_applied` (external force/torque vector)
- Thread-safe with mutex protection
- Automatic expiration after duration
- Applied between `mj_step1` and `mj_step2` in update loop
- **Note:** Wrenches must be expressed in world frame

**Use cases:**

- Disturbance rejection testing
- External force simulation
- Debugging and validation

---

### 10. Auto-Compute Update Rate

**Previous:** Required manual `update_rate` parameter in controller manager config

**Current:** Automatically computed from MuJoCo timestep if not explicitly set

**Computation:** `update_rate = 1.0 / mj_model->opt.timestep`

- Example: MuJoCo timestep = 0.001s → update_rate = 1000 Hz

**Benefits:**

- Reduces configuration boilerplate
- Ensures synchronization between MuJoCo and ros2_control
- Can still be overridden explicitly if needed

---

### 11. Gravity Compensation Validation

**Added:** Publisher for `qfrc_bias` (MuJoCo's computed gravity/Coriolis/centrifugal forces)

Published after `mj_step2` in update loop as Float64MultiArray message.

**Purpose:**

- Validate gravity compensation implementations
- Compare controller-computed gravity terms with MuJoCo ground truth
- Debug Pinocchio integration in `zordi_mit_controller`

**Topic:** `/mujoco/qfrc_bias`

This allows real-time comparison between your controller's gravity compensation and MuJoCo's physics engine calculations.

---

### 12. Thread-Safe Clock Publishing

**Problem:** Multi-threaded architecture (controller manager in separate thread) could cause out-of-order clock messages

**Solution:** Mutex-protected monotonic clock guarantee

**Implementation:**

- Static last_published_time tracking
- Mutex protects clock publishing
- Skip publish if time hasn't advanced (prevents backward jumps)
- Only publish if sim_time > last_published_time

**Benefits:**

- Prevents RViz and other nodes from resetting due to backward time jumps
- Standard practice for multi-threaded ROS2 simulations

---

### 13. Simulation Control Service (Pause/Unpause/Reset)

**Added:** Runtime control of simulation execution state via ROS2 service

#### Service Definition

Service: `mujoco_ros2_control_msgs/srv/SimulationControl.srv`

**Request:** `command` (string) - "pause", "unpause", "reset", or "status"

**Response:**

- `success` (bool) - Whether command succeeded
- `message` (string) - Status message
- `current_state` (string) - "PAUSED" or "RUNNING"

#### Commands

1. **`status`** - Query current state without changing it (read-only)
2. **`pause`** - Freeze physics and time (controllers remain active)
3. **`unpause`** - Resume normal simulation execution
4. **`reset`** - Reset to initial keyframe and transition to PAUSED state

#### Initial State

**Simulation always starts PAUSED** - explicit unpause required to begin execution.

This allows:

- Inspection of initial configuration before starting
- Controller loading/activation without physics running
- Predictable, reproducible test setups

#### Paused State Behavior

When PAUSED:

- **Physics steps skipped:** `mj_step1()` and `mj_step2()` not executed
- **Simulation time frozen:** Clock publishes frozen time value
- **Controllers remain active:** `read/update/write` called with `period=0`
- **State interfaces frozen:** Joint positions/velocities unchanging

**Key insight:** Running controllers with `dt=0` and frozen state prevents:

- Controller activation timeouts (controllers can be loaded while paused)
- Integral windup (no time passes, so integrals don't accumulate)
- State machine issues (controller lifecycle works normally)

#### Implementation

**Update loop logic:**

- Check state with mutex protection
- If PAUSED:
  - Call controller_manager read/update/write with zero period
  - Publish frozen time
  - Return early (skip physics steps)
- If RUNNING:
  - Continue with normal execution (physics + controllers)

#### Usage Examples

**Service name:** `/simulation_control`

**Commands available:**

- Query state: `{command: 'status'}`
- Start simulation: `{command: 'unpause'}`
- Pause for inspection: `{command: 'pause'}`
- Reset to initial state: `{command: 'reset'}`

Use `ros2 service call` with `mujoco_ros2_control_msgs/srv/SimulationControl` or programmatic ROS2 service clients to control simulation state.

#### Use Cases

1. **Testing workflows:** Pause between test phases for data collection
2. **Controller validation:** Load/activate controllers while paused
3. **Debugging:** Freeze simulation to inspect state
4. **Experiment iteration:** Reset to initial conditions for repeated trials
5. **Safe parameter modification:** Pause, modify, resume

#### Thread Safety

- State transitions protected by `sim_state_mutex_`
- Safe concurrent access from service callbacks and main simulation loop
- Atomic state queries via `status` command

#### Reset Integration

The `reset` command integrates with the existing keyframe system:

- Uses `initial_keyframe` parameter configured in launch file
- Queues keyframe reset via existing `reset_to_keyframe` mechanism
- Automatically transitions to PAUSED state after reset
- Allows inspection before resuming with `unpause`

**Benefits:**

- Deterministic initial state for all tests
- No "race condition" at startup (controllers load while simulation frozen)
- Reproducible experiments with clean reset capability
- Debugging-friendly pause/inspect/resume workflow
- Compatible with all existing controllers and hardware interfaces

---

## Demo Package

### Getting Started

**Location:** `mujoco_ros2_control_demos/`

A comprehensive demo showcasing all major features:

- **1-DOF Gravity Compensation Demo** (`test_1dof_gravity.launch.py`)
  - Demonstrates actuator-centric control
  - MIT mode with gravity compensation
  - Initial pose configuration (keyframes)
  - Simulation control (pause/unpause/reset)
  - External wrench application
  - Gravity validation with `qfrc_bias` topic

**Quick Start:**

```bash
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control mujoco_ros2_control_demos zordi_mit_controller
source install/setup.bash
ros2 launch mujoco_ros2_control_demos test_1dof_gravity.launch.py
```

**For detailed usage examples and tutorials:**
See `mujoco_ros2_control_demos/README.md`

---

## Migration Guide

### For Existing Users

#### MuJoCo Model Changes

**Required naming convention:**

- Position actuators: `act_pos_{joint_name}`
- Velocity actuators: `act_vel_{joint_name}`
- Torque actuators: `act_tau_{joint_name}`

**Notes:**

- Must use exact naming convention
- For MIT mode, must add all three actuators
- Set `kv="0"` on position actuators for clean neutralization
- Old naming (e.g., `actuator_joint1`) is no longer supported

#### URDF Changes

**Old:** Conditional interfaces based on control mode

**New:** Always expose all interfaces you want to use in the `<ros2_control>` tag

**Joint interface requirements:**

- Expose command interfaces: position, velocity, and/or effort (as needed)
- Expose state interfaces: position, velocity, and/or effort (as needed)
- For MIT mode: Expose all three command interfaces (position, velocity, effort)

#### Controller Configuration

No changes needed - controllers work as before. Mode switching happens automatically based on which interfaces the controller claims.

---

## Backward Compatibility

### Breaking Changes

- **Required:** Actuator-based control - MuJoCo XML models must now include actuators with specific naming conventions (`act_pos_*`, `act_vel_*`, `act_tau_*`) for each URDF command interface. The previous implementation directly manipulated simulation state (`qpos`/`qvel`/`qfrc_applied`) without requiring actuators.
- **Changed:** Control application method - Commands are now applied through MuJoCo actuators instead of directly setting simulation state variables.

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
  - `ApplyExternalWrench.srv` - Apply forces/torques to bodies
  - `ResetToKeyframe.srv` - Reset simulation to keyframe
  - `SimulationControl.srv` - Pause/unpause/reset/status control

---

## Documentation

### New Documentation Files

- `doc/DAMIAO_POSITION_MODE.md` - Modeling DAMIAO position servo mode
- `doc/MODE_COMPARISON.md` - Comparison of control modes
- `doc/POSITION_SERVO_MODE_FINAL.md` - Position servo implementation
- `doc/REAL_TIME_SYNC_FIX.md` - Real-time synchronization analysis
- `doc/SIMULATION_CONTROL.md` - Simulation control service guide
- `doc/SIMULATION_CONTROL_INTEGRATION_EXAMPLE.md` - Integration examples and patterns
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

## Known Limitations

- **Simulation control service:** Service is at global namespace `/simulation_control` rather than node-namespaced
- **Reset command:** Requires `initial_keyframe` parameter to be configured; fails gracefully with error message if not set

## Future Work

1. **Body-frame wrenches:** Add support for body-frame wrench application in `ApplyExternalWrench` service
2. **Dynamic actuator gain tuning:** Runtime adjustment of kp/kv parameters
3. **Sensor support:** Expand IMU and force-torque sensor capabilities
4. **URDF loading:** Direct URDF to MuJoCo conversion (eliminate XML step)
5. **Simulation speed control:** Add service to adjust real-time factor (run faster/slower than real-time)

---

**Copyright 2025 Zordi, Inc. All rights reserved.**

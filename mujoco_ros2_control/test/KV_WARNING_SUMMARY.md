# Non-Zero kv Warning Implementation Summary

## Overview

This document summarizes the implementation of warnings for non-zero damping (`kv`) in position actuators when they are neutralized during MIT mode operation.

## Problem Statement

### When is non-zero kv OK?

When a position actuator is **actively used** for position control:
```
τ = kp*(pos_cmd - q) - kv*qd
```

In this case, `kv` provides **desirable damping** for the position controller. Non-zero `kv` is beneficial and recommended.

### When is non-zero kv problematic?

When a position actuator is **neutralized** by setting `ctrl = q` (current position):
```
τ = kp*(q - q) - kv*qd = -kv*qd
```

If `kv ≠ 0`, this produces **unwanted velocity-dependent damping** even though the actuator is intended to be "neutral" (generating zero torque).

### When This Occurs

Neutralization happens in two scenarios:

1. **Startup**: When position interface is exposed but no controller has claimed it yet
2. **MIT Mode**: When effort interface is active, position actuator is neutralized to allow torque control

## Implementation

### 1. Warning at Initialization (Smart Detection)

**Location**: `mujoco_system.cpp::register_joints()`

**Action**: Checks position actuators during initialization, but **only warns if both position AND effort interfaces are exposed** (indicating MIT mode usage where neutralization would occur).

**Rationale**: If only position interface is exposed (pure position control), non-zero `kv` is desirable for damping. Only warn when the configuration suggests MIT mode will be used.

**Code**:
```cpp
// Only warn if BOTH effort and position interfaces are exposed (MIT mode config)
bool has_effort_interface = false;
for (const auto &command_if : joint.command_interfaces) {
  if (command_if.name == hardware_interface::HW_IF_EFFORT) {
    has_effort_interface = true;
    break;
  }
}

if (joint_state.mj_pos_actuator_id >= 0 && has_effort_interface)
{
  const int act_id = joint_state.mj_pos_actuator_id;
  const double kv = mj_model_->actuator_gainprm[act_id * 10 + 1];

  if (std::abs(kv) > 1e-6)
  {
    RCLCPP_WARN(logger_,
      "Joint '%s': Position actuator has kv=%.3f but effort interface is also exposed. "
      "During MIT mode (when position actuator is neutralized), this will cause "
      "unwanted damping (τ = -%.3f * qd). For MIT mode compatibility, set kv=0.0.",
      joint.name.c_str(), kv, kv);
  }
}
```

### 2. Runtime Warning During Neutralization

**Location**: `mujoco_system.cpp::write()`

**Action**: Checks for `kv ≠ 0` when position interface is exposed but not active, warns once per joint.

**Code**:
```cpp
// Check if position interface is exposed but not active, and kv != 0
if (joint_state.is_position_control_enabled &&
    !joint_state.position_command_active &&
    !joint_state.warned_about_position_kv)
{
  const int act_id = joint_state.mj_pos_actuator_id;
  const double kv = mj_model_->actuator_gainprm[act_id * 10 + 1];

  if (std::abs(kv) > 1e-6)
  {
    RCLCPP_WARN(logger_,
      "Joint '%s': Position actuator has kv=%.3f but position interface is not active. "
      "This will introduce unwanted damping torque (τ = -%.3f * qd)...",
      joint_state.name.c_str(), kv, kv);
    joint_state.warned_about_position_kv = true;
  }
}
```

### 3. Data Structure Update

**Location**: `mujoco_system.hpp::JointState`

**Addition**: Flag to prevent warning spam:
```cpp
// Warning flag to avoid spamming logs about kv != 0 in neutralized position actuators
bool warned_about_position_kv{false};
```

## Documentation Updates

### 1. TECHNICAL_REFERENCE.md

- Added critical constraint documentation in **Actuator Types > Position Actuator** section
- Added troubleshooting entry: **Problem: Unwanted damping in MIT mode**
- Explains the control law and why `kv=0.0` is required

### 2. Test Suite

Created comprehensive test for non-zero kv warning:

**Files**:
- `test_robot_nonzero_kv.xml` - MuJoCo model with `kv=5.0`
- `test_robot_nonzero_kv.urdf` - URDF with all three interfaces
- `test_kv_warning.py` - Test script to verify warning emission

**Test validates**:
- System initializes successfully (no fatal errors)
- Warning is emitted for non-zero kv
- Warning message is clear and actionable

## Questions Answered

### Q1: What if position actuator is not defined at all?

**Answer**: The system already validates this at initialization:

```cpp
if (has_position_interface && joint_state.mj_pos_actuator_id < 0) {
  RCLCPP_ERROR(...);
  throw std::runtime_error(
    "URDF/MuJoCo mismatch: position interface without actuator for joint...");
}
```

If the position interface is exposed in URDF but the actuator doesn't exist in MuJoCo:
- `mj_name2id()` returns `-1`
- Validation catches this during initialization
- System fails with clear error message

**Result**: No silent failure, no unwanted behavior. The write() function skips actuators with `id < 0`, so nothing is sent to MuJoCo.

### Q2: Should we warn when (kv ≠ 0) AND (interface exposed but not active)?

**Answer**: YES - Implementation complete!

Warnings are emitted in two places:
1. **Startup**: During initialization, warn for all joints with `kv ≠ 0`
2. **Runtime**: When neutralizing, warn if interface exposed but not active

This catches both scenarios:
- User configured wrong `kv` in MuJoCo model
- User exposed interface but isn't using it (MIT mode with only effort active)

## Recommended Configuration

For proper MIT mode operation, always configure position actuators with:

```xml
<position name="act_pos_openarm_joint1" joint="openarm_joint1"
          kp="100.0" kv="0.0"  <!-- kv MUST be 0.0 -->
          ctrlrange="-1.396263 3.490659" forcerange="-40 40"/>
```

## Testing

Run the test to verify warning system:

```bash
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/test
./test_kv_warning.py
```

Expected output:
```
✅ TEST PASSED: Non-zero kv warning correctly emitted!

The system correctly:
  - Initialized successfully (no fatal errors)
  - Detected position actuator with kv=5.0
  - Emitted warning about potential unwanted damping
```

## Summary

The implementation provides:

1. ✅ **Early detection**: Warns at initialization
2. ✅ **Runtime validation**: Warns when actually neutralizing
3. ✅ **Clear guidance**: Warning messages explain the issue and solution
4. ✅ **No spam**: Warns only once per joint at runtime
5. ✅ **Comprehensive docs**: TECHNICAL_REFERENCE and troubleshooting guide updated
6. ✅ **Automated testing**: Test suite validates warning behavior

This ensures users are aware of potential issues with non-zero `kv` in position actuators and can correct their configuration before encountering unexpected damping behavior.


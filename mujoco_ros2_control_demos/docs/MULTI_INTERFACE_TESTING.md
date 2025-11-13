# Multi-Interface Controller Testing

**Date**: 2025-11-13
**Status**: Ready for testing

## Overview

This document describes the test setup for validating multi-interface control modes in the actuator-centric `mujoco_system.cpp` implementation.

## What We're Testing

### Single-Interface Modes (✅ COMPLETED)

- **Mode 1**: Position only → `position_controller` → Position actuator
- **Mode 2**: Velocity only → `velocity_controller` → Velocity actuator
- **Mode 3**: Effort only → `effort_controller` → Torque actuator

All three modes validated successfully in isolated tests.

### Multi-Interface Modes (🔄 IN PROGRESS)

- **Mode 4-5**: Position + Velocity → `joint_trajectory_controller` → Position + velocity actuators
- **Mode 6-7**: Position + Velocity + Effort → `zordi_mit_controller` → MIT mode with PD composition

## Test Configuration

### Files Created

```
mujoco_ros2_control_demos/
├── config/
│   └── test_1dof_mit.yaml                    # Controller configuration
├── launch/
│   └── test_1dof_mit.launch.py              # Launch file
└── scripts/
    ├── test_joint_trajectory_controller.sh  # Manual test: JTC
    ├── test_zordi_mit_controller.sh         # Manual test: zordi
    └── test_multi_interface_modes.sh        # Automated comprehensive test
```

## How to Run Tests

### Prerequisites

1. **Build the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select mujoco_ros2_control zordi_mit_controller mujoco_ros2_control_demos
   source install/setup.bash
   ```

2. **Ensure zordi_mit_controller is installed**:
   ```bash
   ros2 pkg list | grep zordi_mit_controller
   # Should show: zordi_mit_controller
   ```

### Method 1: Automated Comprehensive Test (Recommended)

**Terminal 1** - Launch test environment:
```bash
ros2 launch mujoco_ros2_control_demos test_1dof_mit.launch.py
```

**Terminal 2** - Run comprehensive test:
```bash
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts
./test_multi_interface_modes.sh
```

The script will:
1. Test `joint_trajectory_controller` (position + velocity)
2. Verify tracking accuracy
3. Test `zordi_mit_controller` (position + velocity + effort = MIT mode)
4. Verify tracking accuracy
5. Display summary of results

### Method 2: Manual Testing

**Terminal 1** - Launch test environment:
```bash
ros2 launch mujoco_ros2_control_demos test_1dof_mit.launch.py
```

**Terminal 2** - Test joint_trajectory_controller:
```bash
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts
./test_joint_trajectory_controller.sh
# Follow on-screen instructions
```

**Terminal 2** - Test zordi_mit_controller:
```bash
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts
./test_zordi_mit_controller.sh
# Follow on-screen instructions
```

## What to Look For

### In MuJoCo Logs (Terminal 1)

Look for `[ACTIVE_DBG]` messages showing interface states:

**joint_trajectory_controller (NOT MIT mode)**:
```
[ACTIVE_DBG] t=1.0s j=j1 pos_active=1 vel_active=1 eff_active=0 mit_mode=0
```
- `pos_active=1, vel_active=1, eff_active=0`
- `mit_mode=0` (FALSE)
- Position and velocity actuators driven
- Torque actuator neutralized (0.0)

**zordi_mit_controller (MIT mode)**:
```
[ACTIVE_DBG] t=5.0s j=j1 pos_active=1 vel_active=1 eff_active=1 mit_mode=1
```
- `pos_active=1, vel_active=1, eff_active=1`
- `mit_mode=1` (TRUE)
- Position/velocity actuators neutralized (ctrl_pos=q, ctrl_vel=qd)
- Torque actuator driven with PD composition: `τ = Kp*(pos_cmd-q) + Kd*(vel_cmd-qd) + effort_cmd`

### Expected Behavior

**joint_trajectory_controller**:
- Smooth trajectory tracking using position + velocity actuators
- Joint follows commanded positions
- No MIT mode engaged

**zordi_mit_controller**:
- MIT mode engaged automatically
- PD gains applied by hardware interface
- Position/velocity commands converted to torque via PD
- Effort feedforward (gravity) added by controller
- Smooth trajectory tracking

## Key Implementation Details

### zordi_mit_controller Configuration

From `test_1dof_mit.yaml`:

```yaml
zordi_mit_controller:
  ros__parameters:
    joints: [j1]
    command_interfaces: [position, velocity, effort]  # ← Triggers MIT mode
    state_interfaces: [position, velocity]
    use_gravity_compensation: false  # No gravity on horizontal test joint
```

**Why this triggers MIT mode**:
1. Controller claims all three interfaces: `j1/position`, `j1/velocity`, `j1/effort`
2. `perform_command_mode_switch()` sets all three `*_command_active` flags to true
3. `write()` detects: `effort_command_active && (position_command_active || velocity_command_active)`
4. MIT mode logic activates: neutralize pos/vel actuators, drive torque actuator with PD

### Hardware Interface Logic

From `mujoco_system.cpp` lines 125-126:

```cpp
bool mit_mode = joint_state.effort_command_active &&
                (joint_state.position_command_active || joint_state.velocity_command_active);
```

When `mit_mode = true`:
- Position actuator: `ctrl[pos_actuator_id] = q` (neutralize)
- Velocity actuator: `ctrl[vel_actuator_id] = qd` (neutralize)
- Torque actuator: `ctrl[tau_actuator_id] = Kp*(pos_cmd-q) + Kd*(vel_cmd-qd) + effort_cmd`

## Troubleshooting

### "Controller not found"

If `zordi_mit_controller` is not found:
```bash
# Rebuild and source
cd ~/ros2_ws
colcon build --packages-select zordi_mit_controller
source install/setup.bash
```

### "Joint not moving"

Check:
1. Controller is active: `ros2 control list_controllers`
2. Commands are being sent: Check test script output
3. MuJoCo logs show interface activation
4. Joint state topic: `ros2 topic echo /joint_states`

### "Tracking error too high"

- Check PID gains in URDF (test_1dof_multimode.xacro.urdf)
- Verify actuator configuration in MuJoCo XML
- Check for joint limits or saturation

## Success Criteria

✅ **joint_trajectory_controller**:
- [ ] Controller loads and activates successfully
- [ ] Claims position + velocity interfaces
- [ ] MIT mode = FALSE in logs
- [ ] Joint tracks commanded position within 0.05 rad
- [ ] No torque saturation warnings

✅ **zordi_mit_controller**:
- [ ] Controller loads and activates successfully
- [ ] Claims position + velocity + effort interfaces
- [ ] MIT mode = TRUE in logs
- [ ] Position/velocity actuators neutralized (ctrl=state)
- [ ] Torque actuator shows PD composition
- [ ] Joint tracks commanded position within 0.05 rad
- [ ] No torque saturation warnings

## Next Steps After Validation

Once multi-interface testing is complete:

1. **Test with full OpenARM model** (7-DOF)
2. **Verify gravity compensation** with `zordi_mit_controller`
3. **Test controller switching** (trajectory → MIT → effort)
4. **Validate system identification toolbox** with known dynamics
5. **Production deployment** with real hardware

## References

- TODO.md - Current status and next steps
- ACTUATOR_CENTRIC_CONTROL.md - Design document
- mujoco_system.cpp - Hardware interface implementation
- zordi_mit_controller - Multi-interface trajectory controller with gravity compensation

---

**Copyright 2025 Zordi, Inc. All rights reserved.**


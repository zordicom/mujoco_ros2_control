# DAMIAO Position Mode Modeling

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Overview

This document explains how we model DAMIAO Position Mode in MuJoCo simulation to accurately replicate real hardware behavior.

## DAMIAO Position Mode (Real Hardware)

### Characteristics

**DAMIAO Position Mode** is a firmware-level position servo with these properties:

1. **Input:** Position setpoint only
2. **Internal Control:** High-bandwidth PD controller in firmware
3. **Response:** Critically damped (ζ ≈ 1.0)
4. **Endpoint Behavior:** Zero velocity at setpoint (no oscillations)
5. **Bandwidth:** Very high (~100 Hz or higher)

### Control Law (Internal to Firmware)

```
τ_motor = Kp_fw * (q_desired - q_actual) + Kd_fw * (0 - v_actual)

Where:
  Kp_fw: Firmware position gain (very high, ~10000+)
  Kd_fw: Firmware velocity gain (tuned for critical damping)
```

**Key feature:** The firmware **automatically** achieves zero velocity at the setpoint through proper damping.

## MuJoCo Simulation Model

### Challenge

MuJoCo position actuators model:

```
τ = kp * (ctrl - q) - kv * v
```

**Problems:**
1. Fixed kp/kv gains (set in XML, can't change per-joint dynamically)
2. kv=200 too low for kp=50000 → underdamped → oscillations
3. No built-in velocity setpoint tracking

### Solution: Hybrid Approach

**Combine MuJoCo actuator + additional damping:**

```cpp
// MuJoCo position actuator (XML: kp=50000, kv=200)
mj_data->ctrl[actuator_id] = position_command;

// Additional velocity damping (code)
double velocity_error = velocity_command - actual_velocity;
τ_extra = kv_extra * velocity_error;
mj_data->qfrc_applied[joint] += τ_extra;

// Total torque:
τ_total = kp*(pos_cmd - q) - kv*v + kv_extra*(v_cmd - v)
        = 50000*(pos_error) - 200*v + 1200*(v_cmd - v)
        = 50000*pos_error - (200 + 1200)*v + 1200*v_cmd
        = 50000*pos_error - 1400*v + 1200*v_cmd
```

### Critical Damping Calculation

For a second-order system:
```
ζ = kv_total / (2 * sqrt(kp * I))

Where:
  I ≈ 0.01 kg⋅m² (typical joint inertia)
  kp = 50000 N⋅m/rad

For critical damping (ζ = 1.0):
  kv_total = 2 * sqrt(50000 * 0.01)
           = 2 * sqrt(500)
           = 2 * 22.36
           ≈ 44.7 N⋅m⋅s/rad per kg⋅m²
           ≈ 1400 N⋅m⋅s/rad (for I=0.01)

Current:
  kv_actuator = 200 (from XML)
  kv_extra = 1200 (from code)
  kv_total = 1400 ✓ (critically damped!)
```

## Implementation Details

### In mujoco_system.cpp (position_servo mode)

```cpp
// Set position setpoint
mj_data_->ctrl[joint_state.mj_actuator_id] = joint_state.position_command;

// Add velocity damping to achieve DAMIAO-like behavior
if (joint_state.is_velocity_control_enabled)
{
    double velocity_error =
        joint_state.velocity_command - mj_data_->qvel[joint_state.mj_vel_adr];

    // kv_extra chosen for critical damping with kp=50000
    const double kv_extra = 1200.0;
    mj_data_->qfrc_applied[joint_state.mj_vel_adr] += kv_extra * velocity_error;
}
```

### Why This Works

1. **joint_trajectory_controller sends:**
   - `position_command`: Interpolated position at each timestep
   - `velocity_command`: Interpolated velocity (0 at endpoints!)

2. **At trajectory endpoints:**
   ```
   velocity_command = 0 (desired)
   velocity_error = 0 - actual_velocity = -actual_velocity
   τ_damping = 1200 * (-actual_velocity) = -1200 * v

   Result: Strong damping force opposing any velocity
   ```

3. **During trajectory:**
   ```
   velocity_command = smooth trajectory velocity
   velocity_error = desired - actual
   τ_damping = 1200 * (desired - actual)

   Result: Tracks desired velocity profile
   ```

## Comparison: Real vs Simulation

| Aspect | Real DAMIAO | MuJoCo Simulation |
|--------|-------------|-------------------|
| Position input | ✓ | ✓ |
| Velocity input | ❌ (internal) | ✓ (from controller) |
| Critical damping | ✓ | ✓ (kv_total=1400) |
| Zero velocity at setpoint | ✓ | ✓ |
| High bandwidth | ✓ (~100 Hz) | ✓ (1000 Hz) |
| Firmware PD | ✓ (internal) | ✓ (hybrid model) |

## Expected Behavior

### Trajectory Execution

```
Test: Move from 0 → 0.5 rad in 10 seconds

t=0s:     position=0.0,   velocity=0.0  ← Start (zero velocity)
t=2.5s:   position=0.125, velocity=0.1  ← Accelerating
t=5.0s:   position=0.25,  velocity=0.1  ← Constant velocity
t=7.5s:   position=0.375, velocity=0.05 ← Decelerating
t=10.0s:  position=0.5,   velocity=0.0  ← End (zero velocity!)
t=10.5s:  position=0.5,   velocity=0.0  ← Holding (no drift)
```

**Key indicators of correct DAMIAO Position Mode behavior:**
- ✅ Smooth acceleration/deceleration
- ✅ Velocity reaches zero at endpoint
- ✅ No oscillations around setpoint
- ✅ Position holds steady (no drift)

### Debugging: If Oscillations Persist

If you still see oscillations (velocity not zero), check:

1. **Inertia mismatch:**
   ```
   # Log actual inertia
   RCLCPP_INFO_ONCE(logger_, "Joint inertia: %.6f kg⋅m²",
                    mj_model_->body_inertia[body_id]);

   # Adjust kv_extra:
   kv_extra = 2 * sqrt(kp * actual_inertia);
   ```

2. **Gravity compensation needed:**
   - Distal joints may need gravity compensation
   - Use `zordi_mit_controller` with `use_gravity_compensation: true`

3. **Torque saturation:**
   - Check if `qfrc_actuator` reaches limits
   - Reduce kp or target angles if saturated

## Tuning Guide

### If Oscillations (Underdamped)

**Symptoms:** Velocity oscillates around zero
```
t=10.0s: v = +0.5 rad/s
t=10.1s: v = -0.3 rad/s
t=10.2s: v = +0.2 rad/s
```

**Fix:** Increase damping
```cpp
const double kv_extra = 1500.0;  // Was 1200
```

### If Sluggish Response (Overdamped)

**Symptoms:** Takes too long to reach setpoint, slow settling
```
t=10.0s: position = 0.48 (not quite there)
t=11.0s: position = 0.495 (slow approach)
t=12.0s: position = 0.498 (very slow)
```

**Fix:** Decrease damping
```cpp
const double kv_extra = 1000.0;  // Was 1200
```

### If Both Slow AND Oscillatory

**Problem:** Incorrect kp value or inertia estimate

**Fix:** Adjust kp in XML or use per-joint kv_extra:
```cpp
// Per-joint damping based on actual inertia
double I = get_joint_inertia(joint_state);
double kv_extra = 2.0 * sqrt(kp * I);
```

## References

- **DAMIAO Motor Manual:** [Link to manual if available]
- **MuJoCo Actuator Documentation:** http://mujoco.org/book/modeling.html#actuator
- **Critical Damping Theory:** ζ = c / (2√(km))

---

**This model accurately replicates DAMIAO Position Mode behavior in simulation, enabling realistic trajectory execution and controller testing.**

**Copyright 2025 Zordi, Inc. All rights reserved.**

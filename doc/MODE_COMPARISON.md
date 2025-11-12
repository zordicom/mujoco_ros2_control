# Control Mode Comparison

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Current Implementation

Both modes now use **software PD control** writing to `qfrc_applied`. The only difference is **gain values**.

### MIT Mode (lines 147-177)

```cpp
double torque = 0.0;

// Position term (from URDF parameters)
if (is_position_enabled) {
    torque += position_pid.computeCommand(pos_error, dt);
    // Uses: kp=20, kd=2, ki=0 (from URDF)
}

// Velocity term (from URDF parameters)
if (is_velocity_enabled) {
    torque += velocity_pid.computeCommand(vel_error, dt);
    // Uses: kp=2, kd=0.2, ki=0 (from URDF)
}

// Effort feedforward
if (is_effort_enabled) {
    torque += effort_command;
}

mj_data->qfrc_applied[joint] = torque;
```

**Effective gains:**
- Position: Kp = 20, Kd = 2
- Can add effort feedforward (e.g., gravity compensation)

### position_servo Mode (lines 178-213)

```cpp
double torque = 0.0;

// Position term (hardcoded high gains)
if (is_position_enabled) {
    torque += 5000.0 * pos_error;
}

// Velocity term (hardcoded, tuned for position gains)
if (is_velocity_enabled) {
    torque += 500.0 * vel_error;
}

// No effort feedforward

mj_data->qfrc_applied[joint] = torque;
```

**Gains:**
- Position: Kp = 5000, Kd = 500
- No effort feedforward

## Key Differences

| Aspect | MIT Mode | position_servo Mode |
|--------|----------|---------------------|
| **Position gain (Kp)** | 20 (compliant) | 5000 (stiff) |
| **Velocity gain (Kd)** | 2 | 500 |
| **Stiffness ratio** | 1x | 250x |
| **Effort feedforward** | ✅ Yes | ❌ No |
| **Gain source** | URDF params | Hardcoded |
| **PID integral** | Yes (Ki=0) | No |
| **Use case** | Compliant control | Stiff position tracking |

## Are They Too Similar?

**Yes and no:**

### Yes - Same Structure
- Both use software PD control
- Both write to qfrc_applied
- Both track position + velocity

### No - Different Behavior
```
Same 0.1 rad position error:

MIT mode:        τ = 20 * 0.1 = 2 N⋅m      (gentle)
position_servo:  τ = 5000 * 0.1 = 500 N⋅m  (aggressive)

Result: 250x different response!
```

## The Design Question

**Should position_servo use MuJoCo actuators or software PD?**

### MuJoCo Actuators (Original Intent)

```cpp
// Write to actuator control signal
mj_data_->ctrl[actuator_id] = position_command;

// MuJoCo computes: τ = kp*(ctrl - q) - kv*v
// Stored in qfrc_actuator
```

**Problems:**
- ❌ No velocity setpoint tracking
- ❌ Oscillations inevitable
- ❌ Cannot achieve zero velocity at endpoints

### Software PD (Current)

```cpp
τ = kp*pos_error + kd*vel_error;
mj_data_->qfrc_applied[joint] = τ;
```

**Advantages:**
- ✅ Tracks velocity setpoints
- ✅ Zero velocity at endpoints
- ✅ No oscillations
- ✅ Matches DAMIAO firmware behavior

## Recommendation: Keep Software PD

**Rationale:**
1. **Real DAMIAO firmware is software control** (not a physical actuator)
2. **Velocity tracking is essential** for trajectory execution
3. **MuJoCo actuators are fundamentally limited** for this use case

**The name "position_servo" is still appropriate** - it refers to the control behavior (stiff position tracking), not the implementation method.

## Future: Add Gravity Compensation to position_servo?

DAMIAO Position Mode likely includes gravity compensation in firmware. We could add:

```cpp
// position_servo mode
double torque = 0.0;

// PD control
if (is_position_enabled) torque += kp * pos_error;
if (is_velocity_enabled) torque += kd * vel_error;

// Gravity compensation (optional - matches firmware behavior)
// torque += compute_gravity_torque(joint);  ← Add this?

mj_data->qfrc_applied[joint] = torque;
```

This would make it even more firmware-like!

---

**Current implementation is correct and matches DAMIAO Position Mode behavior. No accidental changes detected.**

**Copyright 2025 Zordi, Inc. All rights reserved.**

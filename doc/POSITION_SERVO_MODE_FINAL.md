# Position Servo Mode - Final Implementation

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Complete Implementation

### DAMIAO Position Mode (position_servo)

```cpp
τ = Kp*(q_desired - q_actual) + Kd*(v_desired - v_actual) + g(q)
  = 5000*pos_error + 500*vel_error + qfrc_bias
```

**Components:**
1. **Position tracking:** Kp = 5000 N⋅m/rad (high stiffness)
2. **Velocity tracking:** Kd = 500 N⋅m⋅s/rad (critically damped)
3. **Gravity compensation:** Automatic (from MuJoCo qfrc_bias)

### DAMIAO MIT Mode (mit)

```cpp
τ = position_pid(pos_error) + velocity_pid(vel_error) + effort_cmd
  = (20*pos_error + 2*vel_error) + effort_cmd
```

**Components:**
1. **Position tracking:** Kp = 20 N⋅m/rad (compliant, from URDF)
2. **Velocity tracking:** Kd = 2 N⋅m⋅s/rad (from URDF)
3. **Effort feedforward:** Controller-provided (e.g., gravity from zordi_mit_controller)

## Key Differences

| Feature | position_servo | MIT Mode |
|---------|---------------|----------|
| **Stiffness** | High (Kp=5000) | Low (Kp=20) |
| **Gravity comp** | ✅ Automatic (firmware-like) | Manual (controller sends) |
| **Effort input** | ❌ No | ✅ Yes |
| **Tuning** | Hardcoded | URDF parameters |
| **Use case** | Stiff trajectory tracking | Compliant / custom control |

## Why This Matches Real Hardware

### Real DAMIAO Position Mode Firmware

**Features:**
- Position setpoint input
- High-bandwidth PD control
- **Built-in gravity compensation** ← Key!
- Critically damped response
- Zero velocity at setpoints

**Our simulation now matches all of these!**

### Real DAMIAO MIT Mode

**Features:**
- Position, velocity, effort inputs
- Lower gains (user-tunable)
- **NO automatic gravity comp** (user must provide)
- Allows force control
- Compliant behavior

**Our simulation matches this too!**

## Why Gravity Compensation is Critical

### Without Gravity Compensation (Old)

```
At q=0 (horizontal arm):
  Gravity torque: g(q) ≈ 30 N⋅m (pulling down)
  Position error: 0.01 rad
  Controller output: τ = 5000*0.01 = 50 N⋅m

  Net: 50 - 30 = 20 N⋅m (moves up, but weak)

At q=0.5 rad:
  Gravity: ≈ 25 N⋅m
  Position error: 0.02 rad (sagging)
  Output: 5000*0.02 = 100 N⋅m

  Net: 100 - 25 = 75 N⋅m (okay)

Problem: Position error varies with configuration!
```

### With Gravity Compensation (New)

```
At q=0:
  Gravity: 30 N⋅m
  Grav comp: +30 N⋅m (cancels out!)
  Position error: 0.001 rad (very small)
  Output: 5000*0.001 + 0 + 30 = 35 N⋅m

At q=0.5:
  Gravity: 25 N⋅m
  Grav comp: +25 N⋅m (cancels!)
  Position error: 0.001 rad (very small)
  Output: 5000*0.001 + 0 + 25 = 30 N⋅m

Result: Consistent small errors, stable everywhere! ✓
```

## Implementation Details

### Source of Gravity (MuJoCo qfrc_bias)

MuJoCo computes `qfrc_bias` after `mj_step1()`:

```
qfrc_bias = -(gravity + Coriolis/centrifugal)
```

**Sign convention:** MuJoCo uses **negative** bias, so we add **positive** to compensate:

```cpp
torque += mj_data_->qfrc_bias[joint];  // Already negative, adding compensates
```

### Update Order (Correct)

```cpp
// In mujoco_ros2_control.cpp::update()
mj_step1();           // Computes qfrc_bias
controller->read();   // Reads positions
controller->update(); // Computes commands
controller->write();  // → Our code writes torques using qfrc_bias
mj_step2();          // Applies torques, integrates
```

**Timing is correct:** We read qfrc_bias from current state, apply in next step.

## Expected Behavior Now

### At Startup (t=0)

```
position_command = 0 (home)
position_actual = 0
velocity = 0
gravity = ~30 N⋅m (horizontal arm)

τ = 5000*0 + 500*0 + 30 = 30 N⋅m

Result: Holds against gravity, no sagging ✓
```

### During Trajectory (t=5s)

```
position_command = 0.25 rad
position_actual = 0.24 rad
velocity_command = 0.1 rad/s
velocity_actual = 0.095 rad/s
gravity ≈ 28 N⋅m

τ = 5000*(0.01) + 500*(0.005) + 28
  = 50 + 2.5 + 28
  = 80.5 N⋅m

Result: Smooth tracking with gravity compensation ✓
```

### At Endpoint (t=10s)

```
position_command = 0.5 rad
position_actual = 0.5 rad
velocity_command = 0.0 rad/s
velocity_actual = 0.0 rad/s
gravity ≈ 25 N⋅m

τ = 5000*0 + 500*0 + 25
  = 25 N⋅m

Result: Holds perfectly with zero velocity ✓
```

## Final Comparison

### position_servo (DAMIAO Position Mode)

```cpp
τ = 5000*pos_error + 500*vel_error + g(q)
```

**Characteristics:**
- Stiff position tracking (250x stiffer than MIT)
- Automatic gravity compensation
- No effort input needed
- Matches DAMIAO Position Mode firmware

### MIT (DAMIAO MIT Mode)

```cpp
τ = 20*pos_error + 2*vel_error + effort_cmd
```

**Characteristics:**
- Compliant (low gains)
- Manual gravity compensation (controller computes)
- Effort input available
- Matches DAMIAO MIT Mode

---

## Test Instructions

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Start simulation (robot should NOT fall at startup!)
ros2 launch openarm_description single_arm.launch.py

# Test trajectory (should complete smoothly!)
python3 scripts/test_position_modes.py
```

**Expected:**
- ✅ Robot stable at startup (gravity compensation working)
- ✅ Smooth 10-second trajectory execution
- ✅ Velocities → 0 at endpoint
- ✅ Action completes successfully
- ✅ No controller loading timeouts

**Copyright 2025 Zordi, Inc. All rights reserved.**

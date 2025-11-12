# MuJoCo ROS2 Control Updates

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Summary (Current Branch vs Main)

**Files Modified:** 2
**Changes:** +134 insertions, -169 deletions (net: -35 lines, simplified)

---

## Key Changes

### 1. Dynamic Control Mode Switching

**Added:** Automatic mode switching based on active controller

**Implementation:**

```cpp
// New variable tracks current mode
std::string current_motor_mode_{"mit"};  // Starts in MIT mode

// perform_command_mode_switch() detects interface pattern
if (has_position && has_velocity && !has_effort)
  current_motor_mode_ = "position_servo";
else
  current_motor_mode_ = "mit";  // Default
```

**Behavior:**

- joint_trajectory_controller activates → switches to position_servo mode
- effort_controller activates → switches to mit mode
- zordi_mit_controller activates → stays in mit mode
- Logs: "Motor mode switch: mit → position_servo (interfaces: pos=1 vel=1 eff=0)"

---

### 2. Two Control Modes (Simplified from Complex Logic)

**position_servo mode:**

```cpp
// Uses MuJoCo position actuators
if (mj_actuator_id >= 0)
  mj_data->ctrl[actuator_id] = position_command;
```

- Triggered: pos+vel interfaces (no effort)
- Implementation: MuJoCo actuators with kp=50000
- Purpose: Matches DAMIAO Position Mode

**mit mode (default):**

```cpp
// Combines all active components
τ = 0;
if (is_position_enabled) τ += Kp * (pos_cmd - pos);
if (is_velocity_enabled) τ += Kd * (vel_cmd - vel);
if (is_effort_enabled) τ += effort_cmd;
mj_data->qfrc_applied[joint] = τ;
```

- Triggered: Any other interface combination
- Implementation: Manual PID + effort feedforward
- Purpose: Matches DAMIAO MIT Mode

---

### 3. Full MIT Mode Support

**Before:** Position/velocity and effort were mutually exclusive

```cpp
// OLD (broken for MIT mode)
if (position_active && !velocity_active && !effort_active)
  apply_position_control();
```

**After:** All three can work together

```cpp
// NEW (proper MIT mode)
τ_total = Kp*(pos_cmd - pos) + Kd*(vel_cmd - vel) + effort_cmd;
// All three components combined additively
```

**Impact:** Controllers can now claim all three interfaces (matches real DAMIAO MIT mode)

---

### 4. Position Actuator Support

**Added to JointState:**

```cpp
int mj_actuator_id{-1};  // MuJoCo actuator ID
```

**In register_joints():**

```cpp
// Map each joint to its actuator
actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "actuator_" + joint.name);
joint_state.mj_actuator_id = actuator_id;
```

**Purpose:** Enable position_servo mode using MuJoCo's built-in position actuators

---

### 5. Always Enable All Interfaces

**Before:** Conditional enabling based on control_mode parameter

```cpp
bool enable = (control_mode_ == "all" || control_mode_ == "position");
is_position_control_enabled = enable;
```

**After:** Always enable (mode switches dynamically)

```cpp
is_position_control_enabled = true;  // Always
is_velocity_control_enabled = true;
is_effort_control_enabled = true;
```

**Impact:** Interfaces always available, mode determines behavior in write()

---

### 6. Simplified write() Logic

**Removed:**

- Complex boolean conditions for mutual exclusion
- position_command_active checks in multiple places
- Nested control flow

**Result:**

- Cleaner two-mode switch statement
- Easier to understand and maintain
- Better matches real hardware implementation

---

## Backward Compatibility

**Breaking changes:**

- Removed "position", "velocity", "effort" control_mode values (only "all" supported)
- control_mode parameter is deprecated (kept for compatibility but unused)
- current_motor_mode_ is now the source of truth

**Migration:**

- Old code using `control_mode:=all` continues to work
- Dynamic switching happens automatically
- No URDF changes needed

---

## Testing

```bash
# Build and test
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control

ros2 launch openarm_description single_arm.launch.py

# Check mode switching
ros2 control switch_controllers --activate effort_controller
# Watch logs for: "Motor mode switch: position_servo → mit"
```

---

## Related Changes

**Requires:** Updated openarm_description package

- Standard interface names (position, velocity, effort)
- Position actuators in MuJoCo XML (kp=50000)
- Updated controller configs

**Enables:** New zordi_mit_controller package

- Claims all three interfaces simultaneously
- Full MIT mode with gravity compensation

---

**Copyright 2025 Zordi, Inc. All rights reserved.**

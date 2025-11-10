# MuJoCo ROS2 Control Updates

## Summary of Changes (Current Branch vs Main)

**Files Modified**: 4 files, +254 insertions, -12 deletions

### 1. Dynamic Controller Switching via ros2_control API

**New Methods** (`mujoco_system.hpp` + `mujoco_system.cpp`):

- `prepare_command_mode_switch()` - Validation phase before switching controllers
- `perform_command_mode_switch()` - Actual switch execution, parses interface names and updates active flags

**Behavior**:

- Receives explicit signals from controller manager when controllers activate/deactivate
- Parses interface names (e.g., `"openarm_joint1/velocity"`) to determine which control mode to enable
- Immediately switches active flags without timeout delays
- Only active in `control_mode:=all` mode; bypassed in specific control modes

### 2. Control Mode Selection Parameter

**New Parameter**: `control_mode` (read from hardware_info at initialization)

- `"all"` (default) - Enables all three interfaces, supports dynamic switching
- `"position"` - Only position control enabled
- `"velocity"` - Only velocity control enabled
- `"effort"` - Only effort control enabled

**Implementation**:

- Read in `register_joints()` from URDF/hardware parameters
- Controls which command interfaces are enabled via `is_*_control_enabled` flags
- Determines whether dynamic switching is allowed

### 3. Active Command Interface Tracking

**New Fields** (`JointState` struct):

- `position_command_active` - Position control is actively commanding
- `velocity_command_active` - Velocity control is actively commanding
- `effort_command_active` - Effort control is actively commanding

**Startup Behavior**:

- `position_command_active = true` by default (prevents robot collapse during startup)
- Other modes start inactive
- Controller manager switches them via `perform_command_mode_switch()`

### 4. Conditional Control Application in write()

**Previous**: Always applied enabled control modes simultaneously

**New**: Applies control based on active flags:

```cpp
apply_position = is_position_control_enabled &&
                 (control_mode == "position" ||
                  (control_mode == "all" && position_command_active &&
                   !velocity_command_active && !effort_command_active))
```

**Mutual Exclusion**: Only one control mode applies per joint at a time in "all" mode

### 5. Debug Logging

Added periodic logging (every 500 cycles) for joint1:

- Position control: `apply_pos`, `is_enabled`, `mode`, `active`, `cmd`, `cur`, `period_ns`
- Velocity control: `apply_vel`, `is_enabled`, `active`, `cmd`
- PID control: `error`, `torque`, `vel`
- Controller switches: "Position interface STARTED/STOPPED for X"

### 6. Simulation Timing Fixes (`mujoco_ros2_control.cpp`)

**Critical Change**: Moved `mj_step1()` to BEFORE reading simulation time

**Previous Order**:

1. Read sim time
2. Publish clock
3. Step simulation
4. Read/write controllers

**New Order**:

1. Step simulation first
2. Read NEW sim time (after step)
3. Publish clock with correct time
4. Read/write controllers

**Rationale**: Ensures published clock matches actual simulation state

### 7. Thread-Safe Clock Publishing

**Added**:

- Static `last_published_time` with mutex
- Monotonic clock guarantee (never publish time going backwards)
- Prevents RViz and other nodes from resetting due to out-of-order messages

**Why Needed**: Controller manager runs in separate thread (`cm_executor_`), creating race conditions

### 8. Plugin Export (`package.xml`)

**Added**: Proper plugin export declaration

```xml
<mujoco_ros2_control plugin="${prefix}/mujoco_system_plugins.xml"/>
```

Allows pluginlib to discover the MujocoSystem implementation

## When Would Controller Switching Error Out in Sim?

**Current Implementation**: `prepare_command_mode_switch()` always returns `OK`

**Potential Error Scenarios** (not currently implemented):

1. ❌ **Interface doesn't exist** - Would need validation that joint/interface exists
2. ❌ **Invalid transition** - Could reject unsafe mode switches (e.g., effort→position without zero force)
3. ❌ **Resource conflicts** - Multiple controllers trying to claim same interface
4. ❌ **Safety violations** - Custom safety checks (joint limits, collision detection)

**In Practice for Simulation**:

- **Never errors** with current implementation
- Simulation has no physical constraints or safety concerns
- All interfaces are virtual and always available
- Only way to error would be to explicitly add validation logic

**For Real Hardware**: Would want to add checks for:

- Hardware safety limits
- Communication with actuators
- Sensor availability
- Emergency stop states

## Key Design Decisions

1. **Position control active at startup** - Prevents robot collapse during controller loading (1-2 second window)
2. **No timeout detection** - Relies entirely on controller manager callbacks (cleaner, instant switching)
3. **Mutual exclusion in "all" mode** - Only one control mode active per joint (prevents conflicts)
4. **Simulation step ordering** - Step first, then read time (correct clock synchronization)

## Testing Recommendations

1. Verify instant switching with debug logs: `ros2 control switch_controllers`
2. Test startup stability (robot should hold initial pose)
3. Check clock monotonicity in RViz (no jumps or resets)
4. Validate each control mode works after switching

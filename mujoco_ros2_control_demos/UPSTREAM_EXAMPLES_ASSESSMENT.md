# Upstream Examples Assessment - Actuator-Centric Design Compatibility

**Date:** 2025-11-14
**Branch:** 2025-11-control-interface
**Issue:** Upstream example MuJoCo models missing required actuators

---

## Summary

**🔴 CRITICAL: All upstream examples are BROKEN and will fail to launch.**

The actuator-centric control design implemented in this branch requires that:
1. Each command interface declared in URDF must have a corresponding actuator in the MuJoCo model
2. Actuators must follow naming convention: `act_pos_{joint}`, `act_vel_{joint}`, `act_tau_{joint}`

**None of the upstream example MuJoCo XML files have ANY actuators defined.**

---

## Affected Examples

### 🔴 Broken - Missing Actuators

| Example | URDF Interfaces | MuJoCo Actuators | Status |
|---------|----------------|------------------|--------|
| `cart_example_position` | position | ❌ NONE | **BROKEN** |
| `cart_example_velocity` | velocity | ❌ NONE | **BROKEN** |
| `cart_example_effort` | effort | ❌ NONE | **BROKEN** |
| `vertical_cart_example_position_pid` | position | ❌ NONE | **BROKEN** |
| `vertical_cart_example_velocity` | velocity | ❌ NONE | **BROKEN** |
| `diff_drive` | velocity (x2 wheels) | ❌ NONE | **BROKEN** |
| `tricycle_drive` | velocity | ❌ NONE | **BROKEN** |
| `gripper_mimic_joint` | position | ❌ NONE | **BROKEN** |
| `ft_sensor_example` | effort | ❌ NONE (likely) | **BROKEN** |
| `camera_example` | N/A (sensors only) | ❌ NONE (likely) | **BROKEN** |

### ✅ Working

| Example | Status | Reason |
|---------|--------|--------|
| `test_1dof_gravity` | ✅ **WORKING** | Updated with proper actuators |

---

## Validation Error Details

When launching any of the broken examples, the system will fail during initialization with:

```
[ERROR] Joint '{joint_name}' declares {interface_type} interface in URDF but no
'act_{type}_{joint_name}' actuator found in MuJoCo model. Please add the actuator
or remove the interface.

[FATAL] URDF/MuJoCo mismatch: {interface_type} interface without actuator for
joint {joint_name}
```

### Example: cart_example_position

**URDF declares:**
```xml
<joint name="slider_to_cart">
  <command_interface name="position" />
  ...
</joint>
```

**MuJoCo model (`test_cart.xml`) has:**
```xml
<mujoco>
  ...
  <!-- NO <actuator> section at all! -->
</mujoco>
```

**Expected validation error:**
```
[ERROR] Joint 'slider_to_cart' declares position interface in URDF but no
'act_pos_slider_to_cart' actuator found in MuJoCo model.
```

---

## Root Cause

The actuator-centric design (implemented in `mujoco_system.cpp`) performs strict validation:

```cpp:mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp
// Check for mismatches and fail initialization
if (has_position_interface && joint_state.mj_pos_actuator_id < 0) {
  RCLCPP_ERROR(/* ... missing actuator error ... */);
  throw std::runtime_error(/* ... */);
}
```

This validation ensures that:
1. Controllers can't claim interfaces without corresponding actuators
2. Actuator neutralization works correctly (preventing interference)
3. MIT mode can properly compose PD + feedforward torques

**Previous behavior:** These examples may have worked if there was:
- No validation (actuators were optional)
- Default actuator creation
- Different control architecture

---

## Required Fixes

Each broken example needs its MuJoCo XML file updated with proper actuators.

### Fix Template for Single-Interface Examples

#### Position Control (cart_example_position, gripper_mimic_joint)

```xml
<actuator>
  <!-- Must use exact naming: act_pos_{joint_name} -->
  <position name="act_pos_slider_to_cart" joint="slider_to_cart"
            kp="100.0" kv="0.0"/>
</actuator>
```

#### Velocity Control (cart_example_velocity, diff_drive, tricycle_drive)

```xml
<actuator>
  <!-- Must use exact naming: act_vel_{joint_name} -->
  <velocity name="act_vel_slider_to_cart" joint="slider_to_cart"
            kv="10.0"/>
</actuator>
```

For diff_drive with 2 wheels:
```xml
<actuator>
  <velocity name="act_vel_left_wheel_joint" joint="left_wheel_joint" kv="10.0"/>
  <velocity name="act_vel_right_wheel_joint" joint="right_wheel_joint" kv="10.0"/>
</actuator>
```

#### Effort Control (cart_example_effort, ft_sensor_example)

```xml
<actuator>
  <!-- Must use exact naming: act_tau_{joint_name} -->
  <motor name="act_tau_slider_to_cart" joint="slider_to_cart"/>
</actuator>
```

### Fix Template for Multi-Interface Examples

If a URDF declares multiple interfaces (for potential MIT mode), need ALL three actuators:

```xml
<actuator>
  <!-- Position actuator: kv=0.0 is CRITICAL for MIT mode -->
  <position name="act_pos_{joint}" joint="{joint}" kp="100.0" kv="0.0"/>

  <!-- Velocity actuator -->
  <velocity name="act_vel_{joint}" joint="{joint}" kv="10.0"/>

  <!-- Torque actuator -->
  <motor name="act_tau_{joint}" joint="{joint}"/>
</actuator>
```

---

## Recommended Actions

### Option 1: Fix All Examples (Comprehensive)

**Pros:**
- All upstream examples work
- Maintains feature parity
- Good for users migrating from upstream

**Cons:**
- Time-consuming (10 examples to fix)
- Need to test each one
- May need to adjust controller configs

**Estimated effort:** 2-4 hours

### Option 2: Document as Breaking Change

**Pros:**
- Quick solution
- Focus on new examples (test_1dof_gravity)
- Users can fix as needed

**Cons:**
- Breaks existing workflows
- Poor user experience
- Appears incomplete

**Implementation:**
1. Add `BREAKING_CHANGES.md` documenting the issue
2. Add warning to main README
3. Provide clear fix instructions

### Option 3: Automated Fix with Script

Use the existing `urdf_to_mjcf.py` script to regenerate models:

```bash
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos
for urdf in urdf/test_cart*.xacro.urdf; do
  python3 scripts/urdf_to_mjcf.py "$urdf" "mujoco_models/$(basename ${urdf%.xacro.urdf}.xml)"
done
```

**Pros:**
- Fast (automated)
- Consistent actuator setup
- Ensures proper naming

**Cons:**
- May overwrite custom MuJoCo settings
- Need to verify each output
- Script may not handle all cases

**Estimated effort:** 1-2 hours (including verification)

---

## Immediate Recommendation

**Use Option 3 (Automated Fix) with verification:**

1. **Backup existing models**
   ```bash
   cp -r mujoco_models mujoco_models.backup
   ```

2. **Regenerate with proper actuators**
   ```bash
   # Use urdf_to_mjcf.py with proper flags
   python3 scripts/urdf_to_mjcf.py urdf/test_cart_position.xacro.urdf \
     mujoco_models/test_cart_position.xml --disable-collisions
   ```

3. **Test each example**
   ```bash
   ros2 launch mujoco_ros2_control_demos cart_example_position.launch.py
   ```

4. **Update launch files if needed**
   - Change `test_cart.xml` → `test_cart_position.xml` (if separate files)
   - Or merge actuators into single `test_cart.xml`

5. **Document the changes**
   - Update README with actuator requirements
   - Note in BREAKING_CHANGES.md

---

## Testing Checklist

After fixes, verify each example:

- [ ] cart_example_position
- [ ] cart_example_velocity
- [ ] cart_example_effort
- [ ] vertical_cart_example_position_pid
- [ ] vertical_cart_example_velocity
- [ ] diff_drive
- [ ] tricycle_drive
- [ ] gripper_mimic_joint
- [ ] ft_sensor_example
- [ ] camera_example
- [x] test_1dof_gravity ✅

---

## Files Requiring Changes

### MuJoCo Models (Primary Changes)
- `mujoco_models/test_cart.xml` - Add actuators
- `mujoco_models/test_vertical_cart.xml` - Add actuators
- `mujoco_models/test_diff_drive.xml` - Add actuators (2 wheels)
- `mujoco_models/test_tricycle_drive.xml` - Add actuators
- `mujoco_models/test_gripper_mimic_joint.xml` - Add actuators
- `mujoco_models/test_ft_sensor.xml` - Add actuators
- `mujoco_models/test_camera.xml` - Check if actuators needed

### Launch Files (Possible Changes)
May need to update `mujoco_model_path` if we create separate XML files per example.

### Documentation (Required)
- `README.md` - Add actuator requirements section
- `BREAKING_CHANGES.md` - Document the requirement
- Example READMEs - Update instructions

---

## Conclusion

**Status:** 🔴 **BLOCKING** - All upstream examples are broken

**Priority:** **HIGH** - This should be fixed before merging the branch

**Recommended Path:** Use automated regeneration (Option 3) to quickly fix all examples, then test and document.

The actuator-centric design is sound, but requires proper MuJoCo model setup. This is a one-time migration cost that ensures robust, validated control interface management.

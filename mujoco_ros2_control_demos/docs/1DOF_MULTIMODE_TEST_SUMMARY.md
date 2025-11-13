# 1-DOF Multimode Control Test - Quick Reference

**Created**: November 13, 2025
**System**: Single-DOF vertical pendulum with gravity compensation
**Purpose**: Test multi-controller operation (JTC vs zordi_mit_controller)

---

## Files Created

### Configuration

```
config/test_1dof_multimode_with_gravity.yaml    # Controller config (JTC + zordi_mit)
config/initial_pose_test.yaml                   # Updated with 5 test poses
```

### Launch

```
launch/test_1dof_multimode_with_gravity.launch.py    # Main launch file (MuJoCo viewer enabled)
```

### Test Scripts

```
scripts/test_1dof_multimode_scenario_a.py    # JTC move → zordi_mit hold (executable)
scripts/test_1dof_multimode_scenario_b.py    # zordi_mit move + hold (executable)
```

### Documentation

```
docs/MANUAL_1DOF_MULTIMODE_TEST.md           # Step-by-step manual test guide
docs/1DOF_MULTIMODE_TEST_SUMMARY.md          # This file
```

---

## Quick Start

### 1. Launch System

```bash
ros2 launch mujoco_ros2_control_demos test_1dof_multimode_with_gravity.launch.py
```

**Expected**:

- MuJoCo viewer opens
- Pendulum starts at upright position (0.0 rad)
- Only `joint_state_broadcaster` active

### 2A. Run Automated Test - Scenario A

```bash
# In new terminal
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts
./test_1dof_multimode_scenario_a.py
```

**What it does**: Tests 5 positions [0.0, 0.5, 1.0, -0.5, -1.0] rad

- Uses JTC to move to position
- Switches to zordi_mit_controller
- Measures drift over 10s
- Reports results table

### 2B. Run Automated Test - Scenario B

```bash
# In new terminal
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts
./test_1dof_multimode_scenario_b.py
```

**What it does**: Tests 5 positions using zordi_mit only

- Uses zordi_mit to move to position
- Controller auto-holds with gravity comp
- Measures drift over 10s
- Reports results table

### 3. Manual Testing

See `docs/MANUAL_1DOF_MULTIMODE_TEST.md` for detailed commands.

**Quick commands**:

```bash
# Activate JTC
ros2 control set_controller_state joint_trajectory_controller active

# Send trajectory to JTC
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['j1'], points: [{positions: [0.5], velocities: [0.0], time_from_start: {sec: 2}}]}"

# Switch to zordi_mit
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['zordi_mit_controller'], deactivate_controllers: ['joint_trajectory_controller'], strictness: 2}"

# Send trajectory to zordi_mit (action)
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['j1'], points: [{positions: [0.5], velocities: [0.0], time_from_start: {sec: 2}}]}}"
```

---

## Test Positions

Defined in `config/initial_pose_test.yaml`:

| Name | Value (rad) | Description |
|------|-------------|-------------|
| `home` | 0.0 | Upright position |
| `pos_small` | 0.5 | Small positive angle |
| `pos_large` | 1.0 | Large positive angle |
| `neg_small` | -0.5 | Small negative angle |
| `neg_large` | -1.0 | Large negative angle |

---

## Expected Results

### Scenario A: JTC → zordi_mit

- **JTC tracking**: Smooth trajectory execution
- **JTC holding**: Robot drifts under gravity (>0.1 rad/10s)
- **After switch to zordi_mit**: Drift stops (<0.001 rad/10s)

### Scenario B: zordi_mit only

- **Tracking**: Smooth trajectory with gravity comp
- **Holding**: Automatic hold with gravity comp (<0.001 rad/10s)

---

## System Architecture

### Controllers Available

**joint_trajectory_controller** (JTC):

- Claims: `[position, velocity]`
- Mode: Position + Velocity actuators
- Gravity comp: NO
- Use for: Standard trajectory tracking

**zordi_mit_controller**:

- Claims: `[position, velocity, effort]`
- Mode: MIT mode (PD + feedforward)
- Gravity comp: YES (via Pinocchio)
- Use for: Trajectory tracking + gravity hold

### MIT Mode Operation

When zordi_mit_controller is active:

1. **Controller** computes and outputs:
   - `position_cmd`: Desired position (q_cmd)
   - `velocity_cmd`: Desired velocity (qd_cmd)
   - `effort_cmd`: Feedforward torque (τ_ff = gravity compensation)

2. **Hardware interface** composes final torque:

   ```
   τ_total = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff
   ```

   Where Kp=100, Kd=10 (from URDF)

3. **MuJoCo** simulates with composed torque

---

## Troubleshooting

### Robot falls even with zordi_mit active

**Check**:

```bash
# Verify gravity comp is enabled
ros2 param get /zordi_mit_controller use_gravity_compensation
# Should return: Boolean value is: True

# Check Pinocchio is installed
python3 -c "import pinocchio; print('OK')"
```

### Controller won't activate

```bash
# List controllers
ros2 control list_controllers

# Stop all controllers
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{deactivate_controllers: ['joint_trajectory_controller', 'zordi_mit_controller'], strictness: 1}"

# Then activate desired one
ros2 control set_controller_state <controller_name> active
```

### Scripts report connection errors

**Ensure** launch file is running and wait ~2 seconds for services to be ready.

---

## Next Steps

After successful testing:

1. Adapt for 6-DOF OpenARM (see `openarm_description/docs/README.md`)
2. Tune PID gains for different payloads
3. Implement custom trajectories
4. Test with physical hardware

---

## References

- **Technical details**: `openarm_description/docs/TECHNICAL_REFERENCE.md`
- **Project history**: `openarm_description/docs/PROJECT_HISTORY.md`
- **Manual testing**: `docs/MANUAL_1DOF_MULTIMODE_TEST.md`
- **zordi_mit_controller**: `zordi_mit_controller/README.md`

---

**Document Version**: 1.0
**Status**: Ready for testing

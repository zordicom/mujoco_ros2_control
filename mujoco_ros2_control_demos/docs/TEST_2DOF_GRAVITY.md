# 2-DOF Vertical Double Pendulum Gravity Compensation Test Guide

**Test system:** Vertical double pendulum with gravity compensation

This guide provides comprehensive testing procedures for the 2-DOF vertical double pendulum example, demonstrating gravity compensation and trajectory tracking with active gravity loading.

## Overview

The `test_2dof_gravity` example features a vertical double pendulum where q=[0, 0] is the natural hanging configuration (stable equilibrium). The test demonstrates gravity compensation by maintaining the robot at various configurations against gravity, starting from q=[0.3, -0.2] which provides measurable gravity torques for verification.

### Architecture

This demo uses the unified lifecycle architecture:

- **Simulation:** MujocoSystem plugin loaded by controller_manager
- **Visualization:** Integrated MuJoCo viewer (enabled via URDF `mujoco_viewer` parameter)
- **Control:** Plugin steps simulation in `write()` method (1000 Hz)
- **Services:** Accessible at `/mujoco_system/*`

The plugin owns the MuJoCo model and simulation. The integrated viewer runs in a background thread at 60 Hz.

### System Specifications

**Physical Configuration:**

- **Link 1:** 1.0 kg mass, 0.5 m length, COM at 0.25 m below joint1
- **Link 2:** 0.5 kg mass, 0.5 m length, COM at 0.25 m below joint2
- **Joint 1:** Revolute joint rotating around Y-axis, mounted at 1.5 m height
- **Joint 2:** Revolute joint rotating around Y-axis, at end of link 1
- **Total reach:** 1.0 m (when fully extended)

**Coordinate System:**

- Both joints rotate in the vertical XZ plane (rotation around Y-axis)
- q=[0, 0] means both links hang straight down (stable equilibrium)
- q=[0.3, -0.2] (test_pose) means link1 slightly forward, link2 slightly back from hanging
- q=[π, π] would mean both links point straight up (unstable upright equilibrium)
- Positive joint angles rotate forward, negative rotate backward

## Expected Physics

### Gravity Torques

When the double pendulum is horizontal (q=[π/2, 0]):

- **Joint 1 max torque:** τ_g1 = g × (m1 × r1 + m2 × L1) = 9.81 × (1.0 × 0.25 + 0.5 × 0.5) ≈ 4.9 Nm
- **Joint 2 max torque:** τ_g2 = g × m2 × r2 = 9.81 × 0.5 × 0.25 ≈ 1.2 Nm

### Behavior Without Gravity Compensation

**Without controllers active:**

- Double pendulum naturally hangs at q=[0, 0] (stable configuration)
- Any disturbance from hanging position experiences restoring gravity torques
- Links oscillate around hanging with natural damping (damping = 0.1)

**With controllers but gravity compensation disabled:**

- Controllers fight gravity with PD control alone
- Large steady-state errors at non-zero configurations
- High energy consumption to maintain position

### Behavior With Gravity Compensation

**Expected with proper gravity compensation:**

- Robot holds upright position q=[0, 0] with minimal effort
- Smooth trajectory tracking without steady-state error
- Low energy consumption (only compensating dynamics, not fighting gravity)
- Backdrivable when using pure gravity compensation mode

## Launch Instructions

### Start the Simulation

```bash
ros2 launch mujoco_ros2_control_demos test_2dof_gravity.launch.py
```

This launches:

- Controller manager with MujocoSystem plugin (lifecycle mode)
- Integrated MuJoCo viewer (enabled by default via URDF parameter)
- 2-DOF vertical double pendulum
- Robot state publisher
- Joint state broadcaster (active)
- Five Zordi controllers + one ROS-native (all inactive):
  - `zordi_grav_comp_controller` - Pure gravity compensation
  - `zordi_joint_trajectory_controller` - Joint space with gravity comp
  - `zordi_joint_mit_rnea_controller` - Joint space with RNEA (MIT mode)
  - `zordi_cartesian_effort_controller` - Cartesian impedance
  - `zordi_cartesian_effort_rnea_controller` - Cartesian with RNEA
  - `joint_trajectory_controller` - ROS-native (no gravity comp)

**Note:** The robot is reset to `test_pose` (q=[0.3, -0.2]) after launch via service call. This provides a configuration with some gravity torque to verify compensation is working.

**Important:** Simulation starts **PAUSED**. You must unpause after activating controllers:

```bash
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

**Viewer Control:** The viewer is enabled/disabled via URDF parameter `mujoco_viewer` (default: true for this demo). To disable, edit `test_2dof_gravity.xacro.urdf`.

**Available keyframes** (defined in MuJoCo XML, loaded via service):

```bash
ros2 service call /mujoco_system/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'test_pose'}"
```

Keyframe options:

- `test_pose`: q=[0.3, -0.2] - Default, near hanging with some offset
- `home`: q=[0, 0] - Both links hanging straight down (stable equilibrium)
- `hanging`: q=[-π/2, -π/2] - Both links bent backward from hanging
- `bent_forward`: q=[0.5, -0.5] - Forward bend configuration
- `bent_back`: q=[-0.5, 0.5] - Backward bend configuration
- `horizontal`: q=[π/2, 0] - Link1 horizontal, link2 hanging
- `folded`: q=[0, π/2] - Link1 hanging, link2 folded forward

### Verify System is Ready

```bash
# Check all controllers loaded
ros2 control list_controllers

# Expected output:
#   joint_state_broadcaster            [active]
#   zordi_grav_comp_controller         [inactive]
#   zordi_joint_trajectory_controller  [inactive]
#   zordi_joint_mit_rnea_controller    [inactive]
#   zordi_cartesian_effort_controller         [inactive]
#   zordi_cartesian_effort_rnea_controller    [inactive]
#   joint_trajectory_controller        [inactive]

# Monitor joint states
ros2 topic echo /joint_states
```

## Test Procedures

### Test 1: Pure Gravity Compensation (Backdrivable)

**Objective:** Verify pure gravity compensation without trajectory tracking.

**Steps:**

1. **Activate the gravity compensation controller:**

```bash
ros2 control set_controller_state zordi_grav_comp_controller active
```

2. **Unpause simulation:**

```bash
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

3. **Observe behavior:**
   - Robot should hold position at q=[0.3, -0.2]
   - Robot should be backdrivable (you can push it in simulation)
   - No trajectory tracking active - purely gravity compensation

**Success Criteria:**

- Robot maintains position without falling
- No oscillations or drift
- Effort output matches gravity torques

### Test 2: Joint Space Trajectory Tracking

**Objective:** Verify trajectory tracking with gravity compensation enabled.

**Steps:**

1. **Switch to joint trajectory controller:**

```bash
ros2 control switch_controllers \
  --activate zordi_joint_trajectory_controller \
  --deactivate zordi_grav_comp_controller
```

2. **Unpause if needed:**

```bash
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

3. **Send a multi-waypoint trajectory:**

```bash
ros2 action send_goal /zordi_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint1, joint2],
      points: [
        {positions: [0.5, -0.1], time_from_start: {sec: 2}},
        {positions: [0.8, -0.3], time_from_start: {sec: 5}},
        {positions: [0.3, -0.2], time_from_start: {sec: 8}}
      ]
    }
  }" --feedback
```

3. **Observe tracking performance:**
   - Smooth motion between waypoints
   - No overshoots or oscillations
   - Returns to upright position at end

**Success Criteria:**

- Trajectory completes successfully (action returns SUCCEEDED)
- Tracking error < 0.05 rad during motion
- Final position within goal tolerance

### Test 3: Cartesian Space Control

**Objective:** Test Cartesian impedance control with gravity compensation in vertical plane.

**Steps:**

1. **Switch to Cartesian controller:**

```bash
ros2 control switch_controllers \
  --activate zordi_cartesian_effort_controller \
  --deactivate zordi_joint_trajectory_controller
```

2. **Calculate end-effector positions** (for reference - based on actual kinematics):
   - At q=[0, 0] (hanging): EE at (x=0, y=0, z=0.5)
   - At q=[0.3, -0.2] (start): EE at (x≈-0.20, y=0, z≈0.53)
   - At q=[0.3, 0.0] (straighten link2): EE at (x≈-0.30, y=0, z≈0.54)
   - Note: Positive joint angles rotate backward (toward -X)

3. **Send a target pose to straighten link2 (move to q=[0.3, 0.0]):**

```bash
ros2 topic pub --once /zordi_cartesian_effort_controller/target_pose \
  geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'world'},
    pose: {
      position: {x: -0.30, y: 0.0, z: 0.54},
      orientation: {w: 0.989, x: 0.0, y: 0.149, z: 0.0}
    }
  }"
```

**Note:** This moves the robot from q=[0.3, -0.2] to q=[0.3, 0.0], straightening link2. The EE moves backward (-X) and slightly up.

4. **Observe Cartesian motion:**
   - End-effector moves smoothly to target pose
   - Joint space configuration changes to achieve Cartesian goal
   - Gravity compensation maintains stability

5. **Send Cartesian trajectory (swing backward and forward):**

```bash
ros2 topic pub --once /zordi_cartesian_effort_controller/cartesian_trajectory \
  moveit_msgs/msg/CartesianTrajectory "{
    header: {frame_id: 'world'},
    tracked_frame: 'ee_link',
    points: [
      {
        point: {
          pose: {
            position: {x: -0.48, y: 0.0, z: 0.63},
            orientation: {w: 0.969, x: 0.0, y: 0.248, z: 0.0}
          }
        },
        time_from_start: {sec: 3}
      },
      {
        point: {
          pose: {
            position: {x: 0.48, y: 0.0, z: 0.63},
            orientation: {w: 0.969, x: 0.0, y: -0.248, z: 0.0}
          }
        },
        time_from_start: {sec: 6}
      },
      {
        point: {
          pose: {
            position: {x: -0.20, y: 0.0, z: 0.53},
            orientation: {w: 0.999, x: 0.0, y: 0.050, z: 0.0}
          }
        },
        time_from_start: {sec: 9}
      }
    ]
  }"
```

**Note:** Coordinate convention: positive joint angles rotate toward -X (backward).

- q≈[0.5, 0.0]: backward swing, EE at x≈-0.48
- q≈[-0.5, 0.0]: forward swing, EE at x≈+0.48
- q=[0.3, -0.2]: return to start, EE at x≈-0.20

**Success Criteria:**

- End-effector reaches target poses accurately
- Smooth Cartesian motion (straight line between waypoints)
- No drift or instability during hold
- Cartesian error < 0.01 m

### Test 4: Compare RNEA vs Base Controllers

**Objective:** Compare tracking performance between base controllers and RNEA (full inverse dynamics) controllers.

**Test 4a: Joint Space RNEA**

1. **Switch to RNEA joint controller:**

```bash
ros2 control switch_controllers \
  --activate zordi_joint_mit_rnea_controller \
  --deactivate zordi_cartesian_effort_controller
```

2. **Send same trajectory as Test 2:**

```bash
ros2 action send_goal /zordi_joint_mit_rnea_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint1, joint2],
      points: [
        {positions: [0.5, -0.1], time_from_start: {sec: 2}},
        {positions: [0.8, -0.3], time_from_start: {sec: 5}},
        {positions: [0.3, -0.2], time_from_start: {sec: 8}}
      ]
    }
  }" --feedback
```

3. **Compare with base controller results:**
   - RNEA should have lower tracking error
   - Smoother acceleration profiles
   - Better disturbance rejection

**Test 4b: Cartesian Space RNEA**

1. **Switch to RNEA Cartesian controller:**

```bash
ros2 control switch_controllers \
  --activate zordi_cartesian_effort_rnea_controller \
  --deactivate zordi_joint_mit_rnea_controller
```

2. **Send same Cartesian trajectory as Test 3**

3. **Compare performance:**
   - More accurate Cartesian tracking
   - Better handling of dynamic motion
   - Reduced Cartesian error

**Success Criteria:**

- RNEA controllers demonstrate improved tracking (measurable in feedback)
- Smooth motion without oscillations
- Both controller types successfully track trajectories

### Test 5: ROS-Native Controller Comparison

**Objective:** Compare Zordi controllers (with gravity comp) to ROS-native controller (without gravity comp).

**Steps:**

1. **Switch to ROS-native joint trajectory controller:**

```bash
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate zordi_cartesian_effort_rnea_controller
```

2. **Send same trajectory as Test 2:**

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [joint1, joint2],
      points: [
        {positions: [0.5, -0.1], time_from_start: {sec: 2}},
        {positions: [0.8, -0.3], time_from_start: {sec: 5}},
        {positions: [0.3, -0.2], time_from_start: {sec: 8}}
      ]
    }
  }" --feedback
```

3. **Observe difference:**
   - ROS-native controller will struggle significantly
   - Large steady-state errors (e.g., 0.04 rad) due to gravity
   - May oscillate slightly when activated
   - Cannot reach targets accurately (e.g., targets 0.8 but reaches ~0.76)
   - Demonstrates importance of gravity compensation

**Why It Struggles:**

- Uses position interface → hardware PD control (kp=100, kp=50)
- PD control alone: τ = kp × error
- At q=[0.8, -0.3]: gravity τ ≈ 5-6 Nm
- Position error: 0.04 rad → PD torque = 100 × 0.04 = 4 Nm
- 4 Nm < 6 Nm → Cannot overcome gravity! ❌

**Success Criteria:**

- Observably worse performance than Zordi controllers
- Steady-state errors of 0.03-0.05 rad (expected and normal)
- Oscillations when activated (expected and normal)
- Highlights critical value of gravity compensation

## Command Quick Reference

### Controller Management

```bash
# List all controllers
ros2 control list_controllers

# Activate a controller
ros2 control set_controller_state <controller_name> active

# Deactivate a controller
ros2 control set_controller_state <controller_name> inactive

# Switch controllers atomically
ros2 control switch_controllers \
  --activate <new_controller> \
  --deactivate <old_controller>
```

### Simulation Control

```bash
# Pause simulation
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'pause'}"

# Unpause simulation
ros2 service call /mujoco_system/simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"

# Reset to keyframe (defined in MuJoCo XML)
ros2 service call /mujoco_system/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'test_pose'}"
```

### Monitoring

```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor controller state
ros2 topic echo /zordi_joint_trajectory_controller/controller_state

# Monitor specific joint
ros2 topic echo /joint_states --field position

# Check system logs
ros2 topic echo /rosout | grep zordi
```

## Success Criteria Summary

### Overall System

- [x] System launches without errors
- [x] All controllers load successfully
- [x] MuJoCo viewer displays robot correctly
- [x] Joint state broadcaster publishes at 1 kHz

### Gravity Compensation

- [x] Robot holds upright position at q=[0, 0] without falling
- [x] Position maintained with < 0.05 rad error
- [x] No steady-state drift or oscillation
- [x] Low torque output when holding (only compensating gravity)

### Joint Space Tracking

- [x] Trajectories execute to completion
- [x] Tracking error < 0.05 rad during motion
- [x] Smooth motion without overshoots
- [x] Action server returns SUCCESS

### Cartesian Space Control

- [x] End-effector reaches target poses
- [x] Cartesian error < 0.01 m
- [x] Smooth straight-line motion between waypoints
- [x] Gravity compensation maintains stability

### RNEA Performance

- [x] RNEA controllers show improved tracking accuracy
- [x] Lower error metrics compared to base controllers
- [x] No additional instability introduced

## Troubleshooting

### Robot Falls Immediately

**Symptom:** Double pendulum falls to hanging position right after launch.

**Causes:**

1. No controller is active
2. Gravity compensation disabled in config
3. Controller activation failed

**Solutions:**

```bash
# Check controller status
ros2 control list_controllers

# Activate controller
ros2 control set_controller_state zordi_joint_trajectory_controller active

# Verify gravity compensation enabled
ros2 param get /zordi_joint_trajectory_controller use_gravity_compensation
# Should return: True
```

### Oscillations or Instability

**Symptom:** Robot oscillates around target position or becomes unstable.

**Causes:**

1. PD gains too high
2. Stiffness too high (Cartesian controller)
3. Conflicting command interfaces

**Solutions:**

For joint controllers, reduce PD gains in URDF:

- Lower `position_kp` (try 50 instead of 100)
- Lower `velocity_kp` (try 5 instead of 10)

For Cartesian controllers, reduce stiffness in YAML:

```yaml
stiffness:
  translation: {x: 100.0, y: 100.0, z: 100.0}  # Lower from 200
  rotation: {x: 10.0, y: 10.0, z: 10.0}        # Lower from 20
```

### Poor Tracking Performance

**Symptom:** Large tracking errors or robot doesn't reach targets.

**Causes:**

1. Insufficient torque limits
2. Trajectory too fast
3. Gravity compensation not working

**Solutions:**

```bash
# Check torque limits in controller state
ros2 topic echo /zordi_joint_trajectory_controller/controller_state

# Verify Pinocchio model loaded
# Should see "Pinocchio model initialized" in logs
ros2 topic echo /rosout | grep -i pinocchio

# Test with slower trajectory (increase time_from_start)
```

### Cartesian Controller Won't Activate

**Symptom:** Cartesian controller fails to activate with error.

**Causes:**

1. Joint controller still active (interface conflict)
2. Frame names don't match URDF
3. Pinocchio model not initialized

**Solutions:**

```bash
# Deactivate joint controller first
ros2 control set_controller_state zordi_joint_trajectory_controller inactive

# Check frame names in URDF
ros2 topic echo /robot_description --once | grep -E "(ee_link|world)"

# Restart simulation if Pinocchio failed to initialize
```

### End-Effector Position Wrong

**Symptom:** End-effector doesn't move to expected Cartesian positions.

**Causes:**

1. Frame reference mismatch
2. Incorrect kinematics
3. Target pose unreachable

**Solutions:**

```bash
# Check current EE pose
ros2 run tf2_ros tf2_echo world ee_link

# Verify target pose is reachable (within 1.0 m reach)
# Total reach = 0.5 + 0.5 = 1.0 m

# Use correct frame_id in pose messages (should be 'world')
```

## Additional Notes

### Controller Feature Comparison

**Joint Space Controllers:**

| Feature | zordi_grav_comp_controller | zordi_joint_trajectory_controller | zordi_joint_mit_rnea_controller |
|---------|---------------------------|----------------------------------|----------------------------|
| Trajectory tracking | ✗ | ✓ | ✓ |
| Gravity compensation | ✓ | ✓ | ✓ |
| Coriolis compensation | ✗ | ✗ | ✓ |
| Feedforward dynamics | ✗ | ✗ | ✓ (full RNEA) |
| Backdrivable | ✓ | ✗ | ✗ |
| Tracking accuracy | N/A | Good | Excellent |
| Computational cost | Low | Low | Medium |

**Cartesian Space Controllers:**

| Feature | zordi_cartesian_effort_controller | zordi_cartesian_effort_rnea_controller |
|---------|---------------------------|--------------------------------|
| Gravity compensation | ✓ | ✓ |
| Task-space impedance | ✓ | ✓ |
| Full dynamics | ✗ | ✓ (full RNEA) |
| Nullspace control | ✓ | ✓ |
| Tracking accuracy | Good | Excellent |

### Physics Validation

To validate gravity compensation accuracy:

```bash
# Compare controller gravity torque to MuJoCo ground truth
ros2 topic echo /joint_states --field effort

# For perfect gravity comp, effort should match:
# Joint 1 (at q=[π/2, 0]): ~4.9 Nm
# Joint 2 (at q=[0, π/2]): ~1.2 Nm
```

### Performance Metrics

Expected performance on standard hardware (1 kHz control rate):

- Control loop jitter: < 0.5 ms
- Trajectory tracking error: < 0.05 rad (joint space)
- Cartesian tracking error: < 0.01 m
- Gravity compensation error: < 0.1 Nm

## Next Steps

- **For theory:** See controller implementation details in `zordi_ros_controllers` package
- **For hardware deployment:** Adapt URDF and configs to match real robot parameters
- **For advanced control:** Explore operational space control (`use_operational_space: true`)
- **For 7-DOF robots:** See nullspace control examples in `USER_GUIDE.md`

---

**Congratulations!** If all tests pass, your gravity compensation and controller system is working correctly.

# Manual 1-DOF Multimode Control Test Guide

**Purpose**: Test multi-controller operation on 1-DOF vertical pendulum with gravity compensation

**System**: Vertical pendulum (1kg mass at 0.5m below pivot)
**Controllers**: `joint_trajectory_controller` (JTC) and `zordi_mit_controller`
**Test Focus**: Controller switching and gravity compensation performance

---

## Prerequisites

- ROS2 Humble installed
- Workspace built: `cd ~/ros2_ws && colcon build --symlink-install`
- Source workspace: `source ~/ros2_ws/install/setup.bash`

---

## Launch System

### Terminal 1: Start Simulation

```bash
ros2 launch mujoco_ros2_control_demos test_1dof_multimode_with_gravity.launch.py
```

**Expected**:

- MuJoCo viewer window opens
- Vertical pendulum at upright position (0.0 rad)
- Console shows controller_manager startup

**Note**: Only `joint_state_broadcaster` is active initially. Controllers must be manually activated.

---

## Check System Status

### Terminal 2: Verify Controllers

```bash
# List available controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
# zordi_mit_controller[zordi_mit_controller/ZordiMITController] inactive
```

### Monitor Joint States

```bash
# Monitor joint position/velocity
ros2 topic echo /joint_states

# Should show:
# name: ['j1']
# position: [~0.0]
# velocity: [~0.0]
```

---

## Scenario A: JTC to Move → zordi_mit to Hold

**Objective**: Use JTC for trajectory execution, then switch to zordi_mit for gravity-compensated hold

### Step 1: Activate JTC

```bash
ros2 control set_controller_state joint_trajectory_controller active
```

**Verify**:

```bash
ros2 control list_controllers
# joint_trajectory_controller should show "active"
```

### Step 2: Send Trajectory

```bash
# Move to 0.5 rad over 2 seconds
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{
  joint_names: ['j1'],
  points: [
    {
      positions: [0.5],
      velocities: [0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

**Observe**:

- Pendulum smoothly moves to 0.5 rad
- After 2s, pendulum may drift downward (no gravity comp in JTC)

### Step 3: Switch to zordi_mit

```bash
# Deactivate JTC and activate zordi_mit
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
"{
  activate_controllers: ['zordi_mit_controller'],
  deactivate_controllers: ['joint_trajectory_controller'],
  strictness: 2,
  start_asap: true
}"
```

**Observe**:

- Pendulum should stop drifting
- Position holds steady with gravity compensation

### Step 4: Monitor Hold Performance

```bash
# In a separate terminal, watch position for 10 seconds
ros2 topic echo /joint_states | grep -A1 "position:"
```

**Expected**: Position should remain within ±0.001 rad of target

### Step 5: Test Other Configurations

Repeat steps 2-4 with different targets:

```bash
# Position 1.0 rad
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['j1'], points: [{positions: [1.0], velocities: [0.0], time_from_start: {sec: 2}}]}"

# Position -0.5 rad
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['j1'], points: [{positions: [-0.5], velocities: [0.0], time_from_start: {sec: 2}}]}"

# Position 0.0 rad (back to home)
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: ['j1'], points: [{positions: [0.0], velocities: [0.0], time_from_start: {sec: 2}}]}"
```

---

## Scenario B: zordi_mit for Both Move and Hold

**Objective**: Use zordi_mit_controller for both trajectory execution and holding

### Step 1: Reset Controllers

```bash
# Ensure only zordi_mit is active
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
"{
  activate_controllers: ['zordi_mit_controller'],
  deactivate_controllers: ['joint_trajectory_controller'],
  strictness: 2
}"
```

### Step 2: Send Trajectory via Action

```bash
# Move to 0.5 rad over 2 seconds
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{
  trajectory: {
    joint_names: ['j1'],
    points: [
      {
        positions: [0.5],
        velocities: [0.0],
        time_from_start: {sec: 2, nanosec: 0}
      }
    ]
  }
}" --feedback
```

**Observe**:

- Pendulum moves to 0.5 rad
- Action provides feedback during execution
- After completion, pendulum automatically holds with gravity comp

### Step 3: Monitor Auto-Hold

```bash
# Watch position after trajectory completes
ros2 topic echo /joint_states | grep -A1 "position:"
```

**Expected**:

- No drift after trajectory completes
- Controller continues applying gravity compensation
- Position stable within ±0.001 rad

### Step 4: Test Other Configurations

```bash
# Position 1.0 rad
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: ['j1'], points: [{positions: [1.0], velocities: [0.0], time_from_start: {sec: 2}}]}}"

# Position -0.5 rad
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: ['j1'], points: [{positions: [-0.5], velocities: [0.0], time_from_start: {sec: 2}}]}}"

# Back to home
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: ['j1'], points: [{positions: [0.0], velocities: [0.0], time_from_start: {sec: 2}}]}}"
```

---

## Alternative: Topic-Based Commands for zordi_mit

Instead of actions, you can use topic commands:

```bash
# Publish to joint_trajectory topic
ros2 topic pub --once /zordi_mit_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{
  joint_names: ['j1'],
  points: [
    {
      positions: [0.5],
      velocities: [0.0],
      time_from_start: {sec: 2}
    }
  ]
}"
```

---

## Observing in MuJoCo Viewer

### Controls

- **Left mouse drag**: Rotate view
- **Right mouse drag**: Pan view
- **Scroll wheel**: Zoom
- **Space**: Pause/unpause
- **Right-click pendulum**: Shows info

### What to Watch

- **Without gravity comp**: Pendulum drifts downward under gravity
- **With gravity comp**: Pendulum holds position stiffly
- **During switching**: Brief transient, then stabilizes

---

## Expected Results

### Scenario A (JTC → zordi_mit)

| Phase | Controller | Drift | Notes |
|-------|-----------|-------|-------|
| Moving | JTC | N/A | Smooth trajectory execution |
| After arrival | JTC | High (>0.1 rad/10s) | Falls under gravity |
| After switch | zordi_mit | Low (<0.001 rad/10s) | Holds with gravity comp |

### Scenario B (zordi_mit only)

| Phase | Controller | Drift | Notes |
|-------|-----------|-------|-------|
| Moving | zordi_mit | N/A | Smooth trajectory with gravity comp |
| After arrival | zordi_mit | Low (<0.001 rad/10s) | Auto-holds with gravity comp |

---

## Troubleshooting

### Pendulum Falls Even with zordi_mit

**Check**:

1. Gravity compensation enabled: `ros2 param get /zordi_mit_controller use_gravity_compensation`
   - Should return: `true`
2. Controller is active: `ros2 control list_controllers`
3. Pinocchio installed: `python3 -c "import pinocchio; print('OK')"`

### Controller Won't Activate

**Solution**:

```bash
# Stop all controllers
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
"{deactivate_controllers: ['joint_trajectory_controller', 'zordi_mit_controller'], strictness: 1}"

# Then activate desired controller
ros2 control set_controller_state <controller_name> active
```

### Trajectory Not Executing

**Check**:

1. Controller is active
2. Topic/action name is correct (`/zordi_mit_controller/...` or `/joint_trajectory_controller/...`)
3. Joint name in trajectory matches: `['j1']`

---

## Cleanup

```bash
# Ctrl+C in Terminal 1 to stop launch file
pkill -9 -f "test_1dof_multimode_with_gravity"
```

---

## Automated Testing

For automated testing instead of manual commands:

```bash
# Scenario A
python3 ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts/test_1dof_multimode_scenario_a.py

# Scenario B
python3 ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/scripts/test_1dof_multimode_scenario_b.py
```

Both scripts will:

- Test all configurations automatically
- Print detailed logs
- Generate summary table of results

---

**Document Version**: 1.0
**Last Updated**: November 13, 2025
**Related**: See `PROJECT_HISTORY.md` for validation results

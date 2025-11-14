# MuJoCo ROS2 Control Demos

**Copyright 2025 Zordi, Inc. All rights reserved.**

This package demonstrates the key features of `mujoco_ros2_control` using simple, easy-to-understand examples.

---

## Quick Start

### Prerequisites

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control mujoco_ros2_control_demos zordi_mit_controller
source install/setup.bash
```

### Run the Basic Demo

```bash
# Launch 1-DOF gravity compensation demo
ros2 launch mujoco_ros2_control_demos test_1dof_gravity.launch.py
```

**You should see:**

- MuJoCo viewer window opens
- Vertical pendulum starts at 0.5 rad (upright position)
- Simulation starts in PAUSED state
- Controller loads as inactive

---

## Core Features Demonstrated

This demo showcases all major features of the actuator-centric control architecture:

### 1. Actuator-Centric Control

**No global mode switching** - Each joint has three independent actuators that are dynamically activated based on which controller claims which interfaces.

**From `test_1dof_gravity.xml`:**

```xml
<actuator>
  <!-- Position actuator: used when position interface is claimed -->
  <position name="act_pos_j1" joint="j1" kp="100.0" kv="0.0"/>

  <!-- Velocity actuator: used when velocity interface is claimed -->
  <velocity name="act_vel_j1" joint="j1" kv="10.0"/>

  <!-- Torque actuator: used when effort interface is claimed -->
  <motor name="act_tau_j1" joint="j1"/>
</actuator>
```

**Key points:**

- Must use naming convention: `act_pos_{joint}`, `act_vel_{joint}`, `act_tau_{joint}`
- Position actuator should have `kv="0.0"` for clean MIT mode operation
- Each actuator is independently driven or neutralized based on active interfaces

### 2. Initial Pose Configuration (Keyframes)

Start simulations from specific configurations using MuJoCo's native keyframe system.

**From `test_1dof_gravity.xml`:**

```xml
<keyframe>
  <key name="test_pose" qpos="0.5"/>
  <key name="home" qpos="0.0"/>
  <key name="pos_large" qpos="1.0"/>
  <key name="neg_small" qpos="-0.5"/>
</keyframe>
```

**Usage in launch file:**

```python
mujoco_node = Node(
    parameters=[{
        "initial_keyframe": "test_pose",  # Start at 0.5 rad
    }]
)
```

**Runtime reset to keyframe:**

```bash
ros2 service call /reset_to_keyframe mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'home'}"
```

### 3. Simulation Control (Pause/Unpause/Reset)

Control simulation execution state at runtime.

**Simulation always starts PAUSED** - Explicit unpause required to begin physics.

```bash
# Query current state
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl \
  "{command: 'status'}"

# Start simulation
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl \
  "{command: 'unpause'}"

# Pause for inspection
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl \
  "{command: 'pause'}"

# Reset to initial keyframe and pause
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl \
  "{command: 'reset'}"
```

**When paused:**

- Physics steps are skipped
- Simulation time is frozen
- Controllers remain active (called with `dt=0`)
- Allows controller loading without physics running

### 4. MIT Mode (Multi-Interface Control)

When a controller claims effort + (position or velocity) interfaces, the system automatically enters MIT mode.

**MIT Mode Detection:**

```cpp
bool mit_mode = effort_active && (position_active || velocity_active);
```

**MIT Mode Behavior:**

- Position and velocity actuators are neutralized (`ctrl_pos = q`, `ctrl_vel = qd`)
- Torque actuator is driven with PD composition: `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff`
- PD gains come from URDF `<ros2_control>` transmission section
- Feedforward torque (`τ_ff`) comes from controller (e.g., gravity compensation)

---

## Usage Examples

### Example 1: Pure Gravity Compensation (Effort-Only Mode)

The controller claims only the effort interface and applies gravity compensation torques.

**Try it:**

```bash
# Terminal 1: Launch (loads both controllers in inactive state)
ros2 launch mujoco_ros2_control_demos test_1dof_gravity.launch.py

# Terminal 2: Unpause simulation
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl \
  "{command: 'unpause'}"

# Activate gravity compensation controller
ros2 control set_controller_state zordi_grav_comp_controller active

# Monitor joint state
ros2 topic echo /joint_states
```

**Expected behavior:**

- Pendulum holds upright position purely via gravity compensation
- Fully backdrivable (no PD control)
- Zero drift over time

### Example 2: MIT Mode with Gravity Compensation

The controller claims position, velocity, and effort interfaces, triggering MIT mode.

**Try it:**

```bash
# Terminal 1: Same launch file - both controllers are already loaded!
# (If not running, start with: ros2 launch mujoco_ros2_control_demos test_1dof_gravity.launch.py)

# Terminal 2: Switch from gravity comp controller to MIT controller
# First, deactivate the gravity comp controller
ros2 control set_controller_state zordi_grav_comp_controller inactive

# Activate MIT controller
ros2 control set_controller_state zordi_mit_controller active

# Send trajectory via action
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['j1'], points: [{positions: [1.0], velocities: [0.0], time_from_start: {sec: 2}}]}}"
```

**Expected behavior:**

- Smooth trajectory tracking with gravity compensation
- Automatic hold after trajectory completion
- Zero drift when holding position

**Additional trajectories to try:**

```bash
# Move to different positions
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['j1'], points: [{positions: [-0.5], time_from_start: {sec: 1}}]}}"

# Multi-point trajectory
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['j1'], points: [{positions: [0.5], time_from_start: {sec: 1}}, {positions: [-0.5], time_from_start: {sec: 2}}, {positions: [0.0], time_from_start: {sec: 3}}]}}"
```

### Example 3: Position-Only Mode

Standard position control using only the position actuator.

**Configuration:**

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints: [j1]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

**Expected behavior:**

- Position actuator driven with position commands
- Velocity and torque actuators neutralized
- Standard trajectory tracking (no gravity compensation)

### Example 4: Testing Different Keyframes

```bash
# Terminal 1: Keep launch running

# Terminal 2: Reset to different poses
ros2 service call /reset_to_keyframe mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'home'}"

ros2 service call /reset_to_keyframe mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'pos_large'}"

ros2 service call /reset_to_keyframe mujoco_ros2_control_msgs/srv/ResetToKeyframe \
  "{keyframe: 'neg_small'}"
```

### Example 5: External Wrench Application

Apply external forces/torques to test disturbance rejection.

```bash
# Apply 5 Nm torque about Y-axis for 2 seconds
ros2 service call /apply_external_wrench mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{body_name: 'pendulum', wrench: {force: {x: 0, y: 0, z: 0}, torque: {x: 0, y: 5.0, z: 0}}, duration: 2.0}"
```

**Note:** Wrenches must be expressed in world frame.

### Example 6: Comparing Gravity Compensation

Validate your controller's gravity compensation implementation against MuJoCo's ground truth.

```bash
# Terminal 1: Keep launch running with controller active

# Terminal 2: Monitor MuJoCo's computed gravity torques
ros2 topic echo /mujoco/qfrc_bias

# Terminal 3: Monitor controller's effort commands
ros2 topic echo /zordi_grav_comp_controller/effort_command
```

Compare the values - they should match closely if gravity compensation is correct.

---

## Control Modes Summary

The actuator-centric design naturally supports these modes without explicit mode switching:

| Mode | Active Interfaces | Position Actuator | Velocity Actuator | Torque Actuator | Use Case |
|------|------------------|-------------------|-------------------|-----------------|----------|
| **Position** | position | Driven (pos_cmd) | Neutralized (qd) | Neutralized (0) | Standard trajectory tracking |
| **Velocity** | velocity | Neutralized (q) | Driven (vel_cmd) | Neutralized (0) | Velocity control |
| **Effort** | effort | Neutralized (q) | Neutralized (qd) | Driven (effort_cmd) | Force control, gravity comp |
| **MIT Mode** | position + velocity + effort | Neutralized (q) | Neutralized (qd) | Driven with PD (Kp\*pos_err + Kd\*vel_err + effort_cmd) | Advanced control with feedforward |

---

## File Structure

```
mujoco_ros2_control_demos/
├── README.md                           # This file
├── config/
│   └── test_1dof_gravity.yaml         # Controller configuration
├── launch/
│   └── test_1dof_gravity.launch.py    # Demo launch file
├── mujoco_models/
│   └── test_1dof_gravity.xml          # MuJoCo model with keyframes
├── urdf/
│   └── test_1dof_gravity.xacro.urdf   # URDF with ros2_control interfaces
└── [other upstream examples...]        # cart, diff_drive, etc.
```

---

## Creating Your Own Controller

### Step 1: Define URDF with Interfaces

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  </hardware>

  <joint name="j1">
    <!-- Expose all interfaces you want to use -->
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>

    <!-- PID gains for MIT mode -->
    <param name="position_p_gain">100.0</param>
    <param name="velocity_d_gain">10.0</param>
  </joint>
</ros2_control>
```

### Step 2: Create Matching MuJoCo Model

**Option A: Automated Conversion (Recommended)**

Use the provided generic conversion script to automatically generate MuJoCo models with proper actuators:

```bash
# Install urdf2mjcf if not already installed
pip install urdf2mjcf

# Convert your URDF
cd ~/ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos
python3 scripts/urdf_to_mjcf.py your_robot.urdf output/your_robot.xml --disable-collisions
```

This automatically:

- Converts your URDF to MuJoCo XML
- Detects all joints and adds three actuators per joint with correct naming (`act_pos_*`, `act_vel_*`, `act_tau_*`)
- Applies validated gains (kp=100, kv=0 for position, kv=10 for velocity)
- Optionally disables collisions (recommended for gravity compensation)
- Copies mesh files to the correct locations
- Validates the output

**Custom gains (optional):**

```bash
python3 scripts/urdf_to_mjcf.py robot.urdf output.xml --kp-pos 200 --kv-vel 20
```

**Note:** If you already have a MuJoCo XML file and just need to add actuators, you can manually add them following the pattern in Option B, or use MuJoCo's native URDF importer and then manually add the actuators.

**Adding Keyframes:**

Keyframes are MuJoCo-specific and must be added manually to the generated XML (not in URDF):

```xml
<keyframe>
  <key name="home" qpos="0 0 0 0 0 0 0"/>
  <key name="ready" qpos="0 -0.5 0 -1.5 0 1.0 0"/>
</keyframe>
```

Add this section after the `<option>` tag in your MuJoCo XML to define initial poses and test configurations.

**Option B: Manual Creation**

If you need to create or modify actuators manually:

```xml
<actuator>
  <!-- MUST use these exact naming conventions -->
  <position name="act_pos_j1" joint="j1" kp="100.0" kv="0.0"/>
  <velocity name="act_vel_j1" joint="j1" kv="10.0"/>
  <motor name="act_tau_j1" joint="j1"/>
</actuator>
```

**Critical Requirements:**

- Actuator names must follow `act_{type}_{joint_name}` convention
- Position actuator should have `kv="0.0"` for MIT mode compatibility
- Each command interface in URDF must have corresponding actuator in MuJoCo
- Joint names in MuJoCo must match those declared in URDF `<ros2_control>` section

### Step 3: Configure Controller

```yaml
my_controller:
  ros__parameters:
    joints: [j1]

    # Choose which interfaces to claim
    command_interfaces: [position, velocity, effort]  # MIT mode
    # OR
    command_interfaces: [position]                     # Position mode
    # OR
    command_interfaces: [effort]                       # Effort mode

    state_interfaces: [position, velocity]
```

The system will automatically select the appropriate control mode based on which interfaces your controller claims.

---

## Troubleshooting

### Pendulum Falls Even with Gravity Compensation

**Check:**

1. Controller is active: `ros2 control list_controllers`
2. Gravity comp enabled: `ros2 param get /zordi_grav_comp_controller use_gravity_compensation`
3. Pinocchio installed: `python3 -c "import pinocchio; print('OK')"`
4. Simulation is unpaused: Check service status

### Validation Error: Missing Actuator

```
[ERROR] Joint 'j1' declares effort interface in URDF but no 'act_tau_j1'
actuator found in MuJoCo model.
```

**Solution:** Add the missing actuator to your MuJoCo XML file with the correct naming convention.

### KV Warning for MIT Mode

```
[WARN] Joint 'j1': Position actuator has kv=5.0 but effort interface is also exposed.
This will cause unwanted damping during MIT mode.
```

**Solution:** Set `kv="0.0"` in your position actuator definition.

### Controller Won't Activate

```bash
# Stop all controllers
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{deactivate_controllers: ['zordi_grav_comp_controller'], strictness: 1}"

# Then activate desired controller
ros2 control set_controller_state zordi_grav_comp_controller active
```

---

## Additional Upstream Examples

This package also includes upstream examples from mujoco_ros2_control:

- `cart_example_position.launch.py` - Cartpole with position control
- `cart_example_velocity.launch.py` - Cartpole with velocity control
- `cart_example_effort.launch.py` - Cartpole with effort control
- `diff_drive.launch.py` - Differential drive robot
- `tricycle_drive.launch.py` - Tricycle drive robot
- `camera_example.launch.py` - Camera sensor example
- `ft_sensor_example.launch.py` - Force/torque sensor example
- `gripper_mimic_joint_example.launch.py` - Mimic joint example

See the original mujoco_ros2_control documentation for details on these examples.

---

## Next Steps

1. **Adapt for your robot:**
   - Create URDF with proper interface declarations
   - Create matching MuJoCo model with correct actuator naming
   - Add keyframes for your desired initial poses

2. **Test control modes:**
   - Start with single-interface modes (position, velocity, or effort only)
   - Progress to MIT mode for advanced control

3. **Integrate custom controllers:**
   - Use `zordi_mit_controller` as reference for implementing gravity compensation
   - Follow ros2_control controller plugin guidelines

4. **Production deployment:**
   - Fine-tune PID gains for your specific hardware
   - Test disturbance rejection with external wrench service
   - Validate gravity compensation accuracy

---

## References

- **Architecture Overview:** `../doc/mujoco_ros2_control_updates.md`
- **zordi_mit_controller:** `~/ros2_ws/src/zordi_mit_controller/README.md`
- **MuJoCo Documentation:** <https://mujoco.readthedocs.io/>
- **ros2_control Documentation:** <https://control.ros.org/>

---

**Copyright 2025 Zordi, Inc. All rights reserved.**

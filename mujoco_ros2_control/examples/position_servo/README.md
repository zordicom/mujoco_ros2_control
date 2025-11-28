# Position Servo Example

This example demonstrates simulating a **position-controlled servo** (like Dynamixel) using MuJoCo.

## Actuator Type

Position servos have:
- Input: Position command only
- Internal PID loop with fixed gains
- User cannot change PD gains at runtime

## Key Configuration

### URDF (position_servo.urdf)

Only the `position` command interface is declared:

```xml
<joint name="joint1">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### MuJoCo Model (position_servo.xml)

Uses a `<position>` actuator with fixed kp/kv:

```xml
<actuator>
  <position name="act_pos_joint1" joint="joint1" kp="100" kv="10"/>
</actuator>
```

### Controller Config (position_servo.yaml)

Uses the standard `joint_trajectory_controller` in position mode:

```yaml
joint_trajectory_controller:
  ros__parameters:
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

## Running the Example

```bash
ros2 launch mujoco_ros2_control position_servo.launch.py
```

## Expected Behavior

- The pendulum will track position commands with the servo's internal PD control
- No gravity compensation is needed (servo handles position tracking)
- Steady-state errors are possible if gains are insufficient

## When to Use This Mode

- Simulating Dynamixel-based robots
- Simple position control applications
- When you don't need torque control or impedance control


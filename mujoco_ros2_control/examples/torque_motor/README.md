# Torque Motor Example

This example demonstrates simulating a **pure torque-controlled motor** (like Kuka iiwa joints) using MuJoCo.

## Actuator Type

Torque motors have:
- Input: Torque command only
- Internal current control (τ = Kt × I)
- Controller must compute ALL control (PD, gravity, dynamics)

## Key Configuration

### URDF (torque_motor.urdf)

Only the `effort` command interface is declared:

```xml
<joint name="joint1">
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### MuJoCo Model (torque_motor.xml)

Uses a `<motor>` actuator for direct torque passthrough:

```xml
<actuator>
  <motor name="act_tau_joint1" joint="joint1" gear="1"/>
</actuator>
```

### Controller Config (torque_motor.yaml)

Uses `ZordiJointController` in effort-only mode:

```yaml
zordi_joint_effort_controller:
  ros__parameters:
    command_interfaces: [effort]
    compute_pd_internally: true
    use_gravity_compensation: true
```

## Running the Example

```bash
ros2 launch mujoco_ros2_control torque_motor.launch.py
```

## Expected Behavior

- The pendulum will be held by gravity compensation
- Controller computes PD feedback and gravity compensation internally
- Full torque control authority

## When to Use This Mode

- Full torque control applications
- Testing torque-based controllers
- Impedance control with software-computed PD
- Maximum control authority needed


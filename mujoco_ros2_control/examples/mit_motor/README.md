# MIT Motor Example

This example demonstrates simulating a **MIT-mode quasi-direct-drive motor** (like Damiao DM-J4310) using MuJoCo.

## Actuator Type

MIT motors accept 5 values per control cycle:
- p_des: desired position
- v_des: desired velocity
- kp: stiffness gain (variable!)
- kd: damping gain (variable!)
- t_ff: feedforward torque

Internal control law: `τ = kp*(p_des - p) + kd*(v_des - v) + t_ff`

**Key feature**: Gains can change EVERY control cycle for variable impedance control.

## Key Configuration

### URDF (mit_motor.urdf)

All 5 command interfaces are declared:

```xml
<joint name="joint1">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <command_interface name="kp"/>
  <command_interface name="kd"/>
  <param name="max_kp">500</param>
  <param name="max_kd">50</param>
</joint>
```

### MuJoCo Model (mit_motor.xml)

Uses motor actuator (position/velocity actuators are neutralized by hardware interface):

```xml
<actuator>
  <motor name="act_tau_joint1" joint="joint1" gear="1"/>
</actuator>
```

### Controller Config (mit_motor.yaml)

Uses `ZordiJointController` in MIT mode with gains from config:

```yaml
zordi_joint_mit_controller:
  ros__parameters:
    command_interfaces: [position, velocity, effort]
    compute_pd_internally: false  # MIT mode
    default_kp: [100.0]
    default_kd: [10.0]
```

## Running the Example

```bash
ros2 launch mujoco_ros2_control mit_motor.launch.py
```

## Expected Behavior

- The pendulum will track trajectories with configurable stiffness
- Hardware interface applies PD using controller-specified gains
- Gravity compensation is included in feedforward torque

## When to Use This Mode

- Simulating Damiao, Unitree, or similar actuators
- Variable impedance control applications
- Compliant manipulation
- Research on impedance control


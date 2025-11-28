# Actuator Types and MuJoCo Simulation

This guide explains the four common actuator types in robotics and how `mujoco_ros2_control` simulates each one.

## Overview

Modern robots use different types of actuators with fundamentally different control interfaces. Understanding these differences is critical for proper simulation setup and controller design.

| Actuator Type | Real Examples | Control Interface | Gains Location |
|---------------|---------------|-------------------|----------------|
| Position Servo | Dynamixel (position only), hobby servos | Position only | Fixed in firmware |
| Position+Velocity Servo | Dynamixel (profile position), ODrive | Position + Velocity | Fixed in firmware |
| Torque Motor | Kuka iiwa, research arms | Torque only | Controller computes all |
| MIT Motor | Damiao (MIT mode), Unitree, Cheetah | 5 values per cycle | Variable per command |

---

## Type 1: Position-Controlled Servos

### Real-World Examples

- Dynamixel X-series servos
- Hobby servos (RC servos)
- Traditional industrial robots (ABB, FANUC, KUKA)

### Characteristics

```
┌─────────────────────────────────────────────────────┐
│  Position Servo (e.g., Dynamixel XM430)             │
│                                                      │
│  Input: Position command only                        │
│  Internal: PID loop at 1-8 kHz (fixed gains)        │
│  Output: Motor current                               │
│                                                      │
│  User CANNOT change PD gains at runtime              │
│  Gains are burned into firmware or set once          │
└─────────────────────────────────────────────────────┘
```

### MuJoCo Simulation

Use MuJoCo's `<position>` actuator with fixed `kp`/`kv` gains:

```xml
<!-- MuJoCo model (*.xml) -->
<actuator>
  <position name="act_pos_joint1" joint="joint1" kp="100" kv="10"/>
</actuator>
```

Controller claims only the `position` interface:

```xml
<!-- URDF ros2_control section -->
<joint name="joint1">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### Compatible Controllers

- `ros2_controllers/joint_trajectory_controller` (position mode)
- Any controller that outputs position commands only

### When to Use

- Simulating traditional servo-based robots
- Simple position control applications
- When you don't need torque control or impedance control

---

## Type 2: Position+Velocity Servos

### Real-World Examples

- Dynamixel X-series servos (Profile Position mode)
- ODrive motor controllers
- Most EtherCAT/CiA 402 drives (Profile Position mode)
- Universal Robots (servoj command)

### Characteristics

```
┌─────────────────────────────────────────────────────────────────────┐
│  Position+Velocity Servo (e.g., Dynamixel Profile Position)         │
│                                                                      │
│  Input: Position command + Velocity command                          │
│  Internal: PD loop at 1-8 kHz (fixed gains)                         │
│  Control law: τ = kp*(p_des - p) + kd*(v_des - v)                   │
│  Output: Motor current                                               │
│                                                                      │
│  User CANNOT change PD gains at runtime                              │
│  Velocity is used as feedforward (target velocity at trajectory pt)  │
└─────────────────────────────────────────────────────────────────────┘
```

**Important Distinction:** This mode uses velocity as a **feedforward target** (the desired velocity at each trajectory point), NOT as a speed limit for ramping. This is different from Damiao's native "position-speed mode" where velocity specifies a maximum speed to reach the target position.

### MuJoCo Simulation

Use separate `<position>` and `<velocity>` actuators with the position actuator's `kv=0`:

```xml
<!-- MuJoCo model (*.xml) -->
<actuator>
  <!-- Position actuator: kp only, no velocity damping -->
  <position name="act_pos_joint1" joint="joint1" kp="100" kv="0"/>
  <!-- Velocity actuator: provides kd term -->
  <velocity name="act_vel_joint1" joint="joint1" kv="10"/>
</actuator>
```

This configuration produces the control law:
```
τ = kp*(pos_cmd - q) + kv*(vel_cmd - qd)
```

Controller claims both `position` and `velocity` interfaces:

```xml
<!-- URDF ros2_control section -->
<joint name="joint1">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### Compatible Controllers

- `ros2_controllers/joint_trajectory_controller` (position+velocity mode)
- Any controller that outputs both position and velocity commands

### Example Configuration

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
    command_interfaces:
      - position
      - velocity
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
```

### When to Use

- When using `joint_trajectory_controller` with velocity feedforward
- Simulating Dynamixel servos in Profile Position mode
- When you want smoother trajectory tracking than position-only mode
- Standard ROS trajectory execution applications

### Comparison with Damiao Position-Speed Mode

| Feature | Position+Velocity (This Mode) | Damiao Position-Speed Mode |
|---------|------------------------------|---------------------------|
| Velocity meaning | Target velocity at trajectory point | Speed **limit** for ramping |
| Control law | `τ = kp*(p_des - p) + kd*(v_des - v)` | Internal trajectory interpolation |
| Example | JTC sends pos=1.0, vel=0.5 → track at 0.5 rad/s | Move to pos=1.0 at max 0.5 rad/s |
| Supported | Yes | Not yet implemented |

---

## Type 3: Pure Torque Motors

### Real-World Examples

- Kuka iiwa joints (torque-controlled)
- High-end research manipulators
- Custom torque-controlled actuators

### Characteristics

```
┌─────────────────────────────────────────────────────┐
│  Torque Motor (e.g., Kuka iiwa joint)               │
│                                                      │
│  Input: Torque command                               │
│  Internal: Current control only (τ = Kt × I)        │
│  Output: Joint torque                                │
│                                                      │
│  External controller must compute EVERYTHING:        │
│  - PD feedback control                               │
│  - Gravity compensation                              │
│  - Coriolis compensation                             │
│  - Inertia feedforward                               │
└─────────────────────────────────────────────────────┘
```

### MuJoCo Simulation

Use MuJoCo's `<motor>` actuator for direct torque passthrough:

```xml
<!-- MuJoCo model (*.xml) -->
<actuator>
  <motor name="act_tau_joint1" joint="joint1" gear="1"/>
</actuator>
```

Controller claims only the `effort` interface:

```xml
<!-- URDF ros2_control section -->
<joint name="joint1">
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### Compatible Controllers

- `ZordiJointController` with `compute_pd_internally: true`
- `ZordiJointRNEAController` (effort-only mode)
- Any controller that computes torques internally

### Example Configuration

```yaml
zordi_joint_effort_controller:
  ros__parameters:
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    compute_pd_internally: true
    use_gravity_compensation: true
```

### When to Use

- Full torque control applications
- Impedance control (with software-computed PD)
- Testing torque-based controllers
- When you need maximum control authority

---

## Type 4: MIT-Mode / Quasi-Direct-Drive Motors

### Real-World Examples

- Damiao DM-J4310 (and other DM series)
- Unitree A1/B1 motors
- MIT Mini Cheetah actuators
- T-Motor AK-series

### Characteristics

```
┌─────────────────────────────────────────────────────┐
│  MIT Motor (e.g., Damiao DM-J4310)                  │
│                                                      │
│  Input: 5 values per CAN frame                       │
│    - p_des (desired position)                        │
│    - v_des (desired velocity)                        │
│    - kp (stiffness gain, 0-500)                     │
│    - kd (damping gain, 0-5)                         │
│    - t_ff (feedforward torque)                      │
│                                                      │
│  Internal control law:                               │
│    τ = kp*(p_des - p) + kd*(v_des - v) + t_ff       │
│                                                      │
│  Gains CAN CHANGE EVERY CONTROL CYCLE               │
│  Enables variable impedance control                  │
└─────────────────────────────────────────────────────┘
```

### MuJoCo Simulation

The hardware interface computes PD torque using controller-specified gains:

```xml
<!-- URDF ros2_control section - declares all 5 interfaces -->
<joint name="joint1">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <command_interface name="kp"/>
  <command_interface name="kd"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
  <!-- Safety limits (not default gains) -->
  <param name="max_kp">500</param>
  <param name="max_kd">50</param>
</joint>
```

MuJoCo model needs motor actuator (position/velocity actuators are neutralized):

```xml
<!-- MuJoCo model (*.xml) - needs all 3 actuator types -->
<actuator>
  <position name="act_pos_joint1" joint="joint1" kp="100" kv="0"/>
  <velocity name="act_vel_joint1" joint="joint1" kv="10"/>
  <motor name="act_tau_joint1" joint="joint1"/>
</actuator>
```

### Compatible Controllers

- `ZordiJointController` with `compute_pd_internally: false`
- `ZordiJointRNEAController` (MIT mode)
- Custom controllers that claim all 5 interfaces

### Example Configuration

```yaml
zordi_joint_mit_controller:
  ros__parameters:
    command_interfaces: [position, velocity, effort]
    state_interfaces: [position, velocity]
    compute_pd_internally: false  # MIT mode
    use_gravity_compensation: true
    # Gains from controller config (not URDF)
    default_kp: [100.0, 100.0, 80.0, 50.0, 30.0, 20.0, 10.0]
    default_kd: [10.0, 10.0, 8.0, 5.0, 3.0, 2.0, 1.0]
```

### When to Use

- Simulating modern quasi-direct-drive robots
- Variable impedance control (different stiffness for different tasks)
- Compliant manipulation
- Force-sensitive applications
- Research on impedance control

---

## Mode Summary

| Mode | Interfaces Claimed | Behavior |
|------|-------------------|----------|
| Position Servo | `[position]` | MuJoCo position actuator handles PD |
| Position+Velocity | `[position, velocity]` | Separate position/velocity actuators provide PD |
| Pure Torque | `[effort]` | Direct passthrough, controller computes all |
| MIT Mode | `[position, velocity, effort, kp, kd]` | Hardware computes PD with controller gains |

### Important: MIT Mode Validation

If a controller claims `[position, velocity, effort]` WITHOUT `[kp, kd]`, the hardware interface will **error out** with a clear message. This prevents misconfiguration where MIT-style control is expected but gains are missing.

---

## Choosing the Right Mode

### Use Position Servo Mode When:

- Simulating simple servo-based robots
- Position-only control applications
- You don't need velocity feedforward or torque control

### Use Position+Velocity Mode When:

- Using `joint_trajectory_controller` with velocity feedforward
- Simulating Dynamixel Profile Position mode or similar
- You want smoother trajectory tracking than position-only
- Standard ROS trajectory execution

### Use Torque Motor Mode When:

- Full torque control is required
- You want controller to compute all dynamics
- Testing pure torque-based algorithms

### Use MIT Mode When:

- Simulating Damiao (MIT mode), Unitree, or similar actuators
- Variable impedance control is needed
- You want hardware-level PD with configurable gains
- Compliant manipulation applications

---

## References

- [Damiao DM-J4310 User Manual](http://www.dmbot.cn/)
- [MuJoCo Actuator Documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#actuator)
- [MIT Mini Cheetah Paper](https://ieeexplore.ieee.org/document/8793865)

---

**Document Version:** 1.1
**Last Updated:** 2025-05-28


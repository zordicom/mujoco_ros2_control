# Variable Impedance Example

This example demonstrates **variable impedance control** using MIT mode, where stiffness (kp) and damping (kd) can change during execution.

## Concept

In variable impedance control, the robot's stiffness changes based on the task:
- **High stiffness**: For precise positioning
- **Low stiffness**: For compliant interaction (contact, human collaboration)
- **Zero stiffness**: For backdrivable operation (teaching mode)

## How It Works

1. The MIT motor example is launched
2. A Python script sends commands to change kp/kd dynamically
3. The robot behavior changes in real-time

## Running the Example

### Terminal 1: Launch the robot

```bash
ros2 launch mujoco_ros2_control mit_motor.launch.py
```

### Terminal 2: Run the variable impedance demo

```bash
ros2 run mujoco_ros2_control variable_impedance_demo.py
```

## Demo Sequence

The demo script cycles through different impedance profiles:

1. **Stiff mode** (kp=200, kd=20): Robot holds position firmly
2. **Compliant mode** (kp=20, kd=5): Robot is backdrivable
3. **Zero stiffness** (kp=0, kd=5): Pure damping, fully backdrivable

## Key Implementation

The demo script publishes to a custom topic that updates the controller's gains:

```python
# Example: Switch to compliant mode
msg = Float64MultiArray()
msg.data = [20.0, 5.0]  # [kp, kd]
self.gain_publisher.publish(msg)
```

## Future Work

For production use, you would:
1. Create a custom controller that accepts dynamic gain commands
2. Implement gain scheduling based on task state
3. Add safety limits for gain transitions

## References

- [Impedance Control: An Approach to Manipulation](https://ieeexplore.ieee.org/document/1087068)
- [Variable Impedance Actuators: A Review](https://www.sciencedirect.com/science/article/pii/S0921889013001188)


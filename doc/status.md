# Status & Architecture Update (November 2025)

## Overview

This document tracks the architectural status and recent major updates to `mujoco_ros2_control`.

---

## 1. Shared Simulation (Singleton Pattern)

**Status:** ✅ Implemented (Nov 30, 2025)

To support complex systems like bimanual robots where multiple `hardware_interface` instances (e.g., left arm, right arm) need to interact within the **same** physics world, `mujoco_ros2_control` now uses a Singleton pattern for the MuJoCo model and data.

### Mechanism

- **Shared State:** `mjModel*`, `mjData*`, and the ROS node are static members shared across all instances of `MujocoSystem`.
- **Primary/Secondary Logic:**
  - The **first** initialized interface becomes the **PRIMARY** instance.
  - Subsequent interfaces become **SECONDARY** instances.
  - Only the **PRIMARY** instance is responsible for:
    - Loading the XML model.
    - Creating the ROS node (`mujoco_system`).
    - Creating global services (`~/simulation_control`, `~/reset_to_keyframe`, `~/apply_external_wrench`).
    - Stepping the simulation (`mj_step1`, `mj_step2`).
    - Publishing `/clock` and `~/qfrc_bias`.
    - Managing the interactive viewer.

### Static Members

```cpp
static std::mutex static_mutex_;
static mjModel* shared_model_;
static mjData* shared_data_;
static int instance_count_;
static rclcpp::Node::SharedPtr shared_node_;
static std::string shared_model_path_;
```

### Lifecycle

| Phase | Primary Instance | Secondary Instance |
|-------|-----------------|-------------------|
| `on_init()` | Loads model, creates `mjData`, creates ROS node | Reuses shared model/data/node |
| `on_configure()` | Creates services, publishers, starts viewer | No-op for services/viewer |
| `read()` | Steps simulation, publishes clock | Only reads joint states |
| `write()` | Writes to `mj_data_->ctrl` | Writes to `mj_data_->ctrl` |
| Destructor | Frees model/data when last instance | Clears local pointers only |

### Model Path Validation

- All instances MUST specify the same `mujoco_model` file.
- If a secondary instance requests a different model path, initialization **fails** with an error.

---

## 2. Control Loop Timing

### Why Step in `read()`?

In ros2_control, the control loop is:

1. `read()` for ALL hardware interfaces
2. `update()` for ALL controllers
3. `write()` for ALL hardware interfaces

By stepping the simulation in the PRIMARY's `read()`:

- Commands from ALL interfaces (written in the previous cycle) are applied **before** the step.
- All interfaces read **consistent** post-step state.

This introduces a 1-cycle latency for secondary interfaces, which is standard for distributed hardware.

---

## 3. MIT Control Mode Support

**Status:** ✅ Implemented

Full support for MIT-style control (Position + Velocity + Feedforward Torque + Kp + Kd).

### Control Law

```
τ = τ_ff + kp * (q_cmd - q) + kd * (qd_cmd - qd)
```

Where:

- `τ_ff` = feedforward torque (effort command)
- `kp`, `kd` = dynamic gains from controller (via kp/kd command interfaces)
- `q_cmd`, `qd_cmd` = position/velocity setpoints

### Interface Requirements

Controllers MUST claim all 5 interfaces:

- `position`, `velocity`, `effort`, `kp`, `kd`

### Safety

- **Hard Error:** If a controller claims `effort` + (`position` OR `velocity`) but NOT `kp`/`kd`, `write()` returns `ERROR` immediately.
- **Gain Clamping:** `kp` and `kd` are clamped to `max_kp`/`max_kd` (configurable via URDF parameters).

---

## 4. Viewer Integration

**Status:** ✅ Threaded & Shared

- Runs in a dedicated background thread (PRIMARY only).
- Uses GLFW/OpenGL for rendering.
- Only **one** viewer per process (GLFW limitation).
- Enable via `<param name="mujoco_viewer">true</param>` in URDF.

**For bimanual setups:** Enable viewer only on ONE hardware interface (e.g., left arm). The right arm should have `mujoco_viewer` set to `false`.

---

## 5. Services (PRIMARY Instance Only)

| Service | Description |
|---------|-------------|
| `~/simulation_control` | Pause/unpause/reset/status |
| `~/reset_to_keyframe` | Reset to named or indexed keyframe |
| `~/apply_external_wrench` | Apply force/torque to a body |

All services are available at `/mujoco_system/...` (node name is `mujoco_system`).

---

## 6. Usage Notes

### Bimanual Robots

1. Define two `<ros2_control>` tags in your URDF (one per arm).
2. Both MUST point to the **same** `mujoco_model` XML file.
3. Enable `mujoco_viewer` on only ONE of them.

Example:

```xml
<!-- Left Arm (PRIMARY, with viewer) -->
<ros2_control name="left_arm" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <param name="mujoco_model">robot_bimanual.xml</param>
    <param name="mujoco_viewer">true</param>
  </hardware>
  <!-- joints... -->
</ros2_control>

<!-- Right Arm (SECONDARY, headless) -->
<ros2_control name="right_arm" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <param name="mujoco_model">robot_bimanual.xml</param>
    <param name="mujoco_viewer">false</param>
  </hardware>
  <!-- joints... -->
</ros2_control>
```

### Simulation Control

- Simulation starts **PAUSED**.
- Unpause via service:

  ```bash
  ros2 service call /mujoco_system/simulation_control \
    mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
  ```

---

## 7. Known Limitations

1. **Single Model Only:** All `MujocoSystem` instances must use the same XML model file. Loading different models is not supported.
2. **GLFW Single-Init:** Only one viewer can exist per process (GLFW constraint).
3. **1-Cycle Latency:** Secondary interfaces have a 1-cycle delay between command and state update.

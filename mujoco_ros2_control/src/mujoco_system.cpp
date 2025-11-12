// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/mujoco_system.hpp"

namespace mujoco_ros2_control
{
MujocoSystem::MujocoSystem() : logger_(rclcpp::get_logger("")) {}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  return std::move(command_interfaces_);
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // Joint states
  for (auto &joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.effort = mj_data_->qfrc_applied[joint_state.mj_vel_adr];
  }

  // IMU Sensor data
  for (auto &data : imu_sensor_data_)
  {
    data.orientation.data.w() = mj_data_->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = mj_data_->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = mj_data_->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = mj_data_->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = mj_data_->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() =
      mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() =
      mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() =
      mj_data_->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() =
      mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() =
      mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }

  // FT Sensor data
  for (auto &data : ft_sensor_data_)
  {
    data.force.data.x() = -mj_data_->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = -mj_data_->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = -mj_data_->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = -mj_data_->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = -mj_data_->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = -mj_data_->sensordata[data.torque.mj_sensor_index + 2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration &period)
{
  // update mimic joint
  for (auto &joint_state : joint_states_)
  {
    if (joint_state.is_mimic)
    {
      joint_state.position_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).position_command;
      joint_state.velocity_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).velocity_command;
      joint_state.effort_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).effort_command;
    }
  }
  // Joint states
  // Note: Controller switching is now handled via prepare/perform_command_mode_switch()
  // callbacks from the controller manager. No timeout-based detection needed.

  // Debug logging counter - incremented once per write() call, not per joint
  static int debug_counter = 0;
  debug_counter++;

  // Check for external forces applied to bodies (e.g., user dragging in viewer)
  // xfrc_applied contains [force_x, force_y, force_z, torque_x, torque_y, torque_z] per body
  if (debug_counter % 70000 == 0)
  {
    for (int body_id = 0; body_id < mj_model_->nbody; body_id++)
    {
      mjtNum* xfrc = &mj_data_->xfrc_applied[6 * body_id];

      // Calculate force and torque magnitudes
      double force_mag = std::sqrt(xfrc[0]*xfrc[0] + xfrc[1]*xfrc[1] + xfrc[2]*xfrc[2]);
      double torque_mag = std::sqrt(xfrc[3]*xfrc[3] + xfrc[4]*xfrc[4] + xfrc[5]*xfrc[5]);

      // Print if significant external force/torque detected
      if (force_mag > 1.0 || torque_mag > 0.5)
      {
        const char* body_name = mj_id2name(mj_model_, mjOBJ_BODY, body_id);
        RCLCPP_INFO(logger_, "[EXTERNAL INTERACTION] Body '%s': force=%.3f N, torque=%.3f N·m",
          body_name ? body_name : "unknown", force_mag, torque_mag);
      }
    }
  }

  for (auto &joint_state : joint_states_)
  {
    // ========================================================================
    // DYNAMIC MODE SWITCHING (Two Modes)
    // ========================================================================
    // position_servo: MuJoCo actuators (DAMIAO Position Mode)
    //                 - Triggered when: pos+vel interfaces active (no effort)
    // mit: Full MIT mode - τ = Kp*(p_cmd-p) + Kd*(v_cmd-v) + τ_ff (DAMIAO MIT Mode)
    //      - Triggered when: Any other interface combination
    //      - Handles effort-only naturally (Kp=0, Kd=0 when pos/vel not enabled)

    if (current_motor_mode_ == "mit")
    {
      // ====== MIT MODE ======
      // Combines position + velocity + effort (all three components)
      // Matches real DAMIAO hardware write() behavior exactly

      double torque = 0.0;

      // Component 1: Position feedback (if interface claimed by controller)
      if (joint_state.is_position_control_enabled)
      {
        double pos_error = joint_state.position_command - mj_data_->qpos[joint_state.mj_pos_adr];
        torque += joint_state.position_pid.computeCommand(pos_error, period.nanoseconds());
      }

      // Component 2: Velocity feedback (if interface claimed by controller)
      if (joint_state.is_velocity_control_enabled)
      {
        double vel_error = joint_state.velocity_command - mj_data_->qvel[joint_state.mj_vel_adr];
        torque += joint_state.velocity_pid.computeCommand(vel_error, period.nanoseconds());
      }

      // Component 3: Torque feedforward (if interface claimed by controller)
      if (joint_state.is_effort_control_enabled)
      {
        torque += joint_state.effort_command;
      }

      // Apply combined MIT mode torque
      mj_data_->qfrc_applied[joint_state.mj_vel_adr] = torque;
    }
    else if (current_motor_mode_ == "position_servo")
    {
      // ====== POSITION SERVO MODE ======
      // Uses MuJoCo position actuators (high stiffness servo model)
      // Triggered when trajectory controller active (pos+vel, no effort)

      if (joint_state.is_position_control_enabled)
      {
        if (joint_state.mj_actuator_id >= 0)
        {
          // Command MuJoCo position actuator
          mj_data_->ctrl[joint_state.mj_actuator_id] = joint_state.position_command;
        }
        else
        {
          // Fallback if no actuator found (shouldn't happen with proper XML)
          static bool warned = false;
          if (!warned) {
            RCLCPP_WARN(logger_,
              "position_servo mode: No actuator found for '%s', using direct qpos fallback",
              joint_state.name.c_str());
            warned = true;
          }
          mj_data_->qpos[joint_state.mj_pos_adr] = joint_state.position_command;
          mj_data_->qvel[joint_state.mj_vel_adr] = 0.0;
        }
      }
    }
    else
    {
      static bool error_logged = false;
      if (!error_logged) {
        RCLCPP_ERROR(logger_,
          "Unknown current_motor_mode_: '%s'. Must be: position_servo or mit",
          current_motor_mode_.c_str());
        error_logged = true;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::prepare_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces)
{
  // Verify that all interfaces exist and determine target mode
  bool has_position = false, has_velocity = false, has_effort = false;

  for (const auto &interface : start_interfaces)
  {
    RCLCPP_DEBUG(logger_, "Preparing to START interface: %s", interface.c_str());

    if (interface.find("/position") != std::string::npos) has_position = true;
    if (interface.find("/velocity") != std::string::npos) has_velocity = true;
    if (interface.find("/effort") != std::string::npos) has_effort = true;
  }
  for (const auto &interface : stop_interfaces)
  {
    RCLCPP_DEBUG(logger_, "Preparing to STOP interface: %s", interface.c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::perform_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces)
{
  // Detect which interfaces are being activated to determine motor mode
  // Matches real hardware behavior: controller switching triggers motor mode switching

  bool has_position = false, has_velocity = false, has_effort = false;

  for (const auto &interface : start_interfaces)
  {
    if (interface.find("/position") != std::string::npos) has_position = true;
    if (interface.find("/velocity") != std::string::npos) has_velocity = true;
    if (interface.find("/effort") != std::string::npos) has_effort = true;
  }

  // Determine motor mode based on interface combination
  std::string new_mode = current_motor_mode_;  // Default: keep current

  if (has_position && has_velocity && !has_effort)
  {
    // Trajectory controller (pos + vel only) → Position Servo Mode
    new_mode = "position_servo";
  }
  else
  {
    // All other combinations → MIT Mode (default, handles everything)
    // - pos + vel + effort → Full MIT mode
    // - effort only → MIT with Kp=0, Kd=0 (naturally)
    // - any other combination → MIT (most flexible)
    new_mode = "mit";
  }

  // Switch mode if changed
  if (new_mode != current_motor_mode_)
  {
    std::string old_mode = current_motor_mode_;
    current_motor_mode_ = new_mode;

    RCLCPP_INFO(logger_,
      "Motor mode switch: %s → %s (interfaces: pos=%d vel=%d eff=%d)",
      old_mode.c_str(), new_mode.c_str(),
      has_position, has_velocity, has_effort);
  }

  return hardware_interface::return_type::OK;
}

bool MujocoSystem::init_sim(
  mjModel *mujoco_model, mjData *mujoco_data, const urdf::Model &urdf_model,
  const hardware_interface::HardwareInfo &hardware_info)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  logger_ = rclcpp::get_logger("mujoco_system");

  register_joints(urdf_model, hardware_info);
  register_sensors(urdf_model, hardware_info);

  set_initial_pose();
  return true;
}

void MujocoSystem::register_joints(
  const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info)
{
  // control_mode parameter is deprecated (read for compatibility but unused)
  // Actual mode switches dynamically via current_motor_mode_
  auto control_mode_it = hardware_info.hardware_parameters.find("control_mode");
  if (control_mode_it != hardware_info.hardware_parameters.end())
  {
    control_mode_ = control_mode_it->second;
    RCLCPP_WARN(logger_, "control_mode parameter is deprecated (value '%s' ignored)", control_mode_.c_str());
    RCLCPP_INFO(logger_, "Mode switches dynamically: joint_trajectory_controller → position_servo, others → mit");
  }
  else
  {
    control_mode_ = "all";  // Kept for compatibility
    RCLCPP_INFO(logger_, "Dynamic mode switching enabled (starts in MIT mode)");
  }

  joint_states_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    int mujoco_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    if (mujoco_joint_id == -1)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find joint in mujoco model, joint name: " << joint.name);
      continue;
    }

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[mujoco_joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[mujoco_joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[mujoco_joint_id];

    // Look for corresponding position actuator (for position_servo mode)
    std::string actuator_name = "actuator_" + joint.name;
    int actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id >= 0)
    {
      joint_state.mj_actuator_id = actuator_id;
      // Only log for first joint to reduce spam
      if (joint.name.find("joint1") != std::string::npos) {
        RCLCPP_INFO(logger_, "Joint '%s' mapped to actuator %d (7 actuators total)",
                    joint.name.c_str(), actuator_id);
      }
    }

    joint_states_.at(joint_index) = joint_state;
    JointState &last_joint_state = joint_states_.at(joint_index);

    // get joint limit from urdf
    get_joint_limits(urdf_model.getJoint(last_joint_state.name), last_joint_state.joint_limits);

    // check if mimicked
    if (joint.parameters.find("mimic") != joint.parameters.end())
    {
      const auto mimicked_joint = joint.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo &info)
        { return info.name == mimicked_joint; });
      if (mimicked_joint_it == hardware_info.joints.end())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }
      last_joint_state.is_mimic = true;
      last_joint_state.mimicked_joint_index =
        std::distance(hardware_info.joints.begin(), mimicked_joint_it);

      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        last_joint_state.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      else
      {
        last_joint_state.mimic_multiplier = 1.0;
      }
    }

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.initial_value.empty())
      {
        double value = std::stod(interface_info.initial_value);
        return value;
      }
      else
      {
        return 0.0;
      }
    };

    // state interfaces
    for (const auto &state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position);
        last_joint_state.position = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity);
        last_joint_state.velocity = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort);
        last_joint_state.effort = get_initial_value(state_if);
      }
    }

    auto get_min_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.min.empty())
      {
        double value = std::stod(interface_info.min);
        return value;
      }
      else
      {
        return -1 * std::numeric_limits<double>::max();
      }
    };

    auto get_max_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.max.empty())
      {
        double value = std::stod(interface_info.max);
        return value;
      }
      else
      {
        return std::numeric_limits<double>::max();
      }
    };

    // command interfaces
    // overwrite joint limit with min/max value
    // Always enable all interfaces (mode switches dynamically)
    for (const auto &command_if : joint.command_interfaces)
    {
      if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      {
        // Always enable - dynamic mode switching handles behavior
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position_command);
        last_joint_state.is_position_control_enabled = true;
        last_joint_state.position_command = last_joint_state.position;
        // Start with position control active to hold initial pose during startup
        // This prevents the robot from falling before controllers are loaded
        // Controller manager will explicitly switch via perform_command_mode_switch()
        last_joint_state.position_command_active = true;
        // TODO(sangteak601): These are not used at all. Potentially can be removed.
        last_joint_state.min_position_command = get_min_value(command_if);
        last_joint_state.max_position_command = get_max_value(command_if);
      }
      else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      {
        // Always enable - dynamic mode switching handles behavior
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity_command);
        last_joint_state.is_velocity_control_enabled = true;
        last_joint_state.velocity_command = last_joint_state.velocity;
        // TODO(sangteak601): These are not used at all. Potentially can be removed.
        last_joint_state.min_velocity_command = get_min_value(command_if);
        last_joint_state.max_velocity_command = get_max_value(command_if);
      }
      else if (command_if.name == hardware_interface::HW_IF_EFFORT)
      {
        // Always enable - dynamic mode switching handles behavior
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort_command);
        last_joint_state.is_effort_control_enabled = true;
        last_joint_state.effort_command = last_joint_state.effort;
        last_joint_state.min_effort_command = get_min_value(command_if);
        last_joint_state.max_effort_command = get_max_value(command_if);
      }

      // For MuJoCo, always enable PID (we have gains in URDF)
      // This allows MIT mode to work
      last_joint_state.is_pid_enabled = true;
    }

    // Get PID gains, if needed
    if (last_joint_state.is_pid_enabled)
    {
      last_joint_state.position_pid = get_pid_gains(joint, hardware_interface::HW_IF_POSITION);
      last_joint_state.velocity_pid = get_pid_gains(joint, hardware_interface::HW_IF_VELOCITY);
    }
  }
}

void MujocoSystem::register_sensors(
  const urdf::Model & /* urdf_model */, const hardware_interface::HardwareInfo &hardware_info)
{
  // Assuming force/torque sensor end with "_fts" in the name,
  // and IMU sensor end with "_imu" in the name
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    std::string sensor_name = sensor.name;
    sensor_name = sensor_name.substr(0, sensor_name.rfind('_'));

    if (sensor.name.find("_fts") != std::string::npos)
    {
      FTSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.force.name = sensor_name + "_force";
      sensor_data.torque.name = sensor_name + "_torque";

      int force_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.force.name.c_str());
      int torque_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.torque.name.c_str());

      if (force_sensor_id == -1 || torque_sensor_id == -1)
      {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Failed to find force/torque sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.force.mj_sensor_index = mj_model_->sensor_adr[force_sensor_id];
      sensor_data.torque.mj_sensor_index = mj_model_->sensor_adr[torque_sensor_id];

      ft_sensor_data_.push_back(sensor_data);
      auto &last_sensor_data = ft_sensor_data_.back();

      for (const auto &state_if : sensor.state_interfaces)
      {
        if (state_if.name == "force.x")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.force.data.x());
        }
        else if (state_if.name == "force.y")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.force.data.y());
        }
        else if (state_if.name == "force.z")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.force.data.z());
        }
        else if (state_if.name == "torque.x")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.torque.data.x());
        }
        else if (state_if.name == "torque.y")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.torque.data.y());
        }
        else if (state_if.name == "torque.z")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.torque.data.z());
        }
      }
    }

    else if (sensor.name.find("_imu") != std::string::npos)
    {
      IMUSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.orientation.name = sensor_name + "_quat";
      sensor_data.angular_velocity.name = sensor_name + "_gyro";
      sensor_data.linear_acceleration.name = sensor_name + "_accel";

      int quat_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
      int gyro_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
      int accel_id =
        mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

      if (quat_id == -1 || gyro_id == -1 || accel_id == -1)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Failed to find IMU sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.orientation.mj_sensor_index = mj_model_->sensor_adr[quat_id];
      sensor_data.angular_velocity.mj_sensor_index = mj_model_->sensor_adr[gyro_id];
      sensor_data.linear_acceleration.mj_sensor_index = mj_model_->sensor_adr[accel_id];

      imu_sensor_data_.push_back(sensor_data);
      auto &last_sensor_data = imu_sensor_data_.back();

      for (const auto &state_if : sensor.state_interfaces)
      {
        if (state_if.name == "orientation.x")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.orientation.data.x());
        }
        else if (state_if.name == "orientation.y")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.orientation.data.y());
        }
        else if (state_if.name == "orientation.z")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.orientation.data.z());
        }
        else if (state_if.name == "orientation.w")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.orientation.data.w());
        }
        else if (state_if.name == "angular_velocity.x")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.angular_velocity.data.x());
        }
        else if (state_if.name == "angular_velocity.y")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.angular_velocity.data.y());
        }
        else if (state_if.name == "angular_velocity.z")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.angular_velocity.data.z());
        }
        else if (state_if.name == "linear_acceleration.x")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.linear_acceleration.data.x());
        }
        else if (state_if.name == "linear_acceleration.y")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.linear_acceleration.data.y());
        }
        else if (state_if.name == "linear_acceleration.z")
        {
          state_interfaces_.emplace_back(
            sensor.name, state_if.name, &last_sensor_data.linear_acceleration.data.z());
        }
      }
    }
  }
}

void MujocoSystem::set_initial_pose()
{
  for (auto &joint_state : joint_states_)
  {
    mj_data_->qpos[joint_state.mj_pos_adr] = joint_state.position;
  }
}

void MujocoSystem::get_joint_limits(
  urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits &joint_limits)
{
  if (urdf_joint->limits)
  {
    joint_limits.min_position = urdf_joint->limits->lower;
    joint_limits.max_position = urdf_joint->limits->upper;
    joint_limits.max_velocity = urdf_joint->limits->velocity;
    joint_limits.max_effort = urdf_joint->limits->effort;
  }
}

control_toolbox::Pid MujocoSystem::get_pid_gains(
  const hardware_interface::ComponentInfo &joint_info, std::string command_interface)
{
  double kp, ki, kd, i_max, i_min;
  std::string key;
  key = command_interface + std::string(PARAM_KP);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    kp = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    kp = 0.0;
  }

  key = command_interface + std::string(PARAM_KI);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    ki = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    ki = 0.0;
  }

  key = command_interface + std::string(PARAM_KD);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    kd = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    kd = 0.0;
  }

  bool enable_anti_windup = false;
  key = command_interface + std::string(PARAM_I_MAX);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    i_max = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  }
  else
  {
    i_max = std::numeric_limits<double>::max();
  }

  key = command_interface + std::string(PARAM_I_MIN);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    i_min = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  }
  else
  {
    i_min = std::numeric_limits<double>::lowest();
  }

  return control_toolbox::Pid(kp, ki, kd, i_max, i_min, enable_anti_windup);
}
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)

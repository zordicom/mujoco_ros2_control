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

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>
#include <fstream>

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

    // Effort: Read from qfrc_applied (both modes write here now)
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
  // update mimic commands
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

  // Drive actuators directly according to enabled command interfaces
  for (auto &joint_state : joint_states_)
  {
    const double q = mj_data_->qpos[joint_state.mj_pos_adr];
    const double qd = mj_data_->qvel[joint_state.mj_vel_adr];

    // Do not clear qfrc_applied here; reserve it for explicit fallback/diagnostics only

    // Determine if MIT mode is active (position/velocity commands with effort)
    // In MIT mode, we neutralize position/velocity actuators and drive torque actuator with PD
    //
    // Use *_command_active flags set by perform_command_mode_switch() to detect
    // which interfaces are actually claimed by controllers (not just exposed)
    bool mit_mode = joint_state.effort_command_active &&
                    (joint_state.position_command_active || joint_state.velocity_command_active);


    // Position actuator: command, neutralize, or neutralize for MIT
    if (joint_state.mj_pos_actuator_id >= 0 &&
        joint_state.mj_pos_actuator_id < mj_model_->nu)
    {
      if (joint_state.position_command_active && !mit_mode)
      {
        // Pure position mode: drive position actuator
        double pos_cmd = joint_state.position_command;
        if (joint_state.joint_limits.has_position_limits)
        {
          pos_cmd = clamp(
            pos_cmd,
            joint_state.joint_limits.min_position,
            joint_state.joint_limits.max_position);
        }
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = pos_cmd;
      }
      else
      {
        // Neutralize: either not claimed or MIT mode
        // WARNING: This neutralization (ctrl = q) assumes kv=0 for the position actuator.
        // If kv != 0, the position actuator will produce damping torque: τ = -kv*qd
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = q;

        // Check if position interface is exposed but not active, and kv != 0
        if (joint_state.is_position_control_enabled &&
            !joint_state.position_command_active &&
            !joint_state.warned_about_position_kv)
        {
          const int act_id = joint_state.mj_pos_actuator_id;

          // Read kv using same logic as initialization
          // For biastype=1: kv = -biasprm[2], otherwise kv = gainprm[1]
          const double kv = (mj_model_->actuator_biastype[act_id] == 1) ?
                            -mj_model_->actuator_biasprm[act_id * 10 + 2] :
                            mj_model_->actuator_gainprm[act_id * 10 + 1];

          if (std::abs(kv) > 1e-6)
          {
            RCLCPP_WARN(
              logger_,
              "Joint '%s': Position actuator has kv=%.3f but position interface is not active. "
              "This will introduce unwanted damping torque (τ = -%.3f * qd) during neutralization. "
              "For proper MIT mode operation, set kv=0.0 in the MuJoCo model.",
              joint_state.name.c_str(), kv, kv);
            joint_state.warned_about_position_kv = true;
          }
        }
      }
    }

    // Velocity actuator: command, neutralize, or neutralize for MIT
    if (joint_state.mj_vel_actuator_id >= 0 &&
        joint_state.mj_vel_actuator_id < mj_model_->nu)
    {
      if (joint_state.velocity_command_active && !mit_mode)
      {
        // Pure velocity mode: drive velocity actuator
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = joint_state.velocity_command;
      }
      else
      {
        // Neutralize: either not claimed or MIT mode
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = qd;
      }
    }

    // Torque actuator: command or neutralize
    // MIT-style: if position/velocity commands are active, compute PD torque and add to effort_command
    if (joint_state.mj_tau_actuator_id >= 0 &&
        joint_state.mj_tau_actuator_id < mj_model_->nu)
    {
      if (joint_state.effort_command_active)
      {
        double tau_total = joint_state.effort_command;

        // MIT-style PD composition: add PD torque if position or velocity commands are active
        // (This enables modes 5, 6, 7 from the comprehensive test)
        if (joint_state.position_command_active || joint_state.velocity_command_active)
        {
          double tau_pd = 0.0;

          // Position PD term (kp * position_error)
          if (joint_state.position_command_active && joint_state.is_pid_enabled)
          {
            double pos_err = joint_state.position_command - q;
            auto pos_gains = joint_state.position_pid.getGains();
            tau_pd += pos_gains.p_gain_ * pos_err;
          }

          // Velocity PD term (kd * velocity_error)
          if (joint_state.velocity_command_active && joint_state.is_pid_enabled)
          {
            double vel_err = joint_state.velocity_command - qd;
            auto vel_gains = joint_state.velocity_pid.getGains();
            tau_pd += vel_gains.d_gain_ * vel_err;
          }

          tau_total += tau_pd;
        }

        const double limit = joint_state.joint_limits.max_effort;
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] =
          clamp(tau_total, -limit, limit);
      }
      else
      {
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::prepare_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces)
{
  // Validate that the requested interface combination is feasible
  // In actuator-centric design, we accept any combination and handle it dynamically
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::perform_command_mode_switch(
  const std::vector<std::string> &start_interfaces,
  const std::vector<std::string> &stop_interfaces)
{
  // Update *_command_active flags to track which interfaces are claimed by controllers
  // This allows write() to detect true MIT mode vs. startup with all interfaces exposed

  for (const auto &interface_name : stop_interfaces)
  {
    // Parse "joint_name/interface_type"
    size_t pos = interface_name.find('/');
    if (pos == std::string::npos) continue;

    std::string joint_name = interface_name.substr(0, pos);
    std::string interface_type = interface_name.substr(pos + 1);

    // Find the joint and update its active flag
    for (auto &joint_state : joint_states_)
    {
      if (joint_state.name == joint_name)
      {
        if (interface_type == hardware_interface::HW_IF_POSITION)
        {
          joint_state.position_command_active = false;
          RCLCPP_INFO(logger_, "Stopped position interface for joint '%s'", joint_name.c_str());
        }
        else if (interface_type == hardware_interface::HW_IF_VELOCITY)
        {
          joint_state.velocity_command_active = false;
          RCLCPP_INFO(logger_, "Stopped velocity interface for joint '%s'", joint_name.c_str());
        }
        else if (interface_type == hardware_interface::HW_IF_EFFORT)
        {
          joint_state.effort_command_active = false;
          RCLCPP_INFO(logger_, "Stopped effort interface for joint '%s'", joint_name.c_str());
        }
        break;
      }
    }
  }

  for (const auto &interface_name : start_interfaces)
  {
    size_t pos = interface_name.find('/');
    if (pos == std::string::npos) continue;

    std::string joint_name = interface_name.substr(0, pos);
    std::string interface_type = interface_name.substr(pos + 1);

    for (auto &joint_state : joint_states_)
    {
      if (joint_state.name == joint_name)
      {
        if (interface_type == hardware_interface::HW_IF_POSITION)
        {
          joint_state.position_command_active = true;
          RCLCPP_INFO(logger_, "Started position interface for joint '%s'", joint_name.c_str());
        }
        else if (interface_type == hardware_interface::HW_IF_VELOCITY)
        {
          joint_state.velocity_command_active = true;
          RCLCPP_INFO(logger_, "Started velocity interface for joint '%s'", joint_name.c_str());
        }
        else if (interface_type == hardware_interface::HW_IF_EFFORT)
        {
          joint_state.effort_command_active = true;
          RCLCPP_INFO(logger_, "Started effort interface for joint '%s'", joint_name.c_str());
        }
        break;
      }
    }
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

  set_initial_pose();  // Applies URDF defaults to qpos

  // Override qpos if initial pose specified
  bool pose_override_applied = apply_initial_pose_override(hardware_info);

  // CRITICAL: Sync position_command with actual qpos (after override)
  // This ensures controllers command the pose we actually set, not URDF defaults
  for (auto &joint_state : joint_states_)
  {
    joint_state.position_command = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity_command = 0.0;
  }
  RCLCPP_INFO(logger_, "Initialized position commands from actual joint positions");

  // Actuator-centric control: no global mode state to set
  (void)pose_override_applied;
  RCLCPP_INFO(logger_, "Actuator-centric control initialized");
  RCLCPP_INFO(logger_, "MuJoCo model: nq=%d nv=%d nu=%d", mj_model_->nq, mj_model_->nv, mj_model_->nu);

  return true;
}

void MujocoSystem::register_joints(
  const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info)
{
  // No global control mode parameters in actuator-centric design

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

    // Look for corresponding actuators (position, velocity, motor)
    // New naming scheme: act_pos_*, act_vel_*, act_tau_*
    std::string pos_actuator_name = "act_pos_" + joint.name;
    std::string vel_actuator_name = "act_vel_" + joint.name;
    std::string tau_actuator_name = "act_tau_" + joint.name;

    joint_state.mj_pos_actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, pos_actuator_name.c_str());
    joint_state.mj_vel_actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, vel_actuator_name.c_str());
    joint_state.mj_tau_actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, tau_actuator_name.c_str());

    // Log actuator mapping
    RCLCPP_INFO(logger_, "Joint '%s' actuators: pos=%d, vel=%d, tau=%d",
                joint.name.c_str(),
                joint_state.mj_pos_actuator_id,
                joint_state.mj_vel_actuator_id,
                joint_state.mj_tau_actuator_id);

    // Backward compatibility: check old naming scheme if new names not found
    if (joint_state.mj_pos_actuator_id < 0) {
      std::string old_actuator_name = "actuator_" + joint.name;
      int old_actuator_id = mj_name2id(mj_model_, mjOBJ_ACTUATOR, old_actuator_name.c_str());
      if (old_actuator_id >= 0) {
        joint_state.mj_pos_actuator_id = old_actuator_id;
        RCLCPP_WARN(logger_, "Using legacy actuator naming for joint '%s' (actuator_%s)",
                    joint.name.c_str(), joint.name.c_str());
      }
    }

    // Check which command interfaces are declared in URDF
    // Used for both validation and MIT mode compatibility checks
    bool has_position_interface = false;
    bool has_velocity_interface = false;
    bool has_effort_interface = false;

    for (const auto &command_if : joint.command_interfaces)
    {
      if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
        has_position_interface = true;
      }
      if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
        has_velocity_interface = true;
      }
      if (command_if.name == hardware_interface::HW_IF_EFFORT) {
        has_effort_interface = true;
      }
    }

    // Validate position actuator kv parameter for MIT mode compatibility
    // Only warn if BOTH effort and position interfaces are exposed (indicating MIT mode usage)
    if (joint_state.mj_pos_actuator_id >= 0 && has_effort_interface)
    {
      const int act_id = joint_state.mj_pos_actuator_id;

      // Read kv parameter from MuJoCo actuator
      // For position actuators with biastype=1 (affine bias):
      //   Control law: τ = kp*(ctrl - q) + bias[0] + bias[1]*q + bias[2]*qd
      //   To match: τ = kp*(ctrl - q) - kv*qd
      //   MuJoCo sets: bias[1] = -kp, bias[2] = -kv
      //   So: kv = -biasprm[2]
      // For other bias types: kv is in gainprm[1]
      const double kv = (mj_model_->actuator_biastype[act_id] == 1) ?
                        -mj_model_->actuator_biasprm[act_id * 10 + 2] :
                        mj_model_->actuator_gainprm[act_id * 10 + 1];

      if (std::abs(kv) > 1e-6)
      {
        RCLCPP_WARN(
          logger_,
          "Joint '%s': Position actuator has kv=%.3f but effort interface is also exposed. "
          "During MIT mode (when position actuator is neutralized), this will cause "
          "unwanted damping (τ = -%.3f * qd). For MIT mode compatibility, set kv=0.0.",
          joint.name.c_str(), kv, kv);
      }
    }

    // Check for mismatches and fail initialization
    if (has_position_interface && joint_state.mj_pos_actuator_id < 0) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' declares position interface in URDF but no 'act_pos_%s' "
        "actuator found in MuJoCo model. Please add the actuator or remove the interface.",
        joint.name.c_str(), joint.name.c_str());
      throw std::runtime_error(
        "URDF/MuJoCo mismatch: position interface without actuator for joint " + joint.name);
    }

    if (has_velocity_interface && joint_state.mj_vel_actuator_id < 0) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' declares velocity interface in URDF but no 'act_vel_%s' "
        "actuator found in MuJoCo model. Please add the actuator or remove the interface.",
        joint.name.c_str(), joint.name.c_str());
      throw std::runtime_error(
        "URDF/MuJoCo mismatch: velocity interface without actuator for joint " + joint.name);
    }

    if (has_effort_interface && joint_state.mj_tau_actuator_id < 0) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' declares effort interface in URDF but no 'act_tau_%s' "
        "actuator found in MuJoCo model. Please add the actuator or remove the interface. "
        "MIT mode requires all three actuators (position, velocity, effort).",
        joint.name.c_str(), joint.name.c_str());
      throw std::runtime_error(
        "URDF/MuJoCo mismatch: effort interface without actuator for joint " + joint.name);
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
        // position_command_active starts false; set to true by perform_command_mode_switch()
        // when a controller claims this interface
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

bool MujocoSystem::apply_initial_pose_override(
  const hardware_interface::HardwareInfo &hardware_info)
{
  // Check if initial pose override is specified
  auto pose_name_it = hardware_info.hardware_parameters.find("initial_pose");
  auto config_path_it = hardware_info.hardware_parameters.find("initial_pose_config");

  if (pose_name_it == hardware_info.hardware_parameters.end() ||
      config_path_it == hardware_info.hardware_parameters.end())
  {
    RCLCPP_INFO(logger_, "No initial pose override - using URDF defaults");
    return false;
  }

  std::string pose_name = pose_name_it->second;
  std::string config_path = config_path_it->second;

  // Load YAML config with exception handling
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(logger_, "Failed to load pose config '%s': %s",
                 config_path.c_str(), e.what());
    return false;
  }

  if (!config["poses"] || !config["poses"][pose_name])
  {
    RCLCPP_ERROR(logger_, "Pose '%s' not found in %s",
                 pose_name.c_str(), config_path.c_str());
    return false;
  }

  auto pose = config["poses"][pose_name];

  RCLCPP_INFO(logger_, "Applying initial pose override: '%s'", pose_name.c_str());

  // Set MuJoCo state (qpos, qvel) and keep joint_states in sync
  int applied_count = 0;
  for (auto &joint_state : joint_states_)
  {
    // Extract joint key from name (e.g., "openarm_joint2" -> "joint2")
    // If "joint" not found, use the full joint name (e.g., "j1")
    size_t pos = joint_state.name.find("joint");
    std::string joint_key;
    if (pos == std::string::npos)
    {
      // Use full joint name for simple names like "j1", "j2", etc.
      joint_key = joint_state.name;
    }
    else
    {
      // Extract from "joint" onwards for names like "openarm_joint2"
      joint_key = joint_state.name.substr(pos);
    }

    if (pose[joint_key])
    {
      double new_position = pose[joint_key].as<double>();

      // Set position and velocity (part of MuJoCo state vector)
      mj_data_->qpos[joint_state.mj_pos_adr] = new_position;
      mj_data_->qvel[joint_state.mj_vel_adr] = 0.0;

      // Keep joint_state in sync
      joint_state.position = new_position;
      joint_state.velocity = 0.0;
      joint_state.position_command = new_position;
      joint_state.velocity_command = 0.0;

      RCLCPP_INFO(logger_, "  %s: %.3f rad (pos_cmd=%.3f)",
                  joint_state.name.c_str(), new_position, joint_state.position_command);
      applied_count++;
    }
  }

  // Forward dynamics: propagate state through kinematics and compute derived quantities
  // (body positions, Jacobians, sensor data, qacc, etc.)
  mj_forward(mj_model_, mj_data_);

  std::string description = "";
  if (pose["description"].IsDefined())
  {
    description = pose["description"].as<std::string>();
  }

  RCLCPP_INFO(logger_, "Applied initial pose '%s': %s (%d joints)",
              pose_name.c_str(), description.c_str(), applied_count);

  // DIAGNOSTIC: Verify qpos was actually set correctly
  RCLCPP_INFO(logger_, "VERIFY qpos after mj_forward: J2=%.4f, J4=%.4f",
              mj_data_->qpos[joint_states_[1].mj_pos_adr],
              mj_data_->qpos[joint_states_[3].mj_pos_adr]);

  return true;  // Successfully applied pose override
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

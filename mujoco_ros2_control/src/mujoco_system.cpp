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

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>

// For package path resolution
#include <ament_index_cpp/get_package_share_directory.hpp>

// For optional viewer
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "GLFW/glfw3.h"

namespace mujoco_ros2_control
{
// Static members initialization
std::mutex MujocoSystem::static_mutex_;
mjModel* MujocoSystem::shared_model_ = nullptr;
mjData* MujocoSystem::shared_data_ = nullptr;
int MujocoSystem::instance_count_ = 0;
rclcpp::Node::SharedPtr MujocoSystem::shared_node_ = nullptr;
std::string MujocoSystem::shared_model_path_ = "";

MujocoSystem::MujocoSystem() : logger_(rclcpp::get_logger("")) {}

MujocoSystem::~MujocoSystem()
{
  // Ensure viewer thread is stopped and joined before destruction
  if (viewer_thread_.joinable()) {
    stop_viewer_ = true;
    viewer_thread_.join();
  }

  // Ensure executor thread is stopped
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  // Free MuJoCo resources ONLY if this is the last instance
  std::lock_guard<std::mutex> lock(static_mutex_);
  instance_count_--;

  if (instance_count_ <= 0) {
    // Last instance exiting, clean up shared resources
    if (mj_data_) {
      mj_deleteData(mj_data_);
      mj_data_ = nullptr;
      shared_data_ = nullptr;
    }
    if (mj_model_) {
      mj_deleteModel(mj_model_);
      mj_model_ = nullptr;
      shared_model_ = nullptr;
    }
    shared_node_.reset();
    shared_model_path_ = "";
  } else {
    // Just clear local pointers
    mj_data_ = nullptr;
    mj_model_ = nullptr;
  }
}

CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger("mujoco_system");

  // Parse mujoco_model + mujoco_model_package
  auto model_it = info_.hardware_parameters.find("mujoco_model");
  auto pkg_it = info_.hardware_parameters.find("mujoco_model_package");

  if (model_it == info_.hardware_parameters.end() ||
      pkg_it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(logger_, "Missing required parameters: mujoco_model, mujoco_model_package");
    return CallbackReturn::ERROR;
  }

  std::string model_path;
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory(pkg_it->second);
    model_path = pkg_share + "/" + model_it->second;
    mujoco_model_path_ = model_path;
    RCLCPP_INFO(logger_, "MuJoCo model path: %s", mujoco_model_path_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to resolve package '%s': %s",
                 pkg_it->second.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  // Camera configuration
  auto cam_it = info_.hardware_parameters.find("enable_cameras");
  if (cam_it != info_.hardware_parameters.end()) {
    RCLCPP_INFO(logger_, "enable_cameras parameter found: '%s'", cam_it->second.c_str());
    // Handle both "true" and "True" (xacro uses Python-style capitalization)
    if (cam_it->second == "true" || cam_it->second == "True") {
      enable_cameras_ = true;

      auto rate_it = info_.hardware_parameters.find("camera_publish_rate");
      if (rate_it != info_.hardware_parameters.end()) {
        camera_publish_rate_ = std::stod(rate_it->second);
      }
      RCLCPP_INFO(logger_, "Cameras ENABLED (%.1f Hz)", camera_publish_rate_);
    } else {
      RCLCPP_INFO(logger_, "Cameras DISABLED (enable_cameras='%s')", cam_it->second.c_str());
    }
  } else {
    RCLCPP_WARN(logger_, "enable_cameras parameter NOT FOUND - cameras disabled by default");
  }

  // Viewer configuration
  auto viewer_it = info_.hardware_parameters.find("mujoco_viewer");
  if (viewer_it != info_.hardware_parameters.end()) {
    // Handle case-insensitive comparison (xacro may output "True" or "true")
    std::string value = viewer_it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "true") {
      mujoco_viewer_ = true;
      RCLCPP_INFO(logger_, "Interactive viewer will be enabled in on_configure()");
    }
  }

  // ---------------------------------------------------------
  // Shared Model Logic (Singleton)
  // ---------------------------------------------------------
  std::lock_guard<std::mutex> lock(static_mutex_);

  if (shared_model_ == nullptr) {
    // First instance: Load model and become PRIMARY
    char error[1000];
    shared_model_ = mj_loadXML(mujoco_model_path_.c_str(), 0, error, 1000);
    if (!shared_model_) {
      RCLCPP_ERROR(logger_, "Failed to load model: %s", error);
      return CallbackReturn::ERROR;
    }
    shared_data_ = mj_makeData(shared_model_);
    shared_model_path_ = mujoco_model_path_;

    // Create shared ROS node
    shared_node_ = rclcpp::Node::make_shared("mujoco_system");
    shared_node_->set_parameter(rclcpp::Parameter("use_sim_time", false));

    is_primary_ = true;
    RCLCPP_INFO(logger_, "Initialized PRIMARY instance (shared model created)");
    RCLCPP_INFO(logger_, "MuJoCo model loaded: nq=%d nv=%d nu=%d",
                shared_model_->nq, shared_model_->nv, shared_model_->nu);
  } else {
    // Subsequent instances: Reuse model and become SECONDARY
    // Verify model path matches (basic safety check)
    if (mujoco_model_path_ != shared_model_path_) {
      RCLCPP_ERROR(logger_,
        "Model path mismatch! Shared model is '%s', but this instance requested '%s'. "
        "All hardware interfaces must use the same MuJoCo model file.",
        shared_model_path_.c_str(), mujoco_model_path_.c_str());
      return CallbackReturn::ERROR;
    }

    is_primary_ = false;
    RCLCPP_INFO(logger_, "Initialized SECONDARY instance (reusing shared model)");
  }

  // Use shared resources
  mj_model_ = shared_model_;
  mj_data_ = shared_data_;
  node_ = shared_node_;
  instance_count_++;

  // Parse URDF model
  urdf::Model urdf;
  if (!urdf.initString(info_.original_xml)) {
    RCLCPP_ERROR(logger_, "Failed to parse URDF");
    return CallbackReturn::ERROR;
  }

  // Register joints and sensors (populates state_interfaces_ and command_interfaces_)
  register_joints(urdf, info_);
  register_sensors(urdf, info_);

  RCLCPP_INFO(logger_, "MujocoSystem initialized successfully");

  return CallbackReturn::SUCCESS;
}

void MujocoSystem::create_services_and_publishers() {
  // ONLY the primary instance creates global services and publishers
  if (!is_primary_) {
    return;
  }

  // Node is already created in on_init (shared_node_)

  // Clock publisher
  if (!clock_publisher_) {
    clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  }

  // Diagnostic publisher
  if (!qfrc_bias_publisher_) {
    qfrc_bias_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/qfrc_bias", 10);
  }

  // Create services
  if (!reset_service_) {
    reset_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::ResetToKeyframe>(
      "~/reset_to_keyframe",
      std::bind(&MujocoSystem::handle_reset_to_keyframe, this,
                std::placeholders::_1, std::placeholders::_2));
  }

  if (!sim_control_service_) {
    sim_control_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::SimulationControl>(
      "~/simulation_control",
      std::bind(&MujocoSystem::handle_simulation_control, this,
                std::placeholders::_1, std::placeholders::_2));
  }

  if (!wrench_service_) {
    wrench_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>(
      "~/apply_external_wrench",
      std::bind(&MujocoSystem::handle_apply_external_wrench, this,
                std::placeholders::_1, std::placeholders::_2));
  }

  // Spin node in background if not already spinning
  if (!executor_thread_.joinable()) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  RCLCPP_INFO(logger_, "Services available:");
  RCLCPP_INFO(logger_, "  - ~/reset_to_keyframe");
  RCLCPP_INFO(logger_, "  - ~/simulation_control");
  RCLCPP_INFO(logger_, "  - ~/apply_external_wrench");
}

CallbackReturn MujocoSystem::on_configure(const rclcpp_lifecycle::State& /* prev */) {
  RCLCPP_INFO(logger_, "Configuring MujocoSystem (%s)...", is_primary_ ? "PRIMARY" : "SECONDARY");

  // Create services and publishers (primary only)
  create_services_and_publishers();

  // =========================================================================
  // OpenGL Context Strategy:
  // All OpenGL rendering (viewer + cameras) must use the SAME context on ONE thread.
  // - If viewer enabled: viewer thread owns the context, cameras render there too
  // - If no viewer: main thread uses a hidden window for offscreen camera rendering
  // =========================================================================

  if (is_primary_ && mujoco_viewer_) {
    // Viewer mode: viewer thread handles both viewer AND camera rendering
    stop_viewer_ = false;
    viewer_thread_ = std::thread([this]() {
      // Initialize GLFW in viewer thread (owns the OpenGL context)
      if (!glfwInit()) {
        RCLCPP_ERROR(logger_, "Failed to initialize GLFW for viewer");
        return;
      }

      // Create window and initialize rendering
      rendering_ = mujoco_ros2_control::MujocoRendering::get_instance();
      rendering_->init(mj_model_, mj_data_);

      // Initialize cameras on THIS thread if enabled (shares OpenGL context with viewer)
      if (enable_cameras_) {
        cameras_ = std::make_unique<MujocoCameras>(node_);
        cameras_->init(mj_model_);

        double physics_rate = 1.0 / mj_model_->opt.timestep;
        camera_interval_ = static_cast<int>(std::round(physics_rate / camera_publish_rate_));

        RCLCPP_INFO(logger_, "Cameras initialized (shared context with viewer): %d steps between frames",
                    camera_interval_);
      }

      RCLCPP_INFO(logger_, "MuJoCo viewer started%s", enable_cameras_ ? " with cameras" : "");

      // Rendering loop - handles both viewer and cameras
      while (!stop_viewer_ && !rendering_->is_close_flag_raised()) {
        rendering_->update();

        // Render cameras on this thread (same OpenGL context)
        if (cameras_ && camera_render_requested_.load()) {
          cameras_->update(mj_model_, mj_data_);
          camera_render_requested_.store(false);
        }

        glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }

      RCLCPP_DEBUG(logger_, "Viewer thread stopping...");

      // Clean up cameras first (uses OpenGL context)
      if (cameras_) {
        cameras_->close();
        cameras_.reset();
      }

      if (rendering_) {
        rendering_->close();
      }
      glfwTerminate();
      RCLCPP_DEBUG(logger_, "Viewer thread stopped");
    });
  } else if (is_primary_ && enable_cameras_) {
    // Headless camera mode: main thread uses hidden window for offscreen rendering
    if (glfwInit()) {
      glfw_initialized_ = true;
      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
      camera_gl_window_ = glfwCreateWindow(1, 1, "mujoco_camera_ctx", nullptr, nullptr);
      if (camera_gl_window_) {
        glfwMakeContextCurrent(camera_gl_window_);

        cameras_ = std::make_unique<MujocoCameras>(node_);
        cameras_->init(mj_model_);

        double physics_rate = 1.0 / mj_model_->opt.timestep;
        camera_interval_ = static_cast<int>(std::round(physics_rate / camera_publish_rate_));

        RCLCPP_INFO(logger_, "Cameras initialized (headless): %d steps between frames", camera_interval_);
      } else {
        RCLCPP_WARN(logger_, "Failed to create offscreen window - cameras disabled");
        enable_cameras_ = false;
        glfwTerminate();
        glfw_initialized_ = false;
      }
    } else {
      RCLCPP_WARN(logger_, "Failed to initialize GLFW - cameras disabled (no display?)");
      enable_cameras_ = false;
    }
  }

  RCLCPP_INFO(logger_, "MujocoSystem configured successfully");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_activate(const rclcpp_lifecycle::State& /* prev */) {
  RCLCPP_INFO(logger_, "Activating MujocoSystem...");

  // Try to load initial keyframe if specified, otherwise reset to zero
  if (!load_keyframe(info_)) {
    // No keyframe loaded, reset to default state
    mj_resetData(mj_model_, mj_data_);

    // Sync commands with state
    for (auto& joint : joint_states_) {
      joint.position_command = mj_data_->qpos[joint.mj_pos_adr];
      joint.velocity_command = 0.0;
      joint.effort_command = 0.0;
    }
  }

  // Start in PAUSED state (user must unpause via service)
  {
    std::lock_guard<std::mutex> lock(sim_state_mutex_);
    sim_state_ = SimulationState::PAUSED;
  }

  RCLCPP_INFO(logger_, "MujocoSystem active with %zu joints", joint_states_.size());
  RCLCPP_INFO(logger_, "Simulation is PAUSED - use ~/simulation_control service to unpause");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_deactivate(const rclcpp_lifecycle::State& /* prev */) {
  RCLCPP_INFO(logger_, "Deactivating MujocoSystem...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystem::on_cleanup(const rclcpp_lifecycle::State& /* prev */) {
  RCLCPP_INFO(logger_, "Cleaning up MujocoSystem (%s)...", is_primary_ ? "PRIMARY" : "SECONDARY");

  // Stop viewer thread if running (PRIMARY only owns the viewer)
  if (is_primary_ && viewer_thread_.joinable()) {
    stop_viewer_ = true;
    viewer_thread_.join();
    RCLCPP_DEBUG(logger_, "Viewer thread stopped");
  }

  // Clean up cameras
  if (cameras_) {
    cameras_->close();
    cameras_.reset();
  }

  // Clean up camera OpenGL window
  if (camera_gl_window_) {
    glfwDestroyWindow(camera_gl_window_);
    camera_gl_window_ = nullptr;
  }

  // Terminate GLFW if we initialized it (after viewer and cameras are done)
  if (glfw_initialized_) {
    glfwTerminate();
    glfw_initialized_ = false;
    RCLCPP_DEBUG(logger_, "GLFW terminated");
  }

  // Stop executor thread (PRIMARY only owns the executor)
  if (is_primary_) {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  // NOTE: Do NOT free mj_model_/mj_data_ here!
  // These are shared resources managed by the singleton pattern.
  // They are freed in the destructor when instance_count_ drops to 0.
  // Just clear the local pointers.
  mj_data_ = nullptr;
  mj_model_ = nullptr;

  return CallbackReturn::SUCCESS;
}

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
  // ----------------------------------------------------------------
  // PRIMARY INSTANCE: STEP SIMULATION (moved from write)
  // ----------------------------------------------------------------
  // We step in read() to ensure commands from ALL interfaces (written in previous cycle)
  // are applied together. This introduces a 1-cycle delay for secondary interfaces,
  // which is standard for distributed hardware architectures.
  if (is_primary_)
  {
    // ALWAYS handle stepping, pause, services
    // Check for pending keyframe reset
    {
      std::lock_guard<std::mutex> lock(reset_mutex_);
      if (pending_reset_.pending) {
        reset_to_keyframe(pending_reset_.keyframe);
        pending_reset_.pending = false;

        // CRITICAL: Re-neutralize actuators after keyframe reset!
        // The ctrl values were set above based on the OLD positions before reset.
        // After keyframe reset, positions change, so we must update ctrl to match.
        for (auto &joint_state : joint_states_) {
          const double q = mj_data_->qpos[joint_state.mj_pos_adr];
          const double qd = mj_data_->qvel[joint_state.mj_vel_adr];

          // Neutralize position actuator to new position
          if (joint_state.mj_pos_actuator_id >= 0 &&
              joint_state.mj_pos_actuator_id < mj_model_->nu) {
            mj_data_->ctrl[joint_state.mj_pos_actuator_id] = q;
          }
          // Neutralize velocity actuator to new velocity
          if (joint_state.mj_vel_actuator_id >= 0 &&
              joint_state.mj_vel_actuator_id < mj_model_->nu) {
            mj_data_->ctrl[joint_state.mj_vel_actuator_id] = qd;
          }
          // Zero torque actuator
          if (joint_state.mj_tau_actuator_id >= 0 &&
              joint_state.mj_tau_actuator_id < mj_model_->nu) {
            mj_data_->ctrl[joint_state.mj_tau_actuator_id] = 0.0;
          }
        }
        RCLCPP_INFO(logger_, "Keyframe reset: re-neutralized all actuators");
      }
    }

    // Check if paused
    bool is_paused;
    {
      std::lock_guard<std::mutex> lock(sim_state_mutex_);
      is_paused = (sim_state_ == SimulationState::PAUSED);
    }

    if (is_paused) {
      // Paused: Update derived quantities without advancing time
      mj_forward(mj_model_, mj_data_);
    }
    else {
      // Step simulation
      mj_step1(mj_model_, mj_data_);

      // Apply external wrench if active
      {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        if (active_wrench_.active && active_wrench_.body_id >= 0) {
          mjtNum* xfrc = &mj_data_->xfrc_applied[6 * active_wrench_.body_id];
          double now = mj_data_->time;

          if (now <= active_wrench_.end_time) {
            xfrc[0] = active_wrench_.fx;
            xfrc[1] = active_wrench_.fy;
            xfrc[2] = active_wrench_.fz;
            xfrc[3] = active_wrench_.tx;
            xfrc[4] = active_wrench_.ty;
            xfrc[5] = active_wrench_.tz;
          } else {
            // Expired, clear
            for (int i = 0; i < 6; i++) xfrc[i] = 0.0;
            active_wrench_.active = false;
          }
        }
      }

      mj_step2(mj_model_, mj_data_);
    }

    // Publish clock
    double sim_time = mj_data_->time;
    int sec = static_cast<int>(sim_time);
    int nsec = static_cast<int>((sim_time - sec) * 1e9);
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = rclcpp::Time(sec, nsec, RCL_ROS_TIME);
    clock_publisher_->publish(clock_msg);

    // Publish qfrc_bias
    std_msgs::msg::Float64MultiArray qfrc_msg;
    qfrc_msg.data.resize(mj_model_->nv);
    for (int i = 0; i < mj_model_->nv; i++) {
      qfrc_msg.data[i] = mj_data_->qfrc_bias[i];
    }
    qfrc_bias_publisher_->publish(qfrc_msg);

    // Update cameras at specified interval
    if (cameras_ && ++camera_counter_ >= camera_interval_) {
      camera_counter_ = 0;
      if (mujoco_viewer_) {
        // Viewer mode: signal viewer thread to render cameras (same GL context)
        camera_render_requested_.store(true);
      } else {
        // Headless mode: render directly on main thread
        cameras_->update(mj_model_, mj_data_);
      }
    }
  } // End if (is_primary_)

  // Joint states (ALL instances read from shared data)
  for (auto &joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];

    // Effort: Sum of explicit actuator forces AND implicit solver forces
    // qfrc_actuator: explicit forces from actuators (e.g., torque actuators, PD error)
    // qfrc_smooth: implicit forces computed by MuJoCo's implicit integrator
    // For position-controlled actuators with implicit integration, the force needed to
    // hold a position against gravity appears in qfrc_smooth, not qfrc_actuator.
    // This sum represents what a real torque sensor would measure.
    joint_state.effort = mj_data_->qfrc_actuator[joint_state.mj_vel_adr] +
                         mj_data_->qfrc_smooth[joint_state.mj_vel_adr];
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
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
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

    // =======================================================================
    // Control Mode Detection and Actuator Driving
    // =======================================================================
    //
    // MODE 1: Position Servo (only position claimed)
    //   - Drive position actuator with position_command
    //   - MuJoCo's fixed kp/kv handle the PD control
    //   - Neutralize velocity/torque actuators
    //
    // MODE 2: Position+Velocity Servo (position + velocity claimed, no effort)
    //   - Drive position actuator with position_command (MuJoCo kp)
    //   - Drive velocity actuator with velocity_command (MuJoCo kv)
    //   - Neutralize torque actuator
    //
    // MODE 3: Pure Torque Motor (only effort claimed)
    //   - Neutralize position/velocity actuators
    //   - Drive torque actuator with effort_command (direct passthrough)
    //   - Controller computes all dynamics internally
    //
    // MODE 4: MIT Mode (effort + position + velocity + kp + kd claimed)
    //   - Neutralize position/velocity actuators (they have fixed gains)
    //   - Compute PD using DYNAMIC kp/kd from controller
    //   - τ = effort_command + kp*(pos_cmd - q) + kd*(vel_cmd - qd)
    //   - This matches real MIT motors (Damiao, Unitree, etc.)
    //
    // =======================================================================

    // Detect MIT mode: effort + (position OR velocity) + kp + kd all claimed
    // In MIT mode, we compute PD in software using dynamic gains
    bool mit_mode = joint_state.effort_command_active &&
                    (joint_state.position_command_active || joint_state.velocity_command_active) &&
                    joint_state.kp_command_active && joint_state.kd_command_active;

    // Position actuator control
    if (joint_state.mj_pos_actuator_id >= 0 &&
        joint_state.mj_pos_actuator_id < mj_model_->nu)
    {
      if (joint_state.position_command_active && !mit_mode)
      {
        // Position Servo or Position+Velocity mode: drive with commanded position
        // MuJoCo's position actuator applies its fixed kp
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
        // Neutralize: MIT mode, Pure Torque mode, or not claimed
        // For MuJoCo position actuators: τ = kp*(ctrl - q)
        // To achieve zero torque: ctrl = q
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = q;
      }
    }

    // Velocity actuator control
    if (joint_state.mj_vel_actuator_id >= 0 &&
        joint_state.mj_vel_actuator_id < mj_model_->nu)
    {
      if (joint_state.velocity_command_active && !mit_mode)
      {
        // Position+Velocity mode: drive with commanded velocity
        // MuJoCo's velocity actuator applies its fixed kv
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = joint_state.velocity_command;
      }
      else
      {
        // Neutralize: MIT mode, Pure Torque mode, or not claimed
        // For MuJoCo velocity actuators: τ = kv*(ctrl - qd)
        // To achieve zero torque: ctrl = qd
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = qd;
      }
    }

    // Torque actuator control
    if (joint_state.mj_tau_actuator_id >= 0 &&
        joint_state.mj_tau_actuator_id < mj_model_->nu)
    {
      if (joint_state.effort_command_active)
      {
        double tau_cmd = joint_state.effort_command;

        // MIT Mode: compute PD using dynamic kp/kd from controller
        // τ = τ_ff + kp*(pos_cmd - q) + kd*(vel_cmd - qd)
        if (mit_mode)
        {
          double kp = clamp(joint_state.kp_command, 0.0, joint_state.max_kp);
          double kd = clamp(joint_state.kd_command, 0.0, joint_state.max_kd);

          double pos_err = joint_state.position_command - q;
          double vel_err = joint_state.velocity_command - qd;

          tau_cmd += kp * pos_err + kd * vel_err;

          // Debug logging (throttled)
          static int mit_debug_count = 0;
          if (mit_debug_count % 10000 == 0)
          {
            RCLCPP_DEBUG(logger_,
              "MIT [%s]: pos_cmd=%.3f, q=%.3f, kp=%.1f, kd=%.1f, tau=%.2f",
              joint_state.name.c_str(), joint_state.position_command, q, kp, kd, tau_cmd);
          }
          mit_debug_count++;
        }

        const double limit = joint_state.joint_limits.max_effort;
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] = clamp(tau_cmd, -limit, limit);
      }
      else
      {
        // Effort interface not claimed: zero torque
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] = 0.0;
      }
    }
  }

  // MJ_STEP MOVED TO READ() for primary instance to synchronize writes

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::prepare_command_mode_switch(
  const std::vector<std::string> & /* start_interfaces */,
  const std::vector<std::string> & /* stop_interfaces */)
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
        }
        else if (interface_type == hardware_interface::HW_IF_VELOCITY)
        {
          joint_state.velocity_command_active = false;
        }
        else if (interface_type == hardware_interface::HW_IF_EFFORT)
        {
          joint_state.effort_command_active = false;
        }
        else if (interface_type == "kp")
        {
          joint_state.kp_command_active = false;
        }
        else if (interface_type == "kd")
        {
          joint_state.kd_command_active = false;
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
        }
        else if (interface_type == hardware_interface::HW_IF_VELOCITY)
        {
          joint_state.velocity_command_active = true;
        }
        else if (interface_type == hardware_interface::HW_IF_EFFORT)
        {
          joint_state.effort_command_active = true;
        }
        else if (interface_type == "kp")
        {
          joint_state.kp_command_active = true;
        }
        else if (interface_type == "kd")
        {
          joint_state.kd_command_active = true;
        }
        break;
      }
    }
  }

  // Log summary of mode switch (not per-joint details)
  RCLCPP_INFO(logger_, "Command mode switch: started %zu interfaces, stopped %zu interfaces",
              start_interfaces.size(), stop_interfaces.size());

  // Log the control mode for each joint at DEBUG level
  for (const auto &joint_state : joint_states_)
  {
    // MIT mode detection: effort + (position OR velocity) + kp + kd
    bool mit_mode = joint_state.effort_command_active &&
                    (joint_state.position_command_active || joint_state.velocity_command_active) &&
                    joint_state.kp_command_active && joint_state.kd_command_active;

    std::string mode;
    if (mit_mode)
    {
      mode = "MIT mode (dynamic kp/kd, software PD)";
    }
    else if (joint_state.effort_command_active && !joint_state.position_command_active &&
             !joint_state.velocity_command_active)
    {
      mode = "Pure torque motor (effort only)";
    }
    else if (joint_state.position_command_active && joint_state.velocity_command_active &&
             !joint_state.effort_command_active)
    {
      mode = "Position+Velocity servo (MuJoCo PD)";
    }
    else if (joint_state.position_command_active && !joint_state.effort_command_active)
    {
      mode = "Position servo (MuJoCo PD)";
    }
    else if (joint_state.velocity_command_active && !joint_state.effort_command_active)
    {
      mode = "Velocity servo";
    }
    else
    {
      mode = "Mixed/custom";
    }

    RCLCPP_DEBUG(logger_, "Joint '%s' control mode: %s", joint_state.name.c_str(), mode.c_str());
  }

  return hardware_interface::return_type::OK;
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

    // Log actuator mapping (DEBUG level - per-joint details)
    RCLCPP_DEBUG(logger_, "Joint '%s' actuators: pos=%d, vel=%d, tau=%d",
                joint.name.c_str(),
                joint_state.mj_pos_actuator_id,
                joint_state.mj_vel_actuator_id,
                joint_state.mj_tau_actuator_id);

    // Debug: Log actuator parameters to understand MuJoCo's internal representation
    auto log_actuator_params = [&](int act_id, const std::string& name) {
      if (act_id >= 0) {
        const int dyn_type = mj_model_->actuator_dyntype[act_id];
        const int gain_type = mj_model_->actuator_gaintype[act_id];
        const int bias_type = mj_model_->actuator_biastype[act_id];
        const double gain0 = mj_model_->actuator_gainprm[act_id * 10];
        const double gain1 = mj_model_->actuator_gainprm[act_id * 10 + 1];
        const double bias0 = mj_model_->actuator_biasprm[act_id * 10];
        const double bias1 = mj_model_->actuator_biasprm[act_id * 10 + 1];
        const double bias2 = mj_model_->actuator_biasprm[act_id * 10 + 2];

        RCLCPP_DEBUG(logger_,
          "  %s: dyn=%d, gain_t=%d, bias_t=%d, gain=[%.2f,%.2f], bias=[%.2f,%.2f,%.2f]",
          name.c_str(), dyn_type, gain_type, bias_type, gain0, gain1, bias0, bias1, bias2);
      }
    };

    log_actuator_params(joint_state.mj_pos_actuator_id, "pos");
    log_actuator_params(joint_state.mj_vel_actuator_id, "vel");
    log_actuator_params(joint_state.mj_tau_actuator_id, "tau");

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
      else if (command_if.name == "kp")
      {
        // MIT mode: dynamic stiffness gain from controller
        command_interfaces_.emplace_back(
          joint.name, "kp", &last_joint_state.kp_command);
        // Read safety limit from URDF (max_kp param)
        auto max_it = joint.parameters.find("max_kp");
        if (max_it != joint.parameters.end()) {
          last_joint_state.max_kp = std::stod(max_it->second);
        }
        RCLCPP_DEBUG(logger_, "Joint '%s': kp interface registered (max_kp=%.1f)",
          joint.name.c_str(), last_joint_state.max_kp);
      }
      else if (command_if.name == "kd")
      {
        // MIT mode: dynamic damping gain from controller
        command_interfaces_.emplace_back(
          joint.name, "kd", &last_joint_state.kd_command);
        // Read safety limit from URDF (max_kd param)
        auto max_it = joint.parameters.find("max_kd");
        if (max_it != joint.parameters.end()) {
          last_joint_state.max_kd = std::stod(max_it->second);
        }
        RCLCPP_DEBUG(logger_, "Joint '%s': kd interface registered (max_kd=%.1f)",
          joint.name.c_str(), last_joint_state.max_kd);
      }
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

int MujocoSystem::find_keyframe_by_name(const std::string &name)
{
  for (int i = 0; i < mj_model_->nkey; i++)
  {
    int name_adr = mj_model_->name_keyadr[i];
    const char *key_name = &mj_model_->names[name_adr];
    if (std::strcmp(key_name, name.c_str()) == 0)
    {
      return i;
    }
  }
  return -1;  // Not found
}

bool MujocoSystem::load_keyframe(const hardware_interface::HardwareInfo &hardware_info)
{
  // Check if initial_keyframe parameter is specified
  auto keyframe_param_it = hardware_info.hardware_parameters.find("initial_keyframe");

  int keyframe_idx = -1;

  if (keyframe_param_it != hardware_info.hardware_parameters.end())
  {
    std::string keyframe_spec = keyframe_param_it->second;

    // Try to parse as integer index
    try
    {
      keyframe_idx = std::stoi(keyframe_spec);
      if (keyframe_idx < 0 || keyframe_idx >= mj_model_->nkey)
      {
        RCLCPP_ERROR(
          logger_,
          "Keyframe index %d out of range [0, %d)",
          keyframe_idx,
          mj_model_->nkey);
        keyframe_idx = -1;
      }
    }
    catch (const std::exception &)
    {
      // Not an integer, try to find by name
      keyframe_idx = find_keyframe_by_name(keyframe_spec);
      if (keyframe_idx < 0)
      {
        RCLCPP_ERROR(logger_, "Keyframe '%s' not found in model", keyframe_spec.c_str());
      }
    }
  }
  else if (mj_model_->nkey > 0)
  {
    // No parameter specified, but keyframes exist - use first one as fallback
    keyframe_idx = 0;
    RCLCPP_INFO(logger_, "No initial_keyframe parameter specified, using first keyframe (index 0)");
  }

  if (keyframe_idx < 0)
  {
    if (mj_model_->nkey == 0)
    {
      RCLCPP_INFO(logger_, "No keyframes defined in model - using URDF defaults");
    }
    return false;
  }

  // Load the keyframe
  mj_resetDataKeyframe(mj_model_, mj_data_, keyframe_idx);

  // Forward dynamics: propagate state through kinematics
  mj_forward(mj_model_, mj_data_);

  // Sync joint_states with loaded qpos/qvel
  for (auto &joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.position_command = joint_state.position;
    joint_state.velocity_command = 0.0;
  }

  // Get keyframe name for logging
  int name_adr = mj_model_->name_keyadr[keyframe_idx];
  const char *key_name = &mj_model_->names[name_adr];

  RCLCPP_INFO(
    logger_,
    "Loaded keyframe %d ('%s') with %d joint positions",
    keyframe_idx,
    key_name,
    static_cast<int>(joint_states_.size()));

  return true;
}

bool MujocoSystem::reset_to_keyframe(const std::string &keyframe_name_or_idx)
{
  int keyframe_idx = -1;

  // Try to parse as integer index
  try
  {
    keyframe_idx = std::stoi(keyframe_name_or_idx);
    if (keyframe_idx < 0 || keyframe_idx >= mj_model_->nkey)
    {
      RCLCPP_ERROR(
        logger_,
        "Keyframe index %d out of range [0, %d)",
        keyframe_idx,
        mj_model_->nkey);
      return false;
    }
  }
  catch (const std::exception &)
  {
    // Not an integer, try to find by name
    keyframe_idx = find_keyframe_by_name(keyframe_name_or_idx);
    if (keyframe_idx < 0)
    {
      RCLCPP_ERROR(logger_, "Keyframe '%s' not found in model", keyframe_name_or_idx.c_str());
      return false;
    }
  }

  // Load the keyframe
  mj_resetDataKeyframe(mj_model_, mj_data_, keyframe_idx);

  // Forward dynamics: propagate state through kinematics
  mj_forward(mj_model_, mj_data_);

  // Sync joint_states with loaded qpos/qvel
  for (auto &joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.position_command = joint_state.position;
    joint_state.velocity_command = 0.0;
  }

  // Get keyframe name for logging
  int name_adr = mj_model_->name_keyadr[keyframe_idx];
  const char *key_name = &mj_model_->names[name_adr];

  RCLCPP_INFO(logger_, "Reset to keyframe %d ('%s')", keyframe_idx, key_name);

  return true;
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

void MujocoSystem::handle_reset_to_keyframe(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Request> req,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Response> res)
{
  std::lock_guard<std::mutex> lock(reset_mutex_);
  pending_reset_.keyframe = req->keyframe;  // Field name is 'keyframe'
  pending_reset_.pending = true;
  res->success = true;
  res->message = "Reset to keyframe '" + req->keyframe + "' scheduled";
  RCLCPP_INFO(logger_, "Reset to keyframe '%s' requested", req->keyframe.c_str());
}

void MujocoSystem::handle_simulation_control(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Request> req,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Response> res)
{
  std::lock_guard<std::mutex> lock(sim_state_mutex_);

  if (req->command == "pause") {
    sim_state_ = SimulationState::PAUSED;
    res->success = true;
    res->message = "Simulation paused";
    res->current_state = "PAUSED";
    RCLCPP_INFO(logger_, "Simulation PAUSED");
  } else if (req->command == "unpause") {
    sim_state_ = SimulationState::RUNNING;
    res->success = true;
    res->message = "Simulation running";
    res->current_state = "RUNNING";
    RCLCPP_INFO(logger_, "Simulation RUNNING");
  } else if (req->command == "reset") {
    mj_resetData(mj_model_, mj_data_);
    res->success = true;
    res->message = "Simulation reset";
    res->current_state = (sim_state_ == SimulationState::RUNNING) ? "RUNNING" : "PAUSED";
    RCLCPP_INFO(logger_, "Simulation RESET");
  } else if (req->command == "status") {
    res->success = true;
    res->message = "Status query";
    res->current_state = (sim_state_ == SimulationState::RUNNING) ? "RUNNING" : "PAUSED";
  } else {
    res->success = false;
    res->message = "Unknown command: " + req->command;
    res->current_state = (sim_state_ == SimulationState::RUNNING) ? "RUNNING" : "PAUSED";
  }
}

void MujocoSystem::handle_apply_external_wrench(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Request> req,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Response> res)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);

  int body_id = mj_name2id(mj_model_, mjOBJ_BODY, req->body_name.c_str());
  if (body_id < 0) {
    res->accepted = false;  // Field name is 'accepted'
    res->message = "Body not found: " + req->body_name;
    return;
  }

  active_wrench_.body_id = body_id;
  active_wrench_.fx = req->wrench.force.x;
  active_wrench_.fy = req->wrench.force.y;
  active_wrench_.fz = req->wrench.force.z;
  active_wrench_.tx = req->wrench.torque.x;
  active_wrench_.ty = req->wrench.torque.y;
  active_wrench_.tz = req->wrench.torque.z;
  active_wrench_.end_time = mj_data_->time + req->duration;  // Convert to absolute time
  active_wrench_.active = true;

  res->accepted = true;
  res->message = "Wrench applied successfully";
  RCLCPP_INFO(logger_, "Wrench applied to '%s' for %.2fs", req->body_name.c_str(), req->duration);
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoSystem, hardware_interface::SystemInterface)

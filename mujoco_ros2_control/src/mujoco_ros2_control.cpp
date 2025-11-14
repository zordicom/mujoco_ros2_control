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

#include <mutex>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{
MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME),
      sim_state_(SimulationState::PAUSED)
{
  RCLCPP_INFO(logger_, "Simulation will start in PAUSED state");
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
}

std::string MujocoRos2Control::get_robot_description()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
    "robot_description_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description"))
  {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // Publish qfrc_bias for gravity compensation validation
  qfrc_bias_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mujoco/qfrc_bias", 10);

  std::string urdf_string = this->get_robot_description();

  // setup actuators and mechanism control node.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  try
  {
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterface>(
      "mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::LibraryLoadException &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<hardware_interface::ResourceManager>();

  try
  {
    resource_manager->load_urdf(urdf_string, false, false);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (auto &hardware : control_hardware_info)
  {
    // Add initial keyframe parameter from node to hardware info
    if (node_->has_parameter("initial_keyframe"))
    {
      hardware.hardware_parameters["initial_keyframe"] =
        node_->get_parameter("initial_keyframe").as_string();
      // Store initial keyframe name for reset command
      if (initial_keyframe_name_.empty())
      {
        initial_keyframe_name_ = node_->get_parameter("initial_keyframe").as_string();
      }
    }

    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try
    {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, hardware))
    {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    // Store raw pointer for service access before moving to resource_manager
    MujocoSystemInterface* system_ptr = mujoco_system.get();
    resource_manager->import_component(std::move(mujoco_system), hardware);
    mujoco_systems_.push_back(system_ptr);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace());
  cm_executor_->add_node(controller_manager_);

  // Add main node to executor so its services (e.g., apply_external_wrench) can respond
  cm_executor_->add_node(node_->get_node_base_interface());

  // Auto-compute update_rate from MuJoCo timestep if not explicitly set
  if (!controller_manager_->has_parameter("update_rate"))
  {
    // Derive update rate from MuJoCo model timestep
    // MuJoCo timestep (e.g., 0.001s) → update_rate (e.g., 1000 Hz)
    int auto_update_rate = static_cast<int>(1.0 / mj_model_->opt.timestep);
    controller_manager_->declare_parameter("update_rate", auto_update_rate);
    RCLCPP_INFO(
      logger_,
      "Auto-set controller update_rate=%d Hz from MuJoCo timestep=%.6f s",
      auto_update_rate, mj_model_->opt.timestep);
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // Service to apply external wrench (writes to mjData->xfrc_applied inside update loop)
  apply_external_wrench_srv_ =
    node_->create_service<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>(
      "apply_external_wrench",
      std::bind(
        &MujocoRos2Control::handle_apply_external_wrench, this,
        std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    logger_,
    "External wrench service ready at '~/apply_external_wrench' (uses xfrc_applied)");

  // Service to reset to keyframe
  reset_to_keyframe_srv_ =
    node_->create_service<mujoco_ros2_control_msgs::srv::ResetToKeyframe>(
      "reset_to_keyframe",
      std::bind(
        &MujocoRos2Control::handle_reset_to_keyframe, this,
        std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "Reset to keyframe service ready at '~/reset_to_keyframe'");

  // Service to control simulation execution (pause/unpause/reset)
  sim_control_srv_ =
    node_->create_service<mujoco_ros2_control_msgs::srv::SimulationControl>(
      "simulation_control",
      std::bind(
        &MujocoRos2Control::handle_simulation_control, this,
        std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(logger_, "Simulation control service ready at '~/simulation_control'");
}

void MujocoRos2Control::update()
{
  // Check for pending keyframe reset FIRST (must happen even when paused)
  // This allows reset command to work immediately
  {
    std::lock_guard<std::mutex> lock(reset_keyframe_mutex_);
    if (pending_reset_.pending)
    {
      RCLCPP_INFO(logger_, "Processing keyframe reset to '%s'", pending_reset_.keyframe.c_str());

      // Reset all systems to the specified keyframe
      bool success = true;
      for (auto* system : mujoco_systems_)
      {
        if (!system->reset_to_keyframe(pending_reset_.keyframe))
        {
          RCLCPP_ERROR(logger_, "Failed to reset system to keyframe '%s'", pending_reset_.keyframe.c_str());
          success = false;
        }
      }

      if (success)
      {
        RCLCPP_INFO(logger_, "Successfully reset to keyframe '%s'", pending_reset_.keyframe.c_str());
      }

      pending_reset_.pending = false;
    }
  }

  // Check if paused AFTER processing any resets
  bool is_paused = false;
  {
    std::lock_guard<std::mutex> lock(sim_state_mutex_);
    is_paused = (sim_state_ == SimulationState::PAUSED);
  }

  if (is_paused)
  {
    // When PAUSED: Allow controller manager to run (for state transitions, service calls)
    // but don't advance simulation time or execute physics
    auto frozen_time_sec = static_cast<int>(mj_data_->time);
    auto frozen_time_nsec = static_cast<int>((mj_data_->time - frozen_time_sec) * 1e9);
    rclcpp::Time frozen_time(frozen_time_sec, frozen_time_nsec, RCL_ROS_TIME);
    
    // Run controller manager with zero period so controllers can be managed
    // but commands won't affect the (frozen) simulation
    rclcpp::Duration zero_period(0, 0);
    controller_manager_->read(frozen_time, zero_period);
    controller_manager_->update(frozen_time, zero_period);
    controller_manager_->write(frozen_time, zero_period);
    
    // Publish frozen time
    publish_sim_time(frozen_time);
    return;
  }

  // Compute current sim time BEFORE stepping (controls apply to upcoming step)
  auto pre_time = mj_data_->time;
  int pre_time_sec = static_cast<int>(pre_time);
  int pre_time_nsec = static_cast<int>((pre_time - pre_time_sec) * 1000000000);
  rclcpp::Time pre_time_ros(pre_time_sec, pre_time_nsec, RCL_ROS_TIME);
  rclcpp::Duration pre_period = pre_time_ros - last_update_sim_time_ros_;

  // Read state and update controllers before stepping
  controller_manager_->read(pre_time_ros, pre_period);
  controller_manager_->update(pre_time_ros, pre_period);

  // First half-step (everything that depends on qpos)
  mj_step1(mj_model_, mj_data_);

  // Apply commands BETWEEN step1 and step2 so MuJoCo uses them in this step
  controller_manager_->write(pre_time_ros, pre_period);

  // Apply any active external wrench before finishing physics step (uses xfrc_applied)
  {
    std::lock_guard<std::mutex> lock(active_wrench_mutex_);
    if (active_wrench_.active && active_wrench_.body_id >= 0)
    {
      const double now = mj_data_->time;

      // On first application, convert duration to absolute end time
      if (active_wrench_.end_time < 100.0) {  // Heuristic: if < 100s, it's a duration not timestamp
        active_wrench_.end_time = now + active_wrench_.end_time;
        RCLCPP_INFO(logger_, "Activating external wrench on body_id=%d for %.2fs (until t=%.2fs)",
          active_wrench_.body_id, active_wrench_.end_time - now, active_wrench_.end_time);
      }

      mjtNum* xfrc = &mj_data_->xfrc_applied[6 * active_wrench_.body_id];
      if (now <= active_wrench_.end_time)
      {
        xfrc[0] = static_cast<mjtNum>(active_wrench_.fx);
        xfrc[1] = static_cast<mjtNum>(active_wrench_.fy);
        xfrc[2] = static_cast<mjtNum>(active_wrench_.fz);
        xfrc[3] = static_cast<mjtNum>(active_wrench_.tx);
        xfrc[4] = static_cast<mjtNum>(active_wrench_.ty);
        xfrc[5] = static_cast<mjtNum>(active_wrench_.tz);
      }
      else
      {
        // Clear and deactivate
        RCLCPP_INFO(logger_, "External wrench EXPIRED, clearing forces");
        xfrc[0] = xfrc[1] = xfrc[2] = xfrc[3] = xfrc[4] = xfrc[5] = 0.0;
        active_wrench_.active = false;
      }
    }
  }

  // Now read the NEW simulation time after completing step
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // Publish clock AFTER stepping, so published time matches current simulation state
  publish_sim_time(sim_time_ros);

  // Record last control time based on post-step time
  if (sim_period >= control_period_) {
    last_update_sim_time_ros_ = sim_time_ros;
  }

  mj_step2(mj_model_, mj_data_);

  // Publish qfrc_bias for validation (computed by mj_step2)
  std_msgs::msg::Float64MultiArray qfrc_bias_msg;
  qfrc_bias_msg.data.resize(mj_model_->nv);
  for (int i = 0; i < mj_model_->nv; i++)
  {
    qfrc_bias_msg.data[i] = mj_data_->qfrc_bias[i];
  }
  qfrc_bias_publisher_->publish(qfrc_bias_msg);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // Thread-safe monotonic clock guarantee:
  // Since controller_manager runs in a separate thread (cm_thread_), there's a race
  // condition where clock messages could be processed out of order. This check ensures
  // we never publish a time that goes backwards, which would cause RViz and other nodes
  // to reset their state. This is standard practice in multi-threaded ROS2 simulations.
  static rclcpp::Time last_published_time(0, 0, RCL_ROS_TIME);
  static std::mutex clock_mutex;

  std::lock_guard<std::mutex> lock(clock_mutex);
  if (sim_time <= last_published_time)
  {
    // Skip if time hasn't advanced (shouldn't happen with correct update() logic)
    return;
  }

  last_published_time = sim_time;
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

void MujocoRos2Control::handle_apply_external_wrench(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Request> request,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Response> response)
{
  // Lookup body id (thread-safe, model is read-only)
  const std::string body_name = request->body_name;
  int body_id = mj_name2id(mj_model_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0)
  {
    response->accepted = false;
    response->message = "Body not found: " + body_name;
    RCLCPP_WARN(logger_, "apply_external_wrench: body '%s' not found", body_name.c_str());
    return;
  }

  // For now, assume wrench is expressed in world frame. MuJoCo expects xfrc_applied to be
  // in world frame; if a future need arises, we can add body-frame support.
  const double fx = request->wrench.force.x;
  const double fy = request->wrench.force.y;
  const double fz = request->wrench.force.z;
  const double tx = request->wrench.torque.x;
  const double ty = request->wrench.torque.y;
  const double tz = request->wrench.torque.z;

  const double duration = std::max(0.0, request->duration);

  // IMPORTANT: Don't access mj_data_ from service thread without mutex
  // Store duration directly, and compute end_time in update() loop
  {
    std::lock_guard<std::mutex> lock(active_wrench_mutex_);
    active_wrench_.body_id = body_id;
    active_wrench_.fx = fx;
    active_wrench_.fy = fy;
    active_wrench_.fz = fz;
    active_wrench_.tx = tx;
    active_wrench_.ty = ty;
    active_wrench_.tz = tz;
    // Store current time atomically in update() loop, use duration for now
    active_wrench_.end_time = duration;  // Will be converted to absolute time in update()
    active_wrench_.active = duration > 0.0;
  }

  response->accepted = true;
  response->message = "Applied wrench to body '" + body_name + "' for " + std::to_string(duration) + "s";
  RCLCPP_INFO(
    logger_, "apply_external_wrench SERVICE RECEIVED: body='%s' (id=%d) F[%.2f,%.2f,%.2f] T[%.2f,%.2f,%.2f], dur=%.3fs",
    body_name.c_str(), body_id, fx, fy, fz, tx, ty, tz, duration);
}

void MujocoRos2Control::handle_reset_to_keyframe(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Request> request,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Response> response)
{
  std::string keyframe = request->keyframe;

  // Set flag for update loop to execute reset (thread-safe)
  {
    std::lock_guard<std::mutex> lock(reset_keyframe_mutex_);
    pending_reset_.keyframe = keyframe;
    pending_reset_.pending = true;
  }

  response->success = true;
  response->message = "Keyframe reset queued: " + keyframe;
  RCLCPP_INFO(logger_, "reset_to_keyframe SERVICE RECEIVED: keyframe='%s'", keyframe.c_str());
}

void MujocoRos2Control::handle_simulation_control(
  const std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Request> request,
  std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Response> response)
{
  const std::string command = request->command;

  // Validate command
  if (command != "pause" && command != "unpause" && command != "reset" && command != "status")
  {
    response->success = false;
    response->message = "Invalid command. Must be 'pause', 'unpause', 'reset', or 'status'";
    response->current_state = "";
    RCLCPP_WARN(logger_, "simulation_control SERVICE: Invalid command '%s'", command.c_str());
    return;
  }

  // Thread-safe state transition
  {
    std::lock_guard<std::mutex> lock(sim_state_mutex_);

    // Handle status query (read-only, no state change)
    if (command == "status")
    {
      response->success = true;
      response->message = (sim_state_ == SimulationState::PAUSED)
        ? "Simulation is paused"
        : "Simulation is running";
      response->current_state = (sim_state_ == SimulationState::PAUSED) ? "PAUSED" : "RUNNING";
      return;
    }

    if (command == "pause")
    {
      if (sim_state_ == SimulationState::PAUSED)
      {
        response->success = true;
        response->message = "Simulation already paused";
        response->current_state = "PAUSED";
        RCLCPP_INFO(logger_, "simulation_control: Already PAUSED");
      }
      else
      {
        sim_state_ = SimulationState::PAUSED;
        response->success = true;
        response->message = "Simulation paused";
        response->current_state = "PAUSED";
        RCLCPP_INFO(logger_, "simulation_control: PAUSED");
      }
    }
    else if (command == "unpause")
    {
      if (sim_state_ == SimulationState::RUNNING)
      {
        response->success = true;
        response->message = "Simulation already running";
        response->current_state = "RUNNING";
        RCLCPP_INFO(logger_, "simulation_control: Already RUNNING");
      }
      else
      {
        sim_state_ = SimulationState::RUNNING;
        response->success = true;
        response->message = "Simulation unpaused";
        response->current_state = "RUNNING";
        RCLCPP_INFO(logger_, "simulation_control: RUNNING");
      }
    }
    else if (command == "reset")
    {
      // Reset to initial keyframe and transition to PAUSED
      if (initial_keyframe_name_.empty())
      {
        response->success = false;
        response->message = "No initial keyframe configured";
        response->current_state = (sim_state_ == SimulationState::PAUSED) ? "PAUSED" : "RUNNING";
        RCLCPP_WARN(logger_, "simulation_control: Cannot reset, no initial keyframe configured");
        return;
      }

      // Queue the keyframe reset
      {
        std::lock_guard<std::mutex> reset_lock(reset_keyframe_mutex_);
        pending_reset_.keyframe = initial_keyframe_name_;
        pending_reset_.pending = true;
      }

      // Transition to PAUSED
      sim_state_ = SimulationState::PAUSED;
      response->success = true;
      response->message = "Reset to keyframe '" + initial_keyframe_name_ + "' and PAUSED";
      response->current_state = "PAUSED";
      RCLCPP_INFO(
        logger_, "simulation_control: RESET to keyframe '%s' and PAUSED",
        initial_keyframe_name_.c_str());
    }
  }
}

}  // namespace mujoco_ros2_control

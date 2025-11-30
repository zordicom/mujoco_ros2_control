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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

// Service messages
#include "mujoco_ros2_control_msgs/srv/reset_to_keyframe.hpp"
#include "mujoco_ros2_control_msgs/srv/simulation_control.hpp"
#include "mujoco_ros2_control_msgs/srv/apply_external_wrench.hpp"

// ROS messages
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Camera support
#include "mujoco_ros2_control/mujoco_cameras.hpp"

namespace mujoco_ros2_control
{
// Forward declaration for optional viewer
class MujocoRendering;

class MujocoSystem : public MujocoSystemInterface
{
public:
  MujocoSystem();
  ~MujocoSystem() override;

  // Lifecycle methods
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(
    const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // Command mode switching support (called by controller manager when controllers start/stop)
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) override;

  // Keyframe management
  bool reset_to_keyframe(const std::string &keyframe_name_or_idx) override;

  struct JointState
  {
    std::string name;
    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
    double min_position_command;
    double max_position_command;
    double min_velocity_command;
    double max_velocity_command;
    double min_effort_command;
    double max_effort_command;

    // MIT mode: dynamic gains from controller (True MIT mode support)
    double kp_command{0.0};           // Dynamic stiffness from controller
    double kd_command{0.0};           // Dynamic damping from controller

    // Safety limits for gains (from URDF max_kp/max_kd params)
    double max_kp{1000.0};            // Maximum allowed stiffness
    double max_kd{100.0};             // Maximum allowed damping

    bool is_position_control_enabled{false};
    bool is_velocity_control_enabled{false};
    bool is_effort_control_enabled{false};
    joint_limits::JointLimits joint_limits;
    bool is_mimic{false};
    int mimicked_joint_index;
    double mimic_multiplier;
    int mj_joint_type;
    int mj_pos_adr;
    int mj_vel_adr;
    // MuJoCo actuator IDs for multi-mode control
    int mj_pos_actuator_id{-1};  // Position actuator ID
    int mj_vel_actuator_id{-1};  // Velocity actuator ID
    int mj_tau_actuator_id{-1};  // Torque (motor) actuator ID

    // Track which command interface is currently active (set via controller manager callbacks)
    bool position_command_active{false};
    bool velocity_command_active{false};
    bool effort_command_active{false};
    bool kp_command_active{false};    // True when controller claims kp interface
    bool kd_command_active{false};    // True when controller claims kd interface

    // Warning flags to avoid spamming logs
    bool warned_about_position_kv{false};   // kv != 0 in neutralized position actuators
  };

  template <typename T>
  struct SensorData
  {
    std::string name;
    T data;
    int mj_sensor_index;
  };

  struct FTSensorData
  {
    std::string name;
    SensorData<Eigen::Vector3d> force;
    SensorData<Eigen::Vector3d> torque;
  };

  struct IMUSensorData
  {
    std::string name;
    SensorData<Eigen::Quaternion<double>> orientation;
    SensorData<Eigen::Vector3d> angular_velocity;
    SensorData<Eigen::Vector3d> linear_acceleration;
  };

private:
  void register_joints(
    const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info);
  void register_sensors(
    const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info);
  int find_keyframe_by_name(const std::string &name);
  bool load_keyframe(const hardware_interface::HardwareInfo &hardware_info);
  void get_joint_limits(
    urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits &joint_limits);
  double clamp(double v, double lo, double hi) { return (v < lo) ? lo : (hi < v) ? hi : v; }

  // Helper to create services and publishers (shared by both modes)
  void create_services_and_publishers();

  // Service handlers (NEW - adapted from MujocoRos2Control)
  void handle_reset_to_keyframe(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Request> req,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Response> res);

  void handle_simulation_control(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Request> req,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Response> res);

  void handle_apply_external_wrench(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Request> req,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Response> res);

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<JointState> joint_states_;
  std::vector<FTSensorData> ft_sensor_data_;
  std::vector<IMUSensorData> imu_sensor_data_;

  mjModel *mj_model_;
  mjData *mj_data_;

  rclcpp::Logger logger_;

  std::string mujoco_model_path_;

  // ROS infrastructure (for lifecycle mode)
  rclcpp::Node::SharedPtr node_;  // For services and publishers
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // Viewer (optional)
  bool mujoco_viewer_{false};
  mujoco_ros2_control::MujocoRendering* rendering_{nullptr};
  std::thread viewer_thread_;
  std::atomic<bool> stop_viewer_{false};

  // Clock publishing
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  // Diagnostic publishing
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr qfrc_bias_publisher_;

  // Services (NEW - from MujocoRos2Control)
  rclcpp::Service<mujoco_ros2_control_msgs::srv::ResetToKeyframe>::SharedPtr reset_service_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::SimulationControl>::SharedPtr sim_control_service_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>::SharedPtr wrench_service_;

  // Simulation state
  enum class SimulationState { PAUSED, RUNNING };
  SimulationState sim_state_{SimulationState::PAUSED};  // Start paused
  mutable std::mutex sim_state_mutex_;

  // External wrench (NEW)
  struct ActiveWrench {
    int body_id = -1;
    double fx = 0.0, fy = 0.0, fz = 0.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
    double end_time = 0.0;
    bool active = false;
  };
  ActiveWrench active_wrench_;
  std::mutex wrench_mutex_;

  // Keyframe reset (NEW)
  struct PendingReset {
    std::string keyframe;
    bool pending = false;
  };
  PendingReset pending_reset_;
  std::mutex reset_mutex_;

  // Camera support
  std::unique_ptr<MujocoCameras> cameras_;
  bool enable_cameras_{false};
  double camera_publish_rate_{6.0};
  int camera_counter_{0};
  int camera_interval_{10};

  // Shared simulation state (Singleton pattern)
  static std::mutex static_mutex_;
  static mjModel* shared_model_;
  static mjData* shared_data_;
  static int instance_count_;
  static rclcpp::Node::SharedPtr shared_node_;
  static std::string shared_model_path_;

  bool is_primary_{false};  // True if this instance is responsible for stepping simulation
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

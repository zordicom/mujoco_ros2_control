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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "controller_manager/controller_manager.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mujoco/mujoco.h"

#include "mujoco_ros2_control/mujoco_system.hpp"
#include "mujoco_ros2_control_msgs/srv/apply_external_wrench.hpp"
#include "mujoco_ros2_control_msgs/srv/reset_to_keyframe.hpp"
#include "mujoco_ros2_control_msgs/srv/simulation_control.hpp"

namespace mujoco_ros2_control
{
class MujocoRos2Control
{
public:
  MujocoRos2Control(rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data);
  ~MujocoRos2Control();
  void init();
  void update();

private:
  void publish_sim_time(rclcpp::Time sim_time);
  std::string get_robot_description();
  void handle_apply_external_wrench(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Request> request,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::ApplyExternalWrench::Response> response);
  void handle_reset_to_keyframe(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Request> request,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetToKeyframe::Response> response);
  void handle_simulation_control(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Request> request,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::SimulationControl::Response> response);

  rclcpp::Node::SharedPtr node_;
  mjModel *mj_model_;
  mjData *mj_data_;

  rclcpp::Logger logger_;
  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> robot_hw_sim_loader_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::Executor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  bool stop_cm_thread_;
  rclcpp::Duration control_period_;

  rclcpp::Time last_update_sim_time_ros_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  // Diagnostic publisher for gravity compensation debugging
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr qfrc_bias_publisher_;

  // External wrench application (headless perturbations)
  struct ActiveWrench
  {
    int body_id = -1;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double end_time = 0.0;  // in simulation time
    bool active = false;
  };
  rclcpp::Service<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>::SharedPtr apply_external_wrench_srv_;
  std::mutex active_wrench_mutex_;
  ActiveWrench active_wrench_;

  // Keyframe reset (runtime reset to specific keyframes)
  struct PendingKeyframeReset
  {
    std::string keyframe;
    bool pending = false;
  };
  rclcpp::Service<mujoco_ros2_control_msgs::srv::ResetToKeyframe>::SharedPtr reset_to_keyframe_srv_;
  std::mutex reset_keyframe_mutex_;
  PendingKeyframeReset pending_reset_;

  // Simulation execution state (pause/unpause/reset)
  enum class SimulationState { PAUSED, RUNNING };
  SimulationState sim_state_;
  mutable std::mutex sim_state_mutex_;
  std::string initial_keyframe_name_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::SimulationControl>::SharedPtr sim_control_srv_;

  std::vector<MujocoSystemInterface*> mujoco_systems_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

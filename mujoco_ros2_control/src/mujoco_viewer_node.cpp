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

#include <chrono>
#include <thread>
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control_msgs/srv/reset_to_keyframe.hpp"
#include "mujoco_ros2_control_msgs/srv/simulation_control.hpp"
#include "mujoco_ros2_control_msgs/srv/apply_external_wrench.hpp"

class MujocoViewerNode : public rclcpp::Node {
public:
  MujocoViewerNode() : Node("mujoco_viewer") {
    // Parameters
    auto model_path = declare_parameter<std::string>("mujoco_model_path");
    service_ns_ = declare_parameter<std::string>("service_namespace", "/mujoco_system");

    // Load MuJoCo model (for visualization only)
    char error[1000];
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!model_) {
      RCLCPP_FATAL(get_logger(), "Failed to load model: %s", error);
      throw std::runtime_error(error);
    }
    data_ = mj_makeData(model_);

    RCLCPP_INFO(get_logger(), "Loaded MuJoCo model for visualization");
    RCLCPP_INFO(get_logger(), "Model: nq=%d nv=%d nu=%d", model_->nq, model_->nv, model_->nu);

    // Initialize GLFW viewer (standard MuJoCo viewer)
    if (!glfwInit()) {
      throw std::runtime_error("GLFW init failed");
    }
    viewer_ = mujoco_ros2_control::MujocoRendering::get_instance();
    viewer_->init(model_, data_);

    RCLCPP_INFO(get_logger(), "MuJoCo viewer initialized");
    RCLCPP_INFO(get_logger(), "Keyboard controls:");
    RCLCPP_INFO(get_logger(), "  P - Pause/unpause simulation");
    RCLCPP_INFO(get_logger(), "  R - Reset to home keyframe");
    RCLCPP_INFO(get_logger(), "  Backspace - Reset simulation");
    RCLCPP_INFO(get_logger(), "  Mouse - Standard MuJoCo camera controls");

    // Subscribe to joint states from actual simulation
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Update visualization model with actual joint states
        for (size_t i = 0; i < msg->name.size(); i++) {
          int idx = mj_name2id(model_, mjOBJ_JOINT, msg->name[i].c_str());
          if (idx >= 0 && idx < model_->nq) {
            data_->qpos[idx] = msg->position[i];
          }
          if (idx >= 0 && idx < model_->nv && i < msg->velocity.size()) {
            data_->qvel[idx] = msg->velocity[i];
          }
        }
        mj_forward(model_, data_);  // Update derived quantities
      });

    // Service clients for interactive controls
    sim_ctrl_ = create_client<mujoco_ros2_control_msgs::srv::SimulationControl>(
      service_ns_ + "/simulation_control");
    reset_client_ = create_client<mujoco_ros2_control_msgs::srv::ResetToKeyframe>(
      service_ns_ + "/reset_to_keyframe");
    wrench_client_ = create_client<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>(
      service_ns_ + "/apply_external_wrench");

    RCLCPP_INFO(get_logger(), "Viewer ready - subscribed to /joint_states");
    RCLCPP_INFO(get_logger(), "Service namespace: %s", service_ns_.c_str());
  }

  ~MujocoViewerNode() {
    if (viewer_) viewer_->close();
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
  }

  void run() {
    // Main viewer loop (60 Hz rendering)
    RCLCPP_INFO(get_logger(), "Starting viewer loop...");

    while (rclcpp::ok() && !viewer_->is_close_flag_raised()) {
      // Spin ROS callbacks (process joint states)
      rclcpp::spin_some(get_node_base_interface());

      // Render visualization
      viewer_->update();

      // 60 Hz rendering
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    RCLCPP_INFO(get_logger(), "Viewer closed");
  }

private:
  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  mujoco_ros2_control::MujocoRendering* viewer_{nullptr};

  std::string service_ns_;
  bool is_paused_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Client<mujoco_ros2_control_msgs::srv::SimulationControl>::SharedPtr sim_ctrl_;
  rclcpp::Client<mujoco_ros2_control_msgs::srv::ResetToKeyframe>::SharedPtr reset_client_;
  rclcpp::Client<mujoco_ros2_control_msgs::srv::ApplyExternalWrench>::SharedPtr wrench_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MujocoViewerNode>();
    node->run();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_viewer"), "Fatal error: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

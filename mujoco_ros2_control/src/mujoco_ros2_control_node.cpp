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

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");

  // Declare required parameters with defaults (if not already declared)
  if (!node->has_parameter("headless")) {
    node->declare_parameter("headless", false);
  }
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter("use_sim_time", true);
  }

  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // Check if headless mode is enabled (no viewer window)
  bool headless = node->get_parameter("headless").as_bool();
  if (headless)
  {
    RCLCPP_INFO(node->get_logger(), "Running in HEADLESS mode (no viewer window)");
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // Initialize rendering and cameras (optional in headless mode)
  mujoco_ros2_control::MujocoRendering *rendering = nullptr;
  std::unique_ptr<mujoco_ros2_control::MujocoCameras> cameras = nullptr;

  if (!headless)
  {
    // initialize mujoco visualization environment for rendering and cameras
    if (!glfwInit())
    {
      mju_error("Could not initialize GLFW");
    }
    rendering = mujoco_ros2_control::MujocoRendering::get_instance();
    rendering->init(mujoco_model, mujoco_data);
    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

    cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
    cameras->init(mujoco_model);
  }

  // run main loop with REAL-TIME SYNCHRONIZATION
  // Physics runs at model timestep rate (typically 1000 Hz = 1ms per step)
  // Each physics step is throttled to maintain 1:1 sim time to real time ratio

  // Get physics timestep from model
  const double physics_timestep = mujoco_model->opt.timestep;  // seconds
  const auto target_dt = std::chrono::duration<double>(physics_timestep);

  // Speedup measurement (optional - uncomment for debugging)
  // auto speedup_measurement_start = std::chrono::steady_clock::now();
  // double sim_time_at_measurement_start = mujoco_data->time;
  // int cycle_count = 0;

  RCLCPP_INFO(node->get_logger(),
    "Starting real-time synchronized simulation loop (timestep=%.6fs, target_rate=%.0f Hz)",
    physics_timestep, 1.0 / physics_timestep);

  if (headless)
  {
    // Headless mode: no rendering or cameras, just simulation with real-time sync
    RCLCPP_INFO(node->get_logger(), "Running in HEADLESS mode with real-time synchronization");

    while (rclcpp::ok())
    {
      auto cycle_start = std::chrono::steady_clock::now();

      // Single physics step
      mujoco_control.update();
      // cycle_count++;

      // Real-time synchronization: sleep to maintain 1:1 ratio
      auto elapsed = std::chrono::steady_clock::now() - cycle_start;
      if (elapsed < target_dt)
      {
        std::this_thread::sleep_for(target_dt - elapsed);
      }

      // Speedup logging disabled (uncomment for debugging)
      /*
      auto measurement_elapsed = std::chrono::steady_clock::now() - speedup_measurement_start;
      if (measurement_elapsed >= std::chrono::seconds(10))
      {
        double real_time_elapsed =
          std::chrono::duration<double>(measurement_elapsed).count();
        double sim_time_elapsed = mujoco_data->time - sim_time_at_measurement_start;
        double speedup = sim_time_elapsed / real_time_elapsed;

        RCLCPP_INFO(node->get_logger(),
          "Simulation speedup: %.2fx (sim=%.1fs, real=%.1fs, cycles=%d)",
          speedup, sim_time_elapsed, real_time_elapsed, cycle_count);

        if (speedup > 1.1)
        {
          RCLCPP_WARN(node->get_logger(),
            "Simulation running %.2fx faster than real-time! CPU too fast or throttling failed.",
            speedup);
        }
        else if (speedup < 0.9)
        {
          RCLCPP_WARN(node->get_logger(),
            "Simulation running %.2fx slower than real-time. CPU may be overloaded.",
            speedup);
        }

        speedup_measurement_start = std::chrono::steady_clock::now();
        sim_time_at_measurement_start = mujoco_data->time;
        cycle_count = 0;
      }
      */
    }
  }
  else
  {
    // Normal mode: with rendering at 60 Hz and real-time physics sync
    RCLCPP_INFO(node->get_logger(),
      "Running with RENDERING at 60 Hz and real-time synchronization");

    int render_counter = 0;
    const int render_interval = static_cast<int>(std::round((1.0 / 60.0) / physics_timestep));
    const int camera_interval = static_cast<int>(std::round((1.0 / 6.0) / physics_timestep));

    RCLCPP_INFO(node->get_logger(),
      "Render every %d physics steps (~60 Hz), cameras every %d steps (~6 Hz)",
      render_interval, camera_interval);

    while (rclcpp::ok() && !rendering->is_close_flag_raised())
    {
      auto cycle_start = std::chrono::steady_clock::now();

      // Single physics step
      mujoco_control.update();
      // cycle_count++;
      render_counter++;

      // Render at 60 Hz (every ~16 physics steps for 1ms timestep)
      if (render_counter >= render_interval)
      {
        rendering->update();
        render_counter = 0;
      }

      // Update cameras at 6 Hz
      static int camera_counter = 0;
      if (++camera_counter >= camera_interval)
      {
        cameras->update(mujoco_model, mujoco_data);
        camera_counter = 0;
      }

      // Real-time synchronization: sleep to maintain 1:1 ratio
      auto elapsed = std::chrono::steady_clock::now() - cycle_start;
      if (elapsed < target_dt)
      {
        std::this_thread::sleep_for(target_dt - elapsed);
      }

      // Speedup logging disabled (uncomment for debugging)
      /*
      auto measurement_elapsed = std::chrono::steady_clock::now() - speedup_measurement_start;
      if (measurement_elapsed >= std::chrono::seconds(10))
      {
        double real_time_elapsed =
          std::chrono::duration<double>(measurement_elapsed).count();
        double sim_time_elapsed = mujoco_data->time - sim_time_at_measurement_start;
        double speedup = sim_time_elapsed / real_time_elapsed;

        RCLCPP_INFO(node->get_logger(),
          "Simulation speedup: %.2fx (sim=%.1fs, real=%.1fs, cycles=%d)",
          speedup, sim_time_elapsed, real_time_elapsed, cycle_count);

        if (speedup > 1.1)
        {
          RCLCPP_WARN(node->get_logger(),
            "Simulation running %.2fx faster than real-time! CPU too fast or throttling failed.",
            speedup);
        }
        else if (speedup < 0.9)
        {
          RCLCPP_WARN(node->get_logger(),
            "Simulation running %.2fx slower than real-time. CPU may be overloaded.",
            speedup);
        }

        speedup_measurement_start = std::chrono::steady_clock::now();
        sim_time_at_measurement_start = mujoco_data->time;
        cycle_count = 0;
      }
      */
    }
  }

  if (rendering)
  {
    rendering->close();
  }
  if (cameras)
  {
    cameras->close();
  }

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}

// Copyright (c) 2025
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

#ifndef MUJOCO_ROS2_CONTROL__PYTHON_VIEWER_HPP_
#define MUJOCO_ROS2_CONTROL__PYTHON_VIEWER_HPP_

#ifdef USE_PYTHON_VIEWER

#include <Python.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mujoco_ros2_control
{

/**
 * @brief Wrapper for embedded Python MuJoCo viewer with full UI
 *
 * This class embeds a Python interpreter and launches mujoco.viewer
 * with direct access to the C++ mjModel* and mjData* pointers.
 * This provides the full simulate binary UI while keeping everything
 * in the same process with zero-copy synchronization.
 */
class PythonViewer
{
public:
  /**
   * @brief Initialize the embedded Python viewer
   * @param model Pointer to MuJoCo model (owned by caller)
   * @param data Pointer to MuJoCo data (owned by caller)
   * @param logger ROS2 logger for status messages
   */
  bool init(mjModel* model, mjData* data, rclcpp::Logger logger);

  /**
   * @brief Update the viewer (sync with current data state)
   * Should be called once per frame
   * @return true if viewer is still running, false if closed
   */
  bool update();

  /**
   * @brief Check if viewer window should close
   * @return true if user closed the window
   */
  bool should_close();

  /**
   * @brief Cleanup and shutdown Python interpreter
   */
  void close();

  /**
   * @brief Destructor - ensures Python is cleaned up
   */
  ~PythonViewer();

private:
  mjModel* mj_model_ = nullptr;
  mjData* mj_data_ = nullptr;
  rclcpp::Logger logger_ = rclcpp::get_logger("python_viewer");

  PyObject* viewer_module_ = nullptr;
  PyObject* viewer_instance_ = nullptr;
  bool initialized_ = false;
  bool python_initialized_ = false;
};

}  // namespace mujoco_ros2_control

#endif  // USE_PYTHON_VIEWER

#endif  // MUJOCO_ROS2_CONTROL__PYTHON_VIEWER_HPP_

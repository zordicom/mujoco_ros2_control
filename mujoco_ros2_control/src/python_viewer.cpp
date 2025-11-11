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

#ifdef USE_PYTHON_VIEWER

#include "mujoco_ros2_control/python_viewer.hpp"
#include <sstream>

namespace mujoco_ros2_control
{

bool PythonViewer::init(mjModel* model, mjData* data, rclcpp::Logger logger)
{
  mj_model_ = model;
  mj_data_ = data;
  logger_ = logger;

  RCLCPP_INFO(logger_, "Initializing embedded Python MuJoCo viewer...");

  // Initialize Python interpreter if not already done
  if (!Py_IsInitialized())
  {
    Py_Initialize();
    python_initialized_ = true;
    RCLCPP_INFO(logger_, "Python interpreter initialized");
  }

  // Check for Python errors
  if (PyErr_Occurred())
  {
    PyErr_Print();
    RCLCPP_ERROR(logger_, "Python error during initialization");
    return false;
  }

  // Import necessary Python modules
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import mujoco");
  PyRun_SimpleString("import mujoco.viewer");

  if (PyErr_Occurred())
  {
    PyErr_Print();
    RCLCPP_ERROR(logger_, "Failed to import mujoco Python modules");
    return false;
  }

  // Pass C++ pointers as integers to Python and wrap them
  // The MuJoCo Python API supports wrapping existing C memory via integer addresses
  std::ostringstream pointer_code;
  pointer_code << "# Pass C++ pointer addresses as integers\n"
               << "model_addr = " << reinterpret_cast<uintptr_t>(mj_model_) << "\n"
               << "data_addr = " << reinterpret_cast<uintptr_t>(mj_data_) << "\n";

  RCLCPP_INFO(logger_, "Passing pointers: model=%p, data=%p", (void*)mj_model_, (void*)mj_data_);

  if (PyRun_SimpleString(pointer_code.str().c_str()) != 0)
  {
    PyErr_Print();
    RCLCPP_ERROR(logger_, "Failed to pass pointer addresses to Python");
    return false;
  }

  // Now try to wrap these pointers using MuJoCo's Python API
  // Try multiple methods as the API may vary by version
  const char* wrap_code =
    "try:\n"
    "    # Method 1: Try from_int_ptr (MuJoCo 3.x)\n"
    "    from mujoco._structs import MjModel, MjData\n"
    "    model = MjModel.from_int_ptr(model_addr)\n"
    "    data = MjData.from_int_ptr(data_addr)\n"
    "    print('✓ Successfully wrapped pointers using from_int_ptr()')\n"
    "except (AttributeError, TypeError) as e:\n"
    "    print('Method 1 failed: ' + str(e))\n"
    "    try:\n"
    "        # Method 2: Try direct constructor with integer\n"
    "        import mujoco\n"
    "        model = mujoco.MjModel(model_addr)\n"
    "        data = mujoco.MjData(data_addr)\n"
    "        print('✓ Successfully wrapped pointers using direct constructor')\n"
    "    except Exception as e2:\n"
    "        print('Method 2 failed: ' + str(e2))\n"
    "        # Method 3: Try ctypes casting\n"
    "        import ctypes\n"
    "        import mujoco._structs\n"
    "        # Cast integer to ctypes pointer, then to MuJoCo wrapper\n"
    "        model_ptr = ctypes.cast(model_addr, ctypes.POINTER(ctypes.c_void_p))\n"
    "        data_ptr = ctypes.cast(data_addr, ctypes.POINTER(ctypes.c_void_p))\n"
    "        print('✓ Created ctypes pointers (fallback mode)')\n";

  if (PyRun_SimpleString(wrap_code) != 0)
  {
    PyErr_Print();
    RCLCPP_ERROR(logger_, "All methods to wrap C++ pointers failed");
    return false;
  }

  // Check which method succeeded by checking Python variables
  PyObject* main_module = PyImport_AddModule("__main__");
  PyObject* main_dict = PyModule_GetDict(main_module);
  PyObject* model_obj = PyDict_GetItemString(main_dict, "model");
  PyObject* data_obj = PyDict_GetItemString(main_dict, "data");

  if (!model_obj || !data_obj)
  {
    RCLCPP_ERROR(logger_, "Failed to create model or data Python objects");
    return false;
  }

  RCLCPP_INFO(logger_, "Created Python MuJoCo objects from C++ pointers");
  RCLCPP_INFO(logger_, "  mjModel*: %p", (void*)mj_model_);
  RCLCPP_INFO(logger_, "  mjData*:  %p", (void*)mj_data_);

  // Launch the passive viewer with error handling
  const char* launch_code =
    "try:\n"
    "    viewer_handle = mujoco.viewer.launch_passive(\n"
    "        model, data,\n"
    "        show_left_ui=True,\n"
    "        show_right_ui=True\n"
    "    )\n"
    "    launch_success = True\n"
    "    launch_error = None\n"
    "except Exception as e:\n"
    "    import traceback\n"
    "    launch_success = False\n"
    "    viewer_handle = None\n"
    "    launch_error = str(e) + '\\n' + traceback.format_exc()\n";

  if (PyRun_SimpleString(launch_code) != 0)
  {
    PyErr_Print();
    RCLCPP_ERROR(logger_, "Failed to execute Python launch code");
    return false;
  }

  // Check if launch was successful and extract error if failed
  PyObject* success_obj = PyDict_GetItemString(main_dict, "launch_success");

  if (!success_obj || !PyObject_IsTrue(success_obj))
  {
    // Try to get the error message from Python
    PyObject* error_code =
      PyRun_String("str(launch_error) if 'launch_error' in dir() else 'Unknown error'",
                   Py_eval_input, main_dict, main_dict);

    if (error_code && PyUnicode_Check(error_code))
    {
      const char* error_msg = PyUnicode_AsUTF8(error_code);
      RCLCPP_ERROR(logger_, "Python viewer launch failed: %s", error_msg);
      Py_DECREF(error_code);
    }
    else
    {
      RCLCPP_ERROR(logger_, "Python viewer launch failed (no error details available)");
    }
    return false;
  }

  RCLCPP_INFO(logger_, "Python MuJoCo viewer launched successfully!");
  RCLCPP_INFO(logger_, "Full UI enabled - use Ctrl+Right-click to apply forces");
  RCLCPP_INFO(logger_, "Forces applied in viewer directly affect the C++ simulation!");

  initialized_ = true;
  return true;
}

bool PythonViewer::update()
{
  if (!initialized_)
  {
    return false;
  }

  // Sync the viewer with current data state
  // The viewer will render the current state of mjData
  const char* sync_code =
    "if viewer_handle and viewer_handle.is_running():\n"
    "    viewer_handle.sync()\n";

  if (PyRun_SimpleString(sync_code) != 0)
  {
    if (PyErr_Occurred())
    {
      PyErr_Print();
    }
    return false;
  }

  return true;
}

bool PythonViewer::should_close()
{
  if (!initialized_)
  {
    return true;
  }

  // Check if viewer window is still running
  PyObject* main_module = PyImport_AddModule("__main__");
  if (!main_module)
  {
    return true;
  }

  PyObject* main_dict = PyModule_GetDict(main_module);
  PyObject* viewer_handle = PyDict_GetItemString(main_dict, "viewer_handle");

  if (!viewer_handle)
  {
    return true;
  }

  // Call viewer_handle.is_running()
  PyObject* is_running = PyObject_CallMethod(viewer_handle, "is_running", NULL);
  if (!is_running)
  {
    if (PyErr_Occurred())
    {
      PyErr_Clear();
    }
    return true;
  }

  bool running = PyObject_IsTrue(is_running);
  Py_DECREF(is_running);

  return !running;
}

void PythonViewer::close()
{
  if (initialized_)
  {
    RCLCPP_INFO(logger_, "Closing Python MuJoCo viewer...");

    // Close the viewer
    const char* close_code =
      "if 'viewer_handle' in dir():\n"
      "    if viewer_handle:\n"
      "        viewer_handle.close()\n";

    PyRun_SimpleString(close_code);

    initialized_ = false;
  }

  // Only finalize Python if we initialized it
  if (python_initialized_ && Py_IsInitialized())
  {
    Py_Finalize();
    python_initialized_ = false;
    RCLCPP_INFO(logger_, "Python interpreter finalized");
  }
}

PythonViewer::~PythonViewer()
{
  close();
}

}  // namespace mujoco_ros2_control

#endif  // USE_PYTHON_VIEWER

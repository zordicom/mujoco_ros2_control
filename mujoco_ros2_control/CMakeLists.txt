cmake_minimum_required(VERSION 3.10)
project(mujoco_ros2_control)

# Set policies to suppress warnings
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # Prefer GLVND OpenGL libraries
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  mujoco_ros2_control_msgs
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
  ament_index_cpp
  rosgraph_msgs
  std_msgs
  geometry_msgs
  rclcpp_lifecycle
  sensor_msgs
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_plugins ${MUJOCO_LIB})
target_include_directories(mujoco_system_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build rendering and cameras as libraries for viewer to use
add_library(mujoco_rendering SHARED src/mujoco_rendering.cpp)
ament_target_dependencies(mujoco_rendering ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_rendering ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_rendering
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

add_library(mujoco_cameras SHARED src/mujoco_cameras.cpp)
ament_target_dependencies(mujoco_cameras ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_cameras ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_cameras
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_rendering
  mujoco_cameras
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Viewer executable - DISABLED (not functional after refactor)
# The separate mujoco_viewer executable has a fundamental incompatibility
# between rclcpp::Node and mujoco_rendering that causes crashes.
# For visualization, use RViz with /joint_states topic.
# See: https://github.com/PickNikRobotics/mujoco_ros2_control/issues/XXX
#
# add_executable(mujoco_viewer src/mujoco_viewer_node.cpp)
# ament_target_dependencies(mujoco_viewer ${THIS_PACKAGE_DEPENDS})
# target_link_libraries(mujoco_viewer ${MUJOCO_LIB} mujoco_rendering glfw)
# target_include_directories(mujoco_viewer
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#     ${MUJOCO_INCLUDE_DIR}
#     ${EIGEN3_INCLUDE_DIR})
#
# install(TARGETS
#   mujoco_viewer
#   DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_plugins.xml)
ament_package()

cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME state_estimation_node_v2_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(param_manager_cpp REQUIRED)
find_package(ros_param_helpers_cpp REQUIRED)
find_package(ros_debug_helpers_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(state_estimation_node_cpp REQUIRED)
find_package(state_estimation_v2_cpp REQUIRED)
find_package(state_estimation_cpp REQUIRED)
find_package(state_estimation_base REQUIRED)
find_package(state_estimation_constants REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tum_type_conversions_ros_cpp REQUIRED)
find_package(ros2_watchdog_cpp REQUIRED)

add_executable(state_estimation_node_v2_cpp_ekf_3d_kin
  src/state_estimation_node_v2_ekf_3d_kin_main.cpp)

# state_estimation_node_cpp's own CMakeLists.txt installs its headers but never calls
# ament_export_include_directories/ament_export_targets, so find_package()+
# ament_target_dependencies() alone does not add its include dir (this is a pre-existing
# gap in the upstream package, not something introduced here) -- resolve it manually from
# where find_package() located that package's cmake config.
get_filename_component(state_estimation_node_cpp_PREFIX
  "${state_estimation_node_cpp_DIR}/../../.." ABSOLUTE)

target_include_directories(state_estimation_node_v2_cpp_ekf_3d_kin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${state_estimation_node_cpp_PREFIX}/include
)

# -O3, not -Ofast -- same rationale as state_estimation_v2_cpp/CMakeLists.txt.
target_compile_options(state_estimation_node_v2_cpp_ekf_3d_kin PUBLIC -O3)

ament_target_dependencies(state_estimation_node_v2_cpp_ekf_3d_kin
  rclcpp
  param_manager_cpp
  ros_param_helpers_cpp
  ros_debug_helpers_cpp
  tum_types_cpp
  tum_helpers_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  msgs
  std_msgs
  sensor_msgs
  diagnostic_msgs
  state_estimation_node_cpp
  state_estimation_v2_cpp
  state_estimation_cpp
  state_estimation_base
  state_estimation_constants
  tum_type_conversions_ros_cpp
  ros2_watchdog_cpp
)

install(TARGETS state_estimation_node_v2_cpp_ekf_3d_kin DESTINATION lib/${PROJECT_NAME})

ament_package()

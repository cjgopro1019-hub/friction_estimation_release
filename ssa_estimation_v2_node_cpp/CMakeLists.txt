cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME ssa_estimation_v2_node_cpp)
set(NODE_EXECUTABLE_NAME ssa_estimation_v2_node)
set(NODE_EXECUTABLE_PATH src/ssa_estimation_v2_node_main.cpp)
set(NODE_DEPENDENCIES
  rclcpp
  param_manager_cpp
  ros_param_helpers_cpp
  ros_debug_helpers_cpp
  tum_types_cpp
  tum_helpers_cpp
  msgs
  nav_msgs
  sensor_msgs
  ssa_estimation_constants
  ssa_estimation_node_cpp
  ssa_estimation_v2_cpp
  diagnostic_msgs
  tum_type_conversions_ros_cpp
  ros2_watchdog_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})

# ssa_estimation_node_cpp (the ORIGINAL, unforked, generic SSAEstimationNode<TConfig> template
# this v2 node reuses) only ever built an executable, never an exported library target with
# ament_export_include_directories -- so find_package() alone doesn't add its install/include
# to our path the way it does for a proper INTERFACE-library dependency. Add it manually.
get_filename_component(SSA_ESTIMATION_NODE_CPP_PREFIX
  "${ssa_estimation_node_cpp_DIR}/../../.." ABSOLUTE)

target_include_directories(${NODE_EXECUTABLE_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        "${SSA_ESTIMATION_NODE_CPP_PREFIX}/include"
)
# -O3 (not -Ofast): -Ofast implies -ffast-math, which permits the compiler to assume no
# NaN/Inf occur and to reorder floating-point operations. The UKF's sigma-point weights
# (P_VDC_alpha near a degenerate scaling) rely on near-exact cancellation of ~1e6-magnitude
# terms; under -ffast-math this cancellation's outcome (finite vs NaN) became
# non-deterministic across otherwise-identical rebuilds -- see
# docs/TODO_Advisor_Action_Items.md. -O3 keeps the optimization level without that hazard.
target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -O3)
ament_target_dependencies(${NODE_EXECUTABLE_NAME} ${NODE_DEPENDENCIES})
ament_export_dependencies(${NODE_DEPENDENCIES})

install(
  TARGETS ${NODE_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY include/
    DESTINATION include
  )
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

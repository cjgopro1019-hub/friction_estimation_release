cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME friction_estimation_cpp)
set(NODE_EXECUTABLE_NAME friction_estimation_node)
set(NODE_DEPENDENCIES
  rclcpp
  friction_estimation_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  msgs
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

add_executable(${NODE_EXECUTABLE_NAME}
  src/friction_estimation_node.cpp
  src/friction_estimation_node_main.cpp
)

target_include_directories(${NODE_EXECUTABLE_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${NODE_EXECUTABLE_NAME} PUBLIC cxx_std_17)
target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -Ofast)
ament_target_dependencies(${NODE_EXECUTABLE_NAME} ${NODE_DEPENDENCIES})

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
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

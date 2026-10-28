cmake_minimum_required(VERSION 3.12)
project(friction_estimation_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrictionEstimate.msg"
  DEPENDENCIES std_msgs
)

ament_package()

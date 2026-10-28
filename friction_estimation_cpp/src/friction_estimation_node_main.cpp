// Copyright 2026
#include <rclcpp/rclcpp.hpp>

#include "friction_estimation_cpp/friction_estimation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<tam::core::friction::FrictionEstimationNode>(options);
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026
// v2 node executable: reuses the ORIGINAL, unmodified, generic stateEstimationNode<TConfig>
// template from state_estimation_node_cpp (it takes the concrete StateEstimationBase
// implementation as a constructor argument, so it never needed forking) -- the only thing
// this file does differently from the original kinematic 3D main.cpp is construct
// state_estimation_v2_cpp's StateEstimationV2 (with the outlier-rejection fix) instead of
// the original's StateEstimation.
//
// Same node name / topic names as the original (hardcoded inside stateEstimationNode's
// constructor) -- this is meant to REPLACE the v1 EKF in the launch file, not run
// alongside it (unlike the v1/v2 SSA-UKF split, which deliberately keeps both as two
// independent opinions). See friction_estimation_demo.launch.py.
#include <rclcpp/rclcpp.hpp>
#include "state_estimation_node_cpp/state_estimation_node_impl.hpp"
#include "state_estimation_v2_cpp/state_estimation.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<stateEstimationNode<tam::core::state::EKF_3D>>(
    std::make_unique<tam::core::state::StateEstimationV2<tam::core::state::EKF_3D>>("kinematic"),
    "kinematic", options);
  tam::ros::validate_param_overrides(argc, argv, node.get());
  rclcpp::executors::StaticSingleThreadedExecutor single_thread_executor;
  single_thread_executor.add_node(node);
  single_thread_executor.spin();
  rclcpp::shutdown();
  return 0;
}

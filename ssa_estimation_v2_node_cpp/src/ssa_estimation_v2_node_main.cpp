// Copyright 2026
// v2 node executable: reuses the ORIGINAL, unmodified, generic SSAEstimationNode<TConfig>
// template from ssa_estimation_node_cpp (it takes the concrete SSAEstimationBase
// implementation as a constructor argument, so it never needed forking) -- the only thing
// this file does differently from the original ssa_estimation_node_main.cpp is construct
// ssa_estimation_v2_cpp's SSAEstimation (with the v2 UKF) instead of the original's.
//
// The node name and topic names (/core/ssa/odometry, /core/ssa/status) are hardcoded inside
// SSAEstimationNode's constructor and are NOT changed here -- to run this side-by-side with
// the original v1 SSA node without collision, remap them at launch time, e.g.:
//   ros2 run ssa_estimation_v2_node_cpp ssa_estimation_v2_node --ros-args \
//     -r __node:=SSAEstimationV2 \
//     -r /core/ssa/odometry:=/core/ssa_v2/odometry \
//     -r /core/ssa/status:=/core/ssa_v2/status
// (see the v2 comparison launch file, which does exactly this).
#include <rclcpp/rclcpp.hpp>
#include "ssa_estimation_node_cpp/ssa_estimation_node_impl.hpp"
#include "ssa_estimation_v2_cpp/ssa_estimation.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SSAEstimationNode<tam::core::ssa::UKF_STM>>(
    std::make_unique<tam::core::ssa::SSAEstimationV2<tam::core::ssa::UKF_STM>>(), options);
  tam::ros::validate_param_overrides(argc, argv, node.get());
  rclcpp::executors::StaticSingleThreadedExecutor single_thread_executor;
  single_thread_executor.add_node(node);
  single_thread_executor.spin();
  rclcpp::shutdown();
  return 0;
}

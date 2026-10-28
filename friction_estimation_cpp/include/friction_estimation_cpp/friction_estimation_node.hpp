// Copyright 2026
#pragma once

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <msgs/msg/steering_report.hpp>
#include <msgs/msg/tum_float64_per_wheel_stamped.hpp>
#include <msgs/msg/tum_debug_values.hpp>
#include <msgs/msg/tum_debug_signal_names.hpp>

#include "friction_estimation_msgs/msg/friction_estimate.hpp"
#include "friction_estimation_cpp/tier1_friction_demand.hpp"
#include "friction_estimation_cpp/input_health.hpp"
#include "friction_estimation_cpp/wear_model.hpp"
#include "friction_estimation_cpp/friction_demand_estimator.hpp"
#include "friction_estimation_cpp/excitation_evaluator.hpp"
#include "friction_estimation_cpp/friction_potential_tracker.hpp"
#include "friction_estimation_cpp/data_quality_gating.hpp"
#include "friction_estimation_cpp/slip_slope_observer.hpp"

namespace tam::core::friction
{

class FrictionEstimationNode : public rclcpp::Node
{
public:
  explicit FrictionEstimationNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 10ms model step: friction demand + excitation-aware potential tracking
   */
  void model_update_callback();

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void steering_callback(const msgs::msg::SteeringReport::SharedPtr msg);
  void wheelspeed_callback(const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void brake_pressure_callback(const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void v2_debug_names_callback(const msgs::msg::TUMDebugSignalNames::SharedPtr msg);
  void v2_debug_values_callback(const msgs::msg::TUMDebugValues::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr model_update_timer_;

  rclcpp::Publisher<friction_estimation_msgs::msg::FrictionEstimate>::SharedPtr
    pub_friction_estimate_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<msgs::msg::SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_wheelspeed_;
  rclcpp::Subscription<msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_brake_pressure_;
  rclcpp::Subscription<msgs::msg::TUMDebugSignalNames>::SharedPtr sub_v2_debug_names_;
  rclcpp::Subscription<msgs::msg::TUMDebugValues>::SharedPtr sub_v2_debug_values_;

  // vehicle constants, reused from ssa_estimation_ukf_config.yml
  Tier1VehicleParams tier1_params_;
  double l_front_m_{1.724};
  double l_rear_m_{1.247};

  // latest values from the 3D-EKF / raw vehicle sensors, zero-order-held between
  // callbacks like the existing SSA-UKF node does. Deliberately no beta/side-slip
  // signal anywhere in this node -- see design notes on why this module family never
  // depends on it.
  double v_mps_{0.0};
  double a_lat_mps2_{0.0};
  double a_long_mps2_{0.0};
  double r_radps_{0.0};          // raw yaw rate, /vehicle/sensor/imu1 angular_velocity.z
  double delta_rad_{0.0};        // /vehicle/sensor/steering_report steering_tire_angle
  WheelSpeeds wheelspeed_radps_;
  double brake_pressure_fl_pa_{0.0};
  double brake_pressure_fr_pa_{0.0};
  double brake_pressure_rl_pa_{0.0};
  double brake_pressure_rr_pa_{0.0};

  FrictionDemandEstimator friction_demand_estimator_;
  ExcitationEvaluator excitation_evaluator_;
  FrictionPotentialTracker friction_potential_tracker_;
  DataQualityGating data_quality_gating_;
  WearModel wear_model_;

  // Online, self-calibrating slip-slope observers -- one per axle, no assumption baked
  // in about which axle is "the driven one". See slip_slope_observer.hpp for why: this
  // vehicle is RWD-biased (research/slip_slope_analysis.py), but the next dataset
  // expected is a structurally different AWD sedan. Diagnostic-only, not yet wired
  // into excitation_score/potential_confidence.
  SlipSlopeObserver slip_slope_observer_front_;
  SlipSlopeObserver slip_slope_observer_rear_;
  // Last result from each observer, held across cycles where gate.valid is false
  // (step() isn't called then) -- a brief exclusion must not make the published
  // confidence flicker down to a blank/zero result when the underlying learned
  // relationship hasn't actually been invalidated.
  SlipSlopeResult last_slip_slope_front_;
  SlipSlopeResult last_slip_slope_rear_;

  rclcpp::Time last_model_update_time_;
  bool has_last_model_update_time_{false};

  // state carried one cycle for ExcitationEvaluator -- see its class doc for why both
  // are deliberately one-cycle-lagged rather than computed fresh each call
  double last_r_radps_{0.0};
  double last_mu_potential_{0.5};  // overwritten with FrictionPotentialTrackerParams::init
                                    // once that struct is built in the constructor

  // v2 axle-force-evidence demand -- read from the v2 SSA-UKF's own debug stream
  // (name-indexed, matching the existing debug-output convention rather than adding a
  // hard dependency on ssa_estimation_v2_cpp's internal types). Deliberately NOT part
  // of the six-input start/stop gate above: v2 is an optional, parallel-runnable node,
  // and its absence must never stop mu_demand (the always-available EKF-based estimate)
  // from being published.
  std::map<std::string, size_t> v2_debug_name_to_idx_;
  double mu_axle_evidence_{0.0};
  bool have_v2_debug_names_{false};
  bool have_v2_debug_values_{false};
  double axle_evidence_staleness_timeout_s_{0.5};
  rclcpp::Time last_v2_debug_time_;

  // input health: staleness + plausibility (Phase 3)
  // Bounds are generous physical limits (not tuned confidence intervals) -- their job is
  // to catch sensor glitches like the ~-643deg/s Kistler spike found in vegas24, not to
  // second-guess normal driving.
  double staleness_timeout_s_{0.1};
  StalenessTracker odometry_staleness_;
  StalenessTracker acceleration_staleness_;
  StalenessTracker imu_staleness_;
  StalenessTracker steering_staleness_;
  StalenessTracker wheelspeed_staleness_;
  StalenessTracker brake_pressure_staleness_;

  // Start/stop gating on the six inputs above: no computation runs before every one of
  // them has published at least once (avoids computing on default-initialized
  // placeholder zeros before real data arrives), and the model_update_timer_ is
  // cancelled the moment every one of them has simultaneously gone stale (avoids
  // continuing to compute on frozen zero-order-held state once the data source --
  // bag or live vehicle -- has stopped, mirroring the analogous fix already made in
  // state_estimation_node for its own IMU inputs). Requiring ALL six (not just one) on
  // both ends is deliberate: a brief single-topic staleness blip must not start/stop
  // the whole node.
  bool has_started_{false};
  PlausibilityGuard v_guard_{/*max_abs_value=*/100.0, /*max_delta_per_cycle=*/5.0};
  PlausibilityGuard a_long_guard_{/*max_abs_value=*/50.0, /*max_delta_per_cycle=*/30.0};
  PlausibilityGuard a_lat_guard_{/*max_abs_value=*/50.0, /*max_delta_per_cycle=*/30.0};
  // set by the callbacks to the outcome of their most recent sample; read (not reset) by
  // the timer, so a rejection stays visible until the next message resolves it either way
  bool v_rejected_{false};
  bool a_long_rejected_{false};
  bool a_lat_rejected_{false};

  // input_health bit assignment
  static constexpr uint32_t kHealthOdometryStale = 1u << 0;
  static constexpr uint32_t kHealthAccelerationStale = 1u << 1;
  static constexpr uint32_t kHealthVelocityRejected = 1u << 2;
  static constexpr uint32_t kHealthALongRejected = 1u << 3;
  static constexpr uint32_t kHealthALatRejected = 1u << 4;
  static constexpr uint32_t kHealthImuStale = 1u << 7;
  static constexpr uint32_t kHealthSteeringStale = 1u << 8;
  static constexpr uint32_t kHealthWheelspeedStale = 1u << 9;
  static constexpr uint32_t kHealthBrakePressureStale = 1u << 10;
};

}  // namespace tam::core::friction

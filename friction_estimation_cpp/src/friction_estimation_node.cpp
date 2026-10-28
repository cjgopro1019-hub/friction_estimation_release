// Copyright 2026
#include "friction_estimation_cpp/friction_estimation_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace tam::core::friction
{

FrictionEstimationNode::FrictionEstimationNode(const rclcpp::NodeOptions & options)
: Node("FrictionEstimation", "/core/friction", options)
{
  // vehicle constants -- values copied from ssa_estimation_ukf_config.yml on purpose,
  // manual sync accepted (see design notes)
  tier1_params_.mass_kg = this->declare_parameter<double>("P_VDC_mass_kg", 800.00);
  tier1_params_.rho_air_kgpm3 = this->declare_parameter<double>("P_VDC_rho_air_kgpm3", 1.22);
  tier1_params_.cL_front = this->declare_parameter<double>("P_VDC_cL_front", -0.7);
  tier1_params_.cL_rear = this->declare_parameter<double>("P_VDC_cL_rear", -0.7);
  friction_demand_estimator_.set_params(tier1_params_);

  l_front_m_ = this->declare_parameter<double>("P_VDC_l_front_m", l_front_m_);
  l_rear_m_ = this->declare_parameter<double>("P_VDC_l_rear_m", l_rear_m_);

  staleness_timeout_s_ = this->declare_parameter<double>(
    "P_VDC_input_staleness_timeout_s", staleness_timeout_s_);

  {
    DataQualityGatingParams p;
    p.wheel_lockout_threshold = this->declare_parameter<double>(
      "P_VDC_wheel_lockout_threshold", p.wheel_lockout_threshold);
    p.track_width_front_m = this->declare_parameter<double>(
      "P_VDC_track_width_front_m", p.track_width_front_m);
    p.track_width_rear_m = this->declare_parameter<double>(
      "P_VDC_track_width_rear_m", p.track_width_rear_m);
    p.tire_radius_front_m = this->declare_parameter<double>(
      "P_VDC_tire_radius_front_m", p.tire_radius_front_m);
    p.tire_radius_rear_m = this->declare_parameter<double>(
      "P_VDC_tire_radius_rear_m", p.tire_radius_rear_m);
    p.brake_pressure_rate_threshold_pa_per_s = this->declare_parameter<double>(
      "P_VDC_abs_brake_pressure_rate_threshold_pa_per_s",
      p.brake_pressure_rate_threshold_pa_per_s);
    p.abs_oscillation_window_samples = this->declare_parameter<int>(
      "P_VDC_abs_oscillation_window_samples", p.abs_oscillation_window_samples);
    p.abs_oscillation_min_sign_changes = this->declare_parameter<int>(
      "P_VDC_abs_oscillation_min_sign_changes", p.abs_oscillation_min_sign_changes);
    data_quality_gating_ = DataQualityGating(p);
  }

  {
    ExcitationEvaluatorParams p;
    p.l_front_m = l_front_m_;
    p.l_rear_m = l_rear_m_;
    p.mass_kg = tier1_params_.mass_kg;
    p.izz_kgm2 = this->declare_parameter<double>("P_VDC_Izz_kgm2", p.izz_kgm2);

    auto declare_mf = [this](const std::string & name, const MfCoeffs & def) {
      const std::vector<double> default_vec(def.begin(), def.end());
      const auto v = this->declare_parameter<std::vector<double>>(name, default_vec);
      return MfCoeffs{v[0], v[1], v[2], v[3]};
    };
    p.mf_long_front = declare_mf("P_VDC_tire_MF_long_front", p.mf_long_front);
    p.mf_long_rear = declare_mf("P_VDC_tire_MF_long_rear", p.mf_long_rear);
    p.mf_lat_front = declare_mf("P_VDC_tire_MF_lat_front", p.mf_lat_front);
    p.mf_lat_rear = declare_mf("P_VDC_tire_MF_lat_rear", p.mf_lat_rear);
    excitation_evaluator_ = ExcitationEvaluator(p);
  }

  {
    FrictionPotentialTrackerParams p;
    p.excitation_threshold = this->declare_parameter<double>(
      "P_VDC_potential_excitation_threshold", p.excitation_threshold);
    p.decay_rate_per_s = this->declare_parameter<double>(
      "P_VDC_potential_decay_rate_per_s", p.decay_rate_per_s);
    p.confidence_decay_tau_s = this->declare_parameter<double>(
      "P_VDC_potential_confidence_decay_tau_s", p.confidence_decay_tau_s);
    p.init = this->declare_parameter<double>("P_VDC_potential_init", p.init);
    p.min = this->declare_parameter<double>("P_VDC_potential_min", p.min);
    p.max = this->declare_parameter<double>("P_VDC_potential_max", p.max);
    p.corroboration_weight = this->declare_parameter<double>(
      "P_VDC_potential_corroboration_weight", p.corroboration_weight);
    friction_potential_tracker_ = FrictionPotentialTracker(p);
    // ExcitationEvaluator's demand_util_component reads last cycle's mu_potential --
    // seed it with the same prior the tracker itself starts from, not 0 (which would
    // read as "zero force utilization" for the first cycle instead of "unknown").
    last_mu_potential_ = p.init;
  }

  wear_model_ = WearModel(
    this->declare_parameter<double>("P_VDC_wear_energy_coefficient", 0.0));

  {
    SlipSlopeObserverParams p;
    p.forgetting_factor = this->declare_parameter<double>(
      "P_VDC_slip_slope_forgetting_factor", p.forgetting_factor);
    p.min_samples_for_confidence = this->declare_parameter<double>(
      "P_VDC_slip_slope_min_samples_for_confidence", p.min_samples_for_confidence);
    p.min_kappa_coverage = this->declare_parameter<double>(
      "P_VDC_slip_slope_min_kappa_coverage", p.min_kappa_coverage);
    p.consistency_tolerance = this->declare_parameter<double>(
      "P_VDC_slip_slope_consistency_tolerance", p.consistency_tolerance);
    slip_slope_observer_front_ = SlipSlopeObserver(p);
    slip_slope_observer_rear_ = SlipSlopeObserver(p);
  }

  pub_friction_estimate_ = this->create_publisher<friction_estimation_msgs::msg::FrictionEstimate>(
    "/core/friction/estimate", 1);

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/core/state/odometry", 1,
    std::bind(&FrictionEstimationNode::odometry_callback, this, std::placeholders::_1));

  sub_acceleration_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/core/state/acceleration", 1,
    std::bind(&FrictionEstimationNode::acceleration_callback, this, std::placeholders::_1));

  // Raw vehicle sensors for excitation scoring and data-quality gating -- deliberately
  // vy-independent, so these are read straight from the vehicle sensors, not fused
  // EKF/SSA state, and there is no beta/side-slip subscription anywhere in this node.
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/vehicle/sensor/imu1", 200,
    std::bind(&FrictionEstimationNode::imu_callback, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<msgs::msg::SteeringReport>(
    "/vehicle/sensor/steering_report", 200,
    std::bind(&FrictionEstimationNode::steering_callback, this, std::placeholders::_1));

  sub_wheelspeed_ = this->create_subscription<msgs::msg::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/wheelspeed_radps", 200,
    std::bind(&FrictionEstimationNode::wheelspeed_callback, this, std::placeholders::_1));

  sub_brake_pressure_ = this->create_subscription<msgs::msg::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/brake_pressure_Pa", 200,
    std::bind(&FrictionEstimationNode::brake_pressure_callback, this, std::placeholders::_1));

  axle_evidence_staleness_timeout_s_ = this->declare_parameter<double>(
    "P_VDC_axle_evidence_staleness_timeout_s", axle_evidence_staleness_timeout_s_);

  // v2 SSA-UKF debug stream -- optional second demand estimate, see friction_estimate_node.hpp
  // comment above have_v2_debug_names_. Node/topic names match ssa_estimation_v2_node_cpp's
  // hardcoded remap convention (see its main.cpp): node renamed to SSAEstimationV2, own
  // debug namespace under /debug/core/ssa/SSAEstimationV2/... independent of v1's.
  sub_v2_debug_names_ = this->create_subscription<msgs::msg::TUMDebugSignalNames>(
    "/debug/core/ssa/SSAEstimationV2/kalman_filter/signal_names", 10,
    std::bind(&FrictionEstimationNode::v2_debug_names_callback, this, std::placeholders::_1));
  sub_v2_debug_values_ = this->create_subscription<msgs::msg::TUMDebugValues>(
    "/debug/core/ssa/SSAEstimationV2/kalman_filter/values", 200,
    std::bind(&FrictionEstimationNode::v2_debug_values_callback, this, std::placeholders::_1));

  model_update_timer_ = this->create_wall_timer(
    10ms, std::bind(&FrictionEstimationNode::model_update_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Friction Estimation Node initialized (excitation-aware demand/potential architecture)");
}

void FrictionEstimationNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_staleness_.touch(this->now());
  v_rejected_ = !v_guard_.check(msg->twist.twist.linear.x);
  if (!v_rejected_) {
    v_mps_ = msg->twist.twist.linear.x;
  }
  // rejected samples are dropped: v_mps_ keeps its last accepted value (zero-order hold)
}

void FrictionEstimationNode::acceleration_callback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  acceleration_staleness_.touch(this->now());
  a_long_rejected_ = !a_long_guard_.check(msg->accel.accel.linear.x);
  if (!a_long_rejected_) {
    a_long_mps2_ = msg->accel.accel.linear.x;
  }
  a_lat_rejected_ = !a_lat_guard_.check(msg->accel.accel.linear.y);
  if (!a_lat_rejected_) {
    a_lat_mps2_ = msg->accel.accel.linear.y;
  }
}

void FrictionEstimationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_staleness_.touch(this->now());
  r_radps_ = msg->angular_velocity.z;
}

void FrictionEstimationNode::steering_callback(const msgs::msg::SteeringReport::SharedPtr msg)
{
  steering_staleness_.touch(this->now());
  delta_rad_ = static_cast<double>(msg->steering_tire_angle);
}

void FrictionEstimationNode::wheelspeed_callback(
  const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  wheelspeed_staleness_.touch(this->now());
  wheelspeed_radps_.fl = msg->data.front_left;
  wheelspeed_radps_.fr = msg->data.front_right;
  wheelspeed_radps_.rl = msg->data.rear_left;
  wheelspeed_radps_.rr = msg->data.rear_right;
}

void FrictionEstimationNode::brake_pressure_callback(
  const msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  brake_pressure_staleness_.touch(this->now());
  brake_pressure_fl_pa_ = msg->data.front_left;
  brake_pressure_fr_pa_ = msg->data.front_right;
  brake_pressure_rl_pa_ = msg->data.rear_left;
  brake_pressure_rr_pa_ = msg->data.rear_right;
}

void FrictionEstimationNode::v2_debug_names_callback(
  const msgs::msg::TUMDebugSignalNames::SharedPtr msg)
{
  v2_debug_name_to_idx_.clear();
  for (size_t i = 0; i < msg->names.size(); ++i) {
    v2_debug_name_to_idx_[msg->names[i]] = i;
  }
  have_v2_debug_names_ = true;
}

void FrictionEstimationNode::v2_debug_values_callback(
  const msgs::msg::TUMDebugValues::SharedPtr msg)
{
  if (!have_v2_debug_names_) {
    return;
  }
  // z_virt_2..5 = VIRTMEAS_FTX_F_N/FTX_R_N/FTY_F_N/FTY_R_N (ssa_estimation_constants/
  // UKF_STM.hpp), load_tire_N_0..3 = per-wheel dynamic tire loads -- same lookup this
  // session's compare_accel_ref.py used to validate mu_axle_evidence against Kistler.
  try {
    const auto & idx = v2_debug_name_to_idx_;
    const double ftx_f = msg->values.at(idx.at("z_virt_2"));
    const double ftx_r = msg->values.at(idx.at("z_virt_3"));
    const double fty_f = msg->values.at(idx.at("z_virt_4"));
    const double fty_r = msg->values.at(idx.at("z_virt_5"));
    const double load_front =
      msg->values.at(idx.at("load_tire_N_0")) + msg->values.at(idx.at("load_tire_N_1"));
    const double load_rear =
      msg->values.at(idx.at("load_tire_N_2")) + msg->values.at(idx.at("load_tire_N_3"));
    const double load_total = load_front + load_rear;
    const double fx_total = ftx_f + ftx_r;
    const double fy_total = fty_f + fty_r;
    const double candidate = (load_total > 1.0)
      ? std::sqrt(fx_total * fx_total + fy_total * fy_total) / load_total : 0.0;
    // v2's SSA-UKF has no watchdog of its own (see the comment on axle_evidence_fresh
    // below): a NaN/Inf payload can arrive right on schedule and pass the freshness
    // check silently. Reject it here instead of publishing it as "valid" -- e.g. NaN
    // load_total makes the ternary above take its false branch (NaN > 1.0 is false),
    // silently reporting a near-zero demand instead of "not available".
    if (!std::isfinite(candidate)) {
      return;
    }
    mu_axle_evidence_ = candidate;
    have_v2_debug_values_ = true;
    last_v2_debug_time_ = this->now();
  } catch (const std::out_of_range &) {
    // v2 debug config hash/signal set doesn't match what we expect -- leave the last
    // good value in place; axle_evidence_valid will go false once it goes stale.
  }
}

void FrictionEstimationNode::model_update_callback()
{
  const rclcpp::Time now = this->now();

  // Each tracker's is_stale() is computed once here and reused everywhere below (the
  // six-input start/stop gate and axle_evidence_fresh) -- avoids recomputing the same
  // four values twice per 10ms cycle and the risk of the two call sites drifting apart.
  const bool odometry_stale = odometry_staleness_.is_stale(now, staleness_timeout_s_);
  const bool acceleration_stale = acceleration_staleness_.is_stale(now, staleness_timeout_s_);
  const bool imu_stale = imu_staleness_.is_stale(now, staleness_timeout_s_);
  const bool steering_stale = steering_staleness_.is_stale(now, staleness_timeout_s_);
  const bool wheelspeed_stale = wheelspeed_staleness_.is_stale(now, staleness_timeout_s_);
  const bool brake_pressure_stale = brake_pressure_staleness_.is_stale(now, staleness_timeout_s_);

  // --- start/stop gating: only compute while the data source (bag or live vehicle) is
  // actually live. See friction_estimation_node.hpp for why this requires ALL six
  // inputs, not just one, on both ends. ---
  const bool all_fresh =
    !odometry_stale && !acceleration_stale && !imu_stale &&
    !steering_stale && !wheelspeed_stale && !brake_pressure_stale;

  if (!has_started_) {
    if (!all_fresh) {
      // haven't seen a real message on every required input yet -- nothing to compute
      return;
    }
    has_started_ = true;
    RCLCPP_INFO(this->get_logger(), "All inputs are live -- friction estimation starting.");
  } else {
    const bool all_stale =
      odometry_stale && acceleration_stale && imu_stale &&
      steering_stale && wheelspeed_stale && brake_pressure_stale;
    if (all_stale) {
      RCLCPP_ERROR(
        this->get_logger(),
        "All inputs have gone stale simultaneously -- stopping friction estimation "
        "rather than continuing to compute on frozen zero-order-held state.");
      model_update_timer_->cancel();
      return;
    }
  }

  const double dt_s = has_last_model_update_time_
    ? (now - last_model_update_time_).seconds() : 0.01;
  last_model_update_time_ = now;
  has_last_model_update_time_ = true;

  // --- friction demand: always computed, vy-independent, never treated as the true limit ---
  const Tier1Output demand = friction_demand_estimator_.step(a_lat_mps2_, a_long_mps2_, v_mps_);

  const double l_total_m = l_front_m_ + l_rear_m_;
  const double fz_front_n = (l_total_m > 1e-6)
    ? demand.fz_total_n * (l_rear_m_ / l_total_m) : 0.0;
  const double fz_rear_n = demand.fz_total_n - fz_front_n;

  // --- data quality gating: excludes only genuinely-invalid samples (wheel lockup/spin,
  // suspected ABS/TC, sensor faults) -- never excludes a sample for being near the limit ---
  const bool inputs_finite_and_in_range =
    !v_rejected_ && !a_long_rejected_ && !a_lat_rejected_ &&
    std::isfinite(r_radps_) && std::isfinite(delta_rad_);
  const GateResult gate = data_quality_gating_.step(
    v_mps_, r_radps_, wheelspeed_radps_,
    brake_pressure_fl_pa_, brake_pressure_fr_pa_, brake_pressure_rl_pa_, brake_pressure_rr_pa_,
    inputs_finite_and_in_range, dt_s);

  // --- excitation scoring: rewards (not excludes) proximity to tire saturation on
  // either axle/axis -- these are exactly the near-limit conditions most informative
  // about the true friction potential. r_dot is a plain backward difference (no extra
  // filtering) -- see ExcitationEvaluator's class doc for why this stays vy/beta-free. ---
  const double r_dot_radps2 = (dt_s > 1e-6) ? (r_radps_ - last_r_radps_) / dt_s : 0.0;
  const ExcitationResult excitation = excitation_evaluator_.step(
    demand.mu_demand, last_mu_potential_, a_lat_mps2_, r_dot_radps2, fz_front_n, fz_rear_n,
    data_quality_gating_.last_kappa_front(), data_quality_gating_.last_kappa_rear());
  last_r_radps_ = r_radps_;

  // --- slip-slope observers: online, self-calibrating, one per axle. No assumption
  // baked in about which axle is driven -- see slip_slope_observer.hpp. Only updated
  // on data-quality-valid samples so wheel-lockup/ABS-corrupted readings can't corrupt
  // the online fit. Computed before FrictionPotentialTracker below so this cycle's
  // result can feed it as an independent corroborating signal. ---
  const double mu_x = (demand.fz_total_n > 1.0)
    ? (tier1_params_.mass_kg * a_long_mps2_) / demand.fz_total_n : 0.0;
  // vx>3m/s guard matches research/slip_slope_analysis.py's own filter -- below this,
  // wheelspeed-derived kappa is noise-dominated, and it also keeps a stationary/just-
  // started vehicle from padding the observer's sample count with degenerate
  // zero-speed (kappa=0) cycles before any real driving has happened.
  if (gate.valid && v_mps_ > 3.0) {
    last_slip_slope_front_ = slip_slope_observer_front_.step(
      data_quality_gating_.last_kappa_front(), mu_x);
    last_slip_slope_rear_ = slip_slope_observer_rear_.step(
      data_quality_gating_.last_kappa_rear(), mu_x);
  }

  // Corroboration fed to FrictionPotentialTracker: whichever axle currently both
  // trusts itself (confidence) AND agrees with this cycle's mu_demand (consistent) --
  // deliberately the max, not a fixed axle, so a structurally different vehicle (e.g.
  // AWD) isn't forced into this RWD car's front/rear pattern. Boost-only downstream --
  // never used to argue mu_potential's own excitation-gated update logic should
  // change, only how much to trust the resulting confidence number.
  const double slip_slope_corroboration = std::max(
    last_slip_slope_front_.consistent ? last_slip_slope_front_.confidence : 0.0,
    last_slip_slope_rear_.consistent ? last_slip_slope_rear_.confidence : 0.0);

  // --- friction potential: forgetting-factor tracker, only moves when excitation is
  // sufficient AND the sample is data-quality-valid; decays toward lower observed
  // demand rather than staying locked at a stale high-mu peak. Confidence gets a soft
  // upward nudge from slip_slope_corroboration above -- never a gate on whether
  // mu_potential itself updates, see friction_potential_tracker.hpp design notes. ---
  const FrictionPotentialResult potential = friction_potential_tracker_.step(
    demand.mu_demand, excitation.score, gate.valid, dt_s, slip_slope_corroboration);
  last_mu_potential_ = potential.mu_potential;

  // --- wear: always-on running integral, vy-independent slip-velocity proxy ---
  const double slip_velocity_mps = data_quality_gating_.last_max_abs_kappa() * v_mps_;
  wear_model_.step(tier1_params_.mass_kg *
    std::sqrt(a_lat_mps2_ * a_lat_mps2_ + a_long_mps2_ * a_long_mps2_),
    slip_velocity_mps, dt_s);
  const double eta_wear = wear_model_.eta_wear();
  const double mu_eff = potential.mu_potential * eta_wear;

  friction_estimation_msgs::msg::FrictionEstimate msg;
  msg.header.stamp = now;
  msg.mu_demand = demand.mu_demand;
  msg.mu_potential = potential.mu_potential;
  msg.mu_eff = mu_eff;
  msg.potential_confidence = potential.confidence;
  msg.excitation_score = excitation.score;
  msg.a_lat_mps2 = a_lat_mps2_;
  msg.a_long_mps2 = a_long_mps2_;
  msg.grip_limit_mps2 = mu_eff * kGravity_mps2;
  msg.eta_wear = eta_wear;
  msg.slip_energy_cumulative_j = wear_model_.cumulative_energy_j();
  msg.fz_front_n = fz_front_n;
  msg.fz_rear_n = fz_rear_n;
  msg.data_quality_valid = gate.valid;
  msg.wheel_lockup = gate.wheel_lockup;
  msg.abs_tc_active = gate.abs_tc_active;
  msg.drs_state = -1;

  msg.slip_slope_front = last_slip_slope_front_.slope;
  msg.slip_slope_front_confidence = last_slip_slope_front_.confidence;
  msg.slip_slope_front_consistent = last_slip_slope_front_.consistent;
  msg.slip_slope_rear = last_slip_slope_rear_.slope;
  msg.slip_slope_rear_confidence = last_slip_slope_rear_.confidence;
  msg.slip_slope_rear_consistent = last_slip_slope_rear_.consistent;

  // v2's SSA-UKF has no watchdog of its own -- it keeps predicting/updating on
  // whatever inputs it last received even once they've gone stale (confirmed by
  // reading ssa_estimation_v2_cpp's step(), which calls predict()/update()
  // unconditionally every cycle; the underlying state machine's STALE status is
  // never actually consulted to gate that). So "a v2 debug message arrived recently"
  // alone is NOT sufficient evidence the number in it is still meaningful -- it can
  // keep publishing right on schedule while quietly computing on frozen sensor data.
  // Cross-check against the same raw-sensor staleness trackers this node already
  // maintains for its own six-input gate (imu1/steering/wheelspeed/brake_pressure are
  // exactly the raw inputs v2's UKF also depends on) rather than trusting v2's own
  // message cadence alone.
  const bool axle_evidence_fresh = have_v2_debug_values_ &&
    (now - last_v2_debug_time_).seconds() <= axle_evidence_staleness_timeout_s_ &&
    !imu_staleness_.is_stale(now, staleness_timeout_s_) &&
    !steering_staleness_.is_stale(now, staleness_timeout_s_) &&
    !wheelspeed_staleness_.is_stale(now, staleness_timeout_s_) &&
    !brake_pressure_staleness_.is_stale(now, staleness_timeout_s_);
  msg.mu_demand_axle_evidence = axle_evidence_fresh ? mu_axle_evidence_ : 0.0;
  msg.axle_evidence_valid = axle_evidence_fresh;

  uint32_t health = 0;
  if (odometry_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthOdometryStale;
  }
  if (acceleration_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthAccelerationStale;
  }
  if (imu_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthImuStale;
  }
  if (steering_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthSteeringStale;
  }
  if (wheelspeed_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthWheelspeedStale;
  }
  if (brake_pressure_staleness_.is_stale(now, staleness_timeout_s_)) {
    health |= kHealthBrakePressureStale;
  }
  if (v_rejected_) {
    health |= kHealthVelocityRejected;
  }
  if (a_long_rejected_) {
    health |= kHealthALongRejected;
  }
  if (a_lat_rejected_) {
    health |= kHealthALatRejected;
  }
  msg.input_health = health;

  pub_friction_estimate_->publish(msg);
}

}  // namespace tam::core::friction

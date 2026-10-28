// Copyright 2026
#pragma once
namespace tam::core::ssa
{

template <class TConfig> UKFv2<TConfig>::UKFv2()
{
  x_.setZero();
  x_pred_.setZero();
  param_manager_ = std::make_shared<tam::core::ParamManager>();

  // vehicle parameters -- same names/defaults as the original ssa_estimation_cpp UKF, so
  // existing config ymls for this vehicle still apply unchanged to these
  param_manager_->declare_parameter(
    "P_VDC_mass_kg", double{800.00}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_Izz_kgm2", double{1000.00}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_l_front_m", double{1.724}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_l_rear_m", double{1.247}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tw_front_m", double{1.639}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tw_rear_m", double{1.524}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_rho_air_kgpm3", double{1.22}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_cL_front", double{-0.65}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_cL_rear", double{-0.8}, tam::types::param::ParameterType::DOUBLE, "");
  // declare brake parameters
  param_manager_->declare_parameter(
    "P_VDC_d_brake_bore_front_m", double{0.0798}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_d_brake_bore_rear_m", double{0.0798}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_r_brake_pads_lever_front_m", double{0.1493}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_r_brake_pads_lever_rear_m", double{0.1493}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_mue_brakes_front", double{0.55}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_mue_brakes_rear", double{0.55}, tam::types::param::ParameterType::DOUBLE, "");
  // declare tire parameters
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_long_front", std::vector<double>{20.0, 1.80, 1.65, 1.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_long_rear", std::vector<double>{20.0, 1.80, 1.70, 1.00},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_lat_front", std::vector<double>{22.0, 2.20, 1.60, 1.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_MF_lat_rear", std::vector<double>{20.0, 2.00, 1.65, 1.00},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");

  // v2 NEW: dynamic/effective tire radius (replaces the original's single fixed
  // P_VDC_rtire_front_m/rear_m constant). Unloaded-radius defaults match the original's fixed
  // radius values (no separate measurement exists); load-stiffness/speed-expansion defaults
  // are GPTireUKF's own, uncalibrated for this vehicle -- see vehicle_model.hpp.
  param_manager_->declare_parameter(
    "P_VDC_rtire_unloaded_front_m", double{0.2999}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_rtire_unloaded_rear_m", double{0.3120}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_load_stiffness_front_Npm", double{200000.0},
    tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_load_stiffness_rear_Npm", double{200000.0},
    tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_speed_expansion_front_mpradps2", double{2.3e-7},
    tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_tire_speed_expansion_rear_mpradps2", double{2.8e-7},
    tam::types::param::ParameterType::DOUBLE, "");

  // v2 NEW: longitudinal load transfer. UNCALIBRATED placeholder, see vehicle_model.hpp.
  param_manager_->declare_parameter(
    "P_VDC_z_cog_m", double{0.5}, tam::types::param::ParameterType::DOUBLE, "");

  // v2 NEW: aero drag + rolling resistance for the rear-axle force-balance evidence term.
  // UNCALIBRATED placeholders, see vehicle_model.hpp.

  // v2 NEW: wheel-speed-difference outlier guard for adaptR -- ported from GPTireUKF's finding
  // (front/rear L/R wheel-speed divergence >8 rad/s inflates R by 1e5), same uncalibrated
  // defaults, see adaptR() below.
  param_manager_->declare_parameter(
    "P_VDC_wheel_speed_diff_outlier_radps", double{8.0},
    tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_wheel_speed_diff_outlier_R_scale", double{1.0e5},
    tam::types::param::ParameterType::DOUBLE, "");

  // declare parameters for sigma point distribution
  param_manager_->declare_parameter(
    "P_VDC_alpha", double{0.001}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_beta", double{2.0}, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_->declare_parameter(
    "P_VDC_kappa", double{0.0}, tam::types::param::ParameterType::DOUBLE, "");
  // declare parameters containing the initial covariance matrices
  param_manager_->declare_parameter(
    "P_VDC_P_Init",
    std::vector<double>{0.5, 0.0004}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_->declare_parameter(
    "P_VDC_ProcessCov_Q",
    std::vector<double>{0.025, 0.00015}, tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  // R: v_stm, v_stm, FxTf, FxTr, FyTf, FyTr
  param_manager_->declare_parameter(
    "P_VDC_MeasCov_R",
    std::vector<double>{1.00, 1.00, 200.0, 200.0, 150.0, 150.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  initialize_ = true;
  vel_min_mps_ = 5.0;
}

template <class TConfig> void UKFv2<TConfig>::initialize()
{
  // ROOT CAUSE FIX (2026-09-25): P_/R_/Q_ are fixed-size Eigen matrices -- unlike VectorXd,
  // they are NOT zero-initialized by the compiler. Only `.diagonal() = ...` was ever
  // assigned below, so the off-diagonal entries held whatever was in memory at
  // construction (confirmed via debug logging to sometimes be `inf`). That garbage fed
  // straight into the sigma-point eigendecomposition and produced NaN, independent of
  // P_VDC_alpha or build flags -- the alpha/-O3 investigation in
  // docs/TODO_Advisor_Action_Items.md was chasing a real but secondary issue; this
  // uninitialized-memory bug is the actual root cause and explains the run-to-run
  // irreproducibility observed there. It exists in upstream ssa_estimation_cpp too
  // (P_.diagonal() only, no setZero()) -- GPTireUKF's own UKF does call P_.setZero() in
  // its constructor, which is the correct pattern this fork now matches for P_/R_/Q_.
  P_.setZero();
  P_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().size());
  P_ = P_.array().square();
  R_.setZero();
  R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
  R_ = R_.array().square();
  Q_.setZero();
  Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(
    param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().data(),
    param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().size());
  Q_ = Q_.array().square();

  l_m_ = param_manager_->get_parameter_value("P_VDC_l_front_m").as_double() +
    param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double();
  lambda_ = std::pow(param_manager_->get_parameter_value("P_VDC_alpha").as_double(), 2) *
    (x_.size() + param_manager_->get_parameter_value("P_VDC_kappa").as_double()) - x_.size();

  const double c = 0.5 / (x_.size() + lambda_);
  W_mean_ = Eigen::VectorXd::Constant(2 * x_.size() + 1, c);
  W_cov_ = Eigen::VectorXd::Constant(2 * x_.size() + 1, c);
  W_mean_[0] = lambda_ / (x_.size() + lambda_);
  W_cov_[0] = W_mean_[0] +
    (1.0 - std::pow(param_manager_->get_parameter_value("P_VDC_alpha").as_double(), 2) +
    param_manager_->get_parameter_value("P_VDC_beta").as_double());

  // v2: populate the vehicle-model parameter struct once, reused every cycle
  vehicle_params_.mass_kg = param_manager_->get_parameter_value("P_VDC_mass_kg").as_double();
  vehicle_params_.Izz_kgm2 = param_manager_->get_parameter_value("P_VDC_Izz_kgm2").as_double();
  vehicle_params_.l_front_m = param_manager_->get_parameter_value("P_VDC_l_front_m").as_double();
  vehicle_params_.l_rear_m = param_manager_->get_parameter_value("P_VDC_l_rear_m").as_double();
  vehicle_params_.tw_front_m = param_manager_->get_parameter_value("P_VDC_tw_front_m").as_double();
  vehicle_params_.tw_rear_m = param_manager_->get_parameter_value("P_VDC_tw_rear_m").as_double();
  vehicle_params_.rho_air_kgpm3 =
    param_manager_->get_parameter_value("P_VDC_rho_air_kgpm3").as_double();
  vehicle_params_.cL_front = param_manager_->get_parameter_value("P_VDC_cL_front").as_double();
  vehicle_params_.cL_rear = param_manager_->get_parameter_value("P_VDC_cL_rear").as_double();
  vehicle_params_.z_cog_m = param_manager_->get_parameter_value("P_VDC_z_cog_m").as_double();
  vehicle_params_.d_brake_bore_front_m =
    param_manager_->get_parameter_value("P_VDC_d_brake_bore_front_m").as_double();
  vehicle_params_.d_brake_bore_rear_m =
    param_manager_->get_parameter_value("P_VDC_d_brake_bore_rear_m").as_double();
  vehicle_params_.r_brake_pads_lever_front_m =
    param_manager_->get_parameter_value("P_VDC_r_brake_pads_lever_front_m").as_double();
  vehicle_params_.r_brake_pads_lever_rear_m =
    param_manager_->get_parameter_value("P_VDC_r_brake_pads_lever_rear_m").as_double();
  vehicle_params_.mue_brakes_front =
    param_manager_->get_parameter_value("P_VDC_mue_brakes_front").as_double();
  vehicle_params_.mue_brakes_rear =
    param_manager_->get_parameter_value("P_VDC_mue_brakes_rear").as_double();
  vehicle_params_.rtire_unloaded_front_m =
    param_manager_->get_parameter_value("P_VDC_rtire_unloaded_front_m").as_double();
  vehicle_params_.rtire_unloaded_rear_m =
    param_manager_->get_parameter_value("P_VDC_rtire_unloaded_rear_m").as_double();
  vehicle_params_.tire_load_stiffness_front_Npm =
    param_manager_->get_parameter_value("P_VDC_tire_load_stiffness_front_Npm").as_double();
  vehicle_params_.tire_load_stiffness_rear_Npm =
    param_manager_->get_parameter_value("P_VDC_tire_load_stiffness_rear_Npm").as_double();
  vehicle_params_.tire_speed_expansion_front_mpradps2 =
    param_manager_->get_parameter_value("P_VDC_tire_speed_expansion_front_mpradps2").as_double();
  vehicle_params_.tire_speed_expansion_rear_mpradps2 =
    param_manager_->get_parameter_value("P_VDC_tire_speed_expansion_rear_mpradps2").as_double();
  {
    const auto mf_lf = param_manager_->get_parameter_value("P_VDC_tire_MF_long_front").as_double_array();
    const auto mf_lr = param_manager_->get_parameter_value("P_VDC_tire_MF_long_rear").as_double_array();
    const auto mf_af = param_manager_->get_parameter_value("P_VDC_tire_MF_lat_front").as_double_array();
    const auto mf_ar = param_manager_->get_parameter_value("P_VDC_tire_MF_lat_rear").as_double_array();
    for (int i = 0; i < 4; ++i) {
      vehicle_params_.tire_MF_long_front[i] = mf_lf[i];
      vehicle_params_.tire_MF_long_rear[i] = mf_lr[i];
      vehicle_params_.tire_MF_lat_front[i] = mf_af[i];
      vehicle_params_.tire_MF_lat_rear[i] = mf_ar[i];
    }
  }
  wheel_speed_diff_outlier_radps_ =
    param_manager_->get_parameter_value("P_VDC_wheel_speed_diff_outlier_radps").as_double();
  wheel_speed_diff_outlier_R_scale_ =
    param_manager_->get_parameter_value("P_VDC_wheel_speed_diff_outlier_R_scale").as_double();

  // v2: no static load estimate exists yet before the first cycle -- seed with a flat static
  // split so get_virtual_measurement's dynamic-radius lookup has something sane on cycle 1
  load_tire_N_ = vehicle_model_v2::calc_tire_loads(0.0, 0.0, vehicle_params_);
}

// predict() is UNCHANGED from the original ssa_estimation_cpp UKF (including its
// eigendecomposition-based sigma-point square root) -- no tire physics happen here.
template <class TConfig> void
  UKFv2<TConfig>::predict(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u)
{
  double vel_wheel_mps = (param_manager_->get_parameter_value("P_VDC_rtire_unloaded_front_m").as_double() *
      (u[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS] + u[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS]) / 2);
  if (initialize_) {
    tam::core::ssa::UKFv2<TConfig>::initialize();
    x_[0] = vel_wheel_mps;
    x_[1] = 0.0;
    initialize_ = false;
    init_time_s_ = 0.0;
  } else if (std::abs(vel_wheel_mps) <= 2.0) {
    x_[0] = vel_wheel_mps;
    x_[1] = 0.0;
    P_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_P_Init").as_double_array().size());
    P_ = P_.array().square();
    init_time_s_ = 0.0;
  } else {
    {
      Eigen::MatrixXd P_scaled = (x_.size() + lambda_) * P_;
      P_scaled = 0.5 * (P_scaled + P_scaled.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P_scaled);
      Eigen::VectorXd eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
      P_pred_ = eigensolver.eigenvectors() * eigenvalues.cwiseSqrt().asDiagonal();
    }
    Eigen::MatrixXd sigmaPoints(x_.size(), 2 * x_.size() + 1);
    sigmaPoints.col(0) = x_;
    for (int i = 0; i < x_.size(); ++i) {
        sigmaPoints.col(i + 1) = x_ - P_pred_.col(i);
        sigmaPoints.col(x_.size() + i + 1) = x_ + P_pred_.col(i);
    }
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
        sigmaPoints_pred_.col(i) =
          tam::core::ssa::UKFv2<TConfig>::process_function(sigmaPoints.col(i), u);
    }
    x_pred_.setZero();
    P_pred_.setZero();
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
        x_pred_ += W_mean_[i] * sigmaPoints_pred_.col(i);
    }
    for (int i = 0; i < sigmaPoints_pred_.cols(); ++i) {
      Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> residual =
        sigmaPoints_pred_.col(i) - x_pred_;
      P_pred_ += W_cov_[i] * residual * residual.transpose();
    }
    Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(
      param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().data(),
      param_manager_->get_parameter_value("P_VDC_ProcessCov_Q").as_double_array().size());
    Q_ = Q_.array().square();
    P_pred_ += Q_;
  }
}

template <class TConfig> void
  UKFv2<TConfig>::update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  Eigen::MatrixXd sigmaPoints_meas(z_virt_.size(), 2 * x_.size() + 1);
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    sigmaPoints_meas.col(i) = tam::core::ssa::UKFv2<TConfig>::transfer_function(
      sigmaPoints_pred_.col(i), z);
  }
  y_.setZero();
  P_meas_.setZero();
  P_cross_.setZero();
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    y_ += W_mean_[i] * sigmaPoints_meas.col(i);
  }
  for (int i = 0; i < sigmaPoints_meas.cols(); ++i) {
    P_meas_ += W_cov_[i] * (sigmaPoints_meas.col(i) - y_) *
        (sigmaPoints_meas.col(i) - y_).transpose();
    P_cross_ += W_cov_[i] * (sigmaPoints_pred_.col(i) - x_pred_) *
        (sigmaPoints_meas.col(i) - y_).transpose();
  }
  tam::core::ssa::UKFv2<TConfig>::adaptR(z);
  P_meas_ += R_;
  // Numerical safety net, independent of the alpha/-O3 fixes above: a tiny diagonal
  // regularization so a near-singular P_meas_ (e.g. from a degenerate sigma-point spread)
  // still inverts to a large-but-finite K_Gain_ instead of NaN/Inf. 1e-6 is negligible
  // against R_'s own scale (R_ diagonal entries are 1 to ~4e4 after squaring) so it does
  // not change behavior when P_meas_ is already well-conditioned.
  P_meas_.diagonal().array() += 1e-6;
  K_Gain_ = P_cross_ * P_meas_.inverse();

  z_virt_ = tam::core::ssa::UKFv2<TConfig>::get_virtual_measurement(z);

  if (init_time_s_ < 20.0) {
      x_[0] = std::clamp(x_[0], z_virt_[0] - 0.40, z_virt_[0] + 0.40);
      x_[1] = std::clamp(x_[1], -0.00001, 0.00001);
      ++init_time_s_;
  } else {
      x_meas_update_ = K_Gain_ * (z_virt_ - y_);
      x_ = x_pred_ + K_Gain_ * (z_virt_ - y_);
      P_ = P_pred_ - K_Gain_ * P_meas_ * K_Gain_.transpose();
      if (z_virt_[0] <= vel_min_mps_) {
        x_[0] = std::clamp(x_[0], z_virt_[0] - 0.25, z_virt_[0] + 0.25);
        x_[1] = std::clamp(x_[1], -0.0001, 0.0001);
      }
  }

  // v2: same "recompute tire states/loads/forces on the final state, for logging" step as the
  // original -- but load must be computed BEFORE slip now, since the dynamic tire radius
  // (inside calc_tire_states) needs a load estimate. This also refreshes load_tire_N_, which
  // get_virtual_measurement() reads next cycle as its (one-cycle-lag) load estimate for the
  // front-axle dynamic radius -- see vehicle_model.hpp's comment on calc_tire_states.
  const double vel_cog_x_mps_final = x_[TConfig::STATE_VEL_MPS] * std::cos(x_[TConfig::STATE_BETA_RAD]);
  load_tire_N_ = vehicle_model_v2::calc_tire_loads(
    vel_cog_x_mps_final, z[TConfig::UPDATE_AX_MPS2], vehicle_params_);
  Eigen::Vector<double, 4> wheelspeeds_final;
  wheelspeeds_final[TConfig::FRONT_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS];
  wheelspeeds_final[TConfig::FRONT_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS];
  wheelspeeds_final[TConfig::REAR_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_RL_RADPS];
  wheelspeeds_final[TConfig::REAR_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_RR_RADPS];
  Eigen::Matrix<double, 4, 2> slip_tire_perc = vehicle_model_v2::calc_tire_states(
    x_[TConfig::STATE_VEL_MPS], x_[TConfig::STATE_BETA_RAD], z[TConfig::UPDATE_DELTA_RAD],
    z[TConfig::UPDATE_DPSI_RADPS], wheelspeeds_final, load_tire_N_, vehicle_params_, vel_min_mps_);
  slip_tire_long_perc_ = slip_tire_perc.col(0);
  slip_tire_lat_perc_ = slip_tire_perc.col(1);
  Eigen::Vector<double, 6> axle_forces_N = vehicle_model_v2::calc_axle_forces(
    slip_tire_long_perc_, slip_tire_lat_perc_, load_tire_N_, vehicle_params_);
  force_axle_front_N_ = axle_forces_N.segment(0, 3);
  force_axle_rear_N_ = axle_forces_N.segment(3, 3);
}

template <class TConfig> Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>
  UKFv2<TConfig>::get_u(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas)
{
  Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> u;
  u[TConfig::PROCESS_DELTA_RAD] = z_meas[TConfig::MEASUREMENT_DELTA_RAD];
  u[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_RL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS];
  u[TConfig::PROCESS_OMEGA_WHEEL_RR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS];
  u[TConfig::PROCESS_DPSI_RADPS] = u_imu[TConfig::INPUT_DPSI_RADPS];
  u[TConfig::PROCESS_AX_MPS2] = u_imu[TConfig::INPUT_AX_MPS2];
  u[TConfig::PROCESS_AY_MPS2] = u_imu[TConfig::INPUT_AY_MPS2];
  u[TConfig::PROCESS_AZ_MPS2] = u_imu[TConfig::INPUT_AZ_MPS2];
  u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD];
  u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD];
  return u;
}

template <class TConfig> Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>
  UKFv2<TConfig>::get_z(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas)
{
  Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> z;
  z[TConfig::UPDATE_DELTA_RAD] = z_meas[TConfig::MEASUREMENT_DELTA_RAD];
  z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_RL_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS];
  z[TConfig::UPDATE_OMEGA_WHEEL_RR_RADPS] = z_meas[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS];
  z[TConfig::UPDATE_DPSI_RADPS] = u_imu[TConfig::INPUT_DPSI_RADPS];
  z[TConfig::UPDATE_AX_MPS2] = u_imu[TConfig::INPUT_AX_MPS2];
  z[TConfig::UPDATE_AY_MPS2] = u_imu[TConfig::INPUT_AY_MPS2];
  z[TConfig::UPDATE_AZ_MPS2] = u_imu[TConfig::INPUT_AZ_MPS2];
  z[TConfig::UPDATE_DRIVETRAIN_TORQUE_NM] = z_meas[TConfig::MEASUREMENT_DRIVETRAIN_TORQUE_NM];
  z[TConfig::UPDATE_BRAKE_PRESSURE_FL_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_FL_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_FR_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_FR_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_RL_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_RL_PA];
  z[TConfig::UPDATE_BRAKE_PRESSURE_RR_PA] = z_meas[TConfig::MEASUREMENT_BRAKE_PRESSURE_RR_PA];
  z[TConfig::UPDATE_YAW_ACC_RADPS2] = z_meas[TConfig::MEASUREMENT_YAW_ACC_RADPS2];
  z[TConfig::UPDATE_ROAD_ANGLES_PITCH_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD];
  z[TConfig::UPDATE_ROAD_ANGLES_ROLL_RAD] = z_meas[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD];
  return z;
}

// v2: original braking/low-speed adaptation UNCHANGED, PLUS a wheel-speed-difference outlier
// guard ported from GPTireUKF -- if either axle's L/R wheel speeds diverge more than
// wheel_speed_diff_outlier_radps_ (default 8 rad/s), that's evidence of lockup/spin/off-track
// on at least one wheel, so both V_STM channels (which assume symmetric, non-slipping front
// wheels) get their variance inflated by wheel_speed_diff_outlier_R_scale_ (default 1e5) --
// the original adaptR had no such guard at all.
template <class TConfig> void
  UKFv2<TConfig>::adaptR(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  if (z[TConfig::PROCESS_AX_MPS2] <= 0.0) {
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_1_MPS] = 20.0;
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_2_MPS] = 20.0;
      R_ = R_.array().square();
  } else {
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_ = R_.array().square();
  }
  if (x_[TConfig::STATE_VEL_MPS] <= 2.0) {
      R_.diagonal() = Eigen::Map<Eigen::VectorXd>(
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().data(),
        param_manager_->get_parameter_value("P_VDC_MeasCov_R").as_double_array().size());
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_1_MPS] = 0.01;
      R_.diagonal()[TConfig::VIRTMEAS_V_STM_2_MPS] = 0.01;
      R_ = R_.array().square();
  }
  // v2 NEW: wheel-speed-difference outlier guard
  const double front_diff_radps = std::abs(
    z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS] - z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS]);
  const double rear_diff_radps = std::abs(
    z[TConfig::UPDATE_OMEGA_WHEEL_RL_RADPS] - z[TConfig::UPDATE_OMEGA_WHEEL_RR_RADPS]);
  if (front_diff_radps > wheel_speed_diff_outlier_radps_ ||
      rear_diff_radps > wheel_speed_diff_outlier_radps_) {
    R_.diagonal()[TConfig::VIRTMEAS_V_STM_1_MPS] *= wheel_speed_diff_outlier_R_scale_;
    R_.diagonal()[TConfig::VIRTMEAS_V_STM_2_MPS] *= wheel_speed_diff_outlier_R_scale_;
  }
}

// process_function() is UNCHANGED from the original -- no tire physics, pure kinematic
// point-mass integration with banking compensation.
template <class TConfig> Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>
  UKFv2<TConfig>::process_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u)
{
  double vel_mps = sigma_point[TConfig::STATE_VEL_MPS];
  double beta_rad = sigma_point[TConfig::STATE_BETA_RAD];
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> sigma_point_calc;
  sigma_point_calc[TConfig::STATE_VEL_MPS] =
    vel_mps + TConfig::TS *
    (std::cos(beta_rad) * (u[TConfig::PROCESS_AX_MPS2] +
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]) +
    std::sin(beta_rad) * (u[TConfig::PROCESS_AY_MPS2] -
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD]) *
      std::cos(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]));
  sigma_point_calc[TConfig::STATE_BETA_RAD] =
    beta_rad + TConfig::TS *
    (-std::sin(beta_rad) * (u[TConfig::PROCESS_AX_MPS2] +
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2]) /
      (vel_mps + 1e-15) +
    (std::cos(beta_rad) * (u[TConfig::PROCESS_AY_MPS2] -
      std::sin(u[TConfig::PROCESS_ROAD_ANGLES_ROLL_RAD]) *
      std::cos(u[TConfig::PROCESS_ROAD_ANGLES_PITCH_RAD]) * u[TConfig::PROCESS_AZ_MPS2])) /
      (vel_mps + 1e-15) -
    u[TConfig::PROCESS_DPSI_RADPS]);
  return sigma_point_calc;
}

// v2: kinematic (wheel-speed) path UNCHANGED except the wheelspeed->velocity conversion now
// uses the dynamic tire radius (load- and speed-dependent) instead of a fixed constant --
// load comes from load_tire_N_, the PREVIOUS cycle's estimate (see update()'s comment).
// Force-balance path: UNCHANGED from v1 (aero drag/rolling resistance deliberately not deducted
// -- see vehicle_model.hpp calc_axle_forces_evidence).
template <class TConfig> Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE>
 UKFv2<TConfig>::get_virtual_measurement(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> z_out;

  const double r_dyn_fl = vehicle_model_v2::calc_dyn_tire_radius(
    vehicle_params_.rtire_unloaded_front_m, load_tire_N_[TConfig::FRONT_LEFT],
    vehicle_params_.tire_load_stiffness_front_Npm, z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS],
    vehicle_params_.tire_speed_expansion_front_mpradps2);
  const double r_dyn_fr = vehicle_model_v2::calc_dyn_tire_radius(
    vehicle_params_.rtire_unloaded_front_m, load_tire_N_[TConfig::FRONT_RIGHT],
    vehicle_params_.tire_load_stiffness_front_Npm, z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS],
    vehicle_params_.tire_speed_expansion_front_mpradps2);

  z_out[TConfig::VIRTMEAS_V_STM_1_MPS] = z[TConfig::UPDATE_OMEGA_WHEEL_FL_RADPS] * r_dyn_fl +
    std::cos(z[TConfig::UPDATE_DELTA_RAD]) * z[TConfig::UPDATE_DPSI_RADPS] *
    vehicle_params_.tw_front_m * 0.5;
  z_out[TConfig::VIRTMEAS_V_STM_2_MPS] = z[TConfig::UPDATE_OMEGA_WHEEL_FR_RADPS] * r_dyn_fr -
    std::cos(z[TConfig::UPDATE_DELTA_RAD]) * z[TConfig::UPDATE_DPSI_RADPS] *
    vehicle_params_.tw_front_m * 0.5;

  Eigen::Vector<double, 4> ftx_fty = vehicle_model_v2::calc_axle_forces_evidence(
    z[TConfig::UPDATE_DELTA_RAD], z[TConfig::UPDATE_AY_MPS2], z[TConfig::UPDATE_YAW_ACC_RADPS2],
    z[TConfig::UPDATE_DRIVETRAIN_TORQUE_NM],
    z[TConfig::UPDATE_BRAKE_PRESSURE_FL_PA], z[TConfig::UPDATE_BRAKE_PRESSURE_FR_PA],
    z[TConfig::UPDATE_BRAKE_PRESSURE_RL_PA], z[TConfig::UPDATE_BRAKE_PRESSURE_RR_PA],
    vehicle_params_);
  z_out[TConfig::VIRTMEAS_FTX_F_N] = ftx_fty[0];
  z_out[TConfig::VIRTMEAS_FTX_R_N] = ftx_fty[1];
  z_out[TConfig::VIRTMEAS_FTY_F_N] = ftx_fty[2];
  z_out[TConfig::VIRTMEAS_FTY_R_N] = ftx_fty[3];
  return z_out;
}

// v2: V_STM prediction from the candidate (V, beta) UNCHANGED (pure kinematics, no radius
// dependency on this side, same as the original). Tire load must now be computed BEFORE slip
// (dynamic radius needs it), so the call order changed from the original's
// slip-then-load-then-forces to load-then-slip-then-forces.
template <class TConfig> Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE>
  UKFv2<TConfig>::transfer_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z)
{
  double vel_mps = sigma_point[TConfig::STATE_VEL_MPS];
  double beta_rad = sigma_point[TConfig::STATE_BETA_RAD];
  Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> transformed_sigma_pnt;
  transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_1_MPS] =
    std::cos(z[TConfig::UPDATE_DELTA_RAD] - beta_rad) * vel_mps +
    std::sin(beta_rad) * vehicle_params_.l_front_m * z[TConfig::UPDATE_DPSI_RADPS];
  transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_2_MPS] =
    transformed_sigma_pnt[TConfig::VIRTMEAS_V_STM_1_MPS];

  const double vel_cog_x_mps = vel_mps * std::cos(beta_rad);
  Eigen::Vector<double, 4> load_tire_N = vehicle_model_v2::calc_tire_loads(
    vel_cog_x_mps, z[TConfig::UPDATE_AX_MPS2], vehicle_params_);

  Eigen::Vector<double, 4> wheelspeeds_radps;
  wheelspeeds_radps[TConfig::FRONT_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_FL_RADPS];
  wheelspeeds_radps[TConfig::FRONT_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_FR_RADPS];
  wheelspeeds_radps[TConfig::REAR_LEFT] = z[TConfig::PROCESS_OMEGA_WHEEL_RL_RADPS];
  wheelspeeds_radps[TConfig::REAR_RIGHT] = z[TConfig::PROCESS_OMEGA_WHEEL_RR_RADPS];
  Eigen::Matrix<double, 4, 2> slip_tire_perc = vehicle_model_v2::calc_tire_states(
    vel_mps, beta_rad, z[TConfig::UPDATE_DELTA_RAD], z[TConfig::UPDATE_DPSI_RADPS],
    wheelspeeds_radps, load_tire_N, vehicle_params_, vel_min_mps_);
  Eigen::Vector<double, 4> slip_tire_long_perc = slip_tire_perc.col(0);
  Eigen::Vector<double, 4> slip_tire_lat_perc = slip_tire_perc.col(1);

  Eigen::Vector<double, 6> axle_forces_N = vehicle_model_v2::calc_axle_forces(
    slip_tire_long_perc, slip_tire_lat_perc, load_tire_N, vehicle_params_);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTX_F_N] = axle_forces_N(0);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTX_R_N] = axle_forces_N(3);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTY_F_N] = axle_forces_N(1);
  transformed_sigma_pnt[TConfig::VIRTMEAS_FTY_R_N] = axle_forces_N(4);
  return transformed_sigma_pnt;
}

template <class TConfig> Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>
  UKFv2<TConfig>::get_state_vector()
{
  return x_;
}

template <class TConfig> Eigen::Matrix<double, 2, 2>
  UKFv2<TConfig>::get_linear_velocity_covariance()
{
  double beta = x_[TConfig::STATE_BETA_RAD];
  double vel = x_[TConfig::STATE_VEL_MPS];
  Eigen::Matrix2d P;
  P(0, 0) = P_(TConfig::STATE_VEL_MPS, TConfig::STATE_VEL_MPS);
  P(0, 1) = P_(TConfig::STATE_VEL_MPS, TConfig::STATE_BETA_RAD);
  P(1, 0) = P_(TConfig::STATE_BETA_RAD, TConfig::STATE_VEL_MPS);
  P(1, 1) = P_(TConfig::STATE_BETA_RAD, TConfig::STATE_BETA_RAD);
  Eigen::Matrix2d J;
  J(0, 0) = std::cos(beta);
  J(0, 1) = -vel * std::sin(beta);
  J(1, 0) = std::sin(beta);
  J(1, 1) = vel * std::cos(beta);
  return J * P * J.transpose();
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase>
  UKFv2<TConfig>::get_param_handler()
{
  return param_manager_;
}

template <class TConfig> std::map<std::string, double>
  UKFv2<TConfig>::get_debug()
{
  std::map<std::string, double> debug_output;

  for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; ++i) {
    debug_output["x_" + std::to_string(i)] = x_[i];
    debug_output["x_pred_" + std::to_string(i)] = x_pred_[i];
    debug_output["x_meas_update_" + std::to_string(i)] = x_meas_update_[i];
    for (int j = 0; j < TConfig::STATE_VECTOR_SIZE; ++j) {
      debug_output["P_" + std::to_string(i) + std::to_string(j)] = P_(i, j);
      debug_output["P_pred_" + std::to_string(i) + std::to_string(j)] = P_pred_(i, j);
    }
    for (int j = 0; j < TConfig::VIRTMEAS_VECTOR_SIZE; ++j) {
      debug_output["K_Gain_" + std::to_string(i) + std::to_string(j)] = K_Gain_(i, j);
    }
  }
  for (int i = 0; i < TConfig::VIRTMEAS_VECTOR_SIZE; ++i) {
    for (int j = 0; j < TConfig::VIRTMEAS_VECTOR_SIZE; ++j) {
      debug_output["P_meas_" + std::to_string(i) + std::to_string(j)] = P_meas_(i, j);
    }
    debug_output["y_" + std::to_string(i)] = y_[i];
    debug_output["z_virt_" + std::to_string(i)] = z_virt_[i];
  }
  for (int i = 0; i < 4; i++) {
    debug_output["slip_tire_long_perc_" + std::to_string(i)] = slip_tire_long_perc_[i];
    debug_output["slip_tire_lat_perc_" + std::to_string(i)] = slip_tire_lat_perc_[i];
    debug_output["load_tire_N_" + std::to_string(i)] = load_tire_N_[i];
  }
  debug_output["force_axle_front_N_long"] = force_axle_front_N_[0];
  debug_output["force_axle_rear_N_long"] = force_axle_rear_N_[0];
  debug_output["force_axle_front_N_lat"] = force_axle_front_N_[1];
  debug_output["force_axle_rear_N_lat"] = force_axle_rear_N_[1];
  debug_output["force_axle_front_N_vert"] = force_axle_front_N_[2];
  debug_output["force_axle_rear_N_vert"] = force_axle_rear_N_[2];
  // v2 NEW debug fields
  debug_output["dyn_tire_radius_fl_m"] = vehicle_model_v2::calc_dyn_tire_radius(
    vehicle_params_.rtire_unloaded_front_m, load_tire_N_[TConfig::FRONT_LEFT],
    vehicle_params_.tire_load_stiffness_front_Npm, 0.0, 0.0);
  debug_output["z_cog_m"] = vehicle_params_.z_cog_m;

  return debug_output;
}
}  // namespace tam::core::ssa

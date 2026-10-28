// Copyright 2026
// v2 fork: verbatim copy of ssa_estimation_cpp/ssa_estimation_impl.hpp (originally Copyright
// 2024 Sven Goblirsch) -- no logic changed here, only the include path in ssa_estimation.hpp
// differs (pulls in this package's v2 UKF instead of the original's).
#pragma once
#include "ssa_estimation_v2_cpp/ssa_estimation.hpp"
namespace tam::core::ssa
{
template <class TConfig>
SSAEstimationV2<TConfig>::SSAEstimationV2()
{
  state_machine_ = std::make_unique<tam::core::ssa::StateMachine<TConfig>>();
  imu_handler_ = std::make_unique<tam::core::state::IMUHandler<TConfig>>();
  kalman_filter_ = std::make_unique<tam::core::ssa::UKFv2<TConfig>>();

  param_manager_composer_ = std::make_shared<tam::core::ParamManagerComposer>(
    std::vector<std::shared_ptr<tam::interfaces::ParamManagerBase>>{
      state_machine_->get_param_handler(), imu_handler_->get_param_handler(),
      kalman_filter_->get_param_handler()});

  param_manager_composer_->declare_parameter(
    "P_VDC_HardAccelerometerOutlierTH", 1000.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_HardAngularVelocityOutlierTH", 1.0, tam::types::param::ParameterType::DOUBLE, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_MaxConsecutiveIMUHardOutliers", 50, tam::types::param::ParameterType::INTEGER, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_Overwrite_Acceleration_Z", false, tam::types::param::ParameterType::BOOL, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_InitialBias", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");
  param_manager_composer_->declare_parameter(
    "P_VDC_yaw_rate_post_filter_coefficients",
    std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01},
    tam::types::param::ParameterType::DOUBLE_ARRAY, "");

  x_out_.setZero();
  u_imu_.setZero();
  u_imu_raw_.setZero();
  z_meas_.setZero();
  outlier_buffer_.setZero();
  updated_acceleration_.setZero();
  updated_angular_velocity_.setZero();
}

template <class TConfig> void SSAEstimationV2<TConfig>::step()
{
  // NOTE (2026-09-25): a "skip step() until a real wheelspeed sample has arrived" gate was
  // tried here and reverted -- it did not help, because the car is genuinely below 2 m/s
  // for the bag's own first ~6.7s regardless of when step() starts, so predict()'s
  // low-speed branch keeps resetting P_ either way. The actual fix for the resulting NaN
  // (P_VDC_alpha + build flags) is in ssa_estimation_v2_ukf_config.yml and the package
  // CMakeLists -- see docs/TODO_Advisor_Action_Items.md.
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias = Eigen::Map<Eigen::VectorXd>(
    param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().data(),
    param_manager_composer_->get_parameter_value("P_VDC_InitialBias").as_double_array().size());
  imu_handler_->set_sensor_bias(imu_bias);

  u_imu_ = imu_handler_->update_input_vector(u_imu_raw_, state_machine_->get_valid_u_fusion_vec());

  if (initialize_filter_) {
    Eigen::VectorXd filter_coefficients = Eigen::Map<Eigen::VectorXd>(
        param_manager_composer_->get_parameter_value(
            "P_VDC_yaw_rate_post_filter_coefficients").as_double_array().data(),
        param_manager_composer_->get_parameter_value(
            "P_VDC_yaw_rate_post_filter_coefficients").as_double_array().size());
    filter_yaw_rate_radps_ = std::make_unique<tam::core::state::FIR>(filter_coefficients);
    initialize_filter_ = false;
  }
  double yaw_rate_radps = filter_yaw_rate_radps_->step(u_imu_[TConfig::INPUT_DPSI_RADPS]);
  yaw_acc_radps2_ = (yaw_rate_radps - prev_yaw_rate_radps_) / TConfig::TS;
  prev_yaw_rate_radps_ = yaw_rate_radps;
  z_meas_[TConfig::MEASUREMENT_YAW_ACC_RADPS2] = yaw_acc_radps2_;

  Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> u =
    kalman_filter_->get_u(u_imu_, z_meas_);
  Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> z =
    kalman_filter_->get_z(u_imu_, z_meas_);
  kalman_filter_->predict(u);
  kalman_filter_->update(z);
  x_out_ = kalman_filter_->get_state_vector();
  state_machine_->check_output_feasability(x_out_);
  state_machine_->update();
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_acceleration(
  const tam::types::control::AccelerationwithCovariances & acceleration, const uint8_t imu_num)
{
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    Eigen::Vector<double, 3> acc_vec;
    acc_vec << acceleration.acceleration_mps2.x, acceleration.acceleration_mps2.y,
      acceleration.acceleration_mps2.z;
    Eigen::Vector<double,  3> outlier_distance;
    outlier_distance = acc_vec.array().abs()
      - u_imu_.segment(TConfig::INPUT_AX_MPS2, 3).array().abs();
    Eigen::Vector<double,  3> outlier_distance_sqr;
    outlier_distance_sqr = outlier_distance.cwiseProduct(outlier_distance);
    if (
      (param_manager_composer_->get_parameter_value("P_VDC_HardAccelerometerOutlierTH").as_double()
         > outlier_distance_sqr.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT ||
         (0.0 == u_imu_.segment(TConfig::INPUT_AX_MPS2, 3).array()).all()) {
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AX_MPS2]
        = acceleration.acceleration_mps2.x;
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AY_MPS2]
        = acceleration.acceleration_mps2.y;
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_AZ_MPS2]
        = acceleration.acceleration_mps2.z;
      outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num] = 0;
    } else {
      outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num]++;
    }
    if (updated_angular_velocity_[imu_num] == 1) {
      u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6) =
        imu_handler_->filter_imu_measurements(
          u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6), imu_num);
      updated_angular_velocity_[imu_num] = 0;
    } else {
      updated_acceleration_[imu_num] = 1;
    }
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_angular_velocity(
  const tam::types::control::Odometry & odometry, const uint8_t imu_num)
{
  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    Eigen::Vector<double, 3> angvel_vec;
    angvel_vec << odometry.angular_velocity_radps.x, odometry.angular_velocity_radps.y,
      odometry.angular_velocity_radps.z;
    Eigen::Vector<double,  3> outlier_distance;
    outlier_distance = angvel_vec.array().abs()
      - u_imu_.segment(TConfig::INPUT_DPHI_RADPS, 3).array().abs();
    Eigen::Vector<double,  3> outlier_distance_sqr;
    outlier_distance_sqr = outlier_distance.cwiseProduct(outlier_distance);
    if (
      (param_manager_composer_->get_parameter_value("P_VDC_HardAngularVelocityOutlierTH").as_double()
         > outlier_distance_sqr.array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT || (0.0 ==
         u_imu_.segment(TConfig::INPUT_DPHI_RADPS, 3).array()).all()) {
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPHI_RADPS]
        = odometry.angular_velocity_radps.x;
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DTHETA_RADPS]
        = odometry.angular_velocity_radps.y;
      u_imu_raw_[imu_num * TConfig::INPUT_VECTOR_SIZE + TConfig::INPUT_DPSI_RADPS]
        = odometry.angular_velocity_radps.z;
      outlier_buffer_[imu_num] = 0;
    } else {
      outlier_buffer_[imu_num]++;
    }
    if (updated_acceleration_[imu_num] == 1) {
      u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6) =
        imu_handler_->filter_imu_measurements(
          u_imu_raw_.segment(imu_num * TConfig::INPUT_VECTOR_SIZE, 6), imu_num);
      updated_acceleration_[imu_num] = 0;
    } else {
      updated_angular_velocity_[imu_num] = 1;
    }
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_imu_status(
  const tam::types::ErrorLvl & status, const uint8_t imu_num)
{
  int max_outlier_threshold =
    param_manager_composer_->get_parameter_value("P_VDC_MaxConsecutiveIMUHardOutliers").as_int();
  tam::types::ErrorLvl imu_status = status;

  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    if (outlier_buffer_[TConfig::NUM_IMU_MEASUREMENT + imu_num] > max_outlier_threshold ||
          outlier_buffer_[imu_num] > max_outlier_threshold) {
          imu_status = tam::types::ErrorLvl::ERROR;
    }
  }

  if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
    state_machine_->set_imu_status(imu_status, imu_num);
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_imu_status(
  const uint8_t status, const uint8_t imu_num)
{
  set_input_imu_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)),
    imu_num);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_imu_timeout(const uint8_t imu_num)
{
  if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
    state_machine_->set_imu_invalid(imu_num);
  } else {
    throw std::invalid_argument(
      "[SSAEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
  }
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_wheelspeeds(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_FL_RADPS] = wheel.front_left;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_FR_RADPS] = wheel.front_right;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_RL_RADPS] = wheel.rear_left;
  z_meas_[TConfig::MEASUREMENT_OMEGA_WHEEL_RR_RADPS] = wheel.rear_right;
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_wheelspeed_status(
  const tam::types::ErrorLvl & status)
{
  state_machine_->set_wheelspeed_status(status);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_wheelspeed_status(
  const uint8_t status)
{
  state_machine_->set_wheelspeed_status(tam::type_conversions::error_type_from_diagnostic_level(
    static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_wheelspeed_timeout()
{
  state_machine_->set_wheelspeed_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_steering_angle(
  const double steering_angle)
{
  z_meas_[TConfig::MEASUREMENT_DELTA_RAD] = steering_angle;
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_steering_angle_status(
  const tam::types::ErrorLvl & status)
{
  state_machine_->set_steering_status(status);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_steering_angle_status(
  const uint8_t status)
{
  state_machine_->set_steering_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_steering_angle_timeout()
{
  state_machine_->set_steering_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_drivetrain_torque(
  const double drivetrain_trq_Nm)
{
  z_meas_[TConfig::MEASUREMENT_DRIVETRAIN_TORQUE_NM] = drivetrain_trq_Nm;
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_drivetrain_torque_status(
  const tam::types::ErrorLvl & status)
{
  state_machine_->set_drivetrain_torque_status(status);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_drivetrain_torque_status(
  const uint8_t status)
{
  state_machine_->set_drivetrain_torque_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_drivetrain_torque_timeout()
{
  state_machine_->set_drivetrain_torque_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_brake_pressure(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_FL_PA] = wheel.front_left;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_FR_PA] = wheel.front_right;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_RL_PA] = wheel.rear_left;
  z_meas_[TConfig::MEASUREMENT_BRAKE_PRESSURE_RR_PA] = wheel.rear_right;
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_brake_pressure_status(
  const tam::types::ErrorLvl & status)
{
  state_machine_->set_brake_pressure_status(status);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_brake_pressure_status(
  const uint8_t status)
{
  state_machine_->set_brake_pressure_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_brake_pressure_timeout()
{
  state_machine_->set_brake_pressure_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> void SSAEstimationV2<TConfig>::set_input_vehicle_orientation(
  const tam::types::control::Odometry & odometry)
{
  z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD] = odometry.orientation_rad.x;
  z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD] = odometry.orientation_rad.y;
}

template <class TConfig> void SSAEstimationV2<TConfig>:: set_input_orientation_status(
  const tam::types::ErrorLvl & status)
{
  state_machine_->set_orientation_status(status);
}

template <class TConfig> void SSAEstimationV2<TConfig>:: set_input_orientation_status(
  const uint8_t status)
{
  state_machine_->set_orientation_status(
    tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
}

template <class TConfig> void SSAEstimationV2<TConfig>:: set_orientation_timeout()
{
  state_machine_->set_orientation_status(tam::types::ErrorLvl::ERROR);
}

template <class TConfig> std::shared_ptr<tam::interfaces::ParamManagerBase> SSAEstimationV2<TConfig>::get_param_handler()
{
  return param_manager_composer_;
}

template <class TConfig> tam::types::control::Odometry SSAEstimationV2<TConfig>::get_odometry()
{
  tam::types::control::Odometry output;

  output.velocity_mps.x = x_out_[TConfig::STATE_VEL_MPS] *
    std::cos(x_out_[TConfig::STATE_BETA_RAD]);
  output.velocity_mps.y = x_out_[TConfig::STATE_VEL_MPS] *
    std::sin(x_out_[TConfig::STATE_BETA_RAD]);
  output.velocity_mps.z = 0.0;

  Eigen::Matrix<double, 2, 2> P = kalman_filter_->get_linear_velocity_covariance();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      output.velocity_covariance[i * 6 + j] = P(i, j);
    }
  }
  return output;
}

template <class TConfig> tam::types::ErrorLvl SSAEstimationV2<TConfig>::get_status()
{
  return state_machine_->get_state();
}

template <class TConfig> uint8_t SSAEstimationV2<TConfig>::get_status_as_uint8_t()
{
  return static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(
    state_machine_->get_state()));
}

template <class TConfig> std::string SSAEstimationV2<TConfig>::get_state_machine_status_msg()
{
  return state_machine_->get_status_msg();
}

template <class TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  SSAEstimationV2<TConfig>::get_state_machine_debug_output()
{
  std::map<std::string, bool> state_machine_debug = state_machine_->get_debug();

  for (std::map<std::string, bool>::const_iterator it = state_machine_debug.begin();
       it != state_machine_debug.end(); ++it) {
    state_machine_debug_container_->log(it->first, it->second);
  }
  state_machine_debug_container_->log(
    "overall_state", static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(
                       state_machine_->get_state())));

  return state_machine_debug_container_;
}

template <class TConfig> tam::types::common::TUMDebugContainer::SharedPtr
  SSAEstimationV2<TConfig>::get_kalman_filter_debug_output()
{
  std::map<std::string, double> kalman_filter_debug = kalman_filter_->get_debug();

  for (std::map<std::string, double>::const_iterator it = kalman_filter_debug.begin();
       it != kalman_filter_debug.end(); ++it) {
    kalman_filter_debug_container_->log(it->first, it->second);
  }
  kalman_filter_debug_container_->log("roll_angle_rad",
    z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_ROLL_RAD]);
  kalman_filter_debug_container_->log("pitch_angle_rad",
    z_meas_[TConfig::MEASUREMENT_ROAD_ANGLES_PITCH_RAD]);
  kalman_filter_debug_container_->log("filtered_acc_x_mps2",
    u_imu_[TConfig::INPUT_AX_MPS2]);
  kalman_filter_debug_container_->log("filtered_acc_y_mps2",
    u_imu_[TConfig::INPUT_AY_MPS2]);
  kalman_filter_debug_container_->log("filtered_acc_z_mps2",
    u_imu_[TConfig::INPUT_AZ_MPS2]);
  kalman_filter_debug_container_->log("filtered_dpsi_radps",
    u_imu_[TConfig::INPUT_DPSI_RADPS]);
  kalman_filter_debug_container_->log("yaw_acc_radps2", yaw_acc_radps2_);
  return kalman_filter_debug_container_;
}
}  // namespace tam::core::ssa

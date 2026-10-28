// Copyright 2026
// v2 fork of 3DVehicleDynamicsStateEstimation's lib_cpp/ssa_estimation_cpp/ssa_estimation.hpp
// (originally Copyright 2024 Sven Goblirsch). This wrapper class is UNCHANGED from the
// original -- input plumbing, state machine, IMU handling are identical. The only actual
// difference vs. the original is which UKF it instantiates (see ssa_estimation_impl.hpp):
// this package's own v2 UKF (ssa_estimation_v2_cpp/submodules/ukf.hpp) instead of the
// original's. state_machine.hpp is intentionally still pulled in from the ORIGINAL
// ssa_estimation_cpp package, unmodified -- no reason to fork logic that didn't change.
#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"

// type convertions
#include "tum_helpers_cpp/rotations.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager_composer.hpp"

// SSA Estimation Base Class -- reused unchanged from the original package
#include "ssa_estimation_base/ssa_estimation_base.hpp"

// Template Input -- reused unchanged from the original package (state vector didn't change)
#include "ssa_estimation_constants/UKF_STM.hpp"

// SSA Estimation State Machine -- reused unchanged from the original package
#include "ssa_estimation_cpp/submodules/state_machine.hpp"
// v2 UKF (this package)
#include "ssa_estimation_v2_cpp/submodules/ukf.hpp"

// IMU Preprocessing -- reused unchanged from the original package
#include "imu_handler/imu_handler.hpp"

// FIR Filter -- reused unchanged from the original package
#include "classical_filter/fir.hpp"

namespace tam::core::ssa
{
template <class TConfig>
class SSAEstimationV2 : public SSAEstimationBase
{
private:
  std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_composer_;
  std::shared_ptr<tam::core::ssa::StateMachine<TConfig>> state_machine_;
  std::shared_ptr<tam::core::state::IMUHandler<TConfig>> imu_handler_;
  std::shared_ptr<tam::core::ssa::UKFv2<TConfig>> kalman_filter_;

  tam::types::common::TUMDebugContainer::SharedPtr state_machine_debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();
  tam::types::common::TUMDebugContainer::SharedPtr kalman_filter_debug_container_ =
    std::make_shared<tam::types::common::TUMDebugContainer>();

  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_out_;
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_imu_;
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE *
    (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_imu_raw_;
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_meas_;
  Eigen::Vector<int, 2 * (TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> outlier_buffer_;
  Eigen::Vector<double, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_acceleration_;
  Eigen::Vector<double, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>
    updated_angular_velocity_;

  std::unique_ptr<tam::core::state::FIR> filter_yaw_rate_radps_;
  bool initialize_filter_ = true;
  double prev_yaw_rate_radps_{0.0};
  double yaw_acc_radps2_{0.0};

public:
  SSAEstimationV2();
  void step() override;

  void set_input_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration,
    const uint8_t imu_num) override;
  void set_input_angular_velocity(const tam::types::control::Odometry & odometry,
    const uint8_t imu_num) override;
  void set_input_imu_status(const tam::types::ErrorLvl & status, const uint8_t imu_num) override;
  void set_input_imu_status(const uint8_t status, const uint8_t imu_num) override;
  void set_imu_timeout(const uint8_t imu_num) override;

  void set_input_wheelspeeds(const tam::types::common::DataPerWheel<double> & wheel) override;
  void set_input_wheelspeed_status(const tam::types::ErrorLvl & status) override;
  void set_input_wheelspeed_status(const uint8_t status) override;
  void set_wheelspeed_timeout() override;

  void set_input_steering_angle(const double steering_angle) override;
  void set_input_steering_angle_status(const tam::types::ErrorLvl & status) override;
  void set_input_steering_angle_status(const uint8_t status) override;
  void set_steering_angle_timeout() override;

  void set_input_drivetrain_torque(const double drivetrain_trq_Nm) override;
  void set_input_drivetrain_torque_status(const tam::types::ErrorLvl & status) override;
  void set_input_drivetrain_torque_status(const uint8_t status) override;
  void set_drivetrain_torque_timeout() override;

  void set_input_brake_pressure(const tam::types::common::DataPerWheel<double> & wheel) override;
  void set_input_brake_pressure_status(const tam::types::ErrorLvl & status) override;
  void set_input_brake_pressure_status(const uint8_t status) override;
  void set_brake_pressure_timeout() override;

  void set_input_vehicle_orientation(const tam::types::control::Odometry & odometry) override;
  void set_input_orientation_status(const tam::types::ErrorLvl & status);
  void set_input_orientation_status(const uint8_t status);
  void set_orientation_timeout();

  std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler() override;
  tam::types::control::Odometry get_odometry() override;
  tam::types::ErrorLvl get_status() override;
  uint8_t get_status_as_uint8_t() override;
  std::string get_state_machine_status_msg() override;
  tam::types::common::TUMDebugContainer::SharedPtr get_state_machine_debug_output() override;
  tam::types::common::TUMDebugContainer::SharedPtr get_kalman_filter_debug_output() override;
};
}  // namespace tam::core::ssa
#include "ssa_estimation_v2_cpp/ssa_estimation_impl.hpp"

// Copyright 2026
// v2 fork of 3DVehicleDynamicsStateEstimation's lib_cpp/ssa_estimation_cpp UKF (originally
// Copyright 2024 Sven Goblirsch), extended with physics ported by hand from TUMFTM/GPTireUKF
// (dynamic tire radius, longitudinal load transfer, richer force-balance evidence path) plus
// one robustness addition (wheel-speed-difference outlier widening in adaptR). See
// docs/versions/v2_draft/direction.md for what changed and why, and
// vehicle_model_v2_cpp/vehicle_model.hpp for exactly what was/wasn't ported from GPTireUKF.
//
// State vector, sigma-point math, process_function, and the overall two-path
// (kinematic-vs-force-balance) design are UNCHANGED from the original -- including its own
// eigendecomposition-based sigma-point square root (this workspace's own numerical-stability
// fix; GPTireUKF itself regressed to plain Cholesky, not ported here on purpose).
#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_manager_cpp/param_manager_base.hpp"
#include "param_manager_cpp/param_manager.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// SSA Estimation constants / template input -- REUSED UNCHANGED from the original package,
// state vector [V, beta] and all vector layouts are identical, no fork needed here.
#include "ssa_estimation_constants/UKF_STM.hpp"

// v2: vehicle/tire physics, forked out into its own package (see file header)
#include "vehicle_model_v2_cpp/vehicle_model.hpp"

namespace tam::core::ssa
{
template <class TConfig>
class UKFv2
{
private:
 bool initialize_;
 double vel_min_mps_;
 double l_m_;
 double lambda_;
 Eigen::Vector<double, 4> slip_tire_long_perc_;
 Eigen::Vector<double, 4> slip_tire_lat_perc_;
 Eigen::Vector<double, 4> load_tire_N_;
 Eigen::Vector<double, 3> force_axle_front_N_;
 Eigen::Vector<double, 3> force_axle_rear_N_;

 // v2: physics parameters, populated from param_manager_ in the constructor, passed by value
 // into the free functions in vehicle_model_v2_cpp -- see that header for what each one means
 // and which are still uncalibrated placeholders (z_cog_m, aero drag coefficients).
 vehicle_model_v2::VehicleModelParams vehicle_params_;

 // v2: wheel-speed-difference outlier guard for adaptR (see adaptR() in ukf_impl.hpp)
 double wheel_speed_diff_outlier_radps_{8.0};
 double wheel_speed_diff_outlier_R_scale_{1.0e5};

public:
 double init_time_s_ = 0.0;
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_;
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_pred_;
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_meas_update_;
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> y_;
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> z_virt_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_pred_;
 Eigen::Matrix<double, TConfig::VIRTMEAS_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> P_meas_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> P_cross_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> K_Gain_;
 Eigen::VectorXd W_mean_;
 Eigen::VectorXd W_cov_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> Q_;
 Eigen::Matrix<double, TConfig::VIRTMEAS_VECTOR_SIZE, TConfig::VIRTMEAS_VECTOR_SIZE> R_;
 Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE,
    2 * TConfig::STATE_VECTOR_SIZE + 1> sigmaPoints_pred_;
 std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;

 UKFv2();
 void initialize();
 void predict(const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u);
 void update(const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);
 Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE> get_u(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas);
 Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE> get_z(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> & u_imu,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> & z_meas);
 void adaptR(const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> process_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::PROCESS_VECTOR_SIZE>> & u);
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> get_virtual_measurement(
   const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);
 Eigen::Vector<double, TConfig::VIRTMEAS_VECTOR_SIZE> transfer_function(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> & sigma_point,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::UPDATE_VECTOR_SIZE>> & z);
 Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> get_state_vector();
 Eigen::Matrix<double, 2, 2> get_linear_velocity_covariance();
 std::shared_ptr<tam::interfaces::ParamManagerBase> get_param_handler();
 std::map<std::string, double> get_debug();
};
}  // namespace tam::core::ssa
#include "ssa_estimation_v2_cpp/submodules/ukf_impl.hpp"

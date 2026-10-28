// Copyright 2026
#pragma once
// v2 fork: physics extracted out of ssa_estimation_cpp's monolithic UKF class into its own
// header-only library, mirroring TUMFTM/GPTireUKF's package split (lib_cpp/vehicle_model_cpp)
// -- ported by hand against this workspace's own param_manager_cpp/Eigen conventions rather
// than importing GPTireUKF's files verbatim, to avoid dragging in its (different) param
// management dependency. Physics content is a deliberate, scoped subset of what GPTireUKF has:
//   - dynamic/effective tire radius (load- and speed-dependent), GPTireUKF vehicle_model.cpp:243-262
//   - longitudinal (pitch) load transfer via z_cog_m, GPTireUKF vehicle_model.cpp:354-359
//   - MF_Simple tire model: unchanged from the v1 SSA-UKF (byte-identical formula)
// Explicitly NOT ported: GP tire model, MF5.2, lateral/roll load transfer (absent in GPTireUKF
// too), limited-slip-differential torque split (GPTireUKF's exact formula wasn't fully verified
// against source before this fork -- left out rather than risk a mis-ported term; see
// docs/versions/v2_draft/direction.md).
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace tam::core::ssa::vehicle_model_v2
{

struct VehicleModelParams
{
  double mass_kg{800.0};
  double Izz_kgm2{1000.0};
  double l_front_m{1.724};
  double l_rear_m{1.247};
  double tw_front_m{1.639};
  double tw_rear_m{1.524};
  double rho_air_kgpm3{1.22};
  double cL_front{-0.7};
  double cL_rear{-0.7};

  // NEW: longitudinal (pitch) load transfer. z_cog_m has no measured value anywhere in this
  // project as of this fork -- 0.5m is GPTireUKF's own default (a similar open-wheel race
  // platform), carried over as a placeholder, not a calibrated number. See open_issues.md.
  double z_cog_m{0.5};

  // brake force model (unchanged from v1 SSA-UKF)
  double d_brake_bore_front_m{0.0798};
  double d_brake_bore_rear_m{0.0798};
  double r_brake_pads_lever_front_m{0.1493};
  double r_brake_pads_lever_rear_m{0.1493};
  double mue_brakes_front{0.55};
  double mue_brakes_rear{0.55};

  // NEW: dynamic/effective tire radius. r_tire_unloaded_* defaults to the same value the v1
  // SSA-UKF used as its single fixed radius (0.2999/0.3120m) since no separate free-radius
  // measurement exists; load_stiffness/speed_expansion defaults are GPTireUKF's own
  // (vehicle_model.cpp:79-99), also uncalibrated for this vehicle.
  double rtire_unloaded_front_m{0.2999};
  double rtire_unloaded_rear_m{0.3120};
  double tire_load_stiffness_front_Npm{200000.0};
  double tire_load_stiffness_rear_Npm{200000.0};
  double tire_speed_expansion_front_mpradps2{2.3e-7};
  double tire_speed_expansion_rear_mpradps2{2.8e-7};

  // Magic-Formula-Simple coefficients [B, C, D, E], unchanged from v1
  Eigen::Vector<double, 4> tire_MF_long_front{20.0, 1.8, 1.65, 1.0};
  Eigen::Vector<double, 4> tire_MF_long_rear{20.0, 1.8, 1.70, 1.0};
  Eigen::Vector<double, 4> tire_MF_lat_front{22.0, 2.2, 1.60, 1.0};
  Eigen::Vector<double, 4> tire_MF_lat_rear{20.0, 2.0, 1.65, 1.0};
};

/**
 * @brief Dynamic/effective rolling radius: static-loaded deflection (1/3 weight) blended with
 *        free-rolling centrifugal growth (2/3 weight) -- ported from GPTireUKF
 *        vehicle_model.cpp:243-262 (`calc_dyn_tire_radius`).
 *        r_dyn = r_unloaded - (Fz/k_load)/3 + (2/3)*(omega^2 * c_speed_expansion)
 *        v1 SSA-UKF used a single fixed r_tire_front/rear_m constant everywhere instead.
 */
inline double calc_dyn_tire_radius(
  double r_unloaded_m, double load_N, double load_stiffness_Npm,
  double wheelspeed_radps, double speed_expansion_coeff)
{
  const double static_deflection_m = load_N / load_stiffness_Npm;
  const double centrifugal_growth_m = wheelspeed_radps * wheelspeed_radps * speed_expansion_coeff;
  return r_unloaded_m - static_deflection_m / 3.0 + (2.0 / 3.0) * centrifugal_growth_m;
}

/**
 * @brief Simplified Magic Formula tire model -- unchanged formula from v1 SSA-UKF's MF_Simple.
 */
inline double mf_simple(
  double slip, double load, const Eigen::Ref<const Eigen::Vector<double, 4>> & tire_params)
{
  return load * tire_params[2] * std::sin(tire_params[1] * std::atan(tire_params[0] *
    slip - tire_params[3] * (tire_params[0] * slip - std::atan(tire_params[0] * slip))));
}

/**
 * @brief Per-wheel longitudinal slip ratio and lateral slip angle from a candidate (V, beta)
 *        state, using the DYNAMIC tire radius (load- and speed-dependent) instead of a fixed
 *        constant -- same kinematic chain as v1 SSA-UKF's calc_tire_states, only the radius
 *        source changed. load_tire_N is the PREVIOUS cycle's load estimate (one-cycle lag,
 *        load and radius are mutually dependent -- resolving that circularity exactly would
 *        need an inner iteration; this fork accepts the lag as GPTireUKF-equivalent practice).
 */
inline Eigen::Matrix<double, 4, 2> calc_tire_states(
  double vel_mps, double beta_rad, double delta_rad, double dpsi_radps,
  const Eigen::Vector<double, 4> & wheelspeeds_radps,
  const Eigen::Vector<double, 4> & load_tire_N,
  const VehicleModelParams & p, double vel_min_mps)
{
  const double vel_cog_x_mps = vel_mps * std::cos(beta_rad);
  const double vel_cog_y_mps = vel_mps * std::sin(beta_rad);

  Eigen::Vector<double, 4> transform_vx = {
    -p.tw_front_m / 2.0, p.tw_front_m / 2.0, -p.tw_rear_m / 2.0, p.tw_rear_m / 2.0};
  Eigen::Vector<double, 4> transform_vy = {p.l_front_m, p.l_front_m, -p.l_rear_m, -p.l_rear_m};
  Eigen::Vector<double, 4> vel_tire_x_mps =
    dpsi_radps * transform_vx + Eigen::VectorXd::Constant(4, vel_cog_x_mps);
  Eigen::Vector<double, 4> vel_tire_y_mps =
    dpsi_radps * transform_vy + Eigen::VectorXd::Constant(4, vel_cog_y_mps);

  Eigen::Vector<double, 4> delta_matrix = {delta_rad, delta_rad, 0.0, 0.0};
  Eigen::Vector<double, 4> vel_tire_T_x_mps =
    delta_matrix.array().cos() * vel_tire_x_mps.array() +
    delta_matrix.array().sin() * vel_tire_y_mps.array();
  Eigen::Vector<double, 4> vel_tire_T_y_mps =
    -delta_matrix.array().sin() * vel_tire_x_mps.array() +
    delta_matrix.array().cos() * vel_tire_y_mps.array();

  Eigen::Vector<double, 4> tire_radius_m;
  tire_radius_m[0] = calc_dyn_tire_radius(
    p.rtire_unloaded_front_m, load_tire_N[0], p.tire_load_stiffness_front_Npm,
    wheelspeeds_radps[0], p.tire_speed_expansion_front_mpradps2);
  tire_radius_m[1] = calc_dyn_tire_radius(
    p.rtire_unloaded_front_m, load_tire_N[1], p.tire_load_stiffness_front_Npm,
    wheelspeeds_radps[1], p.tire_speed_expansion_front_mpradps2);
  tire_radius_m[2] = calc_dyn_tire_radius(
    p.rtire_unloaded_rear_m, load_tire_N[2], p.tire_load_stiffness_rear_Npm,
    wheelspeeds_radps[2], p.tire_speed_expansion_rear_mpradps2);
  tire_radius_m[3] = calc_dyn_tire_radius(
    p.rtire_unloaded_rear_m, load_tire_N[3], p.tire_load_stiffness_rear_Npm,
    wheelspeeds_radps[3], p.tire_speed_expansion_rear_mpradps2);

  Eigen::Matrix<double, 4, 2> slip_tire_perc;
  for (int i = 0; i < 4; ++i) {
    slip_tire_perc(i, 0) =
      (wheelspeeds_radps[i] * tire_radius_m[i] - vel_tire_T_x_mps[i]) /
      (vel_tire_T_x_mps[i] + 1e-15);
    if (vel_tire_T_x_mps[i] < vel_min_mps) {
      slip_tire_perc(i, 0) = std::clamp(slip_tire_perc(i, 0), -0.01, 0.01);
    } else {
      slip_tire_perc(i, 0) = std::clamp(slip_tire_perc(i, 0), -1.0, 1.0);
    }
    slip_tire_perc(i, 1) = -std::atan2(vel_tire_T_y_mps[i], vel_tire_T_x_mps[i]);
  }
  return slip_tire_perc;
}

/**
 * @brief Vertical tire load per wheel: static configuration split + aero downforce (unchanged
 *        from v1) PLUS a longitudinal (pitch) load-transfer term (NEW, from GPTireUKF
 *        vehicle_model.cpp:354-359). Still no lateral/roll transfer -- left/right split stays
 *        a hard 50/50, matching both v1 and GPTireUKF.
 */
inline Eigen::Vector<double, 4> calc_tire_loads(
  double vel_cog_x_mps, double ax_mps2, const VehicleModelParams & p)
{
  const double l_m = p.l_front_m + p.l_rear_m;
  const double load_mass_front_N = p.mass_kg * 9.80665 * p.l_rear_m / l_m;
  const double load_mass_rear_N = p.mass_kg * 9.80665 * p.l_front_m / l_m;
  const double load_aero_front_N =
    -0.5 * p.rho_air_kgpm3 * p.cL_front * vel_cog_x_mps * vel_cog_x_mps;
  const double load_aero_rear_N =
    -0.5 * p.rho_air_kgpm3 * p.cL_rear * vel_cog_x_mps * vel_cog_x_mps;
  // NEW: ax>0 (accelerating) shifts load to the rear, braking shifts it to the front
  const double load_shift_pitch_N = p.z_cog_m * ax_mps2 * p.mass_kg / l_m;

  Eigen::Vector<double, 4> load_tire_N;
  load_tire_N[0] = 0.5 * (load_mass_front_N + load_aero_front_N - load_shift_pitch_N);
  load_tire_N[1] = 0.5 * (load_mass_front_N + load_aero_front_N - load_shift_pitch_N);
  load_tire_N[2] = 0.5 * (load_mass_rear_N + load_aero_rear_N + load_shift_pitch_N);
  load_tire_N[3] = 0.5 * (load_mass_rear_N + load_aero_rear_N + load_shift_pitch_N);
  return load_tire_N;
}

/**
 * @brief Front/rear axle forces (long, lat, vert) from per-wheel slip/load, via MF_Simple --
 *        unchanged formula and axle-lumping (front/rear average, no per-wheel L/R split) from
 *        v1 SSA-UKF's calc_axle_forces.
 */
inline Eigen::Vector<double, 6> calc_axle_forces(
  const Eigen::Vector<double, 4> & slip_tire_long_perc,
  const Eigen::Vector<double, 4> & slip_tire_lat_perc,
  const Eigen::Vector<double, 4> & load_tire_N, const VehicleModelParams & p)
{
  Eigen::Vector<double, 6> axle_forces;
  const double load_axle_front_N = load_tire_N[0] + load_tire_N[1];
  const double load_axle_rear_N = load_tire_N[2] + load_tire_N[3];
  const double slip_axle_front_long = 0.5 * (slip_tire_long_perc[0] + slip_tire_long_perc[1]);
  const double slip_axle_rear_long = 0.5 * (slip_tire_long_perc[2] + slip_tire_long_perc[3]);
  const double slip_axle_front_lat = 0.5 * (slip_tire_lat_perc[0] + slip_tire_lat_perc[1]);
  const double slip_axle_rear_lat = 0.5 * (slip_tire_lat_perc[2] + slip_tire_lat_perc[3]);

  axle_forces[0] = 2.0 * mf_simple(slip_axle_front_long, load_axle_front_N / 2.0, p.tire_MF_long_front);
  axle_forces[1] = 2.0 * mf_simple(slip_axle_front_lat, load_axle_front_N / 2.0, p.tire_MF_lat_front);
  axle_forces[2] = load_axle_front_N;
  axle_forces[3] = 2.0 * mf_simple(slip_axle_rear_long, load_axle_rear_N / 2.0, p.tire_MF_long_rear);
  axle_forces[4] = 2.0 * mf_simple(slip_axle_rear_lat, load_axle_rear_N / 2.0, p.tire_MF_lat_rear);
  axle_forces[5] = load_axle_rear_N;
  return axle_forces;
}

/**
 * @brief "Evidence"-side (model-free) axle forces for the virtual-measurement path: brake
 *        pressure and drivetrain torque -> longitudinal force at the tire contact patch
 *        (unchanged from v1); lateral force split from whole-vehicle force/yaw-moment balance.
 *        Aero drag and rolling resistance are deliberately NOT subtracted: wheel torque / r is
 *        already the contact-patch force, and drag acts on the body, not in the tire-force
 *        balance. Deducting it (tried in an earlier v2 draft) made the evidence a net-body force
 *        that the MF tire model does not predict, and the mismatch grows with v^2 -- it drove the
 *        UKF speed state off after ~41 s of vegas24_pub (docs/TODO_Advisor_Action_Items.md,
 *        Task 1a). Drag would only be ADDED if the evidence came from body acceleration
 *        (F_tire = m*ax + F_drag + F_roll).
 *        Front axle is brake-only (RWD, not driven).
 *        NOT ported: GPTireUKF's limited-slip-differential torque split -- see file header.
 * @param[out] {FTX_front_N, FTX_rear_N, FTY_front_N, FTY_rear_N}
 */
inline Eigen::Vector<double, 4> calc_axle_forces_evidence(
  double delta_rad, double ay_mps2, double yaw_acc_radps2,
  double drivetrain_trq_Nm, double brake_pressure_fl_pa, double brake_pressure_fr_pa,
  double brake_pressure_rl_pa, double brake_pressure_rr_pa, const VehicleModelParams & p)
{
  const double l_m = p.l_front_m + p.l_rear_m;
  Eigen::Vector<double, 4> ftx_fty;

  ftx_fty[0] = -(2.0 * p.mue_brakes_front *
    (brake_pressure_fl_pa + brake_pressure_fr_pa) / 2.0 *
    (M_PI * p.d_brake_bore_front_m * p.d_brake_bore_front_m *
     p.r_brake_pads_lever_front_m / 4.0)) / p.rtire_unloaded_front_m;

  const double rear_brake_force_N = -(2.0 * p.mue_brakes_rear *
    (brake_pressure_rl_pa + brake_pressure_rr_pa) / 2.0 *
    (M_PI * p.d_brake_bore_rear_m * p.d_brake_bore_rear_m *
     p.r_brake_pads_lever_rear_m / 4.0));
  const double drive_force_N = drivetrain_trq_Nm / p.rtire_unloaded_rear_m;
  ftx_fty[1] = rear_brake_force_N + drive_force_N;

  const double fy_cog_N = ay_mps2 * p.mass_kg;
  const double mzz_cog_Nm = yaw_acc_radps2 * p.Izz_kgm2;
  ftx_fty[2] = (1.0 / std::cos(delta_rad)) *
    ((fy_cog_N * p.l_rear_m + mzz_cog_Nm) / l_m - std::sin(delta_rad) * ftx_fty[0]);
  ftx_fty[3] = (fy_cog_N * p.l_front_m - mzz_cog_Nm) / l_m;
  return ftx_fty;
}

}  // namespace tam::core::ssa::vehicle_model_v2

// Copyright 2026
#pragma once

#include <cmath>

namespace tam::core::friction
{

// Standard gravity. Tier 1 uses the static value rather than a live vertical-accel
// measurement -- see design notes for why that refinement is deferred.
constexpr double kGravity_mps2 = 9.80665;

struct Tier1VehicleParams
{
  double mass_kg;
  double rho_air_kgpm3;
  double cL_front;
  double cL_rear;
};

struct Tier1Output
{
  double mu_demand{0.0};
  double fz_total_n{0.0};
};

/**
 * @brief Aero-corrected total vertical load Fz(v) = m*g + F_aero(v)
 *        Mirrors the sign convention in ssa_estimation_cpp's calc_tire_loads:
 *        cL is negative for downforce, so the aero term adds to the static load.
 */
inline double compute_fz_total_n(double v_mps, const Tier1VehicleParams & params)
{
  const double f_aero_n =
    -0.5 * params.rho_air_kgpm3 * (params.cL_front + params.cL_rear) * v_mps * v_mps;
  return params.mass_kg * kGravity_mps2 + f_aero_n;
}

/**
 * @brief Tier 1 kinematic friction demand: mu_demand = m*sqrt(ax^2+ay^2) / Fz(v)
 *        Always computable -- valid at every cycle, but only equals the true grip
 *        limit at the moment the car is actually at the limit.
 */
inline Tier1Output compute_tier1(
  double a_lat_mps2, double a_long_mps2, double v_mps, const Tier1VehicleParams & params)
{
  Tier1Output out;
  out.fz_total_n = compute_fz_total_n(v_mps, params);
  const double demand_n =
    params.mass_kg * std::sqrt(a_lat_mps2 * a_lat_mps2 + a_long_mps2 * a_long_mps2);
  // guard against a near-zero or negative Fz (e.g. before params are populated)
  out.mu_demand = (out.fz_total_n > 1.0) ? demand_n / out.fz_total_n : 0.0;
  return out;
}

}  // namespace tam::core::friction

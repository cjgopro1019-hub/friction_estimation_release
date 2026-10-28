// Copyright 2026
#pragma once

#include <cmath>

namespace tam::core::friction
{

/**
 * @brief Cumulative slip-energy integral S = integral(|F * v_slip|) dt, and the
 *        resulting wear decay factor eta_wear(S) = exp(-k*S).
 *
 *        This is a plain running integral of already-computed quantities (force, slip
 *        velocity), no gate needed -- it accumulates every cycle regardless of whether
 *        the tire is currently saturated. Deliberately vy-independent: the slip
 *        velocity input is a longitudinal-slip proxy (kappa * v_wheel, from
 *        DataQualityGating's wheel-speed-based slip ratio), not a lateral-slip
 *        quantity derived from side-slip angle -- matches the rest of this module
 *        family's decision to never depend on beta. A true combined-slip (long+lat)
 *        version is future work once/if per-axle force decomposition is available.
 */
class WearModel
{
public:
  WearModel() : WearModel(0.0) {}
  explicit WearModel(double energy_coefficient) : k_(energy_coefficient) {}

  /**
   * @param demand_force_n     measured combined tire force magnitude (N), e.g. m*sqrt(ax^2+ay^2)
   * @param slip_velocity_mps  longitudinal slip velocity proxy, max|kappa|*v_wheel (m/s)
   * @param dt_s                cycle time (s)
   */
  void step(double demand_force_n, double slip_velocity_mps, double dt_s)
  {
    cumulative_energy_j_ += std::abs(demand_force_n * slip_velocity_mps) * dt_s;
    eta_wear_ = std::exp(-k_ * cumulative_energy_j_);
  }

  double cumulative_energy_j() const { return cumulative_energy_j_; }
  double eta_wear() const { return eta_wear_; }

private:
  double k_;
  double cumulative_energy_j_{0.0};
  double eta_wear_{1.0};
};

}  // namespace tam::core::friction

// Copyright 2026
#pragma once

#include "friction_estimation_cpp/tier1_friction_demand.hpp"

namespace tam::core::friction
{

/**
 * @brief Kinematic friction-demand estimator: mu_demand = m*sqrt(ax^2+ay^2) / Fz(v).
 *
 *        This class deliberately answers one question only -- "how much of the tire's
 *        capability is being used right now" -- and makes no claim about how much is
 *        AVAILABLE. That split is the core architectural decision this module family
 *        implements: demand (this class, always valid, vy-independent) vs. potential
 *        (FrictionPotentialTracker, only updated under sufficient excitation). Treating
 *        an instantaneous demand sample as the true grip ceiling is exactly the mistake
 *        this split exists to prevent -- under low excitation, mu_demand is only ever a
 *        lower bound on the true limit, never a measurement of it.
 */
class FrictionDemandEstimator
{
public:
  FrictionDemandEstimator() = default;
  explicit FrictionDemandEstimator(const Tier1VehicleParams & params) : params_(params) {}

  Tier1Output step(double a_lat_mps2, double a_long_mps2, double v_mps) const
  {
    return compute_tier1(a_lat_mps2, a_long_mps2, v_mps, params_);
  }

  void set_params(const Tier1VehicleParams & params) { params_ = params; }
  const Tier1VehicleParams & params() const { return params_; }

private:
  Tier1VehicleParams params_;
};

}  // namespace tam::core::friction

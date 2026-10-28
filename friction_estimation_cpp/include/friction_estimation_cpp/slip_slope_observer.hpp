// Copyright 2026
#pragma once

#include <algorithm>
#include <cmath>

namespace tam::core::friction
{

struct SlipSlopeObserverParams
{
  // Exponential forgetting per 10ms cycle -- same philosophy as
  // FrictionPotentialTracker's decay, not a fixed-window average, so this keeps
  // adapting if tires/track condition change mid-session instead of anchoring to
  // whatever the first few minutes looked like.
  double forgetting_factor{0.999};
  // Effective (forgetting-weighted) sample count needed for full sample confidence.
  double min_samples_for_confidence{200.0};
  // max|kappa| that has to have been observed so far for full coverage confidence --
  // this is the knob that keeps a fresh vehicle/session conservative: on vegas24_pub
  // this axle observer barely reaches this even once across an 86.5s stint (see
  // research/slip_slope_analysis.py), so confidence should sit near zero almost the
  // entire time -- that is the intended, safe behavior, not a bug.
  double min_kappa_coverage{0.02};
  // |mu_x - predicted_mu_x| below this counts as "consistent with the online fit".
  double consistency_tolerance{0.05};
};

struct SlipSlopeResult
{
  double slope{0.0};
  double predicted_mu_x{0.0};
  double residual{0.0};
  double confidence{0.0};   // [0,1] -- sample_confidence * coverage_confidence
  bool consistent{false};
};

/**
 * @brief Online, self-calibrating observer of the small-slip relationship between one
 *        axle's mean wheel slip ratio (kappa) and whole-vehicle longitudinal
 *        utilization (mu_x = Fx_total/Fz_total). Deliberately NOT a pre-tuned
 *        per-vehicle coefficient loaded from a config table -- it starts at zero
 *        confidence on a fresh vehicle/session and only grows confident as real
 *        (kappa, mu_x) evidence actually accumulates on whatever vehicle is currently
 *        running. This is what makes it usable across vehicle types (this codebase has
 *        only ever seen one RWD-biased race car's data -- see
 *        research/slip_slope_analysis.py -- and the next dataset expected is a
 *        structurally different AWD production sedan, where a hardcoded "trust the
 *        rear axle" assumption would be wrong) without needing a per-model
 *        recalibration step: run one instance per axle, let each axle's own online
 *        confidence reveal which axle(s) are actually informative on that vehicle.
 *
 *        Diagnostic-only for now -- not wired into excitation_score or
 *        potential_confidence yet, matching this project's practice of validating a
 *        new signal against real data before making it load-bearing.
 */
class SlipSlopeObserver
{
public:
  SlipSlopeObserver() = default;
  explicit SlipSlopeObserver(const SlipSlopeObserverParams & params) : params_(params) {}

  SlipSlopeResult step(double kappa, double mu_x)
  {
    // Forgetting-factor-weighted accumulators for a 1-parameter (slope-through-origin)
    // least-squares fit: minimizing sum(w_i * (mu_x_i - slope*kappa_i)^2) gives
    // slope = sum(w*kappa*mu_x) / sum(w*kappa^2). Intercept is intentionally omitted --
    // mu_x=0 at kappa=0 is a physical constraint (no slip, no longitudinal force), not
    // a free parameter, so fitting only the slope avoids the origin-region noise
    // fitting analysis found (R^2 collapses to ~0.06 in the smallest-slip quartile).
    sum_kk_ = params_.forgetting_factor * sum_kk_ + kappa * kappa;
    sum_kmu_ = params_.forgetting_factor * sum_kmu_ + kappa * mu_x;
    effective_n_ = params_.forgetting_factor * effective_n_ + 1.0;
    // Decaying max, same forgetting factor as everything else here -- NOT a
    // monotonic running max. A monotonic max would let one early spike latch full
    // coverage confidence for the rest of the session even after the slope estimate
    // itself has drifted back into low-speed noise once that evidence ages out of
    // sum_kk_/sum_kmu_'s own memory -- confirmed happening in validation against
    // vegas24_pub (confidence pinned at 1.0 through the back half of the session while
    // the slope had drifted to a small, physically-implausible ~-0.27, far from the
    // +5..+13 range actually measured near real excitation events). Decaying this on
    // the same timescale keeps "confident" meaning "recently corroborated", matching
    // sum_kk_/sum_kmu_'s own memory span.
    max_abs_kappa_seen_ = std::max(
      params_.forgetting_factor * max_abs_kappa_seen_, std::abs(kappa));

    SlipSlopeResult out;
    out.slope = (sum_kk_ > 1e-9) ? sum_kmu_ / sum_kk_ : 0.0;
    out.predicted_mu_x = out.slope * kappa;
    out.residual = mu_x - out.predicted_mu_x;

    const double sample_confidence = std::clamp(
      effective_n_ / params_.min_samples_for_confidence, 0.0, 1.0);
    const double coverage_confidence = std::clamp(
      max_abs_kappa_seen_ / params_.min_kappa_coverage, 0.0, 1.0);
    out.confidence = sample_confidence * coverage_confidence;
    out.consistent = std::abs(out.residual) < params_.consistency_tolerance;
    return out;
  }

  double max_abs_kappa_seen() const { return max_abs_kappa_seen_; }

private:
  SlipSlopeObserverParams params_;
  double sum_kk_{0.0};
  double sum_kmu_{0.0};
  double effective_n_{0.0};
  double max_abs_kappa_seen_{0.0};
};

}  // namespace tam::core::friction

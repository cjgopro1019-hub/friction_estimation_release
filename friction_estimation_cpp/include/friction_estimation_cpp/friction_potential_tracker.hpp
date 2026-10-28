// Copyright 2026
#pragma once

#include <algorithm>
#include <cmath>

namespace tam::core::friction
{

struct FrictionPotentialTrackerParams
{
  // ExcitationResult.score must reach this before a DECAY toward a lower observed
  // demand is allowed -- see decay_rate_per_s below. Rising to match/exceed a higher
  // observed demand is NOT gated by this any more (see step()): mu_demand is always a
  // rigorous lower bound on the true mu_max, so mu_potential falling below it is a
  // proven contradiction, not weak evidence that needs excitation to act on.
  double excitation_threshold{0.5};

  // Decay toward a newly-observed LOWER demand: continuous-time rate constant (1/s).
  // This is what replaces the old monotonic running maximum -- without this, moving
  // from a high-mu surface to a low-mu one would leave mu_potential locked at the
  // stale high value forever. Deliberately slow and still excitation-gated: a single
  // low-demand cycle (e.g. a straight) must not be read as "the road got worse".
  double decay_rate_per_s{0.05};

  // Confidence decays exponentially with time since the last confirming cycle (either
  // an excitation-qualified decay-direction update, or a hard-floor correction); this
  // is the time constant (s). Set so confidence is already negligible (~5%) after 10s
  // without a fresh confirmation -- tau = 10/ln(20).
  double confidence_decay_tau_s{3.3};

  // Lowered from a previous, evidence-free 0.9 guess: 0.5 sits within this project's
  // own observed mu_demand range, so the hard floor below actually gets exercised by
  // real data through the session instead of the estimate sitting stranded near an
  // unjustified high starting value for the whole run (see open_issues.md).
  double init{0.5};
  double min{0.1};
  // Hard physical ceiling: no passenger/race tire on asphalt is expected to exceed
  // roughly this combined-mu envelope -- clamps against a runaway estimate from a
  // bad sample surviving every other guard above.
  double max{1.3};

  // Soft confidence boost from an independent corroborating signal (see
  // slip_slope_observer.hpp) -- how much of the remaining gap to full confidence
  // (1.0) can be closed when that signal is fully available this cycle. This is
  // NOT a gate on whether mu_potential updates. It can only RAISE confidence, never
  // lower it: a still-learning or momentarily-inconsistent axle model must not be
  // able to undermine trust in a base estimate that already stands on its own
  // (time-since-last-confirmation). Default 0.3 keeps this a genuine "nudge", not
  // something that can single-handedly manufacture full confidence out of a stale
  // estimate.
  double corroboration_weight{0.3};
};

struct FrictionPotentialResult
{
  double mu_potential{0.0};
  double confidence{0.0};  // [0,1]
};

/**
 * @brief Tracks the tire/road friction potential mu_max as a decaying, re-confirmable
 *        estimate -- deliberately NOT a monotonic running maximum. A raw running max
 *        (the old PeakExtractor/mu_demand_peak behavior) locks onto the highest demand
 *        ever observed and can never come back down, which is wrong the moment the
 *        vehicle moves from a high-mu surface onto a lower-mu one.
 *
 *        Two distinct mechanisms move this estimate, treated very differently:
 *        - UPWARD correction to at least mu_demand is a HARD, UNCONDITIONAL,
 *          IMMEDIATE floor (data_valid only, no excitation gate) -- mu_demand is
 *          always a rigorous lower bound on the true mu_max, so mu_potential ever
 *          reading below it is a proven contradiction, not an inference that needs
 *          strong evidence to act on. There is no "waiting for enough excitation" to
 *          fix a value that is already known to be wrong.
 *        - DOWNWARD movement toward a newly lower observed demand is the opposite:
 *          genuinely ambiguous (a quiet moment doesn't prove the limit dropped), so it
 *          stays a slow, excitation-gated forgetting-factor pull, never a snap.
 *
 *        Confidence decays with time since the last confirming cycle (either
 *        direction), so a stale estimate is visibly flagged as such rather than
 *        silently trusted -- optionally nudged upward by an independent corroborating
 *        signal (see `corroboration_weight` above), the same "two independent
 *        evidence paths" idiom this vehicle's own SSA-UKF virtual measurement already
 *        uses.
 */
class FrictionPotentialTracker
{
public:
  FrictionPotentialTracker() : FrictionPotentialTracker(FrictionPotentialTrackerParams{}) {}
  explicit FrictionPotentialTracker(const FrictionPotentialTrackerParams & params)
  : params_(params), mu_potential_(params.init) {}

  // slip_slope_corroboration in [0,1]: how strongly an independent path (the online
  // per-axle kappa-vs-mu_x observer) both trusts itself AND agrees with this cycle's
  // mu_demand -- 0.0 (the default) when no such corroboration is available, which
  // makes this call behave identically to the pre-corroboration version.
  FrictionPotentialResult step(
    double mu_demand, double excitation_score, bool data_valid, double dt_s,
    double slip_slope_corroboration = 0.0)
  {
    bool confirmed_this_cycle = false;

    // Hard floor: correct a provably-too-low estimate immediately, regardless of
    // excitation. Still gated on data_valid -- a physically-invalid sample (e.g.
    // during wheel lockup) is not trusted as a clean demand measurement either.
    if (data_valid && mu_demand > mu_potential_) {
      mu_potential_ = std::clamp(mu_demand, params_.min, params_.max);
      confirmed_this_cycle = true;
    }

    // Decay: only toward a lower observed demand, and only when excitation says this
    // sample is informative enough to trust as evidence the limit may have dropped.
    const bool qualifies = data_valid && excitation_score >= params_.excitation_threshold;
    if (qualifies && mu_demand < mu_potential_) {
      const double alpha = 1.0 - std::exp(-params_.decay_rate_per_s * dt_s);
      mu_potential_ += alpha * (mu_demand - mu_potential_);
      mu_potential_ = std::clamp(mu_potential_, params_.min, params_.max);
      confirmed_this_cycle = true;
    }

    time_since_confirm_s_ = confirmed_this_cycle ? 0.0 : (time_since_confirm_s_ + dt_s);

    const double base_confidence =
      std::exp(-time_since_confirm_s_ / params_.confidence_decay_tau_s);
    const double corroboration = std::clamp(slip_slope_corroboration, 0.0, 1.0);
    // pulls partway toward 1.0, never past it, never below base_confidence -- a
    // pure no-op when corroboration is 0
    const double boosted_confidence = base_confidence +
      (1.0 - base_confidence) * params_.corroboration_weight * corroboration;

    FrictionPotentialResult out;
    out.mu_potential = mu_potential_;
    out.confidence = boosted_confidence;
    return out;
  }

  double mu_potential() const { return mu_potential_; }

private:
  FrictionPotentialTrackerParams params_;
  double mu_potential_;
  // starts effectively "never confirmed" so confidence is ~0 until the first real
  // confirming sample arrives
  double time_since_confirm_s_{1.0e6};
};

}  // namespace tam::core::friction

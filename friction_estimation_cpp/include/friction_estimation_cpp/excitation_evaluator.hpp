// Copyright 2026
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace tam::core::friction
{

// [B, C, D, E] Magic-Formula-Simple coefficients, same order/convention as
// P_VDC_tire_MF_long_front etc. in the SSA-UKF configs.
using MfCoeffs = std::array<double, 4>;

struct ExcitationEvaluatorParams
{
  MfCoeffs mf_long_front{20.00, 1.80, 1.65, 1.00};
  MfCoeffs mf_long_rear{20.00, 1.80, 1.70, 1.00};
  MfCoeffs mf_lat_front{22.00, 2.20, 1.60, 1.00};
  MfCoeffs mf_lat_rear{20.00, 2.00, 1.65, 1.00};

  double mass_kg{800.0};
  double izz_kgm2{1000.0};
  double l_front_m{1.724};
  double l_rear_m{1.247};
};

struct ExcitationResult
{
  double score{0.0};                    // = max(kappa_util, lat_force_util) -- see class doc
  double kappa_util_component{0.0};     // max over axles of the longitudinal stiffness-degradation info
  double lat_force_util_component{0.0}; // max over axles of the lateral stiffness-degradation info
  double demand_util_component{0.0};    // mu_demand / mu_potential (previous cycle) -- diagnostic only, NOT part of score, see class doc
  double fy_front_n{0.0};               // raw per-axle lateral force estimate, exposed for debug
  double fy_rear_n{0.0};
};

/**
 * @brief Scores how INFORMATIVE the current sample is about the tire's true friction
 *        limit. Reframed 2026-09-28 (advisor feedback) from "how dynamically strong is
 *        this moment" to "how close is each axle to tire saturation": a dynamically
 *        strong moment (large ay, large mu_demand) can still sit deep in the tire's
 *        LINEAR region, where force is set by cornering/longitudinal stiffness and is
 *        nearly insensitive to mu (dF/dmu ~ 0 there); only once an axle nears or passes
 *        its peak slip does force start to depend sensitively on mu (dF/dmu ~ F/mu on
 *        the saturated branch) -- that is when a sample actually constrains mu_max.
 *
 *        `kappa_util`/`lat_force_util` are each a UTILIZATION RATIO (observed force /
 *        the peak the tire model predicts is available for that axle), not a raw signal
 *        magnitude -- each approaches 1 near saturation regardless of how it was
 *        reached, and stays low for a dynamically busy but still-linear-region sample.
 *        The lateral term needs only each axle's MF peak value D (not B or C), so it
 *        does not depend on an unvalidated nominal cornering stiffness the way a
 *        linear-bicycle-model comparison would.
 *
 *        score = max(kappa_util, lat_force_util) -- 2026-09-30 revision, replacing an
 *        earlier weighted sum of these two plus a third `demand_util` term
 *        (mu_demand/mu_potential). Changed for two independent reasons, both found by
 *        the Task 3 sensitivity/ablation analysis (see TODO_Advisor_Action_Items.md):
 *        (1) `demand_util` correlated 0.94 with `lat_force_util` on the test drive
 *        (near-redundant) and is self-referential -- it compares mu_demand against
 *        mu_potential, which is itself built FROM mu_demand's own recent peaks via the
 *        hard floor in friction_potential_tracker.hpp, so a high value partly reflects
 *        "close to a number this project set from its own recent demand" rather than
 *        independent evidence of saturation. Dropped from the score; still computed and
 *        returned (demand_util_component) as a diagnostic, available for a future
 *        corroboration-style use (see slip_slope_corroboration's existing pattern in
 *        friction_estimation_node.cpp) but not gating the primary decision.
 *        (2) A weighted SUM structurally dilutes a sample that is saturated on only one
 *        axis: e.g. kappa_util=1.0, lat_force_util=0.0 (pure-longitudinal saturation)
 *        scored only 0.35 under the old 0.35/0.35/0.30 weights, even though the tire is
 *        genuinely at its physical limit in that direction. This is also why the old
 *        version's output was so sensitive to the exact weights chosen (Task 3: up to
 *        0.126 RMSE in the resulting mu_potential trajectory across a reasonable weight
 *        sweep) -- max() has no such free parameter to mis-set, and a sample that
 *        saturates either axis is scored accordingly regardless of the other axis.
 *
 *        Deliberately vy/beta-independent, like the rest of this module family (see
 *        friction_estimation_node.hpp): the lateral term recovers per-axle lateral
 *        force from the lateral and yaw equations of motion (m*ay = Fy_front+Fy_rear,
 *        Izz*r_dot = l_front*Fy_front-l_rear*Fy_rear) -- exact for any tire behavior,
 *        linear or saturated, and needs only ay, r and its time derivative, not a
 *        beta/vy estimate.
 *
 *        TRIED AND REVERTED (2026-10-01): replacing the linear ratio above with a
 *        stiffness-degradation measure info(x) = 1 - (local slope at x)/(slope at x=0),
 *        derived from the same MF_Simple curve (info(0)=0, info(x_peak)=1, verified
 *        numerically to suppress the sub-peak range far more aggressively than the
 *        linear ratio -- e.g. longitudinal at 10/50/80% of kappa_peak: 0.036/0.583/0.897
 *        vs the ratio's 0.10/0.50/0.80 -- and to resolve the post-peak non-monotonicity
 *        gap noted below as a side effect). Physically more defensible in the abstract,
 *        but empirically WORSE against the independent Kistler reference on this
 *        dataset: corr dropped 0.922->0.681 (Pearson), 0.922->0.801 (Spearman), top-5%-
 *        highest-score overlap with Kistler's own top-5%-highest-demand instants dropped
 *        52.4%->41.2%. Root cause: `vegas24_pub` rarely approaches true tire saturation
 *        (confirmed again here: 0 samples with |kappa|>=kappa_peak over the full
 *        replay), so info(x)'s correctly-conservative near-zero region swallows nearly
 *        all of this dataset's actual driving-intensity variation, discarding exactly
 *        the sub-peak signal that correlates with demand -- the old linear ratio tracks
 *        better here only because it is closer to a monotonic rescaling of the raw
 *        physical quantities (kappa, y) that already correlate with instantaneous
 *        demand. Kept as a documented negative result, not deleted: see
 *        TODO_Advisor_Action_Items.md for the full numbers and the open question of
 *        whether a dataset with genuine near-limit segments would favor info(x) instead.
 */
class ExcitationEvaluator
{
public:
  ExcitationEvaluator() { recompute_kappa_peaks(); }
  explicit ExcitationEvaluator(const ExcitationEvaluatorParams & params) : params_(params)
  {
    recompute_kappa_peaks();
  }

  ExcitationResult step(
    double mu_demand, double mu_potential_prev, double a_lat_mps2, double r_dot_radps2,
    double fz_front_n, double fz_rear_n, double kappa_front, double kappa_rear) const
  {
    ExcitationResult out;

    // --- longitudinal: |kappa| / kappa_peak per axle -- kappa_peak is the actual slip
    // ratio at which each axle's MF_Simple longitudinal curve peaks, solved once in
    // recompute_kappa_peaks(), not re-derived per cycle ---
    const double kappa_util_front = (kappa_peak_front_ > 1e-9)
      ? std::abs(kappa_front) / kappa_peak_front_ : 0.0;
    const double kappa_util_rear = (kappa_peak_rear_ > 1e-9)
      ? std::abs(kappa_rear) / kappa_peak_rear_ : 0.0;
    out.kappa_util_component = std::clamp(std::max(kappa_util_front, kappa_util_rear), 0.0, 1.0);

    // --- lateral: solve Fy_front/Fy_rear exactly from the lateral+yaw equations of
    // motion (no linear-tire assumption baked in -- this is just Newton's 2nd law and
    // the yaw moment balance), then express each as a fraction of that axle's
    // MF_Simple PEAK lateral force (D * Fz) -- i.e. how much of the axle's available
    // lateral capacity the current maneuver is actually using ---
    const double l_total_m = params_.l_front_m + params_.l_rear_m;
    if (l_total_m > 1e-6) {
      out.fy_front_n = (params_.mass_kg * a_lat_mps2 * params_.l_rear_m +
        params_.izz_kgm2 * r_dot_radps2) / l_total_m;
      out.fy_rear_n = (params_.mass_kg * a_lat_mps2 * params_.l_front_m -
        params_.izz_kgm2 * r_dot_radps2) / l_total_m;
    }
    const double fy_peak_front_n = params_.mf_lat_front[2] * fz_front_n;
    const double fy_peak_rear_n = params_.mf_lat_rear[2] * fz_rear_n;
    const double lat_util_front = (fy_peak_front_n > 1.0)
      ? std::abs(out.fy_front_n) / fy_peak_front_n : 0.0;
    const double lat_util_rear = (fy_peak_rear_n > 1.0)
      ? std::abs(out.fy_rear_n) / fy_peak_rear_n : 0.0;
    out.lat_force_util_component =
      std::clamp(std::max(lat_util_front, lat_util_rear), 0.0, 1.0);

    // --- whole-vehicle force utilization: how much of the PREVIOUS cycle's believed
    // available grip (mu_potential) the CURRENT demand is using. The one-cycle lag is
    // intentional, not an oversight -- mu_potential this cycle is itself a function of
    // this cycle's excitation score (see friction_potential_tracker.hpp), so it cannot
    // also be an input to that same score without the lag (same pattern as
    // vehicle_model.hpp's load_tire_N feeding calc_dyn_tire_radius one cycle behind).
    // Diagnostic-only -- NOT part of score, see class doc. ---
    out.demand_util_component = (mu_potential_prev > 1e-3)
      ? std::clamp(mu_demand / mu_potential_prev, 0.0, 1.0) : 0.0;

    out.score = std::clamp(
      std::max(out.kappa_util_component, out.lat_force_util_component), 0.0, 1.0);
    return out;
  }

private:
  // Peak location of MF_Simple's curve F(x) = D*sin(C*atan(B*x - E*(B*x - atan(B*x)))).
  // dF/dx = D*C*cos(C*u(x))*du/dx, u(x) = B*x*(1-E) + E*atan(B*x). du/dx =
  // B*[(1-E) + E/(1+(B*x)^2)] is strictly positive for E in [0,1], so the first
  // positive root of dF/dx is where cos(C*u(x)) = 0, i.e. u(x) = pi/(2C). Solved once
  // per construction via Newton's method on g(x) = u(x) - pi/(2C) -- exact in one step
  // for the E=1 case used throughout this project's configs (u(x) reduces to
  // atan(B*x), closed form x_peak = tan(pi/(2C))/B), and still correct, just numeric,
  // for any other E in [0,1].
  static double solve_peak_slip(const MfCoeffs & mf)
  {
    const double b = mf[0], c = mf[1], e = mf[3];
    if (b < 1e-9 || c < 1e-9) {
      return 0.0;
    }
    const double target = std::acos(-1.0) / (2.0 * c);
    double x = 1.0 / b;  // initial guess: the slip at which B*x = 1 rad
    for (int i = 0; i < 50; ++i) {
      const double bx = b * x;
      const double atan_bx = std::atan(bx);
      const double u = bx * (1.0 - e) + e * atan_bx;
      const double du_dx = b * ((1.0 - e) + e / (1.0 + bx * bx));
      if (du_dx < 1e-12) {
        break;
      }
      const double newton_step = (u - target) / du_dx;
      x -= newton_step;
      if (std::abs(newton_step) < 1e-10) {
        break;
      }
    }
    return std::abs(x);
  }

  void recompute_kappa_peaks()
  {
    kappa_peak_front_ = solve_peak_slip(params_.mf_long_front);
    kappa_peak_rear_ = solve_peak_slip(params_.mf_long_rear);
  }

  ExcitationEvaluatorParams params_;
  double kappa_peak_front_{0.0};
  double kappa_peak_rear_{0.0};
};

}  // namespace tam::core::friction

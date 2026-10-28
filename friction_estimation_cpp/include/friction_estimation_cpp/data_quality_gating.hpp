// Copyright 2026
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

namespace tam::core::friction
{

struct WheelSpeeds
{
  double fl{0.0};
  double fr{0.0};
  double rl{0.0};
  double rr{0.0};
};

inline double wheel_slip_ratio(double omega_radps, double r_tire_m, double v_wheel_mps)
{
  if (std::abs(v_wheel_mps) < 1e-3) {
    return 0.0;
  }
  return (omega_radps * r_tire_m - v_wheel_mps) / v_wheel_mps;
}

struct DataQualityGatingParams
{
  // Wheel lockup / pure spin: this only excludes NEAR-TOTAL loss of correlation between
  // wheel speed and ground speed (a locked or freely-spinning wheel), not merely "high
  // slip". High slip in the 10-40% range is exactly the informative region
  // ExcitationEvaluator rewards -- it must not be thrown away here. Default is
  // deliberately much higher than the old peak_validity_gates.hpp's 0.25 exclusion
  // threshold, which excluded exactly the near-limit region this redesign wants to keep.
  double wheel_lockout_threshold{0.6};
  double track_width_front_m{1.639};
  double track_width_rear_m{1.524};
  double tire_radius_front_m{0.2958};
  double tire_radius_rear_m{0.3099};

  // ABS/TC proxy: this vehicle's data has no direct ABS/TC status flag, so this
  // approximates intervention from rapid brake-pressure oscillation (sign changes in
  // dP/dt within a short rolling window). First-pass heuristic, not a validated
  // detector -- this dataset (vegas24_pub) has never shown a confirmed genuine ABS
  // event (see project notes), so these defaults are unvalidated against a true
  // positive. Revisit once a dataset with confirmed ABS/TC intervention is available.
  double brake_pressure_rate_threshold_pa_per_s{2.0e6};
  int abs_oscillation_window_samples{20};
  int abs_oscillation_min_sign_changes{4};
};

struct GateResult
{
  bool valid{true};
  bool wheel_lockup{false};
  bool abs_tc_active{false};
  bool sensor_invalid{false};
};

/**
 * @brief Excludes a sample from friction-potential tracking only when it is genuinely
 *        NOT information -- physically invalid, or off the tire model entirely (wheel
 *        lockup/spin, suspected ABS/TC intervention, non-finite/out-of-range sensor
 *        data). Deliberately does NOT exclude high slip ratio, high yaw-rate mismatch,
 *        or high mu_demand on their own -- those are the most informative operating
 *        points and are scored (not punished) by ExcitationEvaluator. This supersedes
 *        peak_validity_gates.hpp's SlipRatioGate/YawMismatchGate/PostPeakDropoffGate,
 *        which treated near-limit operation itself as a reason to reject a sample.
 */
class DataQualityGating
{
public:
  DataQualityGating() = default;
  explicit DataQualityGating(const DataQualityGatingParams & params) : params_(params) {}

  GateResult step(
    double vx_mps, double r_radps, const WheelSpeeds & omega,
    double brake_pressure_fl_pa, double brake_pressure_fr_pa,
    double brake_pressure_rl_pa, double brake_pressure_rr_pa,
    bool inputs_finite_and_in_range, double dt_s)
  {
    GateResult out;
    out.sensor_invalid = !inputs_finite_and_in_range;

    last_max_abs_kappa_ = max_abs_kappa(vx_mps, r_radps, omega);
    out.wheel_lockup = last_max_abs_kappa_ > params_.wheel_lockout_threshold;

    {
      const double v_fl = vx_mps - r_radps * (params_.track_width_front_m / 2.0);
      const double v_fr = vx_mps + r_radps * (params_.track_width_front_m / 2.0);
      const double v_rl = vx_mps - r_radps * (params_.track_width_rear_m / 2.0);
      const double v_rr = vx_mps + r_radps * (params_.track_width_rear_m / 2.0);
      last_kappa_front_ = 0.5 * (
        wheel_slip_ratio(omega.fl, params_.tire_radius_front_m, v_fl) +
        wheel_slip_ratio(omega.fr, params_.tire_radius_front_m, v_fr));
      last_kappa_rear_ = 0.5 * (
        wheel_slip_ratio(omega.rl, params_.tire_radius_rear_m, v_rl) +
        wheel_slip_ratio(omega.rr, params_.tire_radius_rear_m, v_rr));
    }

    out.abs_tc_active = update_abs_proxy(
      brake_pressure_fl_pa + brake_pressure_fr_pa + brake_pressure_rl_pa + brake_pressure_rr_pa,
      dt_s);

    out.valid = !out.sensor_invalid && !out.wheel_lockup && !out.abs_tc_active;
    return out;
  }

  double max_abs_kappa(double vx_mps, double r_radps, const WheelSpeeds & omega) const
  {
    const double v_fl = vx_mps - r_radps * (params_.track_width_front_m / 2.0);
    const double v_fr = vx_mps + r_radps * (params_.track_width_front_m / 2.0);
    const double v_rl = vx_mps - r_radps * (params_.track_width_rear_m / 2.0);
    const double v_rr = vx_mps + r_radps * (params_.track_width_rear_m / 2.0);
    const double k_fl = wheel_slip_ratio(omega.fl, params_.tire_radius_front_m, v_fl);
    const double k_fr = wheel_slip_ratio(omega.fr, params_.tire_radius_front_m, v_fr);
    const double k_rl = wheel_slip_ratio(omega.rl, params_.tire_radius_rear_m, v_rl);
    const double k_rr = wheel_slip_ratio(omega.rr, params_.tire_radius_rear_m, v_rr);
    return std::max({std::abs(k_fl), std::abs(k_fr), std::abs(k_rl), std::abs(k_rr)});
  }

  // last max|kappa| computed by step(), exposed so the node doesn't need to recompute
  // it a second time when feeding ExcitationEvaluator
  double last_max_abs_kappa() const { return last_max_abs_kappa_; }

  // per-axle mean SIGNED kappa (not abs) computed by step(), for SlipSlopeObserver --
  // sign carries the slip direction (accel vs. braking), which max_abs_kappa() discards
  double last_kappa_front() const { return last_kappa_front_; }
  double last_kappa_rear() const { return last_kappa_rear_; }

private:
  bool update_abs_proxy(double total_brake_pressure_pa, double dt_s)
  {
    if (dt_s < 1e-6) {
      return last_abs_state_;
    }
    const double d_pressure_dt = (total_brake_pressure_pa - prev_brake_pressure_pa_) / dt_s;
    prev_brake_pressure_pa_ = total_brake_pressure_pa;

    const bool rising = d_pressure_dt > params_.brake_pressure_rate_threshold_pa_per_s;
    const bool falling = d_pressure_dt < -params_.brake_pressure_rate_threshold_pa_per_s;
    const int sign = rising ? 1 : (falling ? -1 : 0);
    const bool is_sign_change = (sign != 0 && sign != last_sign_ && last_sign_ != 0);
    sign_change_history_.push_back(is_sign_change);
    if (sign != 0) {
      last_sign_ = sign;
    }
    while (static_cast<int>(sign_change_history_.size()) > params_.abs_oscillation_window_samples) {
      sign_change_history_.pop_front();
    }
    const int changes = static_cast<int>(
      std::count(sign_change_history_.begin(), sign_change_history_.end(), true));
    last_abs_state_ = changes >= params_.abs_oscillation_min_sign_changes;
    return last_abs_state_;
  }

  DataQualityGatingParams params_;
  double prev_brake_pressure_pa_{0.0};
  int last_sign_{0};
  std::deque<bool> sign_change_history_;
  bool last_abs_state_{false};
  double last_max_abs_kappa_{0.0};
  double last_kappa_front_{0.0};
  double last_kappa_rear_{0.0};
};

}  // namespace tam::core::friction

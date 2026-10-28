// Copyright 2026
#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace tam::core::friction
{

/**
 * @brief Tracks whether a topic has produced a message within the configured timeout.
 *        Same zero-order-hold contract as the rest of the node: staleness is judged at
 *        read time (in the 10ms tick), not pushed by the callback.
 */
class StalenessTracker
{
public:
  void touch(const rclcpp::Time & now) { last_update_time_ = now; has_received_ = true; }

  bool is_stale(const rclcpp::Time & now, double timeout_s) const
  {
    if (!has_received_) {
      return true;
    }
    return (now - last_update_time_).seconds() > timeout_s;
  }

private:
  rclcpp::Time last_update_time_;
  bool has_received_{false};
};

/**
 * @brief Rejects a scalar sample that is either outside a physically plausible absolute
 *        range, or jumps further than physically possible from the last accepted sample
 *        (e.g. the Kistler yaw-rate spike found in vegas24: a multi-hundred-deg/s jump
 *        that reverts within one cycle). Rejected samples are simply dropped -- the
 *        caller keeps whatever value it already had (zero-order hold).
 *
 *        Escape hatch: if a *plausible* (in-range) candidate keeps getting rejected for
 *        max_consecutive_rejects cycles in a row, resync to it instead of staying locked
 *        out forever. Found by real testing: a single large but real jump right at
 *        cold-start (e.g. the EKF's v output climbing fast while it converges) can latch
 *        last_good_value_ at a stale value that the real signal never returns near,
 *        permanently rejecting every future sample -- N consecutive plausible rejects is
 *        far more likely to mean the reference itself is stale than N consecutive
 *        physical anomalies.
 */
class PlausibilityGuard
{
public:
  PlausibilityGuard() = default;
  PlausibilityGuard(
    double max_abs_value, double max_delta_per_cycle, int max_consecutive_rejects = 20)
  : max_abs_value_(max_abs_value), max_delta_per_cycle_(max_delta_per_cycle),
    max_consecutive_rejects_(max_consecutive_rejects) {}

  bool check(double candidate)
  {
    if (!std::isfinite(candidate) || std::abs(candidate) > max_abs_value_) {
      consecutive_rejects_ = 0;
      return false;
    }
    const bool jumped =
      has_value_ && std::abs(candidate - last_good_value_) > max_delta_per_cycle_;
    if (jumped) {
      ++consecutive_rejects_;
      if (consecutive_rejects_ < max_consecutive_rejects_) {
        return false;
      }
    }
    last_good_value_ = candidate;
    has_value_ = true;
    consecutive_rejects_ = 0;
    return true;
  }

private:
  double max_abs_value_{0.0};
  double max_delta_per_cycle_{0.0};
  int max_consecutive_rejects_{20};
  double last_good_value_{0.0};
  bool has_value_{false};
  int consecutive_rejects_{0};
};

}  // namespace tam::core::friction

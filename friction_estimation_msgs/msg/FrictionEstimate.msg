# Output of the excitation-aware friction demand/potential estimator.
# See the friction_estimation_cpp design notes for the derivation of every field below.
#
# Architecture: demand (mu_demand, always valid, vy-independent) is decoupled from
# potential (mu_potential, only updated under sufficient excitation -- see
# ExcitationEvaluator/FrictionPotentialTracker/DataQualityGating). Under low excitation,
# mu_demand is only ever a LOWER BOUND on the true grip limit, never a measurement of it.

std_msgs/Header header

# --- core result ---
float64 mu_eff              # mu_potential * eta_wear(S) -- the headline "grip available now" number
float64 mu_potential        # tracked friction potential -- forgetting-factor updated, NOT a
                             # monotonic running max; decays toward lower observed demand so a
                             # surface change (high-mu -> low-mu) is actually reflected
float64 mu_demand           # instantaneous kinematic demand, always populated, vy-independent

float64 potential_confidence   # [0,1] -- decays with time since the last excitation-qualified,
                                # data-quality-valid confirmation of mu_potential
float64 excitation_score       # [0,1] -- how informative *this* cycle's sample currently is
                                # about the true limit (high slip / high yaw-rate model mismatch /
                                # high demand all raise this -- they are not exclusion criteria)

# --- friction circle ---
float64 a_lat_mps2      # body-frame lateral accel, from the 3D-EKF
float64 a_long_mps2     # body-frame longitudinal accel, from the 3D-EKF
float64 grip_limit_mps2 # mu_eff * g -- circle radius, precomputed

# --- diagnostics ---
float64 eta_wear                    # wear decay factor, [0, 1]
float64 slip_energy_cumulative_j    # cumulative slip-power integral S
float64 fz_front_n
float64 fz_rear_n

bool data_quality_valid   # false if this cycle's sample was excluded from potential tracking
bool wheel_lockup         # true if any wheel is near-locked/spinning -- exclusion reason
bool abs_tc_active        # true if ABS/TC intervention is suspected -- exclusion reason
                           # (first-pass brake-pressure-oscillation heuristic, see design notes)

int8 drs_state           # meaningful only when P_VDC_has_drs is true; -1 otherwise
uint32 input_health       # bitmask -- which inputs are currently stale or outlier-rejected

# --- slip-slope observers (diagnostic only -- NOT fed into mu_potential/excitation_score) ---
# Online, self-calibrating per-axle observers of mu_x=Fx_total/Fz_total vs. mean wheel
# slip ratio. No assumption baked in about which axle is driven -- each axle's own
# confidence (0 until real evidence accumulates on THIS vehicle/session) reveals which
# axle(s) are actually informative. See slip_slope_observer.hpp design notes.
float64 slip_slope_front
float64 slip_slope_front_confidence
bool slip_slope_front_consistent
float64 slip_slope_rear
float64 slip_slope_rear_confidence
bool slip_slope_rear_consistent

# --- v2 axle-force-evidence demand (second, partially-independent estimate) ---
# mu_demand (above) comes from the 3D-EKF's fused a_lat/a_long -- an inertial-only
# estimate. mu_demand_axle_evidence is computed a structurally different way: from the
# v2 SSA-UKF's own virtual axle-force measurements (brake/drivetrain torque -> Fx per
# axle, plus aero drag/rolling-resistance corrections; Fy per axle from the tire model)
# divided by the v2 UKF's own dynamic tire loads (including CG-height longitudinal load
# transfer). It shares the same IMU ay input as mu_demand only through the UKF's
# internal state update, not directly -- so the two are corroborating, not identical,
# evidence for the same physical quantity. See ssa_estimation_v2_cpp design notes.
float64 mu_demand_axle_evidence
bool axle_evidence_valid   # false if the v2 SSA-UKF debug stream is absent/stale --
                           # distinguishes "genuinely near-zero demand" from "v2 not running"

# friction_estimation

Excitation-aware friction demand/potential estimator for the vehicle dynamics state
estimation stack.

- **Friction demand** (`mu_demand`, `FrictionDemandEstimator`): kinematic utilization, computed
  every cycle from body-frame acceleration and aero-corrected vertical load
  (`mu_demand = m*sqrt(ax^2+ay^2) / Fz(v)`). Always valid, vy-independent, but never itself the
  friction *limit* -- under low excitation it is only ever a lower bound on the true potential.
- **Friction potential** (`mu_potential`, `FrictionPotentialTracker`): the tracked road-mu ceiling.
  A forgetting-factor estimator, **not** a monotonic running maximum -- it rises quickly toward a
  newly-observed higher demand but decays toward a lower one, so a road-surface change (high-mu to
  low-mu) is actually reflected instead of staying locked at a stale peak. Only updates while
  `ExcitationEvaluator` judges the current sample informative enough and `DataQualityGating`
  judges it physically valid; publishes a `potential_confidence` that decays with time since the
  last qualifying update.
- **Excitation evaluator** (`ExcitationEvaluator`): scores how *informative* the current sample is
  about the true limit -- the inverse framing from an earlier design of this repo, which treated
  high slip ratio and high yaw-rate model mismatch as reasons to *exclude* a sample. Near the
  limit is exactly where a tire's nonlinear behavior reveals itself; this class rewards that
  instead of filtering it out.
- **Data quality gating** (`DataQualityGating`): excludes a sample from potential tracking only
  when it is genuinely not information -- wheel lockup/pure spin, suspected ABS/TC intervention
  (first-pass brake-pressure-oscillation heuristic, no ABS/TC status signal exists in this
  dataset), or non-finite/out-of-range sensor data. Deliberately does not exclude high-slip or
  high-yaw-mismatch samples on their own.
- **Slip-slope observers** (`SlipSlopeObserver`, one per axle): online, self-calibrating
  observers of the small-slip relationship between each axle's wheel slip ratio and
  whole-vehicle longitudinal utilization -- deliberately not a pre-tuned coefficient (see
  Status below for why). Feed a soft, boost-only corroboration into `potential_confidence`
  when an axle's online model both trusts itself and agrees with the current sample; never
  gate whether `mu_potential` updates.
- **eta_wear**: cumulative slip-energy decay factor; `mu_eff = mu_potential * eta_wear`. The
  slip-velocity term feeding it is a wheelspeed-derived longitudinal-slip proxy, not a
  beta-derived lateral one -- see `wear_model.hpp`. **`P_VDC_wear_energy_coefficient` is
  currently an unresolved placeholder, not a working calibration -- treat `eta_wear`/`mu_eff`
  as unvalidated** (see Status below).

This entire module family is deliberately **vy/side-slip-angle independent** -- no signal here
ever depends on beta. Confirmed via `research/vy_fusion_observer.py` and this repo's own earlier
SSA-UKF-beta-gated design: beta is not trustworthy on this vehicle's sensor suite (no dedicated
sideslip sensor), so nothing in the demand/potential/excitation/gating chain is allowed to need it.

Packages:
- `friction_estimation_msgs` — `FrictionEstimate.msg` interface, no algorithm code.
- `friction_estimation_cpp` — the estimator node (`friction_estimation_node`), publishing
  `/core/friction/estimate` on a 10ms timer, plus a launch file that brings up the whole
  estimation stack (state_publisher + 3D-EKF + SSA-UKF + this node) with one command. (SSA-UKF is
  still launched because the 3D-EKF's velocity-fusion path depends on it being alive and reporting
  `OK`, not because this node consumes its output -- see design notes.)

`research/` holds algorithm exploration that didn't make it into the shipped node (currently:
a complementary-filter side-slip/vy observer, kept for potential future use — see its own header
for status).

## Dependency: 3DVehicleDynamicsStateEstimation

This package needs `state_estimation_node_cpp`, `ssa_estimation_node_cpp`, `state_publisher`,
`msgs`, and their transitive deps built as sibling packages in the same colcon workspace
(`~/ros2_ws/src/`). Clone **this fork**, not the TUM upstream:

```bash
git clone git@github.com:cjgopro1019-hub/3DVehicleDynamicsStateEstimation.git
```

The fork carries fixes on top of `TUMFTM/3DVehicleDynamicsStateEstimation` that this package
actually depends on for a clean replay — cloning upstream instead will reproduce the bugs:
- Two NaN fixes (3D-EKF Kalman gain, SSA-UKF sigma points) — see its README's
  "Numerical Stability Fixes" section.
- A silent (no NaN, no error) EKF divergence: if every configured IMU input times out at once
  (confirmed trigger: simply reaching the end of a bag replay, when the IMU topic stops
  publishing), the node used to keep predicting forward with a zero input vector, integrating
  an un-opposed gravity term as a fictitious "free fall" that corrupts vx/vy/vz/position within
  seconds and never self-recovers. Fixed by stopping the model update timer instead once that
  happens, publishing the last good state as final.

## Build

```bash
colcon build --symlink-install --packages-select friction_estimation_msgs friction_estimation_cpp
source install/setup.bash
```

## Run — one command, real-time bag replay

```bash
ros2 launch friction_estimation_cpp friction_estimation_demo.launch.py \
  bag_path:=/path/to/vegas24_pub
```

This starts `state_publisher`, the 3D-EKF, the SSA-UKF, and `friction_estimation_node`, waits
`bag_start_delay_s` (default 4s — the EKF/SSA cold-start needs to finish before the first bag
messages arrive, or the EKF can diverge; confirmed by testing), then plays the bag at real-time
(1x) rate. Omit `bag_path` to bring the stack up against live/manually-published sensor data
instead — every node here talks over the same standard topic names either way, so nothing about
the estimation code changes when this moves onto the real car.

Config file locations default to `~/ros2_ws/src/3DVehicleDynamicsStateEstimation/config/...`
(the fork above, checked out at that path); override with `ekf_config_path` / `ssa_config_path`
/ `friction_config_path` if your layout differs.

### Run manually (equivalent, for debugging one node at a time)

```bash
ros2 launch state_publisher state_publisher.py
ros2 run state_estimation_node_cpp state_estimation_node_cpp_ekf_3d_kin --ros-args \
  --params-file /path/to/state_estimation_cpp_3d_ekf_config.yml
ros2 run ssa_estimation_node_cpp ssa_estimation_node --ros-args \
  --params-file /path/to/ssa_estimation_ukf_config.yml
ros2 run friction_estimation_cpp friction_estimation_node --ros-args \
  --params-file friction_estimation_cpp/config/friction_estimation_config.yml
ros2 bag play /path/to/vegas24_pub
```

## Live GUI dashboard

```bash
python3 gui/friction_dashboard.py [--window-s 30]
```

Plain `rclpy` + Tkinter/matplotlib, no extra dependencies beyond a desktop ROS2 install.
Subscribes to `/core/friction/estimate` and `/core/state/odometry` only — it has no idea
whether the data is coming from a bag or a live car, so it needs no changes to move from replay
to on-track use. Shows: mu_demand/mu_potential/mu_eff time series, a friction-circle scatter
(colored by data-quality validity), and a status line with excitation_score, potential_confidence,
exclusion reasons (wheel lockup / suspected ABS-TC), and input_health.

## Status

**Architecture redesign (2026-08-18, branch `excitation-aware-friction-potential`)**: replaced the
earlier two-tier design (Tier 1 demand + beta-gated Tier 2 `mu_track` KF + hard-exclusion peak
gates) with the demand/potential split described above. The earlier design's peak-validity gates
treated near-limit operation itself as a reason to reject a sample, and `mu_track` depended on
SSA-UKF's beta for both its gate and its force-model comparison -- both are now retired in favor
of an excitation-scored, vy-independent potential tracker. Validated against a real 84s
vegas24_pub replay: 0 NaN end to end, `data_quality_valid` stays true almost the entire run
(consistent with earlier findings that this dataset never contains a genuine
lockup/spin/ABS event), `mu_potential`/`excitation_score`/`potential_confidence` all move
smoothly and sanely. Not yet validated against a genuine surface-change or loss-of-control event
(no such event exists in this dataset) -- the decay/forgetting behavior and the ABS/TC proxy in
particular remain first-pass, unvalidated designs pending a dataset that actually exercises them.

Known limitations / next steps:
- `mu_potential` and `eta_wear` are still whole-vehicle proxies (per-axle split is future work).
- **`eta_wear`'s coefficient (`P_VDC_wear_energy_coefficient`) is an unresolved placeholder,
  not a working calibration.** A recalibration attempt (2026-08-19) against the current
  longitudinal-slip S basis was deliberately abandoned mid-investigation: a fixed
  coefficient tuned against one vehicle's data (this project has only ever had
  vegas24_pub) can't be assumed to generalize to a structurally different vehicle -- the
  same objection already acted on for slip-slope, which is why that became an online,
  self-calibrating observer (`SlipSlopeObserver`) instead of a pre-tuned number. Wear
  modeling needs the same treatment (online, confidence-gated, learned per
  vehicle/session) before it's trustworthy -- not yet designed. Until then, treat
  `eta_wear`/`mu_eff` as unvalidated.
- The ABS/TC detector is a brake-pressure-oscillation heuristic with no ground truth to validate
  against in this dataset.
- Longitudinal slip-slope (tire stiffness from the small-slip region) is not yet implemented.
  The required empirical Fx/Fz-vs-kappa analysis has now been run (`research/slip_slope_analysis.py`,
  validated against a full 86.5s vegas24_pub replay, n=7711): rear-axle kappa shows a real,
  statistically significant relationship with longitudinal utilization (R^2=0.35 pooled,
  0.25-0.41 split by braking/driving) and qualifies as an auxiliary excitation/confidence
  feature candidate; front-axle kappa does not (R^2=0.02-0.03, effectively noise on this
  vehicle). Caveats: this only characterizes the narrow near-origin slip band this dataset
  actually visits (kappa never exceeds ~0.04 -- consistent with vegas24_pub never containing a
  genuine near-limit/lockup event), and it's a whole-vehicle-proxy relationship (single
  Fx_total against each axle's own kappa), not a true per-axle force/slip curve. Actually
  wiring kappa_rear into `ExcitationEvaluator` as a weighted term is a deliberate separate
  next step, not yet done.
- **Slip-slope observer added as a diagnostic module (2026-08-19)**: `SlipSlopeObserver`
  (`slip_slope_observer.hpp`) is an online, self-calibrating per-axle observer of
  mu_x-vs-kappa -- deliberately NOT a pre-tuned coefficient, since the only vehicle data
  available is this one RWD-biased car and the next dataset expected is a structurally
  different AWD sedan (a hardcoded "trust the rear axle" assumption would be wrong there).
  It runs one instance per axle with zero prior confidence; confidence only rises as real
  (kappa, mu_x) evidence actually accumulates on whatever vehicle is currently running, and
  decays on the same forgetting timescale as everything else here once that evidence goes
  stale -- so which axle(s) are informative is discovered live per vehicle, not assumed in
  config. `friction_estimation_node` runs both axle observers every cycle (gated on
  `data_quality_valid` and vx>3m/s) and publishes `slip_slope_front/rear` +
  `_confidence`/`_consistent` -- diagnostic only, not yet wired into
  `excitation_score`/`potential_confidence`. Validated against a full vegas24_pub replay
  (0 NaN, n~8100): rear confidence averages 0.59 (54% of the session above 0.5, peaking at
  1.0 near real high-slip events), front averages 0.33 and never exceeds 0.53 -- consistent
  with the offline analysis above without hardcoding it. Two bugs found and fixed during
  this validation: (1) the published confidence field was flickering to 0 on any cycle
  `data_quality_valid` was false instead of holding the observer's actual persisted state;
  (2) the coverage term was a monotonic running max with no decay, so one early high-slip
  event could latch full confidence for the rest of a session even after the slope estimate
  itself had drifted back into low-speed noise -- fixed by decaying it on the same
  timescale as the rest of the observer's state.
- **Slip-slope wired in as a `potential_confidence` corroboration boost (2026-08-19)**:
  chosen over the alternative of adding it as a 4th weighted term inside
  `ExcitationEvaluator` -- that alternative was rejected because it mixes two different
  timescales (a slowly-built, forgetting-factor confidence vs. the other three
  components' per-instant informativeness) and would have diluted already-validated
  weights for a term that's usually near-zero. Instead `FrictionPotentialTracker::step()`
  takes an extra `slip_slope_corroboration` argument (`max` over axles of
  `confidence * consistent`, computed in the node from the observers above) and uses it
  only to nudge `potential_confidence` toward 1.0
  (`confidence += (1-confidence) * P_VDC_potential_corroboration_weight * corroboration`,
  weight default 0.3) -- it is NOT a gate on whether `mu_potential` itself updates
  (`excitation_score >= threshold` is still the only thing that decides that), and it can
  only raise confidence, never lower it, so a still-learning or momentarily-inconsistent
  axle model can never undermine trust in a base estimate that already stands on its own.
  Mirrors the same "two independent evidence paths corroborate each other" idiom this
  vehicle's own SSA-UKF virtual measurement already uses. Validated against a full
  vegas24_pub replay: 0 NaN, `potential_confidence` stays within [0.10, 1.00] (never
  exceeds 1.0), `mu_potential`'s own update trajectory (range, event timing) is
  unchanged from before this change -- confirming the boost is isolated to the
  confidence output only.
- **Start/stop gating added (2026-08-24)**: `friction_estimation_node` previously kept
  computing forever once started, regardless of whether its inputs were still live --
  before the first real message it ran on default-initialized zeros, and after the data
  source (bag or live vehicle) stopped it kept publishing on frozen zero-order-held
  state indefinitely (the wear-model investigation and a real GUI export both surfaced
  this directly: a flat `excitation_score` and smoothly-decaying `potential_confidence`
  tail after the bag ended). Fixed the same way `state_estimation_node` was fixed for
  its own IMU inputs: `model_update_callback()` now checks all six tracked inputs
  (odometry, acceleration, imu, steering, wheelspeed, brake_pressure) every cycle --
  computation doesn't start until every one has published at least once (`all_fresh`),
  and `model_update_timer_->cancel()` fires the moment every one has gone simultaneously
  stale (`all_stale`), requiring ALL six on both ends so a single-topic staleness blip
  can't spuriously start or stop the node. Validated: no messages published before real
  data arrives (confirmed via `ros2 topic echo --once` timing out with nothing
  received); across two runs (1230 + 2021 messages, 0 NaN) the first published message's
  timestamp exactly matches the "All inputs are live" log line, and the stop fires
  93ms after `state_estimation_node`'s own IMU-timeout halt -- a clean, fast cascade
  (bag ends -> EKF stops -> friction node's inputs go stale -> friction node stops too)
  with no phantom accumulation window in between.
- Tire temperature and DRS are config-gated stubs (`P_VDC_enable_tire_temp_model`,
  `P_VDC_has_drs`), off by default — no hardware for either yet.

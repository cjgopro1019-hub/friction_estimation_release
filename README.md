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

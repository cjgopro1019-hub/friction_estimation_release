#!/usr/bin/env python3
"""Real-time dashboard for /core/friction/estimate.

Plain rclpy + Tkinter/matplotlib -- no extra deps beyond what a desktop ROS2
install already ships. Subscribes to standard topic names only, so it runs
identically against `ros2 bag play` (real-time rate) today and against the
live car later: nothing here knows or cares whether the data is coming from
a bag or real sensors.

Usage (after sourcing the workspace install/setup.bash):
    python3 gui/friction_dashboard.py [--window-s 30] [--export-dir DIR]

Click "Export Session" any time (including after the bag has finished) to
dump the FULL session recorded so far -- not just the rolling display
window -- as a timestamped folder containing the raw data (CSV) and
full-time-axis charts (PNG).
"""
import argparse
import csv
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
from matplotlib.widgets import Button  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from friction_estimation_msgs.msg import FrictionEstimate  # noqa: E402

# categorical palette (project convention): mu_demand / mu_potential / mu_eff / excitation
COLOR_DEMAND = "#2a78d6"
COLOR_POTENTIAL = "#eb6834"
COLOR_EFF = "#1baf7a"
COLOR_EXCITATION = "#eda100"
COLOR_INVALID = "#d64545"
COLOR_AXLE_EVIDENCE = "#c264e0"
COLOR_BG = "#14181f"
COLOR_PANEL = "#1c222c"
COLOR_GRID = "#2a323f"
COLOR_TEXT = "#e6e9ef"
COLOR_MUTED = "#8992a6"

HEALTH_BIT_NAMES = [
    (1 << 0, "ODOM_STALE"), (1 << 1, "ACCEL_STALE"), (1 << 2, "V_REJECTED"),
    (1 << 3, "ALONG_REJECTED"), (1 << 4, "ALAT_REJECTED"),
    (1 << 7, "IMU_STALE"), (1 << 8, "STEERING_STALE"),
    (1 << 9, "WHEELSPEED_STALE"), (1 << 10, "BRAKE_PRESSURE_STALE"),
]

# every field written to the exported CSV, in column order
HISTORY_FIELDS = [
    "t", "t_epoch", "mu_demand", "mu_demand_axle_evidence", "axle_evidence_valid",
    "mu_potential", "mu_eff", "potential_confidence",
    "excitation_score", "a_lat_mps2", "a_long_mps2", "grip_limit_mps2",
    "eta_wear", "slip_energy_cumulative_j", "fz_front_n", "fz_rear_n",
    "data_quality_valid", "wheel_lockup", "abs_tc_active", "input_health",
    "speed_mps",
]


class FrictionListener(Node):
    def __init__(self, window_s: float):
        super().__init__("friction_dashboard")
        self.lock = threading.Lock()
        self.window_s = window_s
        maxlen = int(window_s * 150)  # generous headroom above the node's 100Hz publish rate
        self.t = deque(maxlen=maxlen)
        self.mu_demand = deque(maxlen=maxlen)
        self.mu_demand_axle_evidence = deque(maxlen=maxlen)
        self.mu_potential = deque(maxlen=maxlen)
        self.mu_eff = deque(maxlen=maxlen)
        self.a_lat = deque(maxlen=maxlen)
        self.a_long = deque(maxlen=maxlen)
        self.valid = deque(maxlen=maxlen)
        self.t0 = None
        self.latest = None
        self.speed_mps = 0.0

        # unbounded, whole-session record -- separate from the rolling display
        # buffers above, so "export" always has the full run regardless of
        # --window-s. At 100Hz a multi-minute session is still only a few MB.
        self.history = []

        self.create_subscription(
            FrictionEstimate, "/core/friction/estimate", self._friction_cb, 50)
        self.create_subscription(
            Odometry, "/core/state/odometry", self._odom_cb, 5)

    def _friction_cb(self, msg: FrictionEstimate):
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = stamp_s
            rel_t = stamp_s - self.t0
            self.t.append(rel_t)
            self.mu_demand.append(msg.mu_demand)
            self.mu_demand_axle_evidence.append(
                msg.mu_demand_axle_evidence if msg.axle_evidence_valid else float("nan"))
            self.mu_potential.append(msg.mu_potential)
            self.mu_eff.append(msg.mu_eff)
            self.a_lat.append(msg.a_lat_mps2)
            self.a_long.append(msg.a_long_mps2)
            self.valid.append(bool(msg.data_quality_valid))
            self.latest = msg
            self.history.append({
                "t": rel_t,
                # header.stamp verbatim (not t0-relative): under use_sim_time this equals
                # the bag's own original recording epoch, so an offline reconstruction from
                # the same bag can be joined directly on this column instead of needing a
                # cross-correlation search to find the two datasets' relative lag.
                "t_epoch": stamp_s,
                "mu_demand": msg.mu_demand,
                "mu_demand_axle_evidence": msg.mu_demand_axle_evidence,
                "axle_evidence_valid": bool(msg.axle_evidence_valid),
                "mu_potential": msg.mu_potential,
                "mu_eff": msg.mu_eff,
                "potential_confidence": msg.potential_confidence,
                "excitation_score": msg.excitation_score,
                "a_lat_mps2": msg.a_lat_mps2,
                "a_long_mps2": msg.a_long_mps2,
                "grip_limit_mps2": msg.grip_limit_mps2,
                "eta_wear": msg.eta_wear,
                "slip_energy_cumulative_j": msg.slip_energy_cumulative_j,
                "fz_front_n": msg.fz_front_n,
                "fz_rear_n": msg.fz_rear_n,
                "data_quality_valid": bool(msg.data_quality_valid),
                "wheel_lockup": bool(msg.wheel_lockup),
                "abs_tc_active": bool(msg.abs_tc_active),
                "input_health": msg.input_health,
                "speed_mps": self.speed_mps,
            })

    def _odom_cb(self, msg: Odometry):
        with self.lock:
            self.speed_mps = msg.twist.twist.linear.x

    def snapshot(self):
        with self.lock:
            return (
                list(self.t), list(self.mu_demand), list(self.mu_demand_axle_evidence),
                list(self.mu_potential),
                list(self.mu_eff), list(self.a_lat), list(self.a_long),
                list(self.valid), self.latest, self.speed_mps,
            )

    def history_snapshot(self):
        with self.lock:
            return list(self.history)


def decode_bitmask(mask: int, names):
    return [name for bit, name in names if mask & bit] or ["--"]


def export_session(history, base_dir: str) -> str:
    """Write the full session (every sample received so far) to a fresh
    timestamped folder: data.csv, three full-time-axis PNG charts, and a
    plain-text summary. Returns the folder path."""
    if not history:
        raise RuntimeError("no data received yet -- nothing to export")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(base_dir).expanduser() / f"session_{timestamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(out_dir / "data.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=HISTORY_FIELDS)
        writer.writeheader()
        writer.writerows(history)

    t = [row["t"] for row in history]
    mu_d = [row["mu_demand"] for row in history]
    mu_ax = [row["mu_demand_axle_evidence"] if row["axle_evidence_valid"] else float("nan")
             for row in history]
    mu_p = [row["mu_potential"] for row in history]
    mu_e = [row["mu_eff"] for row in history]
    a_lat = [row["a_lat_mps2"] for row in history]
    a_long = [row["a_long_mps2"] for row in history]
    valid = [row["data_quality_valid"] for row in history]
    excitation = [row["excitation_score"] for row in history]
    confidence = [row["potential_confidence"] for row in history]

    # full-session mu time series (the whole run, not just the rolling display window)
    fig1, ax1 = plt.subplots(figsize=(14, 5))
    ax1.plot(t, mu_d, color=COLOR_DEMAND, lw=0.8, alpha=0.6, label="mu_demand (v1, EKF)")
    ax1.plot(t, mu_ax, color=COLOR_AXLE_EVIDENCE, lw=0.8, ls="--", alpha=0.7,
             label="mu_demand_axle_evidence (v2)")
    ax1.plot(t, mu_p, color=COLOR_POTENTIAL, lw=1.5, label="mu_potential")
    ax1.plot(t, mu_e, color=COLOR_EFF, lw=1.2, label="mu_eff")
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("mu")
    ax1.set_title(f"Full session ({t[-1]:.1f}s): mu_demand / mu_potential / mu_eff")
    ax1.grid(True, alpha=0.4)
    ax1.legend()
    fig1.tight_layout()
    fig1.savefig(out_dir / "mu_timeseries.png", dpi=150, facecolor=COLOR_BG)
    plt.close(fig1)

    # full-session excitation / confidence
    fig2, ax2 = plt.subplots(figsize=(14, 4))
    ax2.plot(t, excitation, color=COLOR_EXCITATION, lw=1.0, label="excitation_score")
    ax2.plot(t, confidence, color=COLOR_EFF, lw=1.0, label="potential_confidence")
    ax2.set_xlabel("t (s)")
    ax2.set_ylim(0, 1.05)
    ax2.set_title("Full session: excitation_score / potential_confidence")
    ax2.grid(True, alpha=0.4)
    ax2.legend()
    fig2.tight_layout()
    fig2.savefig(out_dir / "excitation_confidence.png", dpi=150, facecolor=COLOR_BG)
    plt.close(fig2)

    # full-session friction circle
    fig3, ax3 = plt.subplots(figsize=(6, 6))
    valid_x = [x for x, v in zip(a_long, valid) if v]
    valid_y = [y for y, v in zip(a_lat, valid) if v]
    invalid_x = [x for x, v in zip(a_long, valid) if not v]
    invalid_y = [y for y, v in zip(a_lat, valid) if not v]
    ax3.scatter(valid_x, valid_y, s=6, color=COLOR_EFF, alpha=0.4, label="valid")
    ax3.scatter(invalid_x, invalid_y, s=6, color=COLOR_INVALID, alpha=0.5, label="excluded")
    ax3.set_aspect("equal")
    ax3.set_xlabel("a_long (m/s2)")
    ax3.set_ylabel("a_lat (m/s2)")
    ax3.set_title("Full session: friction circle")
    ax3.grid(True, alpha=0.4)
    ax3.legend()
    fig3.tight_layout()
    fig3.savefig(out_dir / "friction_circle.png", dpi=150, facecolor=COLOR_BG)
    plt.close(fig3)

    n = len(history)
    n_valid = sum(1 for v in valid if v)
    n_excited = sum(1 for e in excitation if e >= 0.5)
    with open(out_dir / "summary.txt", "w") as f:
        f.write(f"session export: {timestamp}\n")
        f.write(f"samples: {n}\n")
        f.write(f"duration_s: {t[-1] - t[0]:.2f}\n")
        f.write(f"data_quality_valid: {n_valid}/{n} ({100 * n_valid / n:.1f}%)\n")
        f.write(f"excitation_score>=0.5: {n_excited}/{n} ({100 * n_excited / n:.1f}%)\n")
        f.write(f"mu_demand: min={min(mu_d):.3f} max={max(mu_d):.3f}\n")
        f.write(f"mu_potential: min={min(mu_p):.3f} max={max(mu_p):.3f}\n")

    return str(out_dir)


class Dashboard:
    def __init__(self, node: FrictionListener, export_dir: str):
        self.node = node
        self.export_dir = export_dir
        plt.rcParams["figure.facecolor"] = COLOR_BG
        plt.rcParams["axes.facecolor"] = COLOR_PANEL
        plt.rcParams["axes.edgecolor"] = COLOR_GRID
        plt.rcParams["grid.color"] = COLOR_GRID
        plt.rcParams["text.color"] = COLOR_TEXT
        plt.rcParams["xtick.color"] = COLOR_MUTED
        plt.rcParams["ytick.color"] = COLOR_MUTED
        plt.rcParams["axes.labelcolor"] = COLOR_TEXT
        plt.rcParams["savefig.facecolor"] = COLOR_BG
        plt.rcParams["font.size"] = 9

        self.fig = plt.figure(figsize=(12, 7))
        self.fig.canvas.manager.set_window_title("Friction Estimator -- Live Dashboard")
        self.fig.subplots_adjust(top=0.90)
        gs = self.fig.add_gridspec(2, 3, width_ratios=[2, 1, 1], height_ratios=[3, 1],
                                    top=0.90, hspace=0.35, wspace=0.3)
        self.ax_ts = self.fig.add_subplot(gs[0, 0:2])
        self.ax_circle = self.fig.add_subplot(gs[0, 2])
        self.ax_status = self.fig.add_subplot(gs[1, :])
        self.ax_status.axis("off")

        self.ax_ts.set_title("mu_demand / mu_potential / mu_eff")
        self.ax_ts.set_xlabel("t (s, bag/session relative)")
        self.ax_ts.set_ylabel("mu")
        self.ax_ts.grid(True, alpha=0.4)
        (self.line_demand,) = self.ax_ts.plot([], [], color=COLOR_DEMAND, lw=1.0,
                                               alpha=0.6, label="mu_demand (v1, EKF)")
        (self.line_axle_evidence,) = self.ax_ts.plot(
            [], [], color=COLOR_AXLE_EVIDENCE, lw=1.0, ls="--", alpha=0.7,
            label="mu_demand_axle_evidence (v2)")
        (self.line_potential,) = self.ax_ts.plot([], [], color=COLOR_POTENTIAL, lw=2.0,
                                                  label="mu_potential")
        (self.line_eff,) = self.ax_ts.plot([], [], color=COLOR_EFF, lw=1.5,
                                            label="mu_eff")
        self.ax_ts.legend(loc="upper left", framealpha=0.15, fontsize=8)

        self.ax_circle.set_title("friction circle")
        self.ax_circle.set_xlabel("a_long (m/s2)")
        self.ax_circle.set_ylabel("a_lat (m/s2)")
        self.ax_circle.grid(True, alpha=0.4)
        self.ax_circle.set_aspect("equal")
        self.scatter_valid = self.ax_circle.scatter([], [], s=8, color=COLOR_EFF,
                                                      alpha=0.5, label="data-quality-valid")
        self.scatter_invalid = self.ax_circle.scatter([], [], s=8, color=COLOR_INVALID,
                                                        alpha=0.5, label="excluded (lockup/ABS/fault)")
        self.grip_circle = Circle((0, 0), 0.0, fill=False, color=COLOR_POTENTIAL, lw=1.2)
        self.ax_circle.add_patch(self.grip_circle)
        self.ax_circle.legend(loc="upper right", framealpha=0.15, fontsize=7)

        self.status_text = self.ax_status.text(
            0.0, 0.5, "waiting for /core/friction/estimate ...",
            fontsize=11, family="monospace", va="center", ha="left",
            transform=self.ax_status.transAxes, color=COLOR_TEXT)

        # one-click full-session export: raw CSV + full-time-axis charts into a
        # fresh timestamped folder, independent of the rolling display window
        button_ax = self.fig.add_axes((0.83, 0.925, 0.15, 0.05))
        self.export_button = Button(button_ax, "Export Session",
                                     color=COLOR_PANEL, hovercolor=COLOR_POTENTIAL)
        self.export_button.label.set_color(COLOR_TEXT)
        self.export_button.on_clicked(self._on_export_clicked)
        self.export_status_text = self.fig.text(
            0.83, 0.905, "", fontsize=7, color=COLOR_MUTED, ha="left", va="top")

        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.update)
        self.timer.start()

    def _on_export_clicked(self, _event):
        try:
            history = self.node.history_snapshot()
            path = export_session(history, self.export_dir)
            self.export_status_text.set_color(COLOR_EFF)
            self.export_status_text.set_text(f"exported {len(history)} samples to:\n{path}")
        except Exception as exc:  # noqa: BLE001 -- surface any failure in the GUI, not just stderr
            self.export_status_text.set_color(COLOR_INVALID)
            self.export_status_text.set_text(f"export failed: {exc}")
        self.fig.canvas.draw_idle()

    def update(self):
        t, mu_d, mu_ax, mu_p, mu_e, a_lat, a_long, valid, latest, speed = self.node.snapshot()
        if not t:
            return

        window_start = t[-1] - self.node.window_s
        self.line_demand.set_data(t, mu_d)
        self.line_axle_evidence.set_data(t, mu_ax)
        self.line_potential.set_data(t, mu_p)
        self.line_eff.set_data(t, mu_e)
        self.ax_ts.set_xlim(max(0, window_start), t[-1] + 0.5)
        # mu_ax can be NaN (axle_evidence_valid was False that cycle) -- guard it out of
        # the scaling decision rather than letting a stray NaN silently break max().
        mu_ax_last = mu_ax[-1] if mu_ax and mu_ax[-1] == mu_ax[-1] else 0.0
        y_top = max(1.4, max(mu_p[-1], mu_d[-1], mu_e[-1], mu_ax_last) * 1.3)
        self.ax_ts.set_ylim(0, y_top)

        valid_x = [x for x, v in zip(a_long, valid) if v]
        valid_y = [y for y, v in zip(a_lat, valid) if v]
        invalid_x = [x for x, v in zip(a_long, valid) if not v]
        invalid_y = [y for y, v in zip(a_lat, valid) if not v]
        if valid_x:
            self.scatter_valid.set_offsets(list(zip(valid_x, valid_y)))
        if invalid_x:
            self.scatter_invalid.set_offsets(list(zip(invalid_x, invalid_y)))

        if latest is not None:
            self.grip_circle.set_radius(max(latest.grip_limit_mps2, 0.01))
            lim = max(latest.grip_limit_mps2 * 1.3, 2.0)
            self.ax_circle.set_xlim(-lim, lim)
            self.ax_circle.set_ylim(-lim, lim)

            health = decode_bitmask(latest.input_health, HEALTH_BIT_NAMES)
            exclusion_reasons = []
            if latest.wheel_lockup:
                exclusion_reasons.append("WHEEL_LOCKUP")
            if latest.abs_tc_active:
                exclusion_reasons.append("ABS_TC_ACTIVE")
            if not exclusion_reasons:
                exclusion_reasons = ["--"]
            axle_ev_str = (
                f"{latest.mu_demand_axle_evidence:.3f}" if latest.axle_evidence_valid
                else "-- (v2 not running/stale)")
            status = (
                f"speed: {speed:6.2f} m/s ({speed * 3.6:6.2f} km/h)   |   "
                f"excitation_score: {latest.excitation_score:.2f}   |   "
                f"potential_confidence: {latest.potential_confidence:.2f}\n"
                f"mu_eff: {latest.mu_eff:.3f}   mu_potential: {latest.mu_potential:.3f}   "
                f"mu_demand: {latest.mu_demand:.3f}   mu_demand_axle_evidence: {axle_ev_str}   "
                f"eta_wear: {latest.eta_wear:.3f}\n"
                f"exclusion reasons: {', '.join(exclusion_reasons)}   |   "
                f"input_health: {', '.join(health)}"
            )
            self.status_text.set_text(status)

        self.fig.canvas.draw_idle()


def main():
    default_export_dir = Path(__file__).resolve().parent.parent / "exports"

    parser = argparse.ArgumentParser()
    parser.add_argument("--window-s", type=float, default=30.0,
                         help="rolling time-series window, seconds")
    parser.add_argument("--export-dir", type=str, default=str(default_export_dir),
                         help="parent folder for one-click session exports "
                              "(each export creates its own timestamped subfolder)")
    args = parser.parse_args()

    rclpy.init()
    node = FrictionListener(args.window_s)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    dashboard = Dashboard(node, args.export_dir)  # noqa: F841
    try:
        plt.show()
    finally:
        # Order matters: shutdown() first makes context.ok() false, which is what
        # rclpy.spin()'s internal loop checks to return -- only after the spin thread
        # has actually exited (join) is it safe to destroy_node(). Destroying the node
        # or letting the interpreter tear down the daemon thread's rcl handles while
        # spin() is still mid-callback races the DDS layer and can abort the process
        # (observed as "terminate called without an active exception").
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
